/* Main.cpp - DUKATIMER ESP32-S3 (v0.5 Root Cause Edition)
   
   Version: 0.5.0
   Fokus: Vollständige Core-Isolation, Latenzfreiheit & v0.3.10 Feature-Parität.
   
   Architektur-Verteilung:
   - CORE 0 (TaskIO): Nextion UART, LCD I2C, Sound-Queue, Housekeeping (Temperatur).
   - CORE 1 (TaskRealtime): Encoder-Polling, Input-Queue, Belichtungs-Timer, HAL-Lichtsteuerung.
*/

#include <Arduino.h>
#include <Wire.h>
#include <nvs_flash.h> 
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Encoder.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BMP280.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "rgb_lcd.h"

#include "Config.h"
#include "Types.h"
#include "Globals.h"
#include "Logic_Papers.h"
#include "Logic_Storage.h"
#include "DisplayManager.h"
#include "Logic_Measurement.h"
#include "Logic_Timer.h"
#include "ExposureEngine.h"

// =============================================================================
// GLOBAL VARIABLE DEFINITIONS (Single Source of Truth)
// =============================================================================

SemaphoreHandle_t gTimerMutex = NULL;
SemaphoreHandle_t gPixelMutex = NULL;
SemaphoreHandle_t xI2CMutex   = NULL;
SemaphoreHandle_t xShadowMutex = NULL;
QueueHandle_t xInputQueue     = NULL;
QueueHandle_t xSoundQueue     = NULL;

TwoWire I2C_FAST = TwoWire(1);
Adafruit_TSL2591 tslBase = Adafruit_TSL2591(2591);
Adafruit_TSL2561_Unified tslHead = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_TSL2561_Unified& tslLive = tslHead; 
rgb_lcd lcd;
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
Adafruit_BMP280* bmpPtr = nullptr;
Adafruit_BME280* bmePtr = nullptr;

bool tslBaseOK = false, tslHeadOK = false, tslLiveOK = false;
bool bmpOK = false, bmeOK = false, lcdOK = false, neoPixelOK = false, tempSensorOK = false;
bool isBME280 = false;
volatile bool overheatLock = false;
double tempAlu = 0.0, tempAmbient = 0.0, tempRoom = 0.0, pressRoom = 0.0, humRoom = 0.0;
int currentGainIdx = 0;

bool hwSwitchDoseMode = false;
double target_dose = 100.0, current_dose = 0.0, spectral_ratio = 1.0;
double dose_bw = 12.0, dose_soft = 10.0, dose_hard = 0.0;
double targetDoseSoft = 0.0, targetDoseHard = 0.0;
float time_bw = 12.0, grade_bw = 2.5, time_soft = 10.0, time_hard = 0.0;
float burnEv = 0.0;
double burnGrade = 2.5;

volatile unsigned long starttime = 0;

volatile Mode currentMode = MODE_BW;
SplitState splitState = SPLIT_IDLE;
CalStep calState = CAL_IDLE;
BurnMode burnMode = BURN_OFF;
StepSize globalStepMode = STEP_THIRD;

uint8_t pwmValGreen = 0, pwmValBlue = 0;
float baseFlux = 1.0f;
int multival[11][3] = { {0,255,0}, {0,235,20}, {0,215,40}, {0,195,60}, {0,170,85}, {0,143,112}, {0,115,140}, {0,85,170}, {0,60,200}, {0,30,225}, {0,0,255} };
double paperSpeed[11] = { 1.1, 1.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.1, 1.3, 1.6, 2.0 };

volatile bool statusEnlargerOn = false, statusSafeOn = false, isRoomDarknessActive = false;
volatile bool safeLatch = false, whiteLatch = false, roomLatch = false;
volatile bool measurementOverrideActive = false, screenOffOverride = false;
volatile bool isPaused = false, isMeasuring = false;
volatile bool bootScreenActive = true, setupMenuActive = false;

// FIX A1: Zentrale Licht-Sperrlogik
volatile bool lightOperationActive = false;
volatile bool softAbortActive = false;
volatile unsigned long softAbortUntilMs = 0;

TSState ts = TS_OFF;
uint8_t tsN = DEFAULT_TS_N;
double tsEv = DEFAULT_TS_EV;
double tsA[10] = {0};
uint8_t tsK = 0;
TSChannel tsCh = TS_BW;
double tsSum = 0.0;
bool tsActiveExposure = false;

DensitometerState densState = DENS_IDLE;
DensSubMode densSub = DENS_SUB_MANUAL;
double densRefLux = 0.0, densBaseFog = 0.0, zone8TargetNet = 1.25;

double baseDarkLux = 0.0;
double probeDarkLux = 0.0;

double paper_iso_p = 0.0, paper_iso_r = 0.0;
char activePaperName[32] = "Default";
PaperBank* paperBankPtr = nullptr;

volatile uint32_t totalDroppedPackets = 0, probeEventOverruns = 0;
volatile bool probeConnected = false, probeEventPending = false;
volatile uint8_t probeLastEvent = 0;
volatile float probeLuxG0 = 0.0f, probeLuxG5 = 0.0f;
volatile double remoteLux = 0.0;
volatile unsigned long lastRemotePacketMs = 0;
bool probeFlashActive = false;

volatile SystemError lastSystemError = ERR_NONE;

SettingsObject globalSet;
uint8_t set_safe = 100, set_focus = 255, set_lcd = 100, set_max = 255;
bool useWirelessProbe = true; // Wireless als Standard
uint8_t currentZoneHistogram[11] = {0};
MeasureFocus currentMeasureFocus = FOCUS_HIGHLIGHTS;
float timer_base_seconds = 0.0f;
float calibBaseLuxG0 = 0.0f;
float calibBaseLuxG5 = 0.0f;
float calibTimeSeconds = 0.0f;
double trackDensityEV = 0.0;
double trackGradeSteps = 0.0;
String overlayText = "";
unsigned long infoEndTime = 0;
int pendingPaperSlot = -1;
double measSoftSum = 0.0, measHardSum = 0.0, measBWSum = 0.0;
int measSoftCount = 0, measHardCount = 0, measBWCount = 0;
// BETA-FIX: Schritt 3 - Signalisierung fuer anstehende Messwert-Uebernahme.
volatile bool bwAutoPending = false;

// Protokoll-Grounding: Beide Firmwares muessen dieselben Strukturgroessen sehen.
// Damit verhindern wir schleichende ABI-Fehler durch Padding/Compiler-Unterschiede,
// die sonst genau zu "Anzeige geht, Eingaben gehen nicht" fuehren koennen.
static_assert(sizeof(ProbeEventPacket) == 16, "ProbeEventPacket ABI mismatch: erwartet 16 Byte");
static_assert(sizeof(ProbeRenderPacket) == 63, "ProbeRenderPacket ABI mismatch: erwartet 63 Byte");
static_assert(sizeof(WirelessPacket) == 12, "WirelessPacket ABI mismatch: erwartet 12 Byte");

// =============================================================================
// TASK IO (Core 0): Latenz-Management
// =============================================================================
void vTaskIO(void *pvParameters) {
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();

        // Im Bridge-Modus darf Core 0 keinerlei Bytes aus Serial2 lesen, weil sonst der
        // Upload-Datenstrom zwischen PC und Nextion zerstueckelt wuerde. Deshalb wird die
        // regulare Nextion-Ereignisverarbeitung fuer diesen Spezialmodus hart uebersprungen.
        if (currentMode != MODE_BRIDGE) {
            DM_loop();
        }

        if (bootScreenActive) {
            extern void runBootDiagnostics();
            runBootDiagnostics();

            // ROOT CAUSE FIX:
            // Der Boot-Diagnosepfad schreibt seine Texte nur in den LCD-Shadow-Buffer.
            // Vorher wurde bei aktivem bootScreenActive mit `continue` direkt aus dem
            // Task gesprungen, bevor processLCDShadow() den Puffer auf das echte I2C-LCD
            // ausgeben konnte. Ergebnis: Das 16x2-LCD blieb scheinbar "eingefroren"
            // auf der letzten Seite, die in setup() direkt per lcd.print() geschrieben
            // wurde. Wir flushen deshalb den Shadow-Buffer explizit auch im Bootmodus.
            extern void processLCDShadow();
            processLCDShadow();

            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        extern void processLCDShadow();
        processLCDShadow();
        
        extern void processSoundQueue();
        processSoundQueue();
        
        static unsigned long lastHousekeeping = 0;
        if (millis() - lastHousekeeping > 2000) {
            if (tempSensorOK) {
                sensors.requestTemperatures();
                float t = sensors.getTempCByIndex(0);
                if (t > -100 && t < 150) tempAlu = t;
            }
            // FIX A2: BMP read must be wrapped in xI2CMutex
            if (bmpOK && bmpPtr) {
                if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    tempRoom = bmpPtr->readTemperature();
                    xSemaphoreGive(xI2CMutex);
                }
                tempAmbient = tempRoom;
            }
            
            extern void processDeferredEEPROM();
            processDeferredEEPROM();

            // FIX Punkt 2: Periodische Queue-Overflow-Diagnose
            {
                extern volatile uint32_t inputQueueDrops;
                if (probeEventOverruns > 0 || inputQueueDrops > 0) {
                    Serial.printf("[DIAG] probeEvtOverruns=%lu  inputQDrops=%lu\n",
                                  (unsigned long)probeEventOverruns, (unsigned long)inputQueueDrops);
                }
            }

            lastHousekeeping = millis();
        }
        
        static unsigned long lastUIRefresh = 0;
        if (millis() - lastUIRefresh > 100) {
            // Dieselbe Schutzregel gilt fuer UI-Schreibzugriffe: Waehrend der Bridge darf Core 0
            // keine Display-Kommandos erzeugen. Sonst wuerde der Upload-Stream mit UI-Daten
            // vermischt, auch wenn zusaetzlich der Serial-Mutex gehalten wird.
            if (currentMode != MODE_BRIDGE) {
                extern void updateNextionUI(bool force); // NEXTION UI INTEGRATION
                updateNextionUI(false);
            }
            lastUIRefresh = millis();
        }
        
        static unsigned long lastProbeCheck = 0;
        if (millis() - lastProbeCheck > 250) {
            // Risikoarmer Stabilitaets-Fix:
            // Beim Start der Spektralmessung kann das Handgeraet mehrere Sekunden mit
            // Sensorintegration/Retry beschaeftigt sein. In dieser Phase entstehen sonst
            // false negatives auf der Verbindungsanzeige.
            // Es wird nur die Timeout-Grenze angehoben, keine Ablauf-Logik geaendert.
            if (probeConnected && (millis() - lastRemotePacketMs > 12000)) probeConnected = false;

            // Kommunikations-Race-Fix:
            // Waehrend des spektralen Flash-Handshakes sendet der S3 gezielte
            // CMD_MEASURE_G0/G5-Pakete. Zyklische Hintergrund-Renderpakete aus dieser
            // 250ms-Schleife koennen den C6-Command-Puffer zeitlich ueberlagern und damit
            // den Messauftrag verdraengen. Ergebnis war ein sichtbares "halb verbunden":
            // Display-Updates kamen an, aber Messstart ueber T2 schlug sporadisch/haeufig fehl.
            //
            // Deshalb unterdruecken wir die periodischen Render-Sends, solange ein aktiver
            // Mess-Handshake laeuft. Nach Abschluss laeuft das normale Rendering automatisch
            // weiter.
            extern bool isMeasurementActive();
            const bool handshakeAktiv = isMeasurementActive();
            if (!handshakeAktiv) {
                extern void updateProbeDisplay();
                updateProbeDisplay();
            }
            lastProbeCheck = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================================
// TASK REALTIME (Core 1): Zeitkritische Logik
// =============================================================================
void vTaskRealtime(void *pvParameters) {
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();

        extern void HW_Input_Process();
        HW_Input_Process();
        
        handleInput();
        handleTimer();
        ExposureEngine_Tick();   // Zentrale Engine-Tick für ALLE Belichtungs-Modi
        handleLights();
        handleLCDBacklight();
        
        // =====================================================================
        // PFLICHTENHEFT FIX (Modus 9 / 10): Vollständiges Probe-Event-Routing
        // VORHER: Alle C6-Events gingen ausschließlich an handleMeasurementStateMachine().
        //         Events wie T2_CLICK (Messen/Feuer), T1_CLICK (Undo/Referenz),
        //         ENC_UP/DOWN (Navigation) und ENC_CLICK (Abschließen) wurden
        //         in allen Modi außer der Spektralmessung verworfen.
        // NEU: Modusspezifisches Routing gemäß Pflichtenheft Abschnitt 9.2:
        //   A) Mess-Modus: T2=Spektralmessung, T1=Undo, ENC=Kanal/Abschluss
        //   B) Burn-Modus: T2=Fernauslöser, T1=Abbruch, ENC=Burn-Step-Navigation
        //   C) Kalibrierung: T2=Messung triggern
        //   D) Densitometer: T1=Referenz, T2=Messung
        // Die Spectral-Statemachine (Flash-Handshake) behält Priorität für
        // EVT_LUX_DATA Pakete, die direkt aus dem Handshake kommen.
        // =====================================================================
        uint8_t pEvt = EVT_NONE;
        float pG0 = 0.0f, pG5 = 0.0f;
        extern bool popProbeEvent(uint8_t&, float&, float&);
        extern bool handleMeasurementStateMachine(uint8_t, float, float);
        if (popProbeEvent(pEvt, pG0, pG5)) {
            // Priorität 1: Laufende Spektralmessung (Flash-Handshake) hat Vorrang
            bool consumed = handleMeasurementStateMachine(pEvt, pG0, pG5);
            
            // Priorität 2: Modusspezifisches Event-Routing (Pflichtenheft 9.2)
            if (!consumed) {
                extern void handleProbeEventForCurrentMode(uint8_t evt, float luxG0, float luxG5);
                handleProbeEventForCurrentMode(pEvt, pG0, pG5);
            }
        } else {
            // FIX K1: State Machine zyklisch ticken auch ohne eingehende Events.
            // Warmup-Timer (150ms) und Timeouts (2500ms) feuern sonst nie.
            handleMeasurementStateMachine(EVT_NONE, 0.0f, 0.0f);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
// SYSTEM HELPER (v0.3.10 Legacy Parity)
// =============================================================================

static void panicPSRAMFail() {
    Serial.println("[PSRAM] KRITISCH: PaperBank-Allokation fehlgeschlagen. Systemstopp.");

    lcd.setCursor(0, 0);
    lcd.print(" !!! KRITISCH !!!");
    lcd.setCursor(0, 1);
    lcd.print("  PSRAM FAIL   ");
    lcd.setRGB(255, 0, 0);

    uiTriggerBeep(SND_ALARM);

    extern void processSoundQueue();
    for (;;) {
        processSoundQueue();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void initPSRAMStructures() {
    bool hasPsram = psramFound();
    Serial.printf("[PSRAM] psramFound()=%s  getPsramSize=%d KB\n",
        hasPsram ? "true" : "false", (int)(ESP.getPsramSize() / 1024));

    if (hasPsram) {
        paperBankPtr = (PaperBank*)heap_caps_malloc(sizeof(PaperBank), MALLOC_CAP_SPIRAM);
        if (paperBankPtr) {
            memset(paperBankPtr, 0, sizeof(PaperBank));
            Serial.printf("[PSRAM] PaperBank im PSRAM allokiert: %d Bytes\n", (int)sizeof(PaperBank));
            return;
        }
        Serial.println("[PSRAM] WARN: heap_caps_malloc fehlgeschlagen, Fallback auf int. RAM.");
    } else {
        Serial.println("[PSRAM] WARN: Kein PSRAM erkannt. Pruefe board/memory_type in platformio.ini.");
        Serial.println("[PSRAM]   board=esp32s3  +  board_build.arduino.memory_type=qio_opi  (fuer N16R8)");
        Serial.println("[PSRAM]   Alternativer memory_type: qio_qspi (QSPI-PSRAM), opi_opi (OPI-Flash)");
    }

    // Fallback: Internes RAM. Kein harter Halt – erst Diagnose, dann entscheiden.
    paperBankPtr = (PaperBank*)malloc(sizeof(PaperBank));
    if (paperBankPtr) {
        memset(paperBankPtr, 0, sizeof(PaperBank));
        Serial.printf("[RAM] PaperBank im internen RAM: %d Bytes\n", (int)sizeof(PaperBank));
    } else {
        // Wirklich kein Speicher mehr – jetzt erst Hardstop
        panicPSRAMFail();
    }
}

void logMemoryInfo() {
    Serial.println("\n=== SYSTEM DIAGNOSTIC =================================");
    Serial.printf(" Chip:             ESP32-S3 rev%d\n", ESP.getChipRevision());
    Serial.printf(" CPU Freq:         %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf(" Flash Size:       %d KB\n", ESP.getFlashChipSize() / 1024);

    // PSRAM-Diagnose: Alle drei unabhängigen Erkennungspfade
    bool psramDetected = psramFound();
    size_t psramTotal   = ESP.getPsramSize();
    size_t psramFree    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    Serial.printf(" psramFound():     %s\n", psramDetected ? "JA" : "NEIN");
    Serial.printf(" ESP.getPsramSize: %d KB\n", psramTotal / 1024);
    Serial.printf(" SPIRAM Free:      %d KB\n", psramFree / 1024);

    // Allokationstest: Direkt 1KB im PSRAM anfordern (ganz ohne PSRAM --> NULL)
    void* testPtr = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
    bool allocOk = (testPtr != nullptr);
    Serial.printf(" PSRAM Alloc-Test: %s\n", allocOk ? "OK (1KB allokiert)" : "FEHLGESCHLAGEN");
    if (testPtr) heap_caps_free(testPtr);

    Serial.printf(" Internal Free:    %d KB\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    Serial.printf(" sizeof(PaperBank):%d Bytes\n", (int)sizeof(PaperBank));
    Serial.printf(" paperBankPtr:     %s (0x%08X)\n",
        paperBankPtr ? "OK" : "NULL", (unsigned int)(uintptr_t)paperBankPtr);
    Serial.println("=======================================================\n");

    // Boot-Diagnose direkt auf dem LCD ausgeben (ohne Serial-Monitor nutzbar)
    char l1[17] = {0};
    char l2[17] = {0};

    snprintf(l1, sizeof(l1), "PSRAM:%s %4dK", psramDetected ? "JA" : "NEIN", (int)(psramTotal / 1024));
    snprintf(l2, sizeof(l2), "FREE:%6dK", (int)(psramFree / 1024));
    lcd.setCursor(0, 0); lcd.print("                ");
    lcd.setCursor(0, 1); lcd.print("                ");
    lcd.setCursor(0, 0); lcd.print(l1);
    lcd.setCursor(0, 1); lcd.print(l2);
    delay(1300);

    snprintf(l1, sizeof(l1), "ALLOC:%s", allocOk ? "OK" : "FAIL");
    snprintf(l2, sizeof(l2), "BANK:%6dB", (int)sizeof(PaperBank));
    lcd.setCursor(0, 0); lcd.print("                ");
    lcd.setCursor(0, 1); lcd.print("                ");
    lcd.setCursor(0, 0); lcd.print(l1);
    lcd.setCursor(0, 1); lcd.print(l2);
    delay(1300);

    snprintf(l1, sizeof(l1), "INT RAM:%5dK", (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    snprintf(l2, sizeof(l2), "BOOT -> UI");
    lcd.setCursor(0, 0); lcd.print("                ");
    lcd.setCursor(0, 1); lcd.print("                ");
    lcd.setCursor(0, 0); lcd.print(l1);
    lcd.setCursor(0, 1); lcd.print(l2);
    delay(1000);
}

void wdt_reset() { esp_task_wdt_reset(); yield(); }

// =============================================================================
// SETUP (Bootstrapping v0.5)
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0); 

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("NVS Partition korrupt/leer. Formatiere...");
        nvs_flash_erase();
        err = nvs_flash_init();
    }

    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);

    gTimerMutex = xSemaphoreCreateMutex();
    gPixelMutex = xSemaphoreCreateMutex();
    xI2CMutex   = xSemaphoreCreateMutex();
    xShadowMutex = xSemaphoreCreateMutex();
    xInputQueue = xQueueCreate(32, sizeof(InputEvent));
    xSoundQueue = xQueueCreate(10, sizeof(SoundID));

    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C0_FREQ);
    I2C_FAST.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_FREQ);

    pinMode(PIN_RELAY_ROOMLIGHT, OUTPUT);
    digitalWrite(PIN_RELAY_ROOMLIGHT, LOW); 

    initInput();
    initWireless();
    
    HW_InitLights();
    neoPixelOK = true;

    // ROOT CAUSE FIX: Hardware Bootscreen für I2C Display!
    // Gibt sofort physisches Feedback nach dem Strom-Einschalten.
    lcd.begin(16, 2);
    lcd.setRGB(0, 0, 0); // Background aus
    lcd.setCursor(0, 0);
    lcd.print(" DUKATIMER v0.5 ");
    lcd.setCursor(0, 1);
    lcd.print(" System Boot... ");
    lcd.setRGB(0, 100, 255); // Angenehmes Blau
    
    // ROOT CAUSE FIX: Boot Race Condition (Nextion)
    // Wir warten exakt 1 Sekunde, damit man das LCD lesen kann UND 
    // das Nextion-Display im Hintergrund vollständig hochfahren kann!
    delay(1000); 

    initPSRAMStructures();

    // Jetzt erst wird die serielle Verbindung zum Nextion gestartet.
    DM_init();

    if (tslBase.begin()) tslBaseOK = true;
    if (tslLive.begin(&I2C_FAST)) {
        tslLiveOK = tslHeadOK = true;
        tslLive.enableAutoRange(false);
        tslLive.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
        tslLive.setGain(TSL2561_GAIN_1X);
    }

    bmpPtr = new Adafruit_BMP280(&Wire);
    if (bmpPtr->begin(0x76, 0x58) || bmpPtr->begin(0x76, 0x60)) {
        bmpOK = true;
        bmpPtr->setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::FILTER_OFF, Adafruit_BMP280::STANDBY_MS_1);
    }

    sensors.begin();
    sensors.setWaitForConversion(false);
    if (sensors.getDeviceCount() > 0) tempSensorOK = true;

    initStorage();
    loadSettings();
    initPapers();
    loadActivePaperProfile();

    extern void initializeDoseStateFromCurrentTimes();
    initializeDoseStateFromCurrentTimes();

    triggerInfo(); // Füllt den LCD Puffer mit dem finalen Startbildschirm
    extern void updateNextionUI(bool force);
    updateNextionUI(true); // Füllt das hochgefahrene Nextion mit den ersten Daten

    xTaskCreatePinnedToCore(vTaskIO, "TaskIO", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskRealtime, "TaskRealtime", 8192, NULL, 5, NULL, 1);

    initClosedLoopTask();

    logMemoryInfo();
    uiTriggerBeep(SND_OK);
}

void loop() {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}