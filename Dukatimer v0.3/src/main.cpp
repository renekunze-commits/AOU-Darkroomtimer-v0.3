/* Main.cpp - DUKATIMER ESP32-S3
   Version: 0.3.10 (PSRAM-Aktivierung, Performance-Optimierung, CDC-Fix)

   ÄNDERUNGEN gegenüber v0.3.9.1:
   - PSRAM (OPI) aktiviert: PaperBank liegt jetzt im externen PSRAM
   - NeoPixel-Buffer bleibt bewusst im internen SRAM (RMT-DMA-Kompatibilität)
   - USB-CDC Logging wird während Belichtung unterdrückt (Jitter-Vermeidung)
   - Speicher-Diagnose beim Booten (PSRAM/Heap-Info auf Serial)
   - I2C Bus 1 auf 400 kHz für schnellere TSL2561-Abfragen
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Encoder.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BMP280.h>
#include <esp_heap_caps.h>           // PSRAM: heap_caps_malloc()
#include "rgb_lcd.h"

#include "Config.h"
#include "Types.h"
#include "Globals.h"
#include "Logic_Papers.h"
#include "DisplayManager.h"
#include "Logic_Measurement.h"

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
SemaphoreHandle_t gTimerMutex = NULL;
SemaphoreHandle_t gPixelMutex = NULL;
volatile SystemError lastSystemError = ERR_NONE;
volatile bool softAbortActive = false;
volatile unsigned long softAbortUntilMs = 0;
bool isPaused = false;
bool isMeasuring = false;
bool bootScreenActive = true; // Hält den Sensor-Status am Anfang
bool setupMenuActive = false;

// =============================================================================
// WIRELESS SENSOR (ESP-NOW Bidirektional)
// =============================================================================
bool useWirelessProbe = false;         // Standard: Kabelgebunden
volatile double remoteLux = 0.0;
volatile unsigned long lastRemotePacketMs = 0;
volatile uint8_t probeLastEvent = 0;   // EVT_NONE
volatile float   probeLuxG0 = 0.0f;
volatile float   probeLuxG5 = 0.0f;
volatile bool    probeEventPending = false;
volatile bool    probeConnected = false;
bool probeFlashActive = false;
bool measurementOverrideActive = false;

TwoWire& I2C_SLOW = Wire;
TwoWire I2C_FAST = TwoWire(1);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_TSL2591 tslBase = Adafruit_TSL2591(2591);
Adafruit_TSL2561_Unified tslLive = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
Adafruit_BMP280* bmpPtr = nullptr;
rgb_lcd lcd;

// Status Flags
bool tslBaseOK = false, tslLiveOK = false, lcdOK = false, nextonOK = false;
bool bmpOK = false, tempSensorOK = false, neoPixelOK = false, overheatLock = false;
double tempAlu = 0.0;
double tempRoom = 0.0;
int currentGainIdx = 0;
bool whiteLatch = false, safeLatch = false, roomLatch = false, screenOffOverride = false;

unsigned long starttime = 0;
double time_soft = 10.0, time_hard = 0.0, time_bw = 12.0, grade_bw = 2.5;

BurnMode burnMode = BURN_OFF;
double burnGrade = 2.5;
double burnEv = 0.0;

uint8_t set_safe = 30, set_focus = 255, set_lcd = 100, set_max = 255;
StepSize globalStepMode = STEP_THIRD; 

double trackDensityEV = 0.0;
double trackGradeSteps = 0.0;
String overlayText = "";
unsigned long infoEndTime = 0;
int pendingPaperSlot = -1;
double measSoftSum = 0.0; int measSoftCount = 0;
double measHardSum = 0.0; int measHardCount = 0;
double measBWSum = 0.0; int measBWCount = 0;
MeasureMode measureMode = MM_OFF;
MeasureFocus currentMeasureFocus = FOCUS_HIGHLIGHTS;
MeasureState currentMeasureState = MEASURE_IDLE;
float timer_base_seconds = 0.0;
uint8_t currentZoneHistogram[11] = {0};
double targetDoseSoft = 0.0;
double targetDoseHard = 0.0;
unsigned long measureUiSinceMs = 0;

TSState ts = TS_OFF;
TSChannel tsCh = TS_BW;
uint8_t tsN = DEFAULT_TS_N;
double tsEv = DEFAULT_TS_EV;
uint8_t tsK = 0;
double tsA[10] = {0};
double tsSum = 0.0;
bool tsActiveExposure = false;

Mode currentMode = MODE_BW;
SplitState splitState = SPLIT_IDLE;
SettingsObject globalSet;
bool settingsDirty = false;
unsigned long lastSettingChange = 0;

int multival[11][3] = { {0,255,0}, {0,235,20}, {0,215,40}, {0,195,60}, {0,170,85}, {0,143,112}, {0,115,140}, {0,85,170}, {0,60,200}, {0,30,225}, {0,0,255} };

double paperSpeed[11] = { 1.1, 1.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.1, 1.3, 1.6, 2.0 };
bool sgShiftModeDensity = false;

DensitometerState densState = DENS_IDLE;
DensSubMode densSub = DENS_SUB_MANUAL;
double densRefLux = 0.0;
double densBaseFog = NAN;
double zone8TargetNet = 1.20;

// =============================================================================
// SYSTEM FUNCTIONS
// =============================================================================

// ---------------------------------------------------------------------------
// PSRAM-Initialisierung: Allokiert große Datenstrukturen im externen Speicher
// ---------------------------------------------------------------------------
// Warum PSRAM?
//   Der ESP32-S3-N16R8 hat nur ~512 KB internen SRAM, davon ~320 KB nutzbar.
//   WiFi/ESP-NOW belegen ~40 KB, FreeRTOS-Stacks ~16 KB, I2C-Buffer etc.
//   Durch Auslagerung der PaperBank (~2.4 KB) in den PSRAM bleibt mehr
//   interner Speicher für zeitkritische Interrupts und DMA-Buffer.
//
// Warum NeoPixel NICHT im PSRAM?
//   Der NeoPixel-Buffer ist nur 768 Bytes (256 LEDs × 3 Bytes). Der RMT-
//   Peripherie-Treiber liest den Buffer per CPU-Zugriff und wandelt ihn
//   in RMT-Items um. Obwohl PSRAM funktionieren würde, überwiegt der Vorteil
//   des schnelleren, deterministischen Zugriffs aus dem internen SRAM.
// ---------------------------------------------------------------------------
void initPSRAMStructures() {
    // 1. PaperBank im PSRAM allokieren (Fallback: interner Heap)
    if (psramFound()) {
        paperBankPtr = (PaperBank*)heap_caps_malloc(sizeof(PaperBank), MALLOC_CAP_SPIRAM);
        if (paperBankPtr) {
            Serial.printf("[PSRAM] PaperBank allokiert: %d Bytes in PSRAM\n", sizeof(PaperBank));
        }
    }
    // Fallback: Wenn PSRAM fehlt oder Allokation fehlschlägt → interner Heap
    if (!paperBankPtr) {
        paperBankPtr = (PaperBank*)malloc(sizeof(PaperBank));
        Serial.println("[PSRAM] WARNUNG: PaperBank im internen SRAM (PSRAM nicht verfügbar)");
    }
    // Sicherheits-Nullung (verhindert Müll-Daten vor EEPROM-Load)
    if (paperBankPtr) {
        memset(paperBankPtr, 0, sizeof(PaperBank));
    }
}

// ---------------------------------------------------------------------------
// Speicher-Diagnose: Zeigt verfügbaren Heap und PSRAM beim Booten
// ---------------------------------------------------------------------------
// Nützlich zur Überwachung der Speicherauslastung. Wird einmalig nach
// setup() aufgerufen, damit der Verbrauch aller Init-Routinen sichtbar ist.
// ---------------------------------------------------------------------------
void logMemoryInfo() {
    Serial.println("=== SPEICHER-DIAGNOSE ===");
    Serial.printf("  Interner Heap frei:  %d Bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.printf("  Interner Heap max:   %d Bytes (größter Block)\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (psramFound()) {
        Serial.printf("  PSRAM frei:          %d Bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        Serial.printf("  PSRAM gesamt:        %d Bytes\n", ESP.getPsramSize());
    } else {
        Serial.println("  PSRAM: NICHT ERKANNT!");
    }
    Serial.println("=========================");
}

void wdt_reset() {
  esp_task_wdt_reset();
  yield();
}

void logError(SystemError err, String context) {
  if (lastSystemError == err) return;
  lastSystemError = err;
  saveErrorState(err);
  Serial.printf("[! ERROR %d] in %s\n", (int)err, context.c_str());
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);

  // -----------------------------------------------------------------------
  // USB-CDC Optimierung: Timeout auf 0 ms setzen
  // -----------------------------------------------------------------------
  // Problem: Serial.print() über USB-CDC kann bis zu 100ms blockieren, wenn
  // der Host (PC) nicht schnell genug liest oder kein USB verbunden ist.
  // Das würde den Main-Loop und damit die Belichtungssteuerung stören.
  // Lösung: Mit Timeout 0 wird Serial.print() non-blocking — Daten die
  // nicht sofort gesendet werden können, werden verworfen statt zu warten.
  // -----------------------------------------------------------------------
  Serial.setTxTimeoutMs(0);

  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  // -----------------------------------------------------------------------
  // PSRAM-Strukturen ZUERST initialisieren (vor loadSettings/initPapers!)
  // -----------------------------------------------------------------------
  // initPSRAMStructures() allokiert paperBankPtr im PSRAM. Wenn das nicht
  // vor loadPapers() passiert, würde der NULL-Pointer crashen.
  // -----------------------------------------------------------------------
  initPSRAMStructures();

  gTimerMutex = xSemaphoreCreateMutex();
  gPixelMutex = xSemaphoreCreateMutex();
  initClosedLoopTask();

  // --- INPUT PINS ---
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_SW_ROOMLIGHT, INPUT_PULLUP);
  pinMode(PIN_SW_FOCUS, INPUT_PULLUP);
  pinMode(PIN_SW_SAFE, INPUT_PULLUP);
  pinMode(PIN_SW_ENTER, INPUT_PULLUP); 
  pinMode(PIN_SW_BACK, INPUT_PULLUP);

  // --- KORREKTUR 1: OUTPUT PINS FÜR RELAIS HINZUFÜGEN ---
  pinMode(PIN_RELAY_ROOMLIGHT, OUTPUT);
  pinMode(PIN_RELAY_SAFE, OUTPUT);
  pinMode(PIN_RELAY_ENLARGER, OUTPUT);
  
  // Startzustand der Relais definieren
  digitalWrite(PIN_RELAY_ROOMLIGHT, LOW); 
  digitalWrite(PIN_RELAY_SAFE, LOW);
  digitalWrite(PIN_RELAY_ENLARGER, LOW);
  // -----------------------------------------------------

  I2C_SLOW.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C0_FREQ);
  I2C_FAST.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_FREQ);
  
  // LCD INIT
  lcdOK = false;
  I2C_SLOW.beginTransmission(LCD_I2C_ADDR); 
  if (I2C_SLOW.endTransmission() == 0) {
    lcd.begin(16, 2);
    lcdOK = true;
    lcd.setRGB(255, 255, 255);
    lcd.print("Booting...");
  }

  // Sensoren
  delay(500);
  if (tslBase.begin()) tslBaseOK = true;

  if (tslLive.begin(&I2C_FAST)) {
    tslLiveOK = true;
    tslLive.enableAutoRange(false);
    tslLive.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
    tslLive.setGain(TSL2561_GAIN_1X);
  } else {
    Serial.println("ERR: TSL2561 not found on Bus 1");
  }
  
  // BMP280 Initialisierung
  delay(500);
  bmpPtr = new Adafruit_BMP280(&I2C_SLOW);
  
  // VERSUCH 1: Standard 0x76
  if (!bmpPtr->begin(0x76)) {
      Serial.println("BMP280 @ 0x76 fail, trying ID override...");
      // VERSUCH 2: 0x76 mit erzwungener Chip-ID (0x58 ist Standard, 0x60 für BME)
      if (bmpPtr->begin(0x76, 0x58)) { 
          bmpOK = true; 
      } else if (bmpPtr->begin(0x76, 0x60)) { // Falls es ein BME280 ist
          bmpOK = true;
      } else {
          Serial.println("BMP280 FINAL FAIL");
      }
  } else {
      bmpOK = true;
  }

  // Set BMP280 to forced mode
  if (bmpOK) {
      bmpPtr->setSampling(Adafruit_BMP280::MODE_FORCED,
                          Adafruit_BMP280::SAMPLING_X1, // Temperature oversampling
                          Adafruit_BMP280::SAMPLING_X1, // Pressure oversampling
                          Adafruit_BMP280::FILTER_OFF, // No filtering
                          Adafruit_BMP280::STANDBY_MS_1); // Standby time
  }

  // DS18B20
  delay(500);
  sensors.begin();
  sensors.setWaitForConversion(false);
  if (sensors.getDeviceCount() > 0) tempSensorOK = true;

  initInput();
  initWireless();  // ESP-NOW Empfänger starten
  pixels.begin();
  neoPixelOK = true; // <--- DIESE ZEILE FEHLTE! Ohne sie blockt HW_Lights alle Befehle.
  pixels.clear();    // Einmalig löschen
  pixels.show();
  
delay(500);
  initDisplays();
  
  EEPROM.begin(4096);
  loadSettings();
  initPapers();  // Initialize paper bank after settings
  
  if (nextonOK) updateNextionUI(true);

  // Neuer Boot-Status Bildschirm
  if (lcdOK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SENSOR STATUS:");
    lcd.setCursor(0, 1);
    String status = "";
    status += (tslBaseOK ? "L1 " : "l1- ");
    status += (tslLiveOK ? "L2 " : "l2- ");
    status += (bmpOK ? "BR " : "br- ");
    status += (tempSensorOK ? "T " : "t- ");
    status += (nextonOK ? "N " : "n- ");
    // PSRAM-Status auf dem LCD anzeigen (P = OK, p- = fehlt)
    status += (psramFound() ? "P" : "p-");
    lcd.print(status);
  }

  // Speicher-Diagnose nach vollständiger Initialisierung ausgeben
  logMemoryInfo();
}

// =============================================================================
// FLASH-HANDSHAKE STATE MACHINE & PROBE DISPLAY
// =============================================================================
// Architektur: Asynchrone, deterministische Kommunikation S3 (Licht) <-> C6 (Auge).
// Phase 1: S3 schaltet NeoPixel G0 (Gruen), sendet CMD_MEASURE_G0 an C6
// Phase 2: C6 misst, sendet EVT_LUX_DATA -> S3 schaltet G5 (Blau), CMD_MEASURE_G5
// Phase 3: C6 misst, sendet EVT_LUX_DATA -> S3 verarbeitet Dual-Messwert
// Timeout: 3s -> automatischer Abort. Blockiert NICHT den Main Loop.
//
// Input Merging: Lokale Hardware (Taster, Encoder) und Remote-Events (ESP-NOW)
// werden ZUERST in logische Action-Flags zusammengefuehrt, BEVOR die Modi-Logik
// sie verarbeitet. Die Modi wissen nicht, woher der Klick kam.
// =============================================================================

extern portMUX_TYPE luxMux;  // Definiert in HW_Wireless.cpp

// Tick: Wird jeden Loop-Durchlauf aufgerufen.
// Konsumiert EVT_LUX_DATA Events. Gibt true zurueck wenn Event verarbeitet wurde.
bool tickFlashHandshake(uint8_t evt, float luxG0, float luxG5) {
    return handleMeasurementStateMachine(evt, luxG0, luxG5);
}

void startFlashHandshake() {
    triggerSpectralMeasurement();
}

void updateProbeDisplay() {
    if (!probeConnected || currentMeasureState != MEASURE_IDLE) return;

    char header[16] = {0};
    char line1[16]  = {0};
    char line2[16]  = {0};
    uint8_t mode    = PMODE_IDLE;

    if (starttime != 0) {
        unsigned long elapsed = millis() - starttime;
        snprintf(header, sizeof(header), "[ EXPOSURE ]");
        snprintf(line1,  sizeof(line1),  "T: %.1fs", elapsed / 1000.0);
        if (burnMode != BURN_OFF)
            snprintf(line2, sizeof(line2), "BURN +%.1fEV", burnEv);
        else
            snprintf(line2, sizeof(line2), "%s", currentMode == MODE_BW ? "BW" : "SG");
        mode = PMODE_BURN;
    }
    else if (isMeteringActive()) {
        if (currentMode == MODE_BW) {
            snprintf(header, sizeof(header), "[ BW METER ]");
            snprintf(line1,  sizeof(line1),  "Spots: %d", measBWCount);
            if (measBWCount > 0)
                snprintf(line2, sizeof(line2), "avg:%.1f T2=ADD", measBWSum / (double)measBWCount);
            else
                snprintf(line2, sizeof(line2), "T2=ADD ENC=OK");
            mode = PMODE_METER_BW;
        } else {
            bool isSoft = (measureMode == MM_APPLY_SG_G0);
            int cnt = isSoft ? measSoftCount : measHardCount;
            snprintf(header, sizeof(header), "[ SG %s ]", isSoft ? "SOFT" : "HARD");
            snprintf(line1,  sizeof(line1),  "Spots: %d", cnt);
            snprintf(line2,  sizeof(line2),  "T2=ADD T1=TOG");
            mode = PMODE_METER_SG;
        }
    }
    else if (burnMode != BURN_OFF) {
        snprintf(header, sizeof(header), "[ BURN ]");
        snprintf(line1,  sizeof(line1),  "EV: +%.1f", burnEv);
        double bt = getEffectiveBurnTime();
        snprintf(line2,  sizeof(line2),  "T:%.1fs T2=GO", bt);
        mode = PMODE_BURN;
    }
    else {
        if (currentMode == MODE_BW) {
            snprintf(header, sizeof(header), "[ BW ]");
            snprintf(line1,  sizeof(line1),  "T:%.1fs G:%.1f", time_bw, grade_bw);
        } else {
            snprintf(header, sizeof(header), "[ SPLITGRADE ]");
            snprintf(line1,  sizeof(line1),  "S:%.1f H:%.1f", time_soft, time_hard);
        }
        snprintf(line2, sizeof(line2), "T2=METER");
        mode = PMODE_IDLE;
    }

    sendProbeRender(header, line1, line2, nullptr, HAPTIC_NONE, mode);
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    extern void runCalibrationWizard();

    // Kalibrierungs-Wizard zyklisch aufrufen, wenn aktiv
    extern CalStep calState;
    if (calState != 0) { // CAL_IDLE = 0
      runCalibrationWizard();
      delay(5);
      return;
    }
  wdt_reset();
  
  // Hardware-Interaktion prüfen für Boot-Screen Abruch
  long curSoft = encSoft.getCount();
  long curHard = encHard.getCount();
  long curGrade = encGrade.getCount();
  bool anyButton = (digitalRead(PIN_START) == LOW || 
                    digitalRead(PIN_SW_ENTER) == LOW || 
                    digitalRead(PIN_SW_BACK) == LOW);

  if (bootScreenActive) {
    static long startSoft = curSoft;
    static long startHard = curHard;
    static long startGrade = curGrade;

    if (curSoft != startSoft || curHard != startHard || curGrade != startGrade || anyButton) {
      bootScreenActive = false;
      triggerInfo();
      beepOk();
    } else {
      handleLights();
      DM_loop();
      delay(10);
      return; 
    }
  }

  handleTimer();
  handleLights();
  DM_loop();

  // =====================================================================
  // 1. ATOMICALLY READ PROBE EVENT
  // =====================================================================
  uint8_t probeEvt = EVT_NONE;
  float   pLuxG0 = 0.0f, pLuxG5 = 0.0f;
    popProbeEvent(probeEvt, pLuxG0, pLuxG5);

    // Fokus-Umschaltung (Lichter <-> Schatten) per Remote-Encoder.
    // Sofortiges Render-Feedback ans C6 OLED.
    if (isMeteringActive() && (probeEvt == EVT_ENC_UP || probeEvt == EVT_ENC_DOWN)) {
        currentMeasureFocus = (currentMeasureFocus == FOCUS_HIGHLIGHTS)
            ? FOCUS_SHADOWS : FOCUS_HIGHLIGHTS;
        sendRenderPacketToC6();
        probeEvt = EVT_NONE;
    }

  // =====================================================================
  // 2. TICK FLASH-HANDSHAKE (konsumiert EVT_LUX_DATA)
  // =====================================================================
  if (tickFlashHandshake(probeEvt, pLuxG0, pLuxG5)) {
      probeEvt = EVT_NONE; // Consumed
  }
  if (probeEvt == EVT_HEARTBEAT) probeEvt = EVT_NONE;

  // =====================================================================
  // 3. COLLECT LOCAL INPUTS
  // =====================================================================
  unsigned long now = millis();

  // Grade-Encoder Button (Short/Long Press)
  static bool gradePressed = false;
  static bool gradeLongHandled = false;
  static unsigned long gradePressStart = 0;
  bool gradeRaw = (digitalRead(PIN_SW_GRADE) == LOW);
  bool gradeShort = false;
  bool gradeLong = false;

  if (gradeRaw && !gradePressed) {
    gradePressed = true;
    gradeLongHandled = false;
    gradePressStart = now;
  }
  if (!gradeRaw && gradePressed) {
    if (!gradeLongHandled) gradeShort = true;
    gradePressed = false;
  }
  if (gradePressed && !gradeLongHandled && (now - gradePressStart >= 800)) {
    gradeLong = true;
    gradeLongHandled = true;
  }

  bool evEnter = checkButtonPress(sEnter, PIN_SW_ENTER);
  bool evBack  = checkButtonPress(sBack,  PIN_SW_BACK);
  bool evStart = checkButtonPress(btnStart, PIN_START);

  // Encoder Deltas (lokale Drehgeber)
  static long oldPosSoft = curSoft, oldPosHard = curHard, oldPosGrade = curGrade;
  long dSoft  = curSoft  - oldPosSoft;
  long dHard  = curHard  - oldPosHard;
  long dGrade = curGrade - oldPosGrade;

  // =====================================================================
  // 4. SUPPRESS INPUT DURING ASYNC MEASUREMENT
  // =====================================================================
  // Waehrend Flash-Handshake (C6-Sensor) oder Async-Spot (lokaler Sensor)
  // laeuft: Nur die Messung ticken, alle Eingaben verwerfen.
    bool skipModeLogic = false;
    if (currentMeasureState != MEASURE_IDLE || isAsyncSpotBusy()) {
      if (isMeteringActive()) handleMeteringSession(false, false, false, false, false);

            // Not-Aus: START als Abbruchsignal während aktiver Spektralmessung zulassen.
            if (currentMeasureState != MEASURE_IDLE && (evStart || probeEvt == EVT_T2_CLICK)) {
                abortMeasurementWithError("ABORT");
                probeEvt = EVT_NONE;
            }

      oldPosSoft = curSoft;
      oldPosHard = curHard;
      oldPosGrade = curGrade;
      skipModeLogic = true;
  }

  // =====================================================================
  // 5. INPUT MERGING & MODE DISPATCH
  // =====================================================================
  // Lokale Hardware (Taster, Encoder) und Remote-Events (ESP-NOW) werden
  // ZUERST in logische Action-Flags zusammengefuehrt. Die Modi wissen
  // nicht, woher der Klick kam. → Single Source of Truth
  // =====================================================================
  if (!skipModeLogic) {

    double evStepFactor = 1.0;
    if (globalStepMode == STEP_HALF)  evStepFactor = 0.5;
    if (globalStepMode == STEP_THIRD) evStepFactor = 1.0/3.0;
    if (globalStepMode == STEP_SIXTH) evStepFactor = 1.0/6.0;

    // -----------------------------------------------------------------
    // A) TIMER LAEUFT → nur Stop erlauben
    // -----------------------------------------------------------------
    if (starttime != 0) {
        bool actionStop = evStart
                       || (probeEvt == EVT_T2_CLICK)
                       || (probeEvt == EVT_T1_CLICK);
        if (actionStop) {
            stopTimer();
            if (probeConnected)
                sendProbeRender("[ STOPPED ]", "Timer aborted", "", nullptr, HAPTIC_CLICK, PMODE_IDLE);
        }
    }
    // -----------------------------------------------------------------
    // B) METERING AKTIV → Spot/Save/Toggle/Cancel (lokal + remote)
    // -----------------------------------------------------------------
    else if (isMeteringActive()) {
        bool actionAddLocal  = gradeShort;
        bool actionAddRemote = (probeEvt == EVT_T2_CLICK);
        bool actionSave      = evEnter || (probeEvt == EVT_ENC_CLICK);
        bool actionToggle    = (currentMode == MODE_SG) &&
                               (evBack || probeEvt == EVT_T1_CLICK
                                || probeEvt == EVT_ENC_UP || probeEvt == EVT_ENC_DOWN);
        bool actionCancel    = (currentMode != MODE_SG) &&
                               (evBack || probeEvt == EVT_T1_CLICK);
        bool actionReset     = gradeLong;

        if (actionAddRemote) {
            // Remote-Messung: Flash-Handshake (C6 misst ueber Probe-Sensor)
            startFlashHandshake();
        } else {
            // Lokal: addSpot startet non-blocking Async-Spot (S3-Sensor)
            handleMeteringSession(actionAddLocal, actionSave, actionToggle, actionReset, actionCancel);
        }
    }
    // -----------------------------------------------------------------
    // C) IDLE / BURN → Meter starten, Timer, Encoder, Burn-Toggle
    // -----------------------------------------------------------------
    else {
        // Metering starten (lokal: Grade-Short, remote: T2 im Nicht-Burn)
        bool actionStartMeter = gradeShort
                             || (probeEvt == EVT_T2_CLICK && burnMode == BURN_OFF);
        if (actionStartMeter) {
            startMeteringSession();
        }

        // Timer starten (lokal: Start-Taster, remote: T2 im Burn-Modus)
        else {
            bool actionTimerToggle = evStart
                                  || (probeEvt == EVT_T2_CLICK && burnMode != BURN_OFF);
            if (actionTimerToggle) {
                if (starttime == 0) {
                    startTimer();
                    if (probeConnected)
                        sendProbeRender("[ EXPOSURE ]", "Timer started", "", nullptr, HAPTIC_CLICK, PMODE_BURN);
                } else {
                    stopTimer();
                }
            }

            // EV-Stufe durchschalten (lokal: Enter, remote: Enc-Click)
            bool actionStepCycle = evEnter || (probeEvt == EVT_ENC_CLICK);
            if (actionStepCycle) {
                int nextStep = (int)globalStepMode + 1;
                if (nextStep > 3) nextStep = 0;
                globalStepMode = (StepSize)nextStep;
                beepValue();
                triggerInfo();
            }

            // --- Lokale Encoder ---
            if (dSoft != 0) {
                if (currentMode == MODE_BW) time_bw *= pow(2.0, dSoft * evStepFactor);
                else if (currentMode == MODE_SG) time_soft *= pow(2.0, dSoft * evStepFactor);
                validateTimes();
                triggerInfo();
                oldPosSoft = curSoft;
            }
            if (dHard != 0 && currentMode == MODE_SG) {
                if (time_hard < TIME_MIN_S) time_hard = TIME_MIN_S;
                else time_hard *= pow(2.0, dHard * evStepFactor);
                validateTimes();
                triggerInfo();
                oldPosHard = curHard;
            }
            if (dGrade != 0) {
                if (burnMode != BURN_OFF) {
                    burnEv += (dGrade * evStepFactor);
                    burnEv = constrain(burnEv, 0.0, 3.0);
                    beepValue();
                } else if (currentMode == MODE_BW) {
                    grade_bw += (dGrade * 0.1);
                    grade_bw = constrain(grade_bw, 0.0, 5.0);
                    validateTimes();
                    updateGradeMath();
                }
                triggerInfo();
                oldPosGrade = curGrade;
            }

            // --- Remote Encoder (C6 Drehgeber) ---
            if (probeEvt == EVT_ENC_UP || probeEvt == EVT_ENC_DOWN) {
                double dir = (probeEvt == EVT_ENC_UP) ? 1.0 : -1.0;
                if (burnMode != BURN_OFF) {
                    burnEv += (dir * evStepFactor);
                    burnEv = constrain(burnEv, 0.0, 3.0);
                    beepValue();
                } else if (currentMode == MODE_BW) {
                    time_bw *= pow(2.0, dir * evStepFactor);
                } else if (currentMode == MODE_SG) {
                    time_soft *= pow(2.0, dir * evStepFactor);
                }
                validateTimes();
                triggerInfo();
            }

            // --- Burn-Mode Toggle (langer Grade-Encoder Druck) ---
            if (gradeLong) {
                if (currentMode == MODE_SG) {
                    if (burnMode == BURN_OFF) burnMode = BURN_SG_G0;
                    else if (burnMode == BURN_SG_G0) burnMode = BURN_SG_G5;
                    else burnMode = BURN_OFF;
                } else {
                    if (burnMode == BURN_OFF) { burnMode = BURN_BW; burnGrade = grade_bw; }
                    else burnMode = BURN_OFF;
                }
                burnEv = 0.0;
                beepValue();
                triggerInfo();
            }
        }
    }
  } // end if (!skipModeLogic)

  // =====================================================================
  // 6. HOUSEKEEPING (immer ausgefuehrt, auch waehrend Messung)
  // =====================================================================

  // --- TEMPERATUR MESSUNG (Alle 2 Sekunden) ---
  static unsigned long lastTemp = 0;
  static bool tempRequested = false;

  if (millis() - lastTemp > 2000) {
    if (tempSensorOK) {
        if (!tempRequested) {
            sensors.requestTemperatures();
            tempRequested = true;
        } else {
            float t = sensors.getTempCByIndex(0);
            if (t > -100 && t < 150) tempAlu = t;
            tempRequested = false;
        }
    }
    if (bmpOK && bmpPtr != nullptr) {
       float t = bmpPtr->readTemperature();
       if (!isnan(t) && t > -50 && t < 80) tempRoom = t;
    }
    lastTemp = millis();
  }

  // --- UI UPDATE (100ms) ---
  static unsigned long lastUI = 0;
  if (millis() - lastUI > 100) {
    updateNextionUI(false);
    lastUI = millis();
  }

  // --- PROBE DISPLAY UPDATE (250ms) ---
  if (probeConnected && (millis() - lastRemotePacketMs > 5000)) {
      probeConnected = false;
  }
  {
    static unsigned long lastProbeUpdate = 0;
    if (probeConnected && millis() - lastProbeUpdate > 250) {
        updateProbeDisplay();
        lastProbeUpdate = millis();
    }
  }

    // --- RUNTIME AUDIT TELEMETRY (nur bei Aenderung) ---
    {
        static uint32_t lastDropPkts = 0;
        static uint32_t lastOverruns = 0;
        static unsigned long lastAuditPrint = 0;
        if (millis() - lastAuditPrint > 500) {
            uint32_t dropNow = totalDroppedPackets;
            uint32_t overNow = probeEventOverruns;
            if (dropNow != lastDropPkts || overNow != lastOverruns) {
                Serial.printf("[AUDIT] drop=%lu overrun=%lu qPending=%d conn=%d fh=%d\n",
                    (unsigned long)dropNow,
                    (unsigned long)overNow,
                    probeEventPending ? 1 : 0,
                    probeConnected ? 1 : 0,
                    probeFlashActive ? 1 : 0);
                lastDropPkts = dropNow;
                lastOverruns = overNow;
            }
            lastAuditPrint = millis();
        }
    }

  delay(5);
}