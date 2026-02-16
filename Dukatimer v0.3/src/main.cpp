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
// WIRELESS SENSOR (ESP-NOW)
// =============================================================================
bool useWirelessProbe = false;         // Standard: Kabelgebunden
volatile double remoteLux = 0.0;
volatile unsigned long lastRemotePacketMs = 0;

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
bool whiteLatch = false, safeLatch = false, screenOffOverride = false;

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

  // Metering input (encoder buttons)
  unsigned long now = millis();
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
  bool evBack = checkButtonPress(sBack, PIN_SW_BACK);
  bool evStart = checkButtonPress(btnStart, PIN_START);

  if (isMeteringActive()) {
    bool toggleChannel = (currentMode == MODE_SG) && evBack;
    bool cancel = (currentMode != MODE_SG) && evBack;
    handleMeteringSession(gradeShort, evEnter, toggleChannel, gradeLong, cancel);
    delay(5);
    return;
  }

  if (gradeShort) {
    startMeteringSession();
    delay(5);
    return;
  }

  // EV-Stufen umschalten
  if (evEnter) {
    int nextStep = (int)globalStepMode + 1;
    if (nextStep > 3) nextStep = 0;
    globalStepMode = (StepSize)nextStep;
    beepValue();
    triggerInfo();
  }

  // Encoder Auswertung
  static long oldPosSoft = curSoft, oldPosHard = curHard, oldPosGrade = curGrade;
  long dSoft = curSoft - oldPosSoft;
  long dHard = curHard - oldPosHard;
  long dGrade = curGrade - oldPosGrade;

  double evStepFactor = 1.0;
  if (globalStepMode == STEP_HALF)  evStepFactor = 0.5;
  if (globalStepMode == STEP_THIRD) evStepFactor = 1.0/3.0;
  if (globalStepMode == STEP_SIXTH) evStepFactor = 1.0/6.0;

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
      // Burn-EV Wert begrenzen: Standard 0.0 bis 3.0 (kein negatives Dodging)
      // Wenn negative Werte erlaubt sind: burnEv = constrain(burnEv, -3.0, 3.0);
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

  // ---------------------------------------------------------------------------
  // Burn-Mode Toggle per langem Tastendruck auf Grade-Encoder
  // ---------------------------------------------------------------------------
  // Früher wurde Burn-Mode über das Zeichen 'C' getriggert (handleIdleInput).
  // Da Nextion-Touch-Events jetzt direkt abgearbeitet werden, ist das Aktivieren
  // des Burn-Modes nur noch über den Encoder sinnvoll. Ein langer Druck auf den
  // Grade-Encoder toggelt den Burn-Mode:
  //   - MODE_SG: BURN_OFF → BURN_SG_G0 → BURN_SG_G5 → BURN_OFF
  //   - MODE_BW: BURN_OFF → BURN_BW → BURN_OFF
  // Vorteil: Dead Code in Logic_Timer.cpp wird wieder nutzbar, UI bleibt intuitiv.
  if (gradeLong && !isMeteringActive()) {
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

  // Start-Taster (KORRIGIERT)
  // Pieptöne werden jetzt ausschließlich von der Timer-Engine (Logic_Timer.cpp) gesteuert.
  if (evStart) {
    if (starttime == 0) { startTimer(); }
    else { stopTimer(); }
  }

  // UI Update
// --- TEMPERATUR MESSUNG (Alle 2 Sekunden reicht völlig) ---
  static unsigned long lastTemp = 0;
  static bool tempRequested = false;
  
  if (millis() - lastTemp > 2000) {
    if (tempSensorOK) {
        if (!tempRequested) {
            // REQUEST phase: Non-blocking request
            sensors.requestTemperatures();
            tempRequested = true;
        } else {
            // READ phase: Get result from previous request (conversion has had 2+ seconds)
            float t = sensors.getTempCByIndex(0);
            if (t > -100 && t < 150) tempAlu = t;
            tempRequested = false;
        }
    }

    // 2. Raum Sensor (BMP280) - Instant (~1ms), fine here
    if (bmpOK && bmpPtr != nullptr) {
       float t = bmpPtr->readTemperature();
       if (!isnan(t) && t > -50 && t < 80) tempRoom = t;
    }
    lastTemp = millis();
  }

  // --- UI UPDATE (Weiterhin 100ms für flüssige Bedienung) ---
  static unsigned long lastUI = 0;
  if (millis() - lastUI > 100) {
    updateNextionUI(false); // Nimmt jetzt einfach den gespeicherten Wert von tempAlu
    lastUI = millis();
  }
  
  delay(5);
}