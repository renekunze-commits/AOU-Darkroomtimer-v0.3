
/*
  =============================================================================
  PROJEKT: SINAR P / PHILIPS COLORHEAD CONTROLLER
  VERSION: 2.240 (Probestreifen via Shift+C; Raster 1/6|1/3|1 EV; v2.239 Features bewahrt)
  DATUM:   Januar 2026

  NEU IN 2.240:
  - Probestreifen-Modus (additiv) auf Basis Burn-Logik:
      * Trigger: Shift + C (Numpad)
      * Setup: A/B = ΔEV (Raster aus Step-Mode: 1/6, 1/3, 1), C/D = n (1..10), #=Start, *=Abort
      * Lauf: START = nächste Stufe (additives Inkrement), *=Sofortabbruch, Ende nach n Steps
      * Kanalwahl wie Burn-Kontext: BW / SG(G0/G5)
      * Preflash im Teststreifen deaktiviert
      * Farben: BW=Gradationsfarbe, G0=grün, G5=blau

  ENTHALTENE FEATURES (aus 2.239/2.238):
  - LCD-Backlight korrekt an ON-OFF-ON-Schalter gebunden (RED/OFF/WHITE)
  - Mess-UI: präzise Overlays bei fehlenden K-Werten/Messungen
  - Mess-UI (SG): '*' kurz -> aktiver Kanal löschen; '*' lang (>=1s) -> beide Kanäle löschen
  - Mess-UI (SG): D -> Start (SOFT), erneutes D -> SOFT<->HARD
  - START -> Messung hinzufügen (7×@110ms), Multi-Messungen werden gemittelt
  - '#' -> gemittelte Werte anwenden und Mess-UI beenden
  - Preflash BW & SG (Shift + BTN_D), Flash-Wizard & Slot-Storage, F-Icon
  - Wizard-Fixes (#=SAVE robust + Hardware-Tasten)
  - Densitometer Shortcut (Shift + Numpad 'D')
  - Preflash <0.5s entkoppelt (eigene FLASH_* Grenzwerte)
  =============================================================================
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Keypad.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <math.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>

// =============================================================================
// 1. PIN CONFIG
// =============================================================================

#define PIN_NEOPIX 6
#define PIEZO_PIN 7
#define NUMPIXELS 256

#define PIN_ONE_WIRE 10
#define PIN_SAFE_LIGHT 45
#define PIN_SW_LCD_RED 47
#define PIN_SW_LCD_WHITE 49
#define PIN_LIGHT_ON 51
#define PIN_START 53

#define BTN_A 48
#define BTN_B 46
#define BTN_C 44
#define BTN_D 42   // Hardwaretaste "Gradation -" (Pin 42)
#define BTN_E 40
#define BTN_SHIFT 38

// =============================================================================
// 2. GLOBALE OBJEKTE & DATA STRUCTS
// =============================================================================

Adafruit_TSL2591 tslBase = Adafruit_TSL2591(2591);

bool tslBaseOK = false;
bool tslHeadOK = false;
int tempSensorCount = 0;

double headLux = 0.0;
double tempAlu = 22.0;
double tempAir = 22.0;
const double TEMP_MAX_ALU = 75.0;

double emaLux = NAN;
const double MIN_LUX = 0.5;
const double EMA_ALPHA = 0.20;

bool overheatLock = false;
double currentDose = 0.0;
double targetDose = 0.0;
bool doseModeActive = false;

struct BtnState {
  bool lastLevel;
  bool pressed;
  unsigned long pressStartMs;
  unsigned long lastRepeatMs;
  unsigned long intervalMs;
};

struct SpotMeas {
  double lux;
  uint16_t ch0;
  uint16_t ch1;
  bool ok;
};

// ### PaperProfile inkl. Preflash-Felder (slot-gebunden)
struct PaperProfile {
  double Ksoft;
  double Khard;
  double Kbw;
  bool calibrated;

  // --- Preflash ---
  bool    flashCalibrated; // Schwelle je Slot vorhanden?
  bool    flashEnable;     // Preflash aktiv je Slot
  uint8_t flashLevel;      // 1..5 sehr niedrig
  uint8_t flashColor;      // 0=white, 1=green
  double  flashThreshS;    // Schwellenzeit (Wizard)
  double  flashFactor;     // 0.50..1.50
};

struct BeepSeg { int f; int d; int p; bool prio; };

enum SoundMode { SOUND_NORMAL = 0, SOUND_QUIET = 1, SOUND_OFF = 2 };

struct SettingsObject {
  uint16_t version;
  double t_s;
  double t_h;
  double t_bw;
  double g_bw;
  double burn_g;
  double k_s;
  double k_h;
  double k_bw;
  double std_time;
  uint8_t pwm_safe;
  uint8_t pwm_focus;
  uint8_t pwm_lcd;
  uint8_t pwm_max;
  uint8_t stepMode;
  uint8_t activePaperIdx;
  uint8_t soundMode;
  PaperProfile papers[10];
  bool useDoseMode;
  uint8_t splitMode;
  uint32_t crc;
};

// EEPROM-Version
const uint16_t SETTINGS_VER = 0x0233;

// --- Preflash-Grenzwerte (unabhängig vom Timer-Minimum 0.5s) ---
const double FLASH_MIN_S   = 0.02;  // 20ms
const double FLASH_MAX_S   = 3.00;  // 3.0s
const double FLASH_DT_MIN  = 0.01;  // Wizard-Schritt min
const double FLASH_DT_MAX  = 0.20;  // Wizard-Schritt max

enum Mode { MODE_SG, MODE_BW, MODE_DENS };
enum BurnMode { BURN_OFF, BURN_SG_G0, BURN_SG_G5, BURN_BW };
enum SplitState { SPLIT_IDLE, SPLIT_SOFT_DONE };
enum CurrentExposure { EXP_NONE, EXP_SOFT, EXP_HARD };
enum MeasureMode { MM_OFF, MM_TEACH_SG_G0, MM_TEACH_SG_G5, MM_APPLY_SG_G0, MM_APPLY_SG_G5, MM_TEACH_BW, MM_APPLY_BW };
enum StepSize { STEP_SIXTH, STEP_THIRD, STEP_FULL };

enum CalStep { CAL_IDLE, CAL_G5, CAL_G0, CAL_G25, CAL_REVIEW, CAL_DONE };
CalStep calState = CAL_IDLE;
bool calAbort = false;

// Densitometer
enum DensitometerState { DENS_IDLE, DENS_REF, DENS_BASE, DENS_MEAS };
DensitometerState densState = DENS_IDLE;
double densRefLux = 0.0;
double densBaseFog = NAN;
Mode   prevMode = MODE_SG;
enum DensSubMode { DENS_SUB_MANUAL, DENS_SUB_ZONE1, DENS_SUB_ZONE8 };
DensSubMode densSub = DENS_SUB_MANUAL;
double zone1TargetNet = 0.10;
double zone8TargetNet = 1.20;

// Keypad
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = { { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' }, { '7', '8', '9', 'C' }, { '*', '0', '#', 'D' } };
byte rowPins[ROWS] = { 22, 24, 26, 28 }, colPins[COLS] = { 30, 32, 34, 36 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// LCD & LEDs
rgb_lcd lcd;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIX, NEO_GRB + NEO_KHZ800);

// Laufzustand
Mode currentMode = MODE_SG;
BurnMode burnMode = BURN_OFF;
double burnEv = 0.0, burnGrade = 2.5;
SplitState splitState = SPLIT_IDLE;
CurrentExposure currentExposure = EXP_NONE;
StepSize globalStepMode = STEP_THIRD;

double time_soft = 6.0, time_hard = 10.0, time_bw = 8.0, grade_bw = 2.5;
uint8_t set_safe = 255, set_focus = 255, set_lcd = 255, set_max = 255;

bool settingsDirty = false;
unsigned long lastSettingChange = 0;
const double TIME_MIN_S = 0.5, TIME_MAX_S = 999.0;

unsigned long previousMillis_1 = 0;
unsigned long interval_1 = 1000;
int toggle_timer = 0;
long actual_time = 0;
int starttime = 0;
bool isPaused = false;
bool whiteLatch = false;
int actR = 0, actG = 0, actB = 0, lastLightState = -1;
String lastL1 = "", lastL2 = "";
bool exposureTenths = false;

float trackDensityEV = 0.0;
float trackGradeSteps = 0.0;
unsigned long infoEndTime = 0;
String overlayText = "";

int pendingPaperSlot = -1;
struct KFactors { double Ksoft; double Khard; double Kbw; };
KFactors KF = { NAN, NAN, NAN };
MeasureMode measureMode = MM_OFF;
bool measureHold = false;

#define FILTER_SIZE 5
double luxBuffer[FILTER_SIZE];
int luxBufferIdx = 0, luxBufferCount = 0;
double multiSpotSum = 0.0;
int multiSpotCount = 0;
int currentGainIdx = 1;

int multival[11][3] = { { 0, 255, 0 }, { 0, 235, 20 }, { 0, 215, 40 }, { 0, 195, 60 }, { 0, 170, 85 }, { 0, 143, 112 }, { 0, 115, 140 }, { 0, 85, 170 }, { 0, 60, 200 }, { 0, 30, 225 }, { 0, 0, 255 } };
double paperSpeed[11] = { 1.1, 1.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.1, 1.3, 1.6, 2.0 };
const unsigned long REPEAT_START_INTERVAL = 180, REPEAT_MIN_INTERVAL = 40, REPEAT_ACCEL_STEP = 20;

BtnState sA = { true, false, 0, 0, REPEAT_START_INTERVAL }, sB = { true, false, 0, 0, REPEAT_START_INTERVAL };
BtnState sC = { true, false, 0, 0, REPEAT_START_INTERVAL }, sD = { true, false, 0, 0, REPEAT_START_INTERVAL };
BtnState sE = { true, false, 0, 0, 500 };

unsigned long beepUntil = 0;
SettingsObject globalSet;
unsigned long lastStartEventMs = 0;
const unsigned long START_DEBOUNCE_MS = 120;

// Robust START in Mess-UI
unsigned long measureUiSinceMs = 0;

// --- Akkumulatoren für Mess-UI ---
double measSoftSum = 0.0;   int measSoftCount = 0;
double measHardSum = 0.0;   int measHardCount = 0;
double measBWSum   = 0.0;   int measBWCount   = 0;

// =============================================================================
// 2a. PROBESTREIFEN (TS) – STATES & VARS
// =============================================================================

enum TSState { TS_OFF = 0, TS_SETUP = 1, TS_RUN = 2, TS_DONE = 3 };
TSState ts = TS_OFF;

enum TSChannel { TS_BW = 0, TS_G0 = 1, TS_G5 = 2 };
TSChannel tsCh = TS_BW;

uint8_t tsN = 6;        // Stufen 1..10 (Default 6)
float   tsEv = 1.0f/3.0f;     // ΔEV (Raster: 1/6, 1/3, 1)
uint8_t tsK = 0;        // aktueller Step (0..n-1)
double  tsA[10];        // vorberechnete Inkremente
double  tsSum = 0.0;    // kumuliert
bool    tsActiveExposure = false; // blockt Doppelstart

// =============================================================================
// 3. LOGIK UTIL
// =============================================================================

static uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; ++i) { h ^= data[i]; h *= 16777619u; }
  return h;
}
uint32_t calcCrc(const SettingsObject& s) {
  return fnv1a32((const uint8_t*)&s, sizeof(s) - sizeof(s.crc));
}

void resetTracking() { trackDensityEV = 0.0; trackGradeSteps = 0.0; }

void triggerInfo() {
  String pStr = "P" + String(globalSet.activePaperIdx + 1) + (globalSet.papers[globalSet.activePaperIdx].calibrated ? "[C]" : "[-]");
  String mStr = (globalSet.splitMode == 1) ? "AUT" : "SEQ";
  overlayText = pStr + " " + mStr + " TIME";
  infoEndTime = millis() + 2000;
}
void clearInfo() { infoEndTime = 0; }

int gradeIndex(double g) {
  int idx = (int)lround(g * 2.0);
  if (idx < 0) idx = 0;
  if (idx > 10) idx = 10;
  return idx;
}

void validateTimes() {
  if (isnan(time_soft) || time_soft < TIME_MIN_S) time_soft = 0.5;
  if (time_soft > TIME_MAX_S) time_soft = TIME_MAX_S;
  if (isnan(time_hard) || time_hard < TIME_MIN_S) time_hard = 0.5;
  if (time_hard > TIME_MAX_S) time_hard = TIME_MAX_S;
  if (isnan(time_bw) || time_bw < TIME_MIN_S) time_bw = 0.5;
  if (time_bw > TIME_MAX_S) time_bw = TIME_MAX_S;
}

void markDirty() { settingsDirty = true; lastSettingChange = millis(); }

void applyDensityChange(double factor, double evDelta) {
  time_soft *= factor; time_hard *= factor;
  trackDensityEV += evDelta;
  validateTimes(); markDirty();
}

void applyGradeShift(int direction) {
  if (direction == 0) return;
  float gradeDelta = 0.5f;
  if (globalStepMode == STEP_SIXTH) gradeDelta = 0.1f;
  if (globalStepMode == STEP_FULL)  gradeDelta = 1.0f;
  float signedDelta = (direction > 0) ? gradeDelta : -gradeDelta;
  const double PER_GRADE = 1.20;
  double k = pow(PER_GRADE, signedDelta);
  double T = time_soft + time_hard;
  if (T < 0.1) T = 0.1;
  double r = (time_soft > 0.0) ? (time_hard / time_soft) : 1.0;
  double rNew = r * k;
  double newSoft = T / (1.0 + rNew);
  double newHard = T - newSoft;
  if (newSoft < 0.5) newSoft = 0.5;
  if (newHard < 0.5) newHard = 0.5;
  time_soft = newSoft; time_hard = newHard;
  trackGradeSteps += signedDelta;
  validateTimes(); markDirty();
}

bool allowUiBeeps() { return !(starttime == 1 && !isPaused); }

void beepPattern(const BeepSeg* segs, size_t n) {
  if (globalSet.soundMode == SOUND_OFF) return;
  float scale = (globalSet.soundMode == SOUND_QUIET) ? 0.60f : 1.0f;
  bool hasPrio = false; for (size_t i=0;i<n;i++){ if (segs[i].prio){ hasPrio=true; break; } }
  if (hasPrio) beepUntil = 0;
  for (size_t i=0;i<n;i++){
    wdt_reset();
    int dur = (int)max(20, (int)(segs[i].d * scale));
    int pau = (int)max(0, (int)(segs[i].p * scale));
    tone(PIEZO_PIN, segs[i].f, dur);
    unsigned long tEnd = millis() + dur;
    while (millis() < tEnd) { wdt_reset(); delay(1); }
    if (pau > 0) {
      unsigned long pEnd = millis() + pau;
      while (millis() < pEnd) { wdt_reset(); delay(1); }
    }
  }
  if (n > 0) beepUntil = millis() + 30;
}
void beepNav()        { if (!allowUiBeeps()) return; BeepSeg s[]={{ 800, 60,0,false}}; beepPattern(s,1); }
void beepValue()      { if (!allowUiBeeps()) return; BeepSeg s[]={{ 900, 65,0,false}}; beepPattern(s,1); }
void beepOk()         { BeepSeg s[]={{2200, 60,0,false}}; beepPattern(s,1); }
void beepHint()       { BeepSeg s[]={{ 500,140,0,true }}; beepPattern(s,1); }
void beepWarnLong()   { BeepSeg s[]={{ 400,320,0,true }}; beepPattern(s,1); }
void beepWizardStep() { BeepSeg s[]={{1700, 50,40,false},{2000,60,0,false}}; beepPattern(s,2); }
void beepWizardSave() { BeepSeg s[]={{2000, 50,40,false},{2400,70,0,false}}; beepPattern(s,2); }
void beepStartPattern(){BeepSeg s[]={{1100, 80,60,true },{1400,80,0,true }}; beepPattern(s,2); }
void beepEndPattern() { BeepSeg s[]={{1400,120,30,true },{1700,220,0,true }}; beepPattern(s,2); }
void beepAlarm()      { BeepSeg s[]={{3000,1000,0,true }}; beepPattern(s,1); }

bool updateButton(BtnState& st, int pin, unsigned long nowMs) {
  bool level = digitalRead(pin);
  bool fell = (st.lastLevel == HIGH && level == LOW);
  bool rose = (st.lastLevel == LOW && level == HIGH);
  st.lastLevel = level;
  if (fell) {
    st.pressed = true;
    st.pressStartMs = nowMs;
    st.lastRepeatMs = nowMs;
    st.intervalMs = REPEAT_START_INTERVAL;
    return true;
  }
  if (rose) {
    st.pressed = false;
    return false;
  }
  if (st.pressed && (nowMs - st.pressStartMs >= 350)) {
    if (nowMs - st.lastRepeatMs >= st.intervalMs) {
      st.lastRepeatMs = nowMs;
      if (st.intervalMs > REPEAT_MIN_INTERVAL + REPEAT_ACCEL_STEP) st.intervalMs -= REPEAT_ACCEL_STEP;
      else st.intervalMs = REPEAT_MIN_INTERVAL;
      return true;
    }
  }
  return false;
}

String fmtTime(double t, bool tenths) { return tenths ? String(t,1)+"s" : String((int)t)+"s"; }

long secondsToUnits(double s, int tglFlag) {
  if (s <= 0) return 0;
  bool useTenths = (tglFlag == 1) || (s < 1.0);
  long units = useTenths ? (long)(s * 10.0 + 0.5) : (long)(s + 0.5);
  if (units < 1) units = 1;
  return units;
}

uint8_t scalePwm(int val) { return (uint8_t)((val * set_max) / 255); }

void PaintLED(int r, int g, int b) {
  int r_o = scalePwm(r), g_o = scalePwm(g), b_o = scalePwm(b);
  uint32_t c = pixels.Color(r_o, g_o, b_o);
  pixels.fill(c, 0, NUMPIXELS);
  pixels.show();
}

// ===== LCD Backlight (ON-OFF-ON) =====
void handleLCDBacklight() {
  bool swRed   = (digitalRead(PIN_SW_LCD_RED)   == LOW);
  bool swWhite = (digitalRead(PIN_SW_LCD_WHITE) == LOW);

  int mode = 0; // 0=OFF,1=RED,2=WHITE
  if (swRed && !swWhite)       mode = 1;
  else if (!swRed && swWhite)  mode = 2;
  else                         mode = 0;

  static int lastMode = -1;
  static uint8_t lastBright = 255;

  if (mode != lastMode || set_lcd != lastBright) {
    if (mode == 0) {
      lcd.setRGB(0, 0, 0);
    } else if (mode == 1) {
      uint8_t r = (uint8_t)((50 * set_lcd) / 255);
      lcd.setRGB(r, 0, 0);
    } else {
      uint8_t w = (uint8_t)((50 * set_lcd) / 255);
      lcd.setRGB(w, w, w);
    }
    lastMode   = mode;
    lastBright = set_lcd;
  }
}

/**
 * smartLCD:
 * - Obere Zeile (l1): Content max. 14 Zeichen.
 * - Spalte 15 (Index 14): Dirty-Indikator (voller Block 0xFF), wenn settingsDirty & !starttime.
 * - Spalte 16 (Index 15): frei.
 * - Untere Zeile (l2): 16 Zeichen total.
 */
void smartLCD(String l1, String l2) {
  if (l1.length() > 14) l1 = l1.substring(0,14);
  while (l1.length() < 14) l1 += " ";
  while (l1.length() < 16) l1 += " ";
  if (settingsDirty && !starttime) l1.setCharAt(14, (char)0xFF);
  else l1.setCharAt(14, ' ');
  if (l2.length() > 16) l2 = l2.substring(0,16);
  while (l2.length() < 16) l2 += " ";
  if (l1 != lastL1) { lcd.setCursor(0,0); lcd.print(l1); lastL1 = l1; }
  if (l2 != lastL2) { lcd.setCursor(0,1); lcd.print(l2); lastL2 = l2; }
  handleLCDBacklight();
}

// =============================================================================
// 4. SENSOR
// =============================================================================

SpotMeas readSpot() {
  SpotMeas m = { NAN, 0, 0, false };
  if (!tslBaseOK) return m;
  static uint8_t skip = 0;
  static double lastLux = NAN;
  uint32_t lum = tslBase.getFullLuminosity();
  uint16_t rawFull = lum & 0xFFFF;
  bool gainChanged = false;
  if (rawFull > 60000 && currentGainIdx > 0) { currentGainIdx--; gainChanged = true; }
  else if (rawFull < 500 && currentGainIdx < 2) { currentGainIdx++; gainChanged = true; }
  if (gainChanged) {
    tsl2591Gain_t g = (currentGainIdx == 0) ? TSL2591_GAIN_LOW : (currentGainIdx == 1) ? TSL2591_GAIN_MED : TSL2591_GAIN_HIGH;
    tslBase.setGain(g);
    tslBase.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    skip = 3;
    if (!isnan(lastLux)) { m.lux = lastLux; m.ok = true; }
    return m;
  }
  if (skip > 0) {
    skip--;
    if (!isnan(lastLux)) { m.lux = lastLux; m.ok = true; }
    return m;
  }
  uint16_t ch0 = rawFull;
  uint16_t ch1 = lum >> 16;
  double curLux = tslBase.calculateLux(ch0, ch1);
  if (curLux <= 0.0001) curLux = 0.0001;
  luxBuffer[luxBufferIdx] = curLux;
  luxBufferIdx = (luxBufferIdx + 1) % FILTER_SIZE;
  if (luxBufferCount < FILTER_SIZE) luxBufferCount++;
  double sum = 0;
  for (int i=0;i<luxBufferCount;i++) sum += luxBuffer[i];
  m.lux = sum / (double)luxBufferCount;
  m.ch0 = ch0; m.ch1 = ch1;
  m.ok = true;
  lastLux = m.lux;
  return m;
}

// =============================================================================
// 5. WIZARDS (Papier-Kalibrierung, Densitometer, Preflash)
// =============================================================================

double takeAveragedLux(uint8_t samples = 7, uint16_t delayMs = 110) {
  double acc = 0; int n = 0;
  for (uint8_t i=0; i<samples; ++i) {
    wdt_reset();
    SpotMeas sm = readSpot();
    if (sm.ok) { acc += sm.lux; n++; }
    delay(delayMs);
  }
  if (n == 0) return NAN;
  return acc / (double)n;
}

void setSplitGradeLightForGrade(double g) {
  int idx = gradeIndex(g);
  actR = multival[idx][0];
  actG = multival[idx][1];
  actB = multival[idx][2];
  PaintLED(actR, actG, actB);
}

// --- Papier-Kalibrier-Wizard (robuste Eingaben) ---
void startCalibrationWizard() {
  int p = globalSet.activePaperIdx;
  calState = CAL_G5; calAbort = false;
  lcd.clear(); lcd.print("CAL P"); lcd.print(p + 1);
  lcd.setCursor(0,1); lcd.print("1/3 G5 SHADOW   ");
  PaintLED(0,0,255);
}

void runCalibrationWizard() {
  char k = keypad.getKey();
  bool startPressed = (digitalRead(PIN_START) == LOW);
  if (k == '*') calAbort = true;

  switch (calState) {

    case CAL_G5: {
      smartLCD("G5: SHADOW", "START=MEASURE *ABT");
      if (startPressed) {
        beepOk(); delay(150);
        double avg = takeAveragedLux();
        if (!isnan(avg) && time_hard > 0.05) {
          globalSet.papers[globalSet.activePaperIdx].Khard = avg * time_hard;
          beepWizardStep();

          lcd.clear(); lcd.print("Khard:"); lcd.print(globalSet.papers[globalSet.activePaperIdx].Khard,1);
          lcd.setCursor(0,1); lcd.print("A=REPEAT #=OK ");
          while (1) {
            wdt_reset();
            char kk2 = keypad.getKey();
            unsigned long tNow2 = millis();
            bool aEdge2 = updateButton(sA, BTN_A, tNow2);
            bool bEdge2 = updateButton(sB, BTN_B, tNow2);
            static bool lastStart2 = HIGH;
            bool startH2 = (digitalRead(PIN_START) == LOW);
            bool startEdge2 = (startH2 && !lastStart2);
            lastStart2 = startH2;

            if (kk2 == '*') { calAbort = true; break; }
            if (kk2 == 'A' || aEdge2) { PaintLED(0,0,255); lcd.clear(); lcd.print("Repeat G5..."); delay(400); break; }
            if (kk2 == '#' || startEdge2 || bEdge2) { calState = CAL_G0; PaintLED(0,255,0); lcd.clear(); lcd.print("2/3 G0 HIGHL    "); delay(300); break; }
            delay(10);
          }

        } else {
          beepWarnLong(); lcd.clear(); lcd.print("G5 FAILED");
          lcd.setCursor(0,1); lcd.print("A=RETRY *=ABRT");
          while (1) { wdt_reset(); char kk=keypad.getKey(); unsigned long tNow=millis(); bool aEdge=updateButton(sA,BTN_A,tNow); if (kk=='*'){calAbort=true;break;} if (kk=='A'||aEdge) break; delay(10); }
        }
      }
    } break;

    case CAL_G0: {
      smartLCD("G0: HIGHLIGHT", "START=MEASURE *ABT");
      if (startPressed) {
        beepOk(); delay(150);
        double avg = takeAveragedLux();
        if (!isnan(avg) && time_soft > 0.05) {
          globalSet.papers[globalSet.activePaperIdx].Ksoft = avg * time_soft;
          beepWizardStep();

          lcd.clear(); lcd.print("Ksoft:"); lcd.print(globalSet.papers[globalSet.activePaperIdx].Ksoft,1);
          lcd.setCursor(0,1); lcd.print("A=REPEAT #=OK ");
          while (1) {
            wdt_reset();
            char kk2 = keypad.getKey();
            unsigned long tNow2 = millis();
            bool aEdge2 = updateButton(sA, BTN_A, tNow2);
            bool bEdge2 = updateButton(sB, BTN_B, tNow2);
            static bool lastStart2 = HIGH;
            bool startH2 = (digitalRead(PIN_START) == LOW);
            bool startEdge2 = (startH2 && !lastStart2);
            lastStart2 = startH2;

            if (kk2 == '*') { calAbort = true; break; }
            if (kk2 == 'A' || aEdge2) { PaintLED(0,255,0); lcd.clear(); lcd.print("Repeat G0..."); delay(400); break; }
            if (kk2 == '#' || startEdge2 || bEdge2) { calState = CAL_G25; setSplitGradeLightForGrade(2.5); lcd.clear(); lcd.print("3/3 G2.5 MID    "); delay(300); break; }
            delay(10);
          }

        } else {
          beepWarnLong(); lcd.clear(); lcd.print("G0 FAILED");
          lcd.setCursor(0,1); lcd.print("A=RETRY *=ABRT");
          while (1) { wdt_reset(); char kk=keypad.getKey(); unsigned long tNow=millis(); bool aEdge=updateButton(sA,BTN_A,tNow); if (kk=='*'){calAbort=true;break;} if (kk=='A'||aEdge) break; delay(10); }
        }
      }
    } break;

    case CAL_G25: {
      smartLCD("G2.5: MIDTONE", "START=MEASURE *ABT");
      if (startPressed) {
        beepOk(); delay(150);
        double avg = takeAveragedLux();
        if (!isnan(avg) && time_bw > 0.05) {
          globalSet.papers[globalSet.activePaperIdx].Kbw = avg * time_bw;
          calState = CAL_REVIEW;
          lcd.clear(); lcd.print("REVIEW K-VALUES");
          lcd.setCursor(0,1); lcd.print("#=SAVE A=REP ");
        } else {
          beepWarnLong(); lcd.clear(); lcd.print("G2.5 FAILED");
          lcd.setCursor(0,1); lcd.print("A=RETRY *=ABRT");
          while (1) { wdt_reset(); char kk=keypad.getKey(); unsigned long tNow=millis(); bool aEdge=updateButton(sA,BTN_A,tNow); if (kk=='*'){calAbort=true;break;} if (kk=='A'||aEdge) break; delay(10); }
        }
      }
    } break;

    case CAL_REVIEW: {
      static unsigned long t0 = 0; static uint8_t page = 0;

      char kk = keypad.getKey();
      unsigned long now2 = millis();
      static bool lastStartH = HIGH;
      bool startH = (digitalRead(PIN_START) == LOW);
      bool startEdge = (startH && !lastStartH);
      lastStartH = startH;
      bool aEdge = updateButton(sA, BTN_A, now2);
      bool bEdge = updateButton(sB, BTN_B, now2);

      if (kk == '*') { calAbort=true; break; }
      if (kk == 'A' || aEdge) { calState=CAL_G5; PaintLED(0,0,255); lcd.clear(); lcd.print("Repeat All..."); delay(500); break; }
      if (kk == '#' || startEdge || bEdge) {
        globalSet.papers[globalSet.activePaperIdx].calibrated=true; markDirty(); saveSettings(); beepWizardSave();
        calState=CAL_DONE; lcd.clear(); lcd.print("CALIB SAVED"); delay(700); break;
      }

      unsigned long now = millis();
      if (now - t0 > 900) { t0 = now; page = (page + 1) % 3; }
      if (page==0) smartLCD("Khard:"+String(globalSet.papers[globalSet.activePaperIdx].Khard,1), "#=SAVE  A=REP  ");
      else if (page==1) smartLCD("Ksoft:"+String(globalSet.papers[globalSet.activePaperIdx].Ksoft,1), "#=SAVE  A=REP  ");
      else smartLCD("Kbw:  "+String(globalSet.papers[globalSet.activePaperIdx].Kbw,1), "#=SAVE  A=REP  ");
    } break;

    case CAL_DONE: {
      smartLCD("CALIBRATION OK", "Press START exit");
      if (digitalRead(PIN_START) == LOW) calState = CAL_IDLE;
    } break;

    default: break;
  }

  if (calAbort) {
    calState = CAL_IDLE; calAbort=false;
    PaintLED(0,0,0); lcd.clear(); lcd.print("CALIB ABORTED"); delay(700); lcd.clear();
  }
}

// --- Densitometer Helfer ---
double calculateDensity(double measLux) {
  if (measLux <= 0.0001 || densRefLux <= 0.0001) return NAN;
  return log10(densRefLux / measLux);
}
void densUpdateLed() {
  switch (densState) {
    case DENS_REF:  PaintLED(255,0,0);   break;
    case DENS_BASE: PaintLED(255,200,0); break;
    case DENS_MEAS: PaintLED(0,255,0);   break;
    default:        PaintLED(0,0,0);     break;
  }
}
void enterDensMode() {
  if (starttime) return;
  prevMode = currentMode; currentMode = MODE_DENS;
  densState = DENS_REF; densSub = DENS_SUB_MANUAL;
  densRefLux = 0.0;
  densUpdateLed(); lcd.clear(); beepOk();
}
void exitDensMode() {
  currentMode = prevMode; densState = DENS_IDLE; densSub = DENS_SUB_MANUAL;
  PaintLED(0,0,0); lcd.clear(); beepNav();
}

// =============================================================================
// 6. PREFLASH: Wizard + Pulse + Menü-Funktionen
// =============================================================================

PaperProfile& CURP() { return globalSet.papers[globalSet.activePaperIdx]; }
const PaperProfile& CCURP() { return globalSet.papers[globalSet.activePaperIdx]; }

void flashPulseRaw(uint8_t level, uint8_t color, double secs) {
  if (secs <= 0.0) return;
  uint8_t bri = constrain(level, 1, 5);
  int r=0,g=0,b=0;
  if (color == 0) { r=bri; g=bri; b=bri; } else { g=bri; }
  pixels.clear();
  uint32_t c = pixels.Color(r, g, b);
  for (int i=0;i<NUMPIXELS;i++) pixels.setPixelColor(i, c);
  pixels.show();
  unsigned long until = millis() + (unsigned long)(secs * 1000.0);
  while (millis() < until) { wdt_reset(); delay(1); }
  pixels.clear(); pixels.show();
}

void maybeDoPreflashBeforeExposure() {
  PaperProfile &PP = CURP();
  bool flashArmed = PP.flashCalibrated && PP.flashEnable && (PP.flashThreshS > 0.0);
  if (!flashArmed) return;

  // Hinweis: Probestreifen nutzt eigene Belichtung (kein Preflash)
  if (false) { return; } // kein Preflash im TS (Blockierung über TS-Pfad selbst)

  bool isBurn = (burnMode != BURN_OFF);
  bool okBW   = (currentMode == MODE_BW);
  bool okSG   = (currentMode == MODE_SG && splitState == SPLIT_IDLE);

  if (isBurn) return;
  if (!(okBW || okSG)) return;

  double tF = PP.flashThreshS * max(0.5, min(1.5, PP.flashFactor));
  tF = max(FLASH_MIN_S, min(FLASH_MAX_S, tF));
  beepHint();
  smartLCD("FLASH", String(tF, 2) + "s");
  flashPulseRaw(PP.flashLevel, PP.flashColor, tF);
  delay(50);
}

void flashCalibWizard() {
  PaperProfile &PP = CURP();

  lcd.clear();
  smartLCD("FLASH CALIB", "Set Grey REF");
  delay(800);
  smartLCD("Press START", "to store REF");
  while (digitalRead(PIN_START) == HIGH) { wdt_reset(); delay(10); }
  beepOk();

  double refTime = (currentMode == MODE_BW) ? time_bw : (time_soft > 0.05 ? time_soft : time_hard);
  Mode   refMode = currentMode;
  smartLCD("REF STORED", String(refTime,1)+"s"); delay(600);

  int steps = 6;
  double dt = 0.05;
  uint8_t lvl = (PP.flashLevel >= 1 && PP.flashLevel <= 5) ? PP.flashLevel : 2;
  uint8_t col = (PP.flashColor <= 1) ? PP.flashColor : 0;

  bool paramLoop = true;
  while (paramLoop) {
    String L0 = "ST:"+String(steps)+" dt:"+String(dt,2);
    String L1 = String("LV:")+String(lvl)+" "+(col==0?"WHT":"GRN")+"  #=OK";
    smartLCD(L0, L1);
    char k = keypad.getKey();
    bool evA=false, evB=false, evC=false, evD=false;
    unsigned long now=millis();
    evA = updateButton(sA, BTN_A, now);
    evB = updateButton(sB, BTN_B, now);
    evC = updateButton(sC, BTN_C, now);
    evD = updateButton(sD, BTN_D, now);

    if (k == '#') { beepOk(); break; }
    if (k == '*') { beepWarnLong(); return; }

    if (evA) { steps = constrain(steps + 1, 3, 10); beepValue(); }
    if (evB) { steps = constrain(steps - 1, 3, 10); beepValue(); }
    if (evC) { dt = min(FLASH_DT_MAX, dt + 0.01); beepValue(); }
    if (evD) { dt = max(FLASH_DT_MIN, dt - 0.01); beepValue(); }

    if (digitalRead(BTN_SHIFT) == LOW) {
      if (evA) { lvl = (uint8_t)min(5, (int)lvl + 1); beepValue(); }
      if (evB) { lvl = (uint8_t)max(1, (int)lvl - 1); beepValue(); }
      if (evC) { col = 0; beepValue(); }
      if (evD) { col = 1; beepValue(); }
    }
    delay(15);
  }

  for (int i=0;i<=steps;i++) {
    wdt_reset();
    double tFlash = i * dt;
    tFlash = max(FLASH_MIN_S, min(FLASH_MAX_S, tFlash));

    String L0 = "STEP " + String(i) + "/" + String(steps);
    String L1 = "FLASH t=" + String(tFlash,2) + "s";
    smartLCD(L0, L1);
    while (digitalRead(PIN_START) == HIGH) { wdt_reset(); delay(5); }
    beepHint();
    flashPulseRaw(lvl, col, tFlash);
    delay(80);
    smartLCD("GREY REF", String(refTime,1)+"s");
    if (refMode == MODE_BW) {
      int gi = gradeIndex(grade_bw);
      PaintLED(multival[gi][0], multival[gi][1], multival[gi][2]);
    } else {
      setSplitGradeLightForGrade(2.5);
    }
    unsigned long tEnd = millis() + (unsigned long)(refTime*1000.0);
    while (millis() < tEnd) { wdt_reset(); delay(1); }
    PaintLED(0,0,0);
    delay(200);
  }

  smartLCD("Develop strip", "Pick 1.."+String(steps)+" *=ABT");
  int pick = -1;
  while (pick == -1) {
    wdt_reset();
    char k = keypad.getKey();
    if (k == '*') { beepWarnLong(); return; }
    if (k >= '1' && k <= '9') {
      int v = k - '0';
      if (v >= 1 && v <= steps) pick = v;
    }
    delay(10);
  }
  int idx = max(0, pick - 1); // letzte unkritische Stufe
  double thresh = idx * dt;
  thresh = max(FLASH_MIN_S, min(FLASH_MAX_S, thresh));

  PP.flashCalibrated = true;
  PP.flashEnable = false;
  PP.flashLevel = lvl;
  PP.flashColor = col;
  PP.flashThreshS = thresh;
  PP.flashFactor = 1.00;
  markDirty(); saveSettings();
  beepWizardSave();
  smartLCD("THRESH SAVED", String(thresh,2)+"s");
  delay(800);
}

// =============================================================================
// 7. SETUP (inkl. FLASH-Menü & Dens im Menü)
// =============================================================================

void handleSetup() {
  int menuIdx = 0;
  bool inSetup = true;
  bool inLightSub = false;
  int lightSubIdx = 0;
  bool inSoundSub = false;
  int soundSubIdx = 0;
  bool inFlashSub = false;
  int flashSubIdx = 0;

  lcd.clear(); lastL1 = ""; lastL2 = "";
  while (inSetup) {
    wdt_reset();
    unsigned long now = millis();
    bool menuPrev = updateButton(sA, BTN_A, now), menuNext = updateButton(sB, BTN_B, now);
    bool valUp = updateButton(sC, BTN_C, now), valDown = updateButton(sD, BTN_D, now);
    handleLCDBacklight();

    if (calState != CAL_IDLE) { runCalibrationWizard(); delay(15); continue; }

    if (digitalRead(PIN_START) == LOW) {
      saveSettings(); inSetup = false; lcd.clear(); PaintLED(0,0,0); beepOk();
      while (digitalRead(PIN_START) == LOW) { delay(10); wdt_reset(); }
      delay(40); return;
    }

    // SOUND SUB
    if (inSoundSub) {
      if (menuNext) { soundSubIdx++; if (soundSubIdx>3) soundSubIdx=0; beepNav(); lcd.clear(); }
      if (menuPrev) { soundSubIdx--; if (soundSubIdx<0) soundSubIdx=3; beepNav(); lcd.clear(); }
      String tSub=""; bool apply = (valUp || valDown);
      switch (soundSubIdx) {
        case 0: tSub="PROFILE NORMAL"; if (apply){ globalSet.soundMode=SOUND_NORMAL; markDirty(); beepOk(); } break;
        case 1: tSub="PROFILE QUIET";  if (apply){ globalSet.soundMode=SOUND_QUIET;  markDirty(); beepOk(); } break;
        case 2: tSub="PROFILE OFF";    if (apply){ globalSet.soundMode=SOUND_OFF;    markDirty(); } break;
        case 3: tSub="BACK / EXIT";    if (apply){ inSoundSub=false; beepNav(); lcd.clear(); } break;
      }
      lcd.setCursor(0,0); lcd.print("SOUND: " + tSub);
      lcd.setCursor(0,1);
      String cur=(globalSet.soundMode==SOUND_OFF)?"CUR: OFF   ":(globalSet.soundMode==SOUND_QUIET)?"CUR: QUIET ":"CUR: NORMAL";
      lcd.print(cur); delay(15); continue;
    }

    // LIGHT SUB
    if (inLightSub) {
      if (menuNext) { lightSubIdx++; if (lightSubIdx>4) lightSubIdx=0; beepNav(); lcd.clear(); }
      if (menuPrev) { lightSubIdx--; if (lightSubIdx<0) lightSubIdx=4; beepNav(); lcd.clear(); }
      String tSub=""; uint8_t* vPtr=NULL;
      switch (lightSubIdx) {
        case 0: tSub="SAFE LIGHT"; vPtr=&set_safe; PaintLED(set_safe,0,0); break;
        case 1: tSub="FOCUS LIGHT"; vPtr=&set_focus; PaintLED(set_focus,set_focus,set_focus); break;
        case 2: tSub="LCD BRIGHT"; vPtr=&set_lcd; PaintLED(0,0,0); break;
        case 3: tSub="MAX POWER";  vPtr=&set_max; PaintLED(255,255,255); break;
        case 4: tSub="BACK / EXIT"; PaintLED(0,0,0);
                if (valUp||valDown){ inLightSub=false; beepNav(); lcd.clear(); } break;
      }
      if (vPtr!=NULL){
        if (valUp)   { *vPtr=(uint8_t)constrain((int)*vPtr+5,0,255); beepValue(); }
        if (valDown) { *vPtr=(uint8_t)constrain((int)*vPtr-5,0,255); beepValue(); }
        lcd.setCursor(0,1); lcd.print("VAL: " + String(*vPtr) + "   ");
      }
      lcd.setCursor(0,0); lcd.print("LIGHT: " + tSub); delay(15); continue;
    }

    // FLASH SUB (slot-gebunden)
    if (inFlashSub) {
      if (menuNext) { flashSubIdx++; if (flashSubIdx>4) flashSubIdx=0; beepNav(); lcd.clear(); }
      if (menuPrev) { flashSubIdx--; if (flashSubIdx<0) flashSubIdx=4; beepNav(); lcd.clear(); }

      PaperProfile &PP = CURP();
      String title="FLASH ";
      String L1="";

      switch (flashSubIdx) {
        case 0: {
          title += "ENABLE";
          if (!PP.flashCalibrated || PP.flashThreshS <= 0.0) {
            L1 = "CALIB MISSING   ";
          } else {
            L1 = PP.flashEnable ? "ON  (Up/Down)" : "OFF (Up/Down)";
            if (valUp || valDown) { PP.flashEnable = !PP.flashEnable; markDirty(); beepValue(); }
          }
        } break;
        case 1: {
          title += "FACTOR";
          L1 = String("VAL: ") + String(PP.flashFactor,2);
          if (valUp)   { PP.flashFactor = min(1.50, PP.flashFactor + 0.05); markDirty(); beepValue(); }
          if (valDown) { PP.flashFactor = max(0.50, PP.flashFactor - 0.05); markDirty(); beepValue(); }
        } break;
        case 2: {
          title += "LEVEL";
          L1 = String("VAL: ") + String(PP.flashLevel);
          if (valUp)   { PP.flashLevel = (uint8_t)min(5, (int)PP.flashLevel + 1); markDirty(); beepValue(); }
          if (valDown) { PP.flashLevel = (uint8_t)max(1, (int)PP.flashLevel - 1); markDirty(); beepValue(); }
        } break;
        case 3: {
          title += "COLOR";
          L1 = (PP.flashColor==0) ? "WHITE" : "GREEN";
          if (valUp || valDown) { PP.flashColor = (PP.flashColor==0)?1:0; markDirty(); beepValue(); }
        } break;
        case 4: {
          title += "CALIBRATE";
          L1 = "Up/Down = RUN   ";
          if (valUp || valDown) { beepOk(); lcd.clear(); flashCalibWizard(); lcd.clear(); }
        } break;
      }

      lcd.setCursor(0,0); lcd.print(title);
      lcd.setCursor(0,1); lcd.print(L1);
      delay(15); continue;
    }

    PaintLED(0,0,0);

    // Menü wechseln
    if (menuNext) { menuIdx++; if (menuIdx>9) menuIdx=0; beepNav(); lcd.clear(); lastL1=""; }
    if (menuPrev) { menuIdx--; if (menuIdx<0) menuIdx=9; beepNav(); lcd.clear(); lastL1=""; }

    String title="";
    switch (menuIdx) {
      case 0:
        title="TEACH PAPER"; lcd.setCursor(0,1); lcd.print("Press RED-1 (C)");
        if (valUp || (keypad.getKey()=='C')) { startCalibrationWizard(); }
        break;
      case 1:
        title="LIGHT CONFIG"; lcd.setCursor(0,1); lcd.print("Press Up/Down   ");
        if (valUp||valDown){ inLightSub=true; beepNav(); lcd.clear(); }
        break;
      case 2:
        title="SOUND CONFIG"; lcd.setCursor(0,1); lcd.print("Press Up/Down   ");
        if (valUp||valDown){ inSoundSub=true; beepNav(); lcd.clear(); }
        break;
      case 3:
        title="TIMER MODE"; lcd.setCursor(0,1); lcd.print("TIME (NO HEAD)  ");
        break;
      case 4:
        title="HEAD MONITOR"; lcd.setCursor(0,1); lcd.print("SENSORS DISABLED");
        break;
      case 5:
        title="SPLITGRADE"; lcd.setCursor(0,1); lcd.print(globalSet.splitMode==1?"AUT (CONT)      ":"SEQ (PAUSE)     ");
        if (valUp||valDown){ globalSet.splitMode=!globalSet.splitMode; beepValue(); triggerInfo(); }
        break;
      case 6:
        title="STEP MODE"; lcd.setCursor(0,1);
        {
          String s=(globalStepMode==STEP_SIXTH)?"1/6":(globalStepMode==STEP_FULL)?"1/1":"1/3";
          lcd.print("CUR: "); lcd.print(s); lcd.print("    ");
          if (valUp||valDown){
            if (globalStepMode==STEP_SIXTH) globalStepMode=STEP_THIRD;
            else if (globalStepMode==STEP_THIRD) globalStepMode=STEP_FULL;
            else globalStepMode=STEP_SIXTH;
            globalSet.stepMode=(uint8_t)globalStepMode; markDirty(); beepValue();
          }
        } break;
      case 7:
        title="STD TIME RESET"; lcd.setCursor(0,1);
        lcd.print("VAL: "); lcd.print(globalSet.std_time,1); lcd.print("s    ");
        if (valUp){ globalSet.std_time += 0.5; markDirty(); beepValue(); }
        if (valDown){ globalSet.std_time -= 0.5; if (globalSet.std_time<0.5) globalSet.std_time=0.5; markDirty(); beepValue(); }
        break;
      case 8:
        title="FILM TEST"; lcd.setCursor(0,1); lcd.print("Up/Down = START ");
        if (valUp||valDown){ inSetup=false; lcd.clear(); enterDensMode(); return; }
        break;
      case 9:
        title="FLASH"; lcd.setCursor(0,1); lcd.print("Press Up/Down   ");
        if (valUp||valDown){ inFlashSub=true; flashSubIdx=0; beepNav(); lcd.clear(); }
        break;
    }
    lcd.setCursor(0,0); lcd.print("SETUP: " + title);
    delay(15);
  }
}

// =============================================================================
// 8. SAVE/LOAD
// =============================================================================

void saveSettings() {
  validateTimes();
  globalSet.version = SETTINGS_VER;
  globalSet.t_s = time_soft; globalSet.t_h = time_hard; globalSet.t_bw = time_bw;
  globalSet.g_bw = grade_bw; globalSet.burn_g = burnGrade;
  globalSet.k_s = 0;  globalSet.k_h = 0;  globalSet.k_bw = 0;
  globalSet.pwm_safe = set_safe; globalSet.pwm_focus = set_focus;
  globalSet.pwm_lcd = set_lcd;   globalSet.pwm_max = set_max;
  globalSet.activePaperIdx = constrain(globalSet.activePaperIdx, 0, 9);
  globalSet.stepMode = (uint8_t)globalStepMode;
  globalSet.crc = 0; globalSet.crc = calcCrc(globalSet);
  EEPROM.put(0, globalSet);
  settingsDirty = false;
}

void defaultsSettings() {
  memset(&globalSet, 0, sizeof(globalSet));
  globalSet.version = SETTINGS_VER;
  time_soft = 6.0; time_hard = 10.0; time_bw = 8.0; grade_bw = 2.5;
  set_safe=255; set_focus=255; set_lcd=100; set_max=255;
  globalSet.std_time = 8.0;
  globalSet.t_s=time_soft; globalSet.t_h=time_hard; globalSet.t_bw=time_bw;
  globalSet.g_bw=grade_bw; globalSet.burn_g=burnGrade;
  globalSet.pwm_safe=set_safe; globalSet.pwm_focus=set_focus; globalSet.pwm_lcd=set_lcd; globalSet.pwm_max=set_max;
  globalSet.activePaperIdx=0; globalSet.useDoseMode=false; globalSet.splitMode=0;
  globalStepMode=STEP_THIRD; globalSet.stepMode=(uint8_t)globalStepMode;
  globalSet.soundMode=SOUND_NORMAL;

  for (int i=0;i<10;i++) {
    globalSet.papers[i].Ksoft = 0; globalSet.papers[i].Khard = 0; globalSet.papers[i].Kbw = 0;
    globalSet.papers[i].calibrated = false;
    globalSet.papers[i].flashCalibrated = false;
    globalSet.papers[i].flashEnable = false;
    globalSet.papers[i].flashLevel = 2;
    globalSet.papers[i].flashColor = 0;
    globalSet.papers[i].flashThreshS = 0.0;
    globalSet.papers[i].flashFactor = 1.00;
  }

  globalSet.crc=0; globalSet.crc=calcCrc(globalSet);
  EEPROM.put(0, globalSet);
}

void migratePaperDefaultsIfNeeded() {
  for (int i=0;i<10;i++) {
    PaperProfile &P = globalSet.papers[i];
    if (isnan(P.flashThreshS) || P.flashThreshS < 0.0 || P.flashLevel == 0) {
      P.flashCalibrated = false; P.flashEnable = false;
      P.flashLevel = 2; P.flashColor = 0;
      P.flashThreshS = 0.0; P.flashFactor = 1.00;
    }
  }
}

void loadSettings() {
  EEPROM.get(0, globalSet);
  if (globalSet.version != SETTINGS_VER || calcCrc(globalSet) != globalSet.crc) {
    defaultsSettings();
  } else {
    time_soft = globalSet.t_s; time_hard = globalSet.t_h; time_bw = globalSet.t_bw;
    grade_bw = globalSet.g_bw; burnGrade = globalSet.burn_g;
    set_safe = globalSet.pwm_safe; set_focus = globalSet.pwm_focus;
    set_lcd = globalSet.pwm_lcd;   set_max = globalSet.pwm_max;
    if (globalSet.activePaperIdx > 9) globalSet.activePaperIdx = 0;
    if (globalSet.stepMode <= STEP_FULL) globalStepMode=(StepSize)globalSet.stepMode; else globalStepMode=STEP_THIRD;
    if (globalSet.std_time < 0.1) globalSet.std_time = 8.0;
    if (globalSet.soundMode > SOUND_OFF) globalSet.soundMode = SOUND_NORMAL;
    migratePaperDefaultsIfNeeded();
  }
  validateTimes(); resetTracking();
}

// =============================================================================
// 9. SETUP & LOOP
// =============================================================================

void setup() {
  wdt_enable(WDTO_4S);
  pinMode(PIN_SW_LCD_RED, INPUT_PULLUP);
  pinMode(PIN_SW_LCD_WHITE, INPUT_PULLUP);
  pinMode(PIN_SAFE_LIGHT, INPUT_PULLUP);
  pinMode(PIN_LIGHT_ON, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_D, INPUT_PULLUP);
  pinMode(BTN_E, INPUT_PULLUP);
  pinMode(BTN_SHIFT, INPUT_PULLUP);

  Wire.begin();
  lcd.begin(16, 2);

  // Keypad Hold für Long-Press '*'
  keypad.setHoldTime(1000); // 1s Long-Press für '*'

  if (digitalRead(BTN_SHIFT) == LOW) {
    lcd.setRGB(255,0,0); lcd.print("FACTORY RESET...");
    defaultsSettings(); delay(2000); lcd.clear();
  }

  loadSettings();
  handleLCDBacklight();
  pixels.begin();

  if (tslBase.begin()) {
    tslBaseOK = true;
    tslBase.setGain(TSL2591_GAIN_MED);
    tslBase.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  } else { tslBaseOK = false; }

  tslHeadOK = false; tempSensorCount = 0; globalSet.useDoseMode = false;

  lcd.print("LAB TIMER V2.240");
  lcd.setCursor(0,1);
  if (tslBaseOK) lcd.print("BASE SENS OK"); else lcd.print("BASE SENS MISS");
  delay(1500);
  lcd.clear();
  triggerInfo();
}

bool applyMeasuredLuxToTimes(double luxAvg, MeasureMode mm) {
  if (!(luxAvg > 0.0) || !isfinite(luxAvg)) return false;
  const PaperProfile& PP = CURP();
  if (mm == MM_APPLY_BW) {
    if (!(PP.Kbw > 0.0)) return false;
    time_bw = PP.Kbw / luxAvg; validateTimes(); markDirty(); return true;
  } else if (mm == MM_APPLY_SG_G0) {
    if (!(PP.Ksoft > 0.0)) return false;
    time_soft = PP.Ksoft / luxAvg; validateTimes(); markDirty(); return true;
  } else if (mm == MM_APPLY_SG_G5) {
    if (!(PP.Khard > 0.0)) return false;
    time_hard = PP.Khard / luxAvg; validateTimes(); markDirty(); return true;
  }
  return false;
}

// --- Helfer: EV-Raster zum Faktor ---
float stepModeToEv() {
  if (globalStepMode == STEP_SIXTH) return 1.0f/6.0f;
  if (globalStepMode == STEP_FULL)  return 1.0f;
  return 1.0f/3.0f; // STEP_THIRD
}

// --- Probestreifen: Inkremente vorberechnen (Center-Anchor) ---
void tsPrepareIncrements() {
  // Basiszeit nach Kanal
  double Tbase = (tsCh==TS_BW) ? time_bw : (tsCh==TS_G0) ? time_soft : time_hard;
  if (Tbase < 0.1) Tbase = 0.1;

  // Raster-ΔEV sicherstellen
  float ev = stepModeToEv();
  // tsEv folgt Raster; (tsEv kann z.B. vom Setup angepasst worden sein, bleibt im Raster)
  if (fabs(tsEv - (1.0f/6.0f)) < 0.0001f) ev = 1.0f/6.0f;
  else if (fabs(tsEv - (1.0f/3.0f)) < 0.0001f) ev = 1.0f/3.0f;
  else if (fabs(tsEv - 1.0f) < 0.0001f) ev = 1.0f;
  tsEv = ev;

  double r = pow(2.0, (double)tsEv);
  int kstar = (tsN>0) ? ((tsN-1)/2) : 0;
  double denom = pow(r, (double)(kstar + 1)) - 1.0;
  if (denom <= 0.0) denom = 1.0;
  double a = Tbase * (r - 1.0) / denom;

  for (int i=0;i<tsN;i++) {
    double Ai = a * pow(r, (double)i);
    if (Ai < 0.10) Ai = 0.10; // Mindestzeit
    if (Ai > TIME_MAX_S) Ai = TIME_MAX_S;
    tsA[i] = Ai;
  }
  tsK = 0; tsSum = 0.0; tsActiveExposure = false;
}

// --- Probestreifen: Einen Step (A_k) belichten (blocking) ---
void tsExposeIncrement(double dur) {
  // Focus-Check
  if (digitalRead(PIN_LIGHT_ON) == LOW) {
    beepWarnLong(); lcd.clear(); lcd.print("FOCUS IS ON!");
    delay(900); lcd.clear();
    return;
  }

  // LED-Farbe je Kanal setzen
  int r=0,g=0,b=0;
  if (tsCh == TS_BW) {
    int idx = gradeIndex(grade_bw);
    r = multival[idx][0]; g = multival[idx][1]; b = multival[idx][2];
  } else if (tsCh == TS_G0) { g = 255; }
  else                      { b = 255; }

  // Start
  beepStartPattern();
  PaintLED(r,g,b);

  bool useTenths = (dur < 1.0) || (toggle_timer==1);
  long units = secondsToUnits(dur, useTenths ? 1 : 0);
  unsigned long iv = useTenths ? 100UL : 1000UL;

  unsigned long t0 = millis();
  while (units-- > 0) {
    while (millis() - t0 < iv) { wdt_reset(); delay(1); }
    t0 += iv;
  }

  PaintLED(0,0,0);
  beepEndPattern();
  if (digitalRead(PIN_LIGHT_ON)==LOW) whiteLatch=true; // wie im Haupttimer
}

// --- Probestreifen: Setup-UI Render ---
void tsRenderSetup() {
  // Zeile 1: Kanal + n
  String ch = (tsCh==TS_BW) ? ("BW G"+String(grade_bw,1)) : (tsCh==TS_G0 ? "G0" : "G5");
  String L0 = "TS " + ch + " n=" + String(tsN);
  // Zeile 2: ΔEV + Start
  String evStr = (fabs(tsEv - (1.0f/6.0f))<0.0001f) ? "1/6" : (fabs(tsEv - (1.0f/3.0f))<0.0001f) ? "1/3" : "1/1";
  String L1 = String("\x7F") + "EV=" + evStr + "  #=Start"; // \x7F ~ Pfeil/nützlich; optional
  smartLCD(L0, L1);
}

// --- Probestreifen: Lauf-UI Render ---
void tsRenderRun() {
  double add = (tsK < tsN) ? tsA[tsK] : 0.0;
  String L0 = "TS " + String(tsK) + "/" + String(tsN) + " add:" + String(add, (add<10.0)?1:0) + "s";
  String L1 = String("\xE5") + ":" + String(tsSum, (tsSum<10.0)?1:0) + "s START=NEXT"; // \xE5 ~ Sigma? (LCD 16x2 meist ASCII, belasse "Σ:" als Text)
  // Da LCD evtl. kein Sigma hat, nutze "Σ:" als ASCII:
  L1 = String("S:") + String(tsSum, (tsSum<10.0)?1:0) + "s START=NEXT";
  smartLCD(L0, L1);
}

// =============================================================================
// 10. LOOP
// =============================================================================

void loop() {
  wdt_reset();
  unsigned long now = millis();
  handleLCDBacklight();

  if (overheatLock) {
    wdt_reset();
    smartLCD("!!! OVERHEAT !!!", String("ALU: ")+String(tempAlu,1)+" C");
    char kk = keypad.getKey();
    if (kk=='*' && tempAlu < (TEMP_MAX_ALU - 5.0)) {
      overheatLock = false; lcd.clear(); beepOk();
    }
    delay(200); return;
  }

  bool evA = updateButton(sA, BTN_A, now), evB = updateButton(sB, BTN_B, now);
  bool evC = updateButton(sC, BTN_C, now), evD = updateButton(sD, BTN_D, now);
  bool evE = updateButton(sE, BTN_E, now);
  bool isShift = (digitalRead(BTN_SHIFT) == LOW);
  if (evA || evB || evC || evD || evE || isShift) clearInfo();
  char k = keypad.getKey();
  if (k != NO_KEY) clearInfo();

  // Setup per Shift + #
  if (isShift && k=='#') { handleSetup(); lastLightState=-1; return; }

  // Densitometer Start/Exit: Shift + Numpad D
  if (isShift && k=='D' && starttime==0 && ts==TS_OFF) {
    if (currentMode != MODE_DENS) enterDensMode();
    else                          exitDensMode();
    return;
  }

  // === Probestreifen: Start per Shift + C ===
  if (isShift && k=='C' && starttime==0 && currentMode!=MODE_DENS && ts==TS_OFF) {
    // Kanalwahl aus Kontext
    if (currentMode == MODE_BW) tsCh = TS_BW;
    else {
      if (burnMode == BURN_SG_G5) tsCh = TS_G5;
      else                        tsCh = TS_G0;
    }
    // ΔEV aus Raster
    tsEv = stepModeToEv();
    tsN  = 6;
    ts   = TS_SETUP;
    lcd.clear(); beepOk();
  }

  validateTimes();

  // Safe/Focus Schalter
  bool sSafe = (digitalRead(PIN_SAFE_LIGHT) == LOW), sWhite = (digitalRead(PIN_LIGHT_ON) == LOW);
  int targetLS = 0;
  if (!sWhite) whiteLatch = false;
  if (sSafe) { targetLS = 1; if (sWhite) whiteLatch = true; }
  else if (sWhite && !whiteLatch) { targetLS = 2; }
  if (starttime == 0 && ts==TS_OFF) {
    if (targetLS != lastLightState) {
      if (targetLS == 1) PaintLED(set_safe,0,0);
      else if (targetLS == 2) PaintLED(set_focus,set_focus,set_focus);
      else PaintLED(0,0,0);
      lastLightState = targetLS;
    }
  } else lastLightState = -1;

  // Papierslot Auswahl und allgemeine Shortcuts nur, wenn TS nicht aktiv
  if (ts == TS_OFF) {
    if (k>='0' && k<='9' && starttime==0) {
      pendingPaperSlot = (k=='0')?9:(k-'1'); beepValue();
    } else if (k=='*') {
      if (pendingPaperSlot != -1) {
        globalSet.activePaperIdx = pendingPaperSlot; pendingPaperSlot = -1;
        saveSettings();
        lcd.clear(); lcd.print("P"+String(globalSet.activePaperIdx+1)+" LOADED");
        beepOk(); delay(700); lcd.clear();
        resetTracking(); triggerInfo();
      } else if (starttime==1 && isPaused) {
        starttime=0; isPaused=false; PaintLED(0,0,0); beepWarnLong();
        if (digitalRead(PIN_LIGHT_ON)==LOW) whiteLatch=true;
      } else {
        if (currentMode==MODE_BW) time_bw=globalSet.std_time;
        else { time_soft=globalSet.std_time; time_hard=globalSet.std_time; }
        resetTracking(); beepOk();
        lcd.clear(); lcd.print("RESET TO "); lcd.print(globalSet.std_time,1); lcd.print("s");
        delay(700); lcd.clear(); triggerInfo();
      }
    } else if (k != NO_KEY && pendingPaperSlot != -1) pendingPaperSlot = -1;
    else if (k == '#') { toggle_timer = !toggle_timer; beepValue(); }
    else if (k == 'A' && starttime==0 && currentMode!=MODE_DENS) {
      currentMode = MODE_BW; burnMode=BURN_OFF; measureMode=MM_OFF; beepNav(); triggerInfo(); resetTracking();
    } else if (k == 'B' && starttime==0 && currentMode!=MODE_DENS) {
      currentMode = MODE_SG; splitState=SPLIT_IDLE; burnMode=BURN_OFF; measureMode=MM_OFF; beepNav(); triggerInfo(); resetTracking();
    } else if (k == 'C' && starttime==0 && currentMode!=MODE_DENS) {
      if (currentMode==MODE_SG) {
        if (burnMode==BURN_OFF) burnMode=BURN_SG_G0;
        else if (burnMode==BURN_SG_G0) burnMode=BURN_SG_G5;
        else burnMode=BURN_OFF;
      } else {
        if (burnMode==BURN_OFF) { burnMode=BURN_BW; burnGrade=grade_bw; }
        else burnMode=BURN_OFF;
      }
      burnEv=0.0; beepValue();
    }
  }

  // ===== SHIFT + BTN_D (Pin 42) = Preflash TOGGLE (BW & SG) =====
  if (isShift && evD && starttime==0 && (currentMode==MODE_BW || currentMode==MODE_SG) && ts==TS_OFF) {
    PaperProfile &PP = CURP();
    if (!PP.flashCalibrated || PP.flashThreshS <= 0.0) {
      beepWarnLong(); smartLCD("NO FLASH CAL", "Run FLASH CALIB"); delay(1100);
    } else {
      PP.flashEnable = !PP.flashEnable; markDirty(); saveSettings();
      double eff = PP.flashThreshS * max(0.5, min(1.5, PP.flashFactor));
      eff = max(FLASH_MIN_S, min(FLASH_MAX_S, eff));
      beepOk();
      smartLCD(PP.flashEnable ? "FLASH ON" : "FLASH OFF", String("T=")+String(eff,2)+"s");
      delay(800);
    }
    evD = false;
  }

  // ===== Densitometer MODE =====
  if (currentMode == MODE_DENS) {
    if (starttime) { PaintLED(0,0,0); starttime=0; isPaused=false; }

    SpotMeas m = readSpot();
    bool sStart = (digitalRead(PIN_START) == LOW);
    static bool lastStartD = HIGH;
    bool startEdge = (sStart && !lastStartD);
    lastStartD = sStart;
    bool safeOn = (digitalRead(PIN_SAFE_LIGHT) == LOW);

    if (k=='A') { densSub = DENS_SUB_ZONE1; beepNav(); }
    if (k=='B') { densSub = DENS_SUB_ZONE8; beepNav(); }
    if (densSub == DENS_SUB_ZONE8) {
      if (evC) { zone8TargetNet = min(1.35, zone8TargetNet + 0.05); beepValue(); }
      if (evD) { zone8TargetNet = max(1.10, zone8TargetNet - 0.05); beepValue(); }
    }
    if (k=='*' && densState==DENS_MEAS && m.ok) {
      double Dtot = calculateDensity(m.lux);
      if (!isnan(Dtot)) { densBaseFog = Dtot; beepOk(); } else beepWarnLong();
    }

    if (startEdge) {
      if (densState==DENS_REF) {
        if (safeOn) { beepWarnLong(); }
        else {
          smartLCD("REF AVG...", "Measuring...");
          double avg = takeAveragedLux(7,110);
          if (!isnan(avg)) {
            densRefLux = avg; densState=DENS_BASE; beepOk(); densUpdateLed();
            smartLCD("REF SET OK", String("Lux ")+String(avg,1)); delay(450);
          } else beepWarnLong();
        }
      } else if (densState==DENS_BASE) {
        if (safeOn) { beepWarnLong(); }
        else if (densRefLux > 0.0) {
          smartLCD("BASE AVG...", "Measuring...");
          double avg = takeAveragedLux(7,110);
          if (!isnan(avg)) {
            double Dtot = log10(densRefLux / avg);
            densBaseFog = Dtot; densState=DENS_MEAS; beepOk(); densUpdateLed();
            smartLCD("BASE SET", String("Dmin ")+String(Dtot,2)); delay(450);
          } else beepWarnLong();
        } else beepWarnLong();
      } else {
        beepValue();
      }
    }

    String L0="", L1="";
    if (densState==DENS_REF) {
      L0="REF OPEN GATE";
      L1 = safeOn ? "SAFE OFF! START" : "START=AVG  ShD=EX";
    } else if (densState==DENS_BASE) {
      L0="MEASURE BASE";
      if (safeOn) L1="SAFE OFF! START";
      else if (m.ok && densRefLux>0.0) {
        double Dpreview = log10(densRefLux / max(0.0001, m.lux));
        L1=String("Dbase ")+String(Dpreview,2)+" AVG=START";
      } else L1="Place clear base";
    } else {
      double Dtot = (m.ok && densRefLux>0.0) ? calculateDensity(m.lux) : NAN;
      double Dnet = (!isnan(Dtot) && !isnan(densBaseFog)) ? (Dtot - densBaseFog) : NAN;
      if (densSub==DENS_SUB_MANUAL) { L0 = String("D: ") + (isnan(Dtot)?"--":String(Dtot,2)); String netStr = isnan(Dnet)?"--":String(Dnet,2); L1 = String("Net ")+netStr+" *=Z0 ShD=EX"; }
      else if (densSub==DENS_SUB_ZONE1) { String netStr = isnan(Dnet)?"--":String(Dnet,2); L0="ZONE I TARGET"; L1=String("Net ")+netStr+" ->0.10"; }
      else { String netStr = isnan(Dnet)?"--":String(Dnet,2); String tStr=String(zone8TargetNet,2); L0="ZONE VIII TG"; L1=String("Net ")+netStr+" T:"+tStr; if ((int)L1.length()<16){ while((int)L1.length()<13) L1+=" "; L1+="C/D"; } }
    }
    densUpdateLed();
    smartLCD(L0,L1); delay(15); return;
  }

  // ==============================
  // ===== PROBESTREIFEN UI =======
  // ==============================
  if (ts != TS_OFF) {
    // Setup-Phase
    if (ts == TS_SETUP) {
      tsRenderSetup();

      // Raster-ΔEV umschalten per A/B
      if (evA) {
        // next higher: 1/6 -> 1/3 -> 1/1 -> 1/1
        if (fabs(tsEv - (1.0f/6.0f))<0.0001f) tsEv = 1.0f/3.0f;
        else if (fabs(tsEv - (1.0f/3.0f))<0.0001f) tsEv = 1.0f;
        else tsEv = 1.0f;
        beepValue();
      }
      if (evB) {
        // next lower: 1/1 -> 1/3 -> 1/6 -> 1/6
        if (fabs(tsEv - 1.0f)<0.0001f) tsEv = 1.0f/3.0f;
        else if (fabs(tsEv - (1.0f/3.0f))<0.0001f) tsEv = 1.0f/6.0f;
        else tsEv = 1.0f/6.0f;
        beepValue();
      }

      // n (Stufen) per C/D
      if (evC) { tsN = (uint8_t)min(10, (int)tsN + 1); beepValue(); }
      if (evD) { tsN = (uint8_t)max(1,  (int)tsN - 1); beepValue(); }

      // Start
      if (k == '#') {
        tsPrepareIncrements();
        lcd.clear(); beepOk();
        ts = TS_RUN;
      }

      // Abbruch
      if (k == '*') { ts = TS_OFF; lcd.clear(); beepWarnLong(); }

      delay(15); return;
    }

    // Lauf-Phase
    if (ts == TS_RUN) {
      tsRenderRun();

      // Abort
      if (k == '*') {
        ts = TS_OFF; PaintLED(0,0,0); lcd.clear(); beepWarnLong(); return;
      }

      // Starttaste -> nächstes Inkrement
      bool sStart = (digitalRead(PIN_START) == LOW);
      static bool lastStartTS = HIGH;
      bool startEdgeTS = (sStart && !lastStartTS);
      lastStartTS = sStart;

      if (startEdgeTS && !tsActiveExposure) {
        if (tsK < tsN) {
          tsActiveExposure = true;
          double dur = tsA[tsK];
          tsExposeIncrement(dur);
          tsSum += dur;
          tsK++;
          tsActiveExposure = false;
          if (tsK >= tsN) { ts = TS_DONE; }
        } else {
          ts = TS_DONE;
        }
      }

      delay(15); return;
    }

    // Done-Phase
    if (ts == TS_DONE) {
      smartLCD("TS DONE ("+String(tsN)+")", "Press any key");
      // beliebige Taste oder START -> Exit
      if (k != NO_KEY || (digitalRead(PIN_START)==LOW)) {
        beepOk(); ts = TS_OFF; lcd.clear();
      }
      delay(15); return;
    }
  }

  // ===== Standard-Timer START/Belichtung (nur wenn TS_OFF) =====

  // Start-Taste
  bool sStart = (digitalRead(PIN_START) == LOW);
  static bool lastStart = HIGH;

  if (pendingPaperSlot != -1) {
    if (sStart && !lastStart && (now - lastStartEventMs >= START_DEBOUNCE_MS)) {
      lastStartEventMs = now; beepHint(); lcd.clear(); lcd.print("Confirm P with *");
      delay(800); lastL1 = ""; lastL2 = "";
    }
    lastStart = sStart;
    goto RENDER_SECTION;
  }

  if (sStart && !lastStart) {
    clearInfo();
    if (now - lastStartEventMs >= START_DEBOUNCE_MS) {
      lastStartEventMs = now;
      if (starttime == 0 && (digitalRead(PIN_LIGHT_ON) == LOW)) {
        beepWarnLong(); lcd.clear(); lcd.print("FOCUS IS ON!");
        delay(1200); lastL1 = ""; lastL2 = ""; lastStart = sStart; return;
      }
      if (starttime == 1) {
        isPaused = !isPaused;
        if (isPaused) { PaintLED(set_safe,0,0); beepNav(); lastStart=sStart; return; }
        else { PaintLED(actR,actG,actB); previousMillis_1 = now; beepOk(); lastStart=sStart; return; }
      } else {
        double t_load = 0;
        if (burnMode != BURN_OFF) {
          double base = (burnMode==BURN_SG_G0) ? time_soft : (burnMode==BURN_SG_G5 ? time_hard : time_bw);
          t_load = base * (pow(2.0, burnEv) - 1.0);
          if (burnMode==BURN_SG_G0) { actR=0; actG=255; actB=0; }
          else if (burnMode==BURN_SG_G5) { actR=0; actG=0; actB=255; }
          else { int idx=gradeIndex(burnGrade); actR=multival[idx][0]; actG=multival[idx][1]; actB=multival[idx][2]; }
        } else {
          if (currentMode == MODE_SG) {
            if (splitState == SPLIT_IDLE) {
              t_load = time_soft; actR=0; actG=255; actB=0; currentExposure=EXP_SOFT; splitState=SPLIT_SOFT_DONE;
            } else {
              t_load = time_hard; actR=0; actG=0; actB=255; currentExposure=EXP_HARD; splitState=SPLIT_IDLE;
            }
          } else {
            t_load = time_bw;
            int idx=gradeIndex(grade_bw); actR=multival[idx][0]; actG=multival[idx][1]; actB=multival[idx][2];
            currentExposure=EXP_NONE;
          }
        }

        if (t_load >= 0.1) {
          // Preflash nur im Standardtimer, nicht in TS
          maybeDoPreflashBeforeExposure();

          bool useTenths = (toggle_timer==1) || (t_load < 1.0);
          actual_time = secondsToUnits(t_load, useTenths ? 1 : 0);
          if (actual_time >= 1) {
            interval_1 = useTenths ? 100UL : 1000UL;
            exposureTenths = useTenths;
            PaintLED(actR,actG,actB);
            starttime=1; isPaused=false; previousMillis_1=now;
            beepStartPattern(); lastStart=sStart; return;
          } else beepWarnLong();
        }
      }
    }
  }
  lastStart = sStart;

  // Laufende Belichtung
  if (starttime == 1 && !isPaused) {
    if ((now - previousMillis_1) >= interval_1) {
      previousMillis_1 += interval_1;
      actual_time--;
      if (actual_time <= 0) {
        if (currentMode==MODE_SG && currentExposure==EXP_SOFT && globalSet.splitMode==1 && burnMode==BURN_OFF) {
          PaintLED(0,0,0); delay(120);
          double t_next = time_hard;
          if (t_next >= 0.1) {
            bool useT = (toggle_timer==1) || (t_next < 1.0);
            actual_time = secondsToUnits(t_next, useT ? 1 : 0);
            interval_1 = useT ? 100UL : 1000UL;
            exposureTenths = useT;
            actR=0; actG=0; actB=255; PaintLED(actR,actG,actB);
            currentExposure=EXP_HARD; splitState=SPLIT_IDLE;
            previousMillis_1 = millis();
            beepStartPattern(); return;
          }
        }
        starttime=0; PaintLED(0,0,0); beepEndPattern();
        if (digitalRead(PIN_LIGHT_ON)==LOW) whiteLatch=true;
      }
    }
    double disp = exposureTenths ? (actual_time/10.0) : (double)actual_time;
    smartLCD("EXPOSING...", String(disp,1)+"s"); return;
  } else if (isPaused) {
    double disp = exposureTenths ? (actual_time/10.0) : (double)actual_time;
    smartLCD("** PAUSED **", String("Rest: ")+String(disp,1)+"s"); return;
  }

  if (settingsDirty && (now - lastSettingChange > 1000)) saveSettings();

  // EV Step Mode über BTN_E
  if (evE) {
    if (globalStepMode==STEP_SIXTH) globalStepMode=STEP_THIRD;
    else if (globalStepMode==STEP_THIRD) globalStepMode=STEP_FULL;
    else globalStepMode=STEP_SIXTH;
    globalSet.stepMode=(uint8_t)globalStepMode; markDirty(); beepValue(); clearInfo();
  }

  // Bedienlogik (kein TS aktiv)
  if (measureMode == MM_OFF && pendingPaperSlot == -1) {
    double evS = 0.333; if (globalStepMode==STEP_SIXTH) evS=1.0/6.0; else if (globalStepMode==STEP_THIRD) evS=1.0/3.0; else evS=1.0;
    double fUp=pow(2.0,evS), fDown=pow(2.0,-evS);

    if (burnMode != BURN_OFF) {
      if (evA){ burnEv += evS; markDirty(); }
      if (evB){ burnEv -= evS; if (burnEv < 0) burnEv=0; markDirty(); }
      if (burnMode == BURN_BW) {
        bool ch=false; if (evC){ burnGrade += 0.5; ch=true; } if (evD){ burnGrade -= 0.5; ch=true; }
        if (ch){ burnGrade = constrain(burnGrade,0.0,5.0); markDirty(); }
      } else {
        if (evC || evD) beepValue();
      }
    } else {
      if (currentMode == MODE_SG) {
        if (isShift) {
          if (evA) { applyDensityChange(fUp, evS); }
          if (evB) { applyDensityChange(fDown, -evS); }
          if (evC) { applyGradeShift(1); }
          if (evD) { /* Shift+BTN_D -> Preflash bereits oben */ }
        } else {
          bool ch=false;
          if (evA){ time_soft *= fUp; ch=true; }
          if (evB){ time_soft *= fDown; ch=true; }
          if (evC){ time_hard *= fUp; ch=true; }
          if (evD){ time_hard *= fDown; ch=true; }
          if (ch){ validateTimes(); markDirty(); resetTracking(); }
        }
      } else {
        if (evA){ time_bw *= fUp; trackDensityEV += evS; markDirty(); }
        if (evB){ time_bw *= fDown; trackDensityEV -= evS; markDirty(); }
        int oldI = gradeIndex(grade_bw);
        bool ch=false;
        if (evC){ grade_bw += 0.5; ch=true; }
        if (evD){ grade_bw -= 0.5; ch=true; }
        if (ch){
          grade_bw = constrain(grade_bw,0.0,5.0);
          int nI = gradeIndex(grade_bw);
          time_bw *= (paperSpeed[nI] / paperSpeed[oldI]);
          markDirty();
        }
      }
    }
  }

  // ===== Mess-UI per Numpad D (bestehend) =====
  if (k == 'D' && !isShift) {
    bool busy=false; const char* why="";
    if (starttime==1)                { busy=true; why="BUSY: EXPOSURE"; }
    else if (currentMode==MODE_DENS) { busy=true; why="BUSY: DENS MODE"; }
    else if (calState!=CAL_IDLE)     { busy=true; why="BUSY: WIZARD"; }

    if (busy) { smartLCD(lastL1, why); delay(1200); goto RENDER_SECTION; }

    if (currentMode == MODE_BW) measureMode = MM_APPLY_BW;
    else                        measureMode = MM_APPLY_SG_G0; // Start in SOFT
    multiSpotSum=0.0; multiSpotCount=0; measureHold=false; beepNav(); clearInfo();
    measureUiSinceMs = millis();
  }

RENDER_SECTION:
  if (pendingPaperSlot != -1) {
    String pName = "P" + String(pendingPaperSlot + 1);
    smartLCD("CONFIRM " + pName + "?", CCURP().calibrated ? "PRESS * TO OK" : "NOT CALIBRATED");
  } else {
    String L0="", L1="";
    String stepStr=(globalStepMode==STEP_SIXTH)?"1/6":(globalStepMode==STEP_FULL)?"1/1":"1/3";

    if (measureMode != MM_OFF) {
      SpotMeas m = readSpot();

      // START robust (Flanke oder frühes Hold in 1200ms)
      bool sStartM = (digitalRead(PIN_START)==LOW);
      static bool lastStartM = HIGH;
      bool edgePress = (sStartM && !lastStartM);
      bool earlyHold = (sStartM && (millis() - measureUiSinceMs) < 1200UL);
      bool startTrigger = edgePress || earlyHold;
      lastStartM = sStartM;

      // '*': Short vs. Longpress
      KeyState ks = keypad.getState();
      bool starShort = (k == '*' && (ks == PRESSED || ks == RELEASED));
      bool starHold  = (k == '*' && ks == HOLD);

      if (starHold && currentMode == MODE_SG) {
        measSoftSum = 0.0; measSoftCount = 0;
        measHardSum = 0.0; measHardCount = 0;
        beepValue();
        smartLCD("BOTH CLEARED",""); delay(250);
      } else if (starShort) {
        if (currentMode == MODE_BW) {
          measBWSum = 0.0; measBWCount = 0;
          beepValue(); smartLCD("BW CLEARED",""); delay(250);
        } else {
          if (measureMode == MM_APPLY_SG_G0) {
            measSoftSum = 0.0; measSoftCount = 0;
            beepValue(); smartLCD("SOFT CLEARED",""); delay(250);
          } else if (measureMode == MM_APPLY_SG_G5) {
            measHardSum = 0.0; measHardCount = 0;
            beepValue(); smartLCD("HARD CLEARED",""); delay(250);
          }
        }
      }

      // '#' -> gemittelt anwenden & beenden (mit klareren Fehlermeldungen)
      if (k == '#') {
        const PaperProfile &PP = CCURP();
        bool haveKsoft = (PP.Ksoft > 0.0);
        bool haveKhard = (PP.Khard > 0.0);
        bool haveKbw   = (PP.Kbw   > 0.0);
        bool ok=false;

        if (currentMode==MODE_BW) {
          if (measBWCount > 0 && haveKbw) {
            double avg = measBWSum / (double)measBWCount;
            time_bw = PP.Kbw / avg; validateTimes(); ok = true;
          }
        } else {
          if (measSoftCount > 0 && haveKsoft) {
            double avgS = measSoftSum / (double)measSoftCount;
            time_soft = PP.Ksoft / avgS; validateTimes(); ok = true;
          }
          if (measHardCount > 0 && haveKhard) {
            double avgH = measHardSum / (double)measHardCount;
            time_hard = PP.Khard / avgH; validateTimes(); ok = true;
          }
        }

        if (!ok) {
          if ((currentMode == MODE_BW && measBWCount == 0) ||
              (currentMode == MODE_SG && measSoftCount == 0 && measHardCount == 0)) {
            beepWarnLong(); smartLCD("NO MEASURES", "Use START to add"); delay(700);
          } else if (currentMode == MODE_BW && !haveKbw) {
            beepWarnLong(); smartLCD("NO Kbw", "Run TEACH PAPER"); delay(900);
          } else if (currentMode == MODE_SG) {
            if (!haveKsoft && !haveKhard) { beepWarnLong(); smartLCD("NO Ksoft/Khard", "Run TEACH PAPER"); delay(900); }
            else if (!haveKsoft)          { beepWarnLong(); smartLCD("NO Ksoft", "Run TEACH PAPER"); delay(900); }
            else                           { beepWarnLong(); smartLCD("NO Khard", "Run TEACH PAPER"); delay(900); }
          } else {
            beepWarnLong(); smartLCD("SAVE FAILED", "Check slot/K"); delay(900);
          }
          measureMode = MM_OFF;
          return;
        }

        markDirty(); beepOk(); smartLCD("MEAS SAVED",""); delay(350);
        measureMode = MM_OFF;
        return;
      }

      // START -> Messung aufnehmen & akkumulieren (beendet NICHT die UI)
      if (startTrigger) {
        double avg = takeAveragedLux(7,110);
        if (!isnan(avg) && avg > 0.0) {
          if (currentMode==MODE_BW) { measBWSum += avg; measBWCount++; }
          else {
            if (measureMode == MM_APPLY_SG_G0) { measSoftSum += avg; measSoftCount++; }
            else                               { measHardSum += avg; measHardCount++; }
          }
          beepOk();
        } else {
          beepWarnLong(); smartLCD("MEASURE FAIL", "No sensor"); delay(400);
        }
      }

      // Anzeige im Mess-UI
      if (currentMode==MODE_BW) {
        double avgBW = (measBWCount>0) ? (measBWSum/(double)measBWCount) : (m.ok?m.lux:NAN);
        String avStr = (isnan(avgBW)?"--":String(avgBW,2));
        L0 = String("BW av=") + avStr + " n" + String(measBWCount);
        L1 = "START=ADD  *=CLR";
        if ((int)L1.length()<16){ while((int)L1.length()<12) L1+=" "; L1+="#=SV"; }
      } else {
        if (measureMode == MM_APPLY_SG_G0) {
          double avgS = (measSoftCount>0) ? (measSoftSum/(double)measSoftCount) : (m.ok?m.lux:NAN);
          String avStr = (isnan(avgS)?"--":String(avgS,2));
          L0 = String("S av=") + avStr + " n" + String(measSoftCount);
          L1 = "D=HARD  *=CLR ";
        } else {
          double avgH = (measHardCount>0) ? (measHardSum/(double)measHardCount) : (m.ok?m.lux:NAN);
          String avStr = (isnan(avgH)?"--":String(avgH,2));
          L0 = String("H av=") + avStr + " n" + String(measHardCount);
          L1 = "D=SOFT  *=CLR ";
        }
        if ((int)L1.length()<16){ while((int)L1.length()<12) L1+=" "; L1+="#=SV"; }
      }

      const PaperProfile &PP = CCURP();
      bool flashArmed = PP.flashCalibrated && PP.flashEnable && (PP.flashThreshS > 0.0);
      if (flashArmed) {
        if ((int)L1.length()<16) { while((int)L1.length()<15) L1+=" "; L1+="F"; }
        else L1.setCharAt(15,'F');
      }
      smartLCD(L0,L1);
      return;
    }

    // Normale Anzeige
    if (burnMode != BURN_OFF) {
      double base=(burnMode==BURN_SG_G0)?time_soft:(burnMode==BURN_SG_G5?time_hard:time_bw);
      double burnSeconds = base*(pow(2.0,burnEv)-1.0);

      L0 = (burnMode==BURN_BW)?String("BURN BW G")+String(burnGrade,1):String("BURN ")+String((burnMode==BURN_SG_G0?"G0":"G5"));

      String evStr = "+"+String(burnEv,2)+"EV";
      String tStr = fmtTime(burnSeconds, burnSeconds<10.0);
      L1 = evStr+" "+tStr;
      int maxContent = 14 - (int)stepStr.length(); if (maxContent<0) maxContent=0;
      if ((int)L1.length()>maxContent) L1=L1.substring(0,maxContent);
      while ((int)L1.length()<maxContent) L1+=" ";
      L1 += stepStr;
    } else {
      if (currentMode == MODE_SG) {
        String tS_str=(time_soft>=9.95)?String((int)(time_soft+0.5))+"s":String(time_soft,1)+"s";
        String tH_str=(time_hard>=9.95)?String((int)(time_hard+0.5))+"s":String(time_hard,1)+"s";
        String sLbl=(splitState==SPLIT_IDLE)?">S:":" S:"; String hLbl=(splitState==SPLIT_SOFT_DONE)?">H:":" H:";
        String leftPart=sLbl+tS_str; String rightPart=hLbl+tH_str;
        int spaces=14-(int)leftPart.length()-(int)rightPart.length(); if (spaces<1) spaces=1; String spacer=""; for(int i=0;i<spaces;i++) spacer+=" ";
        L0 = leftPart + spacer + rightPart;
      } else {
        String tB_str=(time_bw>=9.95)?String((int)(time_bw+0.5))+"s":String(time_bw,1)+"s";
        L0 = String("BW G:")+String(grade_bw,1)+" T:"+tB_str;
      }

      if (millis() < infoEndTime) {
        L1 = overlayText;
      } else {
        String leftSide=""; String dSign=(trackDensityEV>=0)?"+":"";
        if (currentMode==MODE_SG) {
          String gSign=(trackGradeSteps>=0)?"+":""; leftSide=String("D")+dSign+String(trackDensityEV,1)+String(" G")+gSign+String(trackGradeSteps,1);
        } else {
          leftSide=String("D")+dSign+String(trackDensityEV,1);
        }
        while ((int)leftSide.length()<13) leftSide+=" ";
        L1 = leftSide + stepStr;
      }

      const PaperProfile &PP = CCURP();
      bool flashArmed = PP.flashCalibrated && PP.flashEnable && (PP.flashThreshS > 0.0);
      if (flashArmed) {
        if ((int)L1.length()<16) { while((int)L1.length()<15) L1+=" "; L1+="F"; }
        else L1.setCharAt(15, 'F');
      }
    }

    smartLCD(L0, L1);
  }
}
