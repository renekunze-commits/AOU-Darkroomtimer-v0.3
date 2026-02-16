/* Logic_Timer.cpp - HIGH PERFORMANCE VERSION
   Optimierungen:
   1. NeoPixel Update nur noch 1x bei Start (nicht mehr im Loop).
   2. Mutex-Locking im Time-Mode entfernt (via Shadow-Variable).
   3. Behält volle Closed-Loop-Funktionalität und Input-Logik bei.
   4. IRAM_ATTR für zeitkritische Funktionen (v0.3.10):
      - getEncDelta(): Im IRAM, da hochfrequent aufgerufen (jeder Loop-Durchlauf).
        Flash-Cache-Miss würde 40-80µs Jitter verursachen.
      - handleExposureMetronome(): Im IRAM, da während laufender Belichtung
        aufgerufen. Deterministische Ausführung verhindert hörbare Takt-Schwankungen.
   5. Closed-Loop Task Stack auf 6144 Bytes erhöht (Headroom für I2C-Treiber).
   6. I2C Bus 1 läuft jetzt mit 400 kHz → Sensor-Abfrage ~4x schneller.
*/

#include <Arduino.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Encoder.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "Globals.h"
#include "Types.h"
#include "Logic_Papers.h"

// =============================================================================
// EXTERNS
// =============================================================================
extern void validateTimes();
extern void markDirty();
extern void resetTracking();
extern void triggerInfo();
extern int gradeIndex(double g);
extern void updateGradeMath();
extern void applyGradeShift(int direction);
extern void PaintLED(int r, int g, int b);
extern void updateNextionUI(bool force);
extern void beepNav();
extern void beepValue();
extern void beepOk();
extern void beepWarnLong();
extern void beepStartPattern();
extern void beepEndPattern();
extern void beepDone();
extern void maybeDoPreflashBeforeExposure(); 

extern ESP32Encoder encSoft;
extern ESP32Encoder encHard;
extern ESP32Encoder encGrade;
extern BtnState sEnter;

extern double paperSpeed[11];
extern BurnMode burnMode;
extern double burnEv;

// =============================================================================
// ENGINE STATE
// =============================================================================
enum TimerMode { TIMER_IDLE, TIMER_TIME_RUNNING, TIMER_DOSE_RUNNING };
static TimerMode timerMode = TIMER_IDLE;

// Shadow Variable für Time-Mode (Performance: Kein Mutex nötig)
static double timeModeTargetS = 0.0;

// Closed Loop Shared State (Protected by Mutex)
enum ClosedLoopResult { CL_RUNNING = 0, CL_DONE = 1, CL_TIMEOUT = 2 };
static bool closedLoopResultPending = false;
static ClosedLoopResult closedLoopResult = CL_RUNNING;

static double closedLoopTarget = 0.0; 
static double closedLoopAccum = 0.0;
static unsigned long closedLoopStartMs = 0;
static unsigned long closedLoopLastMeasureMs = 0;
static uint8_t closedLoopR = 0, closedLoopG = 0, closedLoopB = 0;
static uint8_t closedLoopMaxBright = 255;
static unsigned long closedLoopTimeoutMs = 0;
static double closedLoopTempComp = 1.0;

static const TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(50);

// =============================================================================
// INPUT STATE
// =============================================================================
static long lastPosSoft = 0;
static long lastPosHard = 0;
static long lastPosGrade = 0;

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// ---------------------------------------------------------------------------
// IRAM_ATTR: Funktion liegt im internen SRAM statt im Flash.
// Vorteil: Garantierte Ausführungszeit ohne Flash-Cache-Miss-Verzögerung.
// Wichtig für Encoder-Auswertung, da jeder Loop-Durchlauf diese aufruft
// und ein Cache-Miss 40-80µs Jitter verursachen würde.
// ---------------------------------------------------------------------------
IRAM_ATTR long getEncDelta(ESP32Encoder &enc, long &lastPos) {
  long newPos = enc.getCount(); 
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
}

// Performance: Nur aufrufen wenn sich wirklich was ändert!
void setExposureColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!neoPixelOK) return;
  if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pixels.fill(pixels.Color(r, g, b));
    pixels.show(); // ~8ms Blockade bei 256 LEDs (RMT-basiert, nicht CPU-kritisch)
    xSemaphoreGive(gPixelMutex);
  }
}

// ---------------------------------------------------------------------------
// IRAM_ATTR: Metronom-Tick während Belichtung.
// Liegt im internen SRAM, damit der Sekundentakt während der Belichtung
// nicht durch einen Flash-Cache-Miss verzögert wird. Ein verspäteter Tick
// wäre als ungleichmäßiger Rhythmus hörbar.
// ---------------------------------------------------------------------------
IRAM_ATTR void handleExposureMetronome(unsigned long elapsedMs) {
  static unsigned long lastTick = 0;
  unsigned long currentSec = elapsedMs / 1000;
  if (currentSec != lastTick) {
    tone(PIN_BUZZER, 400, 20); 
    lastTick = currentSec;
  }
}

// =============================================================================
// 1. INPUT LOGIC
// =============================================================================
// (Unverändert, dient der UI-Steuerung im Leerlauf)

void handleIdleInput(char k) {
  unsigned long now = millis();
  bool recalc = false;
  bool uiUpdate = false;

  // --- TASTEN ---
  if (k == '#') { 
      if (globalStepMode == STEP_SIXTH) globalStepMode = STEP_THIRD;
      else if (globalStepMode == STEP_THIRD) globalStepMode = STEP_FULL;
      else globalStepMode = STEP_SIXTH;
      globalSet.stepMode = (uint8_t)globalStepMode;
      markDirty(); beepValue(); uiUpdate = true;
  }
  if (k == '*') { 
      if (currentMode == MODE_BW) time_bw = globalSet.std_time;
      else { time_soft = globalSet.std_time; time_hard = globalSet.std_time; }
      resetTracking(); beepOk(); uiUpdate = true;
  }
  if (k == 'A') { currentMode = MODE_BW; burnMode = BURN_OFF; beepNav(); resetTracking(); uiUpdate = true; }
  if (k == 'B') { currentMode = MODE_SG; splitState = SPLIT_IDLE; burnMode = BURN_OFF; beepNav(); resetTracking(); uiUpdate = true; }
  if (k == 'C') { 
     if (currentMode == MODE_SG) {
        if (burnMode == BURN_OFF) burnMode = BURN_SG_G0;
        else if (burnMode == BURN_SG_G0) burnMode = BURN_SG_G5;
        else burnMode = BURN_OFF;
     } else {
        if (burnMode == BURN_OFF) { burnMode = BURN_BW; burnGrade = grade_bw; }
        else burnMode = BURN_OFF;
     }
     burnEv = 0.0; beepValue(); uiUpdate = true;
  }

  // --- ENCODER ---
  long dSoft  = getEncDelta(encSoft,  lastPosSoft);
  long dHard  = getEncDelta(encHard,  lastPosHard);
  long dGrade = getEncDelta(encGrade, lastPosGrade);

  double evS = (globalStepMode == STEP_SIXTH) ? (1.0/6.0) : ((globalStepMode == STEP_FULL) ? 1.0 : 1.0/3.0);
  double fUp = pow(2.0, evS);
  double fDown = pow(2.0, -evS);

  if (dSoft != 0) {
    if (currentMode == MODE_SG) {
       if (dSoft > 0) time_soft *= fUp; else time_soft *= fDown;
    } else {
       if (dSoft > 0) { time_bw *= fUp; trackDensityEV += evS; }
       else           { time_bw *= fDown; trackDensityEV -= evS; }
    }
    recalc = true; uiUpdate = true; beepNav();
  }

  if (dHard != 0 && currentMode == MODE_SG) {
     if (dHard > 0) time_hard *= fUp; else time_hard *= fDown;
     recalc = true; uiUpdate = true; beepNav();
  }

  if (dGrade != 0) {
    if (burnMode != BURN_OFF) {
       if (dGrade > 0) burnEv += evS; else burnEv -= evS;
       if (burnEv < 0) burnEv = 0;
       beepValue();
    } else if (currentMode == MODE_SG) {
       applyGradeShift(dGrade > 0 ? 1 : -1);
       recalc = true;
    } else {
       double oldG = grade_bw;
       grade_bw += (dGrade * 0.1); 
       grade_bw = constrain(grade_bw, 0.0, 5.0);
       if (abs(grade_bw - oldG) > 0.01) {
          int oldI = gradeIndex(oldG);
          int newI = gradeIndex(grade_bw);
          if (oldI != newI) time_bw *= (paperSpeed[newI] / paperSpeed[oldI]);
       }
       updateGradeMath();
    }
    uiUpdate = true; beepNav();
  }

  if (recalc) { validateTimes(); markDirty(); resetTracking(); }
  if (uiUpdate) triggerInfo();
}

// =============================================================================
// 2. CLOSED-LOOP TASK (Core 1)
// =============================================================================
// Läuft als FreeRTOS-Task auf Core 1 (Main-Loop läuft auf Core 0).
// Misst kontinuierlich Lux-Werte über TSL2561 (I2C Bus 1, 400 kHz) und
// akkumuliert die Licht-Dosis (Lux × Zeit). Wenn die Ziel-Dosis erreicht
// ist, signalisiert er dem Main-Thread über closedLoopResultPending.
//
// KOMMUNIKATION ZWISCHEN DEN KERNEN:
//   - gTimerMutex schützt den Shared State (closedLoopAccum, etc.)
//   - Der Task kopiert zuerst alle Werte lokal ("Shadow Copy"), gibt
//     den Mutex frei, verarbeitet, und schreibt nur das Ergebnis zurück.
//   - Dadurch wird der Mutex nur ~2-5µs gehalten statt ~500µs.
//   - gPixelMutex schützt den NeoPixel-Buffer gegen gleichzeitigen
//     Zugriff von handleLights() auf Core 0.
//
// OPTIMIERUNGEN v0.3.10:
//   - Task-Stack von 4096 auf 6144 Bytes erhöht (I2C-Treiber + Adafruit
//     Library brauchen ~2 KB Stack-Headroom bei 400 kHz)
//   - Sensor-Fehler-Toleranz: 20 aufeinanderfolgende Fehler (~200ms)
//     werden toleriert, um I2C-Glitches durch 230V-Relaisschaltung
//     abzufangen.
// =============================================================================

static void vClosedLoopTask(void *pvParameters) {
  (void)pvParameters;
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t lastWake = xTaskGetTickCount();
  static int consecutiveSensorErrors = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    if (gTimerMutex == NULL) continue;

    TimerMode modeCopy = TIMER_IDLE;
    double targetCopy = 0.0, accumCopy = 0.0, tempCompCopy = 1.0;
    unsigned long startCopy = 0, lastMeasureCopy = 0, timeoutCopy = 0;
    uint8_t rCopy = 0, gCopy = 0, bCopy = 0, maxBrightCopy = 255;

    if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
      modeCopy = timerMode; startCopy = starttime;
      targetCopy = closedLoopTarget; accumCopy = closedLoopAccum;
      lastMeasureCopy = closedLoopLastMeasureMs; tempCompCopy = closedLoopTempComp;
      rCopy = closedLoopR; gCopy = closedLoopG; bCopy = closedLoopB;
      maxBrightCopy = closedLoopMaxBright; timeoutCopy = closedLoopTimeoutMs;
      xSemaphoreGive(gTimerMutex);
    } else continue;

    if (modeCopy != TIMER_DOSE_RUNNING || startCopy == 0) {
      consecutiveSensorErrors = 0; continue;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - startCopy;
    sensors_event_t clEvent; clEvent.light = 0.0;
    bool gotEvent = false;
    
    if (tslLiveOK && digitalRead(PIN_TSL2561_INT) == HIGH) {
        gotEvent = tslLive.getEvent(&clEvent);
        if (!gotEvent) consecutiveSensorErrors++; else consecutiveSensorErrors = 0;
    }

    if (gotEvent && clEvent.light > 0.0) {
      double dt = (now - lastMeasureCopy) / 1000.0;
      accumCopy += (clEvent.light * dt * tempCompCopy);
      lastMeasureCopy = now;
    }

    ClosedLoopResult res = CL_RUNNING;
    if (accumCopy >= targetCopy) res = CL_DONE;
    else if (elapsed > timeoutCopy) res = CL_TIMEOUT;
    // Allow up to 20 consecutive sensor errors (~200ms at 10ms task period) to tolerate
    // transient I2C glitches from 230V relay switching. Only abort if error persists >200ms.
    if (consecutiveSensorErrors > 20) res = CL_TIMEOUT;

    // Soft-Dimming
    double progress = (targetCopy > 0) ? (accumCopy / targetCopy) : 0;
    uint8_t brit = maxBrightCopy;
    if (progress > 0.9) brit = map(progress * 100, 90, 100, maxBrightCopy, maxBrightCopy/4);
    if (brit < 10) brit = 10;

    bool stateChanged = false;
    if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
      closedLoopAccum = accumCopy;
      closedLoopLastMeasureMs = lastMeasureCopy;
      if (res != CL_RUNNING) {
        timerMode = TIMER_IDLE; starttime = 0;
        closedLoopResultPending = true; closedLoopResult = res;
        stateChanged = true;
      }
      xSemaphoreGive(gTimerMutex);
    }

    if (!stateChanged && res == CL_RUNNING) {
       if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
         pixels.setBrightness(brit);
         pixels.fill(pixels.Color(rCopy, gCopy, bCopy));
         pixels.show();
         xSemaphoreGive(gPixelMutex);
       }
       handleExposureMetronome(elapsed);
    } else {
       if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
         pixels.clear(); pixels.show();
         xSemaphoreGive(gPixelMutex);
       }
    }
  }
}

void initClosedLoopTask() {
  // ---------------------------------------------------------------------------
  // Task auf Core 1 pinnen (Core 0 = Main-Loop + WiFi/ESP-NOW).
  // Stack: 6144 Bytes (vorher 4096). Der I2C-Treiber auf Bus 1 (400 kHz)
  // benötigt mehr Stack wegen größerer interner Puffer bei höherer Taktrate.
  // Priorität 3: Höher als Main-Loop (1) und WiFi (0), aber unter
  // Hardware-Interrupts. So wird die Belichtungsregelung nie von der UI
  // oder WiFi-Paketen verzögert.
  // ---------------------------------------------------------------------------
  xTaskCreatePinnedToCore(vClosedLoopTask, "vClosedLoopTask", 6144, NULL, 3, NULL, 1);
}

// =============================================================================
// 3. CONTROL LOGIC & HANDLER (Optimized)
// =============================================================================

void stopTimer() {
  // Egal welcher Mode: Reset — ATOMIC UPDATE: starttime + timerMode zusammen unter Mutex
  if (gTimerMutex != NULL) {
      if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
          starttime = 0;
          timerMode = TIMER_IDLE;
          xSemaphoreGive(gTimerMutex);
      } else {
          // Force: Wenn Timeout, setze trotzdem (verhindert Deadlock)
          starttime = 0;
          timerMode = TIMER_IDLE;
      }
  } else {
      starttime = 0;
      timerMode = TIMER_IDLE;
  }

  if (neoPixelOK) { pixels.clear(); pixels.show(); }
  triggerInfo();
}

void handleTimer() {
  // Optimization: Early Exit wenn Timer aus
  if (starttime == 0) return;

  // === A) FAST PATH: TIME MODE ===
  // Kein Mutex nötig, da timeModeTargetS nur vom Main-Thread geschrieben wird (in startTimer)
  // und Background Task diesen Mode ignoriert.
  if (timerMode == TIMER_TIME_RUNNING) {
     unsigned long elapsed = millis() - starttime;
     
     // Check Time
     if (elapsed >= (timeModeTargetS * 1000.0)) {
        stopTimer(); beepDone();
        // State Advance
        if (currentMode == MODE_SG) {
           if (splitState == SPLIT_DOING_SOFT) splitState = SPLIT_SOFT_DONE;
           else if (splitState == SPLIT_DOING_HARD) splitState = SPLIT_IDLE;
        }
        return;
     }

     // Metronome ONLY (kein pixels.show() mehr!)
     handleExposureMetronome(elapsed);
     return; 
  }

  // === B) SLOW PATH: DOSE MODE ===
  // Hier brauchen wir Mutex für Kommunikation mit Task
  bool pending = false;
  ClosedLoopResult pendingRes = CL_RUNNING;

  if (gTimerMutex && xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
    if (closedLoopResultPending) {
       pending = true; pendingRes = closedLoopResult;
       closedLoopResultPending = false;
    }
    xSemaphoreGive(gTimerMutex);
  }

  if (pending) {
     if (pendingRes == CL_DONE) beepDone(); else beepWarnLong();
     if (currentMode == MODE_SG) {
        if (splitState == SPLIT_DOING_SOFT) splitState = SPLIT_SOFT_DONE;
        else if (splitState == SPLIT_DOING_HARD) splitState = SPLIT_IDLE;
     }
     triggerInfo();
  }
}

// =============================================================================
// 4. START LOGIC (Optimized)
// =============================================================================

void startTimer() {
  maybeDoPreflashBeforeExposure();
  digitalWrite(PIN_RELAY_ROOMLIGHT, LOW);

  double baseTime = 0.0;
  double baseFlux = 1000.0;
  uint8_t r=0, g=0, b=0;
  
  PaperProfile& paper = getActivePaper();
  SplitState nextSplit = splitState;

  // 1. Determine Colors & Times
 if (currentMode == MODE_BW) {
     baseTime = time_bw; baseFlux = paper.Kbw;
     
     // KORREKTUR: Nutze die berechneten Werte aus updateGradeMath()
     r = 0;              // Rot ist bei Multigrade meist aus (oder Safe-Light)
     g = pwmValGreen;    // Berechneter Grün-Anteil
     b = pwmValBlue;     // Berechneter Blau-Anteil
     
    } else { 
     if (splitState == SPLIT_IDLE || splitState == SPLIT_HARD_DONE) {
        baseTime = time_soft; baseFlux = paper.Ksoft;
        nextSplit = SPLIT_DOING_SOFT; r=0; g=255; b=0;
     } else {
        baseTime = time_hard; baseFlux = paper.Khard;
        nextSplit = SPLIT_DOING_HARD; r=0; g=0; b=255;
     }
  }

  // 2. Apply Burn
  if (burnMode != BURN_OFF) {
     if (burnMode == BURN_SG_G0) { baseTime = time_soft; baseFlux = paper.Ksoft; r=0; g=255; b=0; }
     else if (burnMode == BURN_SG_G5) { baseTime = time_hard; baseFlux = paper.Khard; r=0; g=0; b=255; }
     else { baseTime = time_bw; baseFlux = paper.Kbw; r=255; g=180; b=100; }

     baseTime = baseTime * (pow(2.0, burnEv) - 1.0);
     if (baseTime < 0.1) baseTime = 0.1;
     nextSplit = splitState; 
  }

  bool useDose = globalSet.useDoseMode && tslLiveOK;
  
  // 3. Dispatch
  if (useDose) {
     double targetDose = baseTime * baseFlux;
     unsigned long timeout = (unsigned long)(baseTime * 2000) + 5000;
     double tempComp = (!isnan(tempAlu) && tempAlu > 0) ? (1.0 + ((tempAlu - 25.0)*0.005)) : 1.0;

     if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
        starttime = millis();
        timerMode = TIMER_DOSE_RUNNING;
        closedLoopResult = CL_RUNNING; closedLoopResultPending = false;
        closedLoopTarget = targetDose; 
        closedLoopAccum = 0;
        closedLoopStartMs = starttime; closedLoopLastMeasureMs = starttime;
        closedLoopR = r; closedLoopG = g; closedLoopB = b;
        closedLoopMaxBright = set_max; closedLoopTimeoutMs = timeout;
        closedLoopTempComp = tempComp;
        xSemaphoreGive(gTimerMutex);
     }
  } else {
     // TIME MODE OPTIMIZATION
     // 1. Variable setzen (Kein Mutex nötig für diese Shadow Variable)
     timeModeTargetS = baseTime;
     
     // 2. Mode setzen (via Mutex der Sauberkeit halber, falls Task gerade liest)
     if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
        starttime = millis();
        timerMode = TIMER_TIME_RUNNING;
        xSemaphoreGive(gTimerMutex);
     }

     // 3. LED SETZEN - GENAU EINMAL HIER!
     // Da wir im Time-Mode nicht dimmen, reicht einmaliges Setzen.
     setExposureColor(r, g, b);
  }
  
  splitState = nextSplit;
  beepStartPattern();
  triggerInfo();
}

// =============================================================================
// 5. MAIN LOOP WRAPPER
// =============================================================================

void runStandardTimerLoop(char key) {
  // A. Running
  if (starttime != 0) {
     if (digitalRead(PIN_START) == LOW) {
        delay(50);
        if (digitalRead(PIN_START) == LOW) {
           stopTimer(); beepWarnLong();
           while(digitalRead(PIN_START) == LOW) { wdt_reset(); delay(10); }
           return;
        }
     }
     handleTimer(); 
     return;
  }

  // B. Idle Start
  if (digitalRead(PIN_START) == LOW) {
     delay(50);
     if (digitalRead(PIN_START) == LOW) {
        startTimer();
        while(digitalRead(PIN_START) == LOW) { wdt_reset(); delay(10); }
        return;
     }
  }

  // C. Input
  handleIdleInput(key);
}