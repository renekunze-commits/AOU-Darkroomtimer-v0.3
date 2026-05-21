/* Mode_TestStrip.cpp - Strikter 3-Phasen-Teststreifen (Dose-Driven)

   Architektur:
   - Phase 1: TS_SETUP     (Konfiguration)
   - Phase 2: TS_EXECUTE   (Belichtungssequenz)
   - Phase 3: TS_EVALUATE  (Auswahl + Closed Loop)

   Kernregel:
   - Interner Zustand basiert auf Dose (Lux-Sekunden), nicht auf Zeit.
   - Zeiten werden nur zur UI aus Dose/Flux abgeleitet.
*/

#include <Arduino.h>
#include <math.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Timer.h"
#include "ExposureEngine.h"

// Externe Ausgaben/Helfer
extern void beepNav();
extern void beepValue();
extern void beepOk();
extern void beepWarnLong();
extern void smartLCD(const char* l1, const char* l2);
extern void PaintLED(int r, int g, int b);
extern void updateNextionUI(bool force);

// Aktueller Messfluss (Fallback in Dunkelheit)
extern float baseFlux;

// ---------------------------------------------------------------------------
// Lokale State-Machine
// ---------------------------------------------------------------------------
enum TSPhase {
  PHASE_TS_SETUP = 0,
  PHASE_TS_EXECUTE,
  PHASE_TS_EVALUATE
};

static TSPhase tsPhase = PHASE_TS_SETUP;

static uint8_t tsStrips = 5;               // Nur 3, 5, 7
static const float tsEvStepTable[] = { 1.0f/6.0f, 1.0f/4.0f, 1.0f/3.0f, 1.0f/2.0f, 1.0f };
static const uint8_t tsEvStepCount = sizeof(tsEvStepTable) / sizeof(tsEvStepTable[0]);
static uint8_t tsEvStepIdx = 2;            // Default 1/3 EV

static float tsStartDoseBW = 10000.0f;     // Start-Dose aus BW-Basis
static float tsStripDose[7] = {0};         // Maximal 7 Streifen
static float tsStripSec[7] = {0};          // UI-Ableitung

static uint8_t tsExecIndex = 0;            // 0..tsStrips-1

static uint8_t tsEvalIndex = 0;            // 0..tsStrips-1

// Overlay-Rücksprungziel: Hauptmodus (BW/SG), nicht hart BW erzwingen.
static Mode tsReturnMode = MODE_BW;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------
static float getCurrentFluxForUI() {
  float flux = baseFlux;
  if (flux <= 0.001f) flux = 1.0f;
  return flux;
}

static float getStepEVFromGlobal() {
  if (globalStepMode == STEP_SIXTH) return 1.0f/6.0f;
  if (globalStepMode == STEP_HALF)  return 1.0f/2.0f;
  if (globalStepMode == STEP_FULL)  return 1.0f;
  return 1.0f/3.0f;
}

static float getCurrentTsEvStep() {
  return tsEvStepTable[tsEvStepIdx];
}

static void buildExecuteBar(char* out, size_t outSize) {
  if (outSize < 4) return;
  String s = "[";
  for (uint8_t i = 0; i < tsStrips; i++) {
    if (i < tsExecIndex) s += "X";
    else s += "O";
    if (i + 1 < tsStrips) s += " ";
  }
  s += "]";
  snprintf(out, outSize, "%s", s.c_str());
}

static void buildEvaluateCursor(char* out, size_t outSize) {
  if (outSize < 4) return;
  String s = "[";
  for (uint8_t i = 0; i < tsStrips; i++) {
    s += (i == tsEvalIndex) ? "^" : "-";
    if (i + 1 < tsStrips) s += " ";
  }
  s += "]";
  snprintf(out, outSize, "%s", s.c_str());
}

static void recalcStripSeries() {
  float flux = getCurrentFluxForUI();
  float evStep = getCurrentTsEvStep();

  // =================================================================
  // PFLICHTENHEFT FIX: Serien-Zentrierung um die Basis-Dosis.
  // Laut Pflichtenheft: "Die aktuelle Basis-Dosis ist immer exakt 
  // die Mitte der Serie." D.h. bei 5 Streifen a 1/3 EV:
  // Streifen 0 = Basis * 2^(-2 * 1/3) ... Streifen 4 = Basis * 2^(+2 * 1/3)
  // ALT (nur aufwärts, nicht zentralisiert):
  // for (uint8_t i = 0; i < tsStrips; i++) {
  //     float d = tsStartDoseBW * powf(2.0f, (float)i * evStep);
  //     ...
  // }
  // =================================================================
  int halfStrips = (int)tsStrips / 2; // Ganzzahlig: bei 5 = 2, bei 7 = 3
  for (uint8_t i = 0; i < tsStrips; i++) {
    // Offset relativ zur Mitte: i=0 -> -halfStrips, i=half -> 0, i=last -> +halfStrips
    int offset = (int)i - halfStrips;
    float d = tsStartDoseBW * powf(2.0f, (float)offset * evStep);
    if (d < 0.1f) d = 0.1f;
    tsStripDose[i] = d;
    tsStripSec[i] = d / flux;
  }
  updateNextionUI(true);
}

static void enterEvaluatePhase() {
  tsPhase = PHASE_TS_EVALUATE;
  tsEvalIndex = 0;
  ts = TS_RUNNING; // KompatibilitÃ¤t zum bestehenden globalen TS-State
  beepOk();
  updateNextionUI(true);
}

static void exitToBW(bool applied) {
  ExposureEngine_Abort();  // Sicher abbrechen (setzt isMeasuring, Licht, Blackout zurück)
  ts = TS_OFF;

  if (applied) {
    setDoseBW(tsStripDose[tsEvalIndex]);
    refreshDisplayVariables();
    currentMode = tsReturnMode;
    beepOk();
    smartLCD("TS APPLY MAIN", "");
  } else {
    beepWarnLong();
    smartLCD("TS CANCEL", "");
  }
  updateNextionUI(true);
}

void setTestStripReturnMode(Mode mode) {
  if (mode == MODE_BW || mode == MODE_SG) {
    tsReturnMode = mode;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void startTestStripMode() {
  // Fallback für direkte Aufrufe: Wenn aus BW/SG gestartet, dort hin zurück.
  if (currentMode == MODE_BW || currentMode == MODE_SG) {
    tsReturnMode = currentMode;
  }

  tsPhase = PHASE_TS_SETUP;
  ts = TS_SETUP;

  tsStrips = 5;
  tsEvStepIdx = 2; // 1/3 EV

  tsStartDoseBW = getDoseBW();
  if (tsStartDoseBW < 0.1f) tsStartDoseBW = 0.1f;

  tsExecIndex = 0;
  tsEvalIndex = 0;

  recalcStripSeries();

  // Queue leeren
  InputEvent evt;
  while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) { /* flush */ }

  smartLCD("TEST STRIP SETUP", "");
  updateNextionUI(true);
}

void runTestStripLoop(char key) {
  (void)key;
  unsigned long now = millis();

  // =================================================================
  // PFLICHTENHEFT FIX: Korrekte Event-Erkennung für alle 3 Phasen.
  // Laut Pflichtenheft:
  //   Phase 1: Enc 2 Kurz (*) = Bestätigen, Enc 3 Kurz (#) = Abbruch
  //   Phase 2: Start-Taster = Nächster Strip, Enc 3 Kurz (#) = Abbruch
  //   Phase 3: Enc 2 Kurz (*) = Auswahl, Enc 3 Kurz (#) = Verwerfen
  // ALT: evBack (Enc 1 = EVT_BACK_PRESSED) wurde fälschlich als Abbruch
  //      und evEnter (Enc 2) als Start-Taster verwendet.
  // =================================================================
  bool evEnter    = false;  // Enc 2 Kurz = Bestätigen (*)
  bool evGrade    = false;  // Enc 3 Kurz = Abbruch (#)
  bool evStart    = false;  // Start-Taster = Nächster Strip
  // ALT: bool evBack  = false;
  long dEnc1   = 0;
  long dEnc2   = 0;
  long dEnc3   = 0;

  InputEvent evtQueue;
  while (xQueueReceive(xInputQueue, &evtQueue, 0) == pdTRUE) {
      if (evtQueue.type == EVT_ENTER_PRESSED) evEnter = true;
      // PFLICHTENHEFT FIX: Abbruch liegt auf Enc 3 (GRADE), nicht Enc 1 (BACK)
      // ALT: if (evtQueue.type == EVT_BACK_PRESSED)  evBack = true;
      if (evtQueue.type == EVT_GRADE_PRESSED) evGrade = true;
      // PFLICHTENHEFT FIX: Start-Taster für die Belichtungsauslösung
      if (evtQueue.type == EVT_START_PRESSED) evStart = true;
      if (evtQueue.type == EVT_ENC_SOFT)      dEnc1 += evtQueue.value;
      if (evtQueue.type == EVT_ENC_HARD)      dEnc2 += evtQueue.value;
      if (evtQueue.type == EVT_ENC_GRADE)     dEnc3 += evtQueue.value;
  }

  // ------------------------------------------------------------
  // Globaler Abbruch (Enc 3 Kurz = # in allen Phasen)
  // PFLICHTENHEFT FIX: War früher auf evBack (Enc 1), jetzt auf evGrade (Enc 3)
  // ------------------------------------------------------------
  if (evGrade) {
    exitToBW(false);
    return;
  }

  // ------------------------------------------------------------
  // PHASE 1: TS_SETUP
  // ------------------------------------------------------------
  if (tsPhase == PHASE_TS_SETUP) {
    // =================================================================
    // PFLICHTENHEFT FIX: Encoder-Belegung Phase 1 (Setup)
    // Laut Pflichtenheft:
    //   Encoder 1 (Links) = EV-Step wählen (Schrittweite)
    //   Encoder 2 (Mitte) = Anzahl Streifen (3, 5, 7)
    // ALT: Encoder 1 verschob die Start-Dosis (nicht im Pflichtenheft!)
    //      Encoder 3 wählte den EV-Schritt (sollte Enc 1 sein)
    // =================================================================

    // Encoder 1: EV-Schrittweite wählen (Pflichtenheft: "EV-Step wählen")
    // ALT: Encoder 1 war Dosis-Verschiebung:
    // if (dEnc1 != 0) {
    //   float evStep = getStepEVFromGlobal();
    //   tsStartDoseBW *= powf(2.0f, (float)dEnc1 * evStep);
    //   if (tsStartDoseBW < 0.1f) tsStartDoseBW = 0.1f;
    //   recalcStripSeries();
    //   beepNav();
    // }
    if (dEnc1 != 0) {
      int next = (int)tsEvStepIdx + (dEnc1 > 0 ? 1 : -1);
      if (next < 0) next = (int)tsEvStepCount - 1;
      if (next >= (int)tsEvStepCount) next = 0;
      tsEvStepIdx = (uint8_t)next;
      recalcStripSeries();
      beepValue();
    }

    // Encoder 2: Streifenanzahl 3/5/7
    if (dEnc2 != 0) {
      if (dEnc2 > 0) {
        if (tsStrips == 3) tsStrips = 5;
        else if (tsStrips == 5) tsStrips = 7;
        else tsStrips = 3;
      } else {
        if (tsStrips == 7) tsStrips = 5;
        else if (tsStrips == 5) tsStrips = 3;
        else tsStrips = 7;
      }
      recalcStripSeries();
      beepValue();
    }

    // Encoder 3: Im Setup nicht belegt (nur Enc 3 Taster = globaler Abbruch)
    // ALT: Encoder 3 war EV-Step-Auswahl (wurde nach Enc 1 verschoben):
    // if (dEnc3 != 0) {
    //   int next = (int)tsEvStepIdx + (dEnc3 > 0 ? 1 : -1);
    //   if (next < 0) next = (int)tsEvStepCount - 1;
    //   if (next >= (int)tsEvStepCount) next = 0;
    //   tsEvStepIdx = (uint8_t)next;
    //   recalcStripSeries();
    //   beepValue();
    // }
    (void)dEnc3; // Enc 3 Drehen: Im Setup nicht belegt

    char l1[17];
    char l2[17];
    snprintf(l1, sizeof(l1), "S:%u EV:%0.2f", tsStrips, getCurrentTsEvStep());
    snprintf(l2, sizeof(l2), "D:%0.0f #=EXEC", tsStartDoseBW);
    smartLCD(l1, l2);

    // PFLICHTENHEFT FIX: Enc 2 Taster Kurz (*) = Bestätigen → Execute
    // (War vorher korrekt auf evEnter, Kommentar aktualisiert)
    if (evEnter) {
      tsPhase = PHASE_TS_EXECUTE;
      ts = TS_RUNNING;
      tsExecIndex = 0;
      ExposureEngine_Abort();  // Sicherstellen, dass keine alte Belichtung läuft
      beepOk();
      updateNextionUI(true);
    }
    return;
  }

  // ------------------------------------------------------------
  // PHASE 2: TS_EXECUTE
  // ------------------------------------------------------------
  if (tsPhase == PHASE_TS_EXECUTE) {
    // Engine wird zentral aus vTaskRealtime geticked
    if (ExposureEngine_IsRunning()) {
      return; // Während Belichtung: Keine weitere Verarbeitung
    }
    if (ExposureEngine_IsDone()) {
      ExposureEngine_Acknowledge();
      tsExecIndex++;
      updateNextionUI(true);
      if (tsExecIndex >= tsStrips) {
        enterEvaluatePhase();
        return;
      }
    }

    char l1[17];
    char l2[17];
    buildExecuteBar(l2, sizeof(l2));
    if (tsExecIndex < tsStrips) {
      snprintf(l1, sizeof(l1), "STRIP %u/%u %0.1fs", (unsigned)(tsExecIndex + 1), (unsigned)tsStrips, tsStripSec[tsExecIndex]);
    } else {
      snprintf(l1, sizeof(l1), "STRIP DONE");
    }
    smartLCD(l1, l2);

    // =================================================================
    // PFLICHTENHEFT FIX: Start-Taster startet den nächsten Streifen
    // ExposureEngine übernimmt: Pre-Wait, Licht, Metronom, Post-Wait, isMeasuring
    // =================================================================
    if (!ExposureEngine_IsRunning() && evStart && tsExecIndex < tsStrips) {
      unsigned long durationMs = (unsigned long)(tsStripSec[tsExecIndex] * 1000.0f);
      if (durationMs < 100) durationMs = 100;
      ExposureEngine_StartTime(durationMs, pwmValGreen, pwmValBlue);
    }
    return;
  }

  // ------------------------------------------------------------
  // PHASE 3: TS_EVALUATE (Closed Loop)
  // PFLICHTENHEFT: Enc 1 oder 2 = Cursor, Enc 2 Kurz = Auswahl (*),
  //               Enc 3 Kurz = Verwerfen (#)
  // Abbruch via evGrade wird oben bereits global abgefangen.
  // ------------------------------------------------------------
  if (tsPhase == PHASE_TS_EVALUATE) {
    // H05 FIX: Encoder 1 ODER 2 bewegen den Cursor (laut Spec)
    int cursorDelta = dEnc1 + dEnc2;
    if (cursorDelta != 0) {
      int next = (int)tsEvalIndex + (cursorDelta > 0 ? 1 : -1);
      if (next < 0) next = (int)tsStrips - 1;
      if (next >= (int)tsStrips) next = 0;
      tsEvalIndex = (uint8_t)next;
      beepNav();
      updateNextionUI(true);
    }

    char l1[17];
    char l2[17];
    snprintf(l1, sizeof(l1), "Strip %u (%0.1fs)", (unsigned)(tsEvalIndex + 1), tsStripSec[tsEvalIndex]);
    buildEvaluateCursor(l2, sizeof(l2));
    smartLCD(l1, l2);

    // PFLICHTENHEFT FIX: Enc 2 Kurz (*) = Closed Loop anwenden
    // (Korrekt auf evEnter, Kommentar aktualisiert)
    if (evEnter) {
      // =================================================================
      // PFLICHTENHEFT FIX: Zentrierte Serien-Formel
      // Da die Serie jetzt zentriert ist (Streifen tsStrips/2 = Basis),
      // berechnen wir die Dose korrekt mit dem Offset relativ zur Mitte.
      // ALT (nicht zentriert):
      // float finalDose = tsStartDoseBW * powf(2.0f, (float)tsEvalIndex * getCurrentTsEvStep());
      // =================================================================
      int halfStrips = (int)tsStrips / 2;
      int offset = (int)tsEvalIndex - halfStrips;
      float finalDose = tsStartDoseBW * powf(2.0f, (float)offset * getCurrentTsEvStep());
      if (finalDose < 0.1f) finalDose = 0.1f;
      tsStripDose[tsEvalIndex] = finalDose;
      exitToBW(true);
      return;
    }
    return;
  }
}