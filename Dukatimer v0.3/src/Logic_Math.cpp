#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "Globals.h"
#include "Types.h"

// =============================================================================
// GLOBALE AUSGABE-VARIABLEN (werden vom Timer genutzt)
// =============================================================================
uint8_t pwmValGreen = 0;
uint8_t pwmValBlue  = 0;

// =============================================================================
// HILFSFUNKTIONEN - Mathematische Hilfen für Zeiten/Grade
// =============================================================================

// clampDouble:
// - Stellt sicher, dass ein double-Wert innerhalb von [minV, maxV] liegt.
// - Wird für Zeiten und Grade verwendet (z. B. TIME_MIN_S/TIME_MAX_S).
// - Rückgabewert: der begrenzte Wert.
double clampDouble(double val, double minV, double maxV) {
  if (val < minV) return minV;
  if (val > maxV) return maxV;
  return val;
}

// calcFStopOffset:
// - Wandelt eine Basiszeit um, basierend auf einer EV-Änderung (Blendenstufen).
// - Beispiel: base=10s, ev=+1 → 20s (2^1)
// - Liefert 0 für nicht-positive Basiszeiten.
double calcFStopOffset(double baseTime, double ev) {
  if (baseTime <= 0.0) return 0.0;
  // Formel: Neu = Alt * 2^(EV)
  return baseTime * pow(2.0, ev);
} 

// =============================================================================
// HAUPT-FUNKTIONEN (Validierung / Tracking / Mathe)
// =============================================================================

// validateTimes:
// - Sichert, dass konfigurierte Zeiten im erlaubten Bereich liegen.
// - Benutzt Konstanten aus `Config.h` (TIME_MIN_S / TIME_MAX_S).
// - Wird typischerweise nach Einstellungen/Changes aufgerufen.
void validateTimes() {
  // Zeitgrenzen aus Config.h beachten
  time_bw   = clampDouble(time_bw,   TIME_MIN_S, TIME_MAX_S);
  time_soft = clampDouble(time_soft, TIME_MIN_S, TIME_MAX_S);
  time_hard = clampDouble(time_hard, TIME_MIN_S, TIME_MAX_S);

  // Gradation 0.0 bis 5.0
  grade_bw = clampDouble(grade_bw, 0.0, 5.0);
}

// resetTracking:
// - Setzt alle temporären Tracking-Variablen zurück (wird beim Laden oder Reset genutzt)
// - Wichtig, um Mess- und UI-Zustände konsistent zu halten.
void resetTracking() {
  trackDensityEV = 0.0;
  trackGradeSteps = 0.0;
  overlayText = "";
  infoEndTime = 0;
  pendingPaperSlot = -1;

  // Messungen zurücksetzen
  measSoftSum = 0.0; measSoftCount = 0;
  measHardSum = 0.0; measHardCount = 0;
  measBWSum   = 0.0; measBWCount   = 0;

  // Teststrip
  ts = TS_OFF; tsK = 0; tsSum = 0.0; tsActiveExposure = false;
}

// getEffectiveBurnTime:
// - Berechnet die zusätzliche Nachbelichtungszeit basierend auf burnEv (EV-Shifts)
// - Rückgabewert in Sekunden (double). Wenn burnEv <= 0 ergibt sich 0.
// Wandelt die Gradation (0.0 - 5.0) in LED-Helligkeiten um.
// Wird aufgerufen, wenn man am Encoder dreht.
void updateGradeMath() {
  
  // --- A) SPLITGRADE MODUS ---
  // Im SG Modus sind die Farben fix (entweder voll Grün oder voll Blau).
  // Wir berechnen hier nichts für PWM, das macht der Timer selbst.
  // Aber wir könnten hier "Smart Math" machen, wenn du Zeit aus Gradation berechnen willst.
  // (Aktuell lassen wir SG Zeiten so wie sie sind).

  // --- B) STANDARD / BW MODUS (Mischlicht) ---
  // Hier müssen wir Grün und Blau mischen, um den Kontrast zu erzeugen.
  
  // Algorithmus für Ilford Multigrade (Simulation):
  // Grade 0.0: Max Grün, Min Blau
  // Grade 2.5: Viel Grün, Viel Blau (Schnittpunkt)
  // Grade 5.0: Min Grün, Max Blau
  
  double g = grade_bw;
  int rawG = 0;
  int rawB = 0;

  // Wir nutzen eine lineare Rampe mit Crossover bei 2.5
  if (g <= 2.5) {
      // 0.0 bis 2.5:
      // Grün bleibt voll an (Low Contrast Basis)
      rawG = 255;
      // Blau steigt an von 0 bis ca. 80-100%
      rawB = map((long)(g * 100), 0, 250, 0, 220); 
  } 
  else {
      // 2.5 bis 5.0:
      // Blau ist voll an (High Contrast Basis)
      rawB = 255;
      // Grün sinkt von voll auf 0
      rawG = map((long)(g * 100), 250, 500, 220, 0);
  }

  // Sicherheits-Check (Wertebereich 0-255)
  if (rawG < 0) rawG = 0; if (rawG > 255) rawG = 255;
  if (rawB < 0) rawB = 0; if (rawB > 255) rawB = 255;

  // Speichern in globale Variablen (für Logic_Timer.cpp)
  pwmValGreen = (uint8_t)rawG;
  pwmValBlue  = (uint8_t)rawB;
}

// 3. NACHBELICHTUNG (BURN) BERECHNEN
// Wird aufgerufen, wenn Burn-Mode aktiv ist und am Encoder gedreht wird (EV Shift)
// return: Die berechnete Zeit für den Timer (in Sekunden)
double getEffectiveBurnTime() {
  double base = 0.0;

  // Basiszeit wählen
  if (currentMode == MODE_BW) {
      base = time_bw;
  } 
  else if (currentMode == MODE_SG) {
      // Im SG Modus brennen wir meist mit der Farbe, die gerade aktiv ist
      // oder wir nehmen eine Summe? 
      // Einfachheitshalber: Wir nehmen die zuletzt gewählte Zeit als Basis.
      // (Verbesserungspotenzial für später)
      base = (time_soft > time_hard) ? time_soft : time_hard; 
  }

  if (burnEv <= 0.0) return 0.0;
  // calcFStopOffset(base, ev) gibt base * 2^ev zurück.
  // Zusätzliche Burn-Zeit = base * (2^ev - 1)
  return base * (pow(2.0, burnEv) - 1.0);
} 

// =============================================================================
// TESTSTRIP HELPER
// =============================================================================

// =============================================================================
// TESTSTRIP HELPER
// =============================================================================

// Konvertiert Sekunden in "Einheiten" (für Anzeige im Teststreifen-Modus)
// tgl: 0=Sekunden, 1=Blendenstufen (1/10)
long secondsToUnits(double s, int tgl) {
  if (tgl == 0) {
    // Einfach Zeit * 10 (für eine Kommastelle)
    return (long)(s * 10.0);
  } else {
    // Logarithmische Berechnung für f-Stops (komplexer, hier dummy)
    return (long)(s * 10.0); 
  }
}