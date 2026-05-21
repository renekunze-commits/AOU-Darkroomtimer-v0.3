/* Mode_Burn.cpp - Modus 4: Nachbelichten (Burn On-Demand)
   PFLICHTENHEFT ERGÄNZUNG: Dieser Modus fehlte vollständig in der Implementierung.
   
   Architektur (gemäß Pflichtenheft Modus 4):
   Der Burn-Modus ist ein eigenständiger Ausführungs-Status (Overlay).
   Er wird nach der Basis-Belichtung explizit aufgerufen.
   Die 100%-Referenz ist die zuletzt genutzte Basis-Zieldosis.
   
   Encoder-Belegung (Pflichtenheft):
     Encoder 1 (Links):  Burn-Menge (EV)    → Zusatz-Dosis relativ zur Basis
     Enc 1 Taster Kurz:  EV-Step umschalten  → Schrittweite des Encoders
     Encoder 3 (Rechts): Burn-Gradation      → G/B-Verhältnis für diese Nachbelichtung
     Enc 3 Taster Kurz:  Zurück (#)          → Verlässt den Burn-Modus
     Start-Taster:       Burn Ausführen       → Führt genau diese eine Nachbelichtung aus
   
   Interne Logik:
     Die Burn-Zeit ergibt sich aus: t_burn = t_base * (2^burnEv - 1)
     Dies entspricht der Zusatz-Dosis, die über die Basisbelichtung hinausgeht.
     Formel aus Logic_Math.cpp::getEffectiveBurnTime().
*/

#include <Arduino.h>
#include <math.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Timer.h"
#include "Logic_Math.h"
#include "DisplayManager.h"
#include "ExposureEngine.h"

extern void updateNextionUI(bool force);

// =============================================================================
// LOKALER ZUSTAND
// =============================================================================

// Burn-Phasen: SETUP (EV & Gradation einstellen) → ARMED (bereit zum Feuern)
enum BurnPhase { BURN_PHASE_SETUP, BURN_PHASE_ARMED };
static BurnPhase burnPhase = BURN_PHASE_SETUP;
static unsigned long burnMsgUntil = 0;

// Basis-Referenzdosis (wird beim Eintritt gesnappt)
static float burnBaseDose = 0.0f;

// Externe Helfer
extern float baseFlux;

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================

/**
 * Berechnet die effektive Burn-Zeit in Sekunden.
 * Formel: t_burn = t_base * (2^burnEv - 1)
 * Bei burnEv = +1 EV: Burn = Basiszeit (Verdopplung der Gesamtdosis)
 * Bei burnEv = +0.5 EV: Burn ≈ 41% der Basiszeit
 */
static float calcBurnTimeSeconds() {
    return (float)getEffectiveBurnTime();
}

/**
 * Verlässt den Burn-Modus und kehrt zum vorherigen Modus zurück.
 */
static void exitBurnMode() {
    ExposureEngine_Abort();  // Sicher abbrechen (setzt isMeasuring, Licht, Blackout zurück)
    burnMode = BURN_OFF;
    burnPhase = BURN_PHASE_SETUP;
    
    // Zurück zum BW-Modus (oder SG, je nach Kontext)
    // Der Modus bleibt MODE_BURN bis der User per Mode-Dial wechselt
    uiTriggerBeep(SND_BACK);
    uiUpdateLCD("BURN EXIT", "");
    updateNextionUI(true);
}

// =============================================================================
// PUBLIC API
// =============================================================================

/**
 * Eintritt in den Burn-Modus.
 * Wird von handleInput() / EVT_ENTER_PRESSED aufgerufen, wenn currentMode == MODE_BURN.
 */
void startBurnMode() {
    burnPhase = BURN_PHASE_SETUP;
    ExposureEngine_Abort();  // Sicherstellen, dass keine alte Belichtung läuft
    burnMsgUntil = millis() + 500;
    
    // Snap der aktuellen Basis-Dosis als 100%-Referenz (Pflichtenheft)
    burnBaseDose = (float)dose_bw;
    
    // Startwerte für die Burn-Parameter
    if (burnEv <= 0.0f) burnEv = 0.5f;  // Default: +0.5 EV
    // burnGrade behält ihren letzten Wert bei
    
    burnMode = BURN_BW;
    uiUpdateLCD("BURN MODE", "Set EV & Grade");
    uiTriggerBeep(SND_OK);
    updateNextionUI(true);
    
    // Queue leeren, um alte Events zu verwerfen
    InputEvent evt;
    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) { /* flush */ }
}

/**
 * Zyklische Burn-Logik. Wird von handleInput() aufgerufen,
 * wenn burnMode != BURN_OFF.
 * * HINWEIS: Diese Funktion wird NICHT direkt aus dem TaskRealtime gerufen,
 * sondern indirekt über die handleInput() Routing-Logik.
 */
void runBurnLoop() {
    unsigned long now = millis();
    
    // UI-Pause abwarten
    if (now < burnMsgUntil) return;
    
    // --- Input konsumieren ---
    InputEvent evt;
    bool startPressed = false;
    bool gradePressed = false;  // Enc 3 Kurz = Zurück (#)
    int encSoftDelta = 0;       // Encoder 1 = Burn-Menge (EV)
    int encGradeDelta = 0;      // Encoder 3 = Burn-Gradation
    bool backPressed = false;   // Enc 1 Kurz = EV-Step (wie im Hauptmodus)
    
    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_START_PRESSED) startPressed = true;
        if (evt.type == EVT_GRADE_PRESSED) gradePressed = true;
        if (evt.type == EVT_BACK_PRESSED)  backPressed = true;
        if (evt.type == EVT_ENC_SOFT)      encSoftDelta += evt.value;
        if (evt.type == EVT_ENC_GRADE)     encGradeDelta += evt.value;
    }
    
    // --- Globaler Abbruch: Enc 3 Kurz (#) ---
    if (gradePressed) {
        exitBurnMode();
        return;
    }
    
    // --- EV-Step umschalten: Enc 1 Kurz (identisch zum Hauptmodus) ---
    if (backPressed) {
        switch (globalStepMode) {
            case STEP_FULL:  globalStepMode = STEP_HALF;  break;
            case STEP_HALF:  globalStepMode = STEP_THIRD; break;
            case STEP_THIRD: globalStepMode = STEP_SIXTH; break;
            case STEP_SIXTH: globalStepMode = STEP_FULL;  break;
            default:         globalStepMode = STEP_THIRD; break;
        }
        uiTriggerBeep(SND_VALUE);
        updateNextionUI(true);
    }
    
    // --- Aktive Burn-Belichtung: Engine wird zentral aus vTaskRealtime geticked ---
    if (ExposureEngine_IsRunning()) {
        return; // Während Belichtung: Keine Encoder-Verarbeitung
    }
    if (ExposureEngine_IsDone()) {
        ExposureEngine_Abort();
        uiTriggerBeep(SND_END_PATTERN);
        uiUpdateLCD("BURN DONE", "");
        burnMsgUntil = now + 1000;
        updateNextionUI(true);
        return;
    }
    
    // --- EV-Step Berechnung ---
    double evStep;
    switch (globalStepMode) {
        case STEP_FULL:  evStep = 1.0; break;
        case STEP_HALF:  evStep = 0.5; break;
        case STEP_THIRD: evStep = 1.0/3.0; break;
        case STEP_SIXTH: evStep = 1.0/6.0; break;
        default: evStep = 1.0/3.0; break;
    }
    
    // --- Encoder 1 (Links): Burn-Menge in EV verschieben ---
    // Pflichtenheft: "Definiert die Zusatz-Dosis (+1/6, +1/3 EV etc.) relativ zur Basis"
    if (encSoftDelta != 0) {
        burnEv = (float)fmax(0.0, fmin(5.0, (double)burnEv + encSoftDelta * evStep));
        uiTriggerBeep(SND_NAV);
        updateNextionUI(true);
    }
    
    // --- Encoder 3 (Rechts): Burn-Gradation ändern ---
    // Pflichtenheft: "Ändert das G/B-Verhältnis für diese spezifische Nachbelichtung"
    if (encGradeDelta != 0) {
        burnGrade = fmax(0.0, fmin(5.0, burnGrade + encGradeDelta * 0.5));
        uiTriggerBeep(SND_NAV);
        updateNextionUI(true);
    }
    
    // --- LCD Anzeige ---
    char l1[17], l2[17];
    float burnSec = calcBurnTimeSeconds();
    snprintf(l1, 17, "BURN +%0.1f EV", burnEv);
    snprintf(l2, 17, "%0.1fs G:%0.1f", burnSec, burnGrade);
    uiUpdateLCD(l1, l2);
    updateNextionUI(true);
    
    // --- Start-Taster: Burn ausführen ---
    // Pflichtenheft: "Führt genau diese eine Nachbelichtung aus (Metronom tickt)"
    if (startPressed && burnEv > 0.0f) {
        double burnSec = getEffectiveBurnTime();
        if (burnSec < 0.1f) {
            uiUpdateLCD("BURN TOO SHORT", "Increase EV");
            uiTriggerBeep(SND_WARN);
            burnMsgUntil = now + 1000;
            updateNextionUI(true);
            return;
        }
        
        // Lichtfarbe berechnen basierend auf burnGrade
        uint8_t bG = (uint8_t)constrain((1.0 - burnGrade / 5.0) * 255.0, 0, 255);
        uint8_t bB = (uint8_t)constrain((burnGrade / 5.0) * 255.0, 0, 255);
        
        // ExposureEngine übernimmt: Pre-Wait, Licht, esp_timer Shutoff, Metronom, Post-Wait
        ExposureEngine_StartTime((unsigned long)(burnSec * 1000.0f), bG, bB);
        updateNextionUI(true);
    }
}