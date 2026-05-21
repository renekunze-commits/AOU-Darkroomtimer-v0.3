/* Mode_Preflash.cpp - v0.5 Root Cause Edition
    
    Zweck: Preflash-Kalibrierung und Ausführung.
    
    Architektur-Heilung v0.5:
    - Vollständige Beseitigung blockierender while-Schleifen.
    - Integration in die xInputQueue (Core 1).
    - Thread-sichere PSRAM-Manipulation via gTimerMutex.
    - HAL-konforme Licht- und Soundsteuerung.
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Papers.h"
#include "DisplayManager.h"
#include "Logic_Timer.h"
#include "Logic_Math.h"

extern void updateNextionUI(bool force);

// --- Lokaler Status Wizard ---
enum PreflashWizardState {
    PF_IDLE,
    PF_SET_PARAMS,
    PF_WAIT_START,
    PF_PULSING,
    PF_PICK_THRESH,
    PF_SAVE_EXIT
};

static PreflashWizardState wizState = PF_IDLE;
static int pfSteps = 6;
static double pfDt = 0.05;
static int currentStep = 0;
static unsigned long pfTimer = 0;
static unsigned long pfMsgUntil = 0;

// --- Lokaler Status Exposure-Preflash ---
static bool preflashActive = false;
static int preflashPhase = 0; // 0=idle, 1=on, 2=pause
static unsigned long preflashUntil = 0;

// =============================================================================
// 1. EXPOSURE PREFLASH (Wird vom Timer-Task gerufen)
// =============================================================================

void maybeDoPreflashBeforeExposure() {
    // Grounding: Nur ausführen, wenn konfiguriert
    PaperProfile &PP = getActivePaper();
    unsigned long now = millis();

    if (!(PP.flashEnable && PP.flashThreshS > 0.01)) {
        preflashActive = false;
        preflashPhase = 0;
        if (currentMode == MODE_BW || currentMode == MODE_SG) updateNextionUI(true);
        return;
    }

    if (!preflashActive) {
        // Start: Weißlicht an
        HW_SetEnlargerNeoPixel(255, 255, 255);
        preflashActive = true;
        preflashPhase = 1;
        preflashUntil = now + (unsigned long)(PP.flashThreshS * 1000.0);
        if (currentMode == MODE_BW || currentMode == MODE_SG) updateNextionUI(true);
        return;
    }

    if (preflashPhase == 1 && now >= preflashUntil) {
        // Ende Blitz: Dunkelheit
        HW_SetEnlargerNeoPixel(0, 0, 0);
        preflashPhase = 2;
        preflashUntil = now + 300; // Beruhigungs-Pause für das Papier (Post-Wait)
        if (currentMode == MODE_BW || currentMode == MODE_SG) updateNextionUI(true);
        return;
    }

    if (preflashPhase == 2 && now >= preflashUntil) {
        // Sequenz beendet
        preflashActive = false;
        preflashPhase = 0;
        if (currentMode == MODE_BW || currentMode == MODE_SG) updateNextionUI(true);
    }
}

bool preflashBusy() { return preflashActive; }

// =============================================================================
// 2. FLASH CALIBRATION WIZARD (State Machine)
// =============================================================================

void startPreflashWizard() {
    wizState = PF_SET_PARAMS;
    pfSteps = 6;
    pfDt = 0.05;
    pfMsgUntil = millis() + 500;
    uiUpdateLCD("PREFLASH CALIB", "Set Steps & dt");
    uiTriggerBeep(SND_OK);
    updateNextionUI(true);
}

void runPreflashWizard() {
    wdt_reset();
    unsigned long now = millis();
    if (now < pfMsgUntil) return;

    // Input-Queue konsumieren
    InputEvent evt;
    bool startPressed = false;
    bool backPressed = false;
    bool enterPressed = false;
    int encSoftDelta = 0;
    int encHardDelta = 0;

    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_START_PRESSED) startPressed = true;
        if (evt.type == EVT_BACK_PRESSED) backPressed = true;
        if (evt.type == EVT_ENTER_PRESSED) enterPressed = true;
        if (evt.type == EVT_ENC_SOFT) encSoftDelta += evt.value;
        if (evt.type == EVT_ENC_HARD) encHardDelta += evt.value;
    }

    // Globaler Abbruch
    if (backPressed) {
        wizState = PF_IDLE;
        HW_SetEnlargerNeoPixel(0, 0, 0);
        uiUpdateLCD("PF ABORTED", "");
        uiTriggerBeep(SND_WARN);
        updateNextionUI(true);
        return;
    }

    switch (wizState) {
        case PF_SET_PARAMS: {
            if (encSoftDelta != 0) {
                pfSteps = constrain(pfSteps + encSoftDelta, 3, 10);
                uiTriggerBeep(SND_NAV);
                updateNextionUI(true);
            }
            if (encHardDelta != 0) {
                pfDt = constrain(pfDt + (encHardDelta * 0.01), 0.01, 0.25);
                uiTriggerBeep(SND_NAV);
                updateNextionUI(true);
            }

            char l1[17], l2[17];
            snprintf(l1, 17, "ST:%d dt:%.2f", pfSteps, pfDt);
            snprintf(l2, 17, "ENTER=OK  *=ABT");
            uiUpdateLCD(l1, l2);

            if (enterPressed) {
                currentStep = 0;
                wizState = PF_WAIT_START;
                uiTriggerBeep(SND_OK);
                updateNextionUI(true);
            }
        } break;

        case PF_WAIT_START: {
            char l1[17];
            snprintf(l1, 17, "STEP %d/%d", currentStep, pfSteps);
            uiUpdateLCD(l1, "START=FLASH");
            
            if (startPressed) {
                wizState = PF_PULSING;
                pfTimer = now;
                // Turn light on once at transition to avoid repeated updates
                HW_SetEnlargerNeoPixel(255, 255, 255);
                uiTriggerBeep(SND_CLICK);
                updateNextionUI(true);
            }
        } break;

        case PF_PULSING: {
            // Ensure step 0 produces a non-zero flash duration
            double tFlash = (currentStep + 1) * pfDt;
            
            if (now - pfTimer >= (unsigned long)(tFlash * 1000.0)) {
                HW_SetEnlargerNeoPixel(0, 0, 0);
                currentStep++;
                if (currentStep > pfSteps) {
                    wizState = PF_PICK_THRESH;
                } else {
                    wizState = PF_WAIT_START;
                }
                pfMsgUntil = now + 100;
                updateNextionUI(true);
            }
        } break;

        case PF_PICK_THRESH: {
            uiUpdateLCD("Develop Strip", "SOFT-ENC=PICK");
            if (encSoftDelta != 0) {
                currentStep = constrain(currentStep + encSoftDelta, 1, pfSteps);
                uiTriggerBeep(SND_NAV);
                updateNextionUI(true);
            }
            
            char l2[17];
            snprintf(l2, 17, "Pick:%d  #=SAVE", currentStep);
            uiUpdateLCD("Develop Strip", l2);

            if (enterPressed) {
                if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    PaperProfile &PP = getActivePaper();
                    PP.flashThreshS = (currentStep - 1) * pfDt;
                    PP.flashEnable = true;
                    PP.flashCalibrated = true;
                    xSemaphoreGive(gTimerMutex);
                    
                    saveSettings();
                    uiTriggerBeep(SND_DONE);
                    wizState = PF_SAVE_EXIT;
                    updateNextionUI(true);
                }
            }
        } break;

        case PF_SAVE_EXIT:
            uiUpdateLCD("THRESH SAVED", "Press START");
            if (startPressed) {
                wizState = PF_IDLE;
                updateNextionUI(true);
            }
            break;

        default: break;
    }
}