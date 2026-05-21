/*
    Mode_Calibration.cpp - v0.5 Root Cause Edition
    
    Zweck: Geführter Wizard zur Kalibrierung von Papierprofilen.
    
    Änderungen v0.5:
    - Nutzt die asynchrone Measurement-Engine (Logic_Measurement).
    - Thread-sichere PSRAM-Updates via gTimerMutex.
    - Vollständige HAL-Integration (Licht & Sound).
    - ROOT CAUSE FIX: Konsumiert strikt die xInputQueue (Kein ISR/Polling-Konflikt mehr!)
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Papers.h"
#include "Logic_Math.h"

// --- Externe Helfer ---
extern void PaintLED(int r, int g, int b);
extern void markDirty();
extern void saveSettings();
extern void beepOk();
extern void beepWarnLong();
extern void beepWizardStep();
extern void beepWizardSave();
extern void updateNextionUI(bool force);

// --- Status Variablen ---
bool calAbort = false;
unsigned long calMsgUntil = 0;
static bool calMeasBusy = false;  
static int sW0 = 1;
static int sB0 = 21;
static int sW5 = 1;
static int sB5 = 21;

static int clampStoufferStep(int v) {
    if (v < 1) return 1;
    if (v > 21) return 21;
    return v;
}

PaperProfile& CAL_CURP() { return getActivePaper(); }

void startCalibrationWizard() {
    calState = CAL_START;
    calAbort = false;
    calMeasBusy = false;
    sW0 = 1;
    sB0 = 21;
    sW5 = 1;
    sB5 = 21;
    calMsgUntil = millis() + 1000;
    smartLCD("STOUFFER WIZ", "P" + String(getActivePaperIndex() + 1));
    PaintLED(0, 0, 0);
    updateNextionUI(true);
}

void runCalibrationWizard() {
    wdt_reset();

    // =========================================================================
    // 1. ZENTRALE WAHRNEHMUNG (Queue Auslesen & Leeren)
    // =========================================================================
    InputEvent evt;
    bool startPressed = false;
    bool backPressed = false;
    bool enterPressed = false;
    int encDelta = 0;

    // ROOT CAUSE FIX: Die Queue MUSS zwingend geleert werden, sonst Overflow!
    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_START_PRESSED) startPressed = true;
        if (evt.type == EVT_GRADE_PRESSED) backPressed = true;
        if (evt.type == EVT_ENTER_PRESSED) enterPressed = true;
        if (evt.type == EVT_ENC_SOFT || evt.type == EVT_ENC_HARD || evt.type == EVT_ENC_GRADE) {
            encDelta += (int)evt.value;
        }
    }

    if (backPressed) calAbort = true;

    // =========================================================================
    // 3. ABORT HANDLER & UI PAUSE
    // =========================================================================
    if (calAbort) {
        calState = CAL_IDLE;
        calAbort = false;
        calMeasBusy = false;
        isMeasuring = false;
        PaintLED(0, 0, 0);
        smartLCD("CALIB ABORTED", "");
        beepWarnLong();
        calMsgUntil = millis() + 1000;
        updateNextionUI(true);
        return;
    }

    if (calState == CAL_IDLE || millis() < calMsgUntil) return;

    // =========================================================================
    // 4. WIZARD STATE MACHINE
    // =========================================================================
    switch (calState) {
        case CAL_START:
            if (encDelta != 0) {
                sW0 = clampStoufferStep(sW0 + encDelta);
                uiTriggerBeep(SND_VALUE);
                updateNextionUI(true);
            }
            smartLCD("sW0 Lichter G0", "1-21: " + String(sW0));
            PaintLED(0, 255, 0);
            if (startPressed || enterPressed) {
                uiTriggerBeep(SND_CAL_STEP);
                calState = CAL_G5;
                calMsgUntil = millis() + 500;
                updateNextionUI(true);
            }
            break;

        case CAL_G5:
            if (encDelta != 0) {
                sB0 = clampStoufferStep(sB0 + encDelta);
                uiTriggerBeep(SND_VALUE);
                updateNextionUI(true);
            }
            smartLCD("sB0 SchattenG0", "1-21: " + String(sB0));
            PaintLED(0, 255, 0);
            if (startPressed || enterPressed) {
                uiTriggerBeep(SND_CAL_STEP);
                calState = CAL_G0;
                calMsgUntil = millis() + 500;
                updateNextionUI(true);
            }
            break;

        case CAL_G0:
            if (encDelta != 0) {
                sW5 = clampStoufferStep(sW5 + encDelta);
                uiTriggerBeep(SND_VALUE);
                updateNextionUI(true);
            }
            smartLCD("sW5 Lichter G5", "1-21: " + String(sW5));
            PaintLED(0, 0, 255);
            if (startPressed || enterPressed) {
                uiTriggerBeep(SND_CAL_STEP);
                calState = CAL_G25;
                calMsgUntil = millis() + 500;
                updateNextionUI(true);
            }
            break;

        case CAL_G25:
            if (encDelta != 0) {
                sB5 = clampStoufferStep(sB5 + encDelta);
                uiTriggerBeep(SND_VALUE);
                updateNextionUI(true);
            }
            smartLCD("sB5 SchattenG5", "1-21: " + String(sB5));
            PaintLED(0, 0, 255);
            if (startPressed || enterPressed) {
                uiTriggerBeep(SND_CAL_STEP);
                calState = CAL_REVIEW;
                calMsgUntil = millis() + 500;
                updateNextionUI(true);
            }
            break;

        case CAL_REVIEW:
            smartLCD("SAVE Stouffer", "START/ENT=SAVE");
            if (startPressed || enterPressed) {
                calculateAndSavePaperProfile(sW0, sB0, sW5, sB5);
                updateGradeMath();
                CAL_CURP().calibrated = true;
                markDirty();
                saveSettings();
                beepWizardSave();
                smartLCD("STOUFFER OK", "Press START");
                PaintLED(0, 0, 0);
                calMsgUntil = millis() + 1000;
                calState = CAL_DONE;
                updateNextionUI(true);
            }
            break;

        case CAL_DONE:
            if (startPressed) {
                calState = CAL_IDLE;
                PaintLED(0, 0, 0);
                smartLCD("CALIB EXIT", "");
                calMsgUntil = millis() + 500;
                updateNextionUI(true);
            }
            break;

        default:
            calState = CAL_IDLE;
            updateNextionUI(true);
            break;
    }
}