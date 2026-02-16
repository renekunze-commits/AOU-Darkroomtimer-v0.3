#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Papers.h"

// --- Externe Helfer ---
extern void smartLCD(const String& l1, const String& l2);
extern void PaintLED(int r, int g, int b);
extern void markDirty();
extern void saveSettings();
extern void beepOk();
extern void beepWarnLong();
extern void beepWizardStep();
extern void beepWizardSave();
extern double takeAveragedLux(uint8_t samples, uint16_t delayMs);
extern char getNextionKey();
extern void updateNextionUI(bool force);

// --- State Machine ---
// CalStep ist in Types.h definiert! (CAL_IDLE, CAL_G5, CAL_G0, CAL_G25, CAL_REVIEW, CAL_DONE)
CalStep calState = CAL_IDLE;
bool calAbort = false;
unsigned long calMsgUntil = 0;
static double calLastAvg = NAN;

// --- Shortcut zum aktuellen Papierprofil ---
PaperProfile& CAL_CURP() { return getActivePaper(); }

// --- Wizard Start ---
void startCalibrationWizard() {
    calState = CAL_START;
    calAbort = false;
    calMsgUntil = millis() + 1000;
    smartLCD("CALIBRATE PAPER", "P" + String(getActivePaperIndex() + 1));
    PaintLED(0, 0, 0);
}

// --- Wizard Loop (zyklisch aufrufen) ---
void runCalibrationWizard() {
    wdt_reset();
    char k = getNextionKey();
    bool startPressed = checkButtonPress(btnStart, PIN_START);
    bool backPressed = (k == '*');

    if (backPressed) calAbort = true;
    if (calAbort) {
        calState = CAL_IDLE;
        calAbort = false;
        PaintLED(0, 0, 0);
        smartLCD("CALIB ABORTED", "");
        beepWarnLong();
        calMsgUntil = millis() + 1000;
        return;
    }

    if (calState == CAL_IDLE) return;
    if (millis() < calMsgUntil) return;


    switch (calState) {
        case CAL_START:
            smartLCD("CALIBRATE PAPER", "Press START");
            PaintLED(0, 0, 0);
            if (startPressed) {
                calState = CAL_G5;
                calMsgUntil = millis() + 500;
            }
            break;

        case CAL_G5:
            smartLCD("G5: SHADOW", "START=MEASURE *ABT");
            PaintLED(0, 0, 255);
            if (startPressed) {
                isMeasuring = true;
                handleLights();
                lcd.setRGB(0, 0, 0);
                updateNextionUI(true);
                double avg = takeAveragedLux(7, 110);
                isMeasuring = false;
                handleLights();
                if (!isnan(avg) && time_hard > 0.05) {
                    CAL_CURP().Khard = avg * time_hard;
                    beepWizardStep();
                    smartLCD("Khard: " + String(CAL_CURP().Khard, 1), "OK");
                    calMsgUntil = millis() + 800;
                    calState = CAL_G0;
                } else {
                    smartLCD("G5 FAILED", "A=RETRY *=ABT");
                    beepWarnLong();
                    calMsgUntil = millis() + 1000;
                }
            }
            break;

        case CAL_G0:
            smartLCD("G0: HIGHLIGHT", "START=MEASURE *ABT");
            PaintLED(0, 255, 0);
            if (startPressed) {
                isMeasuring = true;
                handleLights();
                lcd.setRGB(0, 0, 0);
                updateNextionUI(true);
                double avg = takeAveragedLux(7, 110);
                isMeasuring = false;
                handleLights();
                if (!isnan(avg) && time_soft > 0.05) {
                    CAL_CURP().Ksoft = avg * time_soft;
                    beepWizardStep();
                    smartLCD("Ksoft: " + String(CAL_CURP().Ksoft, 1), "OK");
                    calMsgUntil = millis() + 800;
                    calState = CAL_G25;
                } else {
                    smartLCD("G0 FAILED", "A=RETRY *=ABT");
                    beepWarnLong();
                    calMsgUntil = millis() + 1000;
                }
            }
            break;

        case CAL_G25:
            smartLCD("BW: MIDTONE", "START=MEASURE *ABT");
            PaintLED(120, 120, 120); // Weiß/Mix
            if (startPressed) {
                isMeasuring = true;
                handleLights();
                lcd.setRGB(0, 0, 0);
                updateNextionUI(true);
                double avg = takeAveragedLux(7, 110);
                isMeasuring = false;
                handleLights();
                if (!isnan(avg) && time_bw > 0.05) {
                    CAL_CURP().Kbw = avg * time_bw;
                    beepWizardStep();
                    smartLCD("Kbw: " + String(CAL_CURP().Kbw, 1), "OK");
                    calMsgUntil = millis() + 800;
                    calState = CAL_REVIEW;
                } else {
                    smartLCD("BW FAILED", "A=RETRY *=ABT");
                    beepWarnLong();
                    calMsgUntil = millis() + 1000;
                }
            }
            break;

        case CAL_REVIEW:
            smartLCD("Review K-Values", "#=SAVE *=ABT");
            // Zeige Werte rotierend oder auf Tastendruck
            if (k == '#') {
                CAL_CURP().calibrated = true;
                markDirty();
                saveSettings();
                beepWizardSave();
                smartLCD("CALIBRATION OK", "Press START");
                PaintLED(0, 0, 0);
                calMsgUntil = millis() + 1000;
                calState = CAL_DONE;
            }
            break;

        case CAL_DONE:
            if (startPressed) {
                calState = CAL_IDLE;
                PaintLED(0, 0, 0);
                smartLCD("CALIB EXIT", "");
                calMsgUntil = millis() + 500;
            }
            break;

        default:
            calState = CAL_IDLE;
            break;
    }
}
