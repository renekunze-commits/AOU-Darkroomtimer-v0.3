/* Mode_SG.cpp - Modus 2: Splitgrade-Belichtung (Input-Handling)

   Ausgelagert aus Logic_Timer.cpp (Phase 2 - B1 Refactoring).
   
   Encoder-Belegung (Pflichtenheft Modus 2):
     Encoder 1 (Links):  Soft-Zeit/Dosis (EV-Schritte)        → dose_soft / time_soft
     Encoder 2 (Mitte):  Hard-Zeit/Dosis (EV-Schritte)        → dose_hard / time_hard
     Encoder 3 (Rechts): Master Helligkeit (proportionaler Shift beider Kanäle)
    Enc 1 Lang:         Spektrale Mess-Sequenz                → triggerSpectralMeasurement()
    Enc 2 Lang:         Reset Soft & Hard auf default         → globalSet.std_time
    Enc 3 Lang:         Spektrale Mess-Sequenz                → triggerSpectralMeasurement()

   Invariante:
     hwSwitchDoseMode entscheidet, ob Zeit oder Dosis geändert wird.
     Die physikalische Mathematik und Timer-Abbruchbedingungen bleiben unangetastet.
*/

#include <Arduino.h>
#include <math.h>
#include <freertos/semphr.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Timer.h"
#include "Logic_Math.h"
#include "Logic_Measurement.h"
#include "Logic_Papers.h"
#include "Mode_SG.h"
#include "ExposureEngine.h"
#include "DisplayManager.h"
#include "UI_Map.h"

static const TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(20);
static bool sgAutoPending = false;
static double sgAutoTime = 0.0;
static double sgAutoGrade = 2.5;

extern void calculateAutoSG(double &suggestedTime, double &suggestedGrade);

static int getHistogramPointCount() {
    int points = 0;

    if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
        for (int i = 0; i < 11; i++) {
            points += ((int)currentZoneHistogram[i] / 20);
        }
        xSemaphoreGive(gTimerMutex);
    }

    return points;
}

static bool isFixedGradePaperActive() {
    bool fixedGrade = false;

    if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
        fixedGrade = getActivePaper().isFixedGrade;
        xSemaphoreGive(gTimerMutex);
    }

    return fixedGrade;
}

static void applyAutoSuggestion() {
    if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
        double tSoft = 0.0;
        double tHard = 0.0;
        calculateSplitTimes(sgAutoTime, sgAutoGrade, tSoft, tHard);

        grade_bw = (float)sgAutoGrade;

        if (hwSwitchDoseMode) {
            double flux = (baseFlux > 0.001f) ? (double)baseFlux : 1.0;
            dose_soft = fmax(0.1, tSoft * flux);
            dose_hard = fmax(0.1, tHard * flux);
        } else {
            time_soft = (float)fmax((double)TIME_MIN_S, tSoft);
            time_hard = (float)fmax((double)TIME_MIN_S, tHard);
        }
        xSemaphoreGive(gTimerMutex);
    }

    updateGradeMath();
    refreshDisplayVariables();
    String appliedMsg = "T" + String(sgAutoTime, 1) + " G" + String(sgAutoGrade, 1);
    smartLCD("AUTO APPLIED", appliedMsg);
    DM_setText(OBJ_SG_HIST, "AUTO APPLIED");
    uiTriggerBeep(SND_DONE);
    sgAutoPending = false;
}

bool handleSGInput(const InputEvent& evt, double evStep) {
    if (ExposureEngine_IsRunning() && evt.type != EVT_START_PRESSED) {
        return false;
    }

    switch (evt.type) {

        // =================================================================
        // Encoder 1 (Soft): Soft-Kanal Zeit/Dosis in EV-Schritten
        // =================================================================
        case EVT_ENC_SOFT:
            if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                if (hwSwitchDoseMode) dose_soft = fmax(0.1, dose_soft * pow(2.0, evt.value * evStep));
                else time_soft = fmax(TIME_MIN_S, time_soft * pow(2.0, evt.value * evStep));
                xSemaphoreGive(gTimerMutex);
            }
            refreshDisplayVariables();
            uiTriggerBeep(SND_NAV);
            return true;

        // =================================================================
        // Encoder 2 (Hard): Hard-Kanal Zeit/Dosis in EV-Schritten
        // =================================================================
        case EVT_ENC_HARD:
            if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                if (hwSwitchDoseMode) dose_hard = fmax(0.1, dose_hard * pow(2.0, evt.value * evStep));
                else time_hard = fmax(TIME_MIN_S, time_hard * pow(2.0, evt.value * evStep));
                xSemaphoreGive(gTimerMutex);
            }
            refreshDisplayVariables();
            uiTriggerBeep(SND_NAV);
            return true;

        // =================================================================
        // Encoder 3 (Grade): Master Helligkeit (proportionaler Shift)
        // H03 FIX: Verschiebt Dosis_Soft UND Dosis_Hard proportional
        // (Kontrast bleibt gleich)
        // =================================================================
        case EVT_ENC_GRADE: {
            double shiftFactor = pow(2.0, evt.value * evStep);
            if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                if (hwSwitchDoseMode) {
                    dose_soft = fmax(0.1, dose_soft * shiftFactor);
                    dose_hard = fmax(0.1, dose_hard * shiftFactor);
                } else {
                    time_soft = fmax(TIME_MIN_S, time_soft * (float)shiftFactor);
                    time_hard = fmax(TIME_MIN_S, time_hard * (float)shiftFactor);
                }
                xSemaphoreGive(gTimerMutex);
            }
            refreshDisplayVariables();
            uiTriggerBeep(SND_NAV);
            return true;
        }

        // =================================================================
        // Enc 1 Lang: Spektrale Mess-Sequenz starten
        // =================================================================
        case EVT_BACK_LONG:
            if (probeConnected && !isMeasurementActive()) {
                if (triggerSpectralMeasurement()) {
                    uiTriggerBeep(SND_OK);
                    Serial.println("[SG] Spektrale Mess-Sequenz gestartet (Enc 1 Lang)");
                } else {
                    uiTriggerBeep(SND_WARN);
                    Serial.println("[SG] Messung konnte nicht gestartet werden");
                }
            } else {
                uiTriggerBeep(SND_HINT);
                Serial.println("[SG] Probe nicht verbunden - lokale Messung nicht implementiert");
            }
            return true;

        // =================================================================
        // Enc 2 Lang: Reset Soft & Hard auf Standardwerte
        // =================================================================
        case EVT_ENTER_LONG:
            if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                time_soft = globalSet.std_time;
                time_hard = globalSet.std_time;
                xSemaphoreGive(gTimerMutex);
                refreshDisplayVariables();
                uiTriggerBeep(SND_DONE);
            }
            return true;

        // =================================================================
        // Enc 2 Kurz: AUTO Vorschlag anzeigen / übernehmen
        // =================================================================
        case EVT_ENTER_PRESSED:
            if (sgAutoPending) {
                applyAutoSuggestion();
                return true;
            }

            if (getHistogramPointCount() >= 2) {
                calculateAutoSG(sgAutoTime, sgAutoGrade);
                if (sgAutoTime > 0.0) {
                    if (isFixedGradePaperActive()) {
                        sgAutoPending = false;
                        String recomMsg = "RECOM: G " + String(sgAutoGrade, 1);
                        smartLCD("FIXED PAPER", recomMsg.c_str());
                        DM_setText(OBJ_SG_HIST, recomMsg);
                        uiTriggerBeep(SND_AUTO_PROP);
                        return true;
                    }

                    sgAutoPending = true;
                    String autoMsg = "AUTO:" + String(sgAutoTime, 1) + "s G" + String(sgAutoGrade, 1);
                    smartLCD("SG AUTO READY", autoMsg.c_str());
                    DM_setText(OBJ_SG_HIST, autoMsg);
                    uiTriggerBeep(SND_AUTO_PROP);
                    return true;
                }
            }

            uiTriggerBeep(SND_AUTO_PROP);
            return true;

        // =================================================================
        // Enc 3 Lang: Spektrale Mess-Sequenz starten
        // =================================================================
        case EVT_GRADE_LONG:
            if (probeConnected && !isMeasurementActive()) {
                if (triggerSpectralMeasurement()) {
                    uiTriggerBeep(SND_OK);
                    Serial.println("[SG] Spektrale Mess-Sequenz gestartet (Enc 3 Lang)");
                } else {
                    uiTriggerBeep(SND_WARN);
                    Serial.println("[SG] Messung konnte nicht gestartet werden");
                }
            } else {
                uiTriggerBeep(SND_HINT);
                Serial.println("[SG] Probe nicht verbunden - lokale Messung nicht implementiert");
            }
            return true;

        default:
            return false;
    }
}
