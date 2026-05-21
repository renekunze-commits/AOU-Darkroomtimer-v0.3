/* Logic_Timer.cpp - v0.5 Root Cause Edition (Phase 2 B2 Final)
   
   ARCHITEKTUR:
   1. Belichtungspräzision: Vollständig an ExposureEngine delegiert (esp_timer + CL-Task).
   2. startTimer/stopTimer: Dünne Wrapper für BW/SG-spezifische Logik (Preflash, Split-State).
   3. handleTimer: Erkennt normale Engine-Beendigung und finalisiert Split-State.
   4. Core-Isolation: Läuft exklusiv auf Core 1 (TaskRealtime).
   5. UI-Grounding: Master Mode-Dial (Encoder 4) schaltet linear durch Modi.
*/

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "Globals.h"
#include "Types.h"
#include "Logic_Papers.h"
#include "Logic_Timer.h"
#include "Logic_Math.h"
#include "DisplayManager.h"
#include "Logic_Measurement.h"
#include "Mode_BW.h"
#include "Mode_SG.h"
#include "ExposureEngine.h"

// NEXTION INTEGRATION
extern void updateNextionUI(bool force);

static const TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(100);
static constexpr uint32_t START_EVENT_GUARD_MS = 150;
static bool sDoseModeLatched = false;
static bool sDoseModeLatchedValid = false;

static bool timerMutexOwnedByCurrentTask() {
    if (gTimerMutex == NULL) return false;
    return xSemaphoreGetMutexHolder(gTimerMutex) == xTaskGetCurrentTaskHandle();
}

static void reportTimerMutexFailure(const char* context, bool notifyUi = false) {
    saveErrorState(ERR_MUTEX_TIMEOUT);
    Serial.printf("[TIMER] Mutex-Timeout in %s. Operation sicher abgebrochen.\n", context);
    if (notifyUi) {
        smartLCD("LOCK TIMEOUT", context);
    }
}

static bool lockTimerMutexIfNeeded(TickType_t timeout, bool &lockedHere, const char* context, bool notifyUi = false) {
    lockedHere = false;
    if (gTimerMutex == NULL) {
        reportTimerMutexFailure(context, notifyUi);
        return false;
    }
    if (timerMutexOwnedByCurrentTask()) return true;

    lockedHere = (xSemaphoreTake(gTimerMutex, timeout) == pdTRUE);
    if (!lockedHere) {
        reportTimerMutexFailure(context, notifyUi);
    }
    return lockedHere;
}

static void unlockTimerMutexIfNeeded(bool lockedHere) {
    if (lockedHere && gTimerMutex != NULL) {
        xSemaphoreGive(gTimerMutex);
    }
}

static void clearExposureState(bool resetSplitState) {
    bool lockedHere = false;
    if (lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "clearExposureState", false)) {
        starttime = 0;
        if (resetSplitState && currentMode == MODE_SG) {
            splitState = SPLIT_IDLE;
        }
        unlockTimerMutexIfNeeded(lockedHere);
    }
    lightOperationActive = false;
    updateNextionUI(true);
}

static void clearDoseModeLatch() {
    sDoseModeLatched = false;
    sDoseModeLatchedValid = false;
}

static void restoreIdleLightState() {
    HW_SetBlackout(false);
    if (safeLatch) HW_SetSafelight(true);
    clearExposureState(true);
    updateNextionUI(true);
}

extern void runSetupTick();
extern void runCalibrationWizard();
extern void runDensitometerWizard();
extern void runTestStripLoop(char key);
extern void runBurnLoop();

void handleInput() {
    if (setupMenuActive) { runSetupTick(); return; }
    if (calState != CAL_IDLE) { runCalibrationWizard(); return; }
    if (densState != DENS_IDLE) { runDensitometerWizard(); return; }
    if (ts != TS_OFF) { runTestStripLoop(' '); return; }
    // PFLICHTENHEFT ERGÄNZUNG: Burn-Modus als eigenständiges Overlay
    if (burnMode != BURN_OFF) { runBurnLoop(); return; }

    InputEvent evt;
    static uint32_t lastAcceptedStartEventMs = 0;
    static uint32_t lastAcceptedStartLongEventMs = 0;
    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_START_PRESSED) {
            uint32_t dt = evt.timestamp - lastAcceptedStartEventMs;
            if (lastAcceptedStartEventMs != 0 && dt < START_EVENT_GUARD_MS) {
                continue;
            }
            lastAcceptedStartEventMs = evt.timestamp;
        }

        if (evt.type == EVT_START_LONG) {
            uint32_t dt = evt.timestamp - lastAcceptedStartLongEventMs;
            if (lastAcceptedStartLongEventMs != 0 && dt < START_EVENT_GUARD_MS) {
                continue;
            }
            lastAcceptedStartLongEventMs = evt.timestamp;

            if (ExposureEngine_IsPaused()) {
                ExposureEngine_Abort();
                clearExposureState(true);
                clearDoseModeLatch();
                triggerInfo();
                uiTriggerBeep(SND_BACK);
                updateNextionUI(true);
            } else {
                uiTriggerBeep(SND_LIMIT);
            }
            continue;
        }

        if (evt.type == EVT_START_PRESSED) {
            if (ExposureEngine_IsPaused()) {
                if (ExposureEngine_Resume()) {
                    uiTriggerBeep(SND_OK);
                    updateNextionUI(true);
                }
                continue;
            }

            if (ExposureEngine_GetState() == EXP_EXPOSING) {
                if (ExposureEngine_Pause()) {
                    uiTriggerBeep(SND_HINT);
                    updateNextionUI(true);
                }
                continue;
            }
        }

        if (ExposureEngine_IsRunning()) {
            if (evt.type == EVT_START_PRESSED) {
                stopTimer();
            } else {
                uiTriggerBeep(SND_LIMIT);
            }
            continue;
        }

        if (starttime != 0) {
            if (evt.type == EVT_START_PRESSED) stopTimer();
            else uiTriggerBeep(SND_LIMIT);
            continue;
        }

        double evStep;
        switch (globalStepMode) {
            case STEP_FULL:  evStep = 1.0; break;
            case STEP_HALF:  evStep = 0.5; break;
            case STEP_THIRD: evStep = 1.0/3.0; break;
            case STEP_SIXTH: evStep = 1.0/6.0; break;
            default: evStep = 1.0/3.0; break;
        }

        switch (evt.type) {
            case EVT_START_PRESSED:
                startTimer();
                break;

            case EVT_DENS_REQUEST:
                extern void startDensitometerMode();
                startDensitometerMode();
                updateNextionUI(true);
                break;

            // =================================================================
            // PFLICHTENHEFT FIX: Enc 2 Kurz (ENTER) = Reiner Bestätigungstaster (*)
            // VORHER: Der Step-Toggle (1/1→1/2→1/3→1/6) lag fälschlich hier.
            // Laut Pflichtenheft Modus 1 & 2: "Enc 2 Taster (Kurz) = ENTER / Bestätigen"
            // Der Step-Toggle gehört auf Enc 1 Kurz (EVT_BACK_PRESSED), siehe unten.
            // Modus-Eintritt (Setup, TestStrip, etc.) bleibt korrekt auf ENTER.
            // =================================================================
            case EVT_ENTER_PRESSED:
                // BETA-FIX: Schritt 4 - Missing Link der zentralen Event-Delegation.
                // ENTER muss im BW-Modus zwingend an handleBWInput() delegiert werden,
                // damit die in Schritt 3 implementierte Apply-Logik erreichbar ist.
                // Ohne diese Delegation wuerde ENTER im BW-Pfad nur akustisches Feedback
                // geben, aber niemals dose_bw aus target_dose uebernehmen.
                if (currentMode == MODE_BW) {
                    handleBWInput(evt, evStep);
                } else if (currentMode == MODE_SG) {
                    handleSGInput(evt, evStep);
                } else if (currentMode == MODE_SETUP) {
                    extern void startSetupMenu();
                    startSetupMenu();
                } else if (currentMode == MODE_TESTSTRIP) {
                    extern void startTestStripMode();
                    startTestStripMode();
                } else if (currentMode == MODE_CALIB) {
                    extern void startCalibrationWizard();
                    startCalibrationWizard();
                } else if (currentMode == MODE_DENS) {
                    extern void startDensitometerMode();
                    startDensitometerMode();
                } else if (currentMode == MODE_BURN) {
                    // PFLICHTENHEFT: Burn-Modus betreten
                    extern void startBurnMode();
                    startBurnMode();
                } else {
                    // PFLICHTENHEFT: Im BW/SG Idle hat ENTER keine Sonderfunktion.
                    // Der Step-Toggle wurde nach EVT_BACK_PRESSED verschoben.
                    // ALT (auskommentiert, da Spec-Verletzung):
                    // switch (globalStepMode) {
                    //     case STEP_FULL:  globalStepMode = STEP_HALF;  break;
                    //     case STEP_HALF:  globalStepMode = STEP_THIRD; break;
                    //     case STEP_THIRD: globalStepMode = STEP_SIXTH; break;
                    //     case STEP_SIXTH: globalStepMode = STEP_FULL;  break;
                    //     default:         globalStepMode = STEP_THIRD; break;
                    // }
                    // uiTriggerBeep(SND_VALUE);
                    uiTriggerBeep(SND_OK);
                }
                updateNextionUI(true);
                break;

            // =================================================================
            // PFLICHTENHEFT FIX: Enc 1 Kurz (BACK) = EV-Step umschalten
            // VORHER: Hier lag ein Zeit-Reset (time_bw = std_time), der lt.
            // Pflichtenheft auf Enc 3 Lang (EVT_GRADE_LONG) gehört – und dort
            // auch korrekt implementiert ist. Der Reset hier war also doppelt.
            // Neu: "Enc 1 Taster (Kurz) = EV-Step umschalten" (1/1→1/2→1/3→1/6)
            // =================================================================
            case EVT_BACK_PRESSED:
                // ALT (auskommentiert – Duplikat von EVT_GRADE_LONG):
                // if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                //     if (currentMode == MODE_SG) {
                //         time_soft = globalSet.std_time;
                //         time_hard = globalSet.std_time;
                //     } else {
                //         time_bw = globalSet.std_time;
                //     }
                //     xSemaphoreGive(gTimerMutex);
                //     refreshDisplayVariables();
                //     uiTriggerBeep(SND_BACK);
                // }
                // NEU: EV-Step zyklisch weiterschalten (Pflichtenheft Modus 1 & 2)
                switch (globalStepMode) {
                    case STEP_FULL:  globalStepMode = STEP_HALF;  break;
                    case STEP_HALF:  globalStepMode = STEP_THIRD; break;
                    case STEP_THIRD: globalStepMode = STEP_SIXTH; break;
                    case STEP_SIXTH: globalStepMode = STEP_FULL;  break;
                    default:         globalStepMode = STEP_THIRD; break;
                }
                uiTriggerBeep(SND_VALUE);
                updateNextionUI(true);
                break;

            // =============================================================
            // PHASE 2 B1: Encoder-Events an Mode_BW / Mode_SG delegieren
            // Die gesamte BW/SG-Logik für EVT_ENC_SOFT, EVT_ENC_HARD und
            // EVT_ENC_GRADE wurde in Mode_BW.cpp bzw. Mode_SG.cpp ausgelagert.
            // =============================================================
            case EVT_ENC_SOFT:
            case EVT_ENC_HARD:
            case EVT_ENC_GRADE:
                if (currentMode == MODE_SG) {
                    handleSGInput(evt, evStep);
                } else if (currentMode == MODE_BW) {
                    handleBWInput(evt, evStep);
                }
                updateNextionUI(true);
                break;

            // K04 FIX: Encoder 3 Kurz = BACK / Abbruch (#)
            case EVT_GRADE_PRESSED:
                // FIX K2: Enc3 kurz = Abbruch (#) — bricht aktive Messung ab
                if (isMeasurementActive()) {
                    abortMeasurementWithError("User Cancel");
                }
                uiTriggerBeep(SND_BACK);
                updateNextionUI(true);
                break;

            // PHASE 2 B1: EVT_GRADE_LONG an Mode_BW / Mode_SG delegieren
            case EVT_GRADE_LONG:
                if (currentMode == MODE_SG) {
                    handleSGInput(evt, evStep);
                } else if (currentMode == MODE_BW) {
                    handleBWInput(evt, evStep);
                }
                updateNextionUI(true);
                break;

            // =============================================================
            // PHASE 2 B1: Long-Press Events an Mode_BW / Mode_SG delegieren
            // =============================================================
            case EVT_BACK_LONG:
            case EVT_ENTER_LONG:
                if (currentMode == MODE_SG) {
                    handleSGInput(evt, evStep);
                } else if (currentMode == MODE_BW) {
                    handleBWInput(evt, evStep);
                } else {
                    uiTriggerBeep(SND_HINT);
                }
                updateNextionUI(true);
                break;

            // ROOT CAUSE FIX: Master Mode Dial Logik (4. Encoder)
            // PFLICHTENHEFT ERGÄNZUNG: Burn (Modus 4) und Kalibrierung (Modus 5)
            // zum Mode-Dial hinzugefügt. Vorher fehlten diese komplett.
            case EVT_ENC_MODE: {
                // FIX: Mode-Dial disabled during pause state (safety)
                if (ExposureEngine_IsPaused()) {
                    uiTriggerBeep(SND_LIMIT);
                    break;
                }

                const Mode previousMode = currentMode;

                static int linearMenuState = 0;
                // Begrenze Sprünge, falls Encoder prellt
                int move = (evt.value > 0) ? 1 : (evt.value < 0 ? -1 : 0);
                const int previousMenuState = linearMenuState;
                int requestedMenuState = linearMenuState + move;
                
                // PFLICHTENHEFT FIX: Erweiterung von 7 auf 9 Modi
                // (Burn + Kalibrierung eingefügt zwischen SG-Dosis und Densitometer)
                if (requestedMenuState < 0) requestedMenuState = 0;
                if (requestedMenuState > 8) requestedMenuState = 8;

                bool fixedGradeActive = false;
                bool lockedHere = false;
                if (lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "mode_select", true)) {
                    fixedGradeActive = getActivePaper().isFixedGrade;
                    unlockTimerMutexIfNeeded(lockedHere);
                } else {
                    break;
                }

                if (fixedGradeActive && (requestedMenuState == 2 || requestedMenuState == 3)) {
                    linearMenuState = previousMenuState;
                    smartLCD("SG BLOCKED", "");
                    uiTriggerBeep(SND_LIMIT);
                    break;
                }

                linearMenuState = requestedMenuState;

                switch (linearMenuState) {
                    case 0: currentMode = MODE_BW; hwSwitchDoseMode = false; break; // Zeit BW
                    case 1: currentMode = MODE_BW; hwSwitchDoseMode = true; break;  // Dosis BW
                    case 2: currentMode = MODE_SG; hwSwitchDoseMode = false; break; // Zeit Split
                    case 3: currentMode = MODE_SG; hwSwitchDoseMode = true; break;  // Dosis Split
                    case 4: currentMode = MODE_BURN; hwSwitchDoseMode = false; break;      // Nachbelichten
                    case 5: currentMode = MODE_CALIB; hwSwitchDoseMode = false; break;     // Kalibrierung
                    case 6: currentMode = MODE_DENS; hwSwitchDoseMode = false; break;      // Densitometer
                    case 7: currentMode = MODE_TESTSTRIP; hwSwitchDoseMode = false; break;  // Teststreifen
                    case 8: currentMode = MODE_SETUP; hwSwitchDoseMode = false; break;      // Setup
                }

                if (currentMode == MODE_TESTSTRIP) {
                    extern void setTestStripReturnMode(Mode mode);
                    setTestStripReturnMode((previousMode == MODE_SG) ? MODE_SG : MODE_BW);
                }
                
                uiTriggerBeep(SND_NAV);
                refreshDisplayVariables(); 
                updateNextionUI(true);
                break;
            }

            /* Fallback: Falls physischer Schalter doch noch verbaut ist
            case EVT_DOSE_SWITCH_CHANGE:
                hwSwitchDoseMode = (evt.value == LOW); 
                refreshDisplayVariables();
                uiTriggerBeep(SND_OK);
                break; */


            default:
                break;
        }
        triggerInfo(); 
    }
}

void modifyExposureByEV(double evDelta) {
    bool lockedHere = false;
    if (lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "modifyExposureByEV")) {
        bool hitLimit = false;
        if (hwSwitchDoseMode) {
            if (currentMode == MODE_BW) {
                double nextDose = dose_bw * pow(2.0, evDelta);
                if (nextDose < 0.1) {
                    dose_bw = 0.1;
                    hitLimit = true;
                } else if (nextDose > 999.0) {
                    dose_bw = 999.0;
                    hitLimit = true;
                } else {
                    dose_bw = nextDose;
                }
            } else {
                double nextSoft = dose_soft * pow(2.0, evDelta);
                double nextHard = dose_hard * pow(2.0, evDelta);

                if (nextSoft < 0.1) {
                    dose_soft = 0.1;
                    hitLimit = true;
                } else if (nextSoft > 999.0) {
                    dose_soft = 999.0;
                    hitLimit = true;
                } else {
                    dose_soft = nextSoft;
                }

                if (nextHard < 0.1) {
                    dose_hard = 0.1;
                    hitLimit = true;
                } else if (nextHard > 999.0) {
                    dose_hard = 999.0;
                    hitLimit = true;
                } else {
                    dose_hard = nextHard;
                }
            }
        } else {
            double nextTime = time_bw * pow(2.0, evDelta);
            if (nextTime < TIME_MIN_S) {
                time_bw = TIME_MIN_S;
                hitLimit = true;
            } else if (nextTime > TIME_MAX_S) {
                time_bw = TIME_MAX_S;
                hitLimit = true;
            } else {
                time_bw = nextTime;
            }
        }
        unlockTimerMutexIfNeeded(lockedHere);
        if (hitLimit) uiTriggerBeep(SND_LIMIT);
        refreshDisplayVariables(); 
        updateNextionUI(true);
    }
}

void refreshDisplayVariables() {
    float flux = (baseFlux > 0.001f) ? baseFlux : 1.0f;
    
    bool lockedHere = false;
    if (lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "refreshDisplayVariables")) {
        if (hwSwitchDoseMode) {
            time_bw = (float)(dose_bw / flux);
            time_soft = (float)(dose_soft / flux);
            time_hard = (float)(dose_hard / flux);
        } else {
            dose_bw = (double)(time_bw * flux);
        }
        unlockTimerMutexIfNeeded(lockedHere);
    }
    updateNextionUI(true);
}

// =============================================================================
// initClosedLoopTask: Delegiert an ExposureEngine_Init()
// Kompatibilitäts-Wrapper — wird aus setup() (main.cpp) aufgerufen.
// =============================================================================
void initClosedLoopTask() {
    ExposureEngine_Init();
}

// =============================================================================
// startTimer: Wrapper für BW/SG Haupt-Belichtung via ExposureEngine
// Handhabt Preflash + modusspezifische Parameter, delegiert an Engine.
// =============================================================================
void startTimer() {
    if (starttime != 0) return;
    if (ExposureEngine_IsRunning()) return;

    if (overheatLock) {
        saveErrorState(ERR_THERMAL_OVERHEAT);
        smartLCD("OVERHEAT", "COOL DOWN");
        uiTriggerBeep(SND_ALARM);
        return;
    }

    if (currentMode == MODE_SG) {
        if (splitState == SPLIT_IDLE) {
            const bool useDoseMode = hwSwitchDoseMode;
            double sgSoft = 0.0;
            double sgHard = 0.0;
            if (useDoseMode) {
                calculateSplitTimes((double)time_bw, (double)grade_bw, sgSoft, sgHard);
            }

            bool lockedHere = false;
            if (!lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "startTimer.sg_soft", true)) {
                return;
            }

                if (useDoseMode) {
                    time_soft = (float)fmax((double)TIME_MIN_S, sgSoft);
                    time_hard = (float)fmax((double)TIME_MIN_S, sgHard);
                } else {
                    // Prio 2 Fix: Im Zeitmodus manuelle SG-Zeiten (Enc1/Enc2) nicht überschreiben.
                    time_soft = (float)fmax((double)TIME_MIN_S, (double)time_soft);
                    time_hard = (float)fmax((double)TIME_MIN_S, (double)time_hard);
                }
                current_dose = 0.0;
                starttime = millis();
                splitState = SPLIT_DOING_SOFT;
                sDoseModeLatched = hwSwitchDoseMode;
                sDoseModeLatchedValid = true;
                const bool useDoseModeLatched = sDoseModeLatchedValid ? sDoseModeLatched : hwSwitchDoseMode;
                const double softDose = dose_soft;
                const double softTimeS = (double)time_soft;
                unlockTimerMutexIfNeeded(lockedHere);

            lightOperationActive = true;
            if (useDoseModeLatched) {
                ExposureEngine_StartDose(softDose, 255, 0);
            } else {
                ExposureEngine_StartTime((unsigned long)(softTimeS * 1000.0), 255, 0);
            }
            if (!ExposureEngine_IsRunning()) {
                restoreIdleLightState();
                clearDoseModeLatch();
                triggerInfo();
            }
            updateNextionUI(true);
            return;
        }

        if (splitState == SPLIT_SOFT_DONE) {
            bool lockedHere = false;
            if (!lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "startTimer.sg_hard", true)) {
                return;
            }

                current_dose = 0.0;
                starttime = millis();
                splitState = SPLIT_DOING_HARD;
                const bool useDoseMode = sDoseModeLatchedValid ? sDoseModeLatched : hwSwitchDoseMode;
                const double hardDose = dose_hard;
                const double hardTimeS = (double)time_hard;
                unlockTimerMutexIfNeeded(lockedHere);

            lightOperationActive = true;
            if (useDoseMode) {
                ExposureEngine_StartDose(hardDose, 0, 255);
            } else {
                ExposureEngine_StartTime((unsigned long)(hardTimeS * 1000.0), 0, 255);
            }
            if (!ExposureEngine_IsRunning()) {
                restoreIdleLightState();
                clearDoseModeLatch();
                triggerInfo();
            }
            updateNextionUI(true);
            return;
        }

        return;
    }

    // Phase 1: Blackout + Pre-Wait (blockierend, 200ms)
    HW_SetBlackout(true);
    HW_SetSafelight(false);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Phase 2: Preflash (blockierend, nur BW)
    extern void maybeDoPreflashBeforeExposure();
    extern bool preflashBusy();
    maybeDoPreflashBeforeExposure();
    while (preflashBusy()) {
        maybeDoPreflashBeforeExposure();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Phase 3: Kanalfarbe bestimmen
    uint8_t green, blue;
    green = pwmValGreen;
    blue  = pwmValBlue;

    // Phase 4: UI-Timestamp setzen + Engine starten
    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "startTimer.main", true)) {
        restoreIdleLightState();
        return;
    }

    current_dose = 0.0;
    starttime = millis();
    sDoseModeLatched = hwSwitchDoseMode;
    sDoseModeLatchedValid = true;

    if (currentMode == MODE_BW) target_dose = dose_bw;

    const bool useDoseMode = sDoseModeLatchedValid ? sDoseModeLatched : hwSwitchDoseMode;
    const double td = target_dose;
    const double targetS = (double)time_bw;
    unlockTimerMutexIfNeeded(lockedHere);

    if (useDoseMode) {
        ExposureEngine_StartDose(td, green, blue, true);
    } else {
        unsigned long durationMs = (unsigned long)(targetS * 1000.0);
        ExposureEngine_StartTime(durationMs, green, blue, true);
    }

    if (!ExposureEngine_IsRunning()) {
        restoreIdleLightState();
        clearDoseModeLatch();
    }
    updateNextionUI(true);
}

// =============================================================================
// stopTimer: User-Abort via Graceful Stop (mit Post-Wait in Engine)
// =============================================================================
void stopTimer() {
    if (starttime == 0) return;

    // Graceful Stop: Licht aus → Engine geht in POST_WAIT → dann IDLE
    ExposureEngine_Stop();

    // Sofort Split-State und starttime finalisieren
    clearExposureState(true);
    clearDoseModeLatch();
    triggerInfo();
    updateNextionUI(true);
}

// =============================================================================
// handleTimer: Erkennt normale Belichtungs-Beendigung (Engine DONE)
// =============================================================================
void handleTimer() {
    // Normale Beendigung: Engine hat POST_WAIT durchlaufen → DONE
    if (starttime != 0 && ExposureEngine_IsDone()) {
        if (currentMode == MODE_SG) {
            bool lockedHere = false;
            if (lockTimerMutexIfNeeded(MUTEX_TIMEOUT, lockedHere, "handleTimer.sg_done", true)) {
                starttime = 0;
                if (splitState == SPLIT_DOING_SOFT) {
                    splitState = SPLIT_SOFT_DONE;
                    lightOperationActive = false;
                    unlockTimerMutexIfNeeded(lockedHere);
                    ExposureEngine_Abort();
                    if (safeLatch) HW_SetSafelight(true);
                    uiTriggerBeep(SND_PHASE_READY);
                    triggerInfo();
                    updateNextionUI(true);
                    return;
                }

                if (splitState == SPLIT_DOING_HARD) {
                    splitState = SPLIT_IDLE;
                    lightOperationActive = false;
                    unlockTimerMutexIfNeeded(lockedHere);
                    ExposureEngine_Abort();
                    clearDoseModeLatch();
                    uiTriggerBeep(SND_END_PATTERN);
                    triggerInfo();
                    updateNextionUI(true);
                    return;
                }

                unlockTimerMutexIfNeeded(lockedHere);
            }
        }

        clearExposureState(false);
        clearDoseModeLatch();
        ExposureEngine_Acknowledge();
        triggerInfo();
        updateNextionUI(true);
    }
}

void handleExposureMetronome(unsigned long elapsedMs) {
    static unsigned long lastTick = 0;
    if (elapsedMs / 1000 != lastTick) {
        lastTick = elapsedMs / 1000;
        if (lastTick > 0) uiTriggerBeep(SND_CLICK); 
    }
}

void initializeDoseStateFromCurrentTimes() { refreshDisplayVariables(); }
void scaleDoseBWByEV(double evDelta)   { modifyExposureByEV(evDelta); }
void scaleDoseSoftByEV(double evDelta) { modifyExposureByEV(evDelta); }
void scaleDoseHardByEV(double evDelta) { modifyExposureByEV(evDelta); }
double getDoseBW() { return dose_bw; }
void setDoseBW(float dose) { dose_bw = (double)dose; }

// =============================================================================
// PFLICHTENHEFT FIX: Fehlende Funktionsdefinitionen aus Logic_Timer.h
// Diese Funktionen waren im Header deklariert, aber nie implementiert.
// Sie werden vom TestStrip- und Burn-Modus benötigt, um die Splitgrade-Dosen
// direkt zu setzen/lesen (z.B. beim Closed-Loop-Rückschreiben).
// =============================================================================
void setDoseSoft(float dose) { dose_soft = (double)dose; }
void setDoseHard(float dose) { dose_hard = (double)dose; }
double getDoseSoft() { return dose_soft; }
double getDoseHard() { return dose_hard; }

// Berechnet die geschätzten Anzeige-Sekunden aus der Dosis (für das UI im Dosis-Modus).
// Formel: Zeit = Dosis / Flux. Wenn kein Flux bekannt ist, Dosis als Fallback.
float getDisplaySecondsBW() {
    float flux = (baseFlux > 0.001f) ? baseFlux : 1.0f;
    return (float)(dose_bw / flux);
}
float getDisplaySecondsSoft() {
    float flux = (baseFlux > 0.001f) ? baseFlux : 1.0f;
    return (float)(dose_soft / flux);
}
float getDisplaySecondsHard() {
    float flux = (baseFlux > 0.001f) ? baseFlux : 1.0f;
    return (float)(dose_hard / flux);
}

void runStandardTimerLoop(char key) { (void)key; }