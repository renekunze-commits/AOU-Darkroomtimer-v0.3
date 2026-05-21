/* Mode_BW.cpp - Modus 1: Schwarz-Weiß-Belichtung (Input-Handling)

   Ausgelagert aus Logic_Timer.cpp (Phase 2 - B1 Refactoring).
   
   Encoder-Belegung (Pflichtenheft Modus 1):
     Encoder 1 (Links):  Belichtungszeit/Dosis (EV-Schritte)  → modifyExposureByEV()
     Encoder 2 (Mitte):  Lineare Zeitänderung (±0.5s)         → Zeit oder Dosis direkt
     Encoder 3 (Rechts): Gradation (G/B-Verhältnis)           → applyGradeShift()
    Enc 1 Lang:         Spektrale Mess-Sequenz                → triggerSpectralMeasurement()
    Enc 2 Lang:         Reset Time & Grade auf default        → globalSet.std_time
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
#include "Mode_BW.h"
#include "ExposureEngine.h"

static const TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(20);

bool handleBWInput(const InputEvent& evt, double evStep) {
    if (ExposureEngine_IsRunning() && evt.type != EVT_START_PRESSED) {
        return false;
    }

    switch (evt.type) {

        // =================================================================
        // Encoder 1 (Soft): Belichtungszeit/Dosis in EV-Schritten
        // Identisch zur bisherigen Logik in Logic_Timer.cpp EVT_ENC_SOFT/BW-Zweig
        // =================================================================
        case EVT_ENC_SOFT:
            // BETA-FIX: Schritt 3 - Jede manuelle Drehung am Encoder 1
            // invalidiert einen zuvor berechneten Auto-Vorschlag, damit keine
            // inkonsistente Uebernahme aus altem Messzustand erfolgen kann.
            bwAutoPending = false;
            modifyExposureByEV((double)evt.value * evStep);
            uiTriggerBeep(SND_NAV);
            return true;

        // =================================================================
        // Encoder 2 (Hard): Lineare Zeitänderung (±0.5s pro Klick)
        // K03 FIX: Encoder 2 im BW-Modus = lineare Zeitänderung
        // =================================================================
        case EVT_ENC_HARD:
            // BETA-FIX: Schritt 3 - Auch lineare manuelle Anpassungen (Encoder 2)
            // loeschen den Pending-Status, da der Nutzer bewusst in die Dosis-/
            // Zeitfuehrung eingegriffen hat.
            bwAutoPending = false;
            if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                if (hwSwitchDoseMode) {
                    double linearDose = baseFlux * 0.5 * evt.value;
                    dose_bw = fmax(0.1, dose_bw + linearDose);
                } else {
                    time_bw = fmax(TIME_MIN_S, time_bw + (float)(evt.value * 0.5));
                }
                xSemaphoreGive(gTimerMutex);
            }
            refreshDisplayVariables();
            uiTriggerBeep(SND_NAV);
            return true;

        // =================================================================
        // Enc 2 Click: Pending-Messwert explizit uebernehmen
        // =================================================================
        case EVT_ENTER_PRESSED:
            // BETA-FIX: Schritt 3 - Der Apply-Mechanismus uebernimmt den zuletzt
            // berechneten BW-Vorschlag nur auf explizite Nutzerbestaetigung.
            // Dadurch bleibt der Workflow sicher: Messung erzeugt nur Vorschlag,
            // ENTER fuehrt die Zustandsaenderung aktiv aus.
            {
                bool didApplyPending = false;
                if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                    // BETA-FIX: Schritt 3 - Pending-Status wird unter Mutex geprueft,
                    // damit zwischen Pruefung und Uebernahme kein Race-Condition-
                    // Fenster auf globalen Dosis-/Flag-Werten entstehen kann.
                    if (bwAutoPending) {
                        dose_bw = target_dose;
                        bwAutoPending = false;
                        didApplyPending = true;
                    }

                    xSemaphoreGive(gTimerMutex);
                }

                if (didApplyPending) {
                    // BETA-FIX: Schritt 3 - Nach erfolgreicher Uebernahme wird die
                    // Anzeige-/Zeitableitung sofort synchronisiert.
                    refreshDisplayVariables();
                    uiTriggerBeep(SND_DONE);
                    return true;
                }

                return false;
            }

        // =================================================================
        // Encoder 3 (Grade): Gradation verschieben
        // =================================================================
        case EVT_ENC_GRADE:
            // BETA-FIX: Schritt 3 - Jede manuelle Gradationsaenderung verwirft
            // den Pending-Vorschlag fuer konsistente Zustandsfuehrung zwischen
            // Messvorschlag und aktuell manuell eingestellter Belichtung.
            bwAutoPending = false;
            applyGradeShift(evt.value);
            uiTriggerBeep(SND_NAV);
            return true;

        // =================================================================
        // Enc 1 Lang: Spektrale Mess-Sequenz starten
        // =================================================================
        case EVT_BACK_LONG:
            if (probeConnected && !isMeasurementActive()) {
                if (triggerSpectralMeasurement()) {
                    uiTriggerBeep(SND_OK);
                    Serial.println("[BW] Spektrale Mess-Sequenz gestartet (Enc 1 Lang)");
                } else {
                    uiTriggerBeep(SND_WARN);
                    Serial.println("[BW] Messung konnte nicht gestartet werden");
                }
            } else {
                uiTriggerBeep(SND_HINT);
                Serial.println("[BW] Probe nicht verbunden - lokale Messung nicht implementiert");
            }
            return true;

        // =================================================================
        // Enc 2 Lang: Reset Time & Grade auf Standardwerte
        // =================================================================
        case EVT_ENTER_LONG:
            if (xSemaphoreTake(gTimerMutex, MUTEX_TIMEOUT) == pdTRUE) {
                time_bw = globalSet.std_time;
                grade_bw = 2.5;
                updateGradeMath();
                xSemaphoreGive(gTimerMutex);
                refreshDisplayVariables();
                uiTriggerBeep(SND_DONE);
            }
            return true;

        // =================================================================
        // Enc 3 Lang: Spektrale Mess-Sequenz starten
        // =================================================================
        case EVT_GRADE_LONG:
            if (probeConnected && !isMeasurementActive()) {
                if (triggerSpectralMeasurement()) {
                    uiTriggerBeep(SND_OK);
                    Serial.println("[BW] Spektrale Mess-Sequenz gestartet (Enc 3 Lang)");
                } else {
                    uiTriggerBeep(SND_WARN);
                    Serial.println("[BW] Messung konnte nicht gestartet werden");
                }
            } else {
                uiTriggerBeep(SND_HINT);
                Serial.println("[BW] Probe nicht verbunden - lokale Messung nicht implementiert");
            }
            return true;

        default:
            return false;
    }
}
