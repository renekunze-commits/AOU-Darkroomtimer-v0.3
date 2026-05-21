/* Logic_Measurement.cpp - v0.5 Root Cause Edition
   
   ARCHITEKTUR-REGELN:
   - Läuft primär auf Core 1 (TaskRealtime).
   - Datensicherheit via gTimerMutex für das Histogramm und den Status.
   - Direkte HAL-Kommunikation für minimalen Jitter bei Mess-Blitzen.
   - Konsistenz: Nutzt currentZoneHistogram aus Globals.h.
   - NEU: probeDarkLux Abzug zur Vermeidung von systematischen Werten bei D>2.0
*/

#include "Logic_Measurement.h"
#include "Globals.h"
#include "Logic_Math.h"
#include "Logic_Papers.h"
#include "DisplayManager.h"
#include <Arduino.h>
#include <math.h>

// NEXTION UI INTEGRATION (Nur Ergänzung)
extern void updateNextionUI(bool force);

// --- Lokaler Status (Isoliert auf Core 1) ---
enum SpectralState {
    SPEC_IDLE,
    SPEC_G0_WARMUP,
    SPEC_G0_WAIT_DATA,
    SPEC_G5_WARMUP,
    SPEC_G5_WAIT_DATA
};

static SpectralState internalSpecState = SPEC_IDLE;
static unsigned long measureTimer = 0;
static float tempLuxG0 = 0.0;
static float tempLuxG5 = 0.0;
static int lastMeasuredZone = -1; 
static bool wasSafelightOn = false;
static bool currentMeasurementUsesWhite = false;

// Externe Wireless-Funktionen (HW_Wireless.cpp)
extern void sendProbeMeasureCmd(uint8_t cmd);
extern void sendRenderPacketToC6();

// =============================================================================
// SESSION MANAGEMENT
// =============================================================================

void startMeteringSession() {
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        internalSpecState = SPEC_IDLE;
        tempLuxG0 = 0.0;
        tempLuxG5 = 0.0;
        lastMeasuredZone = -1;
        
        // Histogramm leeren (Grounding des Belichtungsspeichers)
        for(int i=0; i<11; i++) {
            currentZoneHistogram[i] = 0;
        }
        xSemaphoreGive(gTimerMutex);
        Serial.println("[MEASURE] Neue Spot-Session gestartet. Histogramm geleert.");
        
        updateNextionUI(true); // UI-Trigger
    }
}

void undoLastMeasurement() {
    if (lastMeasuredZone != -1) {
        if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Ein Spot entspricht 20 Einheiten im Histogramm (Anzeige-Skalierung)
            if (currentZoneHistogram[lastMeasuredZone] >= 20) {
                currentZoneHistogram[lastMeasuredZone] -= 20;
            } else {
                currentZoneHistogram[lastMeasuredZone] = 0;
            }
            lastMeasuredZone = -1; 
            xSemaphoreGive(gTimerMutex);
            
            sendRenderPacketToC6(); 
            uiTriggerBeep(SND_BACK);
            
            updateNextionUI(true); // UI-Trigger
        }
    }
}

void finalizeMeteringSession() {
    // FIX K4: Hardware-Restore identisch zu abortMeasurementWithError()
    HW_SetEnlargerNeoPixel(0, 0, 0);
    if (wasSafelightOn) HW_SetSafelight(true);

    internalSpecState = SPEC_IDLE;
    isMeasuring = false;
    lightOperationActive = false;
    currentMeasurementUsesWhite = false;
    Serial.println("[MEASURE] Session beendet.");
    
    updateNextionUI(true); // UI-Trigger
}

bool isMeasurementActive() {
    return (internalSpecState != SPEC_IDLE);
}

bool isMeteringActive() {
    return isMeasurementActive();
}

// =============================================================================
// STATE MACHINE (Core 1 - Hochfrequent)
// =============================================================================

void abortMeasurementWithError(const char* line1) {
    // Hardware-Grounding: Sicherer Lichtzustand wiederherstellen
    HW_SetEnlargerNeoPixel(0, 0, 0);
    if (wasSafelightOn) HW_SetSafelight(true);
    
    internalSpecState = SPEC_IDLE;
    isMeasuring = false; // FIX A5: Beim Abbruch zwingend clearen
    lightOperationActive = false;
    currentMeasurementUsesWhite = false;
    if (line1 != nullptr) Serial.printf("[MEASURE ERROR] %s\n", line1);
    
    uiTriggerBeep(SND_WARN);
    sendRenderPacketToC6(); 
    
    updateNextionUI(true); // UI-Trigger
}

bool triggerSpectralMeasurement() {
    if (internalSpecState != SPEC_IDLE) return false; 

    bool fixedGradeActive = false;
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        // BETA-FIX: Schritt 4 - Beim Start jeder neuen Messung wird ein alter
        // Pending-Vorschlag explizit verworfen. So wird verhindert, dass waehrend
        // einer frischen Mess-Session noch ein vorheriger, nicht uebernommener
        // Dosisvorschlag als gueltig angezeigt oder uebernommen wird.
        bwAutoPending = false;
        fixedGradeActive = getActivePaper().isFixedGrade;
        xSemaphoreGive(gTimerMutex);
    }
    
    isMeasuring = true; // FIX A5: Licht-Sperre aktivieren!
    lightOperationActive = true;
    wasSafelightOn = statusSafeOn;
    currentMeasurementUsesWhite = fixedGradeActive;
    
    // Mess-Blitz Phase 1: Fix-Grade nutzt direkt Weißlicht, VC bleibt beim G/B-Split.
    HW_SetSafelight(false);

    if (fixedGradeActive) {
        HW_SetEnlargerNeoPixel(0, 255, 255);
    } else {
        HW_SetEnlargerNeoPixel(0, 255, 0);
    }
    
    // NEU: Akustisches START-Signal an der Basis für sofortiges Feedback
    uiTriggerBeep(SND_CLICK); 
    
    measureTimer = millis();
    internalSpecState = SPEC_G0_WARMUP;
    
    updateNextionUI(true); // UI-Trigger
    return true;
}

bool handleMeasurementStateMachine(uint8_t evt, float luxG0, float luxG5) {
    if (internalSpecState == SPEC_IDLE) return false;

    // FIX: Remote Cancel (EVT_T1_CLICK vom C6) während aktiver Messung
    if (evt == EVT_T1_CLICK) {
        abortMeasurementWithError("Remote Cancel");
        return true;
    }

    unsigned long now = millis();

    switch (internalSpecState) {
        
        case SPEC_G0_WARMUP:
            // LED Einschwingzeit abwarten (150ms für stabile Farbtemperatur)
            if (now - measureTimer >= 150) { 
                sendProbeMeasureCmd(1); // CMD_MEASURE_G0 an C6
                measureTimer = now; 
                internalSpecState = SPEC_G0_WAIT_DATA;
            }
            break;

        case SPEC_G0_WAIT_DATA:
            if (evt == EVT_LUX_DATA) { 
                tempLuxG0 = luxG0;
                if (currentMeasurementUsesWhite) {
                    // FIX: Protect white-light fallback if both values too low
                    const float whiteLux0 = luxG0;
                    const float whiteLux5 = luxG5;
                    const float whiteLux = (whiteLux0 > 0.01f) ? whiteLux0 :
                                           (whiteLux5 > 0.01f) ? whiteLux5 : 0.0f;
                    if (whiteLux <= 0.0f) {
                        abortMeasurementWithError("low white lux");
                        break;
                    }
                    tempLuxG0 = whiteLux;
                    tempLuxG5 = whiteLux;

                    HW_SetEnlargerNeoPixel(0, 0, 0);
                    if (wasSafelightOn) HW_SetSafelight(true);

                    processSpotMeasurement(tempLuxG0, tempLuxG5);
                    sendRenderPacketToC6(); 
                    internalSpecState = SPEC_IDLE;
                    isMeasuring = false;
                    lightOperationActive = false;
                    currentMeasurementUsesWhite = false;
                    uiTriggerBeep(SND_OK);
                    
                    updateNextionUI(true); // UI-Trigger
                    break;
                }

                // Sofortiger Wechsel zu Blau (G5) für Phase 2
                // ROOT CAUSE FIX: Echtes Blau für G5 Messung
                HW_SetEnlargerNeoPixel(0, 0, 255); 
                measureTimer = millis();
                internalSpecState = SPEC_G5_WARMUP;
            } 
            // Remote-Handshake-Timeout auf 2500ms erhöht
            else if (now - measureTimer > 2500) {
                abortMeasurementWithError("Timeout G0"); 
            }
            break;

        case SPEC_G5_WARMUP:
            if (now - measureTimer >= 150) {
                sendProbeMeasureCmd(2); // CMD_MEASURE_G5 an C6
                measureTimer = now;
                internalSpecState = SPEC_G5_WAIT_DATA;
            }
            break;

        case SPEC_G5_WAIT_DATA:
            if (evt == EVT_LUX_DATA) {
                tempLuxG5 = luxG5;
                
                // Mess-Sequenz beendet: Hardware-Restore
                HW_SetEnlargerNeoPixel(0, 0, 0);
                if (wasSafelightOn) HW_SetSafelight(true);
                
                processSpotMeasurement(tempLuxG0, tempLuxG5);
                sendRenderPacketToC6(); 
                internalSpecState = SPEC_IDLE;
                isMeasuring = false; // FIX A5: Sperre aufheben, Messung fertig
                lightOperationActive = false;
                currentMeasurementUsesWhite = false;
                uiTriggerBeep(SND_OK);
                
                updateNextionUI(true); // UI-Trigger
            }
            else if (now - measureTimer > 2500) {
                abortMeasurementWithError("Timeout G5");
            }
            break;
            
        default:
            break;
    }
    return true; 
}

// =============================================================================
// MATH ENGINE & HISTOGRAM
// =============================================================================

void processSpotMeasurement(float luxG0, float luxG5) {
    // ROOT CAUSE FIX: C6 Dunkelstrom für Dichten > 2.0 eliminieren
    float corrG0 = luxG0 - (float)probeDarkLux;
    float corrG5 = luxG5 - (float)probeDarkLux;

    if (corrG0 <= 0.01f || corrG5 <= 0.01f) {
        Serial.println("[MATH ERROR] Bereinigte Lux-Werte zu gering (Dunkelstrom?). Ignoriert.");
        return;
    }

    // Lux -> EV Berechnung (Logarithmisch zur Basis 2)
    float ev_G0 = log2(corrG0); 
    int berechneteZone = (int)round(ev_G0); 

    // Begrenzung auf Zonen 0 bis 10 (Zonensystem)
    if (berechneteZone < 0) berechneteZone = 0;
    if (berechneteZone > 10) berechneteZone = 10;
    
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Mode modeSnapshot = currentMode;

        // Histogramm-Eintrag (Max 240 für Balken-Anzeige)
        if (currentZoneHistogram[berechneteZone] < 240) {
            currentZoneHistogram[berechneteZone] += 20; 
        }
        lastMeasuredZone = berechneteZone;
        
        // Spektral-Verhältnis (G/B Ratio) für die Papier-Korrektur
        spectral_ratio = (double)(corrG0 / corrG5);
        
        xSemaphoreGive(gTimerMutex);

        // BETA-FIX: BW-Kopplung aktivieren, ohne dose_bw direkt zu ueberschreiben.
        // Die Messung bleibt damit sicher (kein sofortiger Fogging-Risiko-Sprung).
        if (modeSnapshot == MODE_BW) {
            double suggestedDose = target_dose;
            calculateAutoBW(corrG0, suggestedDose);

            if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // BETA-FIX: Schritt 3 - Vorschlag gepuffert in target_dose.
                // Direkte Aenderung von dose_bw ist explizit nicht gewuenscht.
                target_dose = suggestedDose;

                // BETA-FIX: Schritt 3 - Signalisierung erst NACH berechnetem
                // und uebernommenem target_dose, damit der Pending-Status exakt
                // einen konsistenten, aktuellen Vorschlagswert repraesentiert.
                bwAutoPending = true;
                xSemaphoreGive(gTimerMutex);
            }
        }
        
        Serial.printf("[MATH] Spot! Zone: %d | Ratio: %.3f\n", berechneteZone, spectral_ratio);
        
        updateNextionUI(true); // UI-Trigger für Histogramm
    }
}

void handleMeteringSession(bool actionSave, bool actionCancel, bool actionToggle, bool actionReset, bool actionUndo) {
    if (actionCancel) {
        finalizeMeteringSession();
        uiTriggerBeep(SND_BACK);
    }
    if (actionUndo) {
        undoLastMeasurement();
    }
    if (actionReset) {
        startMeteringSession();
        uiTriggerBeep(SND_WARN);
    }
}