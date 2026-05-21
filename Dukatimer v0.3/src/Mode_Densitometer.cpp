/*
    Mode_Densitometer.cpp - v0.5 Root Cause Edition
    
    Zweck: Messung der optischen Dichte mit Dunkelstrom-Kompensation
           (D = log10((Ref - Dark) / (Meas - Dark))).
    
    Änderungen v0.5:
    - Vollständige Core-Isolation (Läuft auf Core 1).
    - I2C-Schutz via xI2CMutex für TSL2591 Zugriffe.
    - Asynchrones Input-Handling via xInputQueue.
    - Shadow-Display Integration zur Vermeidung von Jitter.
    - NEU: Dark Current Kalibrierung getrennt für baseDarkLux (TSL2591).
*/

#include <Arduino.h>
#include <math.h>
#include "Globals.h"
#include "Config.h"
#include "DisplayManager.h"
#include "Logic_Measurement.h"
#include "Logic_Timer.h"
#include "Logic_Storage.h" // Für saveSettings()

extern void updateNextionUI(bool force);

// Lokaler Status
static unsigned long densMsgUntil = 0;
static bool densAbort = false;
static float liveLux = 0.0;
static uint8_t idleMenuIdx = 0; // 0 = Normal, 1 = Dark Calib

void startDensitometerMode() {
    densState = DENS_IDLE;
    densRefLux = 0.0;
    densBaseFog = NAN; 
    densAbort = false;
    idleMenuIdx = 0;
    densMsgUntil = millis() + 1000;
    
    uiUpdateLCD("DENSITOMETER", "Mode Started");
    uiTriggerBeep(SND_OK);
    updateNextionUI(true);
}

/**
 * Hilfsfunktion zum sicheren Auslesen des TSL2591 (Base Sensor)
 */
static float getSafeLiveLux() {
    float result = 0.0;
    if (!tslBaseOK) return 0.0;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint32_t lum = tslBase.getFullLuminosity();
        uint16_t ir = lum >> 16;
        uint16_t full = lum & 0xFFFF;
        result = (float)tslBase.calculateLux(full, ir);
        xSemaphoreGive(xI2CMutex);
    }
    return result;
}

void runDensitometerWizard() {
    wdt_reset();

    // 1. UI-Pause abwarten
    if (millis() < densMsgUntil) return;

    // 2. Input konsumieren
    InputEvent evt;
    bool startPressed = false;
    bool backPressed = false;
    bool enterPressed = false;
    int encDelta = 0;

    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_START_PRESSED) startPressed = true;
        if (evt.type == EVT_GRADE_PRESSED) backPressed = true;
        if (evt.type == EVT_ENTER_PRESSED) enterPressed = true;
        if (evt.type == EVT_ENC_SOFT || evt.type == EVT_ENC_HARD) encDelta += evt.value;
    }

    if (backPressed) densAbort = true;

    // 3. Abort Handler
    if (densAbort) {
        densState = DENS_IDLE;
        densAbort = false;
        HW_SetEnlargerNeoPixel(0, 0, 0);
        isMeasuring = false; // FIX A5: Sperre beim Exit sicherheitshalber aufheben
        uiUpdateLCD("DENS EXIT", "");
        uiTriggerBeep(SND_BACK);
        updateNextionUI(true);
        // Rückkehr zum Hauptmodus erfolgt über die Main-Loop Steuerung
        return;
    }

    // 4. Live-Messung für die Anzeige
    liveLux = getSafeLiveLux();

    // 5. State Machine
    switch (densState) {
        case DENS_IDLE:
            isMeasuring = false; // FIX A5: Im Idle keine Sperre
            // Encoder-Navigation für das Untermenü (Referenz vs. Dunkelstrom)
            if (encDelta != 0) {
                idleMenuIdx = (idleMenuIdx == 0) ? 1 : 0;
                uiTriggerBeep(SND_NAV);
                updateNextionUI(true);
            }

            if (idleMenuIdx == 0) {
                // Normales Menü
                uiUpdateLCD("DENSITOMETER", "START=REF ENTER=M");
                
                if (startPressed) {
                    densState = DENS_REF;
                    uiTriggerBeep(SND_NAV);
                    updateNextionUI(true);
                }
                
                if (enterPressed) {
                    // RefLux muss signifikant höher sein als das Rauschen!
                    if (densRefLux > baseDarkLux + 0.001) {
                        densState = DENS_MEAS;
                        uiTriggerBeep(SND_OK);
                        updateNextionUI(true);
                    } else {
                        uiUpdateLCD("NO REF DATA", "Calibrate first!");
                        uiTriggerBeep(SND_WARN);
                        densMsgUntil = millis() + 1500;
                        updateNextionUI(true);
                    }
                }
            } else {
                // Dark Calibration Menü
                uiUpdateLCD("DENS: DARK CALIB", "START=CAL DARK");
                
                if (startPressed) {
                    isMeasuring = true; // FIX A5: Sperre aktivieren für Dark Calib
                    uiUpdateLCD("MEASURING...", "Keep Dark!");
                    // Absolutes Grounding: Alle Lichter physikalisch aus!
                    HW_SetFocus(false);
                    HW_SetSafelight(false);
                    HW_SetEnlargerNeoPixel(0,0,0);
                    vTaskDelay(pdMS_TO_TICKS(500)); // Sensor beruhigen lassen
                    
                    float sum = 0;
                    for(int i=0; i<10; i++) {
                        sum += getSafeLiveLux();
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    baseDarkLux = (double)(sum / 10.0);
                    
                    saveSettings(); // Dauerhaft im EEPROM ablegen
                    
                    isMeasuring = false; // FIX A5: Sperre aufheben
                    uiUpdateLCD("DARK LUX SAVED", "System Grounded");
                    uiTriggerBeep(SND_DONE);
                    idleMenuIdx = 0;
                    densMsgUntil = millis() + 1500;
                    updateNextionUI(true);
                }
            }
            break;

        case DENS_REF: // Referenzmessung (D=0)
            isMeasuring = true; // FIX A5: Sperre aktivieren für Referenzmessung
            uiUpdateLCD("CALIBRATE REF", "Place Probe!");
            HW_SetFocus(true); // Weißlicht an zum Messen
            
            if (startPressed) {
                float sum = 0;
                for(int i=0; i<5; i++) {
                    sum += getSafeLiveLux();
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                densRefLux = sum / 5.0;
                updateNextionUI(true);
                
                if (densRefLux > baseDarkLux + 0.01) {
                    uiUpdateLCD("REF SAVED", "D=0.00 Set");
                    uiTriggerBeep(SND_DONE);
                    densState = DENS_IDLE;
                    densMsgUntil = millis() + 1000;
                    updateNextionUI(true);
                } else {
                    uiUpdateLCD("TOO DARK!", "Check Light");
                    uiTriggerBeep(SND_WARN);
                    updateNextionUI(true);
                }
                HW_SetFocus(false);
                isMeasuring = false; // FIX A5: Sperre aufheben
            }
            break;

        case DENS_MEAS: // Kontinuierliche Dichte-Messung
        {
            isMeasuring = true; // FIX A5: Sperre aktivieren für die kontinuierliche Messung
            HW_SetFocus(true);
            
            char l1[17], l2[17];
            double D = 0.0;
            double net = 0.0;
            bool netValid = false;
            
            // ROOT CAUSE FIX: Dunkelstrom-Korrektur (Systematic Error bei D > 2.0)
            double corrRef = (double)densRefLux - baseDarkLux;
            double corrLive = (double)liveLux - baseDarkLux;

            // Schutz vor Division-by-Zero oder Logarithmus von Negativwerten
            if (corrRef > 0.00001 && corrLive > 0.00001) {
                D = log10(corrRef / corrLive);
                if (D < 0.0) D = 0.0;
                snprintf(l1, 17, "Lx:%0.1f D:%0.2f", liveLux, D);
            } else {
                snprintf(l1, 17, "Lx:%0.1f D:---", liveLux);
            }

            if (!isnan(densBaseFog)) {
                net = D - densBaseFog;
                netValid = true;
                snprintf(l2, 17, "Net:%0.2f  *Z8*", net);
            } else {
                snprintf(l2, 17, "START=SET BASE");
            }

            uiUpdateLCD(l1, l2);
            updateNextionUI(true);

            if (startPressed) {
                densBaseFog = D;
                uiTriggerBeep(SND_VALUE);
                updateNextionUI(true);
            }
            
            if (enterPressed) {
                if (netValid) {
                    // BW-Dens-Workflow: Messwert uebernehmen -> BW-Belichtung korrigieren.
                    // deltaDensity > 0: Ziel dichter als Ist -> laenger belichten.
                    const double deltaDensity = zone8TargetNet - net;
                    const double evCorrection = deltaDensity * 3.32192809489; // log2(10)
                    const double factor = pow(2.0, evCorrection);

                    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                        if (hwSwitchDoseMode) {
                            dose_bw = fmax(0.1, fmin(999.0, dose_bw * factor));
                        } else {
                            time_bw = (float)fmax(TIME_MIN_S, fmin(TIME_MAX_S, (double)time_bw * factor));
                        }
                        trackDensityEV = evCorrection;
                        measurementOverrideActive = true;
                        xSemaphoreGive(gTimerMutex);
                    }

                    extern void refreshDisplayVariables();
                    refreshDisplayVariables();
                    uiUpdateLCD("DENS APPLIED", "BW UPDATED");
                    uiTriggerBeep(SND_DONE);
                }
                densState = DENS_IDLE;
                HW_SetFocus(false);
                isMeasuring = false; // FIX A5: Sperre aufheben
                updateNextionUI(true);
            }
            break;
        }

        default:
            break;
    }
}