/*
    HW_Display.cpp - UI Controller (Full Beta Integration v0.5.10)
    
    Zentrale Zusammenführung der v0.5.8 Logik mit dem funktionalen UI-Konzept.
    Fix für Befund 1: Vollständige Entkopplung von Core 0 und Core 1 via xInputQueue.
*/

#include <Arduino.h>
#include <cstring>
#include <math.h>
#include "Globals.h"
#include "Config.h"
#include "DisplayManager.h"
#include "UI_Map.h"
#include "ExposureEngine.h"
#include "Logic_Measurement.h"
#include "Logic_Math.h"

// Shadow-Speicher Definition (v0.5.8 Erhalt)
LCDShadow lcdShadow = {"", "", 0, false}; 
static uint8_t lastR = 255, lastG = 255, lastB = 255;

// Nextion State Tracking
static int lastBtScr = -1;
static int lastMode = -1;
static double lastTimeBW = -1.0;
static double lastGradeBW = -1.0;
static bool lastLockPageActive = false;
static int lastZoneVal = -1;
static int lastSentZoneVal = -1;
static String lastZoneHist = "";
static String lastSgPhase = "";
static uint8_t prevZoneHistogram[11] = {0};

/**
 * Helfer: Injiziert ein Event in die Input-Queue.
 * Stellt sicher, dass die Geschäftslogik auf Core 1 (TaskRealtime) ausgeführt wird.
 */
static void injectVirtualButton(InputEventType type) {
    InputEvent evt;
    evt.type = type;
    evt.value = 0;
    evt.timestamp = millis();
    xQueueSend(xInputQueue, &evt, 0);
}

static const char* stepModeToLabel(StepSize mode) {
    switch (mode) {
        case STEP_FULL:  return "1/1";
        case STEP_HALF:  return "1/2";
        case STEP_THIRD: return "1/3";
        case STEP_SIXTH: return "1/6";
        default:         return "1/3";
    }
}

/**
 * v0.5.8 Hilfsfunktion: Fügt den EV-Step rechtsbündig in die LCD-Zeile ein.
 * Verhindert das manuelle Formatieren in jedem triggerInfo-Zweig.
 */
static void appendRightAlignedStep(char* line, size_t lineSize) {
    if (lineSize < 17) return;
    const char* step = stepModeToLabel(globalStepMode);
    const size_t stepLen = strlen(step);
    if (stepLen == 0 || stepLen > 16) return;
    const size_t stepPos = 16 - stepLen;
    const size_t maxBaseLen = (stepPos > 0) ? (stepPos - 1) : 0;

    char merged[17];
    memset(merged, ' ', 16);
    merged[16] = '\0';

    size_t baseLen = strlen(line);
    if (baseLen > maxBaseLen) baseLen = maxBaseLen;
    if (baseLen > 0) memcpy(merged, line, baseLen);
    if (stepPos > 0) merged[stepPos - 1] = ' ';
    memcpy(merged + stepPos, step, stepLen);
    memcpy(line, merged, 17);
}

static bool streq(const char* a, const char* b) {
    return (strcmp(a, b) == 0);
}

// =============================================================================
// LCD LOGIK (Shadow-Copy & Hardware-I2C)
// =============================================================================

void uiUpdateLCD(const char* l1, const char* l2, uint8_t progress) {
    if (xSemaphoreTake(xShadowMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // FIX A4: Hartes Padding mit Leerzeichen, um Geister-Zeichen zu überschreiben
        memset(lcdShadow.line1, ' ', 16);
        size_t len1 = strlen(l1);
        if (len1 > 16) len1 = 16;
        memcpy(lcdShadow.line1, l1, len1);
        lcdShadow.line1[16] = '\0';

        memset(lcdShadow.line2, ' ', 16);
        size_t len2 = strlen(l2);
        if (len2 > 16) len2 = 16;
        memcpy(lcdShadow.line2, l2, len2);
        lcdShadow.line2[16] = '\0';

        lcdShadow.progress = progress;
        lcdShadow.dirty = true;
        xSemaphoreGive(xShadowMutex);
    }
}

void processLCDShadow() {
    if (!lcdShadow.dirty) return;
    if (xSemaphoreTake(xShadowMutex, 0) == pdTRUE) {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            lcd.setCursor(0, 0); lcd.print(lcdShadow.line1);
            lcd.setCursor(0, 1); lcd.print(lcdShadow.line2);
            xSemaphoreGive(xI2CMutex);
        }
        lcdShadow.dirty = false;
        xSemaphoreGive(xShadowMutex);
    }
}

void smartLCD(const char* l1, const char* l2) { uiUpdateLCD(l1, l2, 0); }
void smartLCD(String l1, String l2) { uiUpdateLCD(l1.c_str(), l2.c_str(), 0); }

void handleLCDBacklight() {
    uint8_t r = 0, g = 0, b = 0;
    if (isRoomDarknessActive) { r = 0; g = 0; b = 0; }
    else if (overheatLock) { r = (millis() % 1000 < 500) ? 255 : 0; g = 0; b = 0; }
    else if (statusSafeOn) { r = (set_max > 0) ? set_max : 255; g = 0; b = 0; }
    else if (statusEnlargerOn) { r = 255; g = 255; b = 255; }
    else { uint8_t v = (set_lcd > 0) ? set_lcd : 100; r = v; g = v; b = v; }

    if (r != lastR || g != lastG || b != lastB) {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lcd.setRGB(r, g, b);
            xSemaphoreGive(xI2CMutex);
            lastR = r; lastG = g; lastB = b;
        }
    }
}

void triggerInfo() {
    char l1[17] = {0}; char l2[17] = {0};
    
    if (overheatLock) {
        strcpy(l1, "!! OVERHEAT !!"); strcpy(l2, "SYSTEM HALTED");
    } 
    else {
        switch (currentMode) {
            case MODE_BW:
                snprintf(l1, 17, "BW Mode %s", (burnEv != 0.0) ? "BURN" : "");
                // BETA-FIX: Schritt 3 - Solange ein neuer Messwert zur Uebernahme
                // ansteht, wird die zweite Zeile als eindeutige Apply-Vorschau
                // dargestellt. Die normale Dosis-/Zeitdarstellung wird nur gezeigt,
                // wenn kein Pending-Messwert vorhanden ist.
                if (bwAutoPending) {
                    // BETA-FIX: Schritt 4 - Zusaetzliches visuelles Prompting.
                    // Neben dem Dosisvorschlag blinkt ein Marker ("!"), damit der
                    // Nutzer eindeutig sieht, dass eine aktive Bestaetigung (ENTER)
                    // aussteht. Das Blinken nutzt nur millis() und blockiert nicht.
                    const bool blinkOn = ((millis() / 500UL) % 2UL) == 0UL;
                    snprintf(l2, 17, "APPLY? %0.1fd%s", target_dose, blinkOn ? "!" : " ");
                } else if (hwSwitchDoseMode) {
                    snprintf(l2, 17, "D:%0.1f G:%0.1f", dose_bw, grade_bw);
                } else {
                    snprintf(l2, 17, "T:%0.1fs G:%0.1f", time_bw, grade_bw);
                }
                break;
            case MODE_SG:
                strcpy(l1, "Splitgrade Mode");
                if (hwSwitchDoseMode) snprintf(l2, 17, "S:%0.1f H:%0.1f", dose_soft, dose_hard);
                else snprintf(l2, 17, "S:%0.1fs H:%0.1fs", time_soft, time_hard);
                break;
            case MODE_BURN:
                strcpy(l1, "Burn Mode");
                snprintf(l2, 17, "+%0.1f EV G:%0.1f", burnEv, burnGrade);
                break;
            case MODE_CALIB:
                strcpy(l1, "Paper Calibrate"); strcpy(l2, "ENTER to Start");
                break;
            case MODE_DENS:
                strcpy(l1, "Densitometer"); strcpy(l2, "ENTER to Start");
                break;
            case MODE_TESTSTRIP:
                strcpy(l1, "Test Strip"); strcpy(l2, "ENTER to Start");
                break;
            case MODE_SETUP:
                strcpy(l1, "System Setup"); strcpy(l2, "ENTER to Start");
                break;
            default:
                strcpy(l1, "Dukatimer v0.5"); break;
        }
        appendRightAlignedStep(l1, sizeof(l1));
    }
    uiUpdateLCD(l1, l2, 0);
}

// =============================================================================
// NEXTION UI LOGIK
// =============================================================================

/**
 * v0.5.8 Hilfsfunktion: Histogramm-Snapshot und Delta-Update.
 * Erhält die Performance bei vielen Messwerten.
 */
static void updateNextionHistogram(bool force, bool liveMetering) {
    uint8_t histogramSnapshot[11] = {0};
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    memcpy(histogramSnapshot, currentZoneHistogram, sizeof(histogramSnapshot));
    xSemaphoreGive(gTimerMutex);

    int changedZone = -1;
    for (int i = 0; i < 11; i++) {
        if (histogramSnapshot[i] != prevZoneHistogram[i]) changedZone = i;
        prevZoneHistogram[i] = histogramSnapshot[i];
    }
    if (changedZone >= 0) lastZoneVal = changedZone;
    if (lastZoneVal < 0) lastZoneVal = 0;

    if (force || liveMetering || lastZoneVal != lastSentZoneVal) {
        DM_setNumber(OBJ_SG_ZONE, lastZoneVal);
        lastSentZoneVal = lastZoneVal;
    }

    String hist; hist.reserve(64);
    for (int i = 0; i < 11; i++) {
        if (i > 0) hist += ",";
        hist += String((int)histogramSnapshot[i]);
    }
    if (force || liveMetering || hist != lastZoneHist) {
        DM_setText(OBJ_SG_HIST, hist);
        lastZoneHist = hist;
    }
}

void updateNextionUI(bool force) {
    const bool lockPageActive = ExposureEngine_IsRunning() || lightOperationActive;

    // 1. Seitensteuerung (Switch-Kaskade für Pages 0-9)
    if (force || currentMode != lastMode || lockPageActive != lastLockPageActive) {
        String cmd = "page ";
        if (lockPageActive) cmd += PAGE_LOCK;
        else if (bootScreenActive) cmd += PAGE_BOOT;
        else {
            switch(currentMode) {
                case MODE_BW: cmd += PAGE_MAIN; break;
                case MODE_SG: cmd += PAGE_SG; break;
                case MODE_BURN: cmd += PAGE_BURN; break;
                case MODE_CALIB: cmd += PAGE_CALIB; break;
                case MODE_TESTSTRIP: cmd += PAGE_TESTSTRIP; break;
                case MODE_DENS: cmd += PAGE_WIZARD; break;
                case MODE_SETUP: cmd += PAGE_MENU; break;
                case MODE_BRIDGE: cmd += PAGE_DIAG; break;
                default: cmd += PAGE_MAIN; break;
            }
        }
        DM_sendCommand(cmd.c_str());
        lastMode = currentMode;
        lastLockPageActive = lockPageActive;
    }

    // 2. Modus-spezifische Updates
    if (lockPageActive) {
        const unsigned long elapsedMs = ExposureEngine_GetElapsedMs();
        DM_setText(TXT_LOCK_TIME, String(elapsedMs / 1000.0, 1) + "s");
        DM_setNumber(OBJ_LOCK_PROG, 0);
    } 
    else {
        switch (currentMode) {
            case MODE_BW: {
                if (force || time_bw != lastTimeBW) { 
                    DM_setText(TXT_MAIN_TIME, String(time_bw, 1) + (hwSwitchDoseMode ? "d" : "s")); 
                    lastTimeBW = time_bw; 
                }
                if (force || grade_bw != lastGradeBW) { 
                    DM_setText(TXT_MAIN_GRADE, String(grade_bw, 1)); 
                    lastGradeBW = grade_bw; 
                }
                break;
            }

            case MODE_SG: {
                String sgPhase;
                if (splitState == SPLIT_DOING_SOFT) sgPhase = "SOFT RUN";
                else if (splitState == SPLIT_SOFT_DONE) sgPhase = "HARD READY";
                else if (splitState == SPLIT_DOING_HARD) sgPhase = "HARD RUN";
                else sgPhase = "SOFT READY";

                if (force || sgPhase != lastSgPhase) {
                    DM_setText(OBJ_SG_PHASE, sgPhase);
                    lastSgPhase = sgPhase;
                }
                updateNextionHistogram(force, isMeasurementActive());
                break;
            }

            case MODE_BURN: {
                DM_setText(TXT_BURN_TIME, String(getEffectiveBurnTime(), 1) + "s");
                DM_setText(TXT_BURN_GRADE, String(burnGrade, 1));
                DM_setText(TXT_BURN_EV, String(burnEv, 1) + " EV");
                break;
            }

            case MODE_DENS: {
                DM_setText(TXT_DENS_LUX, "Lux: " + String(remoteLux, 1));
                const double dens = (densRefLux > 0) ? log10(densRefLux / (remoteLux > 0.001 ? remoteLux : 0.001)) : 0.0;
                DM_setText(TXT_DENS_VAL, "D: " + String(dens, 2));
                break;
            }

            case MODE_BRIDGE: {
                DM_setText(TXT_DIAG_STATUS, probeConnected ? "CONNECTED" : "DISCONNECTED");
                DM_setText(TXT_DIAG_G0, "G0: " + String(probeLuxG0, 1));
                DM_setText(TXT_DIAG_G5, "G5: " + String(probeLuxG5, 1));
                DM_setText(TXT_DIAG_VAL, "Lost: " + String(totalDroppedPackets));
                break;
            }
        }
        
        // Globale Parameter (z.B. Screen Off)
        int valScr = isRoomDarknessActive ? 1 : 0;
        if (force || valScr != lastBtScr) {
            DM_setNumber(NUM_MAIN_SCR, valScr); lastBtScr = valScr;
        }
    }
}

// =============================================================================
// EVENT HANDLING (Nextion -> System)
// =============================================================================

void handleNextionEvent(const char* msg) {
    if (!msg || !*msg) return;

    // Funktionale Events (Neu)
    if (streq(msg, EVT_MAIN_START))  { beepClick(); injectVirtualButton(EVT_START_PRESSED); return; }
    if (streq(msg, EVT_MAIN_SAFE))   { beepClick(); safeLatch = !safeLatch; if (safeLatch) whiteLatch = false; return; }
    if (streq(msg, EVT_MAIN_FOCUS))  { beepClick(); if (!safeLatch) whiteLatch = !whiteLatch; return; }
    if (streq(msg, EVT_MAIN_SCREEN)) { beepClick(); isRoomDarknessActive = !isRoomDarknessActive; return; }
    if (streq(msg, EVT_MAIN_DENS))   { beepClick(); injectVirtualButton(EVT_DENS_REQUEST); return; }
    if (streq(msg, EVT_MAIN_APPLY))  { injectVirtualButton(EVT_ENTER_PRESSED); return; }
    if (streq(msg, EVT_MAIN_BRIDGE)) { currentMode = MODE_BRIDGE; return; }

    if (streq(msg, EVT_SG_MEASURE))  { injectVirtualButton(EVT_GRADE_LONG); return; }
    if (streq(msg, EVT_SG_START))    { injectVirtualButton(EVT_START_PRESSED); return; }
    if (streq(msg, EVT_SG_RESET))    { injectVirtualButton(EVT_GRADE_LONG); return; }
}

/**
 * handleNextionCommand: Erhält die Abwärtskompatibilität zu v0.5.8 (Button-IDs).
 */
void handleNextionCommand(const char* cmd) {
    if (!cmd) return;
    const char* payload = cmd;

    // Support for page scripts that prepend ASCII header "TEXT".
    if (strncmp(payload, "TEXT", 4) == 0) {
        payload += 4;
    }

    // First, try new functional string events (main_*, sg_*).
    handleNextionEvent(payload);

    // Keep legacy button IDs for v0.5.8 HMI compatibility.
    if (streq(payload, EVT_CMD_START)) { beepClick(); injectVirtualButton(EVT_START_PRESSED); }
    else if (streq(payload, EVT_CMD_SAFE)) { beepClick(); safeLatch = !safeLatch; if (safeLatch) whiteLatch = false; }
    else if (streq(payload, EVT_CMD_FOCUS)) { beepClick(); if (!safeLatch) whiteLatch = !whiteLatch; }
    else if (streq(payload, EVT_CMD_SCREEN)) { beepClick(); isRoomDarknessActive = !isRoomDarknessActive; }
    else if (streq(payload, EVT_CMD_DENS)) { beepClick(); injectVirtualButton(EVT_DENS_REQUEST); }
    else if (streq(payload, EVT_CMD_APPLY)) { injectVirtualButton(EVT_ENTER_PRESSED); }
}

void initDisplays() {
    DM_init(); 
    triggerInfo(); 
    handleLCDBacklight(); 
}