/* Mode_Setup.cpp - v0.5 Root Cause Edition
    
    Zweck: Systemkonfiguration und Papier-Management.
    
    Architektur-Heilung v0.5:
    - Umstellung auf non-blocking State-Machine (kein while-Trauma).
    - Konsumierung der zentralen xInputQueue (Core 1).
    - Thread-Sichere Parameteränderung via gTimerMutex.
    - Vollständige Integration in das Shadow-Copy Display-Verfahren.
    - NEU: Sub-State für C6 Dark Current Kalibrierung (Systematic Error Fix).
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "DisplayManager.h"
#include "Logic_Papers.h"
#include "Logic_Storage.h"
#include "Logic_Timer.h"
#include "Logic_Math.h"

extern void updateNextionUI(bool force);

// --- Lokaler Status ---
static int menuIdx = 0;
static const int NUM_ITEMS = 16; // v0.5 + BETA-FIX [Phase 2]: BW Ziel-Zone ergänzt
static unsigned long setupMsgUntil = 0;
static int setupSubState = 0;    // 0 = Normales Menü, 1 = C6 Dark Calib Wizard

// Externe Modus-Starter (Wizards)
extern void startCalibrationWizard();
extern void startTestStripMode();

// H01 FIX: Nav/Edit State Machine (laut Spec: 2 harte Zustände)
static bool editActive = false;
static double editBackup = 0;

static bool isActionItem(int idx) {
    // BETA-FIX: [Phase 2] Durch den neuen Edit-Eintrag "BW Ziel-Zone"
    // verschieben sich die Action-Indizes um +1 ab dem alten Index 5.
    return idx == 10 || idx == 11 || idx == 12 || idx == 14 || idx == 15;
}

static void saveEditSnapshot() {
    switch (menuIdx) {
        case 0:  editBackup = globalSet.std_time; break;
        case 1:  editBackup = globalSet.splitMode; break;
        case 2:  editBackup = set_lcd; break;
        case 3:  editBackup = set_focus; break;
        case 4:  editBackup = set_max; break;
        case 5:  editBackup = globalSet.bwTargetZone; break;
        case 6:  editBackup = getActivePaper().flashEnable; break;
        case 7:  editBackup = (int)globalSet.soundMode; break;
        case 8:  editBackup = getActivePaper().flashLevel; break;
        case 9:  editBackup = getActivePaper().flashColor; break;
        case 13: editBackup = useWirelessProbe; break;
        default: editBackup = 0; break;
    }
}

static void restoreEditSnapshot() {
    switch (menuIdx) {
        case 0:  globalSet.std_time = editBackup; break;
        case 1:  globalSet.splitMode = (uint8_t)editBackup; break;
        case 2:  set_lcd = (uint8_t)editBackup; break;
        case 3:  set_focus = (uint8_t)editBackup; break;
        case 4:  set_max = (uint8_t)editBackup; break;
        case 5:  globalSet.bwTargetZone = (float)editBackup; break;
        case 6:  getActivePaper().flashEnable = (bool)(int)editBackup; break;
        case 7:  globalSet.soundMode = (SoundMode)(int)editBackup; break;
        case 8:  getActivePaper().flashLevel = (int)editBackup; break;
        case 9:  getActivePaper().flashColor = (int)editBackup; break;
        case 13: useWirelessProbe = (bool)(int)editBackup; break;
        default: break;
    }
}

// =============================================================================
// SETUP MENU LOGIC
// =============================================================================

void startSetupMenu() {
    setupMenuActive = true;
    menuIdx = 0;
    setupSubState = 0; editActive = false; // Grounding: Immer im Hauptmenü starten
    setupMsgUntil = millis() + 500;
    uiUpdateLCD("SYSTEM SETUP", "Encoder to Nav");
    uiTriggerBeep(SND_OK);
    updateNextionUI(true);
}

/**
 * runSetupTick
 * Wird vom TaskRealtime gerufen. Konsumiert die Input-Queue exklusiv,
 * wenn setupMenuActive aktiv ist.
 */
void runSetupTick() {
    wdt_reset();
    unsigned long now = millis();
    if (now < setupMsgUntil) return;

    // 1. Input-Queue konsumieren
    InputEvent evt;
    bool enterPressed = false;
    bool backPressed = false;
    int encDelta = 0;    // Encoder 1: Einzige Steuerung (laut Spec)

    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_ENTER_PRESSED) enterPressed = true;
        if (evt.type == EVT_BACK_PRESSED)  backPressed = true;
        if (evt.type == EVT_ENC_SOFT)      encDelta += evt.value;
        // EVT_ENC_HARD: Im Setup gesperrt (laut Spec)
    }

    // H01 FIX: Nav/Edit Input-Routing
    int navDelta = 0;   // Cursor (nur Nav-State)
    int adjDelta = 0;   // Wertänderung (nur Edit-State)
    if (editActive) { adjDelta = encDelta; } else { navDelta = encDelta; }

    // 2. Navigation (Encoder 1, nur im Nav-State + Hauptmenü)
    if (navDelta != 0 && setupSubState == 0 && !editActive) {
        int nextIdx = menuIdx + navDelta;
        if (nextIdx < 0) {
            menuIdx = 0;
            uiTriggerBeep(SND_LIMIT);
        } else if (nextIdx >= NUM_ITEMS) {
            menuIdx = NUM_ITEMS - 1;
            uiTriggerBeep(SND_LIMIT);
        } else {
            menuIdx = nextIdx;
            uiTriggerBeep(SND_NAV);
        }
        updateNextionUI(true);
    }

    // 3. ENTER: State-Transitions (Nav→Edit / Edit→Save)
    if (enterPressed && setupSubState == 0) {
        if (!editActive) {
            if (!isActionItem(menuIdx)) {
                editActive = true;
                saveEditSnapshot();
                uiTriggerBeep(SND_OK);
                enterPressed = false;
                updateNextionUI(true);
            }
            // Action-Items: enterPressed bleibt für Switch
        } else {
            editActive = false;
            uiTriggerBeep(SND_DONE);
            enterPressed = false;
            updateNextionUI(true);
        }
    }

    // 4. BACK: Escape (Edit→Nav) oder Save & Exit (Nav→Haupt)
    if (backPressed) {
        if (editActive) {
            restoreEditSnapshot();
            editActive = false;
            uiTriggerBeep(SND_BACK);
            backPressed = false;
            updateNextionUI(true);
        } else if (setupSubState == 0) {
            saveSettings();
            uiUpdateLCD("SETTINGS SAVED", "");
            uiTriggerBeep(SND_DONE);
            setupMenuActive = false;
            editActive = false;
            setupMsgUntil = now + 800;
            updateNextionUI(true);
            return;
        }
    }

    // 5. Menü-Verarbeitung (Grounding via gTimerMutex)
    char l1[17] = {0}, l2[17] = {0};
    const char* eM = editActive ? ">" : " "; // Edit-Marker

    // Wir sichern den Zugriff auf die globalen Settings ab
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        
        // --- SUB-STATE: C6 DARK CALIBRATION WIZARD ---
        if (setupSubState == 1) {
            strcpy(l1, "DARK: PRESS C6-M"); // Anweisung: "Messen" auf C6 drücken
            snprintf(l2, 17, "Lx:%.3f ENT=SAV", probeLuxG0); // Zeigt asynchron empfangenen Wert

            // Hardware-Grounding: Erzwinge absolute Dunkelheit im Raum!
            HW_SetFocus(false);
            HW_SetSafelight(false);
            HW_SetEnlargerNeoPixel(0, 0, 0);

            if (enterPressed) {
                // Wert übernehmen und im RAM/EEPROM verankern
                probeDarkLux = probeLuxG0;
                globalSet.probe_dark_lux = probeDarkLux;
                setupSubState = 0;
                
                uiTriggerBeep(SND_DONE);
                uiUpdateLCD("DARK LUX SAVED", "");
                setupMsgUntil = now + 1000;
                updateNextionUI(true);
                xSemaphoreGive(gTimerMutex);
                return; // Early Exit, um Display-Overwrite zu verhindern
            } 
            else if (backPressed) {
                setupSubState = 0;
                uiTriggerBeep(SND_BACK);
                uiUpdateLCD("CALIB ABORTED", "");
                setupMsgUntil = now + 1000;
                updateNextionUI(true);
                xSemaphoreGive(gTimerMutex);
                return; // Early Exit
            }
        } 
        // --- NORMALES HAUPTMENÜ ---
        else {
            switch (menuIdx) {
                case 0: // STANDARD TIME
                    strcpy(l1, "STD TIME BW");
                    if (adjDelta != 0) {
                        globalSet.std_time = constrain(globalSet.std_time + (adjDelta * 0.5), 1.0, 60.0);
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%.1f s", eM, globalSet.std_time);
                    break;

                case 1: // SPLIT MODE
                    strcpy(l1, "SPLIT MODE");
                    if (adjDelta != 0) {
                        globalSet.splitMode = !globalSet.splitMode;
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%s", eM, globalSet.splitMode ? "AUTO FILL" : "SEPARATE");
                    break;

                case 2: // LCD BRIGHTNESS
                    strcpy(l1, "LCD BRIGHT");
                    if (adjDelta != 0) {
                        set_lcd = (uint8_t)constrain((int)set_lcd + (adjDelta * 10), 10, 255);
                        uiTriggerBeep(SND_VALUE);
                        handleLCDBacklight();
                    }
                    snprintf(l2, 17, "%s%d", eM, set_lcd);
                    break;

                case 3: // FOCUS BRIGHTNESS
                    strcpy(l1, "FOCUS BRIGHT");
                    if (adjDelta != 0) {
                        set_focus = (uint8_t)constrain((int)set_focus + (adjDelta * 10), 0, 255);
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%d", eM, set_focus);
                    break;

                case 4: // SAFE LIGHT LIMIT
                    strcpy(l1, "SAFE LIGHT MAX");
                    if (adjDelta != 0) {
                        set_max = (uint8_t)constrain((int)set_max + (adjDelta * 10), 10, 255);
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%d", eM, set_max);
                    break;

                case 5: // GLOBAL FLASH ENABLE
                    // BETA-FIX: [Phase 2] Dynamische Zielzone fuer Auto-BW.
                    // Einstellbereich 4.0..9.0 in 0.5er Schritten.
                    strcpy(l1, "BW ZIEL-ZONE");
                    if (adjDelta != 0) {
                        double next = (double)globalSet.bwTargetZone + ((double)adjDelta * 0.5);
                        globalSet.bwTargetZone = (float)constrain(next, 4.0, 9.0);
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%.1f", eM, globalSet.bwTargetZone);
                    break;

                case 6: // GLOBAL FLASH ENABLE
                    strcpy(l1, "FLASH GLOBAL");
                    if (adjDelta != 0) {
                        getActivePaper().flashEnable = !getActivePaper().flashEnable;
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%s", eM, getActivePaper().flashEnable ? "ENABLED" : "DISABLED");
                    break;

                case 7: // SOUND CONFIG
                    strcpy(l1, "SOUND MODE");
                    if (adjDelta != 0) {
                        int s = (int)globalSet.soundMode + adjDelta;
                        if (s > 2) s = 0; if (s < 0) s = 2;
                        globalSet.soundMode = (SoundMode)s;
                        uiTriggerBeep(SND_VALUE);
                    }
                    {
                        const char* sMap[] = {"OFF", "QUIET", "NORMAL"};
                        snprintf(l2, 17, "%s%s", eM, sMap[globalSet.soundMode]);
                    }
                    break;

                case 8: // PREFLASH LEVEL
                    strcpy(l1, "FLASH LEVEL");
                    if (adjDelta != 0) {
                        getActivePaper().flashLevel = (uint8_t)constrain((int)getActivePaper().flashLevel + adjDelta, 1, 5);
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%sLevel: %d/5", eM, getActivePaper().flashLevel);
                    break;

                case 9: // PREFLASH COLOR
                    strcpy(l1, "FLASH COLOR");
                    if (adjDelta != 0) {
                        getActivePaper().flashColor = (getActivePaper().flashColor == 0) ? 1 : 0;
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%s", eM, (getActivePaper().flashColor == 1) ? "GREEN" : "WHITE");
                    break;

                case 10: // TEACH PAPER
                    strcpy(l1, "TEACH PAPER");
                    strcpy(l2, "ENTER = START");
                    if (enterPressed) {
                        startCalibrationWizard();
                        // Wir deaktivieren das Setup-Menü, damit der Wizard die Queue bekommt
                        setupMenuActive = false; 
                        updateNextionUI(true);
                    }
                    break;

                case 11: // TEST STRIP
                    strcpy(l1, "TEST STRIP");
                    strcpy(l2, "ENTER = START");
                    if (enterPressed) {
                        startTestStripMode();
                        setupMenuActive = false;
                        updateNextionUI(true);
                    }
                    break;

                case 12: // ERROR RESET
                    strcpy(l1, "SYSTEM ERRORS");
                    if (lastSystemError == ERR_NONE) {
                        strcpy(l2, "Status: OK");
                    } else {
                        strcpy(l2, "ENTER=RESET");
                        if (enterPressed) {
                            lastSystemError = ERR_NONE;
                            clearErrorState();
                            uiTriggerBeep(SND_OK);
                            updateNextionUI(true);
                        }
                    }
                    break;

                case 13: // SENSOR SOURCE
                    strcpy(l1, "SENSOR SOURCE");
                    if (adjDelta != 0) {
                        useWirelessProbe = !useWirelessProbe;
                        uiTriggerBeep(SND_VALUE);
                    }
                    snprintf(l2, 17, "%s%s", eM, useWirelessProbe ? "WIRELESS" : "WIRED");
                    break;
                    
                case 14: // ROOT CAUSE FIX: C6 DARK CALIBRATION
                    strcpy(l1, "C6 DARK CALIB");
                    strcpy(l2, "ENTER = START");
                    if (enterPressed) {
                        setupSubState = 1;
                        setupMsgUntil = now + 500; // Kurze Pause, um Fehl-Doppelklicks zu vermeiden
                        uiTriggerBeep(SND_NAV);
                        updateNextionUI(true);
                    }
                    break;

                case 15: // Nextion USB-Bridge fuer Firmware-Upload vom PC auf das Display
                    strcpy(l1, "USB BRIDGE");
                    strcpy(l2, "ENTER = START");
                    if (enterPressed) {
                        // Der Nutzer bekommt vor dem blockierenden Bridge-Modus ein eindeutiges
                        // LCD-Feedback, damit das scheinbar "eingefrorene" Verhalten erwartet ist.
                        smartLCD("BRIDGE ACTIVE", "BACK = EXIT");
                        uiTriggerBeep(SND_OK);

                        // Dieser Mutex-Zugriff muss VOR dem Bridge-Einstieg wieder freigegeben
                        // werden, weil enterNextionUploadMode() absichtlich blockierend arbeitet.
                        // Wuerden wir gTimerMutex behalten, koennten andere Logik-Pfade unnötig
                        // am Timer-Mutex festhaengen, obwohl sie mit der Nextion-Bridge nichts
                        // zu tun haben.
                        xSemaphoreGive(gTimerMutex);

                        // Kurze Wartezeit, damit die Meldung garantiert sichtbar wird, bevor wir
                        // in die hochfrequente Bridge-Schleife wechseln.
                        delay(150);

                        // Der eigentliche Bridge-Modus blockiert absichtlich, bis irgendein
                        // Eingabe-Event in xInputQueue auftaucht. Danach kehren wir hierhin zurück.
                        enterNextionUploadMode();

                        // Nach dem Verlassen der Bridge zeigen wir einen kurzen Status an, damit
                        // der Ruecksprung in den normalen Setup-Betrieb sofort ersichtlich ist.
                        uiUpdateLCD("USB BRIDGE", "BRIDGE STOPPED");
                        setupMsgUntil = now + 600;
                        updateNextionUI(true);
                        return;
                    }
                    break;
            }
        }
        xSemaphoreGive(gTimerMutex);
    }

    // Anzeige auf dem RGB-LCD (Shadow Copy)
    uiUpdateLCD(l1, l2);
    updateNextionUI(true);
}