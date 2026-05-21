/* Mode_BootDiag.cpp - v0.5 Root Cause Edition
   
   Zweck: System-Monitor & Pre-Flight Check auf dem NEXTION Display.
   Fängt alle Input-Events ab, bis der Nutzer START drückt.
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "UI_Map.h"
#include "DisplayManager.h"

static unsigned long lastNextionUpdate = 0;
static bool pageInitSent = false;
static unsigned long bootDiagStartedAt = 0;

static void leaveBootDiagnostics() {
    // Beim Verlassen muessen die Boot-Statiken sauber zurueckgesetzt werden,
    // damit ein spaeterer Wiedereintritt deterministisch wieder bei Null startet.
    bootScreenActive = false;
    pageInitSent = false;
    bootDiagStartedAt = 0;

    uiTriggerBeep(SND_OK);
    triggerInfo();

    // Wenn das Nextion spaeter doch verfuegbar ist, synchronisieren wir es beim
    // Uebergang in den Normalbetrieb sofort auf die Hauptseite.
    extern void updateNextionUI(bool force);
    updateNextionUI(true);
}

void runBootDiagnostics() {
    wdt_reset();
    unsigned long now = millis();

    // Der Boot-Diagnosemodus darf den normalen Betrieb nicht dauerhaft blockieren,
    // nur weil das Nextion noch leer, defekt oder absichtlich nicht angeschlossen ist.
    // Deshalb merken wir uns den Eintrittszeitpunkt und erlauben nach kurzer Anzeige
    // einen automatischen Uebergang in den LCD-Only-Betrieb.
    if (bootDiagStartedAt == 0) {
        bootDiagStartedAt = now;
    }

    // 1. Zwinge das Nextion exakt 1x auf das Dashboard (Page 4)
    if (!pageInitSent) {
        DM_sendCommand("page " PAGE_BOOT);
        pageInitSent = true;
    }

    InputEvent evt;
    bool exitRequested = false;

    // 2. Zentrale Wahrnehmung (Blockiert alles andere)
    while (xQueueReceive(xInputQueue, &evt, 0) == pdTRUE) {
        if (evt.type == EVT_START_PRESSED || evt.type == EVT_ENTER_PRESSED || evt.type == EVT_BACK_PRESSED) {
            exitRequested = true;
        }
    }

    // LCD-Only-Anforderung:
    // Selbst ohne funktionierendes Nextion muss das Geraet nach kurzer Voranzeige
    // alleine ueber das 16x2-LCD benutzbar sein. Der Diagnosebildschirm ist daher
    // nur noch ein kurzer Pre-Flight, kein harter Stopp mehr.
    const bool autoExitRequested = (now - bootDiagStartedAt) >= 4000;

    // 3. Abbruch / Start in den Normalbetrieb
    if (exitRequested || autoExitRequested) {
        leaveBootDiagnostics();
        return;
    }

    // 4. LCD Minimal-Anzeige
    // Der Text verweist bewusst NICHT mehr auf das Nextion, weil der Benutzer das
    // System auch ohne Nextion-Display starten und bedienen koennen muss.
    uiUpdateLCD("SYSTEM DIAG", "START=WEITER");

    // 5. Live-Update zum Nextion (Gedrosselt auf 5 Hz für UART-Stabilität)
    if (now - lastNextionUpdate > 200) {
        char buf[64];

        // Sensoren Status (inkl. Auto-Detect für BME/BMP)
        snprintf(buf, sizeof(buf), "Base:%s Head:%s %s:%s Alu:%s", 
                 tslBaseOK ? "OK" : "ERR", 
                 tslHeadOK ? "OK" : "ERR", 
                 isBME280 ? "BME" : "BMP",
                 bmpOK ? "OK" : "ERR", 
                 tempSensorOK ? "OK" : "ERR");
        DM_setText(TXT_BOOT_SENS, buf);

        // Umwelt (Nutzt Feuchtigkeit nur, wenn wirklich ein BME280 verlötet ist)
        if (isBME280) {
            snprintf(buf, sizeof(buf), "%.1f C | %.0f hPa | Hum: %.1f %%", tempRoom, pressRoom, humRoom);
        } else {
            snprintf(buf, sizeof(buf), "%.1f C | %.0f hPa | Hum: N/A", tempRoom, pressRoom);
        }
        DM_setText(TXT_BOOT_ENV, buf);

        // Hardware-Schalter
        bool swFoc = (digitalRead(PIN_SW_FOCUS) == LOW);
        bool swSaf = (digitalRead(PIN_SW_SAFE) == LOW);
        bool swRoo = (digitalRead(PIN_SW_ROOMLIGHT) == LOW);
        bool swDos = hwSwitchDoseMode; 
        snprintf(buf, sizeof(buf), "Foc:%d Saf:%d Roo:%d Dos:%d", swFoc, swSaf, swRoo, swDos);
        DM_setText(TXT_BOOT_SW, buf);

        // Relais & NeoPixel
        snprintf(buf, sizeof(buf), "Roo:%s Saf:%s | PWM F:%d S:%d", 
                 isRoomDarknessActive ? "ON" : "OFF", 
                 statusSafeOn ? "ON" : "OFF", 
                 set_focus, set_safe);
        DM_setText(TXT_BOOT_RELAY, buf);

        // Wireless C6 Handgerät
        if (probeConnected) {
            snprintf(buf, sizeof(buf), "Conn: YES | Lux: %.3f", (double)probeLuxG0);
        } else {
            snprintf(buf, sizeof(buf), "Conn: NO | Lux: ---");
        }
        DM_setText(TXT_BOOT_C6, buf);

        lastNextionUpdate = now;
    }
}