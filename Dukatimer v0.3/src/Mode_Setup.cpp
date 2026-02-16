/* Mode_Setup.ino - Encoder Version */
#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "DisplayManager.h"
#include "Logic_Papers.h"

/*
  Dieses Modul stellt das Setup-Menü bereit, das per zwei Dreh-Encoder
  (Navigation + Werte ändern) und zwei Tastern (Enter, Back) bedient wird.
  - Enc 1 (encSoft): Menü nach oben/unten
  - Enc 2 (encHard): Wert-Anpassung der ausgewählten Option
  - sEnter: Auswählen / Toggle
  - sBack: Zurück / Speichern

  Ziel: Kurz, nicht-blockierend und mit Rückmeldung (Piep/Tasten) arbeiten.
*/

// Externe Objekte (Hardware/Utilities aus anderen Modulen)
extern ESP32Encoder encSoft;
extern ESP32Encoder encHard;
extern BtnState sEnter;
extern BtnState sBack;
extern void beepNav();   // kurzes Navigations-Signal
extern void beepOk();    // Bestätigungs-Signal (nicht immer verwendet)
extern void beepValue(); // Signal für Wert-Änderung
extern void smartLCD(const char* l1, const char* l2); // Aktualisiert LCD/Overlay
extern void saveSettings(); // Persistiert Einstellungen
extern char getNextionKey();

// -----------------------------------------------------------------------------
// Hilfsfunktionen
// -----------------------------------------------------------------------------
// Beide Funktionen liefern die Differenz (Delta) der Encoder-Position seit
// dem letzten Aufruf zurück. So werden 'Schritte' des Benutzers erfasst.
// Division durch 2 glättet die Zählrate (abhängig vom Encoder-Setup).
long getEnc1Delta() {
  static long lastPos = 0;
  long newPos = encSoft.getCount() / 2;
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
}

long getEnc2Delta() {
  static long lastPos = 0;
  long newPos = encHard.getCount() / 2;
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
}

// -----------------------------------------------------------------------------
// Haupt-Handler für das Setup-Menü
// Diese Funktion läuft blockierend bis der Benutzer mit "Back" beendet.
// Innerhalb der Schleife wird regelmäßig Input gelesen und die Anzeige
// aktualisiert. Watchdog wird zurückgesetzt, um WDT-Resets während längerer
// Bedienung zu verhindern.
// -----------------------------------------------------------------------------
void handleSetup() {
  setupMenuActive = true;
  // Encoders kurz auf Null setzen (verhindert Sprünge beim Eintritt)
  getEnc1Delta();
  getEnc2Delta();

  int menuIdx = 0;                // aktuell markiertes Menü-Item
  const int NUM_ITEMS = 13;        // Anzahl Menüeinträge (erweitert)
  bool inMenu = true;

  while (inMenu) {
    // Watchdog füttern
    wdt_reset();
    handleLights();  // ← Safety: keep relays responsive
    DM_loop();       // ← Keep Nextion RX buffer drained

    // --- INPUT ---
    unsigned long now = millis();
    bool evEnter = updateButton(sEnter, PIN_SW_ENTER, now); // Taster-Events
    bool evBack  = updateButton(sBack,  PIN_SW_BACK,  now);
    char key = getNextionKey();
    if (key == '*') evBack = true;
    long dMenu   = getEnc1Delta(); // Encoder 1: Menü-Navigation
    long dVal    = getEnc2Delta(); // Encoder 2: Wertänderung

    // --- NAVIGATION (Encoder 1) ---
    // Rechts/Links oder Vor/Zurück am Encoder wechselt durch das Menü
    if (dMenu > 0) { menuIdx++; if (menuIdx >= NUM_ITEMS) menuIdx = 0; beepNav(); }
    if (dMenu < 0) { menuIdx--; if (menuIdx < 0) menuIdx = NUM_ITEMS - 1; beepNav(); }

    // --- EXIT mit Back-Taste ---
    if (evBack) {
      // Änderungen persistieren und kurzes Feedback
      saveSettings();
      lcd.clear(); lcd.print("SETTINGS SAVED");
      delay(800);
      setupMenuActive = false;
      DM_setNumber("btMNU", 0);
      triggerInfo();
      updateNextionUI(true);
      inMenu = false;
      return; // Zurück zum Hauptmenü
    }

    // --- MENÜ-ELEMENTE (anzeige / bearbeiten) ---
    // l1: Zeile 1 der Anzeige, l2: Zeile 2 (Wert / Hinweis)
    String l1 = "";
    String l2 = "";


    switch (menuIdx) {
      case 0: // STD TIME (Standard-Belichtungszeit für BW)
        l1 = "STD TIME BW";
        l2 = String(globalSet.std_time, 1) + " s";
        if (dVal != 0) {
          globalSet.std_time += (dVal > 0 ? 0.5 : -0.5);
          if (globalSet.std_time < 1.0) globalSet.std_time = 1.0;
          beepValue();
        }
        break;
      case 1: // SPLIT MODE
        l1 = "SPLIT MODE";
        l2 = (globalSet.splitMode == 0) ? "SEPARATE" : "AUTO FILL";
        if (dVal != 0 || evEnter) {
          globalSet.splitMode = !globalSet.splitMode;
          beepValue();
        }
        break;
      case 2: // LCD BACKLIGHT
        l1 = "LCD BRIGHT";
        l2 = String(set_lcd);
        if (dVal != 0) {
          int n = set_lcd + (dVal * 5);
          set_lcd = constrain(n, 10, 255);
          beepValue();
        }
        break;
      case 3: // FOCUS LIGHT
        l1 = "FOCUS BRIGHT";
        l2 = String(set_focus);
        if (dVal != 0) {
          int n = set_focus + (dVal * 5);
          set_focus = constrain(n, 10, 255);
          beepValue();
        }
        break;
      case 4: // SAFE LIGHT MAX
        l1 = "SAFE LIGHT MAX";
        l2 = String(set_max);
        if (dVal != 0) {
          int n = set_max + (dVal * 5);
          set_max = constrain(n, 10, 255);
          beepValue();
        }
        break;
      case 5: // PAPER FLASH ENABLE
        l1 = "FLASH GLOBAL";
        {
          bool flashOn = getActivePaper().flashEnable;
          l2 = flashOn ? "ON" : "OFF";
          if (dVal != 0 || evEnter) {
            getActivePaper().flashEnable = !flashOn;
            beepValue();
          }
        }
        break;
      case 6: // SOUND MODE
        l1 = "SOUND MODE";
        switch (globalSet.soundMode) {
          case 2: l2 = "OFF"; break;
          case 1: l2 = "QUIET"; break;
          default: l2 = "NORMAL"; break;
        }
        if (dVal != 0 || evEnter) {
          int mode = globalSet.soundMode + (dVal > 0 ? 1 : -1);
          if (mode > 2) mode = 0;
          if (mode < 0) mode = 2;
          globalSet.soundMode = mode;
          beepValue();
        }
        break;
      case 7: // FLASH LEVEL
        l1 = "FLASH LEVEL";
        l2 = String(getActivePaper().flashLevel);
        if (dVal != 0) {
          int lvl = getActivePaper().flashLevel + (dVal > 0 ? 1 : -1);
          if (lvl < 1) lvl = 1;
          if (lvl > 5) lvl = 5;
          getActivePaper().flashLevel = lvl;
          beepValue();
        }
        break;
      case 8: // FLASH COLOR
        l1 = "FLASH COLOR";
        l2 = (getActivePaper().flashColor == 1) ? "GREEN" : "WHITE";
        if (dVal != 0 || evEnter) {
          int col = getActivePaper().flashColor;
          col = (col == 0) ? 1 : 0;
          getActivePaper().flashColor = col;
          beepValue();
        }
        break;
      case 9: // TEACH PAPER (Kalibrierung)
        l1 = "TEACH PAPER";
        l2 = "Start Wizard";
        if (evEnter) {
          startCalibrationWizard();
          beepOk();
        }
        break;
      case 10: // TEST STRIP
        l1 = "TEST STRIP";
        l2 = "Start Mode";
        if (evEnter) {
          startTestStripMode();
          beepOk();
        }
        break;
      case 11: // ERROR RESET
        l1 = "ERROR RESET";
        l2 = (lastSystemError == ERR_NONE) ? "No Errors" : "Reset Errors?";
        if (evEnter && lastSystemError != ERR_NONE) {
          lastSystemError = ERR_NONE;
          clearErrorState();
          beepOk();
        }
        break;
      case 12: // SENSOR SOURCE
        l1 = "SENSOR SOURCE";
        l2 = useWirelessProbe ? "WIRELESS (W)" : "WIRED (L)";
        
        if (dVal != 0 || evEnter) {
             useWirelessProbe = !useWirelessProbe;
             beepValue();
             
             // Optional: Direkt Settings speichern, damit es nach Neustart bleibt
             // globalSet.useWireless = useWirelessProbe; // Müsste in SettingsObject ergänzt werden
        }
        break;
    }

    // Anzeige aktualisieren (Smart LCD: nutzt Overlay, Nextion-Queue etc.)
    smartLCD(l1, l2);

    // Live-Backlight-Anpassung anzeigen (nur für LCD Helligkeits-Item)
    if (menuIdx == 2) handleLCDBacklight();

    // Kurze Pause, um CPU zu schonen und Encoder-Entprellung zu helfen
    delay(10);
  }
}
