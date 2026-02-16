/*
   HW_Display.cpp - Optimierte Version für Dukatimer v0.3
   
   Funktionen:
   - Steuert Grove RGB LCD (I2C) mit Fehler-Farbcodierung
   - Steuert Nextion Display (UART) via DisplayManager
   - Anzeige von Raumtemperatur (BMP280) und LED-Temperatur (DS18B20)
   - Synchronisiert beide Displays (Overlay, Icons, Werte)
   - Behandelt Touch-Events gemäß "Entwicklung UI.txt"
*/

#include <Arduino.h>
#include <cstring>
#include "Globals.h"
#include "Config.h"
#include "DisplayManager.h"

extern void beepClick();

// =============================================================================
// KRITISCH: Direktes Starten/Stoppen des Timers über Nextion-Start-Button
// =============================================================================
// Der Start-Button (ID_MAIN_START) ruft jetzt direkt startTimer()/stopTimer auf,
// statt das Zeichen '#' in den Key-Buffer zu legen. Dadurch wird das Umschalten
// der EV-Schrittweite (handleIdleInput) nicht mehr fälschlich ausgelöst.
// Vorteil: Touch-UI verhält sich wie physischer Start-Taster.
extern unsigned long starttime;
extern void startTimer();
extern void stopTimer();

// =============================================================================
// KONFIGURATION: NEXTION IDs (Synchron mit Entwicklung UI.txt)
// =============================================================================
#define PAGE_MAIN   0

#define ID_MAIN_SW      NEXTION_BTN_SW_ID
#define ID_MAIN_SG      NEXTION_BTN_SG_ID
#define ID_MAIN_DENS    NEXTION_BTN_DENS_ID
#define ID_MAIN_HASH    NEXTION_BTN_HSH_ID
#define ID_MAIN_START   NEXTION_BTN_STR_ID
#define ID_MAIN_FOCUS   NEXTION_BTN_FOC_ID
#define ID_MAIN_SL      NEXTION_BTN_SL_ID
#define ID_MAIN_MENU    NEXTION_BTN_MNU_ID
#define ID_MAIN_SCREEN  NEXTION_BTN_SCR_ID

// =============================================================================
// GLOBALE & STATISCHE VARIABLEN
// =============================================================================
static char lastL1[17] = {0};
static char lastL2[17] = {0};
static unsigned long hashPressStart = 0;
static bool lastDimState = false; 
static char nextionKeyBuf = 0; 

char getNextionKey() { 
    char k = nextionKeyBuf; 
    nextionKeyBuf = 0; 
    return k; 
}

// =============================================================================
// INITIALISIERUNG
// =============================================================================

void initDisplays() {
  DM_init(); 
  if (lcdOK) {
    lcd.setRGB(255, 255, 255);
    lcd.setCursor(0, 0); 
    lcd.print("Dukatimer v0.3");
  }
  // Kurze Pause für Nextion Boot-Phase
  delay(500);
}

// =============================================================================
// LCD LOGIK (Inkl. Schalter 13 / Button 9 Logik)
// =============================================================================

void smartLCD(const char* l1, const char* l2) {
  if (!lcdOK) return;

  // --- 1. FARB-LOGIK & DUNKELKAMMER-MODUS ---
  bool isScreenOff = (digitalRead(PIN_SW_ROOMLIGHT) == LOW) || screenOffOverride; 
  char buf1[17], buf2[17];
  const char *display1 = l1, *display2 = l2;

  if (isScreenOff) {
    lcd.setRGB(0, 0, 0); // LCD Backlight AUS
  }
  else if (softAbortActive && (millis() < softAbortUntilMs)) {
    lcd.setRGB(255, 0, 0); // Alarm Rot bei Sensorverlust
    display1 = "SOFT ABORT";
    display2 = "SENSOR LOST";
  } 
  else if (lastSystemError != ERR_NONE) {
    lcd.setRGB(255, 0, 0); // Fehler Rot
  } 
  else if (starttime != 0 || isMeasuring) {
    lcd.setRGB(0, 0, 0); // Während Belichtung aus (Dunkelkammer-Schutz)
  } 
  else if ((digitalRead(PIN_SW_SAFE) == LOW) || safeLatch) {
    lcd.setRGB(200, 0, 0); // Rotlicht Modus
  } 
  else {
    lcd.setRGB(255, 0, 0); // Standby Rot
  }

  // --- 2. HARDWARE UPDATE (16 Zeichen formatieren) ---
  // Use fixed buffers instead of String concatenation
  snprintf(buf1, sizeof(buf1), "%-16s", display1 ? display1 : "");
  snprintf(buf2, sizeof(buf2), "%-16s", display2 ? display2 : "");
  
  if (strcmp(lastL1, buf1) != 0) { 
    lcd.setCursor(0, 0); lcd.print(buf1); 
    strncpy(lastL1, buf1, 16);
  }
  if (strcmp(lastL2, buf2) != 0) { 
    lcd.setCursor(0, 1); lcd.print(buf2); 
    strncpy(lastL2, buf2, 16);
  }
}

void smartLCD(const String& l1, const String& l2) {
  smartLCD(l1.c_str(), l2.c_str());
}

void handleLCDBacklight() {
  if (!lcdOK) return;

  bool isScreenOff = (digitalRead(PIN_SW_ROOMLIGHT) == LOW) || screenOffOverride;
  if (isScreenOff) {
    lcd.setRGB(0, 0, 0);
    return;
  }

  uint8_t v = set_lcd;
  lcd.setRGB(v, v, v);
}

void triggerInfo() {
  String L1 = "";
  String L2 = "";
  
  String stepText = stepNames[(int)globalStepMode]; 

  if (currentMode == MODE_BW) {
    L1 = "BW Mode";
    String evLabel = "EV " + stepText;
    while (L1.length() + evLabel.length() < 16) L1 += " ";
    L1 += evLabel;
    L2 = "T:" + String(time_bw, 1) + "s G:" + String(grade_bw, 1);
  } 
  else {
    L1 = "S:" + String(time_soft, 1) + "s";
    while (L1.length() + stepText.length() < 16) L1 += " ";
    L1 += stepText;
    L2 = "H:" + String(time_hard, 1) + "s";
  }

  smartLCD(L1, L2);
  updateNextionUI(false);
}

// =============================================================================
// NEXTION UI UPDATE
// =============================================================================

void updateNextionUI(bool force) {
  // Statische Speicher für den letzten gesendeten Zustand
  static char lastTimeStr[32] = {0};
  static char lastPowStr[32] = {0};
  static char lastTempRStr[32] = {0};
  static char lastTempAStr[32] = {0};
  static int lastBtScr = -1;
  static int lastBtMnu = -1;
  static int lastBtSw = -1;
  static int lastBtSg = -1;
  static int lastBtSl = -1;
  static int lastBtFoc = -1;
  static int lastT2Visible = -1;

  // --- 1. ZEIT ANZEIGE ---
  char buf[32];
  if (currentMode == MODE_BW) {
    snprintf(buf, sizeof(buf), "%.1fs", time_bw);
  } else {
    snprintf(buf, sizeof(buf), "%.1f/%.1fs", time_soft, time_hard);
  }
  
  // Nur senden, wenn geändert oder force
  if (force || strcmp(buf, lastTimeStr) != 0) {
    DM_setText("tTime", buf);
    strncpy(lastTimeStr, buf, sizeof(lastTimeStr) - 1);
    lastTimeStr[sizeof(lastTimeStr) - 1] = '\0';
  }

  // --- 2. LEISTUNG ---
  snprintf(buf, sizeof(buf), "%d", set_max);
  if (force || strcmp(buf, lastPowStr) != 0) {
    DM_setText("tPow", buf);
    strncpy(lastPowStr, buf, sizeof(lastPowStr) - 1);
    lastPowStr[sizeof(lastPowStr) - 1] = '\0';
  }
  
  // --- 3. TEMPERATUR (Raum) ---
  if (bmpOK && bmpPtr != nullptr) {
    snprintf(buf, sizeof(buf), "%.1f C", tempRoom); // Nutzt die globale Variable aus main loop
    if (force || strcmp(buf, lastTempRStr) != 0) {
      DM_setText("tTempR", buf);
      strncpy(lastTempRStr, buf, sizeof(lastTempRStr) - 1);
      lastTempRStr[sizeof(lastTempRStr) - 1] = '\0';
    }
  }

  // --- 4. TEMPERATUR (Alu) ---
  if (tempSensorOK) {
    snprintf(buf, sizeof(buf), "%.1f C", tempAlu);
    if (force || strcmp(buf, lastTempAStr) != 0) {
      DM_setText("tTempA", buf);
      strncpy(lastTempAStr, buf, sizeof(lastTempAStr) - 1);
      lastTempAStr[sizeof(lastTempAStr) - 1] = '\0';
    }
  }

  // --- 5. BUTTONS & ICONS ---
  bool isScreenOff = (digitalRead(PIN_SW_ROOMLIGHT) == LOW) || screenOffOverride;
  bool screenOffState = isScreenOff || isMeasuring;
  int valScr = screenOffState ? 1 : 0;
  
  if (force || valScr != lastBtScr) {
    DM_setNumber("btSCR", valScr);
    // Dimmen direkt hier handhaben
    DM_sendCommand(screenOffState ? "dim=5" : "dim=100");
    lastBtScr = valScr;
    lastDimState = screenOffState; // Sync für globale Variable
  }

  int valMnu = setupMenuActive ? 1 : 0;
  if (force || valMnu != lastBtMnu) {
    DM_setNumber("btMNU", valMnu);
    lastBtMnu = valMnu;
  }

  bool wirelessFresh = (millis() - lastRemotePacketMs) < 2000;
  bool sensorConnected = useWirelessProbe ? wirelessFresh : tslBaseOK;
  int valT2 = sensorConnected ? 1 : 0;
  if (force || valT2 != lastT2Visible) {
    DM_setVisible("t2", sensorConnected);
    DM_setText("t2", sensorConnected ? "WIFI" : "---");
    lastT2Visible = valT2;
  }

  // Modus Buttons
  int valSw = (currentMode == MODE_BW ? 1 : 0);
  if (force || valSw != lastBtSw) {
    DM_setNumber("btSW", valSw);
    lastBtSw = valSw;
  }

  int valSg = (currentMode == MODE_SG ? 1 : 0);
  if (force || valSg != lastBtSg) {
    DM_setNumber("btSG", valSg);
    lastBtSg = valSg;
  }
  
  // Safe Light Status (basiert jetzt auf der synchronisierten Variable)
  int valSl = safeLatch ? 1 : 0;
  if (force || valSl != lastBtSl) {
    DM_setNumber("btSL", valSl);
    lastBtSl = valSl;
  }
  
  // Focus Light Status
  int valFoc = whiteLatch ? 1 : 0;
  if (force || valFoc != lastBtFoc) {
    DM_setNumber("btFOC", valFoc);
    lastBtFoc = valFoc;
  }
}
// =============================================================================
// TOUCH HANDLER
// =============================================================================

void handleNextionTouch(uint8_t page, uint8_t id, uint8_t event) {
    extern void handleSetup();
    // Menü öffnen (ID15)
    if (id == ID_MAIN_MENU && event == 0) {
      beepClick();
      if (setupMenuActive) {
        nextionKeyBuf = '*';
      } else {
        handleSetup();
      }
      return;
    }
  if (page != PAGE_MAIN) return;

  switch(id) {
    case ID_MAIN_SW: 
      if (event == 0) { beepClick(); currentMode = MODE_BW; triggerInfo(); }
      break;
      
    case ID_MAIN_SG: 
      if (event == 0) { beepClick(); currentMode = MODE_SG; triggerInfo(); }
      break;

    case ID_MAIN_DENS:
      if (event == 0) { beepClick(); enterDensMode(); }
      break;
      
    case ID_MAIN_START:
      if (event == 0) {
        beepClick();
        // Direktes Starten/Stoppen des Timers
        if (starttime == 0) startTimer();
        else stopTimer();
      }
      break;
      
    case ID_MAIN_SL: 
      if (event == 0) { 
        beepClick();
        safeLatch = !safeLatch; 
        if (safeLatch) whiteLatch = false; 
      } 
      break;
      
    case ID_MAIN_FOCUS: 
      if (event == 0) {
        beepClick();
        if (!safeLatch && digitalRead(PIN_SW_SAFE) == HIGH) {
           whiteLatch = !whiteLatch;
        }
      } 
      break;
      
    case ID_MAIN_SCREEN: 
      if (event == 0) { beepClick(); screenOffOverride = !screenOffOverride; }
      break;
      
    case ID_MAIN_HASH: 
      if (event == 1) hashPressStart = millis(); 
      else if (event == 0) {
        if (millis() - hashPressStart > 2000) {
          defaultsSettings();
        } else {
          beepClick();
          nextionKeyBuf = '*';
        }
      }
      break;
  }
  // NOTE: Do NOT call handleLights() or triggerInfo() here - they are called every main loop iteration
  // Calling them here causes redundant execution and potential static cache inconsistency in updateNextionUI()
}

void refreshPage(uint8_t pageId) {
  if (pageId == PAGE_MAIN) triggerInfo();
}