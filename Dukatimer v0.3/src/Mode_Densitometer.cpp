/* Mode_Densitometer.ino - Densitometer & Film-Test

   Beschreibung:
   - Misst relative Lichtintensitäten und berechnet optische Dichte D = log10(Ref / Meas).
   - Unterstützt einen REF-Calibration-Schritt (A=calibrate) und kontinuierliche Messung (B=meas).
   - Zeigt optional Netto-Dichte gegen eine zuvor gesetzte Base-Fog an.
   - Werte in Lux, D ist dimensionslos (optische Dichte, logarithmisch)
*/
#include <Arduino.h>
#include "Globals.h"
#include "Config.h"

// Externe Abhängigkeiten
extern void smartLCD(const char* l1, const char* l2);
extern void PaintLED(int r, int g, int b);
extern void beepOk();
extern void beepValue();
extern void beepWarnLong();
extern double takeAveragedLux(uint8_t samples, uint16_t delayMs);

// Helper
static Mode prevMode = MODE_SG;
static unsigned long densMsgUntil = 0;

void enterDensMode() {
  prevMode = currentMode;
  currentMode = MODE_DENS;
  densState = DENS_IDLE;
  densRefLux = 0.0;
  densBaseFog = NAN; // Reset Base Fog

  PaintLED(0, 0, 0);
  lcd.clear();
  lcd.print("DENSITOMETER");
  densMsgUntil = millis() + 800; // non-blocking
}

void exitDensMode() {
  currentMode = prevMode;
  PaintLED(0, 0, 0);
  lcd.clear();
  lcd.print("EXIT DENS...");
  densMsgUntil = millis() + 600; // non-blocking
}

void runDensitometerLoop(char key) {
  // Navigation
  if (key == '*') {
      if (densState == DENS_IDLE) { exitDensMode(); return; }
      else {
       densState = DENS_IDLE;
       isMeasuring = false;
       handleLights();
       updateNextionUI(true);
       beepWarnLong();
       lcd.clear();
      }
  }

  // If a short message is showing, don't proceed yet
  if (millis() < densMsgUntil) return;

  // --- HIER WAR DER FEHLER: PIN_SAFE_LIGHT -> PIN_SW_RED ---
  bool safeOn = (digitalRead(PIN_RELAY_SAFE) == HIGH);
  bool lightOn = (digitalRead(PIN_RELAY_ENLARGER) == HIGH);
  
  // Lichtsteuerung im Densitometer-Modus
  // Wir erlauben manuelles Licht zum Positionieren
  static int lastDensLight = -1;
  int targetL = 0;
  if (safeOn) targetL = 1;
  if (lightOn) targetL = 2;
  
  if (targetL != lastDensLight) {
    if (targetL == 0) PaintLED(0,0,0);
    else if (targetL == 1) PaintLED(set_safe > 0 ? set_safe : 255, 0, 0); // Rot
    else PaintLED(set_focus, set_focus, set_focus); // Weiß
    lastDensLight = targetL;
  }

  // State Machine
  switch (densState) {
    case DENS_IDLE: {
       smartLCD("DENSITOMETER", "A=CAL REF  B=MEAS");
       if (key == 'A') { 
         densState = DENS_REF; 
         beepOk(); 
         lcd.clear(); lcd.print("CALIBRATE REF"); 
         densMsgUntil = millis() + 600;
       }
       if (key == 'B') {
         if (densRefLux > 0.0001) {
           densState = DENS_MEAS;
           beepOk();
           lcd.clear();
         } else {
           beepWarnLong();
           smartLCD("NO REF DATA", "Please Calib (A)");
           densMsgUntil = millis() + 1000;
         }
       }
    } break;

    case DENS_REF: {
       smartLCD("1. LIGHT ON (W)", "2. PRESS # SET");
       // Messen und Anzeigen (Live)
       if (tslBaseOK) {
          uint32_t lum = tslBase.getFullLuminosity();
          uint16_t ir = lum >> 16;
          uint16_t full = lum & 0xFFFF;
          double lux = tslBase.calculateLux(full, ir);
          lcd.setCursor(12, 1); lcd.print(String(lux, 1));
       }
       
       if (key == '#') {
         isMeasuring = true;
         handleLights();
         lcd.setRGB(0, 0, 0);
         updateNextionUI(true);
         double avg = takeAveragedLux(10, 50);
         isMeasuring = false;
         handleLights();
          if (avg > 0.1) {
             densRefLux = avg;
             densState = DENS_IDLE;
             beepOk();
             smartLCD("REF SAVED", String(densRefLux, 2) + " lx");
             densMsgUntil = millis() + 1000;
          } else {
             beepWarnLong();
             smartLCD("TOO DARK", "Light on?");
             densMsgUntil = millis() + 800;
          }
       }
    } break;

    case DENS_MEAS: {
       if (!isMeasuring) {
         isMeasuring = true;
         handleLights();
         updateNextionUI(true);
       }
       lcd.setRGB(0, 0, 0);
       // Kontinuierliche Messung
       double lux = 0.0;
       if (tslBaseOK) {
          uint32_t lum = tslBase.getFullLuminosity();
          uint16_t ir = lum >> 16;
          uint16_t full = lum & 0xFFFF;
          lux = tslBase.calculateLux(full, ir);
       }
       
       String l1 = "L:" + String(lux, 2);
       String l2 = "";
       
       if (densRefLux > 0 && lux > 0) {
         double D = log10(densRefLux / lux);
         if (D < 0) D = 0.0;
         l1 += " D=" + String(D, 2);
         
         // Zonen-Logik (Optional)
         // Wenn BaseFog gesetzt ist, zeigen wir Netto-Dichte
         if (!isnan(densBaseFog)) {
            double net = D - densBaseFog;
            l2 = "Net:" + String(net, 2);
            // Zone 8 Check (ca 1.20 - 1.30 über Base)
            if (abs(net - zone8TargetNet) < 0.05) l2 += " *Z8*";
         } else {
            l2 = "#=SET BASE FOG";
         }
       } else {
         l1 += " --";
       }
       
       smartLCD(l1, l2);
       
       // Base Fog setzen
       if (key == '#') {
         if (densRefLux > 0 && lux > 0) {
            double D = log10(densRefLux / lux);
            densBaseFog = D;
            beepOk();
            smartLCD("BASE FOG SET", String(densBaseFog, 2));
            delay(800);
         }
       }
       
       // Reset Base Fog
       if (key == 'C') {
         densBaseFog = NAN;
         beepValue();
       }

    } break;
  }
}