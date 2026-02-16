/* Mode_TestStrip.ino - Teststreifen-Modus (Non-blocking)

   Beschreibung:
   - Berechnet eine Serie von Belichtungszeiten (multigrade/ev steps)
   - Startet jede Belichtung nicht-blockierend: setzt `tsExposureActive` und
     `tsExposureEnd` statt `delay()`.
   - UI: Encoder 1 ändert Basiszeit, Encoder 2 setzt EV-Schrittweite; Enter startet
     den aktuellen Streifen.
*/
#include <Arduino.h>
#include "Globals.h"
#include "Config.h"

// Externe
extern ESP32Encoder encSoft;
extern ESP32Encoder encHard;
extern BtnState sEnter;
extern BtnState sBack;

extern void beepNav();
extern void beepValue();
extern void beepOk();
extern void beepStartPattern();
extern void beepEndPattern();
extern void smartLCD(const char* l1, const char* l2);
extern void PaintLED(int r, int g, int b);
extern long secondsToUnits(double s, int tgl);

// Helper
static long getEnc1TS() {
  static long lastPos = 0;
  long newPos = encSoft.getCount() / 2;
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
}

static long getEnc2TS() {
  static long lastPos = 0;
  long newPos = encHard.getCount() / 2;
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
}

// Teststrip: nicht-blockierende Exposure-Logik
static bool tsExposureActive = false;
static unsigned long tsExposureEnd = 0;
static unsigned long tsMsgUntil = 0;

// startTestStripMode:
// - Initialisiert den Teststreifen-Modus: setzt Kanal (BW/Soft/Hard), Anzahl und EV-Schritt
// - Setzt eine kurze, nicht-blockierende Pause damit die UI den Modus anzeigen kann
void startTestStripMode() {
  // Init wie Legacy: Kanalwahl aus Kontext
  if (currentMode == MODE_BW) tsCh = TS_BW;
  else tsCh = TS_SOFT; // Default im SG Modus

  ts = TS_SETUP;
  tsN = 6; // Default wie Legacy
  tsEv = 1.0/3.0; // Default 1/3 EV

  // Encoders nullen
  getEnc1TS();
  getEnc2TS();

  lcd.clear(); lcd.print("TEST STRIP MODE");
  tsMsgUntil = millis() + 500;
  tsK = 0; tsSum = 0.0;
  tsExposureActive = false;
}

void runTestStripLoop(char key) {
  unsigned long now = millis();

  // Input
  bool evEnter = updateButton(sEnter, PIN_SW_ENTER, now);
  bool evBack  = updateButton(sBack,  PIN_SW_BACK,  now);
  long dTime   = getEnc1TS();
  long dSteps  = getEnc2TS();

  if (now < tsMsgUntil) return;

  // Exposure Ablauf
  if (tsExposureActive) {
    if (now >= tsExposureEnd) {
      PaintLED(0,0,0);
      beepEndPattern();
      tsExposureActive = false;
      tsK++;
      tsMsgUntil = now + 200;
    }
    return;
  }

  // Abbruch
  if (evBack) {
    ts = TS_OFF;
    PaintLED(0,0,0);
    lcd.clear(); lcd.print("TEST STRIP END"); tsMsgUntil = now + 500; return;
  }

  // --- STATE MACHINE ---
  if (ts == TS_SETUP) {
    // Setup-UI: EV-Raster und Streifenanzahl wie Legacy
    String l1 = "N:" + String(tsN) + "  dEV:" + String(tsEv, 2);
    String l2 = "T:" + String((currentMode == MODE_BW) ? time_bw : (tsCh == TS_HARD ? time_hard : time_soft), 1) + "s #=GO";

    // Encoder 1: Zeit ändern
    double *baseT = (currentMode == MODE_BW) ? &time_bw : (tsCh == TS_HARD ? &time_hard : &time_soft);
    if (dTime != 0) {
      double fac = (dTime > 0) ? 1.1 : 0.9;
      *baseT *= fac;
      if (*baseT < 0.5) *baseT = 0.5;
      beepNav();
    }

    // Encoder 2: EV Schritte ändern (1/6, 1/3, 1)
    if (dSteps != 0) {
      if (tsEv < 0.18) tsEv = 1.0/3.0;
      else if (tsEv < 0.34) tsEv = 1.0;
      else tsEv = 1.0/6.0;
      beepValue();
    }

    // Streifenanzahl per langem Druck auf sEnter (4, 6, 8, 10)
    static unsigned long enterPressStart = 0;
    static bool enterLongHandled = false;
    bool enterRaw = digitalRead(PIN_SW_ENTER) == LOW;
    if (enterRaw && enterPressStart == 0) enterPressStart = now;
    if (!enterRaw && enterPressStart != 0) {
      if (!enterLongHandled && (now - enterPressStart >= 800)) {
        // Zyklisch durch 4, 6, 8, 10
        int nextN = tsN;
        if (tsN == 4) nextN = 6;
        else if (tsN == 6) nextN = 8;
        else if (tsN == 8) nextN = 10;
        else nextN = 4;
        tsN = nextN;
        beepValue();
        lcd.clear(); lcd.print("STRIPS: " + String(tsN)); tsMsgUntil = now + 500;
        enterLongHandled = true;
      }
      enterPressStart = 0;
      enterLongHandled = false;
    }
    if (enterRaw && (now - enterPressStart >= 800)) enterLongHandled = false;

    // START
    if (evEnter) {
      // Zeiten berechnen wie Legacy (additiv, Center-Anchor)
      int center = tsN / 2;
      double *refT = (currentMode == MODE_BW) ? &time_bw : (tsCh == TS_HARD ? &time_hard : &time_soft);
      for(int i=0; i<tsN; i++) {
        int diff = i - center;
        tsA[i] = (*refT) * pow(2.0, diff * tsEv);
        if (tsA[i] < 0.10) tsA[i] = 0.10;
      }
      ts = TS_RUNNING;
      tsK = 0;
      tsSum = 0.0;
      beepOk();
      lcd.clear(); lcd.print("READY TO START"); tsMsgUntil = now + 500;
    }
    smartLCD(l1, l2);
  }
  else if (ts == TS_RUNNING) {
    if (tsK >= tsN) {
      ts = TS_SETUP;
      beepEndPattern();
      lcd.clear(); lcd.print("TEST STRIP DONE"); tsMsgUntil = now + 500; return;
    }
    double tExp = tsA[tsK];
    String l1 = "STRIP " + String(tsK+1) + "/" + String(tsN);
    String l2 = "EXP: " + String(tExp, 1) + "s #=GO";
    smartLCD(l1, l2);
    if (evEnter) {
      int r=0, g=0, b=0;
      if (currentMode == MODE_BW) { r=120; g=120; b=120; }
      else if (tsCh == TS_SOFT)   { r=0; g=255; b=0; }
      else                        { r=0; g=0; b=255; }
      PaintLED(r,g,b);
      beepStartPattern();
      unsigned long duration = (unsigned long)(tExp * 1000);
      tsExposureEnd = now + duration;
      tsExposureActive = true;
      tsSum += tExp;
    }
  }
}