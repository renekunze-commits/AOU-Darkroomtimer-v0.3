/* Mode_Preflash.ino - Preflash / Flash Calibration Wizard

   Beschreibung:
   - Ermöglicht das Setzen einer Kurz-Preflash-Dauer für das aktuell aktive Papierprofil.
   - Bietet einen Test-Flash: Die eigentliche Testauslösung erfolgt non-blocking
     (Flash-Flag + flashEnd), jedoch nutzt `maybeDoPreflashBeforeExposure()` kurze
     blocking `delay()`-Aufrufe, um das Preflash korrekt vor einer Aufnahme auszuführen.

   Hinweis (Finding): Die `maybeDoPreflashBeforeExposure()`-Funktion verwendet
   kurze `delay()`-Aufrufe. Das ist bewusst und in der Praxis unkritisch, könnte
   aber in zukünftigen Versionen auf non-blocking-Timer umgestellt werden.
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Papers.h"
#include "DisplayManager.h"

extern ESP32Encoder encSoft;
extern ESP32Encoder encHard;
extern BtnState sEnter;
extern BtnState sBack;
extern void beepNav();
extern void beepValue();
extern void beepOk();
extern void smartLCD(const char* l1, const char* l2);
extern void saveSettings();
extern void PaintLED(int r, int g, int b);

// Lokale Helper
static long getEnc1DeltaLocal() {
  static long lastPos = 0;
  long newPos = encSoft.getCount() / 2;
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
} 

static long getEnc2DeltaLocal() {
  static long lastPos = 0;
  long newPos = encHard.getCount() / 2;
  long delta = newPos - lastPos;
  lastPos = newPos;
  return delta;
}

// Non-blocking Preflash state (used by maybeDoPreflashBeforeExposure)
static bool preflashActive = false;
static unsigned long preflashUntil = 0;
static int preflashPhase = 0; // 0=idle, 1=preflash on, 2=pause before exposure

// maybeDoPreflashBeforeExposure:
// - Führt (wenn konfiguriert) ein kurzes Preflash aus, ist aber NON-BLOCKING.
// - Muss periodisch aufgerufen werden (z.B. aus main loop / Timer), damit es voranschreitet.
void maybeDoPreflashBeforeExposure() {
  PaperProfile &PP = getActivePaper();
  unsigned long now = millis();

  // Wenn Preflash deaktiviert ist, Reset aller Stati und raus
  if (!(PP.flashEnable && PP.flashThreshS > 0.05)) {
    preflashActive = false;
    preflashPhase = 0;
    return;
  }

  // Startet das Preflash (einmalig)
  if (!preflashActive) {
    PaintLED(255, 255, 255);
    preflashActive = true;
    preflashPhase = 1;
    preflashUntil = now + (unsigned long)(PP.flashThreshS * 1000);
    return;
  }

  // Phase: Preflash beenden -> Pause vor eigentlicher Aufnahme
  if (preflashPhase == 1 && now >= preflashUntil) {
    PaintLED(0, 0, 0);
    preflashPhase = 2;
    preflashUntil = now + 300; // non-blocking Pause
    return;
  }

  // Phase: Pause beendet -> Preflash komplett
  if (preflashPhase == 2 && now >= preflashUntil) {
    preflashActive = false;
    preflashPhase = 0;
    return;
  }
}

// Hilfsfunktion: Status abfragen (ob Preflash noch läuft)
bool preflashBusy() {
  return preflashActive;
}

// HIER WURDE startTestStripMode() GELÖSCHT (War doppelt)

// Der "Flash Calibration Wizard"
void runFlashCalibWizard() {
  // Reset Encoders
  getEnc1DeltaLocal();
  getEnc2DeltaLocal();

  PaperProfile &PP = getActivePaper();

  // --- Parameter-Phase: Level, Color, Steps, dt ---
  int steps = 6;
  double dt = 0.05;
  uint8_t lvl = (PP.flashLevel >= 1 && PP.flashLevel <= 5) ? PP.flashLevel : 2;
  uint8_t col = (PP.flashColor <= 1) ? PP.flashColor : 0;
  bool paramLoop = true;
  while (paramLoop) {
    wdt_reset(); handleLights(); DM_loop();
    unsigned long now = millis();
    long d1 = getEnc1DeltaLocal();
    long d2 = getEnc2DeltaLocal();
    bool evEnter = updateButton(sEnter, PIN_SW_ENTER, now);
    bool evBack  = updateButton(sBack,  PIN_SW_BACK,  now);
    if (evBack) { lcd.clear(); return; }
    if (d1 > 0) { steps = constrain(steps + 1, 3, 10); beepValue(); }
    if (d1 < 0) { steps = constrain(steps - 1, 3, 10); beepValue(); }
    if (d2 > 0) { dt = min(0.25, dt + 0.01); beepValue(); }
    if (d2 < 0) { dt = max(0.01, dt - 0.01); beepValue(); }
    if (evEnter) { paramLoop = false; beepOk(); }
    String l1 = "ST:"+String(steps)+" dt:"+String(dt,2);
    String l2 = String("LV:")+String(lvl)+" "+(col==0?"WHT":"GRN")+" #=OK";
    smartLCD(l1, l2);
    delay(10);
  }

  // --- Testpulse-Phase ---
  for (int i=0; i<=steps; i++) {
    wdt_reset(); handleLights(); DM_loop();
    double tFlash = i * dt;
    tFlash = max(0.01, min(0.25, tFlash));
    String L0 = "STEP "+String(i)+"/"+String(steps);
    String L1 = "FLASH t="+String(tFlash,2)+"s";
    smartLCD(L0, L1);
    // Warte auf Start-Taste
    while (digitalRead(PIN_SW_ENTER) == HIGH) { wdt_reset(); delay(5); }
    beepOk();
    PaintLED(col==0?lvl:0, col==1?lvl:0, col==0?lvl:0); // White/Green
    unsigned long until = millis() + (unsigned long)(tFlash * 1000.0);
    while (millis() < until) { wdt_reset(); delay(1); }
    PaintLED(0,0,0);
    delay(80);
  }

  // --- Schwellenwert-Auswahl ---
  smartLCD("Develop strip", "Pick 1.."+String(steps)+" *=ABT");
  int pick = -1;
  while (pick == -1) {
    wdt_reset(); handleLights(); DM_loop();
    char k = getNextionKey();
    if (k == '*') { beepWarnLong(); return; }
    if (k >= '1' && k <= '9') {
      int v = k - '0';
      if (v >= 1 && v <= steps) pick = v;
    }
    delay(10);
  }
  int idx = max(0, pick - 1);
  double thresh = idx * dt;
  thresh = max(0.01, min(0.25, thresh));

  PP.flashCalibrated = true;
  PP.flashEnable = false;
  PP.flashLevel = lvl;
  PP.flashColor = col;
  PP.flashThreshS = thresh;
  PP.flashFactor = 1.00;
  markDirty(); saveSettings();
  beepWizardSave();
  smartLCD("THRESH SAVED", String(thresh,2)+"s");
  delay(800);
}