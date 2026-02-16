/* HW_Sound.ino - Buzzer / Piep-Funktionen

   Verantwortlichkeiten:
   - Erzeugt kurze akustische Rückmeldungen (Navigation, OK, Warnungen)
   - Unterstützt einfache Patterns (mehrere Segmente)
   - Verhindert UI-Beeps während aktiver Belichtung (configurable)

   Hinweis:
   - Die Funktion `beepPattern` ist intern blocking während des Abspielens
     (kleine Delays zum Tönen). Das ist akzeptabel für kurze Signale,
     aber vermeide lange Patterns während zeitkritischer Aktionen.
*/
#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
//#include <avr/wdt.h>

static unsigned long beepUntil = 0;

// allowUiBeeps: verhindert UI-Signale während aktiver Belichtung
bool allowUiBeeps() { 
  return !(starttime == 1 && !isPaused);
} 

void beepPattern(const BeepSeg* segs, size_t n) {
  if (globalSet.soundMode == SOUND_OFF) return;
  double scale = (globalSet.soundMode == SOUND_QUIET) ? SOUND_QUIET_SCALE : 1.0;
  
  bool hasPrio = false; 
  for (size_t i = 0; i < n; i++) { if (segs[i].prio) { hasPrio = true; break; } }
  
  if (hasPrio) beepUntil = 0;
  if (millis() < beepUntil) return;

  for (size_t i = 0; i < n; i++) {
    wdt_reset(); 
    int dur = (int)max(20, (int)(segs[i].d * scale));
    int pau = (int)max(0,  (int)(segs[i].p * scale));
    tone(PIN_BUZZER, segs[i].f, dur);
    
    unsigned long tEnd = millis() + dur;
    while (millis() < tEnd) { wdt_reset(); delay(1); }
    
    if (pau > 0) {
      unsigned long pEnd = millis() + pau;
      while (millis() < pEnd) { wdt_reset(); delay(1); }
    }
  }
  if (n > 0) beepUntil = millis() + 30;
}

void beepNav() { 
  if (!allowUiBeeps()) return; 
  BeepSeg s[] = { { 800, 60, 0, false } }; beepPattern(s, 1);
}
void beepValue() { 
  if (!allowUiBeeps()) return; 
  BeepSeg s[] = { { 900, 65, 0, false } }; beepPattern(s, 1);
}
void beepClick() {
  if (!allowUiBeeps()) return;
  BeepSeg s[] = { { 2600, 18, 0, false } };
  beepPattern(s, 1);
}
void beepOk() { 
  BeepSeg s[] = { { 2200, 60, 0, false } }; beepPattern(s, 1);
}
void beepHint() { 
  BeepSeg s[] = { { 500, 140, 0, true } }; beepPattern(s, 1);
}
void beepWarnLong() { 
  BeepSeg s[] = { { 400, 320, 0, true } }; beepPattern(s, 1); 
}
void beepWizardStep() { 
  BeepSeg s[] = { { 1700, 50, 40, false }, { 2000, 60, 0,  false } }; 
  beepPattern(s, 2);
}
void beepWizardSave() { 
  BeepSeg s[] = { { 2000, 50, 40, false }, { 2400, 70, 0,  false } }; 
  beepPattern(s, 2); 
}
void beepStartPattern() {
  BeepSeg s[] = { { 1100, 80, 60, true }, { 1400, 80, 0,  true } }; 
  beepPattern(s, 2);
}
void beepEndPattern() { 
  BeepSeg s[] = { { 1400, 120, 30, true }, { 1700, 220, 0,  true } }; 
  beepPattern(s, 2); 
}
void beepDone() {
  BeepSeg s[] = { { 1800, 150, 50, true }, { 2200, 200, 0, true } };
  beepPattern(s, 2);
}
void beepAlarm() { 
  BeepSeg s[] = { { 3000, 1000, 0, true } }; beepPattern(s, 1);
}