#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>

void initInput();

// Encoder Navigation (PCNT Hardware Counter)
long getEncoderValue();

// Taster mit Continuous Debounce (50ms, Flankenauswertung)
// Gibt true zurück bei frischer Druck-Flanke (einmalig pro Tastendruck)
bool isT1Pressed();     // Taster 1 (oben):  UNDO / ZURÜCK / REFERENZ
bool isT2Pressed();     // Taster 2 (unten): MESSEN / FEUER
bool isEncPressed();    // Encoder Klick:    ENTER / ABSCHLIESSEN

#endif