#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>

void initInput();

int16_t getAndClearEncoderDelta();
void restoreEncoderDelta(int16_t delta);
uint8_t getActiveButtonMask();
float getLuxValue();
uint32_t getLuxSampleAgeMs(uint32_t nowMs = 0);
void publishLuxValue(float lux, uint32_t sampleTimestampMs = 0);

// Taster mit Continuous Debounce (50ms, Flankenauswertung)
// Gibt true zurück bei frischer Druck-Flanke (einmalig pro Tastendruck)
bool isT1Pressed();     // Taster 1 (oben):  UNDO / ZURÜCK / REFERENZ
bool isT2Pressed();     // Taster 2 (unten): MESSEN / FEUER
bool isEncPressed();    // Encoder Klick:    ENTER / ABSCHLIESSEN
bool isEncLongPressed(); // Encoder Langdruck: Messmodus starten (Enc3 lang)

#endif