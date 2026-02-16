#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>

void initInput();
long getEncoderValue();
bool isButtonPressed(); // Gibt true zurück, wenn frisch gedrückt (Entprellt)

#endif