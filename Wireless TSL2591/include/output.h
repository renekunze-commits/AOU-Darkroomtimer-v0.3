#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>

// Initialisiert das OLED-Display (auf dem zweiten I2C Bus) und den Buzzer
void initOutput();

// Zeigt den Startbildschirm an
void showStartup();

// Zeigt eine Fehlermeldung auf dem Display an
void showError(const char* msg);

// Aktualisiert die Hauptanzeige mit den aktuellen Mess- und Statuswerten
void updateDisplay(float lux, long encVal, uint32_t seq, bool txOK);

// Sound-Hilfsfunktionen für den Piezo-Buzzer
void beep(int freq, int duration);
void clickSound();

#endif