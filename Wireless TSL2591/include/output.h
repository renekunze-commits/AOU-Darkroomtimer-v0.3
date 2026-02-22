#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>
#include "config.h"

// Initialisiert LCD Display und Buzzer
void initOutput();

// Zeigt den Startbildschirm an
void showStartup();

// Zeigt eine Fehlermeldung auf dem Display an
void showError(const char* msg);

// Rendert den vom S3 empfangenen Render-Befehl auf dem LCD
void renderFromPacket(const ProbeRenderPacket& pkt);

// Zeigt den Standby-Bildschirm (kein aktiver Modus)
void showStandby(float lastLux, uint32_t seq, bool txOK);

// Sound-Hilfsfunktionen
void beep(int freq, int duration);
void clickSound();

// Haptisches Feedback gemäß ProbeHaptic-Enum
void playHaptic(uint8_t hapticCode);

#endif