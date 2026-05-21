#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>
#include "config.h"

// Display-Funktionen
void setupDisplay();
void updateDisplay();
void wakeDisplay();
void checkDisplaySleep();

// Neue Setter-Funktion für gekapselte Datenübergabe
void setDisplayData(const char* header, const char* line1, const char* line2, const uint8_t* histogram);

// Haupt-Output-Funktionen
void initOutput();
void showStartup();
void showError(const char* message);
void showStandby(float lux, uint32_t seq, bool connected);
void renderFromPacket(const ProbeRenderPacket& packet);

// Audio-Funktionen
void clickSound();

#endif