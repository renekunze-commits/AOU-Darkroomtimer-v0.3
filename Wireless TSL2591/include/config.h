#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- CLEAN EDGE HARDWARE LAYOUT (ESP32-C6-Zero) ---
// VERBOTENE PINS (Rückseiten-Pads): 6, 7, 8, 9, 12, 13, 23

// Rechte Flanke: I2C Bus & Outputs
const int PIN_I2C_SDA = 22; // Shared by OLED and TSL2591
const int PIN_I2C_SCL = 21;
const int PIN_VIB_MOTOR = 20;

// Linke Flanke: Inputs (Encoder & 2x Taster)
const int PIN_ENC_CLK = 0;
const int PIN_ENC_DT  = 1;
const int PIN_ENC_SW  = 2;
const int PIN_BTN_1   = 3;  // Taster 1 (z.B. Messen)
const int PIN_BTN_2   = 4;  // Taster 2 (z.B. Moduswechsel / Lichter-Schatten Toggle)

// Sensor-Interrupt (separat vom Bedienblock)
const int PIN_TSL_INT = 5;

// --- SYSTEM SETTINGS ---
constexpr uint32_t DISPLAY_UPDATE_MS = 250;
constexpr uint32_t DISPLAY_SLEEP_TIMEOUT = 30000;
constexpr uint32_t REMOTE_SEND_INTERVAL_MS = 50;
constexpr uint32_t REMOTE_KEEPALIVE_MS = 1000;
constexpr uint32_t REMOTE_SLAVE_TIMEOUT_MS = 3000;

// Platzhalter: durch die MAC des Hauptgeraete-ESP ersetzen, sobald bekannt.
constexpr uint8_t REMOTE_RECEIVER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#endif