#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- I2C PINS (Bus 0: Sensor & OLED Links) ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- SENSOR PINS ---
#define PIN_TSL_INT 4

// --- I2C PINS (Bus 1: OLED Rechts) ---
#define I2C_SDA_OLED 16
#define I2C_SCL_OLED 17

// --- ENCODER PINS ---
#define ENC_A   32
#define ENC_B   33
#define ENC_BTN 25

// --- BUZZER PIN ---
#define PIN_BUZZER 26

// --- DISPLAY SETTINGS (Provisorische kleine OLEDs) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C 

// --- SYSTEM SETTINGS ---
#define REMOTE_MAGIC 0xD4
#define SEND_INTERVAL_MS 200 

// --- DATENSTRUKTUR ---
struct WirelessPacket {
    uint8_t magic;
    uint32_t seq;
    float lux;
    int encoderVal; 
};

#endif