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
#define REMOTE_MAGIC        0xD4
#define HEARTBEAT_INTERVAL  1000
#define DISPLAY_UPDATE_MS   250
#define DISPLAY_SLEEP_TIMEOUT 30000  // 30 Sekunden

// =============================================================================
// ESP-NOW WIRELESS PROTOCOL
// =============================================================================
enum ProbeEventType : uint8_t {
    EVT_NONE        = 0x00,
    EVT_T2_CLICK    = 0x01,
    EVT_T1_CLICK    = 0x02,
    EVT_ENC_CLICK   = 0x03,
    EVT_ENC_UP      = 0x04,
    EVT_ENC_DOWN    = 0x05,
    EVT_LUX_DATA    = 0x10,
    EVT_HEARTBEAT   = 0xFF
};

struct ProbeEventPacket {
    uint8_t  magic;
    uint8_t  event_type;
    uint32_t seq;
    float    lux_raw_g0;
    float    lux_raw_g5;
};

enum ProbeCommand : uint8_t {
    CMD_RENDER      = 0x00,
    CMD_MEASURE_G0  = 0x01,
    CMD_MEASURE_G5  = 0x02,
    CMD_IDLE        = 0x03
};

enum ProbeHaptic : uint8_t {
    HAPTIC_NONE     = 0x00,
    HAPTIC_CLICK    = 0x01,
    HAPTIC_ERROR    = 0x02,
    HAPTIC_DONE     = 0x03
};

enum ProbeDisplayMode : uint8_t {
    PMODE_IDLE       = 0x00,
    PMODE_METER_BW   = 0x01,
    PMODE_METER_SG   = 0x02,
    PMODE_BURN       = 0x03,
    PMODE_CALIBRATE  = 0x04,
    PMODE_DENSITOM   = 0x05
};

struct ProbeRenderPacket {
    uint8_t  magic;
    uint8_t  command;
    char     header_text[16];
    char     line1_text[16];
    char     line2_text[16];
    uint8_t  zone_histogram[11];
    uint8_t  haptic_feedback;
    uint8_t  display_mode;
};

struct WirelessPacket {
    uint8_t magic;
    uint32_t seq;
    float lux;
};

#endif