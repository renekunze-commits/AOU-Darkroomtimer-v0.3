#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- I2C PINS (Bus 0: Sensor & LCD Display) ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- SENSOR PINS ---
#define PIN_TSL_INT 4

// --- ENCODER PINS (Rechts unten, Push-Button) ---
#define ENC1_A   32
#define ENC1_B   33
#define ENC1_BTN 25

// --- TASTER PINS (Links, vertikal angeordnet) ---
// Physikalisches Layout gemäß Spec:
//   Taster 1 (oben)  = UNDO / ZURÜCK / REFERENZ
//   Taster 2 (unten) = AKTION / MESSEN / FEUER (häufigste Aktion)
#define PIN_BTN_T1 16   // Taster 1 (oben links)
#define PIN_BTN_T2 17   // Taster 2 (unten links)

// --- BUZZER PIN ---
#define PIN_BUZZER 27

// --- DISPLAY SETTINGS (LCD 16x2 I2C) ---
#define LCD_COLS 16
#define LCD_ROWS 2
#define LCD_ADDRESS 0x27

// --- SYSTEM SETTINGS ---
#define REMOTE_MAGIC        0xD4
#define HEARTBEAT_INTERVAL  1000     // Heartbeat alle 1000ms
#define DISPLAY_UPDATE_MS   250

// =============================================================================
// ESP-NOW WIRELESS PROTOCOL (Identisch mit S3 Types.h)
// =============================================================================
// Paketgrößen (ESP-NOW Max: 250 Bytes):
//   Typ A (C6→S3): 14 Bytes
//   Typ B (S3→C6): 63 Bytes

// --- Paket Typ A: C6 → S3 = Event-Trigger ---
enum ProbeEventType : uint8_t {
    EVT_NONE        = 0x00,
    EVT_T2_CLICK    = 0x01,  // MESSEN / FEUER
    EVT_T1_CLICK    = 0x02,  // UNDO / ZURÜCK / REFERENZ
    EVT_ENC_CLICK   = 0x03,  // ENTER / ABSCHLIESSEN
    EVT_ENC_UP      = 0x04,  // Encoder CW
    EVT_ENC_DOWN    = 0x05,  // Encoder CCW
    EVT_LUX_DATA    = 0x10,  // Antwort auf CMD_MEASURE mit Lux-Werten
    EVT_HEARTBEAT   = 0xFF   // Periodischer Ping
};

struct ProbeEventPacket {
    uint8_t  magic;           // 0xD4
    uint8_t  event_type;      // ProbeEventType
    uint32_t seq;             // Paket-Zähler
    float    lux_raw_g0;      // Grün-Messung (nur bei EVT_LUX_DATA)
    float    lux_raw_g5;      // Blau-Messung (nur bei EVT_LUX_DATA)
};

// --- Paket Typ B: S3 → C6 = Render-Befehl ---
enum ProbeCommand : uint8_t {
    CMD_RENDER      = 0x00,  // Display-Update
    CMD_MEASURE_G0  = 0x01,  // Grün-Messung anfordern (Flash-Handshake Phase 1)
    CMD_MEASURE_G5  = 0x02,  // Blau-Messung anfordern (Flash-Handshake Phase 2)
    CMD_IDLE        = 0x03   // Standby
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
    uint8_t  magic;              // 0xD4
    uint8_t  command;            // ProbeCommand
    char     header_text[16];    // z.B. "[ BW METERING ]"
    char     line1_text[16];     // z.B. "Time: 14.5s"
    char     line2_text[16];     // z.B. "Grade: 2.5"
    uint8_t  zone_histogram[11]; // Balkenhöhe Zonen 0-X (0-255)
    uint8_t  haptic_feedback;    // ProbeHaptic
    uint8_t  display_mode;       // ProbeDisplayMode
};

// --- Legacy Paket (wird nicht mehr aktiv gesendet, nur für Referenz) ---
struct WirelessPacket {
    uint8_t magic;
    uint32_t seq;
    float lux;
};

#endif#endif