/* =============================================================================
 * Types.h - DUKATIMER BETA (v0.917.4)
 * * Zentrale Typen- und Enum-Definitionen.
 * * REVISION: Wireless Protocol Integration (ESP-NOW) & Dynamisches Routing
 * ========================================================================== */

#pragma once

#include <stdint.h>

/**
 * Alle Betriebsmodi des Dukatimers.
 * Dient als Index für das App-Routing und Nextion-Seitenmanagement.
 */
enum Mode
{
    MODE_BW,           // Schwarz-Weiß (Zeit)
    MODE_SG,           // Split-Grade (Zeit)
    MODE_CALIBRATION,  // Papier-Kalibrierung
    MODE_SETUP,        // Systemeinstellungen
    MODE_BURN,         // Nachbelichtung (Einbrennen)
    MODE_DENSITOMETER, // Densitometer-Messmodus
    MODE_PREFLASH,     // Vorbelichtung
    MODE_TEST_STRIP,   // Teststreifen-Erstellung
    MODE_LIVE_VIEW,    // Live-Helligkeitsanzeige (Fokus-Hilfe)
    MODE_BW_DOSE,      // Schwarz-Weiß (Dosis-gesteuert)
    MODE_SG_DOSE,      // Split-Grade (Dosis-gesteuert)
    MODE_BW_FSTOP,     // F-Stop Timer Modus
    MODE_ZONE,         // Zonen-System Belichtung

    _MODE_COUNT // WICHTIG: Muss IMMER das letzte Element bleiben (Wrap-Around)
};

/**
 * Systemweite Fehlercodes.
 * Werden im HardwareStatus getrackt und im Setup-Menü angezeigt.
 */
enum SystemError
{
    ERR_NONE = 0,            // Alles OK
    ERR_STORAGE_CORRUPTED,   // LittleFS Lesefehler oder Hash-Mismatch
    ERR_MUTEX_TIMEOUT,       // Deadlock-Gefahr oder CPU-Überlastung
    ERR_THERMAL_OVERHEAT,    // LED-Kühlkörper zu heiß
    ERR_SENSOR_DISCONNECTED, // I2C Kommunikation zum Sensor unterbrochen
    ERR_MATH_INVALID         // Rechenfehler (Division durch Null etc.)
};

/**
 * Systemweite Eingabe-Events.
 * Werden vom InputManager generiert und vom AppManager an die aktive App verteilt.
 */
enum InputEvent
{
    // --- System / Notfall ---
    EV_ABORT = 0, // System-Guard: Not-Aus / Abbruch

    // --- Encoder 1: Soft (Menü / Papierauswahl) ---
    EV_MENU_UP,
    EV_MENU_DOWN,
    EV_MENU_CLICK,

    // --- Encoder 2: Hard (Zeit / Basiswerte) ---
    EV_TIME_UP,
    EV_TIME_DOWN,
    EV_TIME_CLICK,

    // --- Encoder 3: Grade (Kontrast / Dosis / D-Werte) ---
    EV_GRADE_UP,
    EV_GRADE_DOWN,
    EV_GRADE_CLICK,

    // --- Encoder 4: Mode (Betriebsmodus) ---
    EV_MODE_NEXT,
    EV_MODE_PREV,
    EV_MODE_CLICK,

    // --- Externe Taster (Lokal am Basisgerät) ---
    EV_START, // Start-Taster (Panel/Fußschalter)

    // --- NEU: REMOTE EVENTS (VOM HANDGERÄT C6) ---
    EV_REMOTE_T1_CLICK,  // Top Button (Back / Undo)
    EV_REMOTE_T2_CLICK,  // Bottom Button (Start / Measure)
    EV_REMOTE_ENC_UP,    // Side Encoder Up
    EV_REMOTE_ENC_DOWN,  // Side Encoder Down
    EV_REMOTE_ENC_CLICK, // Side Encoder Click (Apply)
    EV_REMOTE_ENC_LONG,  // Side Encoder Long-Press

    // --- Marker für ungültige / leere Events ---
    EV_NONE = 0xFFFF
};

// =============================================================================
// ESP-NOW PROTOKOLL DEFINITIONEN (Rückwärtskompatibel zu v0.3)
// =============================================================================
#define REMOTE_MAGIC 0xD4

/**
 * Roh-Event Typen vom C6 Handgerät.
 */
enum ProbeEventType
{
    PRB_EVT_NONE = 0x00,
    PRB_EVT_T2_CLICK = 0x01,  // Bottom Button (Measure/Start)
    PRB_EVT_T1_CLICK = 0x02,  // Top Button (Undo/Back)
    PRB_EVT_ENC_CLICK = 0x03, // Side Encoder Click
    PRB_EVT_ENC_UP = 0x04,
    PRB_EVT_ENC_DOWN = 0x05,
    PRB_EVT_ENC_LONG = 0x06,
    PRB_EVT_LUX_DATA = 0x10, // Messdaten-Paket
    PRB_EVT_HEARTBEAT = 0xFF
};

/**
 * Befehle vom S3 an das C6.
 */
enum ProbeCommand
{
    PRB_CMD_RENDER = 0,     // Text-Update
    PRB_CMD_MEASURE_G0 = 1, // Starte Messung (Grün)
    PRB_CMD_MEASURE_G5 = 2, // Starte Messung (Blau)
    PRB_CMD_IDLE = 3        // Schlafmodus / Inaktiv
};

/**
 * Display-Zustand des C6 (für OLED Darstellung).
 */
enum ProbeDisplayMode
{
    PRB_DISP_IDLE = 0,
    PRB_DISP_METER_BW = 1,
    PRB_DISP_METER_SG = 2,
    PRB_DISP_BURN = 3,
    PRB_DISP_CALIBRATE = 4,
    PRB_DISP_DENSITOM = 5
};

// Eingehend vom Handgerät (C6 -> S3)
struct __attribute__((packed)) ProbeEventPacket
{
    uint8_t magic;      // Muss REMOTE_MAGIC (0xD4) sein
    uint8_t event_type; // Roh-Event vom C6 (wird im WirelessMgr übersetzt)
    uint32_t seq;       // Sequenznummer gegen Packet-Loss
    float lux_raw_g0;   // Rohwert Grün-Messung
    float lux_raw_g5;   // Rohwert Blau-Messung
};

// Ausgehend zum Handgerät (S3 -> C6)
struct __attribute__((packed)) ProbeRenderPacket
{
    uint8_t magic;   // Muss REMOTE_MAGIC (0xD4) sein
    uint8_t command; // 0=Render, 1=MeasG0, 2=MeasG5, 3=Idle
    char header_text[16];
    char line1_text[16];
    char line2_text[16];
    uint8_t zone_histogram[11]; // Das Array aus dem SystemContext (WorkflowFlags)
    uint8_t haptic_feedback;    // 0=None, 1=Click, 2=Error, 3=Done
    uint8_t display_mode;
};

// Statische ABI-Prüfung zur Sicherheit, damit die Structs durch
// eventuelle Compiler-Updates niemals heimlich ein Padding-Byte erhalten.
static_assert(sizeof(ProbeEventPacket) == 14, "ABI Mismatch");
static_assert(sizeof(ProbeRenderPacket) == 63, "ABI Mismatch");
