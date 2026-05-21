#pragma once
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// BeWeMe Kommunikationsprotokoll – gemeinsamer Header
// =============================================================================
// REGEL: Sender (BeWeMe-C6) und Empfänger (BEWeMe-S3) binden ausschließlich
//        diese Datei ein. Struct-Definitionen NIEMALS lokal redefinieren!
//
// Bei strukturellen Änderungen an SensorData:
//   1. PROTOCOL_VERSION erhöhen
//   2. Alte SensorData als SensorDataVx anlegen (Rückwärtskompatibilität)
//   3. decodeIncomingSensorPacket() im Empfänger um neuen Fall erweitern
// =============================================================================

static constexpr uint8_t PROTOCOL_VERSION_V1 = 1;
static constexpr uint8_t PROTOCOL_VERSION_V2 = 2;
static constexpr uint8_t PROTOCOL_VERSION_V3 = 3;
static constexpr uint8_t PROTOCOL_VERSION_V4 = 4;
static constexpr uint8_t PROTOCOL_VERSION = PROTOCOL_VERSION_V4;
static constexpr uint8_t CONTROL_PACKET_MAGIC = 0xB7;
static constexpr uint8_t CONTROL_COMMAND_SET_LED_OUTPUT = 1;

// Presence-Byte Bitdefinitionen (SensorData.presence):
//   Bit 0 (SENSOR_PRESENCE_BIT):    0 = keine Praesenz, 1 = Praesenz erkannt (LD2410)
//   Bit 7 (SENSOR_FLAG_CLIMATE_ONLY): 1 = Timer-Wakeup ohne Radar, nur Klimadaten gueltig;
//                                     distance und energy sind ungueltig (immer 0)
//
// Empfaenger: Bit 0 fuer Praesenz-LED verwenden; bei Bit 7 = 1 Distanz-/Energie-Anzeige
// und Histogramm-Eintrag ueberspringen.
static constexpr uint8_t SENSOR_PRESENCE_BIT      = 0x01;
static constexpr uint8_t SENSOR_FLAG_CLIMATE_ONLY = 0x80;

// ---- V1: Erstes Format, kein Versions-Byte, kein BMP280-Temp ----
struct __attribute__((packed)) SensorDataV1 {
    uint8_t  presence;   // 0/1
    uint16_t distance;   // [cm]
    uint8_t  energy;     // Bewegungsenergie [%]
    float    temp;       // AHT20 Temperatur [°C]
    float    humidity;   // AHT20 relative Feuchte [%]
    float    pressure;   // BMP280 Luftdruck [hPa]
    float    vcc;        // Versorgungsspannung [V]
};

// ---- V2: Mit BMP280-Temperatur, noch kein Versions-Byte ----
struct __attribute__((packed)) SensorDataV2 {
    uint8_t  presence;   // 0/1
    uint16_t distance;   // [cm]
    uint8_t  energy;     // Bewegungsenergie [%]
    float    temp;       // AHT20 Temperatur [°C]
    float    humidity;   // AHT20 relative Feuchte [%]
    float    pressure;   // BMP280 Luftdruck [hPa]
    float    vcc;        // Versorgungsspannung [V] — 0.0 = nicht gemessen
    float    tempBmp;    // BMP280 Temperatur [°C]
};

// ---- V3: Explizites Versions-Byte als erstes Feld ----
struct __attribute__((packed)) SensorDataV3 {
    uint8_t  version;    // = PROTOCOL_VERSION_V3
    uint8_t  presence;   // 0/1
    uint16_t distance;   // [cm]
    uint8_t  energy;     // Bewegungsenergie [%]
    float    temp;       // AHT20 Temperatur [°C]
    float    humidity;   // AHT20 relative Feuchte [%]
    float    pressure;   // BMP280 Luftdruck [hPa]
    float    vcc;        // Versorgungsspannung [V] — 0.0 = nicht gemessen
    float    tempBmp;    // BMP280 Temperatur [°C]
};

// ---- V4 (aktuell): Session + Sequenznummern zur Paketverlust-Erkennung ----
struct __attribute__((packed)) SensorData {
    uint8_t  version;    // = PROTOCOL_VERSION
    uint32_t sessionId;  // bleibt ueber Deep-Sleep erhalten, aendert sich bei echtem Neustart
    uint32_t sequence;   // monoton pro Session, Start bei 1
    uint8_t  presence;   // 0/1
    uint16_t distance;   // [cm]
    uint8_t  energy;     // Bewegungsenergie [%]
    float    temp;       // AHT20 Temperatur [°C]
    float    humidity;   // AHT20 relative Feuchte [%]
    float    pressure;   // BMP280 Luftdruck [hPa]
    float    vcc;        // Versorgungsspannung [V] — 0.0 = nicht gemessen
    float    tempBmp;    // BMP280 Temperatur [°C]
};

struct __attribute__((packed)) ControlPacket {
    uint8_t magic;       // = CONTROL_PACKET_MAGIC
    uint8_t command;     // siehe CONTROL_COMMAND_*
    uint8_t value;       // kommandoabhaengig; fuer LED-Ausgabe: 0=aus, 1=an
    uint8_t reserved;    // fuer spaetere Erweiterungen
};

// Compile-time-Größenprüfung
#ifdef __cplusplus
static_assert(sizeof(SensorDataV1) == 20, "SensorDataV1 Groesse unerwartet!");
static_assert(sizeof(SensorDataV2) == 24, "SensorDataV2 Groesse unerwartet!");
static_assert(sizeof(SensorDataV3) == 25, "SensorDataV3 Groesse unerwartet!");
static_assert(sizeof(SensorData)   == 33, "SensorData V4 Groesse unerwartet!");
static_assert(sizeof(ControlPacket) == 4, "ControlPacket Groesse unerwartet!");
#endif
