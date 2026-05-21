/* =============================================================================
 * Config.h - DUKATIMER BETA (v0.916.8)
 * * ZENTRALE HARDWARE-KONFIGURATION (Exakt v0.5 Wiring)
 * * HÄRTUNG: Pegel-Definitionen für Schalter und Relais.
 * ========================================================================== */

#pragma once
#include <Arduino.h>

#define SW_VERSION_NAME "v0.916.8 Beta"
#define SW_VERSION_HEX 0x0916 

#define NVS_SPACE "dukatimer"

// =============================================================================
// 1. BENUTZERSCHNITTSTELLE (EINGABEN) - Exakt v0.5 Layout
// =============================================================================

// Encoder 1: Soft (Links)
#define PIN_ENC_SOFT_A  5
#define PIN_ENC_SOFT_B  4
#define PIN_ENC_SOFT_SW 42 

// Encoder 2: Hard (Mitte Links)
#define PIN_ENC_HARD_A  6
#define PIN_ENC_HARD_B  7
#define PIN_ENC_HARD_SW 41 

// Encoder 3: Grade (Mitte Rechts)
#define PIN_ENC_GRADE_A 15
#define PIN_ENC_GRADE_B 16
#define PIN_ENC_GRADE_SW 39 

// Encoder 4: Mode (Rechts / Master)
#define PIN_ENC_MODE_A  40
#define PIN_ENC_MODE_B  47 
#define PIN_ENC_MODE_SW 3 

// Mechanische Schalter & Taster (Inputs)
// WICHTIG: Diese Schalter ziehen in deinem v0.5 Layout gegen GND.
#define PIN_BTN_START 11 
#define PIN_BTN_RED   21   // Savelight Schalter (Eingang)
#define PIN_BTN_WHITE 14   // Focus Schalter (Eingang)
#define PIN_BTN_ROOM  13   // Raumlicht Schalter (Eingang)

// =============================================================================
// 2. AKTOREN (AUSGÄNGE)
// =============================================================================
#define PIN_NEOPIXEL   38    
#define NEOPIXEL_COUNT 256 

// Relais-Steuerung
// Pin 12 ist das einzige Relais (Raumlicht).
#define PIN_RELAY_ROOMLIGHT 12
//#define PIN_RELAY_FOCUS     -1
//#define PIN_RELAY_SAFE      -1
//#define PIN_RELAY_ENLARGER  -1

// Buzzer (Auf Pin 45 verschoben, da Pin 36/37 beim S3 Octal-RAM kritisch sind)
#define PIN_BUZZER 45 

// =============================================================================
// 3. KOMMUNIKATION & SENSORIK
// =============================================================================
#define NEXTION_TX 2
#define NEXTION_RX 1

#define I2C_SDA  8
#define I2C_SCL  9
#define I2C_FREQ 100000 

#define ADDR_LCD     0x3E     
#define ADDR_TSL2591 0x29 
#define ADDR_BMP280  0x76  

#define PIN_I2C1_SDA 17
#define PIN_I2C1_SCL 18
#define I2C1_FREQ    400000 

#define ADDR_TSL2561    0x39  
#define PIN_TSL2561_INT 46  

#define PIN_ONEWIRE 10

// =============================================================================
// 4. SOFTWARE PARAMETER
// =============================================================================
#define kStateLockWait pdMS_TO_TICKS(100)
#define kDoseLockWait  pdMS_TO_TICKS(100)
#define kPreWaitMs 500U

#define TIME_MIN_S 0.5
#define TIME_MAX_S 999.0
#define TEMP_MAX_ALU 60.0
#define SOUND_QUIET_SCALE 0.60

static const char* const stepNames[] = {"1/1", "1/2", "1/3", "1/6"};