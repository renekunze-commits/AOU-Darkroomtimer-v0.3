#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// CONFIG.H - Zentrale Hardware- und Software-Konstanten (v0.5)
// =============================================================================

// --- ENCODER ---
#define ENC_SOFT_A      5
#define ENC_SOFT_B      4
#define ENC_HARD_A      6
#define ENC_HARD_B      7
#define ENC_GRADE_A     15
#define ENC_GRADE_B     16

// NEU: Master Mode Encoder (PCNT 3)
#define ENC_MODE_A      40 // Ehemaliger Pin für PIN_SW_DOSEMODE
#define ENC_MODE_B      47 // ACHTUNG: Bitte prüfen, ob Pin 19 auf dem S3-Board frei ist!

// Encoder Taster (Physische Belegung)
#define PIN_SW_ENTER    41  // Encoder 2 Klick (Tatsächlich auf 41)
#define PIN_SW_BACK     42  // Encoder 1 Klick (Tatsächlich auf 42)
#define PIN_SW_GRADE    39  // Encoder 3 Klick
#define PIN_SW_MODE     3   // ACHTUNG: Bitte prüfen, ob Pin 3 auf dem S3-Board frei ist!

// Logische Aliase (Vermeidung von Doppelbelegungen)
#define PIN_SW_STEP_SELECT  PIN_SW_BACK 

// --- DISPLAYS ---
#define NEXTION_RX      1 
#define NEXTION_TX      2 

// --- I2C BUSSE ---
#define PIN_I2C0_SDA     8
#define PIN_I2C0_SCL     9
#define I2C0_FREQ        100000  
#define LCD_I2C_ADDR     0x3E  
#define TSL2591_I2C_ADDR 0x29  
#define BMP280_I2C_ADDR  0x76  // <--- Hier ist der Umgebungstemp-Sensor!

#define PIN_I2C1_SDA     17
#define PIN_I2C1_SCL     18
#define I2C1_FREQ        400000 
#define TSL2561_I2C_ADDR 0x39  

// --- SENSOREN ---
#define PIN_ONEWIRE     10  
#define PIN_TSL2561_INT 46  

// --- MECHANISCHE SCHALTER (INPUTS) ---
#define PIN_START           11  
#define PIN_SW_ROOMLIGHT    13  
#define PIN_SW_FOCUS        14  
#define PIN_SW_SAFE         21  

// HINWEIS: PIN_SW_DOSEMODE wurde durch den Mode-Encoder ersetzt!

// --- AUSGÄNGE ---
#define PIN_RELAY_ROOMLIGHT 12 
#undef  PIN_NEOPIXEL
#define PIN_NEOPIXEL        38    
#define NEOPIXEL_COUNT      256 
#define PIN_BUZZER          36  
//#define PIN_RELAY_SAFE      22
//#define PIN_RELAY_ENLARGER  23
//#define PIN_RELAY_SAFE      1
//#define PIN_RELAY_ENLARGER  2



// =============================================================================
// SOFTWARE KONFIGURATION
// =============================================================================
#define SW_VERSION 0x0500

// CODE_REVIEW FIX (Audit Kritisch #12): ODR-Verletzung behoben.
// 'const String stepNames[]' in einem Header erzeugt pro Translation Unit eine eigene
// Kopie mit separaten Konstruktor-Aufrufen (Arduino String ist kein Literal-Typ).
// Das verschwendet RAM und erzeugt unnötige Runtime-Initialisierung.
// Fix: Statische const char* Zeiger-Tabelle. Kein Konstruktor, kein ODR-Problem.
// const String stepNames[] = {"1/1", "1/2", "1/3", "1/6"};  // ALT: ODR-Verletzung
static const char* const stepNames[] = {"1/1", "1/2", "1/3", "1/6"};

#define TIME_MIN_S      0.5
#define TIME_MAX_S      999.0

#define DEFAULT_TS_N    6
#define DEFAULT_TS_EV   (1.0/3.0)
#define TS_EV_MIN       0.1
#define TS_EV_MAX       1.0
#define TS_EV_STEP      (1.0/6.0)

#define SOUND_QUIET_SCALE 0.60
#define TEMP_MAX_ALU    60.0
#define FILTER_SIZE     10 

#endif