#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// CONFIG.H - Zentrale Hardware- und Software-Konstanten
// =============================================================================

// --- ENCODER ---
#define ENC_SOFT_A      4
#define ENC_SOFT_B      5
#define ENC_HARD_A      6
#define ENC_HARD_B      7
#define ENC_GRADE_A     15
#define ENC_GRADE_B     16

// Encoder Taster
#define PIN_SW_ENTER    42
#define PIN_SW_BACK     41
#define PIN_SW_GRADE    39

// Pin für die EV-Umschaltung (Encoder 1 Taster)
#define PIN_SW_STEP_SELECT  42


// --- DISPLAYS ---
#define NEXTION_RX      RX    
#define NEXTION_TX      TX   

// --- I2C BUSSE ---
#define PIN_I2C0_SDA     8
#define PIN_I2C0_SCL     9
#define I2C0_FREQ        100000  
#define LCD_I2C_ADDR     0x3E  
#define TSL2591_I2C_ADDR 0x29  

#define PIN_I2C1_SDA     17
#define PIN_I2C1_SCL     18
// I2C Bus 1 (TSL2561 Live-Sensor): 400 kHz Fast-Mode
// Der TSL2561 unterstützt offiziell bis 400 kHz. Höhere Busgeschwindigkeit
// reduziert die Blockierzeit pro Messwert-Abfrage von ~1ms auf ~0.25ms,
// was dem Closed-Loop-Task mehr CPU-Zeit für die Belichtungsregelung gibt.
#define I2C1_FREQ        400000
#define TSL2561_I2C_ADDR 0x39  

// --- SENSOREN ---
#define PIN_ONEWIRE     10  
#define PIN_TSL2561_INT 46   

// --- SCHALTER (INPUTS) ---
#define PIN_START           11  
#define PIN_SW_ROOMLIGHT    13  
#define PIN_SW_FOCUS        14  
#define PIN_SW_SAFE         21  

// --- AUSGÄNGE ---
#define PIN_RELAY_ROOMLIGHT 12 
#ifdef PIN_NEOPIXEL
  #undef PIN_NEOPIXEL
#endif 
#define PIN_NEOPIXEL        38    
#define NEOPIXEL_COUNT      256 
#define PIN_BUZZER          48  
#define PIN_RELAY_SAFE      22
#define PIN_RELAY_ENLARGER  23

// =============================================================================
// SOFTWARE KONFIGURATION
// =============================================================================

#define SW_VERSION 0x0302

// Anzeige-Texte für die EV-Stufen
const String stepNames[] = {"1/1", "1/2", "1/3", "1/6"}; //

// Zeitgrenzen
#define TIME_MIN_S      0.5
#define TIME_MAX_S      999.0

// --- TESTSTRIP KONFIGURATION (Behebt Fehler in Mode_TestStrip.cpp) ---
#define DEFAULT_TS_N    6
#define DEFAULT_TS_EV   (1.0/3.0)
#define TS_EV_MIN       0.1       //
#define TS_EV_MAX       1.0       //
#define TS_EV_STEP      (1.0/6.0) //

// --- SOUND KONFIGURATION (Behebt Fehler in HW_Sound.cpp) ---
#define SOUND_QUIET_SCALE 0.60    //

// --- SONSTIGES ---
#define TEMP_MAX_ALU    60.0
#define FILTER_SIZE     10 

// Nextion IDs
#define PAGE_MAIN             0
#define NEXTION_BTN_SW_ID     9 
#define NEXTION_BTN_SG_ID     10  
#define NEXTION_BTN_DENS_ID   11  
#define NEXTION_BTN_HSH_ID    12  
#define NEXTION_BTN_STR_ID    13  
#define NEXTION_BTN_FOC_ID    14  
#define NEXTION_BTN_SL_ID     15  
#define NEXTION_BTN_MNU_ID    16  
#define NEXTION_BTN_SCR_ID    17  

#endif