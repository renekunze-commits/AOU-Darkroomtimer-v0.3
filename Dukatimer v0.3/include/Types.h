#ifndef DUKATIMER_TYPES_H
#define DUKATIMER_TYPES_H
#pragma message("DUKATIMER Types.h included")

#include <Arduino.h>

// =============================================================================
// DUKATIMER - Types.h
//
// Zweck:
// - Zentrale Definition aller Datentypen/Strukturen, die projektweit benutzt werden
// - Beinhaltet PaperProfile, SettingsObject (EEPROM), Button-State, Messwerte
// - Enums für Modi/States und Sound/Step-Größen
//
// Hinweise:
// - Verwende `double` für Zeit-/Messwerte für konsistente Genauigkeit.
// - `SettingsObject` enthält ein `crc`-Feld am Ende; bei Änderungen an der
//   Struktur muss die CRC-Berechnung berücksichtigt werden.
// =============================================================================

// =============================================================================
// SOUND (Beep) - Typ für Piepton-Segmente
// =============================================================================
struct BeepSeg {
  int f;    // Frequenz
  int d;    // Dauer (ms)
  int p;    // Pause (ms)
  bool prio;
};

// =============================================================================
// PAPER PROFILE & SETTINGS (EEPROM)
// =============================================================================
struct PaperProfile {
  char name[16];     // Längerer Name für App-Anzeige
  
  // Standard Splitgrade (berechnet alles dazwischen)
  double Ksoft;      
  double Khard;      
  double Kbw;        
  
  // NEU: Optionale manuelle Gradations-Messwerte (0..5)
  // Wenn > 0, wird dieser Wert statt der Berechnung genutzt
  double gradeK[6];  // Index 0=G0, 1=G1 ... 5=G5
  
  bool calibrated;   
  
  // Flash Settings
  bool flashEnable;
  double flashThreshS;
  // Flash Legacy
  bool flashCalibrated;
  int flashLevel;
  int flashColor;
  double flashFactor;
};

// Container für alle Papiere (separat von Settings!)
struct PaperBank {
  uint16_t version;     // Versionierung für App-Sync wichtig
  uint8_t activeIndex;  // Welches Papier ist gerade gewählt
  PaperProfile profiles[20]; // Platz für 20 Papiere
};

// SettingsObject wird schlanker (enthält KEINE Papiere mehr)
struct SettingsObject {
  // Versions-Header
  uint16_t version;
  // Laufzeitwerte
  double t_s;      // Time Soft
  double t_h;      // Time Hard
  double t_bw;     // Time BW
  double g_bw;     // Grade BW
  double burn_g;   // Burn Grade
  // Legacy
  double k_s, k_h, k_bw;
  // PWM / Brightness
  uint8_t pwm_safe;   
  uint8_t pwm_focus;  
  uint8_t pwm_lcd;    
  uint8_t pwm_max;    
  // Logic Config
  bool useDoseMode;   
  double std_time;
  uint8_t splitMode;    
  uint8_t stepMode;     
  uint8_t soundMode; 
  uint16_t crc;
};

// =============================================================================
// BUTTON STATE - Kompletter Satz Felder, wie im Projekt gebraucht
// =============================================================================
struct BtnState {
    // Hardware Debounce
    bool lastState;       // letzter Rohwert (gepullt)
    bool isPressed;       // logischer Zustand (gedrückt ja/nein)
    unsigned long lastDebounceTime;
    // Long Press / Repeat
    unsigned long pressStartMs;
    unsigned long lastRepeatMs;
    unsigned long intervalMs;
    // Konfig
    unsigned long longPressDuration;
};

// =============================================================================
// Messwerte
// =============================================================================
struct SpotMeas {
  double lux;
  double temp;
  bool ok;     
  uint16_t ch0;
  uint16_t ch1;
};

// =============================================================================
// ENUMS
// =============================================================================

enum Mode { MODE_BW, MODE_SG, MODE_DENS };
enum BurnMode { BURN_OFF, BURN_BW, BURN_SG_G0, BURN_SG_G5 };
enum SplitState { SPLIT_IDLE, SPLIT_DOING_SOFT, SPLIT_SOFT_DONE, SPLIT_DOING_HARD, SPLIT_HARD_DONE };

enum TSState { TS_OFF, TS_SETUP, TS_RUNNING };
enum TSChannel { TS_BW, TS_SOFT, TS_HARD };

enum CalStep { CAL_IDLE, CAL_START, CAL_G5, CAL_G0, CAL_G25, CAL_REVIEW, CAL_DONE };

enum DensitometerState { DENS_IDLE, DENS_REF, DENS_MEAS };
enum DensSubMode { DENS_SUB_MANUAL, DENS_SUB_AUTO };
enum MeasureMode { MM_OFF, MM_APPLY_BW, MM_APPLY_SG_G0, MM_APPLY_SG_G5 };
enum SoundMode { SOUND_OFF, SOUND_QUIET, SOUND_NORMAL };

enum StepSize { STEP_FULL, STEP_HALF, STEP_THIRD, STEP_SIXTH };

// =============================================================================
// ESP-NOW WIRELESS PROTOCOL
// =============================================================================
#define REMOTE_MAGIC 0xD4        // Kennung für Dukatimer Pakete
#define ESPNOW_CHANNEL 1

struct WirelessPacket {
    uint8_t magic;    // Sicherheits-Check
    uint32_t seq;     // Paket-Zähler
    float lux;        // Messwert
    // Optional: Batteriestatus könnte hier noch rein
};

#endif