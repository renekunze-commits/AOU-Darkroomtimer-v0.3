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
enum MeasureFocus { FOCUS_HIGHLIGHTS, FOCUS_SHADOWS };
enum MeasureState { MEASURE_IDLE, MEASURE_G0_WAIT_DATA, MEASURE_G5_WAIT_DATA, MEASURE_ERROR };
enum SoundMode { SOUND_OFF, SOUND_QUIET, SOUND_NORMAL };

enum StepSize { STEP_FULL, STEP_HALF, STEP_THIRD, STEP_SIXTH };

// LED-Mode für explizite Lichtsteuerung im Flash-Handshake
enum LedMode : uint8_t {
  LED_OFF = 0,
  LED_GREEN,
  LED_BLUE,
  LED_FOCUS,
  LED_SAFELIGHT
};

// =============================================================================
// ESP-NOW WIRELESS PROTOCOL (Bidirektional)
// =============================================================================
// Architektur: Das Handgerät (C6) ist ein "Dumb Terminal".
// Es hält keinen eigenen Status, berechnet keine Mathematik und kennt keine
// Papier-Profile. Es liest Taster aus, sendet Events an den S3 und zeichnet
// auf dem Display exakt das, was der S3 ihm im Antwort-Paket befiehlt.
// Dies verhindert Status-Asynchronität (System-Dissoziation).
//
// Paketgrößen (ESP-NOW Max: 250 Bytes):
//   Typ A (C6→S3): 14 Bytes
//   Typ B (S3→C6): 63 Bytes
// =============================================================================
#define REMOTE_MAGIC 0xD4        // Kennung für Dukatimer Pakete
#define ESPNOW_CHANNEL 1

// --- Legacy Paket (Abwärtskompatibel für reinen Lux-Broadcast) ---
struct WirelessPacket {
    uint8_t magic;    // Sicherheits-Check
    uint32_t seq;     // Paket-Zähler
    float lux;        // Messwert
};

// --- Paket Typ A: C6 (Handgerät) → S3 (Basis) = Event-Trigger ---
// Das Handgerät sendet ausschließlich Rohdaten und Hardware-Events.
// Größe: 14 Bytes (well within 250 Byte ESP-NOW Limit)

enum ProbeEventType : uint8_t {
    EVT_NONE        = 0x00,
    EVT_T2_CLICK    = 0x01,  // Taster 2 (unten links): MESSEN / FEUER
    EVT_T1_CLICK    = 0x02,  // Taster 1 (oben links):  UNDO / ZURÜCK / REFERENZ
    EVT_ENC_CLICK   = 0x03,  // Encoder Klick:          ENTER / ABSCHLIESSEN
    EVT_ENC_UP      = 0x04,  // Encoder Drehen CW:      NAVIGATION vorwärts
    EVT_ENC_DOWN    = 0x05,  // Encoder Drehen CCW:      NAVIGATION rückwärts
    EVT_LUX_DATA    = 0x10,  // Antwort auf CMD_MEASURE: Enthält Lux-Messwert
    EVT_HEARTBEAT   = 0xFF   // Periodischer Lebenszeichen-Ping
};

struct ProbeEventPacket {
    uint8_t  magic;           // 0xD4
    uint8_t  event_type;      // ProbeEventType
    uint32_t seq;             // Paket-Zähler (fortlaufend)
    float    lux_raw_g0;      // Nur befüllt bei EVT_LUX_DATA (Grün-Messung)
    float    lux_raw_g5;      // Nur befüllt bei EVT_LUX_DATA (Blau-Messung)
};
// sizeof(ProbeEventPacket) = 14 Bytes

// --- Paket Typ B: S3 (Basis) → C6 (Handgerät) = Render-Befehl ---
// Der S3 verarbeitet das Event in seiner State-Machine und sendet exakt
// zurück, was auf dem OLED stehen soll.
// Größe: 63 Bytes

enum ProbeCommand : uint8_t {
    CMD_RENDER      = 0x00,  // Normaler Display-Update (Header + Lines + Histogram)
    CMD_MEASURE_G0  = 0x01,  // S3 fordert Grün-Messung an (Flash-Handshake Phase 1)
    CMD_MEASURE_G5  = 0x02,  // S3 fordert Blau-Messung an (Flash-Handshake Phase 2)
    CMD_IDLE        = 0x03   // C6 soll in Standby gehen (kein aktiver Modus)
};

enum ProbeHaptic : uint8_t {
    HAPTIC_NONE     = 0x00,  // Kein Feedback
    HAPTIC_CLICK    = 0x01,  // Kurzer Bestätigungs-Klick
    HAPTIC_ERROR    = 0x02,  // Langer Fehler-Brummer
    HAPTIC_DONE     = 0x03   // Doppel-Klick (Aktion abgeschlossen)
};

// Aktueller Modus des S3 (damit C6 weiß, welches Layout anzuzeigen ist)
enum ProbeDisplayMode : uint8_t {
    PMODE_IDLE       = 0x00,
    PMODE_METER_BW   = 0x01,  // BW Zonen-Messung
    PMODE_METER_SG   = 0x02,  // Splitgrade Zonen-Messung
    PMODE_BURN       = 0x03,  // Burn-Modus (Nachbelichten Remote)
    PMODE_CALIBRATE  = 0x04,  // Kalibrierungs-Modus
    PMODE_DENSITOM   = 0x05   // Densitometer-Modus
};

struct ProbeRenderPacket {
    uint8_t  magic;              // 0xD4
    uint8_t  command;            // ProbeCommand
    char     header_text[16];    // z.B. "[ BW METERING ]" oder "[ BURN 1 ARMED ]"
    char     line1_text[16];     // z.B. "Time: 14.5s"
    char     line2_text[16];     // z.B. "Grade: 2.5"
    uint8_t  zone_histogram[11]; // Balkenhöhe der Zonen 0-X (0-255 je Pixel)
    uint8_t  haptic_feedback;    // ProbeHaptic
    uint8_t  display_mode;       // ProbeDisplayMode (Layout-Hinweis für C6)
};
// sizeof(ProbeRenderPacket) = 63 Bytes

#endif