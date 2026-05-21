#ifndef DUKATIMER_TYPES_H
#define DUKATIMER_TYPES_H
#pragma message("DUKATIMER Types.h included")

#include <Arduino.h>

struct BeepSeg {
  int f;    
  int d;    
  int p;    
  bool prio;
};

struct PaperProfile {
  char name[32];     
  bool useIsoMath;   
    bool isFixedGrade;
    float fixedGradeValue;
  double isoP;       
  double isoR;       
  double gradeK_Soft[11];
  double gradeK_Hard[11];
  double Ksoft;      
  double Khard;      
  double Kbw;        
  bool calibrated;   
  bool flashEnable;
  double flashThreshS;
  bool flashCalibrated;
  int flashLevel;
  int flashColor;
  double flashFactor;
};

struct PaperBank {
  uint16_t version;     
  uint8_t activeIndex;  
  PaperProfile profiles[20]; 
};

struct SettingsObject {
  uint16_t version;
  double t_s;      
  double t_h;      
  double t_bw;     
  double g_bw;     
  double burn_g;   
  double k_s, k_h, k_bw;
  uint8_t pwm_safe;   
  uint8_t pwm_focus;  
  uint8_t pwm_lcd;    
  uint8_t pwm_max;    
  bool useWirelessProbe; // Persistent speichern
  bool useDoseMode;   
  double std_time;
  uint8_t splitMode;    
  uint8_t stepMode;     
  uint8_t soundMode; 
  double base_dark_lux;
  double probe_dark_lux;
    // BETA-FIX: [Phase 2] Speichert die Zielzone fuer die automatische
    // BW-Belichtungsberechnung persistent im Setup-Menue.
    // Standardwert (Lichter-Prioritaet) ist 8.0.
    float bwTargetZone;
  uint16_t crc;
};

struct BtnState {
    bool lastState;       
    bool isPressed;       
    unsigned long lastDebounceTime;
    unsigned long pressStartMs;
    unsigned long lastRepeatMs;
    unsigned long intervalMs;
    unsigned long longPressDuration;
};

struct SpotMeas {
  double lux;
  double temp;
  bool ok;     
  uint16_t ch0;
  uint16_t ch1;
};

enum Mode {
    MODE_BW,
    MODE_SG,
    MODE_TESTSTRIP,
    MODE_BURN,
    MODE_CALIB,
    MODE_DENS,
    MODE_SETUP,
    MODE_TIMER,
    MODE_BRIDGE
};

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

enum SystemError : uint8_t {
    ERR_NONE = 0,
    ERR_I2C_TIMEOUT = 1,
    ERR_SENSOR_DISCONNECTED = 2,
    ERR_THERMAL_OVERHEAT = 3,
    ERR_STORAGE_CORRUPTED = 4,
    ERR_WIRELESS_DROPPED = 5,
    ERR_I2C_FAST_FAIL = 6,
    ERR_MUTEX_TIMEOUT = 7,
    ERR_MATH_INVALID = 8
};

enum LedMode : uint8_t { LED_OFF = 0, LED_GREEN, LED_BLUE, LED_FOCUS, LED_SAFELIGHT };

#define REMOTE_MAGIC 0xD4        
#define ESPNOW_CHANNEL 1

struct WirelessPacket {
    uint8_t magic;    
    uint32_t seq;     
    float lux;        
};

enum ProbeEventType : uint8_t {
    EVT_NONE        = 0x00,
    EVT_T2_CLICK    = 0x01,  
    EVT_T1_CLICK    = 0x02,  
    EVT_ENC_CLICK   = 0x03,  
    EVT_ENC_UP      = 0x04,  
    EVT_ENC_DOWN    = 0x05,  
    EVT_ENC_LONG    = 0x06,
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

// Protokoll-Grounding: Diese Groessen muessen auf beiden MCU-Seiten exakt gleich sein,
// damit keine stillen ABI/Padding-Abweichungen die Funkkommunikation brechen.
static_assert(sizeof(ProbeEventPacket) == 16, "ProbeEventPacket ABI mismatch: erwartet 16 Byte");
static_assert(sizeof(ProbeRenderPacket) == 63, "ProbeRenderPacket ABI mismatch: erwartet 63 Byte");
static_assert(sizeof(WirelessPacket) == 12, "WirelessPacket ABI mismatch: erwartet 12 Byte");

// =============================================================================
// INPUT EVENT SYSTEM (RTOS Queue basiert)
// =============================================================================
typedef enum {
    EVT_START_PRESSED,
    EVT_START_LONG,
    EVT_ENTER_PRESSED,
    EVT_DENS_REQUEST,
    EVT_BACK_PRESSED,
    EVT_GRADE_PRESSED,
    EVT_MODE_PRESSED,
    EVT_ENC_SOFT,
    EVT_ENC_HARD,
    EVT_ENC_GRADE,
    EVT_ENC_MODE,
    EVT_DOSE_SWITCH_CHANGE,
    // H02 FIX: Long-Press Event Types
    EVT_ENTER_LONG,     // Enc 2 lang gehalten
    EVT_BACK_LONG,      // Enc 1 lang gehalten
    EVT_GRADE_LONG      // Enc 3 lang gehalten
} InputEventType;

typedef struct {
    InputEventType type;
    int32_t value;      
    uint32_t timestamp;
} InputEvent;

// =============================================================================
// SOUND SYSTEM (Queue basiert)
// =============================================================================
typedef enum {
    SND_NAV, SND_OK, SND_BACK, SND_VALUE, SND_WARN, SND_DONE,
    SND_START_PATTERN, SND_END_PATTERN, SND_CLICK, SND_HINT,
    SND_LIMIT,
    SND_PHASE_READY, SND_AUTO_PROP, SND_CAL_STEP, SND_ALARM
} SoundID;

// =============================================================================
// LCD SHADOW BUFFER (Display-Entkopplung)
// =============================================================================
typedef struct {
    char line1[17];
    char line2[17];
    uint8_t progress;   
    bool dirty;         
} LCDShadow;
#endif