/*
  Logic_Storage.cpp - EEPROM Management (ESP32)
  Fix: Added missing markDirty() implementation
  Refactor: Papers now managed separately via Logic_Papers
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"
#include "Logic_Papers.h"
#include <EEPROM.h>

extern void validateTimes();
extern void resetTracking();

// Helper für CRC Check
static uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    h ^= data[i];
    h *= 16777619u;
  }
  return h;
}

uint32_t calcCrc(const SettingsObject& s) {
  return fnv1a32((const uint8_t*)&s, sizeof(s) - sizeof(s.crc));
}

// --- FEHLENDE FUNKTION HIER EINGEFÜGT ---
void markDirty() {
  settingsDirty = true;
  lastSettingChange = millis();
  
  // Optional: Debugging
  // Serial.println("Settings marked dirty (unsaved changes)");
}

// saveSettings: Persistiert Einstellungen
void saveSettings() {
  validateTimes();
  globalSet.version = SW_VERSION;
  
  globalSet.t_s  = time_soft; 
  globalSet.t_h  = time_hard;
  globalSet.t_bw = time_bw;
  globalSet.g_bw = grade_bw;
  globalSet.burn_g = burnGrade;
  globalSet.k_s = 0;  globalSet.k_h = 0;  globalSet.k_bw = 0;

  globalSet.pwm_safe  = set_safe;
  globalSet.pwm_focus = set_focus;
  globalSet.pwm_lcd   = set_lcd;
  globalSet.pwm_max   = set_max;

  globalSet.stepMode = (uint8_t)globalStepMode;

  globalSet.crc = 0; 
  globalSet.crc = calcCrc(globalSet);
  
  EEPROM.put(0, globalSet);
  EEPROM.commit(); 
  
  // Save papers separately
  savePapers();
  
  settingsDirty = false;
  Serial.println("Settings saved to EEPROM");
} 

// defaultsSettings: Werkszustand
void defaultsSettings() {
  memset(&globalSet, 0, sizeof(globalSet));
  globalSet.version = SW_VERSION;
  
  time_soft = 6.0; 
  time_hard = 10.0; 
  time_bw = 8.0; 
  grade_bw = 2.5;
  burnGrade = 2.5;

  set_safe = 100; // Standard Safe-Light
  set_focus = 255; 
  set_lcd = 100; 
  set_max = 255;
  
  globalSet.std_time = 8.0;
  globalSet.t_s = time_soft; 
  globalSet.t_h = time_hard; 
  globalSet.t_bw = time_bw;
  globalSet.g_bw = grade_bw;
  globalSet.burn_g = burnGrade;
  
  globalSet.pwm_safe = set_safe; 
  globalSet.pwm_focus = set_focus; 
  globalSet.pwm_lcd = set_lcd; 
  globalSet.pwm_max = set_max;
  
  globalSet.useDoseMode = false;
  globalSet.splitMode = 0; 
  
  globalStepMode = STEP_THIRD;
  globalSet.stepMode = (uint8_t)globalStepMode;
  globalSet.soundMode = SOUND_NORMAL;

  globalSet.crc = 0; 
  globalSet.crc = calcCrc(globalSet);
  
  EEPROM.put(0, globalSet);
  EEPROM.commit(); 
  
  // Reset papers separately
  defaultPapers();
  
  Serial.println("Factory defaults restored");
}

// Fehler-Speicher (Error Log im EEPROM)
static const int EEPROM_ERROR_ADDR = 2000;

void saveErrorState(SystemError err) {
  int errVal = (int)err;
  EEPROM.put(EEPROM_ERROR_ADDR, errVal);
  EEPROM.commit();
}

SystemError loadErrorState() {
  int errVal = 0;
  EEPROM.get(EEPROM_ERROR_ADDR, errVal);
  if (errVal < (int)ERR_NONE || errVal > (int)ERR_I2C_FAST_FAIL) {
    return ERR_NONE;
  }
  return (SystemError)errVal;
}

void clearErrorState() {
  saveErrorState(ERR_NONE);
}

// loadSettings: Lädt Daten beim Start
void loadSettings() {
  EEPROM.get(0, globalSet);
  
  if (globalSet.version != SW_VERSION || calcCrc(globalSet) != globalSet.crc) {
    Serial.println("Settings invalid/old -> Loading defaults");
    defaultsSettings();
  } else {
    time_soft = globalSet.t_s; 
    time_hard = globalSet.t_h; 
    time_bw   = globalSet.t_bw;
    grade_bw  = globalSet.g_bw;
    burnGrade = globalSet.burn_g;
    
    set_safe  = globalSet.pwm_safe; 
    set_focus = globalSet.pwm_focus;
    set_lcd   = globalSet.pwm_lcd;   
    set_max   = globalSet.pwm_max;
    
    if (globalSet.stepMode <= STEP_FULL) globalStepMode = (StepSize)globalSet.stepMode; 
    else globalStepMode = STEP_THIRD;
    
    if (globalSet.std_time < 0.1) globalSet.std_time = 8.0;
    if (globalSet.soundMode > SOUND_OFF) globalSet.soundMode = SOUND_NORMAL;
  }
  
  // Load papers from separate storage
  loadPapers();
  
  validateTimes(); 
  resetTracking();
  updateGradeMath(); // Berechnet pwmValGreen/Blue basierend auf geladener grade_bw
}