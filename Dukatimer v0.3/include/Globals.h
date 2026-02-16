#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_TSL2561_U.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "Types.h"
#include "Config.h"
#include <Adafruit_BMP280.h>
#include <esp_heap_caps.h>          // heap_caps_malloc() für PSRAM-Allokation

// =============================================================================
// GLOBALER ZUSTAND (Globals.h)
//
// Zweck:
// - Definiert die zentralen Hardware-Instanzen (Sensoren, Displays, LEDs)
// - Stellt globale Zustandsvariablen (Timer, Messergebnisse, UI Overlay) zur
//   Verfügung, die von verschiedenen Modulen gemeinsam genutzt werden.
// Hinweise:
// - Verwende möglichst Getter/Setter in neuen Modulen statt direktem Zugriff,
//   aber für das bestehende, stark gekoppeltes Codebase sind diese externen
//   Variablen pragmatisch. Neue Interfaces können die Abhängigkeiten reduzieren.
// =============================================================================
// HARDWARE INSTANZEN
// =============================================================================
// I2C Busse
extern TwoWire& I2C_SLOW;   // Bus 0 (Alias für Wire)
extern TwoWire I2C_FAST;    // Bus 1 (Eigene Instanz)

// Sensoren
extern Adafruit_TSL2591 tslBase;    // Vormessung (Bus 0)
extern Adafruit_TSL2561_Unified tslLive;  // Live-Regelung (Bus 1)
extern rgb_lcd lcd;  // Grove RGB LCD 16x2
extern Adafruit_BMP280* bmpPtr;  // BMP280 Pointer (wird in setup() erstellt)
extern bool bmpOK;

// NeoPixel Matrix (256 LEDs = 16x16) - Buffer in PSRAM für Stromsparung im SRAM
extern Adafruit_NeoPixel pixels;

// Encoder Objekte
extern ESP32Encoder encSoft;
extern ESP32Encoder encHard;
extern ESP32Encoder encGrade;

// Taster Zustände (aus HW_Input)
extern BtnState sEnter;
extern BtnState sBack;
extern BtnState btnStart;
extern BtnState btnRedLed; // Pin 21 - NeoPixel Red LED
extern BtnState btnWhiteLed; // Pin 14 - NeoPixel White LED

// Screen off override for btSCR logic
extern bool screenOffOverride;

// =============================================================================
// SYSTEM STATE
// =============================================================================
extern SettingsObject globalSet;

// =============================================================================
// PSRAM-OPTIMIERUNG: PaperBank (20 Profile × ~120 Bytes ≈ 2.4 KB)
// Wird in den externen PSRAM ausgelagert, um internen SRAM für zeitkritische
// Tasks (I2C-Interrupts, FreeRTOS-Stacks, WiFi/ESP-NOW) freizuhalten.
// Das Makro "paperBank" leitet alle bestehenden Zugriffe automatisch über
// den Pointer um → kein Code muss angepasst werden.
// =============================================================================
extern PaperBank* paperBankPtr;
#define paperBank (*paperBankPtr)

extern bool settingsDirty;
extern unsigned long lastSettingChange;

extern bool tslBaseOK;
extern bool tslLiveOK;
extern bool lcdOK;
extern bool nextonOK;
extern bool tempSensorOK;
extern bool neoPixelOK;
extern bool overheatLock;
extern double tempAlu;
extern double tempRoom;
extern int currentGainIdx;

// Software toggles for lights
extern bool whiteLatch;
extern bool safeLatch;
extern bool sgShiftModeDensity;

// Tracking & UI Helper
extern double trackDensityEV;
extern double trackGradeSteps;
extern String overlayText;
extern unsigned long infoEndTime;
extern int pendingPaperSlot;

// Hilfstabellen
extern int multival[11][3];
extern double paperSpeed[11];

// Wizards & Unter-Modi
extern CalStep calState;
extern bool calAbort;

extern DensitometerState densState;
extern DensSubMode densSub;
extern double densRefLux;
extern double densBaseFog;
extern double zone8TargetNet;

// Teststrip / TS
extern TSState ts;
extern TSChannel tsCh;
extern uint8_t tsN;
extern double tsEv;
extern uint8_t tsK;
extern double tsA[10];
extern double tsSum;
extern bool tsActiveExposure;

// Measurement
extern MeasureMode measureMode;
extern unsigned long measureUiSinceMs;
extern double measSoftSum;
extern int measSoftCount;
extern double measHardSum;
extern int measHardCount;
extern double measBWSum;
extern int measBWCount;

// Misc globals
extern bool isPaused;
extern bool isMeasuring;
extern bool setupMenuActive;

// =============================================================================
// WIRELESS SENSOR (ESP-NOW)
// =============================================================================
extern bool useWirelessProbe;                    // Die Einstellung (Menu)
extern volatile double remoteLux;                // Der letzte empfangene Wert
extern volatile unsigned long lastRemotePacketMs; // Zeitstempel des letzten Pakets
extern void initWireless();                      // Init Funktion

// Task / synchronization
extern SemaphoreHandle_t gTimerMutex;
extern SemaphoreHandle_t gPixelMutex;
extern void initClosedLoopTask();

// =============================================================================
// PSRAM HILFSFUNKTIONEN
// =============================================================================
// Initialisiert PSRAM-basierte Datenstrukturen (PaperBank).
// Muss früh in setup() aufgerufen werden, BEVOR loadSettings()/initPapers().
extern void initPSRAMStructures();
// Gibt PSRAM/Heap-Diagnose auf Serial aus (Debug-Hilfe beim Booten).
extern void logMemoryInfo();

// Timer helpers
extern void handleTimer();
extern void startTimer();
extern void stopTimer();

// =============================================================================
// TIMER STATUS & MATHE
// =============================================================================
extern unsigned long starttime;
extern double time_soft;
extern double time_hard;
extern double time_bw;
extern double grade_bw;
extern double burnEv;
extern double burnGrade;
extern uint8_t set_safe;
extern uint8_t set_focus;
extern uint8_t set_lcd;
extern uint8_t set_max;
extern StepSize globalStepMode;

extern Mode currentMode;
extern BurnMode burnMode;
extern SplitState splitState;

// PWM Werte für Matrix (aus Logic_Math)
extern uint8_t pwmValGreen;
extern uint8_t pwmValBlue;

// Fehler-Codes für das System
enum SystemError {
    ERR_NONE = 0,
    ERR_MUTEX_TIMEOUT,   // Mutex konnte nicht rechtzeitig genommen werden
    ERR_SENSOR_LOST,    // TSL2561 antwortet nicht mehr
    ERR_TASK_OVERLOAD,  // Task braucht länger als 10ms pro Zyklus
    ERR_I2C_SLOW_FAIL,  // Bus 0 (LCD, TSL2591)
    ERR_I2C_FAST_FAIL   // Bus 1 (TSL2561)
};

extern volatile SystemError lastSystemError;
void logError(SystemError err, String context);
void saveErrorState(SystemError err);
SystemError loadErrorState();
void clearErrorState();
extern volatile bool softAbortActive;
extern volatile unsigned long softAbortUntilMs;

// =============================================================================
// FUNKTIONS PROTOTYPEN
// =============================================================================
extern void startCalibrationWizard();
extern void runCalibrationWizard();
extern void startTestStripMode();
extern void runTestStripLoop(char key);

// --- INPUT & LICHT (NEU) ---
extern void initInput();
extern void handleInput();
extern void handleLights(); 
// Brücken-Funktion für alte Mode-Dateien:
extern void PaintLED(int r, int g, int b); 
extern bool updateButton(BtnState &st, int pin, unsigned long now); // Helper
extern bool checkButtonPress(BtnState &st, int pin); // Entprellung

// --- LOGIK & MATHE ---
extern void updateGradeMath();
extern void validateTimes();
extern void resetTracking();
extern double getEffectiveBurnTime();
extern long secondsToUnits(double s, int tgl); // Fehlt oft in TestStrip
extern double clampDouble(double val, double minV, double maxV);

// --- SYSTEM ---
extern void initDisplays();
extern void updateNextionUI(bool force);
extern void smartLCD(const char* l1, const char* l2);
extern void smartLCD(const String& l1, const String& l2);
extern void triggerInfo();
extern void markDirty();
extern void saveSettings();
extern void loadSettings();
extern void startMeteringSession();
extern bool isMeteringActive();
extern void handleMeteringSession(bool addSpot, bool saveApply, bool toggleChannel, bool resetAll, bool cancel);

// Setup handler available to call from UI
extern void handleSetup();

// Misc Helpers
extern void handleLCDBacklight();
extern char getNextionKey();
extern SpotMeas readSpot();
extern double takeAveragedLux(uint8_t samples, uint16_t delayMs);
extern void wdt_reset();

// --- SOUND ---
extern void beepNav();
extern void beepOk();
extern void beepValue();
extern void beepClick();
extern void beepWarnLong();
extern void beepDone();
extern void beepStartPattern(); // Für TestStrip
extern void beepEndPattern();
extern void beepWizardSave();

// Deklaration der Funktion enterDensMode
void enterDensMode();

void defaultsSettings();

#define MODE_DENSITOMETER 2 // Densitometer-Modus

#endif