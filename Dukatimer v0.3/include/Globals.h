#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_TSL2561_U.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <NeoPixelBus.h>
#include "Types.h"
#include "Config.h"
#include <Adafruit_BMP280.h>
#include <DallasTemperature.h>
#include <esp_heap_caps.h>
#include <Adafruit_BME280.h>

extern SemaphoreHandle_t gTimerMutex;      
extern SemaphoreHandle_t gPixelMutex;      
extern SemaphoreHandle_t xI2CMutex;        
extern SemaphoreHandle_t xShadowMutex;     
extern SemaphoreHandle_t xNexMutex;

extern QueueHandle_t xInputQueue;
extern QueueHandle_t xSoundQueue;
extern LCDShadow lcdShadow;

extern Adafruit_TSL2591 tslBase;
extern Adafruit_TSL2561_Unified& tslLive;
extern Adafruit_TSL2561_Unified tslHead;
extern rgb_lcd lcd;
extern NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixels;

// ROOT CAUSE FIX: Alle 4 Encoder global deklarieren
extern ESP32Encoder encSoft;
extern ESP32Encoder encHard;
extern ESP32Encoder encGrade;
extern ESP32Encoder encMode; 

extern Adafruit_BMP280* bmpPtr;
extern Adafruit_BME280* bmePtr;
extern DallasTemperature sensors;
extern bool isBME280;

extern bool tslBaseOK, tslLiveOK, tslHeadOK, bmpOK, bmeOK, lcdOK, neoPixelOK, tempSensorOK;
extern volatile bool overheatLock;
extern double tempAlu, tempAmbient, tempRoom, pressRoom, humRoom;
extern int currentGainIdx;

extern bool hwSwitchDoseMode;    
extern double target_dose, current_dose, spectral_ratio;
extern double dose_bw, dose_soft, dose_hard, targetDoseSoft, targetDoseHard;
extern float time_bw, grade_bw, time_soft, time_hard, burnEv;
extern double burnGrade;

// ROOT CAUSE FIX: starttime MUSS volatile sein (K01 Bug)
extern volatile unsigned long starttime;
extern volatile Mode currentMode;
extern SplitState splitState;
extern CalStep calState;
extern BurnMode burnMode;
extern StepSize globalStepMode;

extern uint8_t pwmValGreen, pwmValBlue;
extern float baseFlux;
extern int multival[11][3];
extern double paperSpeed[11];

extern volatile bool statusEnlargerOn, statusSafeOn, isRoomDarknessActive;
extern volatile bool safeLatch, whiteLatch, roomLatch, measurementOverrideActive, screenOffOverride, bootScreenActive, setupMenuActive;
extern volatile bool isPaused, isMeasuring;

// FIX A1: Zentrale Licht-Sperrlogik
extern volatile bool lightOperationActive;

extern volatile bool softAbortActive;
extern volatile unsigned long softAbortUntilMs;

extern TSState ts;
extern uint8_t tsN;
extern double tsEv;
extern double tsA[10];
extern uint8_t tsK;
extern TSChannel tsCh;
extern double tsSum;
extern bool tsActiveExposure;

extern DensitometerState densState;
extern DensSubMode densSub;
extern double densRefLux, densBaseFog, zone8TargetNet;
extern double baseDarkLux, probeDarkLux;

extern double paper_iso_p, paper_iso_r;
extern char activePaperName[32];
extern PaperBank* paperBankPtr;
#define paperBank (*paperBankPtr)

extern volatile uint32_t totalDroppedPackets, probeEventOverruns;
extern volatile bool probeConnected, probeEventPending;
extern volatile uint8_t probeLastEvent;
extern volatile float probeLuxG0, probeLuxG5;
extern volatile double remoteLux;
extern volatile unsigned long lastRemotePacketMs;
extern bool probeFlashActive;

extern SettingsObject globalSet;
extern uint8_t set_safe, set_focus, set_lcd, set_max;
extern bool useWirelessProbe;
extern uint8_t currentZoneHistogram[11];
extern MeasureFocus currentMeasureFocus;
extern float timer_base_seconds, calibBaseLuxG0, calibBaseLuxG5, calibTimeSeconds;
extern double trackDensityEV, trackGradeSteps;
extern String overlayText;
extern unsigned long infoEndTime;
extern int pendingPaperSlot;
extern double measSoftSum, measHardSum, measBWSum;
extern int measSoftCount, measHardCount, measBWCount;
extern volatile bool settingsDirty;
extern volatile unsigned long lastSettingChange;
// BETA-FIX: Schritt 3 - Signalisierung fuer anstehende Messwert-Uebernahme.
extern volatile bool bwAutoPending;

void uiUpdateLCD(const char* l1, const char* l2, uint8_t progress = 0);
void uiTriggerBeep(SoundID id);
void processLCDShadow();
void processSoundQueue();

bool allowUiBeeps();
void beepPattern(const BeepSeg* segs, size_t n);
void beepNav(); void beepValue(); void beepClick(); void beepOk();
void beepHint(); void beepWarnLong(); void beepWizardStep();
void beepWizardSave(); void beepStartPattern(); void beepEndPattern();
void beepDone(); void beepAlarm();

void HW_InitLights();
void HW_SetBlackout(bool active);
void HW_SetSafelight(bool active);
void HW_SetFocus(bool active);
void HW_SetEnlargerNeoPixel(uint8_t r, uint8_t g, uint8_t b);
void HW_EmergencyShutoff();

extern void initInput();
extern void handleInput();
extern void handleLights();
extern void handleLCDBacklight();
extern void startTimer();
extern void stopTimer();
extern void refreshDisplayVariables();
extern void applySpotMeasurement(double boardLux);
extern void modifyExposureByEV(double evDelta);
extern void wdt_reset();

void triggerInfo(); void saveSettings(); void loadSettings();
void smartLCD(const char* l1, const char* l2); void smartLCD(String l1, String l2);
void cancelAsyncSpot(); void handleExposureMetronome(unsigned long elapsedMs);
bool tickAsyncSpot(); double getAsyncSpotResult();
void enterNextionUploadMode();

bool checkButtonPress(BtnState &st, int pin);
bool updateButton(BtnState &st, int pin, unsigned long now);
void updateNextionUI(bool force);
void initWireless(); void clearErrorState();
bool startAsyncSpot(uint8_t samples, uint16_t intervalMs, uint16_t settleMs);
bool isAsyncSpotBusy();

extern BtnState sEnter, sBack, btnStart, btnEnc3, btnRedLed, btnWhiteLed;
extern volatile SystemError lastSystemError;
extern void saveErrorState(SystemError err);

#endif