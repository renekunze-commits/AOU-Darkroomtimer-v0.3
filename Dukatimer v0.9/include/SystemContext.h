#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <atomic>
#include "types.h"
#include "config.h"
#include "IDataProvider.h"

/* =============================================================================
 * SystemContext.h - DUKATIMER BETA (v0.917.4)
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Single Source of Truth: Verwaltet thread-sicher alle globalen Zustände.
 * 2. ABI-Stabilität: Persistente Structs nutzen strikt uint8_t statt bool.
 * * REVISION: Histogramm-Integration (Bereinigt von Deklarations-Duplikaten)
 * ========================================================================== */

struct ExposureParams
{
    float targetDoseBw = 12.0f;
    float targetDoseSoft = 10.0f;
    float targetDoseHard = 5.0f;
    float grade = 2.5f;
    bool doseModeActive = true;
};

struct HardwareStatus
{
    float headLux = 0.0f;
    float baseLux = 0.0f;
    float tempAlu = 25.0f;
    float tempRoom = 22.0f;
    float liveDose = 0.0f;
    float liveTime = 0.0f;
    bool overheatActive = false;
    SystemError lastError = ERR_NONE;
};

struct AppSharedState
{
    Mode activeStateMode = MODE_BW; // Tag / Discriminator

    union
    {
        struct
        {
            int activeStep;
            float lastLux;
        } testStrip;

        struct
        {
            uint8_t wizardState;
        } calibration;

        struct
        {
            float remainingTime;
            bool isRunning;
        } exposure;

        struct
        {
            float refLux;
            float currentDensity;
            bool isZeroed;
        } densitometer;

        struct
        {
            float targetDose;
            float remainingDose;
            bool isRunning;
        } bwDose;

        struct
        {
            float targetSoft;
            float targetHard;
            float remainingDose;
            uint8_t filterState;
            bool isRunning;
            bool waitingForUser;
        } sgDose;

        struct
        {
            float baseTime;
            int fStopTicks;
            float calculatedTime;
            float remainingTime;
            bool isRunning;
        } bwFStop;

        struct
        {
            uint8_t selectedZone;
            float baseDose;
            float calculatedDose;
            bool isRunning;
        } zoneMode;

        struct
        {
            uint8_t state;
            uint8_t currentStep;
            uint8_t totalSteps;
            float stepDose;
            float remainingDose;
            bool isRunning;
        } preflash;

        struct
        {
            char line1[17];
            char line2[17];
        } setup;

        struct
        {
            float burnEv;
            float burnGrade;
            float baseDose;
            float calcBurnDose;
            int stepMode;
            bool isRunning;
        } burn;
    };
};

struct WorkflowFlags
{
    Mode currentMode = MODE_BW;
    bool isExposureRunning = false;
    bool isMeasuring = false;
    uint8_t measProgress = 0;
    bool bwAutoPending = false;
    bool sgAutoPending = false;
    bool isCalibrating = false;
    bool modePreviewActive = false;
    Mode previewMode = MODE_BW;
    float pendingDose = 0.0f;
    float pendingGrade = 2.5f;
    AppSharedState appState = {};

    // --- HISTOGRAMM FÜR WIRELESS/OLED ---
    uint8_t zoneHistogram[11];
    int8_t lastMeasuredZone; // Für einfaches Undo
};

struct __attribute__((packed)) SystemPreferences
{
    uint8_t pwmSafe = 100;
    uint8_t pwmFocus = 255;
    uint8_t pwmLcd = 100;
    uint8_t pwmMax = 255;
    float baseDarkLux = 0.0f;
    float probeDarkLux = 0.0f;
    uint8_t soundMode = 1;
    uint8_t stepMode = 2;
    float bwTargetZone = 8.0f;
    uint8_t useWirelessProbe = 1;
    float stdTime = 15.0f;
    uint8_t splitModeAuto = 1;
};

class SystemContext : public IDataProvider
{
public:
    SystemContext();
    ~SystemContext();

    bool getExposure(ExposureParams &out);
    bool getStatus(HardwareStatus &out);
    bool getFlags(WorkflowFlags &out);
    bool getPreferences(SystemPreferences &out);

    bool setBWDose(float dose);
    bool setSplitDoses(float soft, float hard);
    bool setGrade(float grade);
    bool setDoseMode(bool active);
    bool setPreferences(const SystemPreferences &prefs);
    bool setMode(Mode mode);
    bool setModePreview(bool active, Mode preview);
    bool setExposureState(bool running);
    bool setMeasuringState(bool measuring);
    bool updateMeasuringProgress(uint8_t progress);
    bool setBWPending(bool pending, float suggestedDose);
    bool setSGPending(bool pending, float suggestedDose, float suggestedGrade);

    // Zeitgesteuerte Split-Grade-Belichtung (SGTimeApp)
    bool setSGTimePending(float softS, float hardS);
    void triggerAbort();
    bool setAppState(const AppSharedState &state);
    bool setLastError(SystemError err);

    bool updateSensors(float head, float base, float tempA, float tempR);
    bool updateLiveDose(float currentDose);
    bool updateLiveTime(float seconds);

    // --- HISTOGRAMM METHODEN ---
    void clearHistogram();
    void addHistogramSpot(uint8_t zone);
    void undoLastHistogramSpot();

    bool isDirty() const override;
    void clearDirty() override;
    void markDirty() override;
    bool serialize(uint8_t **buffer, size_t *length, uint16_t *outHash) override;
    bool deserialize(const uint8_t *buffer, size_t length, uint16_t hash) override;
    const char *getFileName() const override;

private:
    SemaphoreHandle_t _mutex;
    std::atomic<bool> _isDirty{false};
    ExposureParams _exp;
    HardwareStatus _hw;
    WorkflowFlags _flags;
    SystemPreferences _prefs;
};

static_assert(sizeof(SystemPreferences) == 24, "CRITICAL: SystemPreferences size changed! Migration logic must be updated.");