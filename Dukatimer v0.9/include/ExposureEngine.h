#ifndef EXPOSURE_ENGINE_H
#define EXPOSURE_ENGINE_H

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <Adafruit_TSL2561_U.h>
#include "SystemContext.h"
#include "HardwareManager.h"
#include "config.h"

/* =============================================================================
 * ExposureEngine.h - DUKATIMER BETA (v0.916.1)
 * * ZWECK:
 * - Hochpräzise State-Machine für Belichtungen (Open-Loop & Closed-Loop).
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Thread-Separation: Core 0 (tick/UI) vs Core 1 (Dosis-Integration).
 * 2. RAII-Sicherheit: Alle kritischen Sektionen via MutexGuard geschützt.
 * 3. Lebenszyklus-Härtung: Sicherer Task-Abbau zur Vermeidung von Mutex-Leichen.
 * ========================================================================== */

enum ExposureState
{
    EXP_IDLE,
    EXP_PRE_WAIT,
    EXP_EXPOSING,
    EXP_PAUSED,
    EXP_POST_WAIT,
    EXP_DONE
};

enum ExposureMode
{
    EXPMODE_TIME,
    EXPMODE_DOSE
};

class ExposureEngine
{
public:
    ExposureEngine(SystemContext *ctx, HardwareManager *hw);
    ~ExposureEngine();

    // Initialisierung der Sensoren und des Core-1 Integration-Tasks
    void init();

    // Start-Methoden (werden über die Pull-Brücke im tick() getriggert)
    void startTime(float seconds, uint8_t g, uint8_t b, bool skipPreWait = false);
    void startDose(float targetDose, uint8_t g, uint8_t b, bool skipPreWait = false);

    // Haupt-Zustandsmaschine (muss zyklisch auf Core 0 gerufen werden)
    void tick();

    // Kontroll-Methoden
    void abort();
    void stop();
    bool pause();
    bool resume();

    // Thread-sichere Status-Abfragen
    bool isRunning() const;
    bool isDone() const;
    ExposureState getState() const;
    float getCurrentDose() const;

private:
    SystemContext *_ctx = nullptr;
    HardwareManager *_hw = nullptr;

    Adafruit_TSL2561_Unified _tsl;

    // Synchronisations-Primitive
    SemaphoreHandle_t _stateMutex = nullptr;
    SemaphoreHandle_t _doseMutex = nullptr;

    TaskHandle_t _clTaskHandle = nullptr;
    esp_timer_handle_t _shutoffTimer = nullptr;

    // Atomics für lock-freie Signalwege (Core 1 -> Core 0)
    std::atomic<bool> _initialized{false};
    std::atomic<bool> _stopTask{false}; // NEU: Sicherer Abbruch für Core 1 Task
    std::atomic<bool> _sensorReady{false};
    std::atomic<bool> _shutoffArmed{false};
    std::atomic<bool> _shutoffFired{false};

    ExposureState _state = EXP_IDLE;
    ExposureMode _mode = EXPMODE_TIME;

    float _currentDose = 0.0f;
    float _targetDose = 0.0f;
    float _lastLux = 0.0f;

    uint32_t _exposureStartedMs = 0;
    uint32_t _phaseStartedMs = 0;
    uint32_t _pausedAtMs = 0;

    uint32_t _timeTargetMs = 0;
    uint32_t _remainingTimeMs = 0;
    uint32_t _lastDoseSampleMs = 0;
    uint32_t _lastMetronomeMs = 0;

    uint8_t _green = 0;
    uint8_t _blue = 0;

    bool _splitTimeSequenceActive = false;
    float _splitTimeSecondPhaseS = 0.0f;
    uint8_t _splitTimeSecondGreen = 0;
    uint8_t _splitTimeSecondBlue = 0;

    bool _splitDoseSequenceActive = false;
    float _splitDoseSecondPhase = 0.0f;
    uint8_t _splitDoseSecondGreen = 0;
    uint8_t _splitDoseSecondBlue = 0;

    bool _gracefulStopRequested = false;

    // Task- und Timer-Wrapper (C-Style für ESP32 API)
    static void clTaskWrapper(void *arg);
    static void timerCallbackWrapper(void *arg);

    // Kern-Logik
    void clTaskLoop();
    void onPredictiveShutoff();

    // Interne Helfer
    void beginExposure();
    void finishExposure(bool gracefulStop);
    void enterPostWait(bool gracefulStop);
    void armTimeModeTimer(uint32_t durationMs);
    void disarmPredictiveTimer();
    void updateDoseAndPredictiveTimer(float lux, float dtSeconds);
};

#endif