#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_BMP280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "SystemContext.h"
#include "HardwareManager.h"
#include "config.h"

/* =============================================================================
 * SensorManager.h - DUKATIMER BETA (v0.916.3)
 * * Zentrale Verwaltung der Peripherie-Sensoren auf Core 0 (I2C-0 & OneWire).
 * * INTEGRATION:
 * 1. Wireless-Support: Weiche zwischen Kabel-Sonde und Wireless-Paketen.
 * 2. Auto-Gain: Akribische Empfindlichkeitssteuerung mit Frame-Skip.
 * 3. Umwelt-Polling: Hintergrund-Überwachung von Temperatur und Raumhelligkeit.
 * * (+ FIX T8: Thread-Sicherheit durch _stateMutex für State-Machine garantiert)
 * ========================================================================== */

// Phasen der Mess-State-Machine (Präzisionsmessung)
enum MeasurementState
{
    MEAS_IDLE,
    MEAS_PRE_WAIT,
    MEAS_EVAL_GAIN_WAIT, 
    MEAS_EVAL_GAIN_READ, 
    MEAS_PRECISION_WAIT, 
    MEAS_PRECISION_READ, 
    MEAS_POST_WAIT,
    MEAS_DONE
};

class SensorManager
{
public:
    SensorManager(SystemContext *ctx, HardwareManager *hw);
    ~SensorManager(); // Wichtig für RAII Cleanup des Mutex

    void init();
    void update(); 

    /**
     * Startet eine asynchrone Hochpräzisionsmessung (Spot).
     * @param withEnlarger Schaltet das Licht für die Dauer der Messung ein.
     */
    bool startMeasurement(bool withEnlarger);
    void abortMeasurement();

    // Thread-sichere Getter (Snapshot-Prinzip)
    bool isMeasurementRunning() const;
    bool isMeasurementDone() const;
    uint8_t getMeasurementProgress() const;

    // Modifizierender Getter: Liest das Ergebnis und setzt den Zustand sicher auf IDLE zurück
    float getMeasurementResult();

    /**
     * Ermöglicht dem Funk-Modul (BLE/ESP-NOW), externe Lux-Werte einzuspeisen.
     */
    void injectWirelessLux(float lux);

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;

    // --- Hardware Treiber ---
    Adafruit_TSL2591  _tslProbe;
    Adafruit_BMP280   _bmpEnv;
    OneWire           _oneWire;
    DallasTemperature _dsTemp;

    // --- Status Flags ---
    bool _tslOk;
    bool _bmpOk;
    bool _dsOk;

    // --- Wireless Puffer ---
    float         _wirelessLux;
    unsigned long _lastWirelessPacketMs;

    // --- Glättung & Auto-Gain ---
    static constexpr uint8_t FILTER_SIZE = 10;
    float    _luxBuffer[FILTER_SIZE];
    uint8_t  _filterIdx;
    uint8_t  _filterCount;
    float    _lastValidBaseLux;
    uint8_t  _sensorSkipFrames; // Verwirft Frames nach Gain-Wechsel

    // --- Timing ---
    unsigned long _lastLuxPollMs;
    unsigned long _lastTempPollMs;

    // --- State Machine ---
    MeasurementState _measState;
    unsigned long    _measPhaseStartMs;
    float            _measResultLux;
    uint8_t          _measProgress;
    bool             _measRequiresEnlarger;
    tsl2591Gain_t    _currentGain;
    uint8_t          _gainAttempts;

    static constexpr uint32_t PRE_WAIT_MS = 200;
    static constexpr uint32_t POST_WAIT_MS = 400;
    static constexpr uint16_t TSL2591_MAX_COUNT_100MS = 36000; 

    // T8: Mutex zur formalen Thread-Sicherung der State-Machine
    SemaphoreHandle_t _stateMutex = nullptr; 

    // --- Hilfsmethoden ---
    void  _pollAmbientLux(unsigned long now);
    void  _pollTemperatures(unsigned long now);
    float _calculateFilteredLux(float newSample);
    void  _tickMeasurement(unsigned long now);
    void  _finishMeasurement(bool graceful);
};