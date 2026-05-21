#include "SensorManager.h"
#include <cmath>

/* =============================================================================
 * SensorManager.cpp - DUKATIMER BETA (v0.916.4)
 * * Implementierung des Sensor-Layers für Core 0.
 * (+ FIX: Mutex-Loch in update() geschlossen. Die State-Machine ist nun 
 * auch beim zyklischen Tick zu 100% vor Preemption geschützt.)
 * ========================================================================== */

namespace
{
    // Sicheres RAII-Pattern für FreeRTOS Mutexe
    class MutexGuard
    {
        SemaphoreHandle_t _mutex;
        bool _acquired;
    public:
        MutexGuard(SemaphoreHandle_t m, TickType_t t)
            : _mutex(m), _acquired(m != nullptr && xSemaphoreTake(m, t) == pdTRUE) {}
        ~MutexGuard() { if (_acquired) xSemaphoreGive(_mutex); }
        bool isAcquired() const { return _acquired; }
    };
}

SensorManager::SensorManager(SystemContext *ctx, HardwareManager *hw)
    : _ctx(ctx), _hw(hw), 
      _tslProbe(2591), _oneWire(PIN_ONEWIRE), _dsTemp(&_oneWire),
      _tslOk(false), _bmpOk(false), _dsOk(false),
      _wirelessLux(0.0f), _lastWirelessPacketMs(0),
      _filterIdx(0), _filterCount(0), _lastValidBaseLux(0.0f),
      _sensorSkipFrames(0), _lastLuxPollMs(0), _lastTempPollMs(0),
      _measState(MEAS_IDLE), _measPhaseStartMs(0), _measResultLux(0.0f), 
      _measProgress(0), _measRequiresEnlarger(false),
      _currentGain(TSL2591_GAIN_MED), _gainAttempts(0)
{
    _stateMutex = xSemaphoreCreateMutex();
    for (int i = 0; i < FILTER_SIZE; i++) _luxBuffer[i] = 0.0f;
}

SensorManager::~SensorManager()
{
    if (_stateMutex) vSemaphoreDelete(_stateMutex);
}

void SensorManager::init()
{
    _dsTemp.begin();
    _dsTemp.setWaitForConversion(false); 
    _dsTemp.requestTemperatures();
    _dsOk = (_dsTemp.getDeviceCount() > 0);

    if (_hw && _hw->takeI2C())
    {
        _bmpOk = _bmpEnv.begin(ADDR_BMP280);
        if (_bmpOk) {
            _bmpEnv.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                                Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                                Adafruit_BMP280::STANDBY_MS_500);
        }

        _tslOk = _tslProbe.begin(_hw->getI2C0Bus());
        if (_tslOk) {
            _tslProbe.setGain(TSL2591_GAIN_MED);
            _tslProbe.setTiming(TSL2591_INTEGRATIONTIME_100MS);
        }
        _hw->giveI2C();
    }
}

void SensorManager::injectWirelessLux(float lux)
{
    _wirelessLux = lux;
    _lastWirelessPacketMs = millis();
}

void SensorManager::update()
{
    if (!_ctx || !_hw) return;

    WorkflowFlags flags;
    if (_ctx->getFlags(flags) && flags.isExposureRunning) return;

    unsigned long now = millis();

    // HÄRTUNG (Double-Check): Wir evaluieren den Status sicher unter Mutex.
    bool isRunning = false;
    {
        MutexGuard lock(_stateMutex, pdMS_TO_TICKS(10));
        isRunning = (lock.isAcquired() && _measState != MEAS_IDLE && _measState != MEAS_DONE);
    }

    if (isRunning) {
        // HÄRTUNG: Die Abarbeitung der State-Machine MUSS gelockt sein,
        // damit kein Aufruf von abortMeasurement() dazwischenfunken kann!
        MutexGuard lock(_stateMutex, pdMS_TO_TICKS(50));
        if (lock.isAcquired()) {
            _tickMeasurement(now);
        }
        // Fortschritt asynchron an UI pushen (ohne Mutex, da Thread-sicher im Context)
        _ctx->updateMeasuringProgress(_measProgress);
        return; 
    }

    // Polling-Pfad (Umwelt/Hintergrund) - läuft nur, wenn keine Messung aktiv ist
    if (now - _lastLuxPollMs >= 200UL) {
        _lastLuxPollMs = now;
        _pollAmbientLux(now);
    }

    if (now - _lastTempPollMs >= 2000UL) {
        _lastTempPollMs = now;
        _pollTemperatures(now);
    }
}

void SensorManager::_pollAmbientLux(unsigned long now)
{
    SystemPreferences prefs;
    if (!_ctx->getPreferences(prefs)) return;

    float currentLux = 0.0f;
    bool sourceOk = false;

    if (prefs.useWirelessProbe != 0) {
        if (now - _lastWirelessPacketMs < 2000UL) {
            currentLux = _wirelessLux;
            sourceOk = true;
        }
    } 
    else if (_tslOk && _hw->takeI2C()) {
        uint32_t lum = _tslProbe.getFullLuminosity();
        uint16_t full = lum & 0xFFFF;
        uint16_t ir   = lum >> 16;

        if (full > 60000 && _currentGain > TSL2591_GAIN_LOW) {
            _currentGain = static_cast<tsl2591Gain_t>(_currentGain - 1);
            _tslProbe.setGain(_currentGain);
            _sensorSkipFrames = 3;
        } else if (full < 500 && _currentGain < TSL2591_GAIN_HIGH) {
            _currentGain = static_cast<tsl2591Gain_t>(_currentGain + 1);
            _tslProbe.setGain(_currentGain);
            _sensorSkipFrames = 3;
        }

        if (_sensorSkipFrames > 0) {
            _sensorSkipFrames--;
            _hw->giveI2C();
            return; 
        }

        currentLux = _tslProbe.calculateLux(full, ir);
        _hw->giveI2C();
        sourceOk = !std::isnan(currentLux);
    }

    if (sourceOk) {
        if (currentLux < 0.0001f) currentLux = 0.0001f;
        _lastValidBaseLux = _calculateFilteredLux(currentLux);
        
        HardwareStatus status;
        if (_ctx->getStatus(status)) {
            _ctx->updateSensors(status.headLux, _lastValidBaseLux, status.tempAlu, status.tempRoom);
        }
    }
}

void SensorManager::_tickMeasurement(unsigned long now)
{
    // WICHTIG: Diese Methode wird nun garantiert vom _stateMutex geschützt aufgerufen!
    switch (_measState)
    {
        case MEAS_PRE_WAIT:
            _measProgress = 5;
            if (now - _measPhaseStartMs >= PRE_WAIT_MS) {
                if (_hw->takeI2C()) {
                    _currentGain = TSL2591_GAIN_MED;
                    _tslProbe.setGain(_currentGain);
                    _tslProbe.setTiming(TSL2591_INTEGRATIONTIME_100MS);
                    _hw->giveI2C();
                }
                _measPhaseStartMs = now;
                _measState = MEAS_EVAL_GAIN_WAIT;
            }
            break;

        case MEAS_EVAL_GAIN_READ:
            if (_hw->takeI2C()) {
                uint32_t lum = _tslProbe.getFullLuminosity();
                uint16_t ch0 = lum & 0xFFFF;
                bool gainOptimal = true;
                _gainAttempts++;

                if (ch0 > TSL2591_MAX_COUNT_100MS && _currentGain != TSL2591_GAIN_LOW) { _currentGain = TSL2591_GAIN_LOW; gainOptimal = false; }
                else if (ch0 < 300 && _currentGain == TSL2591_GAIN_LOW) { _currentGain = TSL2591_GAIN_MED; gainOptimal = false; }
                else if (ch0 < 300 && _currentGain == TSL2591_GAIN_MED) { _currentGain = TSL2591_GAIN_HIGH; gainOptimal = false; }
                else if (ch0 < 50 && _currentGain == TSL2591_GAIN_HIGH) { _currentGain = TSL2591_GAIN_MAX; gainOptimal = false; }

                if (_gainAttempts > 4) gainOptimal = true;

                if (!gainOptimal) {
                    _tslProbe.setGain(_currentGain);
                    _hw->giveI2C();
                    _measPhaseStartMs = millis();
                    _measState = MEAS_EVAL_GAIN_WAIT; 
                } else {
                    _tslProbe.setTiming(TSL2591_INTEGRATIONTIME_600MS);
                    _hw->giveI2C();
                    _measPhaseStartMs = millis();
                    _measState = MEAS_PRECISION_WAIT;
                }
            }
            break;

        case MEAS_PRECISION_READ:
            _measProgress = 90;
            if (_hw->takeI2C()) {
                uint32_t lum = _tslProbe.getFullLuminosity();
                _measResultLux = _tslProbe.calculateLux(lum & 0xFFFF, lum >> 16);
                _tslProbe.setGain(TSL2591_GAIN_MED);
                _tslProbe.setTiming(TSL2591_INTEGRATIONTIME_100MS);
                _hw->giveI2C();
            }
            _hw->allLightsOff();
            _measPhaseStartMs = now;
            _measState = MEAS_POST_WAIT;
            break;

        case MEAS_PRECISION_WAIT:
            {
                float ratio = static_cast<float>(now - _measPhaseStartMs) / 650.0f;
                _measProgress = 20 + static_cast<uint8_t>(fminf(ratio, 1.0f) * 60.0f);
            }
            if (now - _measPhaseStartMs >= 650UL) _measState = MEAS_PRECISION_READ;
            break;

        case MEAS_EVAL_GAIN_WAIT:
            if (now - _measPhaseStartMs >= 120UL) _measState = MEAS_EVAL_GAIN_READ;
            break;

        case MEAS_POST_WAIT:
            if (now - _measPhaseStartMs >= POST_WAIT_MS) _finishMeasurement(true);
            break;

        default: break;
    }
}

bool SensorManager::startMeasurement(bool withEnlarger)
{
    MutexGuard lock(_stateMutex, pdMS_TO_TICKS(50));
    if (!lock.isAcquired()) return false;

    if (_measState != MEAS_IDLE && _measState != MEAS_DONE) return false;
    if (!_tslOk) return false;

    _measRequiresEnlarger = withEnlarger;
    _measResultLux = 0.0f;
    _measProgress = 0;
    _gainAttempts = 0;

    _hw->allLightsOff();
    if (_measRequiresEnlarger) _hw->setEnlarger(true);
    _hw->setSafelight(false);

    _ctx->setMeasuringState(true);
    _hw->setExposureLock(true); 
    
    _measPhaseStartMs = millis();
    _measState = MEAS_PRE_WAIT;
    return true;
}

void SensorManager::abortMeasurement() 
{ 
    bool needAbort = false;
    {
        MutexGuard lock(_stateMutex, pdMS_TO_TICKS(10));
        if (lock.isAcquired() && _measState != MEAS_IDLE && _measState != MEAS_DONE) {
            needAbort = true;
        }
    }
    
    if (needAbort) {
        // Der Aufruf von _finishMeasurement greift intern wieder den Mutex für die 
        // Variablen-Aktualisierung. Daher geben wir den Mutex im Scope vorher frei 
        // (verhindert Deadlock bei nicht-rekursiven FreeRTOS Mutexen).
        _finishMeasurement(false);
    }
}

void SensorManager::_finishMeasurement(bool graceful)
{
    _hw->allLightsOff();
    _hw->setSafelight(true);
    _hw->setExposureLock(false);
    _ctx->setMeasuringState(false);
    
    {
        MutexGuard lock(_stateMutex, pdMS_TO_TICKS(20));
        if (lock.isAcquired()) {
            _measProgress = graceful ? 100 : 0;
            _measState = graceful ? MEAS_DONE : MEAS_IDLE;
        }
    }
    
    _hw->playBeep(graceful ? BEEP_OK : BEEP_WARN);
}

float SensorManager::_calculateFilteredLux(float newSample)
{
    _luxBuffer[_filterIdx] = newSample;
    _filterIdx = (_filterIdx + 1) % FILTER_SIZE;
    if (_filterCount < FILTER_SIZE) _filterCount++;
    float sum = 0.0f;
    for (uint8_t i = 0; i < _filterCount; i++) sum += _luxBuffer[i];
    return sum / static_cast<float>(_filterCount);
}

void SensorManager::_pollTemperatures(unsigned long now)
{
    HardwareStatus status;
    if (!_ctx->getStatus(status)) return;
    float tAlu = status.tempAlu, tRoom = status.tempRoom;

    if (_dsOk) {
        float t = _dsTemp.getTempCByIndex(0);
        if (t > -50.0f && t < 150.0f) tAlu = t;
        _dsTemp.requestTemperatures();
    }

    if (_bmpOk && _hw->takeI2C()) {
        float t = _bmpEnv.readTemperature();
        if (!std::isnan(t)) tRoom = t;
        _hw->giveI2C();
    }
    _ctx->updateSensors(status.headLux, _lastValidBaseLux, tAlu, tRoom);
}

bool SensorManager::isMeasurementRunning() const 
{ 
    MutexGuard lock(_stateMutex, pdMS_TO_TICKS(10));
    return lock.isAcquired() ? (_measState != MEAS_IDLE && _measState != MEAS_DONE) : false; 
}

bool SensorManager::isMeasurementDone() const 
{ 
    MutexGuard lock(_stateMutex, pdMS_TO_TICKS(10));
    return lock.isAcquired() ? (_measState == MEAS_DONE) : false; 
}

uint8_t SensorManager::getMeasurementProgress() const 
{ 
    MutexGuard lock(_stateMutex, pdMS_TO_TICKS(10));
    return lock.isAcquired() ? _measProgress : 0; 
}

float SensorManager::getMeasurementResult() 
{ 
    MutexGuard lock(_stateMutex, pdMS_TO_TICKS(10));
    if (!lock.isAcquired()) return 0.0f;
    
    float res = _measResultLux; 
    _measState = MEAS_IDLE; 
    return res; 
}