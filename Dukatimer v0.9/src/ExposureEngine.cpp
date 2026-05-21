#include "ExposureEngine.h"

/* =============================================================================
 * ExposureEngine.cpp - DUKATIMER BETA (v0.916.3)
 * * (+ FIX: Vollständiges Refactoring auf RAII MutexGuard)
 * * Eliminierung aller manuellen lock/unlock Paare zur Deadlock-Vermeidung.
 * ========================================================================== */

namespace
{
    class MutexGuard
    {
        SemaphoreHandle_t _mutex;
        bool _acquired;

    public:
        MutexGuard(SemaphoreHandle_t m, TickType_t t)
            : _mutex(m), _acquired(m != nullptr && xSemaphoreTake(m, t) == pdTRUE) {}
        ~MutexGuard()
        {
            if (_acquired)
                xSemaphoreGive(_mutex);
        }
        bool isAcquired() const { return _acquired; }
    };

    const uint32_t kPostWaitMs = 100;
}

ExposureEngine::ExposureEngine(SystemContext *ctx, HardwareManager *hw)
    : _ctx(ctx), _hw(hw), _tsl(static_cast<uint8_t>(ADDR_TSL2561), 12345L)
{
    _stateMutex = xSemaphoreCreateMutex();
    _doseMutex = xSemaphoreCreateMutex();
}

ExposureEngine::~ExposureEngine()
{
    if (_clTaskHandle)
    {
        _stopTask.store(true);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (_shutoffTimer)
        esp_timer_delete(_shutoffTimer);
    if (_stateMutex)
        vSemaphoreDelete(_stateMutex);
    if (_doseMutex)
        vSemaphoreDelete(_doseMutex);
}

void ExposureEngine::init()
{
    if (_initialized.load())
        return;

    if (_hw)
    {
        _sensorReady.store(_tsl.begin(_hw->getI2C1Bus()));
        if (_sensorReady.load())
        {
            _tsl.enableAutoRange(true);
            _tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
        }
    }

    esp_timer_create_args_t args = {};
    args.callback = &ExposureEngine::timerCallbackWrapper;
    args.arg = this;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "exp_shutoff";
    esp_timer_create(&args, &_shutoffTimer);

    xTaskCreatePinnedToCore(
        &ExposureEngine::clTaskWrapper,
        "ExposureCL",
        8192,
        this,
        configMAX_PRIORITIES - 2,
        &_clTaskHandle,
        1);

    _initialized.store(true);
}

void ExposureEngine::tick()
{
    if (!_initialized.load() || !_ctx || !_hw)
        return;

    WorkflowFlags flags;
    if (_ctx->getFlags(flags))
    {
        bool isPhysicallyRunning = false;
        {
            MutexGuard lock(_stateMutex, kStateLockWait);
            if (lock.isAcquired())
            {
                isPhysicallyRunning = (_state != EXP_IDLE && _state != EXP_DONE);
            }
        }

        if (isPhysicallyRunning && !flags.isExposureRunning)
        {
            abort();
            return;
        }

        if (!isPhysicallyRunning)
        {
            if (_splitTimeSequenceActive)
            {
                const float secondPhaseTime = _splitTimeSecondPhaseS;
                const uint8_t secondGreen = _splitTimeSecondGreen;
                const uint8_t secondBlue = _splitTimeSecondBlue;

                _splitTimeSequenceActive = false;
                _splitTimeSecondPhaseS = 0.0f;
                _ctx->setSGPending(false, 0.0f, 0.0f);

                if (secondPhaseTime > 0.0f)
                {
                    startTime(secondPhaseTime, secondGreen, secondBlue, false);
                    return;
                }
            }

            if (_splitDoseSequenceActive)
            {
                const float secondPhaseDose = _splitDoseSecondPhase;
                const uint8_t secondGreen = _splitDoseSecondGreen;
                const uint8_t secondBlue = _splitDoseSecondBlue;

                _splitDoseSequenceActive = false;
                _splitDoseSecondPhase = 0.0f;

                if (secondPhaseDose > 0.0f)
                {
                    startDose(secondPhaseDose, secondGreen, secondBlue, false);
                    return;
                }
            }

            if (flags.bwAutoPending)
            {
                float target = flags.pendingDose;
                ExposureParams exp;
                _ctx->setBWPending(false, 0.0f);

                if (flags.currentMode == MODE_BW && _ctx->getExposure(exp) && !exp.doseModeActive)
                {
                    const float softSeconds = fmaxf(exp.targetDoseSoft, 0.0f);
                    const float hardSeconds = fmaxf(exp.targetDoseHard, 0.0f);

                    if (hardSeconds > 0.0f)
                    {
                        _splitTimeSequenceActive = true;
                        _splitTimeSecondPhaseS = hardSeconds;
                        _splitTimeSecondGreen = 0;
                        _splitTimeSecondBlue = 255;
                    }

                    if (softSeconds > 0.0f)
                    {
                        startTime(softSeconds, 255, 0, false);
                        return;
                    }

                    if (_splitTimeSequenceActive)
                    {
                        const float secondPhaseTime = _splitTimeSecondPhaseS;
                        _splitTimeSequenceActive = false;
                        _splitTimeSecondPhaseS = 0.0f;
                        startTime(secondPhaseTime, 0, 255, false);
                        return;
                    }

                    startTime(target, 255, 255, false);
                }
                else if (flags.currentMode == MODE_BW_DOSE && _ctx->getExposure(exp) && exp.doseModeActive)
                {
                    const float softDose = fmaxf(exp.targetDoseSoft, 0.0f);
                    const float hardDose = fmaxf(exp.targetDoseHard, 0.0f);

                    if (hardDose > 0.0f)
                    {
                        _splitDoseSequenceActive = true;
                        _splitDoseSecondPhase = hardDose;
                        _splitDoseSecondGreen = 0;
                        _splitDoseSecondBlue = 255;
                    }

                    if (softDose > 0.0f)
                    {
                        startDose(softDose, 255, 0, false);
                        return;
                    }

                    if (_splitDoseSequenceActive)
                    {
                        const float secondPhaseDose = _splitDoseSecondPhase;
                        _splitDoseSequenceActive = false;
                        _splitDoseSecondPhase = 0.0f;
                        startDose(secondPhaseDose, 0, 255, false);
                        return;
                    }

                    startDose(target, 255, 255, false);
                }
                else if (flags.currentMode == MODE_BW_FSTOP)
                    startTime(target, 255, 255, false);
                else
                    startDose(target, 255, 255, false);
            }
            else if (flags.sgAutoPending)
            {
                ExposureParams exp;
                if (flags.currentMode == MODE_SG && _ctx->getExposure(exp) && !exp.doseModeActive)
                {
                    const float softSeconds = fmaxf(exp.targetDoseSoft, 0.0f);
                    const float hardSeconds = fmaxf(exp.targetDoseHard, 0.0f);

                    if (hardSeconds > 0.0f)
                    {
                        _splitTimeSequenceActive = true;
                        _splitTimeSecondPhaseS = hardSeconds;
                        _splitTimeSecondGreen = 0;
                        _splitTimeSecondBlue = 255;
                    }

                    if (softSeconds > 0.0f)
                    {
                        startTime(softSeconds, 255, 0, false);
                        return;
                    }

                    if (_splitTimeSequenceActive)
                    {
                        const float secondPhaseTime = _splitTimeSecondPhaseS;
                        _splitTimeSequenceActive = false;
                        _splitTimeSecondPhaseS = 0.0f;
                        _ctx->setSGPending(false, 0.0f, 0.0f);
                        startTime(secondPhaseTime, 0, 255, false);
                        return;
                    }

                    _ctx->setSGPending(false, 0.0f, 0.0f);
                }
                else
                {
                    float target = flags.pendingDose;
                    float grade = flags.pendingGrade;
                    _ctx->setSGPending(false, 0.0f, 0.0f);
                    uint8_t g = (grade < 1.0f) ? 255 : 0;
                    uint8_t b = (grade > 4.0f) ? 255 : 0;
                    startDose(target, g, b, false);
                }
            }
        }
    }

    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return;

    const uint32_t now = millis();

    switch (_state)
    {
    case EXP_PRE_WAIT:
        if ((now - _phaseStartedMs) >= kPreWaitMs)
            beginExposure();
        break;

    case EXP_EXPOSING:
        if ((now - _lastMetronomeMs) >= 1000U)
        {
            _lastMetronomeMs = now;
            _hw->playBeep(BEEP_TICK);
        }

        _ctx->updateLiveTime(static_cast<float>(now - _exposureStartedMs) / 1000.0f);

        if (_shutoffFired.exchange(false))
        {
            _hw->allLightsOff();
            _state = EXP_POST_WAIT;
            _phaseStartedMs = now;
            break;
        }

        if (_mode == EXPMODE_TIME)
        {
            if ((now - _exposureStartedMs) >= _timeTargetMs + 100U)
            {
                _hw->allLightsOff();
                enterPostWait(_gracefulStopRequested);
            }
        }
        break;

    case EXP_POST_WAIT:
        if ((now - _phaseStartedMs) >= kPostWaitMs)
            finishExposure(_gracefulStopRequested);
        break;

    default:
        break;
    }
}

void ExposureEngine::startTime(float seconds, uint8_t g, uint8_t b, bool skipPreWait)
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return;
    if (_state != EXP_IDLE && _state != EXP_DONE)
        return;

    _mode = EXPMODE_TIME;
    _timeTargetMs = static_cast<uint32_t>(fmaxf(seconds, 0.1f) * 1000.0f);
    _green = g;
    _blue = b;
    _gracefulStopRequested = false;
    _shutoffArmed.store(false);
    _shutoffFired.store(false);

    {
        MutexGuard dLock(_doseMutex, kDoseLockWait);
        if (dLock.isAcquired())
        {
            _currentDose = 0.0f;
            _targetDose = 0.0f;
        }
    }

    _ctx->setExposureState(true);
    _hw->setExposureLock(true);

    if (skipPreWait)
        beginExposure();
    else
    {
        _hw->allLightsOff();
        _hw->setSafelight(false);
        _phaseStartedMs = millis();
        _state = EXP_PRE_WAIT;
    }
}

void ExposureEngine::startDose(float targetDose, uint8_t g, uint8_t b, bool skipPreWait)
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return;
    if (!_sensorReady.load() || targetDose <= 0.0f)
        return;
    if (_state != EXP_IDLE && _state != EXP_DONE)
        return;

    _mode = EXPMODE_DOSE;
    _green = g;
    _blue = b;
    _gracefulStopRequested = false;
    _shutoffArmed.store(false);
    _shutoffFired.store(false);

    {
        MutexGuard dLock(_doseMutex, kDoseLockWait);
        if (dLock.isAcquired())
        {
            _currentDose = 0.0f;
            _targetDose = targetDose;
            _lastLux = 0.0f;
        }
    }
    _lastDoseSampleMs = millis();

    _ctx->setExposureState(true);
    _hw->setExposureLock(true);

    if (skipPreWait)
        beginExposure();
    else
    {
        _hw->allLightsOff();
        _hw->setSafelight(false);
        _phaseStartedMs = millis();
        _state = EXP_PRE_WAIT;
    }
}

void ExposureEngine::beginExposure()
{
    _hw->updateNeoPixels(0, _green, _blue);
    _exposureStartedMs = millis();
    _lastMetronomeMs = _exposureStartedMs;
    _shutoffFired.store(false);

    if (_mode == EXPMODE_TIME)
        armTimeModeTimer(_timeTargetMs);
    else
    {
        _shutoffArmed.store(false);
        _lastDoseSampleMs = millis();
    }
    _state = EXP_EXPOSING;
}

void ExposureEngine::abort()
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return;

    disarmPredictiveTimer();
    _splitTimeSequenceActive = false;
    _splitTimeSecondPhaseS = 0.0f;
    _splitDoseSequenceActive = false;
    _splitDoseSecondPhase = 0.0f;
    _hw->allLightsOff();
    _hw->setSafelight(true);
    _state = EXP_IDLE;
    _shutoffFired.store(false);
    _ctx->setExposureState(false);
    _hw->setExposureLock(false);
}

void ExposureEngine::stop()
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return;

    if (_state == EXP_IDLE || _state == EXP_DONE || _state == EXP_POST_WAIT)
        return;

    disarmPredictiveTimer();
    _splitTimeSequenceActive = false;
    _splitTimeSecondPhaseS = 0.0f;
    _splitDoseSequenceActive = false;
    _splitDoseSecondPhase = 0.0f;
    _hw->allLightsOff();

    if (_state == EXP_PRE_WAIT)
    {
        _hw->setSafelight(true);
        _state = EXP_IDLE;
        _ctx->setExposureState(false);
        _hw->setExposureLock(false);
        return;
    }

    enterPostWait(true);
}

bool ExposureEngine::pause()
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return false;
    if (_state != EXP_EXPOSING)
        return false;

    disarmPredictiveTimer();
    _pausedAtMs = millis();
    if (_mode == EXPMODE_TIME)
    {
        const uint32_t elapsed = _pausedAtMs - _exposureStartedMs;
        _remainingTimeMs = (elapsed >= _timeTargetMs) ? 1U : (_timeTargetMs - elapsed);
    }
    _hw->allLightsOff();
    _state = EXP_PAUSED;
    return true;
}

bool ExposureEngine::resume()
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    if (!lock.isAcquired())
        return false;
    if (_state != EXP_PAUSED)
        return false;

    _hw->updateNeoPixels(0, _green, _blue);
    uint32_t pauseDuration = millis() - _pausedAtMs;
    _exposureStartedMs += pauseDuration;
    _lastMetronomeMs += pauseDuration;
    _shutoffFired.store(false);

    if (_mode == EXPMODE_TIME)
        armTimeModeTimer((_remainingTimeMs < 1U) ? 1U : _remainingTimeMs);
    else
    {
        _shutoffArmed.store(false);
        _lastDoseSampleMs = millis();
    }

    _state = EXP_EXPOSING;
    return true;
}

void ExposureEngine::finishExposure(bool gracefulStop)
{
    disarmPredictiveTimer();
    _ctx->setExposureState(false);
    _hw->setExposureLock(false);
    _hw->setSafelight(true);
    _state = gracefulStop ? EXP_IDLE : EXP_DONE;
    _hw->playBeep(BEEP_OK);
}

void ExposureEngine::enterPostWait(bool gracefulStop)
{
    _gracefulStopRequested = gracefulStop;
    _phaseStartedMs = millis();
    _state = EXP_POST_WAIT;
}

bool ExposureEngine::isRunning() const
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    return (lock.isAcquired() && _state != EXP_IDLE && _state != EXP_DONE);
}

bool ExposureEngine::isDone() const
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    return (lock.isAcquired() && _state == EXP_DONE);
}

ExposureState ExposureEngine::getState() const
{
    MutexGuard lock(_stateMutex, kStateLockWait);
    return lock.isAcquired() ? _state : EXP_IDLE;
}

float ExposureEngine::getCurrentDose() const
{
    MutexGuard lock(_doseMutex, kDoseLockWait);
    return lock.isAcquired() ? _currentDose : 0.0f;
}

void ExposureEngine::clTaskWrapper(void *arg) { static_cast<ExposureEngine *>(arg)->clTaskLoop(); }
void ExposureEngine::timerCallbackWrapper(void *arg) { static_cast<ExposureEngine *>(arg)->onPredictiveShutoff(); }

void ExposureEngine::clTaskLoop()
{
    sensors_event_t event;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

        if (_stopTask.load())
        {
            vTaskDelete(nullptr);
            return;
        }

        if (!_initialized.load() || !_sensorReady.load())
            continue;

        bool runLoop = false;
        {
            MutexGuard lock(_stateMutex, kStateLockWait);
            runLoop = (lock.isAcquired() && _state == EXP_EXPOSING && _mode == EXPMODE_DOSE);
        }

        if (!runLoop)
        {
            _lastDoseSampleMs = millis();
            continue;
        }

        if (!_tsl.getEvent(&event))
            continue;

        const uint32_t now = millis();
        float dt = static_cast<float>(now - _lastDoseSampleMs) / 1000.0f;
        if (dt <= 0.0f)
            dt = 0.01f;
        _lastDoseSampleMs = now;

        updateDoseAndPredictiveTimer(event.light, dt);
    }
}

void ExposureEngine::onPredictiveShutoff()
{
    _shutoffArmed.store(false);
    _shutoffFired.store(true);
}

void ExposureEngine::armTimeModeTimer(uint32_t durationMs)
{
    if (!_shutoffTimer)
        return;
    uint64_t latchUs = 8000;
    uint64_t minUs = 500;
    uint64_t durUs = static_cast<uint64_t>(durationMs) * 1000ULL;
    durUs = (durUs > (latchUs + minUs)) ? (durUs - latchUs) : minUs;
    disarmPredictiveTimer();
    if (esp_timer_start_once(_shutoffTimer, durUs) == ESP_OK)
        _shutoffArmed.store(true);
}

void ExposureEngine::disarmPredictiveTimer()
{
    if (!_shutoffTimer)
        return;
    if (_shutoffArmed.load())
        esp_timer_stop(_shutoffTimer);
    _shutoffArmed.store(false);
    _shutoffFired.store(false);
}

void ExposureEngine::updateDoseAndPredictiveTimer(float lux, float dtSeconds)
{
    float lx = fmaxf(lux, 0.0f);
    float cur = 0.0f;
    float tgt = 0.0f;

    {
        MutexGuard lock(_doseMutex, kDoseLockWait);
        if (!lock.isAcquired())
            return;
        _lastLux = lx;
        _currentDose += (lx * dtSeconds);
        cur = _currentDose;
        tgt = _targetDose;
    }

    _ctx->updateLiveDose(cur);

    if (_shutoffArmed.load())
        return;

    const float rem = tgt - cur;
    if (rem <= 0.0f)
    {
        _shutoffFired.store(true);
        return;
    }

    if (cur >= (tgt * 0.95f) && lx > 0.01f && _shutoffTimer)
    {
        uint64_t latchUs = 8000;
        uint64_t minUs = 500;
        // Erhöhte Präzision durch double
        uint64_t remUs = static_cast<uint64_t>((static_cast<double>(rem) / static_cast<double>(lx)) * 1000000.0);
        remUs = (remUs > (latchUs + minUs)) ? (remUs - latchUs) : minUs;
        if (esp_timer_start_once(_shutoffTimer, remUs) == ESP_OK)
            _shutoffArmed.store(true);
    }
}