#include "CalibrationApp.h"
#include "SensorManager.h"
#include <cstdio>
#include <cstring>

/* =============================================================================
 * CalibrationApp.cpp - DUKATIMER BETA (v0.916.3)
 * * Implementierung des Stouffer-Messwizards.
 * (+ FIX: Enum-Mismatch und fehlende Member-Variablen behoben)
 * (+ HÄRTUNG: Vollständige Integration des asynchronen SensorManagers)
 * (+ ANPASSUNG: Platzhalter-Werte für 21-Stufen Stouffer-Keil wiederhergestellt)
 * (+ FIX T4: Explizites Tagging des AppSharedState mit MODE_CALIBRATION)
 * ========================================================================== */

CalibrationApp::CalibrationApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm)
    : _ctx(ctx), _hw(hw), _pm(pm), _sm(sm),
      _state(STATE_IDLE),
      _luxG0(0.0f), _luxG5(0.0f)
{
}

CalibrationApp::~CalibrationApp() {}

void CalibrationApp::onEnter()
{
    _luxG0 = 0.0f;
    _luxG5 = 0.0f;
    enterState(STATE_WAIT_G0);
}

void CalibrationApp::handleInput(int event)
{
    if (event == EV_ABORT) // EV_ABORT
    {
        if (_sm->isMeasurementRunning())
        {
            _sm->abortMeasurement();
        }
        enterState(STATE_IDLE);
        _hw->playBeep(BEEP_WARN);
    }
    else if (event == EV_START) // EV_START
    {
        if (_state == STATE_WAIT_G0)
        {
            if (_sm->startMeasurement(true))
            {
                enterState(STATE_MEAS_G0);
            }
            else
            {
                _hw->playBeep(BEEP_WARN);
            }
        }
        else if (_state == STATE_WAIT_G5)
        {
            if (_sm->startMeasurement(true))
            {
                enterState(STATE_MEAS_G5);
            }
            else
            {
                _hw->playBeep(BEEP_WARN);
            }
        }
        else if (_state == STATE_COMPLETE || _state == STATE_IDLE)
        {
            onEnter();
        }
    }
}

void CalibrationApp::onUpdate()
{
    switch (_state)
    {
    case STATE_MEAS_G0:
        if (_sm->isMeasurementDone())
        {
            _luxG0 = _sm->getMeasurementResult();
            Serial.printf("[CALIB] Grade 0 Measured: %.3f Lux\n", _luxG0);

            enterState(STATE_WAIT_G5);
            _hw->playBeep(BEEP_OK);
        }
        break;

    case STATE_MEAS_G5:
        if (_sm->isMeasurementDone())
        {
            _luxG5 = _sm->getMeasurementResult();
            Serial.printf("[CALIB] Grade 5 Measured: %.3f Lux\n", _luxG5);

            enterState(STATE_COMPLETE);
            _hw->playBeep(BEEP_SAVE);
        }
        break;

    default:
        break;
    }
}

void CalibrationApp::enterState(State s)
{
    _state = s;
    Serial.printf("[CALIB] enterState %d\n", (int)s);

    WorkflowFlags flags;
    if (_ctx->getFlags(flags))
    {
        flags.appState.activeStateMode = MODE_CALIBRATION;
        flags.appState.calibration.wizardState = static_cast<uint8_t>(s);

        _ctx->setAppState(flags.appState);
    }

    if (s == STATE_COMPLETE)
    {
        const int stepW0 = 1;
        const int stepB0 = 21;
        const int stepW5 = 1;
        const int stepB5 = 10;
        const float calibTime = 10.0f;

        bool success = _pm->calculateAndSaveCalibration(stepW0, stepB0, stepW5, stepB5, calibTime, _luxG0, _luxG5);

        if (success)
        {
            Serial.println("[CALIB] Profile D-logH Matrix generated & saved.");
        }
        else
        {
            Serial.println("[CALIB] FAILED to save profile. Bad math or I/O.");
            _ctx->setLastError(ERR_MATH_INVALID);
            _hw->playBeep(BEEP_ALARM);
        }
    }
}

void CalibrationApp::onExit()
{
    if (_sm->isMeasurementRunning())
    {
        _sm->abortMeasurement();
    }
    Serial.println("[CALIB] Exited.");
}