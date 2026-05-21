#include "DensApp.h"
#include "SensorManager.h"
#include <cmath>
#include <cstdio>
#include <cstring>

/* =============================================================================
 * DensApp.cpp - DUKATIMER BETA (v0.916.3)
 * * Implementierung des Densitometers.
 * (+ FIX: Variable st für das AppSharedState Tagging korrigiert)
 * ========================================================================== */

DensApp::DensApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm)
    : _ctx(ctx), _hw(hw), _pm(pm), _sm(sm),
      _state(STATE_IDLE), _zeroRef(0.0f), _hasZeroRef(false), _lastDensity(0.0f)
{
}

DensApp::~DensApp() {}

void DensApp::onEnter()
{
    Serial.println("[DENS] onEnter() - Densitometer Ready");
    enterState(STATE_IDLE);

    AppSharedState st{};
    st.activeStateMode = MODE_DENSITOMETER;
    st.densitometer.refLux = _zeroRef;
    st.densitometer.currentDensity = _lastDensity;
    st.densitometer.isZeroed = _hasZeroRef;
    _ctx->setAppState(st);
}

void DensApp::handleInput(int event)
{
    if (event == EV_ABORT)
    {
        if (_sm->isMeasurementRunning())
        {
            _sm->abortMeasurement();
        }

        _hasZeroRef = false;
        _zeroRef = 0.0f;
        _lastDensity = 0.0f;

        AppSharedState st{};
        st.activeStateMode = MODE_DENSITOMETER;
        st.densitometer.refLux = _zeroRef;
        st.densitometer.currentDensity = _lastDensity;
        st.densitometer.isZeroed = false;
        _ctx->setAppState(st);

        enterState(STATE_IDLE);
        Serial.println("[DENS] Zero-Reference gelöscht.");
        _hw->playBeep(BEEP_WARN);
    }
    else if (event == EV_START)
    {
        if (_state == STATE_IDLE)
        {
            if (_sm->startMeasurement(true))
            {
                enterState(STATE_MEASURING);
            }
            else
            {
                Serial.println("[DENS] Fehler: Sensor blockiert!");
                _hw->playBeep(BEEP_WARN);
            }
        }
    }
}

void DensApp::onUpdate()
{
    if (_state == STATE_MEASURING && _sm->isMeasurementDone())
    {
        float lux = _sm->getMeasurementResult();

        if (!_hasZeroRef)
        {
            _zeroRef = lux;
            _hasZeroRef = true;
            _lastDensity = 0.0f;
            Serial.printf("[DENS] ZERO REF SET: %.3f Lux\n", _zeroRef);
            _hw->playBeep(BEEP_OK);
        }
        else
        {
            if (lux > LUX_MIN_THRESHOLD && _zeroRef > LUX_MIN_THRESHOLD)
            {
                _lastDensity = std::log10(_zeroRef / lux);
                if (_lastDensity < 0.0f)
                    _lastDensity = 0.0f;

                Serial.printf("[DENS] RESULT | Meas: %.3f Lux | Density: %.2f D\n", lux, _lastDensity);
                _hw->playBeep(BEEP_OK);
            }
            else
            {
                Serial.println("[DENS] MATH ERROR: Lux zu niedrig (Division by Zero oder Log-Fehler)");
                _ctx->setLastError(ERR_MATH_INVALID);
                _hw->playBeep(BEEP_ALARM);
            }
        }

        AppSharedState st{};
        st.activeStateMode = MODE_DENSITOMETER; // FIX: Richtige Variable st verwendet
        st.densitometer.refLux = _zeroRef;
        st.densitometer.currentDensity = _lastDensity;
        st.densitometer.isZeroed = _hasZeroRef;
        _ctx->setAppState(st);

        enterState(STATE_IDLE);
    }
}

void DensApp::enterState(State s)
{
    _state = s;
}

void DensApp::onExit()
{
    if (_sm->isMeasurementRunning())
    {
        _sm->abortMeasurement();
    }
    Serial.println("[DENS] onExit()");
}