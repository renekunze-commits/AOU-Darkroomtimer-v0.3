/* =============================================================================
 * CalibrationApp.h - DUKATIMER BETA (v0.912)
 * * Wizard für die Stouffer-Keil Kalibrierung.
 * * ARCHITEKTUR: Nutzt SensorManager für asynchrone Messungen auf Core 0.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class SensorManager;

class CalibrationApp : public IAppMode
{
public:
    CalibrationApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm);
    ~CalibrationApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    
    const char *getName() const override { return "CALIB"; }

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;
    PaperManager    *_pm;
    SensorManager   *_sm;

    enum State : uint8_t
    {
        STATE_IDLE = 0,
        STATE_WAIT_G0,
        STATE_MEAS_G0,
        STATE_WAIT_G5,
        STATE_MEAS_G5,
        STATE_COMPLETE
    } _state;

    float _luxG0;
    float _luxG5;

    void enterState(State s);
};