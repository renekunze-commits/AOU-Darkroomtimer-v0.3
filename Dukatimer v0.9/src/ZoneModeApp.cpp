#include "ZoneModeApp.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"
#include <cmath>

/* =============================================================================
 * ZoneModeApp.cpp - DUKATIMER BETA (v0.916.6)
 * * (+ FIX: Fehlende Includes und Variablen-Deklarationen korrigiert)
 * * (+ FIX: Echte Hardware-Ansteuerung über Pull-Brücke implementiert)
 * ========================================================================== */

ZoneModeApp::ZoneModeApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm), _selectedZone(5), _baseDose(1.0f), _running(false)
{
}

ZoneModeApp::~ZoneModeApp() {}

void ZoneModeApp::onEnter()
{
    PaperProfile p;
    if (_pm && _pm->getActiveProfileCopy(p))
    {
        _baseDose = p.Kbw > 0.0f ? p.Kbw : 1.0f;
    }
    _selectedZone = 5;
    _running = false;

    onUpdate(); // Erster UI-Push
}

void ZoneModeApp::handleInput(int event)
{
    if (!_ctx || !_hw)
        return;

    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // Event mapping: 1=inc zone, 2=dec zone, 3=start/stop
    if (event == EV_GRADE_UP && _selectedZone < 10 && !flags.isExposureRunning)
        _selectedZone++;
    else if (event == EV_GRADE_DOWN && _selectedZone > 0 && !flags.isExposureRunning)
        _selectedZone--;
    else if (event == EV_START)
    {
        if (flags.isExposureRunning)
        {
            _ctx->setExposureState(false);
        }
        else if (flags.bwAutoPending)
        {
            _ctx->setBWPending(false, 0.0f);
        }
        else
        {
            uint8_t anchor = 2;
            float calc = _baseDose * exp2f(static_cast<float>(static_cast<int>(_selectedZone) - static_cast<int>(anchor)));
            _ctx->setBWPending(true, calc);
            _hw->playBeep(BEEP_OK);
        }
    }
    onUpdate();
}

void ZoneModeApp::onUpdate()
{
    if (!_ctx)
        return;

    ExposureParams e;
    if (_ctx->getExposure(e))
    {
        _baseDose = e.targetDoseBw > 0.0f ? e.targetDoseBw : _baseDose;
    }

    WorkflowFlags flags;
    if (_ctx->getFlags(flags))
    {
        _running = flags.isExposureRunning || flags.bwAutoPending;
    }

    uint8_t anchor = 2;
    float calc = _baseDose * exp2f(static_cast<float>(static_cast<int>(_selectedZone) - static_cast<int>(anchor)));

    AppSharedState s{};
    s.activeStateMode = MODE_ZONE;
    s.zoneMode.selectedZone = _selectedZone;
    s.zoneMode.baseDose = _baseDose;
    s.zoneMode.calculatedDose = calc;
    s.zoneMode.isRunning = _running;

    _ctx->setAppState(s);
}

void ZoneModeApp::onExit()
{
    if (_ctx)
        _ctx->setBWPending(false, 0.0f);
}