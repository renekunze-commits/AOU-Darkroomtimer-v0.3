#include "SGTimeApp.h"
#include "types.h"

/* =============================================================================
 * SGTimeApp.cpp - DUKATIMER BETA (v0.916.12)
 * * Logik für Split-Grade Zeit-Belichtung.
 * ========================================================================== */

SGTimeApp::SGTimeApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm),
      _timeSoft(10.0f), _timeHard(10.0f), _isEditingHard(false)
{
}

void SGTimeApp::onEnter()
{
    ExposureParams exp;
    if (_ctx->getExposure(exp))
    {
        _timeSoft = exp.targetDoseSoft;
        _timeHard = exp.targetDoseHard;
    }

    _isEditingHard = false;
    _hw->allLightsOff();
}

void SGTimeApp::onExit()
{
}

void SGTimeApp::onUpdate()
{
    WorkflowFlags wf;
    if (!_ctx->getFlags(wf))
    {
        return;
    }

    AppSharedState s{};
    s.activeStateMode = MODE_SG;
    s.sgDose.targetSoft = _timeSoft;
    s.sgDose.targetHard = _timeHard;
    s.sgDose.filterState = _isEditingHard ? 1 : 0;
    s.sgDose.isRunning = wf.isExposureRunning || wf.sgAutoPending;

    _ctx->setAppState(s);
}

void SGTimeApp::handleInput(int event)
{
    if (event == EV_ABORT)
    {
        _ctx->setExposureState(false);
        _ctx->setSGPending(false, 0.0f, 0.0f);
        _hw->playBeep(BEEP_WARN);
        return;
    }

    switch (event)
    {
    case EV_TIME_UP:
        if (_isEditingHard)
            _timeHard += 0.5f;
        else
            _timeSoft += 0.5f;
        break;

    case EV_TIME_DOWN:
        if (_isEditingHard)
        {
            _timeHard -= 0.5f;
            if (_timeHard < 0.1f)
                _timeHard = 0.1f;
        }
        else
        {
            _timeSoft -= 0.5f;
            if (_timeSoft < 0.1f)
                _timeSoft = 0.1f;
        }
        break;

    case EV_TIME_CLICK:
        // ENTER wechselt gezielt den Edit-Fokus zwischen Soft- und Hard-Zeit.
        _isEditingHard = !_isEditingHard;
        _hw->playBeep(BEEP_TICK);
        break;

    case EV_START:
        if (_timeSoft > 0.0f || _timeHard > 0.0f)
        {
            _ctx->setDoseMode(false);
            _ctx->setSplitDoses(_timeSoft, _timeHard);
            _ctx->setSGTimePending(_timeSoft, _timeHard);
            _hw->playBeep(BEEP_OK);
        }
        break;

    default:
        break;
    }
}