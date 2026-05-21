#include "BWTimeApp.h"
#include "types.h"

/* =============================================================================
 * BWTimeApp.cpp - DUKATIMER BETA (v0.916.12)
 * * Logik für reine Zeit-Belichtung (ohne Sensormessung).
 * * Verarbeitet korrekterweise die Events EV_TIME_UP/DOWN und EV_START.
 * ========================================================================== */

BWTimeApp::BWTimeApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm),
      _targetTimeS(10.0f), _currentGrade(2.5f)
{
}

void BWTimeApp::onEnter()
{
    ExposureParams exp;
    SystemPreferences prefs;

    if (_ctx->getExposure(exp))
    {
        _currentGrade = exp.grade;
    }
    if (_ctx->getPreferences(prefs))
    {
        _targetTimeS = prefs.stdTime;
    }
    if (_pm && _pm->isFixedGradeActive())
    {
        _currentGrade = _pm->getActiveFixedGradeValue();
    }

    _ctx->setGrade(_currentGrade);

    _hw->allLightsOff();
}

void BWTimeApp::onExit()
{
    // Cleanup falls nötig
}

void BWTimeApp::onUpdate()
{
    HardwareStatus hs;
    WorkflowFlags wf;
    if (!_ctx->getStatus(hs) || !_ctx->getFlags(wf))
    {
        return;
    }

    AppSharedState s{};
    s.activeStateMode = MODE_BW;
    s.exposure.remainingTime = wf.isExposureRunning ? fmaxf(0.0f, _targetTimeS - hs.liveTime) : _targetTimeS;
    s.exposure.isRunning = wf.isExposureRunning || wf.bwAutoPending;

    _ctx->setAppState(s);
}

void BWTimeApp::handleInput(int event)
{
    if (event == EV_ABORT)
    {
        _ctx->setExposureState(false);
        _ctx->setBWPending(false, 0.0f);
        _hw->playBeep(BEEP_WARN);
        return;
    }

    switch (event)
    {
    case EV_TIME_UP:
        _targetTimeS += 0.5f; // Passe die Inkremente an deine v0.5-Logik an
        break;

    case EV_TIME_DOWN:
        _targetTimeS -= 0.5f;
        if (_targetTimeS < 0.1f)
            _targetTimeS = 0.1f;
        break;

    case EV_GRADE_UP:
        if (_pm && _pm->isFixedGradeActive())
        {
            _currentGrade = _pm->getActiveFixedGradeValue();
            _hw->playBeep(BEEP_WARN);
            break;
        }
        _currentGrade += 0.5f;
        if (_currentGrade > 5.0f)
            _currentGrade = 5.0f;
        _ctx->setGrade(_currentGrade);
        _hw->playBeep(BEEP_TICK);
        break;

    case EV_GRADE_DOWN:
        if (_pm && _pm->isFixedGradeActive())
        {
            _currentGrade = _pm->getActiveFixedGradeValue();
            _hw->playBeep(BEEP_WARN);
            break;
        }
        _currentGrade -= 0.5f;
        if (_currentGrade < 0.0f)
            _currentGrade = 0.0f;
        _ctx->setGrade(_currentGrade);
        _hw->playBeep(BEEP_TICK);
        break;

    case EV_TIME_CLICK:
        _ctx->setDoseMode(false);
        _ctx->setGrade(_currentGrade);
        _hw->playBeep(BEEP_OK);
        break;

    case EV_START:
        if (_targetTimeS > 0.0f)
        {
            float softTime = _targetTimeS;
            float hardTime = 0.0f;

            _ctx->setDoseMode(false);
            _ctx->setGrade(_currentGrade);
            if (_pm)
            {
                _pm->calculateSplitGradeTimes(_targetTimeS, _currentGrade, softTime, hardTime);
            }
            _ctx->setSplitDoses(softTime, hardTime);
            _ctx->setBWPending(true, _targetTimeS);
            _hw->playBeep(BEEP_OK);
        }
        break;

    default:
        break;
    }
}