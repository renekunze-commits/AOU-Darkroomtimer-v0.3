#include "BWDoseApp.h"
#include <cstring>

/* =============================================================================
 * BWDoseApp.cpp - DUKATIMER BETA (v0.912)
 * * Implementierung der Monograde-Dosis-Logik.
 * (+ ARCHITEKTUR-FIX: Strikte Nutzung von setBWPending als Start-Trigger)
 * (+ ARCHITEKTUR-FIX: Encoder-Logik über setBWDose)
 * (+ HÄRTUNG: 1-Tick UI-Flicker durch strikte isExposureRunning Trennung behoben)
 * (+ CLEANUP: Ungenutzte Includes und ungenutzten PaperManager-Member entfernt)
 * ========================================================================== */

BWDoseApp::BWDoseApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm)
{
}

BWDoseApp::~BWDoseApp() {}

void BWDoseApp::onEnter()
{
    ExposureParams exp;
    _ctx->getExposure(exp);

    AppSharedState s{};
    s.bwDose.targetDose = exp.targetDoseBw;
    s.bwDose.remainingDose = exp.targetDoseBw;
    s.bwDose.isRunning = false;
    _ctx->setAppState(s);
}

void BWDoseApp::handleInput(int event)
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // --- 1. ABBRUCH-LOGIK (Event 0) ---
    if (event == EV_ABORT)
    {
        if (flags.isExposureRunning)
        {
            _ctx->setExposureState(false); // Signal an Core 1: Abort
            _hw->playBeep(BEEP_WARN);
        }
        else if (flags.bwAutoPending)
        {
            _ctx->setBWPending(false, 0.0f); // Pending-Flag löschen
            _hw->playBeep(BEEP_WARN);
        }
        return;
    }

    // --- 2. START-LOGIK (Event 1) ---
    if (event == EV_START)
    {
        if (!flags.isExposureRunning && !flags.bwAutoPending)
        {
            ExposureParams exp;
            if (_ctx->getExposure(exp))
            {
                float softDose = exp.targetDoseBw;
                float hardDose = 0.0f;

                if (_pm)
                {
                    _pm->calculateSplitGradeDose(exp.targetDoseBw, exp.grade, softDose, hardDose);
                }

                _ctx->setDoseMode(true);
                _ctx->setSplitDoses(softDose, hardDose);
                // Signalisiert dem Backend den Belichtungswunsch
                _ctx->setBWPending(true, exp.targetDoseBw);
                _hw->playBeep(BEEP_OK);
            }
        }
        return;
    }

    // --- 3. ENCODER-LOGIK (Events 3 & 4) ---
    // UI-Sperre: Nur wenn keine Belichtung aktiv oder geplant ist
    if (!flags.isExposureRunning && !flags.bwAutoPending)
    {
        int delta = (event == EV_TIME_UP) ? 1 : (event == EV_TIME_DOWN) ? -1
                                                                        : 0;
        if (delta != 0)
        {
            ExposureParams exp;
            if (_ctx->getExposure(exp))
            {
                // Inkrement in 0.1er Schritten
                float newDose = exp.targetDoseBw + (delta * 0.1f);
                if (newDose < 0.1f)
                    newDose = 0.1f;
                if (newDose > 999.0f)
                    newDose = 999.0f;

                _ctx->setBWDose(newDose);
                _hw->playBeep(BEEP_TICK);
            }
        }
    }
}

void BWDoseApp::onUpdate()
{
    ExposureParams exp;
    HardwareStatus hs;
    WorkflowFlags wf;

    // Atomare Snapshots für das UI-Rendering
    if (!_ctx->getExposure(exp) || !_ctx->getStatus(hs) || !_ctx->getFlags(wf))
        return;

    float target = exp.targetDoseBw;

    // FIX: liveDose wird NUR subtrahiert, wenn die Hardware wirklich belichtet.
    // Während "Pending" wartet die App noch, die alte liveDose der letzten
    // Belichtung darf das Display nicht korrumpieren!
    float remaining = wf.isExposureRunning ? (target - hs.liveDose) : target;

    if (remaining < 0.0f)
        remaining = 0.0f;

    AppSharedState s{};
    s.activeStateMode = MODE_BW_DOSE;
    s.bwDose.targetDose = target;
    s.bwDose.remainingDose = remaining;

    // UI signalisiert "RUNNING", sobald Pending oder aktiv belichtet wird
    s.bwDose.isRunning = wf.isExposureRunning || wf.bwAutoPending;

    _ctx->setAppState(s);
}

void BWDoseApp::onExit()
{
    // Sicherstellen, dass keine "Geister"-Belichtungen in der Warteschlange hängen bleiben,
    // falls der User den Modus im allerletzten Moment vor dem Engine-Pickup wechselt.
    _ctx->setBWPending(false, 0.0f);

    AppSharedState s{};
    std::memset(&s, 0, sizeof(AppSharedState));
    _ctx->setAppState(s);
}