#include "SGDoseApp.h"
#include <cstring>
#include <cmath>

/* =============================================================================
 * SGDoseApp.cpp - DUKATIMER BETA (v0.912)
 * * Implementierung der Split-Grade Logik.
 * (+ HÄRTUNG: STATE_COMPLETE entfernt für deterministische und lückenlose Zyklen)
 * (+ HÄRTUNG: Double-Beep UX-Bug bei Phasenübergängen behoben)
 * (+ FIX: UI-Flicker Protection bei Belichtungsstart integriert)
 * ========================================================================== */

SGDoseApp::SGDoseApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _state(STATE_IDLE), _editFocus(0)
{
    // Signatur-Erfüllung: pm wird in diesem reinen Dosis-Modus noch nicht benötigt
    (void)pm;
}

SGDoseApp::~SGDoseApp() {}

void SGDoseApp::onEnter()
{
    _state = STATE_IDLE;
    _editFocus = 0;

    ExposureParams exp;
    _ctx->getExposure(exp);

    AppSharedState s{};
    s.sgDose.targetSoft = exp.targetDoseSoft;
    s.sgDose.targetHard = exp.targetDoseHard;
    s.sgDose.remainingDose = exp.targetDoseSoft;
    s.sgDose.filterState = 0;
    s.sgDose.isRunning = false;
    s.sgDose.waitingForUser = false;
    _ctx->setAppState(s);
}

void SGDoseApp::handleInput(int event)
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // --- 1. NOT-AUS / ABBRUCH (Event 0) ---
    if (event == EV_ABORT)
    {
        // Garantiert den harten Stopp der Core 1 Engine
        if (flags.isExposureRunning)
            _ctx->setExposureState(false);

        // Verhindert das Ausführen von noch nicht gestarteten Belichtungen
        if (flags.sgAutoPending)
            _ctx->setSGPending(false, 0.0f, 0.0f);

        _state = STATE_IDLE;
        _editFocus = 0;
        _hw->playBeep(BEEP_WARN);
        return;
    }

    // --- 2. START / WEITER (Event 1) ---
    if (event == EV_START)
    {
        // Event-Debouncing: Verhindert Mehrfachstarts bei Tastenprellen
        if (!flags.isExposureRunning && !flags.sgAutoPending)
        {
            ExposureParams exp;
            if (!_ctx->getExposure(exp))
                return;

            if (_state == STATE_IDLE)
            {
                // Startet Phase 1: Soft
                _ctx->setSGPending(true, exp.targetDoseSoft, 0.0f);
                _state = STATE_EXPOSING_SOFT;
                _hw->playBeep(BEEP_OK);
            }
            else if (_state == STATE_WAIT_FOR_FILTER)
            {
                // Startet Phase 2: Hard (Nach Bestätigung des Filterwechsels)
                _ctx->setSGPending(true, exp.targetDoseHard, 5.0f);
                _state = STATE_EXPOSING_HARD;
                _hw->playBeep(BEEP_OK);
            }
        }
        return;
    }

    // --- 3. ENCODER & FOKUS (Event 2, 3, 4) ---
    // UI-Sperre: Werden während aktiver/geplanter Belichtungen ignoriert
    if (!flags.isExposureRunning && !flags.sgAutoPending)
    {
        if (event == EV_GRADE_DOWN && _state == STATE_IDLE)
        {
            _editFocus = (_editFocus == 0) ? 1 : 0;
            _hw->playBeep(BEEP_TICK);
            return;
        }

        if (_state == STATE_IDLE)
        {
            int delta = (event == EV_TIME_UP) ? 1 : (event == EV_TIME_DOWN) ? -1
                                                                            : 0;
            if (delta != 0)
            {
                ExposureParams exp;
                if (_ctx->getExposure(exp))
                {
                    float nSoft = exp.targetDoseSoft;
                    float nHard = exp.targetDoseHard;

                    if (_editFocus == 0)
                        nSoft = fmaxf(0.1f, nSoft + (delta * 0.1f));
                    else
                        nHard = fmaxf(0.1f, nHard + (delta * 0.1f));

                    _ctx->setSplitDoses(nSoft, nHard);
                    _hw->playBeep(BEEP_TICK);
                }
            }
        }
    }
}

void SGDoseApp::onUpdate()
{
    ExposureParams exp;
    HardwareStatus hs;
    WorkflowFlags wf;

    if (!_ctx->getExposure(exp) || !_ctx->getStatus(hs) || !_ctx->getFlags(wf))
        return;

    bool isActivelyRunning = wf.isExposureRunning || wf.sgAutoPending;

    // --- AUTOMATISCHE PHASEN-ÜBERGÄNGE ---
    // Wir erkennen das Ende einer Belichtungsphase am Erlöschen des isRunning-Flags.
    // ACHTUNG: Hier werden BEWUSST keine Beeps abgespielt, da die ExposureEngine
    // das Ende physikalisch mit BEEP_OK quittiert!
    if (_state == STATE_EXPOSING_SOFT && !isActivelyRunning)
    {
        _state = STATE_WAIT_FOR_FILTER;
    }
    else if (_state == STATE_EXPOSING_HARD && !isActivelyRunning)
    {
        _state = STATE_IDLE; // Deterministischer Reset für den nächsten Print
    }

    // --- BERECHNUNG DER REST-DOSIS (FLICKER-SCHUTZ) ---
    // KRITISCH: Wir subtrahieren status.liveDose NUR wenn die Hardware auch real läuft.
    // Während "Pending" steht im Status oft noch der Restwert der Vorbelichtung!
    float live = wf.isExposureRunning ? hs.liveDose : 0.0f;
    float remaining = 0.0f;

    if (_state <= STATE_EXPOSING_SOFT)
    {
        remaining = fmaxf(0.0f, exp.targetDoseSoft - live);
    }
    else
    {
        remaining = fmaxf(0.0f, exp.targetDoseHard - live);
    }

    // --- UI-UNION UPDATE ---
    AppSharedState s{};
    s.activeStateMode = MODE_SG_DOSE;
    s.sgDose.targetSoft = exp.targetDoseSoft;
    s.sgDose.targetHard = exp.targetDoseHard;
    s.sgDose.remainingDose = remaining;

    // Filter-Zustand für das Display (0=Soft, 1=Hard)
    s.sgDose.filterState = (_state >= STATE_WAIT_FOR_FILTER) ? 1 : 0;

    s.sgDose.isRunning = isActivelyRunning;
    s.sgDose.waitingForUser = (_state == STATE_WAIT_FOR_FILTER);

    _ctx->setAppState(s);
}

void SGDoseApp::onExit()
{
    // Absolute Sicherheits-Routine: Alles stoppen und Warteschlange leeren
    WorkflowFlags flags;
    if (_ctx->getFlags(flags))
    {
        if (flags.sgAutoPending)
            _ctx->setSGPending(false, 0.0f, 0.0f);
        if (flags.isExposureRunning)
            _ctx->setExposureState(false);
    }

    // Union beim Verlassen nullen, um Datenkorruption in der Anzeige zu vermeiden
    AppSharedState s{};
    std::memset(&s, 0, sizeof(AppSharedState));
    _ctx->setAppState(s);
}