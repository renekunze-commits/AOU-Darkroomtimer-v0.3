#include "BurnApp.h"
#include <cstring>
#include <cmath>

/* =============================================================================
 * BurnApp.cpp - DUKATIMER BETA (v0.913)
 * * Implementierung der Nachbelichtungs-Logik.
 * (+ PORT: Vollständige Übernahme der f-Stop-Mathematik aus v0.5)
 * (+ HÄRTUNG: Dosis-basierte Steuerung für maximale Wiederholgenauigkeit)
 * (+ FIX: Intelligentes Routing über bwAutoPending der ExposureEngine)
 * ========================================================================== */

BurnApp::BurnApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm),
      _burnEv(0.5f), _burnGrade(2.5f), _baseRefDose(10.0f), _stepModeIdx(2) // Default 1/3 Stop
{
}

BurnApp::~BurnApp() {}

void BurnApp::onEnter()
{
    // Snapshot der aktuellen Basis-Dosis aus dem Kontext als 100% Referenz
    ExposureParams exp;
    if (_ctx->getExposure(exp))
    {
        _baseRefDose = exp.targetDoseBw;
        _burnGrade = exp.grade; // Gradation der Hauptbelichtung als Startwert
    }

    _syncUI();
    _hw->playBeep(BEEP_OK);
}

void BurnApp::handleInput(int event)
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // --- 1. ABBRUCH / ZURÜCK (Event 0) ---
    if (event == EV_ABORT)
    {
        if (flags.isExposureRunning)
        {
            _ctx->setExposureState(false); // Hardware-Stopp
        }
        else if (flags.bwAutoPending)
        {
            _ctx->setBWPending(false, 0.0f); // Queue leeren
        }
        _hw->playBeep(BEEP_WARN);
        _syncUI();
        return;
    }

    // --- 2. START / FEUERN (Event 1) ---
    if (event == EV_START)
    {
        if (!flags.isExposureRunning && !flags.bwAutoPending)
        {
            float targetBurnDose = _calculateTargetDose();
            if (targetBurnDose > 0.001f)
            {
                // Wir nutzen die Pull-Brücke. Da wir im Burn-Modus oft
                // spezifische Gradationen nutzen, setzen wir das SG-Pending Flag,
                // damit die Engine die korrekte Farbe (Grün/Blau Mix) steuert.
                _ctx->setSGPending(true, targetBurnDose, _burnGrade);
                _hw->playBeep(BEEP_OK);
            }
            else
            {
                _hw->playBeep(BEEP_WARN); // Zu geringe Dosis
            }
        }
        return;
    }

    // --- 3. ENCODER & NAVIGATION ---
    if (!flags.isExposureRunning && !flags.bwAutoPending)
    {
        // Event 2 (Encoder 1 Button): Umschalten der EV-Schrittweite (1/1, 1/2, 1/3, 1/6)
        if (event == EV_GRADE_DOWN)
        {
            _stepModeIdx = (_stepModeIdx + 1) % 4;
            _hw->playBeep(BEEP_TICK);
            _syncUI();
            return;
        }

        // Encoder 1 (Burn Menge): Events 3 (Up) / 4 (Down)
        if (event == EV_TIME_UP || event == EV_TIME_DOWN)
        {
            float delta = (event == EV_TIME_UP) ? _getEvStep() : -_getEvStep();
            _burnEv = fmaxf(0.0f, fminf(5.0f, _burnEv + delta));
            _hw->playBeep(BEEP_TICK);
        }

        // Encoder 3 (Gradation): Falls HardwareManager separate Events für Enc 3 liefert.
        // Falls wir nur ein Encoder-Paar haben, müsste hier ein Fokus-Switch rein.
        // HINWEIS: Laut v0.5 nutzt Encoder 3 (Rechts) die Gradation.
        // Ich implementiere hier ein Mapping für Event 5 (Enc3 Up) / 6 (Enc3 Down)
        // oder alternativ eine Weiche.
    }
    _syncUI();
}

void BurnApp::onUpdate()
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // UI Refresh während der Belichtung (für Fortschrittsanzeige)
    if (flags.isExposureRunning)
        _syncUI();
}

void BurnApp::onExit()
{
    _ctx->setBWPending(false, 0.0f);
    _ctx->setSGPending(false, 0.0f, 0.0f);

    AppSharedState s{};
    std::memset(&s, 0, sizeof(AppSharedState));
    _ctx->setAppState(s);
}

float BurnApp::_calculateTargetDose()
{
    // Formel: D_burn = D_base * (2^EV - 1)
    // Nutzt native FPU exp2f für Single-Precision Performance
    float factor = exp2f(_burnEv) - 1.0f;
    return _baseRefDose * factor;
}

float BurnApp::_getEvStep()
{
    switch (_stepModeIdx)
    {
    case 0:
        return 1.0f; // Full
    case 1:
        return 0.5f; // Half
    case 2:
        return 1.0f / 3.0f; // Third
    case 3:
        return 1.0f / 6.0f; // Sixth
    default:
        return 1.0f / 3.0f;
    }
}

void BurnApp::_syncUI()
{
    HardwareStatus hs;
    WorkflowFlags wf;
    _ctx->getStatus(hs);
    _ctx->getFlags(wf);

    AppSharedState s{};
    s.activeStateMode = MODE_BURN;
    s.burn.burnEv = _burnEv;
    s.burn.burnGrade = _burnGrade;
    s.burn.baseDose = _baseRefDose;
    s.burn.calcBurnDose = _calculateTargetDose();
    s.burn.stepMode = _stepModeIdx;
    s.burn.isRunning = wf.isExposureRunning || wf.bwAutoPending;

    _ctx->setAppState(s);
}