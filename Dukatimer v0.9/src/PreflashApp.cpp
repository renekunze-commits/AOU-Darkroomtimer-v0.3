#include "PreflashApp.h"
#include <cstring>
#include <cmath>

/* =============================================================================
 * PreflashApp.cpp - DUKATIMER BETA (v0.913)
 * * Implementierung des dosisbasierten Vorbelichtungs-Wizards.
 * * (+ HÄRTUNG: Vollständige Integration in den v0.913 PaperManager Pfad)
 * * (+ HÄRTUNG: Schutz gegen Flicker und asynchrone State-Drifts)
 * ========================================================================== */

PreflashApp::PreflashApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm),
      _wizState(WIZ_OFF), _pfSteps(6), _pfDoseStep(0.1f), _currentStep(0), _editFocus(0)
{
}

PreflashApp::~PreflashApp() {}

void PreflashApp::onEnter()
{
    _wizState = WIZ_OFF;
    _editFocus = 0;
    _currentStep = 0;

    // Initial-Inkrement aus den Standard-Präferenzen ableiten (optional)
    _syncUI();
}

void PreflashApp::handleInput(int event)
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // --- 1. GLOBALER ABBRUCH (Event 0) ---
    if (event == EV_ABORT)
    {
        if (flags.isExposureRunning)
        {
            _ctx->setExposureState(false); // Hardware-Stopp anfordern
        }
        else if (flags.bwAutoPending)
        {
            _ctx->setBWPending(false, 0.0f); // Queue leeren
        }

        _wizState = WIZ_OFF;
        _hw->playBeep(BEEP_WARN);
        _syncUI();
        return;
    }

    // --- 2. WEITER / START (Event 1) ---
    if (event == EV_START)
    {
        if (_wizState == WIZ_OFF)
        {
            // Normal-Modus: Prüfe ob Profil bereits kalibriert
            PaperProfile prof;
            if (_pm->getActiveProfileCopy(prof) && prof.flashCalibrated)
            {
                if (!flags.isExposureRunning && !flags.bwAutoPending)
                {
                    _startSingleFlash(prof.flashThreshD);
                    _hw->playBeep(BEEP_OK);
                }
            }
            else
            {
                // Unkalibriert: Automatischer Start des Wizards
                _wizState = WIZ_SET_PARAMS;
                _hw->playBeep(BEEP_TICK);
            }
        }
        else if (_wizState == WIZ_SET_PARAMS)
        {
            _currentStep = 0;
            _wizState = WIZ_WAIT_START;
            _hw->playBeep(BEEP_OK);
        }
        else if (_wizState == WIZ_WAIT_START)
        {
            if (!flags.isExposureRunning && !flags.bwAutoPending)
            {
                // Dosis berechnen: (Schritt + 1) * Inkrement
                float targetDose = static_cast<float>(_currentStep + 1) * _pfDoseStep;
                _startSingleFlash(targetDose);
                _wizState = WIZ_PULSING;
                _hw->playBeep(BEEP_OK);
            }
        }
        else if (_wizState == WIZ_PICK_THRESH)
        {
            // Speichern der Kalibrierung im PaperManager
            PaperProfile prof;
            if (_pm->getActiveProfileCopy(prof))
            {
                // Schwellenwert: Stufe VOR der ersten sichtbaren Schwärzung
                float resultDose = static_cast<float>(_currentStep - 1) * _pfDoseStep;
                if (resultDose < 0.0f)
                    resultDose = 0.0f;

                prof.flashThreshD = resultDose;
                prof.flashEnable = (resultDose > 0.001f);
                prof.flashCalibrated = true;

                _pm->updateActiveProfile(prof);
                _hw->playBeep(BEEP_SAVE);
            }
            _wizState = WIZ_OFF;
        }
        _syncUI();
        return;
    }

    // --- 3. ENCODER-STEUERUNG (Event 2=Fokus, 3=Up, 4=Down) ---
    if (!flags.isExposureRunning && !flags.bwAutoPending)
    {
        // Fokuswechsel zwischen Schritten und Inkrement-Größe
        if (event == EV_GRADE_DOWN && _wizState == WIZ_SET_PARAMS)
        {
            _editFocus = (_editFocus == 0) ? 1 : 0;
            _hw->playBeep(BEEP_TICK);
        }

        int delta = (event == EV_TIME_UP) ? 1 : (event == EV_TIME_DOWN) ? -1
                                                                        : 0;
        if (delta != 0)
        {
            if (_wizState == WIZ_SET_PARAMS)
            {
                if (_editFocus == 0)
                {
                    _pfSteps = constrain(_pfSteps + delta, 3, 10);
                }
                else
                {
                    _pfDoseStep = constrain(_pfDoseStep + (static_cast<float>(delta) * 0.01f), 0.01f, 0.5f);
                }
            }
            else if (_wizState == WIZ_PICK_THRESH)
            {
                _currentStep = constrain(_currentStep + delta, 1, _pfSteps);
            }
            _hw->playBeep(BEEP_TICK);
        }
    }
    _syncUI();
}

void PreflashApp::onUpdate()
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // Automatischer Phasenübergang: Puls-Ende erkennen
    if (_wizState == WIZ_PULSING && !flags.isExposureRunning && !flags.bwAutoPending)
    {
        _currentStep++;
        if (_currentStep > _pfSteps)
        {
            _wizState = WIZ_PICK_THRESH;
        }
        else
        {
            _wizState = WIZ_WAIT_START;
        }
        _hw->playBeep(BEEP_OK);
        _syncUI();
    }

    // Kontinuierliche Telemetrie während der Belichtung (für Ladebalken/Countdown)
    if (flags.isExposureRunning)
        _syncUI();
}

void PreflashApp::onExit()
{
    // Sicherheitsnetz: Warteschlange bei Modus-Wechsel leeren
    _ctx->setBWPending(false, 0.0f);

    // AppSharedState Union für den nächsten Modus sauber hinterlassen
    AppSharedState s{};
    std::memset(&s, 0, sizeof(AppSharedState));
    _ctx->setAppState(s);
}

void PreflashApp::_startSingleFlash(float targetDose)
{
    // Nutzt den Standard-Pending-Weg. Da currentMode = MODE_PREFLASH ist,
    // triggert die ExposureEngine automatisch die Dosis-Steuerung (v0.913).
    _ctx->setBWPending(true, targetDose);
}

void PreflashApp::_syncUI()
{
    HardwareStatus hs;
    WorkflowFlags wf;
    _ctx->getStatus(hs);
    _ctx->getFlags(wf);

    AppSharedState s{};
    s.activeStateMode = MODE_PREFLASH;
    s.preflash.state = static_cast<uint8_t>(_wizState);
    s.preflash.currentStep = _currentStep;
    s.preflash.totalSteps = _pfSteps;

    // Berechnung der Dosis-Telemetrie
    float target = 0.0f;
    if (_wizState == WIZ_PULSING)
    {
        target = static_cast<float>(_currentStep + 1) * _pfDoseStep;
    }
    else if (_wizState == WIZ_OFF)
    {
        PaperProfile prof;
        if (_pm->getActiveProfileCopy(prof))
            target = prof.flashThreshD;
    }

    s.preflash.stepDose = target;
    s.preflash.remainingDose = fmaxf(0.0f, target - hs.liveDose);
    s.preflash.isRunning = wf.isExposureRunning || wf.bwAutoPending;

    _ctx->setAppState(s);
}