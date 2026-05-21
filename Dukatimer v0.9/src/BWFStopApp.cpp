#include "BWFStopApp.h"
#include <cstring>
#include <cmath>

/* =============================================================================
 * BWFStopApp.cpp - DUKATIMER BETA (v0.912)
 * * Implementierung der F-Stop Logik (Zeit-Modus).
 * (+ HÄRTUNG: Vollständige Entkopplung von der Hardware via Pending-Flags)
 * (+ FIX: Amnesie-Landmine behoben -> Basiszeit wird persistent in Prefs gesichert)
 * (+ FIX: Unendliche Zeiten verhindert -> fStopTicks auf ±5 Blenden limitiert)
 * (+ UX: Abort-Button (0) fungiert im IDLE-Status als Quick-Reset für Ticks)
 * ========================================================================== */

BWFStopApp::BWFStopApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _baseTime(10.0f), _fStopTicks(0), _editFocus(0)
{
    // Signatur-Erfüllung: pm wird im reinen Timer-Modus nicht benötigt.
    (void)pm;
}

BWFStopApp::~BWFStopApp() {}

void BWFStopApp::onEnter()
{
    _fStopTicks = 0;
    _editFocus = 0; // Standard: Ticks verstellen

    // Initial-Basiszeit aus den globalen Einstellungen laden
    SystemPreferences prefs;
    if (_ctx->getPreferences(prefs))
    {
        _baseTime = prefs.stdTime;
    }

    // Initiales UI-Update vorbereiten
    const float stopResolution = getStopResolution();
    float initialCalc = _baseTime * exp2f(static_cast<float>(_fStopTicks) / stopResolution);

    AppSharedState s{};
    s.bwFStop.baseTime = _baseTime;
    s.bwFStop.fStopTicks = _fStopTicks;
    s.bwFStop.calculatedTime = initialCalc;
    s.bwFStop.remainingTime = initialCalc;
    s.bwFStop.isRunning = false;
    _ctx->setAppState(s);
}

void BWFStopApp::handleInput(int event)
{
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return;

    // --- 1. ABBRUCH- & RESET-PFAD (Event 0) ---
    if (event == EV_ABORT)
    {
        if (flags.isExposureRunning)
        {
            _ctx->setExposureState(false); // Stop Hardware
        }
        else if (flags.bwAutoPending)
        {
            _ctx->setBWPending(false, 0.0f); // Clear Queue
        }
        else
        {
            // FIX (UX): Wenn das System im Leerlauf ist, dient der Abort-Button
            // als Quick-Reset, um sofort zur Basiszeit (0 Ticks) zurückzukehren.
            _fStopTicks = 0;
        }

        _hw->playBeep(BEEP_WARN);
        return;
    }

    // --- 2. START-PFAD (Event 1) ---
    if (event == EV_START)
    {
        if (!flags.isExposureRunning && !flags.bwAutoPending)
        {
            const float stopResolution = getStopResolution();
            float calcTime = _baseTime * exp2f(static_cast<float>(_fStopTicks) / stopResolution);

            // ARCHITEKTUR-HINWEIS: Wir nutzen bwAutoPending. Die Engine muss
            // im Modus MODE_BW_FSTOP zwingend engine.startTime() triggern!
            _ctx->setBWPending(true, calcTime);
            _hw->playBeep(BEEP_OK);
        }
        return;
    }

    // --- 3. ENCODER-PFAD (Event 2, 3, 4) ---
    if (!flags.isExposureRunning && !flags.bwAutoPending)
    {
        // Fokus wechseln (Ticks <-> Basiszeit)
        if (event == EV_GRADE_DOWN)
        {
            _editFocus = (_editFocus == 0) ? 1 : 0;
            _hw->playBeep(BEEP_TICK);
            return;
        }

        // Werte ändern (Rechts=3, Links=4)
        int delta = (event == EV_TIME_UP) ? 1 : (event == EV_TIME_DOWN) ? -1
                                                                        : 0;
        if (delta != 0)
        {
            if (_editFocus == 0)
            {
                const int maxTicks = static_cast<int>(getStopResolution() * MAX_STOP_RANGE);
                _fStopTicks += delta;
                if (_fStopTicks > maxTicks)
                    _fStopTicks = maxTicks;
                if (_fStopTicks < -maxTicks)
                    _fStopTicks = -maxTicks;
            }
            else
            {
                // FIX: Amnesie-Landmine behoben! Basiszeit wird direkt in den Prefs
                // gespeichert, damit sie Modus-Wechsel übersteht. Der StorageManager
                // sichert dies nach 2.5s Idle-Zeit asynchron ins LittleFS.
                SystemPreferences prefs;
                if (_ctx->getPreferences(prefs))
                {
                    prefs.stdTime += (static_cast<float>(delta) * 0.5f);
                    if (prefs.stdTime < 0.5f)
                        prefs.stdTime = 0.5f;
                    if (prefs.stdTime > 999.0f)
                        prefs.stdTime = 999.0f;

                    _ctx->setPreferences(prefs);
                    _baseTime = prefs.stdTime; // Lokale Kopie synchron halten
                }
            }
            _hw->playBeep(BEEP_TICK);
        }
    }
}

void BWFStopApp::onUpdate()
{
    HardwareStatus hs;
    WorkflowFlags wf;

    if (!_ctx->getStatus(hs) || !_ctx->getFlags(wf))
        return;

    // FPU-optimierte Berechnung mit Single-Precision
    const float stopResolution = getStopResolution();
    float calcTime = _baseTime * exp2f(static_cast<float>(_fStopTicks) / stopResolution);

    // FLICKER-SCHUTZ: liveTime (Telemetrie von Core 1) darf NUR subtrahiert werden,
    // wenn die Hardware physisch belichtet. Während "Pending" zeigt das UI
    // die volle Zielzeit an.
    float live = wf.isExposureRunning ? hs.liveTime : 0.0f;
    float remaining = calcTime - live;

    if (remaining < 0.0f)
        remaining = 0.0f;

    // Atomares Update der UI-Union
    AppSharedState s{};
    s.activeStateMode = MODE_BW_FSTOP;
    s.bwFStop.baseTime = _baseTime;
    s.bwFStop.fStopTicks = _fStopTicks;
    s.bwFStop.calculatedTime = calcTime;
    s.bwFStop.remainingTime = remaining;

    s.bwFStop.isRunning = wf.isExposureRunning || wf.bwAutoPending;

    _ctx->setAppState(s);
}

void BWFStopApp::onExit()
{
    // Sicherheits-Netz: Keine hängenden Belichtungswünsche beim Modus-Wechsel
    _ctx->setBWPending(false, 0.0f);

    // Union löschen
    AppSharedState s{};
    std::memset(&s, 0, sizeof(AppSharedState));
    _ctx->setAppState(s);
}

float BWFStopApp::getStopResolution() const
{
    SystemPreferences prefs;
    if (_ctx && _ctx->getPreferences(prefs))
    {
        switch (prefs.stepMode)
        {
        case 1:
            return 1.0f;
        case 2:
            return 2.0f;
        case 3:
            return 3.0f;
        case 4:
            return 6.0f;
        default:
            break;
        }
    }

    return 3.0f;
}