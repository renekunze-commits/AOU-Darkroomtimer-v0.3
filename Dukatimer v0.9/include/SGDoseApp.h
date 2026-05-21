/* =============================================================================
 * SGDoseApp.h - DUKATIMER BETA (v0.912)
 * * Split-Grade Belichtung: Soft gefolgt von Hard.
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Phasen-Steuerung: Soft -> Pause (Filterwechsel) -> Hard -> Done.
 * 2. Kapselung: Hardware-Trigger rein über SystemContext Pending-Flags.
 * 3. UI-Synchronisation: Atomare Snapshots, kein Caching lokaler Kopien.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class SGDoseApp : public IAppMode
{
public:
    SGDoseApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~SGDoseApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    
    const char *getName() const override { return "SGDose"; }

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;

    // Interne Zustandsmaschine für den zweistufigen Ablauf.
    // HÄRTUNG: Redundanter 'STATE_COMPLETE' entfernt. Zyklen enden stets in IDLE.
    enum State : uint8_t
    {
        STATE_IDLE = 0,
        STATE_EXPOSING_SOFT,
        STATE_WAIT_FOR_FILTER, // Interaktive Pause für den Anwender (Filterwechsel)
        STATE_EXPOSING_HARD
    };

    State   _state;
    uint8_t _editFocus; // 0 = Soft-Dosis verstellen, 1 = Hard-Dosis verstellen
};