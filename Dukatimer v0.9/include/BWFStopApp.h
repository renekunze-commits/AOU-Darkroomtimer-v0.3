/* =============================================================================
 * BWFStopApp.h - DUKATIMER BETA (v0.912)
 * * F-Stop basierter Timer-Modus (fraktionale Blendenstufen).
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Single-Precision FPU: Nutzt exp2f() für hochpräzise Berechnungen.
 * 2. Kapselung: Hardware-Trigger rein über SystemContext Pending-Flags.
 * 3. Zeit-Delegation: Das UI zählt keine Zeiten selbst, sondern liest liveTime.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class BWFStopApp : public IAppMode
{
public:
    BWFStopApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~BWFStopApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;

    const char *getName() const override { return "BWFStop"; }

private:
    SystemContext *_ctx;
    HardwareManager *_hw;

    float _baseTime;    // Basiszeit in Sekunden
    int _fStopTicks;    // Anzahl der fraktionalen Blendenstufen
    uint8_t _editFocus; // 0 = fStopTicks ändern, 1 = Basiszeit ändern

    static constexpr float MAX_STOP_RANGE = 5.0f;

    float getStopResolution() const;
};