#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"
#include <cmath>

/* =============================================================================
 * ZoneModeApp.h - DUKATIMER BETA (v0.916.6)
 *
 * Zonenmodus: Basierend auf einer kalibrierten Basis-Dosis pro Zone. Jede Zone
 * verschiebt die Belichtung um 1 EV. Spiegelung in SystemContext::appState.zoneMode
 * * (+ FIX: Explizite Includes für Manager-Abhängigkeiten ergänzt)
 * ========================================================================== */

class ZoneModeApp : public IAppMode
{
public:
    ZoneModeApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~ZoneModeApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    const char *getName() const override { return "ZoneMode"; }

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    PaperManager *_pm;

    uint8_t _selectedZone;
    float _baseDose;
    bool _running;
};