/* =============================================================================
 * BWTimeApp.h - DUKATIMER BETA (v0.916.12)
 * * Modus: Klassische Schwarz-Weiß Belichtung (Rein Zeitgesteuert)
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class BWTimeApp : public IAppMode
{
public:
    BWTimeApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    virtual ~BWTimeApp() = default;

    const char *getName() const override { return "BW-Time"; }

    void onEnter() override;
    void onExit() override;
    void onUpdate() override;
    void handleInput(int event) override;

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    PaperManager *_pm;

    float _targetTimeS;
    float _currentGrade;
};