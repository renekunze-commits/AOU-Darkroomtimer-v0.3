/* =============================================================================
 * SGTimeApp.h - DUKATIMER BETA (v0.916.12)
 * * Modus: Split-Grade Belichtung (Rein Zeitgesteuert)
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class SGTimeApp : public IAppMode
{
public:
    SGTimeApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    virtual ~SGTimeApp() = default;

    const char* getName() const override { return "SG-Time"; }

    void onEnter() override;
    void onExit() override;
    void onUpdate() override;
    void handleInput(int event) override;

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    PaperManager *_pm;

    float _timeSoft;
    float _timeHard;
    bool _isEditingHard; // Toggle zwischen Soft/Hard Zeiteinstellung
};