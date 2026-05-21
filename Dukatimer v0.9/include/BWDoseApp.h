/* =============================================================================
 * BWDoseApp.h - DUKATIMER BETA (v0.912)
 * * Monograde-Belichtung basierend auf Echtzeit-Dosis.
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Kapselung: Die App steuert KEINE Hardware direkt.
 * 2. Asynchrone Kommunikation: Start-Trigger über 'bwAutoPending'.
 * 3. Non-Blocking: Synchronisiert UI-States ohne FreeRTOS-Delays.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class BWDoseApp : public IAppMode
{
public:
    BWDoseApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~BWDoseApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;

    const char *getName() const override { return "BWDose"; }

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    PaperManager *_pm;
};