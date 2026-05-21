#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

/* =============================================================================
 * TestStripApp.h - DUKATIMER BETA (v0.916.6)
 *
 * Probestreifen-Modus: Führt eine Sequenz von Testmessungen durch und zeigt
 * Zwischenergebnisse. Keine Blocking-Operationen; Ergebnisse optional in
 * PaperManager ablegen.
 * * (+ FIX: Explizite Includes für Manager-Abhängigkeiten ergänzt, um 
 * Compiler-Fehler ('does not name a type') absolut auszuschließen)
 * ========================================================================== */

class TestStripApp : public IAppMode
{
public:
    TestStripApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~TestStripApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    const char *getName() const override { return "TestStrip"; }

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    PaperManager *_pm;

    unsigned long _lastMs;
    static constexpr unsigned long INTERVAL_MS = 300UL;
    int _step;
};