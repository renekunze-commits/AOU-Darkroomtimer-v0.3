/* =============================================================================
 * LiveViewApp.h - DUKATIMER BETA (v0.916.3)
 *
 * Live-Übersicht der Sensordaten. Liest periodisch die aktuellen Messwerte aus
 * `SystemContext::getStatus()` und gibt diese über `Serial.printf()` aus.
 *
 * Ressourcen: SystemContext (Lesezugriff), HardwareManager (nur auf Anfrage).
 * Keine direkten I2C-Aufrufe, keine blockierenden Aufrufe in onUpdate().
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class LiveViewApp : public IAppMode
{
public:
    LiveViewApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~LiveViewApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    const char *getName() const override { return "LiveView"; }

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    PaperManager *_pm;

    // Intervallsteuerung (nicht-blockierend)
    unsigned long _lastMillis;
    static constexpr unsigned long INTERVAL_MS = 500UL;
};