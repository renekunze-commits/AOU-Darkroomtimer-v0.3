/* =============================================================================
 * DensApp.h - DUKATIMER BETA (v0.912)
 * * Digitales Densitometer (Transmissions-Modus).
 * * ARCHITEKTUR: Nutzt 600ms High-Precision Messungen des SensorManagers.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class SensorManager;

class DensApp : public IAppMode
{
public:
    DensApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm);
    ~DensApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;

    const char *getName() const override { return "Densitometer"; }

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;
    PaperManager    *_pm;
    SensorManager   *_sm;

    // Interne Zustandsmaschine für den Messablauf
    enum State : uint8_t
    {
        STATE_IDLE = 0,
        STATE_MEASURING // SensorManager führt 600ms Integration aus
    };

    State _state;

    // --- Physikalische Messdaten ---
    float _zeroRef;      // L0: Referenzlichtstärke in Lux (Null-Abgleich)
    bool  _hasZeroRef;   // Flag: Referenzwert vorhanden?
    float _lastDensity;  // Berechnetes Resultat in D (log10)

    // Mathematische Sicherheitskonstante: Unterer Grenzwert für Lux, 
    // um Division durch Null oder Logarithmus-Fehler zu vermeiden.
    static constexpr float LUX_MIN_THRESHOLD = 0.0001f;

    /**
     * Wechselt den internen Zustand und setzt Zeitstempel.
     */
    void enterState(State s);
};