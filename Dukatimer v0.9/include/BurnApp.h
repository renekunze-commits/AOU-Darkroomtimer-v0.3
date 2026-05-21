/* =============================================================================
 * BurnApp.h - DUKATIMER BETA (v0.913)
 * * Modus für selektive Nachbelichtung (Abwedeln/Nachbelichten).
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Relative Dosis: Berechnet Zusatzmenge basierend auf f-Stop-Differenz.
 * 2. Unabhängige Gradation: Erlaubt andere Filterung als die Hauptbelichtung.
 * 3. Asynchron: Nutzt die ExposureEngine Pull-Brücke im Dosis-Modus.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class BurnApp : public IAppMode
{
public:
    BurnApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~BurnApp() override;

    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    
    const char *getName() const override { return "BURN"; }

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;
    PaperManager    *_pm;

    float   _burnEv;        // Gewünschte Zusatzbelichtung in Blendenstufen
    float   _burnGrade;     // Gradation für den Burn-Vorgang
    float   _baseRefDose;   // Die 100%-Referenz (letzte Basisbelichtung)
    uint8_t _stepModeIdx;   // Aktuelle Schrittweite des Encoders

    void _syncUI();
    float _calculateTargetDose();
    float _getEvStep();
};