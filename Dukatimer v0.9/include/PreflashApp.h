/* =============================================================================
 * PreflashApp.h - DUKATIMER BETA (v0.913)
 * * Modus für die gezielte Vorbelichtung des Papiers zur Kontraststeuerung.
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Dosis-Integrität: Arbeitet ausschließlich mit Lux*Sekunden (Dosis).
 * 2. Wizard-Logik: Integrierter Kalibrierungs-Wizard zur Ermittlung der Schwelle.
 * 3. Non-Blocking: Nutzt die ExposureEngine als asynchronen Dienstleister.
 * ========================================================================== */

#pragma once

#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

class PreflashApp : public IAppMode
{
public:
    PreflashApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm);
    ~PreflashApp() override;

    // --- Lebenszyklus (IAppMode Interface) ---
    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    
    const char *getName() const override { return "PREFLASH"; }

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;
    PaperManager    *_pm;

    // Phasen der internen Zustandsmaschine (Wizard)
    enum WizState : uint8_t {
        WIZ_OFF,         // Normaler Modus: Einzelbelichtung bei Knopfdruck
        WIZ_SET_PARAMS,  // Einstellung: Anzahl Testschritte & Dosis-Inkrement
        WIZ_WAIT_START,  // Warten auf Start-Taste für den nächsten Puls
        WIZ_PULSING,     // Hardware führt Belichtung aktiv aus
        WIZ_PICK_THRESH, // Anwender wählt die Stufe vor der ersten Graureaktion
        WIZ_SAVE_EXIT    // Speichern der ermittelten flashThreshD ins Profil
    };

    WizState _wizState;
    int      _pfSteps;      // Anzahl der Teststreifen (z.B. 6)
    float    _pfDoseStep;   // Dosis pro Schritt (z.B. 0.10 Lux*s)
    int      _currentStep;  // Aktueller Index in der Messreihe
    uint8_t  _editFocus;    // 0 = Schritte, 1 = Dosis-Inkrement

    /**
     * Synchronisiert alle lokalen Zustände in den globalen SystemContext.
     */
    void _syncUI();

    /**
     * Setzt das Pending-Flag zur Einleitung einer Dosis-Belichtung.
     */
    void _startSingleFlash(float targetDose);
};