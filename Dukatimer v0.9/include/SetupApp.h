/* =============================================================================
 * SetupApp.h - DUKATIMER BETA (v0.912)
 * * Zentrale Einstellungs-App für Systemparameter und Kalibrierung.
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Transactional Editing: Änderungen werden erst bei "Save & Exit" persistent.
 * 2. Dependency Injection: Nutzt SensorManager für asynchrone Dark-Current Messung.
 * 3. Non-Blocking: Vermeidet Latenzen durch asynchrone State-Machine.
 * (+ FIX: Linker-Error behoben - processMenuInput entfernt)
 * (+ HÄRTUNG: override-Keywords für Interface-Integrität hinzugefügt)
 * ========================================================================== */

#pragma once

#include <Arduino.h>
#include "IAppMode.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

// Forward Declaration zur Vermeidung zirkulärer Abhängigkeiten
class SensorManager;

class SetupApp : public IAppMode
{
public:
    /**
     * Konstruktor mit vollständiger Dependency Injection.
     */
    SetupApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm);
    
    /**
     * Destruktor nutzt Standard-Implementierung.
     */
    ~SetupApp() override = default;

    // --- IAppMode Interface Implementierung ---
    void onEnter() override;
    void handleInput(int event) override;
    void onUpdate() override;
    void onExit() override;
    const char *getName() const override { return "SETUP"; }

private:
    // Externe Kern-Dienste
    SystemContext   *_ctx;
    HardwareManager *_hw;
    PaperManager    *_pm;
    SensorManager   *_sm;

    // --- Lokale Puffer (Cancel-Schutz) ---
    // Änderungen werden hier gesammelt und erst beim Speichern in den
    // globalen SystemContext zurückgeschrieben.
    SystemPreferences _editPrefs;
    PaperProfile      _editPaper;

    // --- Zustandssteuerung ---
    uint8_t _menuIdx;    // Aktueller Menüpunkt (0 bis 12)
    bool    _editActive; // Flag: Ist das aktuelle Feld im Edit-Modus?
    
    // Sub-State Machine für den Dark-Calibration Wizard:
    // 0: Menü-Anzeige / Inaktiv
    // 1: Aufforderung "Sensor abdecken"
    // 2: Messung läuft asynchron über SensorManager
    uint8_t _subState; 

    /**
     * Anzahl der verfügbaren Menüpunkte (10 Settings + 3 Actions).
     */
    static constexpr uint8_t MAX_MENU_ITEMS = 13;

    /**
     * Wendet Encoder-Deltas auf die lokalen Puffer an.
     * @param delta Positiver oder negativer Inkrementwert.
     */
    void applyValueChange(int delta);

    /**
     * Bereitet die Anzeige-Strings für den DisplayManager in der Union vor.
     */
    void renderUI();
};