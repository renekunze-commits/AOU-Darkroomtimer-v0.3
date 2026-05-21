/* =============================================================================
 * AppManager.h - DUKATIMER BETA (v0.916.12)
 * * Zentrale Routing-Engine für App-Modi und Events.
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Dependency Injection: Verwaltet alle App-Instanzen und reicht
 * Hardware/Sensoren/Context durch.
 * 2. Event-Guard: Blockiert kritische Benutzereingaben auf Systemebene.
 * 3. Asynchrones Event-Handling: FreeRTOS Queue Entkopplung (mit Priorität).
 * * FIX: BWTimeApp und SGTimeApp für die reinen Zeitmodi integriert.
 * ========================================================================== */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

// Forward Declaration zur Vermeidung zirkulärer Header-Abhängigkeiten
class SensorManager;

#include "IAppMode.h"
#include "LiveViewApp.h"
#include "BWDoseApp.h"
#include "SGDoseApp.h"
#include "BWFStopApp.h"
#include "ZoneModeApp.h"
#include "CalibrationApp.h"
#include "DensApp.h"
#include "BurnApp.h"
#include "PreflashApp.h"
#include "TestStripApp.h"
#include "SetupApp.h" 

// NEU: Header für die reinen Zeit-Modi
#include "BWTimeApp.h"
#include "SGTimeApp.h"

class AppManager
{
public:
    AppManager(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm);
    ~AppManager();

    bool switchMode(Mode newMode);
    
    // Asynchrone Schnittstelle für den InputManager.
    // toFront = true drängt das Event an die Spitze der Queue (für EV_ABORT / Not-Aus).
    // Gibt true zurück, wenn das Event erfolgreich in die Queue geschrieben wurde.
    bool postEvent(int event, bool toFront = false);
    
    // Legacy/Synchroner Handler (wird intern von onUpdate aufgerufen)
    void handleInput(int event);
    
    // Main-Loop Update
    void onUpdate();

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;
    PaperManager    *_pm;
    SensorManager   *_sm; 

    // FreeRTOS Queue für die vollständige Entkopplung der Eingaben
    QueueHandle_t   _eventQueue;

    // Instanziierung aller App-Modi (Lebensdauer an AppManager gebunden)
    LiveViewApp    _appLive;
    CalibrationApp _appCalib;
    DensApp        _appDens;
    BurnApp        _appBurn;
    PreflashApp    _appPre;
    TestStripApp   _appTest;
    BWDoseApp      _appBWDose;
    SGDoseApp      _appSGDose;
    BWFStopApp     _appBWFStop;
    ZoneModeApp    _appZone;
    SetupApp       _appSetup; 
    
    // NEU: Instanzen der Zeit-Apps
    BWTimeApp      _appBWTime;
    SGTimeApp      _appSGTime;

    IAppMode *_activeApp;
};