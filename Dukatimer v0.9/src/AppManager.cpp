#include "AppManager.h"
#include "types.h"

/* =============================================================================
 * AppManager.cpp - DUKATIMER BETA (v0.916.12)
 * * Zentrale Event- und State-Routing Engine.
 * * FIX: Routing-Loch geschlossen. MODE_BW und MODE_SG laden nun korrekt
 * * die _appBWTime und _appSGTime Instanzen.
 * * FIX: Lazy Initialization der Queue (FreeRTOS Heap Protection).
 * * FIX: Globale Modus-Wechsel-Logik (EV_MODE_NEXT/PREV) integriert.
 * ========================================================================== */

AppManager::AppManager(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm)
    : _ctx(ctx), _hw(hw), _pm(pm), _sm(sm),
      _appLive(ctx, hw, pm),
      _appCalib(ctx, hw, pm, sm),
      _appDens(ctx, hw, pm, sm),
      _appBurn(ctx, hw, pm),
      _appPre(ctx, hw, pm),
      _appTest(ctx, hw, pm),
      _appBWDose(ctx, hw, pm),
      _appSGDose(ctx, hw, pm),
      _appBWFStop(ctx, hw, pm),
      _appZone(ctx, hw, pm),
      _appSetup(ctx, hw, pm, sm),
      _appBWTime(ctx, hw, pm), // NEU: Initialisierung BW Time App
      _appSGTime(ctx, hw, pm), // NEU: Initialisierung SG Time App
      _activeApp(nullptr)
{
    // WICHTIG: Hier keine FreeRTOS Objekte (Queues/Semaphore) erstellen!
    // Falls AppManager global in main.cpp instanziiert wird, crasht/fehlt das
    // stillschweigend, weil der FreeRTOS Kernel/Heap noch nicht bereit ist.
    _eventQueue = nullptr;
}

AppManager::~AppManager()
{
    if (_activeApp)
    {
        _activeApp->onExit();
    }

    if (_eventQueue)
    {
        vQueueDelete(_eventQueue);
    }
}

bool AppManager::switchMode(Mode newMode)
{
    // Schutz: Wechsle den Modus nicht während kritischer Hardware-Phasen
    WorkflowFlags flags;
    if (!_ctx->getFlags(flags))
        return false;

    // Moduswechsel sowohl bei Belichtung als auch bei Sensormessung sperren
    if (flags.isExposureRunning || flags.isMeasuring || flags.bwAutoPending || flags.sgAutoPending)
        return false;

    // Notifiziere die alte App über den Abbruch/Wechsel
    if (_activeApp)
    {
        _activeApp->onExit();
    }

    // Wähle neue App
    IAppMode *next = nullptr;
    switch (newMode)
    {
    case MODE_LIVE_VIEW:
        next = &_appLive;
        break;
    case MODE_BW:
        next = &_appBWTime;
        break; // FIX: Routing für BW Time
    case MODE_SG:
        next = &_appSGTime;
        break; // FIX: Routing für SG Time
    case MODE_BW_DOSE:
        next = &_appBWDose;
        break;
    case MODE_SG_DOSE:
        next = &_appSGDose;
        break;
    case MODE_BW_FSTOP:
        next = &_appBWFStop;
        break;
    case MODE_ZONE:
        next = &_appZone;
        break;
    case MODE_CALIBRATION:
        next = &_appCalib;
        break;
    case MODE_DENSITOMETER:
        next = &_appDens;
        break;
    case MODE_BURN:
        next = &_appBurn;
        break;
    case MODE_PREFLASH:
        next = &_appPre;
        break;
    case MODE_TEST_STRIP:
        next = &_appTest;
        break;
    case MODE_SETUP:
        next = &_appSetup;
        break;
    default:
        next = &_appLive;
        break;
    }

    _activeApp = next;

    // Aktualisiere den Context-Mode (thread-safe für andere Tasks)
    if (!_ctx->setMode(newMode))
        return false;

    // Starte die neue App
    if (_activeApp)
    {
        _activeApp->onEnter();
    }

    return true;
}

bool AppManager::postEvent(int event, bool toFront)
{
    // Lazy Initialization: Queue erst erstellen, wenn der ESP32 sicher bootet
    // und das erste Event gesendet wird (zu dem Zeitpunkt läuft setup() oder loop()).
    if (!_eventQueue)
    {
        _eventQueue = xQueueCreate(80, sizeof(int));
        if (!_eventQueue)
            return false; // Out of Memory
    }

    if (toFront)
    {
        // Not-Aus (ABORT) drängelt sich direkt an Position 0 vor.
        return (xQueueSendToFront(_eventQueue, &event, 0) == pdTRUE);
    }
    // Normale Events stellen sich hinten an.
    return (xQueueSend(_eventQueue, &event, 0) == pdTRUE);
}

void AppManager::handleInput(int event)
{
    if (!_activeApp)
        return;

    WorkflowFlags flags;
    if (_ctx->getFlags(flags))
    {
        // --- ABSOLUTER SECURITY EVENT GUARD ---
        if (flags.isExposureRunning || flags.isMeasuring)
        {
            if (event == EV_START)
            {
                _ctx->triggerAbort();
                if (_hw)
                {
                    _hw->allLightsOff();
                    _hw->setExposureLock(false);
                    _hw->playBeep(BEEP_ALARM);
                }
            }

            return;
        }
        else
        {
            // --- GLOBALES SYSTEM ROUTING ---
            // Moduswechsel über den dedizierten Mode-Encoder (Encoder 4)
            if (event == EV_MODE_NEXT || event == EV_MODE_PREV)
            {
                int current = static_cast<int>(flags.currentMode);
                int next = current + ((event == EV_MODE_NEXT) ? 1 : -1);

                // Wrap-Around über alle Modi (MODE_BW = 0 bis MODE_ZONE = 12)
                if (next > static_cast<int>(MODE_ZONE))
                    next = static_cast<int>(MODE_BW);
                if (next < static_cast<int>(MODE_BW))
                    next = static_cast<int>(MODE_ZONE);

                switchMode(static_cast<Mode>(next));
                return; // Event ist verarbeitet, nicht mehr an App weiterreichen
            }
        }
    }

    if (event == EV_GRADE_CLICK)
    {
        event = EV_ABORT;
    }
    else if (event == EV_TIME_CLICK)
    {
        switch (flags.currentMode)
        {
        case MODE_SETUP:
        case MODE_TEST_STRIP:
            event = EV_MENU_CLICK;
            break;
        default:
            break;
        }
    }

    _activeApp->handleInput(event);
}

void AppManager::onUpdate()
{
    // Lazy Initialization: Verhindert Absturz, falls onUpdate vor erstem Event läuft
    if (!_eventQueue)
    {
        _eventQueue = xQueueCreate(80, sizeof(int));
        if (!_eventQueue)
            return;
    }

    // 1. Queue abarbeiten: Pollt alle angestauten Inputs threadsicher ab.
    int queuedEvent;
    while (xQueueReceive(_eventQueue, &queuedEvent, 0) == pdTRUE)
    {
        handleInput(queuedEvent);
    }

    // 2. Laufende App aktualisieren
    if (_activeApp)
    {
        _activeApp->onUpdate();
    }
}