#include <Arduino.h>
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"
#include "StorageManager.h"
#include "AppManager.h"
#include "InputManager.h"
#include "ExposureEngine.h"
#include "SensorManager.h"
#include "DisplayManager.h" // FIX: DisplayManager muss inkludiert sein

/* =============================================================================
 * main.cpp - DUKATIMER BETA (v0.912)
 * (+ T3 Fix: Komplette System-Instanziierung & Bootstrapping)
 * (+ Fix: StorageManager Konstruktor Parameter)
 * (+ Prio 0.1: SensorManager als Kern-Dienstleister integriert)
 * (+ FIX: AppManager Dependency Injection vervollständigt)
 * (+ FIX: DisplayManager wiederhergestellt)
 * ========================================================================== */

// --- Kern-Manager (Statische Allokation im BSS) ---

SystemContext context;
HardwareManager hw(&context);
PaperManager papers(&context);
StorageManager storage(&context);

// Verwaltet langsame Umweltsensoren und hochpräzise Spotmessungen auf Core 0
SensorManager sensorManager(&context, &hw);

// AppManager routet Events und verwaltet die Fotolabor-Zustandsmaschinen
// FIX: SensorManager-Pointer wird nun übergeben!
AppManager appManager(&context, &hw, &papers, &sensorManager);

// Steuert den Core 1 Task für die 10ms-Dosisintegration
ExposureEngine engine(&context, &hw);

// Liest PCNT (Hardware-Encoder) und Tasten via Debouncing
InputManager inputManager(&appManager, &hw);

// FIX: Steuert LCD und Nextion
DisplayManager displayManager(&context, &hw);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[SYSTEM] Dukatimer v0.912 starting...");

    // 1. Hardware-Basis initialisieren (Pins, I2C-Busse, LCD)
    hw.init();
    displayManager.init(); // FIX: Display initialisieren

    // 2. Daten-Infrastruktur aufbauen und registrieren
    papers.init();
    storage.init();
    storage.registerProvider(&context);
    storage.registerProvider(&papers);

    // 3. Sensorik initialisieren (I2C-0 & OneWire auf Core 0)
    sensorManager.init();

    // 4. App & Engine Integration
    engine.init();       // Startet den dedizierten ExposureCL Task auf Core 1
    inputManager.init(); // Bindet die Hardware-Encoder an den PCNT-Block
    appManager.switchMode(MODE_BW_DOSE);
    Serial.println("[SYSTEM] Boot complete.");
}

void loop() {
    // --- STRIKTE AUFRUF-REIHENFOLGE (Core 0 Loop) ---
    // Diese Reihenfolge minimiert Latenzen zwischen Sensor-Input und UI-Feedback.

    // 1. Speicher-Verwaltung (Deferred Writes)
    // Zieht clearDirty() vor, schreibt asynchron ins Flash (geschützt durch isExposureRunning)
    storage.process();

    // 2. Sensor-Updates (Asynchron & Polling)
    sensorManager.update();

    // 3. Eingaben holen & Events an AppManager senden
    inputManager.process(); 

    // 4. App-Logik aktualisieren
    appManager.onUpdate();

    // 5. UI aktualisieren
    // FIX: DisplayManager rendert die neuen States auf LCD/Nextion
    displayManager.update();

    // 6. Belichtungs-State-Machine (Core 0 Bridge)
    engine.tick();

    // Kurze Atempause für den FreeRTOS Hintergrund-Task
    vTaskDelay(1);
}