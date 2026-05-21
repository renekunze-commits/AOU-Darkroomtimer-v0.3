/* =============================================================================
 * HardwareManager.h - DUKATIMER BETA (v0.916.11)
 * * Zentrale HAL (Hardware Abstraction Layer).
 * * GEHÄRTETE VERSION: Inklusive State-Caching und I2C-Mutex-Management.
 * * FIX: Deklaration von setRoomlight() und _stateRoomlight hinzugefügt.
 * ========================================================================== */

#pragma once

#include <atomic>
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "SystemContext.h"

// --- Typensichere Definitionen ---
enum BeepType
{
    BEEP_TICK, // Metronom-Klick (sehr kurz)
    BEEP_OK,   // Bestätigung (z.B. ENTER)
    BEEP_WARN, // Warnung / Fehler
    BEEP_SAVE, // Langer Speicherton (Wizard)
    BEEP_ALARM // Overheat / Hard Abort
};

class HardwareManager
{
private:
    SystemContext *_ctx;

    // Thread-Sicherheit & Lockouts
    std::atomic<bool> _exposureLockActive{false};
    // IMPORTANT: This mutex protects only I2C-0 (Wire).
    // I2C-1 (Wire1) is Core-1 exclusive and intentionally lock-free.
    SemaphoreHandle_t _i2cMutex;

    // State-Caching (Vermeidung redundanter GPIO-Aufrufe)
    bool _stateSafelight;
    bool _stateFocus;
    bool _stateEnlarger;
    bool _stateRoomlight; // Software-Override für Darkness/Blackout
    bool _focusLatch;
    uint8_t _currentR, _currentG, _currentB;

    void renderLightOutputs();
    uint8_t getSafelightPwm() const;
    uint8_t getFocuslightPwm() const;

public:
    HardwareManager(SystemContext *ctx);
    ~HardwareManager();

    // Initialisierung der Pins & I2C-Busse (Wire + Wire1)
    void init();

    // =========================================================================
    // I2C BUS MANAGEMENT
    // =========================================================================
    // Exposure lock gate for Core-0 low-priority accesses.
    void setExposureLock(bool locked);

    // Mutex API for I2C-0 (Wire) only.
    // Use for LCD/BMP280/TSL2591 transactions on Core 0.
    bool takeI2C();
    void giveI2C();

    // Bus accessors for explicit sensor-to-bus binding.
    // Core 0 slow peripherals: LCD/BMP280/TSL2591.
    TwoWire *getI2C0Bus() { return &Wire; }
    // Core 1 realtime dose sensor: TSL2561 (no mutex by design).
    TwoWire *getI2C1Bus() { return &Wire1; }

    // =========================================================================
    // LICHTSTEUERUNG (Zentrale Autorität)
    // =========================================================================
    void setSafelight(bool active);
    void setFocuslight(bool active);
    void setEnlarger(bool active);
    void setRoomlight(bool active); // Software-Override für Dunkelheit/Relais
    void syncPhysicalLights();      // v0.3: Licht-Hierarchie & Latch aus internen State-Flags
    bool isDarknessModeActive() const;

    // Not-Aus oder Blackout
    void allLightsOff();

    // =========================================================================
    // SPEKTRAL-STEUERUNG (NeoPixel)
    // =========================================================================
    void updateNeoPixels(uint8_t r, uint8_t g, uint8_t b);

    // =========================================================================
    // AKUSTISCHES FEEDBACK
    // =========================================================================
    void playBeep(BeepType type);
};