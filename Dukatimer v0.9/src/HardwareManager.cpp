#include "HardwareManager.h"
#include <NeoPixelBus.h>

/* =============================================================================
 * HardwareManager.cpp - DUKATIMER BETA (v0.916.11)
 * * ECHTES FUNDAMENT:
 * * 1. PIMPL-artige Kapselung des NeoPixelBus (hält Header sauber).
 * * 2. FreeRTOS Software Timer für asynchrone, nicht-blockierende Töne.
 * * (+ FIX: Implementierung von setRoomlight inkl. State-Caching)
 * ========================================================================== */

// --- Versteckte Hardware-Instanzen (Niemals im Header deklarieren!) ---
// Nutzt die Neo800KbpsMethod (Standard für WS2812x am ESP32-S3 via RMT)
static NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> *neoStrip = nullptr;

// FreeRTOS Timer für den Buzzer (Verhindert Blockieren des UI-Tasks)
static TimerHandle_t buzzerTimer = nullptr;

// Fester LEDC Channel für den Buzzer, um dynamisches Setup-Blocking zu vermeiden
static const uint8_t BUZZER_LEDC_CHANNEL = 0;

// Callback: Wird vom RTOS aufgerufen, wenn die Ton-Dauer abgelaufen ist
static void buzzerOffCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    // Hardware-PWM direkt abschalten (strikt non-blocking, ohne Timer-Teardown)
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);
}

// =============================================================================

HardwareManager::HardwareManager(SystemContext *ctx) : _ctx(ctx),
                                                       _exposureLockActive(false),
                                                       _stateSafelight(false),
                                                       _stateFocus(false),
                                                       _stateEnlarger(false),
                                                       _stateRoomlight(false),
                                                       _focusLatch(false),
                                                       _currentR(0), _currentG(0), _currentB(0)
{
    _i2cMutex = xSemaphoreCreateMutex();
}

HardwareManager::~HardwareManager()
{
    if (_i2cMutex)
        vSemaphoreDelete(_i2cMutex);
    if (neoStrip)
    {
        delete neoStrip;
        neoStrip = nullptr;
    }
    if (buzzerTimer)
    {
        xTimerDelete(buzzerTimer, 0);
    }
}

void HardwareManager::init()
{
    // 1. Relais initialisieren
#ifdef PIN_RELAY_ENLARGER
    pinMode(PIN_RELAY_ENLARGER, OUTPUT);
#endif
#ifdef PIN_RELAY_SAFE
    pinMode(PIN_RELAY_SAFE, OUTPUT);
#endif
#ifdef PIN_RELAY_FOCUS
    pinMode(PIN_RELAY_FOCUS, OUTPUT);
#endif
#ifdef PIN_RELAY_ROOMLIGHT // NEU: Init für das Raumlicht Relais
    pinMode(PIN_RELAY_ROOMLIGHT, OUTPUT);
#endif

    // 2. Audio-Hardware (LEDC) fest vorkonfigurieren statt dynamischem tone()
    // Setup: Channel 0, 2000Hz Standard, 8-bit Resolution
    ledcSetup(BUZZER_LEDC_CHANNEL, 2000, 8);
    ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CHANNEL);
    ledcWrite(BUZZER_LEDC_CHANNEL, 0); // Initial garantiert stumm

    // 3. FreeRTOS Timer für Audio anlegen (Auto-Reload = pdFALSE -> One-Shot)
    buzzerTimer = xTimerCreate("BuzzerTmr", pdMS_TO_TICKS(10), pdFALSE, (void *)0, buzzerOffCallback);

    // 4. NeoPixel Hardware hochfahren
    if (!neoStrip)
    {
        neoStrip = new NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>(NEOPIXEL_COUNT, PIN_NEOPIXEL);
        neoStrip->Begin();
        neoStrip->ClearTo(RgbColor(0, 0, 0));
        neoStrip->Show();
    }

    allLightsOff();
    renderLightOutputs();

    // I2C-0 (Wire): Core 0 low-priority peripherals (LCD/BMP280/TSL2591)
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    // I2C-1 (Wire1): Core 1 realtime dose path (TSL2561), no shared mutex
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_FREQ);

    // Sensor bus binding examples for future SensorManager integration:
    // - TSL2561 (dose, Core 1):    tsl2561.begin(&Wire1, ADDR_TSL2561);
    // - BMP280 (ambient, Core 0):  bmp280.begin(ADDR_BMP280, &Wire);
    // - TSL2591 (probe, Core 0):   tsl2591.begin(ADDR_TSL2591, &Wire);
}

// =============================================================================
// I2C BUS MANAGEMENT
// NOTE: _i2cMutex, takeI2C(), giveI2C() protect only I2C-0 (Wire).
// I2C-1 (Wire1) remains lock-free because it is Core-1 exclusive.
// =============================================================================

void HardwareManager::setExposureLock(bool locked)
{
    _exposureLockActive.store(locked);
    if (locked)
    {
        // Zwingender Blackout: Physische Sicherheit geht vor
        _stateSafelight = false;
        _stateFocus = false;
        renderLightOutputs();
    }
}

bool HardwareManager::takeI2C()
{
    // CRITICAL: This lock is ONLY for I2C-0 (Wire).
    // Wire1 (I2C-1) is physically isolated to Core 1 and MUST NEVER be mutex-blocked.
    if (_exposureLockActive.load())
        return false;
    // 20ms timeout for I2C-0 only (Wire).
    // Prevents long stalls if a low-priority peripheral blocks.
    if (_i2cMutex && xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        // Doppel-Check: Wurde das Lock während unserer Wartezeit aktiviert?
        if (_exposureLockActive.load())
        {
            xSemaphoreGive(_i2cMutex);
            return false;
        }
        return true;
    }
    return false;
}

void HardwareManager::giveI2C()
{
    // CRITICAL: Releases only the I2C-0 (Wire) mutex.
    // Do not gate or serialize Wire1 (I2C-1) via this or any software lock.
    if (_i2cMutex)
    {
        xSemaphoreGive(_i2cMutex);
    }
}

// =============================================================================
// LICHT & AKTOREN
// =============================================================================

void HardwareManager::setSafelight(bool active)
{
    if (_stateSafelight && !active && _stateFocus)
    {
        _focusLatch = true;
    }

    if (_stateSafelight == active)
    {
        return;
    }

    _stateSafelight = active;
    syncPhysicalLights();
}

void HardwareManager::setFocuslight(bool active)
{
    if (!active)
    {
        _focusLatch = false;
    }

    if (_stateFocus == active)
    {
        return;
    }

    _stateFocus = active;
    syncPhysicalLights();
}

void HardwareManager::setEnlarger(bool active)
{
    if (_stateEnlarger == active)
    {
        return;
    }

    _stateEnlarger = active;
    if (!active)
    {
        _currentR = 0;
        _currentG = 0;
        _currentB = 0;
    }
    renderLightOutputs();
}

void HardwareManager::setRoomlight(bool active)
{
    if (_stateRoomlight == active)
    {
        return;
    }

    _stateRoomlight = active;
    syncPhysicalLights();
}

void HardwareManager::syncPhysicalLights()
{
    if (_stateSafelight && _stateFocus)
    {
        _focusLatch = true;
    }

    if (!_stateFocus)
    {
        _focusLatch = false;
    }

    renderLightOutputs();
}

bool HardwareManager::isDarknessModeActive() const
{
    return _stateRoomlight;
}

void HardwareManager::allLightsOff()
{
    _stateEnlarger = false;
    _stateSafelight = false;
    _stateFocus = false;
    _currentR = 0;
    _currentG = 0;
    _currentB = 0;
    renderLightOutputs();
}

void HardwareManager::updateNeoPixels(uint8_t r, uint8_t g, uint8_t b)
{
    if (_currentR == r && _currentG == g && _currentB == b && _stateEnlarger == (r != 0 || g != 0 || b != 0))
        return;

    _currentR = r;
    _currentG = g;
    _currentB = b;
    _stateEnlarger = (r != 0 || g != 0 || b != 0);
    renderLightOutputs();
}

uint8_t HardwareManager::getSafelightPwm() const
{
    SystemPreferences prefs;
    if (_ctx && _ctx->getPreferences(prefs))
    {
        return prefs.pwmSafe;
    }

    return 100;
}

uint8_t HardwareManager::getFocuslightPwm() const
{
    SystemPreferences prefs;
    if (_ctx && _ctx->getPreferences(prefs))
    {
        return prefs.pwmFocus;
    }

    return 255;
}

void HardwareManager::renderLightOutputs()
{
    const bool darknessActive = isDarknessModeActive();
    const bool safelightActive = !darknessActive && _stateSafelight;
    const bool focusActive = !darknessActive && !safelightActive && !_focusLatch && _stateFocus;
    const bool enlargerActive = !darknessActive && _stateEnlarger;

#ifdef PIN_RELAY_ROOMLIGHT
    digitalWrite(PIN_RELAY_ROOMLIGHT, darknessActive ? LOW : HIGH);
#endif

#ifdef PIN_RELAY_SAFE
    digitalWrite(PIN_RELAY_SAFE, safelightActive ? HIGH : LOW);
#endif

#ifdef PIN_RELAY_FOCUS
    digitalWrite(PIN_RELAY_FOCUS, focusActive ? HIGH : LOW);
#endif

#ifdef PIN_RELAY_ENLARGER
    digitalWrite(PIN_RELAY_ENLARGER, enlargerActive ? HIGH : LOW);
#endif

    if (neoStrip)
    {
        RgbColor targetColor(0, 0, 0);

        if (enlargerActive)
        {
            targetColor = RgbColor(_currentR, _currentG, _currentB);
        }
        else if (safelightActive)
        {
            targetColor = RgbColor(getSafelightPwm(), 0, 0);
        }
        else if (focusActive)
        {
            const uint8_t focusPwm = getFocuslightPwm();
            targetColor = RgbColor(focusPwm, focusPwm, focusPwm);
        }

        neoStrip->ClearTo(targetColor);
        neoStrip->Show();
    }
}

// =============================================================================
// AUDIO (RTOS Asynchron)
// =============================================================================

void HardwareManager::playBeep(BeepType type)
{
    uint16_t freq = 0;
    uint32_t duration = 0;

    switch (type)
    {
    case BEEP_TICK:
        freq = 800;
        duration = 10;
        break;
    case BEEP_OK:
        freq = 2000;
        duration = 50;
        break;
    case BEEP_WARN:
        freq = 400;
        duration = 200;
        break;
    case BEEP_SAVE:
        freq = 1000;
        duration = 400;
        break;
    case BEEP_ALARM:
        freq = 300;
        duration = 1000;
        break;
    }

    if (freq > 0 && duration > 0)
    {
        // FIX: Hardware-gebackenes LEDC Setup blockiert den Main-Loop nicht!
        ledcWriteTone(BUZZER_LEDC_CHANNEL, freq);

        // RTOS Timer beauftragen, den Ton nach 'duration' Millisekunden abzustellen
        if (buzzerTimer)
        {
            xTimerChangePeriod(buzzerTimer, pdMS_TO_TICKS(duration), 0);
            xTimerStart(buzzerTimer, 0);
        }
    }
}