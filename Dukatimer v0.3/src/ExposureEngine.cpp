/* ExposureEngine.cpp - Universal Exposure Engine (Phase 2 B2 Final)
   
   EINZIGE Belichtungs-Engine für ALLE Modi (BW, SG, Burn, TestStrip).
   
   Architektur:
   - esp_timer Hardware-Interrupt für <1ms Shutoff-Präzision
   - Predictive Shutoff: Bei ≥95% Zieldosis → Restzeit berechnen, esp_timer armen
   - NeoPixel-Latch-Kompensation: ~7.7ms (256 LEDs × 30µs + 50µs) abgezogen
   - Closed-Loop Task (10ms Periode): Liest TSL2561, akkumuliert Dosis
   
   Guard-Times:
     PRE_WAIT:  200ms (Relais-Settle, Blackout)
     POST_WAIT: 400ms (Nachleuchten-Abklingen)
   
   Thread-Kontext:
   - Tick() läuft auf Core 1 (vTaskRealtime)
   - CL-Task läuft auf Core 1 (Prio 10, vExposureCLTask)
   - esp_timer Callback läuft im ESP_TIMER_TASK Kontext
*/

#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Adafruit_TSL2561_U.h>
#include "ExposureEngine.h"
#include "Globals.h"

// =============================================================================
// KONSTANTEN
// =============================================================================
#ifndef NEOPIXEL_COUNT
#define NEOPIXEL_COUNT 256
#endif

static const uint64_t NEOPIXEL_LATCH_DELAY_US = (NEOPIXEL_COUNT * 30) + 50;
static const unsigned long PRE_WAIT_MS  = 200;
static const unsigned long POST_WAIT_MS = 400;
static const TickType_t ENGINE_MUTEX_TIMEOUT = pdMS_TO_TICKS(100);

// =============================================================================
// INTERNER ZUSTAND
// =============================================================================
static volatile ExposureState expState = EXP_IDLE;
static ExposureMode expMode            = EXPMODE_TIME;
static unsigned long expPhaseStart     = 0;
static unsigned long expDurationMs     = 0;
static unsigned long expRemainingMs    = 0;
static double        expTargetDose     = 0.0;
static uint8_t       expGreen          = 0;
static uint8_t       expBlue           = 0;

// Flag: Graceful Stop → POST_WAIT geht direkt zu IDLE statt DONE
static volatile bool expAbortFlag      = false;

// =============================================================================
// ESP_TIMER: Predictive Shutoff
// =============================================================================
static esp_timer_handle_t expShutoffTimer = NULL;
static volatile bool expShutoffArmed     = false;
static volatile bool expShutoffFired     = false;

// ISR-Kontext: Muss minimal bleiben — nur LEDs aus + Flag setzen
static void expShutoffCallback(void* arg) {
    (void)arg;
    HW_EmergencyShutoff();
    expShutoffFired = true;
}

static void stopShutoffTimer() {
    if (expShutoffTimer != NULL && expShutoffArmed) {
        esp_timer_stop(expShutoffTimer);
    }
    expShutoffArmed = false;
    expShutoffFired = false;
}

static bool lockEngineTimerMutex(const char* context) {
    if (gTimerMutex == NULL) {
        saveErrorState(ERR_MUTEX_TIMEOUT);
        Serial.printf("[EXP] gTimerMutex fehlt in %s.\n", context);
        return false;
    }

    if (xSemaphoreTake(gTimerMutex, ENGINE_MUTEX_TIMEOUT) != pdTRUE) {
        saveErrorState(ERR_MUTEX_TIMEOUT);
        Serial.printf("[EXP] Mutex-Timeout in %s.\n", context);
        return false;
    }

    return true;
}

static void emergencyExposureStop(SystemError err, const char* l1, const char* l2, SoundID sound) {
    stopShutoffTimer();
    HW_SetEnlargerNeoPixel(0, 0, 0);
    HW_SetBlackout(false);
    if (safeLatch) HW_SetSafelight(true);

    isMeasuring = false;
    isPaused = false;
    lightOperationActive = false;
    expAbortFlag = false;
    expState = EXP_IDLE;

    saveErrorState(err);
    smartLCD(l1, l2);
    uiTriggerBeep(sound);
}

static bool canStartExposure(bool needsLiveSensor, const char* context) {
    if (expState != EXP_IDLE) {
        return false;
    }

    if (overheatLock) {
        emergencyExposureStop(ERR_THERMAL_OVERHEAT, "OVERHEAT", "ENGINE LOCKED", SND_ALARM);
        Serial.printf("[EXP] Start in %s wegen Overheat blockiert.\n", context);
        return false;
    }

    if (needsLiveSensor && !tslLiveOK) {
        saveErrorState(ERR_SENSOR_DISCONNECTED);
        smartLCD("LIVE SENSOR", "REQUIRED");
        uiTriggerBeep(SND_LIMIT);
        Serial.printf("[EXP] Start in %s ohne Live-Sensor blockiert.\n", context);
        return false;
    }

    return true;
}

// =============================================================================
// CLOSED-LOOP TASK: TSL2561 Sensor → Dosis-Akkumulation → Predictive Shutoff
// =============================================================================
static unsigned long clLastMeasMs = 0;

static void vExposureCLTask(void* pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t lastWake = xTaskGetTickCount();
    sensors_event_t event;

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        // Nur aktiv wenn Engine im Dosis-Modus belichtet
        if (expState != EXP_EXPOSING || expMode != EXPMODE_DOSE) {
            clLastMeasMs = millis();
            continue;
        }

        // TSL2561 auslesen (I2C Bus 1, schneller Bus)
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (tslHead.getEvent(&event)) {
                unsigned long now = millis();
                double dt = (now - clLastMeasMs) / 1000.0;
                if (dt <= 0) dt = 0.01;
                clLastMeasMs = now;
                baseFlux = event.light;

                if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                    current_dose += (event.light * dt);
                    xSemaphoreGive(gTimerMutex);
                }
            }
            xSemaphoreGive(xI2CMutex);
        }

        // Predictive Shutoff: Bei ≥95% Zieldosis esp_timer armen
        if (!expShutoffArmed) {
            double remaining = expTargetDose - current_dose;

            if (remaining <= 0.0) {
                // Zieldosis überschritten → sofort abschalten
                expShutoffCallback(NULL);
                expShutoffArmed = true;
            }
            else if (current_dose >= (expTargetDose * 0.95) || remaining < (baseFlux * 0.1)) {
                if (baseFlux > 0.01) {
                    uint64_t rem_us = (uint64_t)((remaining / baseFlux) * 1000000.0);
                    // Sniper-Fix: NeoPixel-Latch-Latenz abziehen
                    if (rem_us > NEOPIXEL_LATCH_DELAY_US + 500) {
                        rem_us -= NEOPIXEL_LATCH_DELAY_US;
                    } else {
                        rem_us = 500;
                    }
                    esp_timer_start_once(expShutoffTimer, rem_us);
                    expShutoffArmed = true;
                }
            }
        }
    }
}

// =============================================================================
// INITIALISIERUNG
// =============================================================================
void ExposureEngine_Init() {
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &expShutoffCallback;
    timer_args.name = "exp_predictive_shutoff";
    timer_args.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&timer_args, &expShutoffTimer);

    xTaskCreatePinnedToCore(vExposureCLTask, "ExpCL", 6144, NULL, 10, NULL, 1);
}

// =============================================================================
// INTERNE HILFSFUNKTION: Belichtung starten (nach Pre-Wait)
// =============================================================================
static void expBeginExposure() {
    // Licht einschalten
    HW_SetEnlargerNeoPixel(0, expGreen, expBlue);
    uiTriggerBeep(SND_START_PATTERN);

    expPhaseStart   = millis();
    expShutoffFired = false;
    expShutoffArmed = false;

    if (expMode == EXPMODE_TIME) {
        // Zeit-Modus: esp_timer sofort armen (mit NeoPixel-Latch-Kompensation)
        uint64_t duration_us = (uint64_t)expDurationMs * 1000ULL;
        if (duration_us > NEOPIXEL_LATCH_DELAY_US) {
            duration_us -= NEOPIXEL_LATCH_DELAY_US;
        } else {
            duration_us = 100;
        }
        esp_timer_start_once(expShutoffTimer, duration_us);
        expShutoffArmed = true;
    }
    // Dosis-Modus: CL-Task übernimmt das Armen des esp_timer

    expState = EXP_EXPOSING;
}

// =============================================================================
// START-FUNKTIONEN
// =============================================================================
void ExposureEngine_StartTime(unsigned long durationMs, uint8_t green, uint8_t blue,
                              bool skipPreWait) {
    if (!canStartExposure(false, "ExposureEngine_StartTime")) return;

    expMode       = EXPMODE_TIME;
    expDurationMs = (durationMs < 100) ? 100 : durationMs;
    expGreen      = green;
    expBlue       = blue;
    expAbortFlag  = false;
    isPaused      = false;

    isMeasuring          = true;
    lightOperationActive = true;

    if (skipPreWait) {
        // Aufrufer (startTimer) hat Blackout/Pre-Wait bereits erledigt
        expBeginExposure();
    } else {
        // Burn/TestStrip: Engine übernimmt Pre-Wait
        HW_SetBlackout(true);
        HW_SetSafelight(false);
        expPhaseStart = millis();
        expState = EXP_PRE_WAIT;
    }
}

void ExposureEngine_StartDose(double targetDose, uint8_t green, uint8_t blue,
                              bool skipPreWait) {
    if (!canStartExposure(true, "ExposureEngine_StartDose")) return;

    if (isnan(targetDose) || isinf(targetDose) || targetDose <= 0.0) {
        saveErrorState(ERR_MATH_INVALID);
        smartLCD("DOSE INVALID", "START BLOCKED");
        uiTriggerBeep(SND_LIMIT);
        return;
    }

    expMode       = EXPMODE_DOSE;
    expTargetDose = targetDose;
    expGreen      = green;
    expBlue       = blue;
    expAbortFlag  = false;
    isPaused      = false;

    // Globale Dosis-Variablen unter Mutex initialisieren
    if (!lockEngineTimerMutex("ExposureEngine_StartDose")) {
        smartLCD("LOCK TIMEOUT", "DOSE START");
        return;
    }
    current_dose = 0.0;
    target_dose  = targetDose;
    xSemaphoreGive(gTimerMutex);

    isMeasuring          = true;
    lightOperationActive = true;

    if (skipPreWait) {
        expBeginExposure();
    } else {
        HW_SetBlackout(true);
        HW_SetSafelight(false);
        expPhaseStart = millis();
        expState = EXP_PRE_WAIT;
    }
}

// =============================================================================
// TICK: Zyklische State-Machine (aus vTaskRealtime aufrufen)
// =============================================================================
void ExposureEngine_Tick() {
    unsigned long now = millis();

    if (overheatLock && expState != EXP_IDLE) {
        emergencyExposureStop(ERR_THERMAL_OVERHEAT, "OVERHEAT", "ENGINE STOP", SND_ALARM);
        return;
    }

    switch (expState) {
        case EXP_IDLE:
        case EXP_DONE:
            return;

        case EXP_PRE_WAIT:
            if ((now - expPhaseStart) >= PRE_WAIT_MS) {
                expBeginExposure();
            }
            break;

        case EXP_PAUSED:
            return;

        case EXP_EXPOSING: {
            unsigned long elapsed = now - expPhaseStart;
            handleExposureMetronome(elapsed);

            if (expShutoffFired) {
                // esp_timer hat LEDs bereits abgeschaltet → POST_WAIT
                expPhaseStart = now;
                expState = EXP_POST_WAIT;
            }
            // Safety-Fallback: millis()-Check mit 100ms Toleranz (nur Zeit-Modus)
            else if (expMode == EXPMODE_TIME && elapsed >= expDurationMs + 100) {
                HW_SetEnlargerNeoPixel(0, 0, 0);
                expPhaseStart = now;
                expState = EXP_POST_WAIT;
            }
            break;
        }

        case EXP_POST_WAIT:
            if ((now - expPhaseStart) >= POST_WAIT_MS) {
                HW_SetBlackout(false);
                if (safeLatch) HW_SetSafelight(true);
                isMeasuring          = false;
                lightOperationActive = false;
                uiTriggerBeep(SND_END_PATTERN);

                // Graceful Stop → direkt zu IDLE (kein Acknowledge nötig)
                // Normale Beendigung → DONE (wartet auf Acknowledge)
                expState = expAbortFlag ? EXP_IDLE : EXP_DONE;
                expAbortFlag = false;
            }
            break;
    }
}

// =============================================================================
// ABORT: Sofort-Abbruch ohne Post-Wait (Burn/TestStrip Cancel)
// =============================================================================
void ExposureEngine_Abort() {
    if (expState == EXP_IDLE) return;

    stopShutoffTimer();

    HW_SetEnlargerNeoPixel(0, 0, 0);
    HW_SetBlackout(false);
    if (safeLatch) HW_SetSafelight(true);
    isMeasuring          = false;
    isPaused             = false;
    lightOperationActive = false;
    expState = EXP_IDLE;
}

// =============================================================================
// STOP: Graceful Stop mit Post-Wait (Haupt-Timer User-Abort)
// =============================================================================
void ExposureEngine_Stop() {
    if (expState == EXP_IDLE || expState == EXP_DONE || expState == EXP_POST_WAIT) return;

    stopShutoffTimer();

    HW_SetEnlargerNeoPixel(0, 0, 0);
    isPaused = false;

    if (expState == EXP_PRE_WAIT) {
        // Kein Licht emittiert → kein Post-Wait nötig
        HW_SetBlackout(false);
        if (safeLatch) HW_SetSafelight(true);
        isMeasuring          = false;
        lightOperationActive = false;
        expState = EXP_IDLE;
        return;
    }

    // Licht war an → Post-Wait für Phosphor-Abklingen
    expAbortFlag  = true;
    expPhaseStart = millis();
    expState = EXP_POST_WAIT;
}

bool ExposureEngine_Pause() {
    if (expState != EXP_EXPOSING) return false;

    stopShutoffTimer();

    if (expMode == EXPMODE_TIME) {
        unsigned long elapsed = millis() - expPhaseStart;
        expRemainingMs = (elapsed >= expDurationMs) ? 1 : (expDurationMs - elapsed);
    }

    HW_SetEnlargerNeoPixel(0, 0, 0);
    isPaused = true;
    expState = EXP_PAUSED;
    return true;
}

bool ExposureEngine_Resume() {
    if (expState != EXP_PAUSED) return false;
    if (overheatLock) {
        emergencyExposureStop(ERR_THERMAL_OVERHEAT, "OVERHEAT", "RESUME BLOCK", SND_ALARM);
        return false;
    }

    HW_SetEnlargerNeoPixel(0, expGreen, expBlue);
    expPhaseStart = millis();
    expShutoffFired = false;

    if (expMode == EXPMODE_TIME) {
        expDurationMs = (expRemainingMs < 1) ? 1 : expRemainingMs;
        uint64_t duration_us = (uint64_t)expDurationMs * 1000ULL;
        if (duration_us > NEOPIXEL_LATCH_DELAY_US) {
            duration_us -= NEOPIXEL_LATCH_DELAY_US;
        } else {
            duration_us = 100;
        }
        esp_timer_start_once(expShutoffTimer, duration_us);
        expShutoffArmed = true;
    } else {
        expShutoffArmed = false;
    }

    isPaused = false;
    expState = EXP_EXPOSING;
    return true;
}

// =============================================================================
// STATUS-ABFRAGEN
// =============================================================================
bool ExposureEngine_IsRunning() {
    return (expState != EXP_IDLE && expState != EXP_DONE);
}

bool ExposureEngine_IsPaused() {
    return (expState == EXP_PAUSED);
}

bool ExposureEngine_IsDone() {
    return (expState == EXP_DONE);
}

void ExposureEngine_Acknowledge() {
    if (expState == EXP_DONE) {
        expState = EXP_IDLE;
    }
}

ExposureState ExposureEngine_GetState() {
    return expState;
}

unsigned long ExposureEngine_GetElapsedMs() {
    if (expState == EXP_EXPOSING) {
        return millis() - expPhaseStart;
    }
    return 0;
}

double ExposureEngine_GetCurrentDose() {
    double d = 0.0;
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        d = current_dose;
        xSemaphoreGive(gTimerMutex);
    }
    return d;
}
