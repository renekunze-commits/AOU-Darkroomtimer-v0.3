/* ExposureEngine.h - Universal Exposure Engine (Phase 2 B2 Final)
   
   Zweck: EINZIGE Belichtungs-Engine für ALLE Modi (BW, SG, Burn, TestStrip).
   Ersetzt die redundanten millis()-Timer UND den bisherigen esp_timer/vClosedLoopTask
   aus Logic_Timer.cpp.
   
   Präzision:
     - esp_timer Hardware-Interrupt für <1ms Shutoff-Genauigkeit
     - Predictive Shutoff: Bei 95% Zieldosis/Zeit → verbleibende µs berechnen
     - NeoPixel-Latch-Kompensation: ~7.7ms für 256 LEDs abgezogen
   
   Modi:
     EXPMODE_TIME: Open-Loop (zeitbasiert) — BW-Zeit, Burn, TestStrip
     EXPMODE_DOSE: Closed-Loop (dosisbasiert via TSL2561) — BW-Dosis, SG-Dosis
   
   State-Machine:
     EXP_IDLE → EXP_PRE_WAIT (200ms) → EXP_EXPOSING → EXP_POST_WAIT (400ms) → EXP_DONE
   
   Thread-Sicherheit: gTimerMutex für Dosis/Zeit-Werte, xI2CMutex für Sensor-I2C.
   CL-Task läuft permanent auf Core 1 (Prio 10), liest TSL2561 alle 10ms.
*/

#ifndef EXPOSURE_ENGINE_H
#define EXPOSURE_ENGINE_H

#include <Arduino.h>

enum ExposureState {
    EXP_IDLE,
    EXP_PRE_WAIT,    // 200ms Blackout + Relais-Settle
    EXP_EXPOSING,    // Licht an, esp_timer armed, Metronom aktiv
  EXP_PAUSED,      // Licht aus, Belichtung angehalten
    EXP_POST_WAIT,   // 400ms Nachleuchten-Abklingen
    EXP_DONE         // Fertig, wartet auf Acknowledge
};

enum ExposureMode {
    EXPMODE_TIME,     // Open-Loop: zeitbasiert
    EXPMODE_DOSE      // Closed-Loop: dosisbasiert via TSL2561
};

/** Initialisiert esp_timer und Closed-Loop FreeRTOS-Task. Einmal in setup() aufrufen. */
void ExposureEngine_Init();

/**
 * Startet eine zeitbasierte Belichtung (Open-Loop).
 * @param durationMs   Belichtungsdauer in Millisekunden (min. 100ms)
 * @param green        PWM-Wert für den grünen Kanal (0-255)
 * @param blue         PWM-Wert für den blauen Kanal (0-255)
 * @param skipPreWait  true wenn der Aufrufer Pre-Wait/Blackout bereits erledigt hat
 */
void ExposureEngine_StartTime(unsigned long durationMs, uint8_t green, uint8_t blue,
                              bool skipPreWait = false);

/**
 * Startet eine dosisbasierte Belichtung (Closed-Loop via TSL2561).
 * @param targetDose   Zieldosis in Lux-Sekunden
 * @param green        PWM-Wert für den grünen Kanal (0-255)
 * @param blue         PWM-Wert für den blauen Kanal (0-255)
 * @param skipPreWait  true wenn der Aufrufer Pre-Wait/Blackout bereits erledigt hat
 */
void ExposureEngine_StartDose(double targetDose, uint8_t green, uint8_t blue,
                              bool skipPreWait = false);

/**
 * Zyklisch aufrufen (aus vTaskRealtime).
 * Steuert die State-Machine durch alle Phasen inkl. Metronom.
 */
void ExposureEngine_Tick();

/** Sofort-Abbruch ohne Post-Wait (für Burn/TestStrip Cancel). */
void ExposureEngine_Abort();

/** Graceful Stop: Licht aus, dann Post-Wait Phase (für Haupt-Timer User-Abort). */
void ExposureEngine_Stop();

/** Pausiert eine laufende Belichtung (nur EXP_EXPOSING). */
bool ExposureEngine_Pause();

/** Setzt eine pausierte Belichtung fort (nur EXP_PAUSED). */
bool ExposureEngine_Resume();

/** true wenn Engine im Zustand EXP_PAUSED ist. */
bool ExposureEngine_IsPaused();

/** true wenn Engine nicht IDLE und nicht DONE ist (inkl. PRE_WAIT/POST_WAIT). */
bool ExposureEngine_IsRunning();

/** true wenn Engine im Zustand EXP_DONE ist (Belichtung komplett beendet). */
bool ExposureEngine_IsDone();

/** Setzt den Zustand von EXP_DONE auf EXP_IDLE zurück. */
void ExposureEngine_Acknowledge();

/** Gibt den aktuellen Zustand zurück (für Debug/UI). */
ExposureState ExposureEngine_GetState();

/** Gibt die seit Belichtungsstart (EXPOSING) vergangene Zeit in ms zurück. */
unsigned long ExposureEngine_GetElapsedMs();

/** Gibt die aktuell akkumulierte Dosis zurück (Thread-safe via gTimerMutex). */
double ExposureEngine_GetCurrentDose();

#endif // EXPOSURE_ENGINE_H
