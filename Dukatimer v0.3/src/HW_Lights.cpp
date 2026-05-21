/*
  HW_Lights.cpp - Hardware Abstraction Layer (HAL) für Beleuchtung
  v0.5 Root Cause Edition (DMA & NeoPixelBus)
  
  ARCHITEKTUR-REGELN:
  - KEIN Polling von Schaltern (Das macht HW_Input.cpp via ISR).
  - KEINE millis() basierten Delays.
  - REINER Befehlsempfänger (Dumb Executor).
  - NUTZT NeoPixelBus (DMA/RMT) um CPU-Blockaden zu verhindern.
*/

#include <Arduino.h>
#include <NeoPixelBus.h>
#include "Config.h"
#include "Globals.h"
#include "DisplayManager.h"

// =============================================================================
// DMA LED-TREIBER INITIALISIERUNG
// =============================================================================
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixels(NEOPIXEL_COUNT, PIN_NEOPIXEL);

static const TickType_t PIXEL_MUTEX_TIMEOUT = pdMS_TO_TICKS(50);

// =============================================================================
// INTERNE HARDWARE-STATUS (Priority Pipeline)
// =============================================================================
static bool reqFocus = false;
static bool reqSafe = false;
static bool expActive = false;
static uint8_t expR = 0, expG = 0, expB = 0;

/**
 * ZENTRALE STATUS-VERWALTUNG:
 * Synchronisiert die globalen Status-Flags für das LCD-Display.
 * Das LCD darf nur dann Weißlicht-Helligkeit annehmen, wenn der Vergrößerer 
 * auch physikalisch leuchtet (Schutz vor Streulicht bei aktivem Safety Latch).
 */
static void updateHardwareStatusFlags() {
    statusSafeOn = reqSafe;
    statusEnlargerOn = expActive || (reqFocus && !reqSafe); 
}

/**
 * ZENTRALE RENDER-FUNKTION (SAFETY LATCH EDITION):
 * Setzt Hardware-Sperren konsequent durch.
 */
static void renderNeoPixels() {
    if (gPixelMutex != NULL && xSemaphoreTake(gPixelMutex, PIXEL_MUTEX_TIMEOUT) == pdTRUE) {
        if (expActive) {
            // Priorität 1: Timer-Belichtung läuft (System Override)
            pixels.ClearTo(RgbColor(expR, expG, expB));
        } else if (reqSafe) {
            // Priorität 2: SAFETY LATCH! Safelight hat absolute Priorität.
            // Blockiert das Fokuslicht zwingend, um offenes Fotopapier zu schützen.
            uint8_t s = (set_safe > 0) ? set_safe : 30;
            pixels.ClearTo(RgbColor(s, 0, 0));
        } else if (reqFocus) {
            // Priorität 3: Fokus-Einrichtlicht (Weiß). Geht nur an, wenn Safe AUS ist.
            uint8_t f = (set_focus > 0) ? set_focus : 255;
            pixels.ClearTo(RgbColor(f, f, f));
        } else {
            // Priorität 4: Alles aus
            pixels.ClearTo(RgbColor(0, 0, 0));
        }
        pixels.Show(); 
        xSemaphoreGive(gPixelMutex);
    }
}

void HW_InitLights() {
    // Hardware-Grounding gegen schwebende "Geister-Signale"
    pinMode(PIN_SW_FOCUS, INPUT_PULLUP);
    pinMode(PIN_SW_SAFE, INPUT_PULLUP);
    pinMode(PIN_SW_ROOMLIGHT, INPUT_PULLUP);

    pixels.Begin();
    renderNeoPixels(); // Setzt initial alle LEDs sauber auf Schwarz
}

// =============================================================================
// 1. BLACKOUT CONTROL (Raumlicht & Displays)
// =============================================================================
void HW_SetBlackout(bool active) {
    isRoomDarknessActive = active;
    
    // Relais schalten (LOW zieht Relais an = Dunkelheit)
    digitalWrite(PIN_RELAY_ROOMLIGHT, active ? LOW : HIGH);

    // Displays absolut synchron dimmen (Verhindert Streulicht)
    if (active) {
        DM_setDimming(0);
    } else {
        uint8_t current_set_lcd = 100;
        if (gTimerMutex != NULL && xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_set_lcd = set_lcd;
            xSemaphoreGive(gTimerMutex);
        } else {
            current_set_lcd = set_lcd; 
        }
        uint8_t nexDim = map(current_set_lcd, 0, 255, 10, 100); 
        DM_setDimming(nexDim);
    }
}

// =============================================================================
// 2. SAFELIGHT CONTROL (Rotlicht)
// =============================================================================
void HW_SetSafelight(bool active) {
    reqSafe = active;
    updateHardwareStatusFlags();
    renderNeoPixels();
}

// =============================================================================
// 3. FOCUS CONTROL (Einrichtlicht)
// =============================================================================
void HW_SetFocus(bool active) {
    reqFocus = active;
    updateHardwareStatusFlags();
    renderNeoPixels();
}

// =============================================================================
// 4. EXPOSURE CONTROL (Belichtung)
// =============================================================================
void HW_SetEnlargerNeoPixel(uint8_t r, uint8_t g, uint8_t b) {
    expR = r; expG = g; expB = b;
    expActive = (r > 0 || g > 0 || b > 0);
    updateHardwareStatusFlags();
    renderNeoPixels();
}

void HW_EmergencyShutoff() {
    expR = 0;
    expG = 0;
    expB = 0;
    expActive = false;
    updateHardwareStatusFlags();

    if (gPixelMutex != NULL && xSemaphoreTake(gPixelMutex, 0) == pdTRUE) {
        pixels.ClearTo(RgbColor(0, 0, 0));
        pixels.Show();
        xSemaphoreGive(gPixelMutex);
        return;
    }

    pixels.ClearTo(RgbColor(0, 0, 0));
    pixels.Show();
}

// =============================================================================
// 5. ZYKLISCHES LICHT-UPDATE & SCHALTER-POLLING
// =============================================================================

// Legacy-Wrapper (Für Abwärtskompatibilität)
void PaintLED(int r, int g, int b) { 
    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
    HW_SetEnlargerNeoPixel((uint8_t)r, (uint8_t)g, (uint8_t)b); 
}

// Physische Schalter einlesen und mit UI kombinieren
void handleLights() {
    // FIX A1: Light-Lock - block all light changes during exposure/measurement
    if (lightOperationActive || isMeasuring) {
        return;
    }

    static bool wasBusy = false;

    // 1. CPU entlasten und Hardware-Schalter blockieren, 
    // wenn der Timer oder das Densitometer gerade das Licht kontrollieren!
    if (starttime != 0 || isMeasuring) {
        wasBusy = true; 
        return; 
    }

    // 2. Physische Hardware-Schalter einlesen (Low = Aktiviert)
    // BETA-FIX: Schalter-Zuverlaessigkeit
    // Die drei Hardware-Schalter werden lokal entprellt, damit kurze
    // Kontaktpreller/EMV-Spitzen nicht als gueltiger ON-Zustand haengen bleiben.
    // Dadurch wird insbesondere verhindert, dass Safelight scheinbar "nur noch an"
    // geht, aber nicht sauber wieder aus.
    static bool focusRawLast = false, safeRawLast = false, roomRawLast = false;
    static bool focusSwitch = false, safeSwitch = false, roomSwitch = false;
    static unsigned long focusRawChangedAt = 0, safeRawChangedAt = 0, roomRawChangedAt = 0;
    const unsigned long now = millis();
    const unsigned long SWITCH_DEBOUNCE_MS = 30;

    bool focusRaw = (digitalRead(PIN_SW_FOCUS) == LOW);
    bool safeRaw  = (digitalRead(PIN_SW_SAFE) == LOW);
    bool roomRaw  = (digitalRead(PIN_SW_ROOMLIGHT) == LOW);

    if (focusRaw != focusRawLast) { focusRawLast = focusRaw; focusRawChangedAt = now; }
    if (safeRaw  != safeRawLast)  { safeRawLast  = safeRaw;  safeRawChangedAt  = now; }
    if (roomRaw  != roomRawLast)  { roomRawLast  = roomRaw;  roomRawChangedAt  = now; }

    if ((now - focusRawChangedAt) >= SWITCH_DEBOUNCE_MS) focusSwitch = focusRaw;
    if ((now - safeRawChangedAt)  >= SWITCH_DEBOUNCE_MS) safeSwitch  = safeRaw;
    if ((now - roomRawChangedAt)  >= SWITCH_DEBOUNCE_MS) roomSwitch  = roomRaw;

    // BETA-FIX: Physischer OFF hat Vorrang vor Software-Latches.
    // Wenn der Nutzer einen der realen Schalter auf AUS setzt, wird der zugehoerige
    // UI-Latch geloescht. So kann ein historischer Nextion-Latch den Hardware-Schalter
    // nicht mehr "ueberstimmen" und die Leuchte ungewollt anlassen.
    if (!safeSwitch)  safeLatch = false;
    if (!focusSwitch) whiteLatch = false;
    if (!roomSwitch)  roomLatch = false;

    // 3. Mit den Software-Zuständen (Nextion / UI Latch) kombinieren
    bool targetSafe  = safeSwitch || safeLatch;
    bool targetRoom  = roomSwitch || roomLatch;

    // --- ROOT CAUSE FIX: Die Safety Latch Verriegelung ---
    bool rawFocus = focusSwitch || whiteLatch;
    static bool lastRawFocus = false;
    static bool focusTrapLock = false;

    // Wenn Safelight aktiv ist und Fokus NEU eingeschaltet wird, schnappt die Falle zu!
    if (rawFocus && !lastRawFocus && targetSafe) {
        focusTrapLock = true;
        whiteLatch = false; // UI-Software-Button direkt wieder abwerfen!
    }

    // Die Falle löst sich erst wieder, wenn der Fokus-Schalter physisch auf AUS steht.
    if (!rawFocus) {
        focusTrapLock = false;
    }
    
    lastRawFocus = rawFocus;

    // Der finale, gefilterte Fokus-Befehl (Verriegelung blockiert das Signal hart)
    bool targetFocus = rawFocus && !focusTrapLock;

    // 4. Flankenerkennung: Nur an die Hardware senden, wenn sich etwas ändert!
    static bool lastFocus = !targetFocus;
    static bool lastSafe  = !targetSafe;
    static bool lastRoom  = !targetRoom;

    // Wenn der Timer fertig ist, zwingen wir das System, sich exakt 
    // auf die aktuellen physikalischen Schalter-Positionen zu synchronisieren!
    bool forceSync = wasBusy;

    if (forceSync || targetFocus != lastFocus) {
        HW_SetFocus(targetFocus);
        lastFocus = targetFocus;
    }

    if (forceSync || targetSafe != lastSafe) {
        HW_SetSafelight(targetSafe);
        lastSafe = targetSafe;
    }

    if (forceSync || targetRoom != lastRoom) {
        HW_SetBlackout(targetRoom);
        lastRoom = targetRoom;
    }

    wasBusy = false;
}
