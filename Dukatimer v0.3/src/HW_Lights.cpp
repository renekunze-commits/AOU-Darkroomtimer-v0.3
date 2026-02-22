#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "Config.h"
#include "Globals.h"

/*
  HW_Lights.cpp - Lichtsteuerung mit "Last Action Wins" und Boot-Sync
*/

// Hilfsvariablen für UI Status
bool statusEnlargerOn = false;
bool statusSafeOn = false; 

void handleLights() {
  // Mess-State-Machine hat temporär exklusive Lichtkontrolle.
  // Wichtig: Kein konkurrierender Pin-Zugriff aus zwei Zustandsmaschinen.
  if (measurementOverrideActive) return;

  // --- STATISCHE VARIABLEN FÜR FLANKENERKENNUNG ---
  // Continuous Debounce: Separates Tracking von Rohwert-Flanken (prevRaw*)
  // und stabilem entprelltem Zustand (lastRaw*). Timer startet bei JEDEM
  // Flankenwechsel im Rohsignal neu. Erst nach 50ms absoluter Stille
  // wird der neue Zustand übernommen. Kein Snapshot-Fehler mehr möglich.
  static int lastRawSafe = -1;    // Letzter stabiler (entprellter) Zustand (-1 = Boot)
  static int lastRawFocus = -1;
  static int lastRawRoom = -1;
  static int prevRawSafe = HIGH;  // Vorheriger Rohwert (Flankenerkennung)
  static int prevRawFocus = HIGH;
  static int prevRawRoom = HIGH;
  static bool lastSafeState = false;
  static bool lastFocusState = false;
  static bool lastTimerState = false;
  static unsigned long safeDebounceMs = 0;
  static unsigned long focusDebounceMs = 0;
  static unsigned long roomDebounceMs = 0;
  
  // 1. INPUTS LESEN
  int rawSafe = digitalRead(PIN_SW_SAFE);   // LOW = AN (Input Pullup)
  int rawFocus = digitalRead(PIN_SW_FOCUS); // LOW = AN (Input Pullup)
  int rawRoom = digitalRead(PIN_SW_ROOMLIGHT); // LOW = AN (Input Pullup)
  
  // A) Safe-Light Schalter (Continuous Debounce, 50ms)
  if (lastRawSafe == -1) {
      // BOOT SYNC: Übernehme sofort den physikalischen Zustand!
      lastRawSafe = rawSafe;
      prevRawSafe = rawSafe;
      safeLatch = (rawSafe == LOW); 
  }
  else {
      // Bei jedem Flankenwechsel im Rohsignal: Timer neu starten
      if (rawSafe != prevRawSafe) {
          safeDebounceMs = millis();
          prevRawSafe = rawSafe;
      }
      // Erst nach 50ms absoluter Stille neuen Zustand übernehmen
      if (safeDebounceMs != 0 && (millis() - safeDebounceMs >= 50)) {
          if (rawSafe != lastRawSafe) {
              safeLatch = (rawSafe == LOW);
              lastRawSafe = rawSafe;
          }
          safeDebounceMs = 0;
      }
  }

  // B) Fokus-Licht Schalter (Continuous Debounce, 50ms)
  if (lastRawFocus == -1) {
      // BOOT SYNC
      lastRawFocus = rawFocus;
      prevRawFocus = rawFocus;
      whiteLatch = (rawFocus == LOW);
  }
  else {
      if (rawFocus != prevRawFocus) {
          focusDebounceMs = millis();
          prevRawFocus = rawFocus;
      }
      if (focusDebounceMs != 0 && (millis() - focusDebounceMs >= 50)) {
          if (rawFocus != lastRawFocus) {
              whiteLatch = (rawFocus == LOW);
              lastRawFocus = rawFocus;
          }
          focusDebounceMs = 0;
      }
  }

  // C) Raumlicht Schalter (Continuous Debounce, 50ms)
  if (lastRawRoom == -1) {
      // BOOT SYNC
      lastRawRoom = rawRoom;
      prevRawRoom = rawRoom;
      roomLatch = (rawRoom == LOW);
  }
  else {
      if (rawRoom != prevRawRoom) {
          roomDebounceMs = millis();
          prevRawRoom = rawRoom;
      }
      if (roomDebounceMs != 0 && (millis() - roomDebounceMs >= 50)) {
          if (rawRoom != lastRawRoom) {
              roomLatch = (rawRoom == LOW);
              lastRawRoom = rawRoom;
          }
          roomDebounceMs = 0;
      }
  }

  // D) Finale Status-Variablen
  bool swSafe  = safeLatch; 
  bool swFocus = whiteLatch;
  bool swRoom  = roomLatch || screenOffOverride;
  bool timerRunning = (starttime != 0);

  // ------------------------------------------------------------
  // 2. RELAIS STEUERUNG
  // ------------------------------------------------------------
  
  // Raumlicht
    if (!swRoom && !timerRunning && !swFocus && !swSafe && !isMeasuring) {
      digitalWrite(PIN_RELAY_ROOMLIGHT, HIGH); 
  } else {
      digitalWrite(PIN_RELAY_ROOMLIGHT, LOW); 
  }

  // Safe-Light Relais
  digitalWrite(PIN_RELAY_SAFE, swSafe ? HIGH : LOW);

  // Vergrößerer Relais
  digitalWrite(PIN_RELAY_ENLARGER, (timerRunning || swFocus) ? HIGH : LOW);

  // ------------------------------------------------------------
  // 3. MATRIX FARB-MANAGEMENT
  // ------------------------------------------------------------
  
  // FALL 1: TIMER LÄUFT
  if (timerRunning) {
      statusEnlargerOn = true; statusSafeOn = false;
      if (!lastTimerState) { lastTimerState = true; lastSafeState = false; lastFocusState = false; }
  }
  
  // FALL 2: SAFELIGHT
  else if (swSafe) {
      lastTimerState = false; statusEnlargerOn = false; statusSafeOn = true;
      
      // Safe-Priority: Fokus aus, wenn Safe an
      if (whiteLatch) whiteLatch = false; 
      
      if (neoPixelOK && !lastSafeState) {
        uint8_t val = (set_safe > 0) ? set_safe : 30; 
        if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          pixels.fill(pixels.Color(val, 0, 0));
          pixels.show();
          xSemaphoreGive(gPixelMutex);
        }
        lastSafeState = true; lastFocusState = false;
      }
  }
  
  // FALL 3: FOKUS
  else if (swFocus) {
      lastTimerState = false; statusEnlargerOn = true; statusSafeOn = false;
      if (neoPixelOK && !lastFocusState) {
        if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          pixels.fill(pixels.Color(set_focus, set_focus, set_focus));
          pixels.show();
          xSemaphoreGive(gPixelMutex);
        }
        lastFocusState = true; lastSafeState = false;
      }
  }
  
  // FALL 4: ALLES AUS
  else {
      lastTimerState = false; statusEnlargerOn = false; statusSafeOn = false;
      if (neoPixelOK && (lastSafeState || lastFocusState)) {
        if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          pixels.clear(); pixels.show();
          xSemaphoreGive(gPixelMutex);
        }
        lastSafeState = false; lastFocusState = false;
      }
  }
}

// Explizite LED-Modussteuerung für den Flash-Handshake.
// Wichtig: Diese Funktion steuert nur die NeoPixel-Matrixfarbe und blockiert
// maximal kurz auf den Pixel-Mutex. Sie ist damit für den Main-Loop geeignet.
void setLEDMode(uint8_t mode) {
  measurementOverrideActive = true; // Haupt-State-Machine pausieren

  // Alle relevanten Relais in sicheren Grundzustand bringen.
  // Raumlicht aus, Safelight aus, Vergrößerer aus.
  digitalWrite(PIN_RELAY_ROOMLIGHT, LOW);
  digitalWrite(PIN_RELAY_SAFE, LOW);
  digitalWrite(PIN_RELAY_ENLARGER, LOW);

  if (!neoPixelOK) return;
  if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    switch ((LedMode)mode) {
      case LED_GREEN:
        pixels.fill(pixels.Color(0, 255, 0));
        break;
      case LED_BLUE:
        pixels.fill(pixels.Color(0, 0, 255));
        break;
      case LED_FOCUS:
        pixels.fill(pixels.Color(set_focus, set_focus, set_focus));
        break;
      case LED_SAFELIGHT: {
        uint8_t val = (set_safe > 0) ? set_safe : 30;
        pixels.fill(pixels.Color(val, 0, 0));
        break;
      }
      case LED_OFF:
      default:
        pixels.clear();
        break;
    }
    pixels.show();
    xSemaphoreGive(gPixelMutex);
  }
}

// Wrapper für Kompatibilität
void PaintLED(int r, int g, int b) {
  if (!neoPixelOK) return;
  if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pixels.fill(pixels.Color(r, g, b));
    pixels.show();
    xSemaphoreGive(gPixelMutex);
  }
}