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
  // --- STATISCHE VARIABLEN FÜR FLANKENERKENNUNG ---
  static int lastRawSafe = -1;  // -1 signalisiert: Erster Durchlauf (Boot)
  static int lastRawFocus = -1;
  static bool lastSafeState = false;
  static bool lastFocusState = false;
  static bool lastTimerState = false;
  static unsigned long safeDebounceMs = 0;
  static unsigned long focusDebounceMs = 0;
  
  // 1. INPUTS LESEN
  int rawSafe = digitalRead(PIN_SW_SAFE);   // LOW = AN (Input Pullup)
  int rawFocus = digitalRead(PIN_SW_FOCUS); // LOW = AN (Input Pullup)
  
  // A) Safe-Light Schalter Logik
  if (lastRawSafe == -1) {
      // BOOT SYNC: Übernehme sofort den physikalischen Zustand!
      lastRawSafe = rawSafe;
      safeLatch = (rawSafe == LOW); 
  }
  else if (rawSafe != lastRawSafe) {
      // ÄNDERUNG IM BETRIEB: Hardware überschreibt Software (Timestamp-basiertes Entprellen)
      if (safeDebounceMs == 0) { safeDebounceMs = millis(); }
      else if (millis() - safeDebounceMs > 20) {
          if (digitalRead(PIN_SW_SAFE) == rawSafe) {
              safeLatch = (rawSafe == LOW);
              lastRawSafe = rawSafe;
          }
          safeDebounceMs = 0;
      }
  }

  // B) Fokus-Licht Schalter Logik
  if (lastRawFocus == -1) {
      // BOOT SYNC
      lastRawFocus = rawFocus;
      whiteLatch = (rawFocus == LOW);
  }
  else if (rawFocus != lastRawFocus) {
      // ÄNDERUNG IM BETRIEB (Timestamp-basiertes Entprellen)
      if (focusDebounceMs == 0) { focusDebounceMs = millis(); }
      else if (millis() - focusDebounceMs > 20) {
          if (digitalRead(PIN_SW_FOCUS) == rawFocus) {
              whiteLatch = (rawFocus == LOW);
              lastRawFocus = rawFocus;
          }
          focusDebounceMs = 0;
      }
  }

  // C) Finale Status-Variablen
  bool swSafe  = safeLatch; 
  bool swFocus = whiteLatch;
  bool swRoom  = (digitalRead(PIN_SW_ROOMLIGHT) == LOW) || screenOffOverride;
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

// Wrapper für Kompatibilität
void PaintLED(int r, int g, int b) {
  if (!neoPixelOK) return;
  if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pixels.fill(pixels.Color(r, g, b));
    pixels.show();
    xSemaphoreGive(gPixelMutex);
  }
}