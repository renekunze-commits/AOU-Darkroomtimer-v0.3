/* Logic_Exposure.cpp - Closed-Loop Belichtung

   Verantwortlich fuer:
   - Zeitkritische Lichtmengen-Regelung mit TSL2561 (Head-Sensor)
   - Interleaved Timing: Messen -> Rechnen -> NeoPixel-Update
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TSL2561_U.h>
#include "Config.h"
#include "Globals.h"
#include "Types.h"
#include <esp_task_wdt.h>

// =============================================================================
// CLOSED-LOOP BELICHTUNG (Zeitkritische Lichtregelung mit TSL2561)
// =============================================================================

/**
 * exposureClosedLoop - Belichtung mit Lichtmengen-Regelung
 * 
 * Diese Funktion fuehrt eine Belichtung durch, bei der die tatsaechlich abgegebene
 * Lichtmenge (Lux*Sekunden) kontinuierlich gemessen und geregelt wird.
 * 
 * Timing-Strategie (Interleaved):
 * 1. Phase A: TSL2561 auslesen (Bus 1, ~13ms)
 * 2. Phase B: Helligkeit berechnen (falls Anpassung noetig)
 * 3. Phase C: NeoPixel.show() - INTERRUPTS AUS! (~8ms)
 * 4. Wiederholen bis Ziel-Lichtmenge erreicht
 * 
 * @param targetLuxSeconds Ziel-Lichtmenge in Lux*Sekunden (integriert)
 * @param baseR, baseG, baseB Basis-Farbe der Matrix (0-255)
 * @param maxBrightness Maximale Helligkeit (0-255), Standard: 255
 * @param timeoutMs Maximale Belichtungsdauer in ms (Sicherheit), Standard: 300000 (5min)
 * @return true wenn Ziel-Lichtmenge erreicht, false bei Timeout oder Abbruch
 */
bool exposureClosedLoop(double targetLuxSeconds, uint8_t baseR, uint8_t baseG, uint8_t baseB, 
                        uint8_t maxBrightness = 255, unsigned long timeoutMs = 300000, double tempComp = 1.0) {
  
  // Sicherheitschecks
  if (!neoPixelOK || !tslLiveOK) {
    Serial.println("ERROR: NeoPixel oder TSL2561 nicht verfuegbar!");
    return false;
  }
  
  if (targetLuxSeconds <= 0.0) {
    Serial.println("ERROR: targetLuxSeconds muss > 0 sein!");
    return false;
  }
  
  // Initialisierung
  double accumulatedLux = 0.0;  // Akkumulierte Lichtmenge
  uint8_t currentBrightness = maxBrightness;  // Start mit voller Helligkeit
  unsigned long startMs = millis();
  unsigned long lastMeasureMs = startMs;
  
  // Raumlicht aus (Sicherheit)
  digitalWrite(PIN_RELAY_ROOMLIGHT, LOW);
  
  // Initiale Matrix-Farbe setzen
  if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pixels.setBrightness(currentBrightness);
    pixels.fill(pixels.Color(baseR, baseG, baseB));
    pixels.show();  // Interrupts AUS fuer ~8ms
    xSemaphoreGive(gPixelMutex);
  }
  
  Serial.print("Closed-Loop Start: Ziel=");
  Serial.print(targetLuxSeconds);
  Serial.println(" lux*s");
  
  // Hauptschleife
  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startMs;
    
    // === PHASE A: MESSEN (TSL2561 auf Bus 1) ===
    // INT-Synchronisation: Nur messen, wenn INT-Pin HIGH (Sensor bereit)
    if (digitalRead(PIN_TSL2561_INT) == HIGH) {
      sensors_event_t event;
      tslLive.getEvent(&event);
    
      if (event.light > 0) {
        // Zeitdelta seit letzter Messung (in Sekunden)
        double deltaTimeS = (now - lastMeasureMs) / 1000.0;
        // Temperatur-Kompensation
        double tempFactor = tempComp;
        // Lichtmenge akkumulieren: Lux * Zeit = Lux*Sekunden
        double deltaLux = event.light * deltaTimeS * tempFactor;
        accumulatedLux += deltaLux;
        lastMeasureMs = now;
      
      #ifdef DEBUG_ENABLED
      if ((int)(elapsed / 100) % 10 == 0) {  // Alle 1 Sekunde loggen
        Serial.print("t=");
        Serial.print(elapsed / 1000.0, 2);
        Serial.print("s, Lux=");
        Serial.print(event.light, 1);
        Serial.print(", Acc=");
        Serial.print(accumulatedLux, 2);
        Serial.print("/");
        Serial.print(targetLuxSeconds, 2);
        Serial.print(", Bright=");
        Serial.println(currentBrightness);
      }
      #endif
      
        // Ziel erreicht?
        if (accumulatedLux >= targetLuxSeconds) {
          if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            pixels.clear();
            pixels.show();
            xSemaphoreGive(gPixelMutex);
          }
          Serial.print("Closed-Loop FERTIG: ");
          Serial.print(accumulatedLux, 2);
          Serial.print(" lux*s in ");
          Serial.print(elapsed / 1000.0, 2);
          Serial.println("s");
          beepDone();
          return true;
        }
      } else {
        Serial.println("WARNING: TSL2561 Messfehler (event.light = 0)");
      }
    } else {
      // Sensor-Fehler oder Ueberbelichtung
      Serial.println("WARNING: TSL2561 Messfehler (event.light = 0)");
    }
    
    // === PHASE B: REGELUNG (Optional: Helligkeit anpassen) ===
    // Einfache proportionale Regelung:
    // Wenn wir noch weit vom Ziel entfernt sind, volle Helligkeit
    // Wenn wir nahe dran sind, reduzieren (sanftes Ende)
    
    double progress = accumulatedLux / targetLuxSeconds;
    
    if (progress > 0.9) {
      // Letzte 10%: Helligkeit reduzieren fuer sanften Abschluss
      currentBrightness = map(progress * 100, 90, 100, maxBrightness, maxBrightness / 4);
      currentBrightness = constrain(currentBrightness, 10, maxBrightness);
    } else {
      // Volle Kraft voraus
      currentBrightness = maxBrightness;
    }
    
    // === PHASE C: LED UPDATE (INTERRUPTS AUS!) ===
    if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      pixels.setBrightness(currentBrightness);
      pixels.fill(pixels.Color(baseR, baseG, baseB));
      pixels.show();  // KRITISCH: ~8ms ohne Interrupts
      xSemaphoreGive(gPixelMutex);
    }
    
    // Timeout-Check
    if (elapsed > timeoutMs) {
      if (gPixelMutex && xSemaphoreTake(gPixelMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        pixels.clear();
        pixels.show();
        xSemaphoreGive(gPixelMutex);
      }
      
      Serial.print("Closed-Loop TIMEOUT nach ");
      Serial.print(elapsed / 1000.0, 1);
      Serial.print("s (Erreicht: ");
      Serial.print(accumulatedLux, 2);
      Serial.print("/");
      Serial.print(targetLuxSeconds, 2);
      Serial.println(" lux*s)");
      
      return false;
    }
    
    // Kurze Pause zwischen Zyklen (optimal: ~15-20ms total pro Loop)
    // 13ms TSL2561 + 8ms pixels.show() + 2ms Code = ~23ms/Zyklus
    delay(2);
    
    // Watchdog fuettern
    esp_task_wdt_reset();
  }
}
