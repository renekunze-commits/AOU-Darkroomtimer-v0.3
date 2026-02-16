/* HW_Sensors.ino - Sensorzugriff und Glättung

   - Liest den TSL2591 (Luxen) aus und führt eine einfache Glättung (Gleitender Durchschnitt)
   - Implementiert Auto-Gain (passt `currentGainIdx` an und verwirft einige Frames)
   - `readSpot()` gibt eine strukturierte Messung (SpotMeas) zurück; `takeAveragedLux`
     führt mehrere Messungen seriell durch (blocking, aber nützlich für Kalibrierungsabläufe)
*/
#include <Arduino.h>
#include "Globals.h"
#include "Config.h"

// Buffer für Glättung
#define FILTER_SIZE 10
static double luxBuffer[FILTER_SIZE];
static int luxBufferIdx = 0;
static int luxBufferCount = 0;
static double lastValidLux = NAN;
static uint8_t sensorSkipFrames = 0; 




SpotMeas readSpot() {
  SpotMeas m = { NAN, 0, 0, false };
  
  // =============================================================================
  // WEICHE: WIRELESS ODER KABEL
  // =============================================================================
  if (useWirelessProbe) {
      // Modus: Drahtloser Sensor
      // Prüfe ob Daten frisch sind (jünger als 2 Sekunden)
      if (millis() - lastRemotePacketMs < 2000) {
          m.lux = remoteLux;
          m.ok = true;
          // Wireless sendet keine Raw Channels, daher 0
          m.ch0 = 0; 
          m.ch1 = 0;
      } else {
          // Timeout! Sensor hat Verbindung verloren
          m.lux = 0.0;
          m.ok = false; 
          // Tipp: Hier könnte man später im UI "NO SIGNAL" anzeigen
      }
      return m; // Hier raus, wir ignorieren den lokalen Sensor
  }
  
  // =============================================================================
  // FALLBACK: LOKALER SENSOR (TSL2591 über Kabel)
  // =============================================================================
  // Sofort zurück wenn Sensor nicht verfügbar
  if (!tslBaseOK) {
    m.lux = 0.0;
    return m;
  }

  uint32_t lum = tslBase.getFullLuminosity();
  uint16_t rawFull = lum & 0xFFFF; 
  bool gainChanged = false;

  // Auto-Gain Logik
  if (rawFull > 60000 && currentGainIdx > 0) {
    currentGainIdx--; gainChanged = true;
  }
  else if (rawFull < 500 && currentGainIdx < 2) {
    currentGainIdx++; gainChanged = true;
  }

  if (gainChanged) {
    tsl2591Gain_t g = (currentGainIdx == 0) ? TSL2591_GAIN_LOW : 
                      (currentGainIdx == 1) ? TSL2591_GAIN_MED : TSL2591_GAIN_HIGH;
    tslBase.setGain(g);
    tslBase.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    sensorSkipFrames = 3; // Ein paar Frames verwerfen nach Gain-Change
    if (!isnan(lastValidLux)) { m.lux = lastValidLux; m.ok = true; }
    return m;
  }

  if (sensorSkipFrames > 0) {
    sensorSkipFrames--;
    if (!isnan(lastValidLux)) { m.lux = lastValidLux; m.ok = true; }
    return m;
  }

  uint16_t ch0 = rawFull;
  uint16_t ch1 = lum >> 16; 
  double curLux = tslBase.calculateLux(ch0, ch1);
  if (curLux <= 0.0001) curLux = 0.0001;

  // Gleitender Durchschnitt
  luxBuffer[luxBufferIdx] = curLux;
  luxBufferIdx = (luxBufferIdx + 1) % FILTER_SIZE;
  if (luxBufferCount < FILTER_SIZE) luxBufferCount++;

  double sum = 0;
  for (int i = 0; i < luxBufferCount; i++) sum += luxBuffer[i];
  
  m.lux = sum / (double)luxBufferCount;
  m.ch0 = ch0; m.ch1 = ch1; m.ok = true;
  lastValidLux = m.lux;
  return m;
}

double takeAveragedLux(uint8_t samples, uint16_t delayMs) {
  double acc = 0; int n = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    wdt_reset(); 
    SpotMeas sm = readSpot();
    if (sm.ok) { acc += sm.lux; n++; }
    delay(delayMs);
  }
  if (n == 0) return NAN;
  return acc / (double)n;
}