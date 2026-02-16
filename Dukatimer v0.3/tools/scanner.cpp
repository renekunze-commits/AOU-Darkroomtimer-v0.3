/* Dukatimer v0.3 - I2C Scanner mit LCD-Ausgabe
   Scannt Bus 0 (Slow) und Bus 1 (Fast) und zeigt Ergebnisse auf dem LCD.
*/

#include <Arduino.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "config.h" //

rgb_lcd diagLcd;
TwoWire I2C_F = TwoWire(1);

void setup() {
  Serial.begin(115200);
  
  // Initialisiere Busse
  Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C0_FREQ);
  I2C_F.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_FREQ);

  // LCD Start
  diagLcd.begin(16, 2);
  diagLcd.setRGB(255, 255, 255);
  diagLcd.print("I2C Scanner");
  diagLcd.setCursor(0, 1);
  diagLcd.print("Dukatimer v0.3");
  delay(2000);
}

void scanAndDisplay(TwoWire &bus, String busName, int busNum) {
  int nDevices = 0;
  diagLcd.clear();
  diagLcd.setRGB(255, 255, 255);
  diagLcd.print("Scan " + busName);
  Serial.println("Scanning " + busName + "...");

  for (byte address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    byte error = bus.endTransmission();

    if (error == 0) {
      nDevices++;
      String addrHex = String(address, HEX);
      addrHex.toUpperCase();
      
      // Identifizierung
      String devName = "Unknown";
      if (busNum == 0) {
        if (address == 0x27 || address == 0x3E) devName = "LCD";
        else if (address == 0x29) devName = "TSL2591";
        else if (address == 0x76 || address == 0x77) devName = "BMP280";
      } else {
        if (address == 0x39 || address == 0x49) devName = "TSL2561";
      }

      // LCD Anzeige für jedes Gerät
      diagLcd.setCursor(0, 1);
      diagLcd.print("0x" + addrHex + ": " + devName + "  ");
      Serial.println("Found 0x" + addrHex + " (" + devName + ")");
      delay(1500); // Zeit zum Lesen geben
    }
  }

  diagLcd.setCursor(0, 1);
  if (nDevices == 0) {
    diagLcd.setRGB(255, 0, 0);
    diagLcd.print("No Devices!   ");
  } else {
    diagLcd.print("Done: " + String(nDevices) + " Found");
  }
  delay(3000);
}

void loop() {
  // Endloser Scan-Durchlauf
  scanAndDisplay(Wire, "Bus 0 (Slow)", 0);
  scanAndDisplay(I2C_F, "Bus 1 (Fast)", 1);
}
