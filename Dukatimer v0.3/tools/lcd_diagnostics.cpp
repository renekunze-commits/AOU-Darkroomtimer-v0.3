// LCD Diagnose-Sketch für I2C LCD
// Nutze diesen Code zur Fehlersuche bei LCD-Problemen

#include <Arduino.h>
#include <Wire.h>
#include "Globals.h"

// I2C Pins für ESP32-S3
#define PIN_I2C_SDA     8
#define PIN_I2C_SCL     9

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== LCD DIAGNOSTICS ===\n");
  
  // 1. I2C Scanner - alle Adressen testen
  Serial.println("1. I2C ADRESS SCAN:");
  I2C_SLOW.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(100);
  
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    I2C_SLOW.beginTransmission(address);
    error = I2C_SLOW.endTransmission();
    
    if (error == 0) {
      Serial.print("   FOUND: 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0)
    Serial.println("   ERROR: Kein I2C Gerät gefunden!");
  else
    Serial.println("   Gefundene Geräte: " + String(nDevices));
  
  // 2. LCD Initialisierungsversuche
  Serial.println("\n2. LCD INITIALIZATION TESTS:\n");
  
  // Versuch 1: 0x27 (Standard)
  testLCD(0x27, 20, 4, "0x27 (20x4)");
  
  // Versuch 2: 0x3F (Alternative)
  testLCD(0x3F, 20, 4, "0x3F (20x4)");
  
  // Versuch 3: 16x2
  testLCD(0x27, 16, 2, "0x27 (16x2)");
  
  Serial.println("\n=== END DIAGNOSTICS ===\n");
}

void testLCD(byte addr, int cols, int rows, String description) {
  Serial.print("   Testing ");
  Serial.print(description);
  Serial.print("... ");
  
  LiquidCrystal_I2C testLcd(addr, cols, rows);
  
  I2C_SLOW.beginTransmission(addr);
  if (I2C_SLOW.endTransmission() == 0) {
    Serial.println("FOUND!");
    testLcd.init();
    testLcd.backlight();
    testLcd.setCursor(0, 0);
    testLcd.print("LCD OK!");
    testLcd.setCursor(0, 1);
    testLcd.print(description);
    delay(1000);
    testLcd.noBacklight();
  } else {
    Serial.println("NOT FOUND");
  }
}

void loop() {
  // Nichts
}
