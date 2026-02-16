#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_TSL2591.h>

#include "config.h"
#include "input.h"
#include "output.h"

// --- GLOBALE OBJEKTE ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t seqCounter = 0;
long lastEncVal = 0;

// --- INTERRUPT LOGIK ---
volatile bool tslInterruptOccurred = false;

// Die ISR (Interrupt Service Routine)
void IRAM_ATTR tslISR() {
    tslInterruptOccurred = true;
}

// --- ESP-NOW CALLBACK ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optionales Flag-Handling
}

void setup() {
  Serial.begin(115200);
  
  // 1. Hardware & I2C Init
  Wire.begin(I2C_SDA, I2C_SCL);
  initOutput();
  showStartup();
  initInput();

  // INT-Pin als Eingang mit Pull-Up
  pinMode(PIN_TSL_INT, INPUT_PULLUP);

  // 2. Sensor Init
  if (!tsl.begin()) {
    showError("No Sensor");
    while(1); 
  }
  
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  // 3. Sensor Interrupt konfigurieren
  tsl.clearInterrupt(); 
  
  // KORREKTUR: Korrekter Bibliotheksaufruf für den Interrupt
  // Löst bei jeder fertigen Messung aus (Data Ready)
  tsl.registerInterrupt(0, 0xFFFF, TSL2591_PERSIST_ANY); 
  
  attachInterrupt(digitalPinToInterrupt(PIN_TSL_INT), tslISR, FALLING);

  // 4. ESP-NOW Init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    showError("ESP-NOW Fail");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // 1. Input Lesen (UI reagiert sofort!)
  long currentEnc = getEncoderValue();
  bool btnClick = isButtonPressed();

  if (currentEnc != lastEncVal) {
      clickSound();
      lastEncVal = currentEnc;
  }

  if (btnClick) {
      beep(2000, 100);
  }

  // 2. Messen & Senden (Interrupt-gesteuert ODER erzwungen durch Button)
  if (tslInterruptOccurred || btnClick) {
    tslInterruptOccurred = false;
    tsl.clearInterrupt();         

    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);
    if (lux < 0.0) lux = 0.0;

    WirelessPacket packet;
    packet.magic = REMOTE_MAGIC;
    packet.seq = seqCounter++;
    packet.lux = lux;
    packet.encoderVal = (int)currentEnc;
    
    esp_err_t res = esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    bool txOK = (res == ESP_OK);

    updateDisplay(lux, currentEnc, packet.seq, txOK);
  }
}