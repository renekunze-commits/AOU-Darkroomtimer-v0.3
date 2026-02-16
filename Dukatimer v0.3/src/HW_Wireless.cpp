/* HW_Wireless.cpp - ESP-NOW Empfänger für drahtlosen TSL2591 Sensor
   
   Zweck:
   - Empfängt Lux-Messwerte von einem externen ESP32-C3/S3 Sensor via ESP-NOW
   - Callback-basiert, läuft asynchron im Hintergrund
   - Aktualisiert globale Variable remoteLux für readSpot() in HW_Sensors.cpp
   
   Hinweise:
   - ESP-NOW benötigt WiFi Station Mode (aber keine Verbindung zu AP)
   - Die Callback-Funktion läuft im Interrupt-Kontext -> schnell halten!
   - Magic Byte (0xD4) verhindert Empfang von falschen Paketen
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "Globals.h"
#include "Types.h"

// =============================================================================
// CALLBACK: Wird ausgeführt, wenn Daten ankommen (Interrupt-Kontext!)
// =============================================================================
// IRAM_ATTR: Diese Funktion wird vom WiFi-Treiber im Interrupt-ähnlichen
// Kontext aufgerufen. Durch Platzierung im internen SRAM wird verhindert,
// dass ein Flash-Cache-Miss den WiFi-Stack blockiert. Besonders wichtig,
// da ESP-NOW-Pakete auch während einer laufenden Belichtung eintreffen
// können und die remoteLux-Variable zeitnah aktualisiert werden muss.
// ---------------------------------------------------------------------------
IRAM_ATTR void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Prüfe Paketgröße
  if (len != sizeof(WirelessPacket)) return; 

  WirelessPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  // Prüfe Magic Byte (Dukatimer-Kennung)
  if (packet.magic == REMOTE_MAGIC) {
      remoteLux = (double)packet.lux;
      lastRemotePacketMs = millis();
      
      // Optional: Debug-Ausgabe (nur für Tests, auskommentieren im Produktivbetrieb)
      // Serial.printf("[WIRELESS] Seq=%lu, Lux=%.2f\n", packet.seq, packet.lux);
  }
}

// =============================================================================
// INITIALISIERUNG: ESP-NOW starten
// =============================================================================
void initWireless() {
  // ESP-NOW benötigt WiFi Station Mode (ohne AP-Verbindung)
  WiFi.mode(WIFI_STA);
  
  // Optional: WiFi-Kanal festlegen (Sender muss auf demselben Kanal sein!)
  // Für Broadcast-Betrieb meist nicht nötig, da ESP-NOW alle Kanäle scannt
  // Wenn du Performance brauchst, kannst du dies aktivieren:
  /*
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  */

  // ESP-NOW initialisieren
  if (esp_now_init() != ESP_OK) {
    Serial.println("[WIRELESS] Error initializing ESP-NOW");
    return;
  }
  
  // Empfangs-Callback registrieren
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  Serial.println("[WIRELESS] ESP-NOW Ready. Listening for remote probe...");
}
