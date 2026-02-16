/* ============================================================================
   ESP32-C3/S3 WIRELESS PROBE - TSL2591 Sensor Sender
   ============================================================================
   
   Zweck:
   - Liest TSL2591 Sensor über I2C aus
   - Sendet Lux-Werte via ESP-NOW Broadcast an Dukatimer-Hauptgerät
   - Optimiert für batteriebetriebene Verwendung
   
   Hardware:
   - ESP32-C3 SuperMini (oder ESP32-S3 Mini)
   - TSL2591 Luxsensor (I2C)
   - LiPo Batterie + Lade-Controller (optional)
   
   Verkabelung (ESP32-C3 SuperMini Standard):
   - SDA: GPIO 8
   - SCL: GPIO 9
   - VCC: 3.3V
   - GND: GND
   
   Hinweise:
   - Broadcast-Adresse FF:FF:FF:FF:FF:FF erreicht alle Geräte in Reichweite
   - Keine Pairing-Notwendig, sofort einsatzbereit
   - Senderate: 5x pro Sekunde (200ms Intervall)
   
   ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_TSL2591.h>

// =============================================================================
// KONFIGURATION
// =============================================================================
#define REMOTE_MAGIC 0xD4        // Kennung für Dukatimer Pakete (muss mit Empfänger übereinstimmen!)
#define SEND_INTERVAL_MS 200     // 5x pro Sekunde senden

// I2C Pins (ESP32-C3 SuperMini Standard)
#define I2C_SDA 8
#define I2C_SCL 9

// Optional: LED für Status-Anzeige (falls vorhanden)
// #define LED_BUILTIN 2

// =============================================================================
// DATENSTRUKTUR (Muss identisch zum Empfänger sein!)
// =============================================================================
struct WirelessPacket {
    uint8_t magic;
    uint32_t seq;
    float lux;
};

// =============================================================================
// GLOBALE VARIABLEN
// =============================================================================
// Broadcast Adresse (an alle Geräte)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Sensor-Instanz
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// Paketzähler
uint32_t seqCounter = 0;

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=================================");
  Serial.println("DUKATIMER WIRELESS PROBE");
  Serial.println("ESP-NOW TSL2591 Sender");
  Serial.println("=================================\n");
  
  // Optional: LED initialisieren
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);
  
  // --- I2C INITIALISIERUNG ---
  Serial.printf("Starting I2C (SDA=%d, SCL=%d)...\n", I2C_SDA, I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL); 
  
  // --- TSL2591 SENSOR ---
  Serial.println("Initializing TSL2591...");
  if (!tsl.begin()) {
    Serial.println("ERROR: TSL2591 not found!");
    Serial.println("Check wiring:");
    Serial.printf("  SDA -> GPIO %d\n", I2C_SDA);
    Serial.printf("  SCL -> GPIO %d\n", I2C_SCL);
    Serial.println("  VCC -> 3.3V");
    Serial.println("  GND -> GND");
    while(1) {
      // Blinke LED falls vorhanden
      // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }
  Serial.println("TSL2591 OK");
  
  // Sensor-Einstellungen (anpassen je nach Anwendung)
  tsl.setGain(TSL2591_GAIN_MED);                      // Mittlere Verstärkung
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);       // 100ms Integration
  
  Serial.printf("Gain: MED, Integration: 100ms\n");

  // --- WIFI INITIALISIERUNG ---
  Serial.println("Starting WiFi Station Mode...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Keine Verbindung zu AP aufbauen
  
  // MAC Adresse ausgeben (für Debugging)
  Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());

  // --- ESP-NOW INITIALISIERUNG ---
  Serial.println("Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW Init Failed!");
    ESP.restart();
  }
  Serial.println("ESP-NOW OK");

  // Broadcast-Peer registrieren
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;   // 0 = auto (alle Kanäle scannen)
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("ERROR: Failed to add broadcast peer");
    ESP.restart();
  }
  
  Serial.println("\n=================================");
  Serial.println("READY TO TRANSMIT");
  Serial.println("=================================\n");
  
  // Optional: LED einschalten als "Ready"-Signal
  // digitalWrite(LED_BUILTIN, HIGH);
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  // --- 1. MESSUNG DURCHFÜHREN ---
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  float lux = tsl.calculateLux(full, ir);
  
  // Niedriger Lux-Wert Korrektur (verhindert negative Werte)
  if (lux < 0.0001) lux = 0.0001;

  // --- 2. PAKET SCHNÜREN ---
  WirelessPacket packet;
  packet.magic = REMOTE_MAGIC;
  packet.seq = seqCounter++;
  packet.lux = lux;

  // --- 3. SENDEN ---
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));

  // --- 4. FEEDBACK ---
  if (result == ESP_OK) {
    // Erfolg
    Serial.printf("[%06lu] TX OK  | Lux: %8.2f | IR: %5u | Full: %5u\n", 
                  packet.seq, lux, ir, full);
    
    // Optional: LED kurz blinken bei erfolgreicher Übertragung
    // digitalWrite(LED_BUILTIN, HIGH);
    // delayMicroseconds(10);
    // digitalWrite(LED_BUILTIN, LOW);
  } else {
    // Fehler
    Serial.printf("[%06lu] TX FAIL | Error: %d\n", packet.seq, result);
  }

  // --- 5. WARTEN ---
  delay(SEND_INTERVAL_MS);
}

// =============================================================================
// ZUSÄTZLICHE FUNKTIONEN FÜR BATTERIEOPTIMIERUNG (Optional)
// =============================================================================

/* 
 * Deep Sleep Modus (für batteriebetriebene Anwendung):
 * 
 * Wenn du Batterie sparen willst, könntest du zwischen Messungen in Deep Sleep gehen:
 * 
 * void goToSleep(uint32_t seconds) {
 *   Serial.printf("Going to sleep for %d seconds...\n", seconds);
 *   esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
 *   esp_deep_sleep_start();
 * }
 * 
 * Am Ende von loop() dann:
 * goToSleep(1);  // 1 Sekunde schlafen statt delay(200)
 * 
 * ACHTUNG: Deep Sleep leert SRAM, ESP startet immer von setup() neu!
 * Für Dukatimer-Anwendung eher nicht geeignet, da kontinuierliche Messung benötigt wird.
 */

/* 
 * Batteriespannungs-Überwachung (wenn ADC-Pin angeschlossen):
 * 
 * float readBatteryVoltage() {
 *   // Beispiel: Spannungsteiler an GPIO 0 (ADC1_CH0)
 *   // R1=100k (zu Batterie+), R2=100k (zu GND)
 *   int raw = analogRead(0);
 *   float voltage = (raw / 4095.0) * 3.3 * 2.0;  // *2 für Spannungsteiler
 *   return voltage;
 * }
 * 
 * Optional im Paket übertragen:
 * packet.batteryVolts = readBatteryVoltage();
 */
