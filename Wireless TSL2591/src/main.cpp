/* Wireless TSL2591 Probe - ESP32-C6 "Dumb Terminal"
   
   Architektur:
   Das Handgerät hält keinen eigenen Status, berechnet keine Mathematik und
   kennt keine Papier-Profile. Es liest Taster/Encoder aus, sendet Events
   (Paket Typ A, 14 Bytes) an den ESP32-S3 und zeichnet auf dem LCD exakt
   das, was der S3 im Antwort-Paket (Typ B, 63 Bytes) befiehlt.

   Kommunikationsablauf:
   1. C6 sendet ProbeEventPacket (Button/Encoder-Events + Heartbeat)
   2. S3 verarbeitet Event in seiner State-Machine
   3. S3 sendet ProbeRenderPacket zurück (Display-Inhalt + Haptic)
   4. C6 rendert das Paket blind auf dem LCD

   Flash-Handshake (Spektralmessung):
   1. C6 sendet EVT_T2_CLICK
   2. S3 schaltet G0 (Grün) auf 100%, wartet, sendet CMD_MEASURE_G0
   3. C6 misst TSL2591, sendet EVT_LUX_DATA mit lux_raw_g0
   4. S3 schaltet G5 (Blau) auf 100%, wartet, sendet CMD_MEASURE_G5
   5. C6 misst TSL2591, sendet EVT_LUX_DATA mit lux_raw_g5
   6. S3 berechnet Dosis, sendet finales ProbeRenderPacket
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_TSL2591.h>

#include "config.h"
#include "input.h"
#include "output.h"

// =============================================================================
// GLOBALE OBJEKTE
// =============================================================================
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t seqCounter = 0;

// Zustandsvariablen
long lastEncVal = 0;
float lastLux = 0.0f;
volatile bool lastTxSuccess = false;
unsigned long lastDisplayUpdateMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRenderPacketMs = 0;

// Empfangener Render-Befehl vom S3 (wird im Callback geschrieben)
volatile bool renderPending = false;
ProbeRenderPacket lastRenderPacket;

// Interrupt-Flag für Sensor
volatile bool tslInterruptOccurred = false;

// =============================================================================
// ISR: TSL2591 Data Ready
// =============================================================================
void IRAM_ATTR tslISR() {
    tslInterruptOccurred = true;
}

// =============================================================================
// ESP-NOW CALLBACKS
// =============================================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    lastTxSuccess = (status == ESP_NOW_SEND_SUCCESS);
}

// Empfangs-Callback: S3 → C6 (Render-Befehle)
// IRAM_ATTR: Läuft im WiFi-Treiber Kontext, muss schnell sein.
IRAM_ATTR void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    // Prüfe ob es ein ProbeRenderPacket ist (63 Bytes, Magic 0xD4)
    if (len == sizeof(ProbeRenderPacket) && data[0] == REMOTE_MAGIC) {
        memcpy((void*)&lastRenderPacket, data, sizeof(ProbeRenderPacket));
        renderPending = true;
        lastRenderPacketMs = millis();
    }
}

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================

// Sendet ein Event-Paket an den S3
void sendEvent(ProbeEventType type, float luxG0 = 0.0f, float luxG5 = 0.0f) {
    ProbeEventPacket pkt;
    pkt.magic      = REMOTE_MAGIC;
    pkt.event_type = (uint8_t)type;
    pkt.seq        = seqCounter++;
    pkt.lux_raw_g0 = luxG0;
    pkt.lux_raw_g5 = luxG5;
    
    esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
}

// Führt eine TSL2591-Messung durch und gibt Lux zurück
float measureLux() {
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);
    return (lux < 0.0f) ? 0.0f : lux;
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    
    // 1. Hardware Init
    Wire.begin(I2C_SDA, I2C_SCL);
    initOutput();
    showStartup();
    initInput();
    
    // INT-Pin für Sensor
    pinMode(PIN_TSL_INT, INPUT_PULLUP);
    
    // 2. Sensor Init
    if (!tsl.begin()) {
        showError("No Sensor");
        while(1);
    }
    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    tsl.clearInterrupt();
    tsl.registerInterrupt(0, 0xFFFF, TSL2591_PERSIST_ANY);
    attachInterrupt(digitalPinToInterrupt(PIN_TSL_INT), tslISR, FALLING);
    
    // 3. ESP-NOW Init (Bidirektional)
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        showError("ESP-NOW Fail");
        delay(2000);
        ESP.restart();
    }
    
    // Sende- UND Empfangs-Callbacks registrieren
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    
    // Broadcast-Peer für Senden
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    Serial.println("[PROBE] Bidirectional ESP-NOW ready");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    unsigned long now = millis();
    
    // =========================================================================
    // 1. INPUT AUSWERTUNG & EVENT-VERSAND
    // =========================================================================
    
    // --- Taster 2 (unten links): MESSEN / FEUER ---
    if (isT2Pressed()) {
        clickSound();
        sendEvent(EVT_T2_CLICK);
    }
    
    // --- Taster 1 (oben links): UNDO / ZURÜCK / REFERENZ ---
    if (isT1Pressed()) {
        clickSound();
        sendEvent(EVT_T1_CLICK);
    }
    
    // --- Encoder Klick: ENTER / ABSCHLIESSEN ---
    if (isEncPressed()) {
        clickSound();
        sendEvent(EVT_ENC_CLICK);
    }
    
    // --- Encoder Drehen: NAVIGATION ---
    long currentEnc = getEncoderValue();
    if (currentEnc != lastEncVal) {
        long delta = currentEnc - lastEncVal;
        // Sende pro Rast-Schritt ein Event (kein Batching für präzise Navigation)
        if (delta > 0) {
            for (long i = 0; i < delta; i++) sendEvent(EVT_ENC_UP);
        } else {
            for (long i = 0; i < -delta; i++) sendEvent(EVT_ENC_DOWN);
        }
        clickSound();
        lastEncVal = currentEnc;
    }
    
    // --- Heartbeat: Periodischer Lebenszeichen-Ping ---
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
        sendEvent(EVT_HEARTBEAT);
        lastHeartbeatMs = now;
    }
    
    // =========================================================================
    // 2. RENDER-BEFEHL VOM S3 VERARBEITEN
    // =========================================================================
    if (renderPending) {
        renderPending = false;
        
        ProbeRenderPacket pkt;
        memcpy(&pkt, (void*)&lastRenderPacket, sizeof(pkt));
        
        // Mess-Befehle vom Flash-Handshake?
        if (pkt.command == CMD_MEASURE_G0) {
            // S3 hat Grün eingeschaltet → Messen und zurücksenden
            delay(50);  // Sensor-Einschwingzeit abwarten
            float luxG0 = measureLux();
            sendEvent(EVT_LUX_DATA, luxG0, 0.0f);
        }
        else if (pkt.command == CMD_MEASURE_G5) {
            // S3 hat Blau eingeschaltet → Messen und zurücksenden
            delay(50);
            float luxG5 = measureLux();
            sendEvent(EVT_LUX_DATA, 0.0f, luxG5);
        }
        else {
            // CMD_RENDER oder CMD_IDLE: Display aktualisieren
            renderFromPacket(pkt);
            lastDisplayUpdateMs = now;
        }
    }
    
    // =========================================================================
    // 3. PASSIVES SENSOR-UPDATE (Hintergrund-Lux für Standby-Anzeige)
    // =========================================================================
    if (tslInterruptOccurred) {
        tslInterruptOccurred = false;
        lastLux = measureLux();
    }
    
    // =========================================================================
    // 4. FALLBACK DISPLAY (Wenn S3 lange nichts sendet)
    // =========================================================================
    bool s3Connected = (now - lastRenderPacketMs) < 3000;
    if (!s3Connected && (now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
        showStandby(lastLux, seqCounter, lastTxSuccess);
        lastDisplayUpdateMs = now;
    }
    
    // 5. CPU-Entlastung
    delay(5);
}