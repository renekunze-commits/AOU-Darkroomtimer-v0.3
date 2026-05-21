/* Wireless TSL2591 Probe - ESP32-C6 "Dumb Terminal"
   
   Architektur:
   Das Handgerät hält keinen eigenen Status, berechnet keine Mathematik und
   kennt keine Papier-Profile. Es liest Taster/Encoder aus, sendet Events
   an den ESP32-S3 und zeichnet auf dem LCD exakt das, was der S3 
   im Antwort-Paket befiehlt.
   
   Sensorik: 
   600ms High-Precision Integration mit dynamischem Auto-Ranging 
   und 36.000-ADC-Clipping-Schutz.
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
// GLOBALE OBJEKTE & STATUS
// =============================================================================
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t seqCounter = 0;

long lastEncVal = 0;
float lastLux = 0.0f;
unsigned long lastDisplayUpdateMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRenderPacketMs = 0;

volatile bool renderPending = false;
ProbeRenderPacket lastRenderPacket;

unsigned long lastSensorPollMs = 0;
#define SENSOR_POLL_INTERVAL 800  // Passives Polling (> 600ms Integrationszeit)
bool tslAvailable = false;        
static tsl2591Gain_t currentGain = TSL2591_GAIN_MED; 

// =============================================================================
// ESP-NOW CALLBACKS (ESP-IDF v5 / Arduino 3.x Kompatibel)
// =============================================================================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
#endif
    if (len == sizeof(ProbeRenderPacket) && data[0] == REMOTE_MAGIC) {
        memcpy((void*)&lastRenderPacket, data, sizeof(ProbeRenderPacket));
        renderPending = true;
        lastRenderPacketMs = millis();
    }
}

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================
void sendEvent(ProbeEventType type, float luxG0 = 0.0f, float luxG5 = 0.0f) {
    ProbeEventPacket pkt;
    pkt.magic      = REMOTE_MAGIC;
    pkt.event_type = (uint8_t)type;
    pkt.seq        = seqCounter++;
    pkt.lux_raw_g0 = luxG0;
    pkt.lux_raw_g5 = luxG5;
    
    esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
}

void recoverI2CBus() {
    // ROOT CAUSE FIX: Wire.end() entfernt! 
    // In IDF v5 crasht dies den Treiber, wenn Wire vorher noch nicht lief.
    
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(PIN_I2C_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(5);
        if (digitalRead(PIN_I2C_SDA) == HIGH) break; 
    }
    pinMode(PIN_I2C_SDA, OUTPUT);
    digitalWrite(PIN_I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SDA, HIGH);
    delayMicroseconds(5);
    
    // Globale Initialisierung des Busses mit den Custom-Pins
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
}

// =============================================================================
// AUTO-RANGING SENSOR LOGIK (600ms High Precision)
// =============================================================================
float measureLux() {
    if (!tslAvailable) return -1.0f;
    
    // Blockiert bei 600ms Integration automatisch für ca. 720ms
    uint32_t lum = tsl.getFullLuminosity(); 
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    
    // 1. Sättigungs-Check (Zu hell)
    // Harte ADC-Wand bei 36863 antizipieren! 
    if (full > 36000 || ir > 36000) {
        if (currentGain == TSL2591_GAIN_MAX) {
            currentGain = TSL2591_GAIN_HIGH;
            tsl.setGain(currentGain);
            return -1.0f; 
        } else if (currentGain == TSL2591_GAIN_HIGH) {
            currentGain = TSL2591_GAIN_MED;
            tsl.setGain(currentGain);
            return -1.0f; 
        } else if (currentGain == TSL2591_GAIN_MED) {
            currentGain = TSL2591_GAIN_LOW;
            tsl.setGain(currentGain);
            return -1.0f;
        }
    }
    
    // 2. Rauschen-Check (Zu dunkel)
    if (full < 300 && full > 0) { 
        if (currentGain == TSL2591_GAIN_LOW) {
            currentGain = TSL2591_GAIN_MED;
            tsl.setGain(currentGain);
            return -1.0f;
        } else if (currentGain == TSL2591_GAIN_MED) {
            currentGain = TSL2591_GAIN_HIGH;
            tsl.setGain(currentGain);
            return -1.0f;
        } else if (currentGain == TSL2591_GAIN_HIGH) {
            currentGain = TSL2591_GAIN_MAX; 
            tsl.setGain(currentGain);
            return -1.0f;
        }
    }

    float lux = tsl.calculateLux(full, ir);
    if (isnan(lux) || isinf(lux) || lux < 0.0f) return 0.0f;
    return lux;
}

bool initTSL2591() {
    if (!tsl.begin()) return false;
    currentGain = TSL2591_GAIN_MED;
    tsl.setGain(currentGain);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS); // Max. Präzision
    return true;
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500); // Kurze Pause für stabiles Hochfahren
    
    // ROOT CAUSE FIX: Reihenfolge zwingend einhalten!
    // 1. I2C Bus physikalisch entstören und mit Custom-Pins (SDA=22, SCL=21) global starten
    recoverI2CBus();

    // 2. JETZT ERST die Displays/Sensoren rufen (sie nutzen dann den konfigurierten Bus)
    initOutput();     
    showStartup();

    // 3. I2C Hardware Scan
    byte error, address;
    int nDevices = 0;
    bool tslFound = false;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            nDevices++;
            if (address == 0x29) tslFound = true;
        }
    }
    
    if (nDevices == 0 || !tslFound) {
        showError(nDevices == 0 ? "No I2C HW!" : "TSL missing!");
        while(1) { 
            digitalWrite(PIN_VIB_MOTOR, HIGH); delay(100);
            digitalWrite(PIN_VIB_MOTOR, LOW); delay(500);
        }
    }

    initInput(); // Encoder & Buttons
    tslAvailable = initTSL2591();
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        showError("ESP-NOW Fail");
        delay(2000);
        ESP.restart();
    }
    
    esp_now_register_recv_cb(OnDataRecv);
    
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    unsigned long now = millis();
    
    // 1. INPUT
    if (isT2Pressed()) { clickSound(); sendEvent(EVT_T2_CLICK); }
    if (isT1Pressed()) { clickSound(); sendEvent(EVT_T1_CLICK); }
    if (isEncPressed()) { clickSound(); sendEvent(EVT_ENC_CLICK); }
    
    long currentEnc = getEncoderValue();
    if (currentEnc != lastEncVal) {
        long delta = currentEnc - lastEncVal;
        if (delta > 0) {
            for (long i = 0; i < delta; i++) sendEvent(EVT_ENC_UP);
        } else {
            for (long i = 0; i < -delta; i++) sendEvent(EVT_ENC_DOWN);
        }
        clickSound();
        lastEncVal = currentEnc;
    }
    
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
        sendEvent(EVT_HEARTBEAT);
        lastHeartbeatMs = now;
    }
    
    // 2. RENDER & HANDSHAKES
    if (renderPending) {
        renderPending = false;
        ProbeRenderPacket pkt;
        memcpy(&pkt, (void*)&lastRenderPacket, sizeof(pkt));
        
        if (pkt.command == CMD_MEASURE_G0 || pkt.command == CMD_MEASURE_G5) {
            
            // Haptische Signalisierung für Messstart (nur beim ersten Schritt G0)
            if (pkt.command == CMD_MEASURE_G0) {
                digitalWrite(PIN_VIB_MOTOR, HIGH); delay(40); digitalWrite(PIN_VIB_MOTOR, LOW);
            }

            delay(150); // S3 LED Einschwingzeit
            
            float lux = measureLux();
            int retries = 0;
            
            // Abfangen von Gain-Wechseln (measureLux blockiert selbst, kein delay nötig)
            while (lux < 0.0f && retries < 4) {
                lux = measureLux();
                retries++;
            }
            if (lux < 0.0f) {
                recoverI2CBus();
                tslAvailable = initTSL2591();
                lux = measureLux();
            }
            
            if (pkt.command == CMD_MEASURE_G0) {
                sendEvent(EVT_LUX_DATA, (lux >= 0.0f ? lux : 0.0f), 0.0f);
            } else {
                sendEvent(EVT_LUX_DATA, 0.0f, (lux >= 0.0f ? lux : 0.0f));
            }
        }
        else {
            renderFromPacket(pkt);
            lastDisplayUpdateMs = now;
        }
    }
    
    // 3. PASSIVES POLLING
    if (tslAvailable && (now - lastSensorPollMs >= SENSOR_POLL_INTERVAL)) {
        lastSensorPollMs = now;
        float lux = measureLux();
        if (lux >= 0.0f) lastLux = lux;
    }
    
    // 4. FALLBACK DISPLAY
    bool s3Connected = (lastRenderPacketMs > 0) && ((now - lastRenderPacketMs) < 3000);
    if (!s3Connected && (now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
        showStandby(lastLux, seqCounter, s3Connected);
        lastDisplayUpdateMs = now;
    }
    
    // 5. DISPLAY SLEEP (Pflichtenheft-Ergänzung: Energiesparen bei Inaktivität)
    // checkDisplaySleep() schaltet das OLED nach DISPLAY_SLEEP_TIMEOUT in PowerSave.
    // War in output.h/output.cpp definiert, aber nie aufgerufen.
    checkDisplaySleep();
    
    delay(5); // Kooperatives Multitasking für FreeRTOS
}