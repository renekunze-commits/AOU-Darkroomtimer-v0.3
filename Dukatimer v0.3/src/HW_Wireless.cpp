/* HW_Wireless.cpp - ESP-NOW Bidirektionale Kommunikation (S3 ↔ C6)
   
   Architektur (Single Source of Truth):
   - Empfängt ProbeEventPacket (14 Bytes) vom Handgerät (Button/Encoder-Events)
   - Sendet ProbeRenderPacket (63 Bytes) als Render-Befehl an das Handgerät
   - Implementiert den Flash-Handshake für spektrale Messungen
   - Abwärtskompatibel: Empfängt auch Legacy WirelessPacket (Lux-Broadcast)

   Kommunikationsablauf:
   1. C6 → S3: ProbeEventPacket (EVT_T2_CLICK, EVT_ENC_UP, etc.)
   2. S3 verarbeitet Event in State-Machine (main loop)
   3. S3 → C6: ProbeRenderPacket (Header + Lines + Histogram + Haptic)

   Flash-Handshake (Spektralmessung):
   1. S3 empfängt EVT_T2_CLICK im Mess-Modus
   2. S3 schaltet Safe/Focus AUS, G0 (Grün) auf 100%
   3. S3 sendet CMD_MEASURE_G0 an C6
   4. C6 misst, sendet EVT_LUX_DATA mit lux_raw_g0
   5. S3 schaltet G5 (Blau) auf 100%
   6. S3 sendet CMD_MEASURE_G5 an C6
   7. C6 misst, sendet EVT_LUX_DATA mit lux_raw_g5
   8. S3 berechnet Dosis, sendet finales Render-Paket
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "Globals.h"
#include "Types.h"
#include "freertos/portmacro.h"

// =============================================================================
// STATISCHE VARIABLEN
// =============================================================================
static uint32_t expectedSeq = 0;
volatile uint32_t totalDroppedPackets = 0;
volatile uint32_t probeEventOverruns = 0;
static uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool espNowReady = false;

portMUX_TYPE luxMux = portMUX_INITIALIZER_UNLOCKED;

// Bounded FIFO fuer Probe-Events (ISR Producer -> Main Loop Consumer)
static volatile uint8_t probeEvtQ[8];
static volatile uint8_t probeEvtHead = 0;
static volatile uint8_t probeEvtTail = 0;
static volatile uint8_t probeEvtCount = 0;

static inline void IRAM_ATTR pushProbeEvent(uint8_t evt) {
    if (probeEvtCount < sizeof(probeEvtQ)) {
        probeEvtQ[probeEvtHead] = evt;
        probeEvtHead = (probeEvtHead + 1) % sizeof(probeEvtQ);
        probeEvtCount++;
        probeLastEvent = evt;      // Legacy-Sicht
        probeEventPending = true;  // Legacy-Sicht
    } else {
        probeEventOverruns++;
    }
}

bool popProbeEvent(uint8_t &evt, float &luxG0, float &luxG5) {
    bool hasEvent = false;
    portENTER_CRITICAL(&luxMux);
    if (probeEvtCount > 0) {
        evt = probeEvtQ[probeEvtTail];
        probeEvtTail = (probeEvtTail + 1) % sizeof(probeEvtQ);
        probeEvtCount--;
        hasEvent = true;
    } else {
        evt = EVT_NONE;
    }

    luxG0 = probeLuxG0;
    luxG5 = probeLuxG5;

    if (probeEvtCount == 0) {
        probeLastEvent = EVT_NONE;
        probeEventPending = false;
    } else {
        probeLastEvent = probeEvtQ[probeEvtTail];
        probeEventPending = true;
    }
    portEXIT_CRITICAL(&luxMux);
    return hasEvent;
}

// =============================================================================
// EMPFANGS-CALLBACK (Interrupt-Kontext → schnell halten!)
// =============================================================================
// Unterstützt beide Pakettypen:
//   - ProbeEventPacket (14 Bytes) = neues bidirektionales Protokoll
//   - WirelessPacket (9 Bytes) = Legacy Lux-Broadcast (Abwärtskompatibel)
// =============================================================================

IRAM_ATTR void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len < 1 || incomingData[0] != REMOTE_MAGIC) return;
    
    // --- Neues Protokoll: ProbeEventPacket (14 Bytes) ---
    if (len == sizeof(ProbeEventPacket)) {
        ProbeEventPacket pkt;
        memcpy(&pkt, incomingData, sizeof(pkt));
        
        // Sequenz-Tracking (Packet Loss Detection)
        if (pkt.seq > expectedSeq && expectedSeq > 0) {
            totalDroppedPackets += (pkt.seq - expectedSeq);
        }
        expectedSeq = pkt.seq + 1;
        
        portENTER_CRITICAL_ISR(&luxMux);
        lastRemotePacketMs = millis();
        
        if (pkt.event_type == EVT_LUX_DATA) {
            // Mess-Antwort vom Flash-Handshake
            probeLuxG0 = pkt.lux_raw_g0;
            probeLuxG5 = pkt.lux_raw_g5;
            // Auch remoteLux aktualisieren (Kompatibilität)
            float maxLux = (pkt.lux_raw_g0 > pkt.lux_raw_g5) ? pkt.lux_raw_g0 : pkt.lux_raw_g5;
            if (maxLux > 0.0f) remoteLux = (double)maxLux;
        }
        else if (pkt.event_type == EVT_HEARTBEAT) {
            probeConnected = true;
        }
        else if (pkt.event_type != EVT_NONE) {
            // Button/Encoder-Event -> FIFO fuer Main Loop
            pushProbeEvent(pkt.event_type);
        }
        
        portEXIT_CRITICAL_ISR(&luxMux);
        return;
    }
    
    // --- Legacy Protokoll: WirelessPacket (9 Bytes) ---
    if (len == sizeof(WirelessPacket)) {
        WirelessPacket pkt;
        memcpy(&pkt, incomingData, sizeof(pkt));
        
        portENTER_CRITICAL_ISR(&luxMux);
        remoteLux = (double)pkt.lux;
        lastRemotePacketMs = millis();
        probeConnected = true;
        portEXIT_CRITICAL_ISR(&luxMux);
    }
}

// =============================================================================
// SENDE-CALLBACK
// =============================================================================
static volatile bool lastS3TxSuccess = false;

IRAM_ATTR void OnDataSentS3(const uint8_t *mac_addr, esp_now_send_status_t status) {
    lastS3TxSuccess = (status == ESP_NOW_SEND_SUCCESS);
}

// =============================================================================
// SENDE-FUNKTIONEN: S3 → C6
// =============================================================================

// Sendet ein vollständiges Render-Paket an das Handgerät
void sendProbeRender(const char* header, const char* line1, const char* line2,
                     const uint8_t* histogram, uint8_t haptic, uint8_t mode) {
    if (!espNowReady) return;
    
    ProbeRenderPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = REMOTE_MAGIC;
    pkt.command = CMD_RENDER;
    
    if (header) strncpy(pkt.header_text, header, 15);
    if (line1)  strncpy(pkt.line1_text,  line1,  15);
    if (line2)  strncpy(pkt.line2_text,  line2,  15);
    if (histogram) memcpy(pkt.zone_histogram, histogram, 11);
    
    pkt.haptic_feedback = haptic;
    pkt.display_mode = mode;
    
    esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}

void sendRenderPacketToC6() {
    ProbeRenderPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = 0xD4;
    pkt.command = CMD_RENDER; // RENDER_UPDATE

    // 1. Titel anhand des Fokus dynamisch setzen
    if (currentMeasureFocus == FOCUS_HIGHLIGHTS) {
        strncpy(pkt.header_text, "[ LICHTER ]", sizeof(pkt.header_text) - 1);
    } else {
        strncpy(pkt.header_text, "[ SCHATTEN ]", sizeof(pkt.header_text) - 1);
    }
    pkt.header_text[sizeof(pkt.header_text) - 1] = '\0'; // Null-Terminierung

    // 2. Errechnete Basis-Zeit anzeigen
    snprintf(pkt.line1_text, sizeof(pkt.line1_text), "Zeit: %.1fs", timer_base_seconds);

    // Gradation (vorerst statisch, wird später berechnet)
    snprintf(pkt.line2_text, sizeof(pkt.line2_text), "Grad: %.1f", 2.5);

    // 3. Das Histogramm aus der Measurement-Logik kopieren
    memcpy(pkt.zone_histogram, currentZoneHistogram, 11);

    // 4. Haptisches Feedback auslösen (1 = Kurzer Klick)
    pkt.haptic_feedback = 1;
    pkt.display_mode = (currentMode == MODE_BW) ? PMODE_METER_BW : PMODE_METER_SG;

    // Paket absenden
    esp_err_t result = esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(ProbeRenderPacket));

    if (result != ESP_OK) {
        Serial.println("Fehler beim Senden des Render-Pakets");
    }
}

// Sendet einen Mess-Befehl (Flash-Handshake Phase 1 oder 2)
void sendProbeMeasureCmd(uint8_t cmd) {
    if (!espNowReady) return;
    
    ProbeRenderPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = REMOTE_MAGIC;
    pkt.command = cmd;  // CMD_MEASURE_G0 oder CMD_MEASURE_G5
    
    // Informative Texte für den Fall, dass C6 das Paket auch rendert
    if (cmd == CMD_MEASURE_G0) {
        strncpy(pkt.header_text, "[ MEASURING ]", 15);
        strncpy(pkt.line1_text,  "Phase: GREEN", 15);
    } else {
        strncpy(pkt.header_text, "[ MEASURING ]", 15);
        strncpy(pkt.line1_text,  "Phase: BLUE", 15);
    }
    
    esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}

// Sendet Idle-Befehl (Handgerät geht in Standby)
void sendProbeIdle() {
    if (!espNowReady) return;
    
    ProbeRenderPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = REMOTE_MAGIC;
    pkt.command = CMD_IDLE;
    strncpy(pkt.header_text, "[ STANDBY ]", 15);
    strncpy(pkt.line1_text,  "Waiting...", 15);
    
    esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}

// =============================================================================
// INITIALISIERUNG
// =============================================================================
void initWireless() {
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("[WIRELESS] Error initializing ESP-NOW");
        return;
    }
    
    // Empfangs- UND Sende-Callbacks registrieren
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSentS3));
    
    // Broadcast-Peer registrieren (für Senden an C6)
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    espNowReady = true;
    Serial.println("[WIRELESS] Bidirectional ESP-NOW Ready. Listening + Sending.");
}
