/* HW_Wireless.cpp - ESP-NOW Bidirektionale Kommunikation (S3 ↔ C6)
   
   Architektur (Single Source of Truth):
    - Empfängt ProbeEventPacket (16 Bytes) vom Handgerät (Button/Encoder-Events)
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

// NEXTION UI INTEGRATION (Ergänzung gemäß Vorgabe)
extern void updateNextionUI(bool force);

// =============================================================================
// STATISCHE VARIABLEN
// =============================================================================
static uint32_t expectedSeq = 0;
// `totalDroppedPackets` and `probeEventOverruns` are defined in src/main.cpp (Globals).
// Keep file-local state minimal to avoid multiple-definition linker errors.
static uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool espNowReady = false;

// FIX Punkt 4: MAC-Binding gegen Cross-Talk bei mehreren Geräten im selben RF-Raum.
// Beim ersten gültigen ProbeEventPacket wird die Absender-MAC gespeichert.
// Danach werden Pakete von fremden MACs verworfen.
static uint8_t boundPeerMAC[6] = {0};
static bool peerMACBound = false;
static volatile bool wirelessNextionRefreshPending = false;

portMUX_TYPE luxMux = portMUX_INITIALIZER_UNLOCKED;

// Bounded FIFO fuer Probe-Events (ISR Producer -> Main Loop Consumer)
static volatile uint8_t probeEvtQ[32];
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
//   - ProbeEventPacket (16 Bytes) = neues bidirektionales Protokoll
//   - WirelessPacket (12 Bytes) = Legacy Lux-Broadcast (Abwärtskompatibel)
// =============================================================================

IRAM_ATTR void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len < 1 || incomingData[0] != REMOTE_MAGIC) return;
    
    // --- Neues Protokoll: ProbeEventPacket (16 Bytes) ---
    if (len == sizeof(ProbeEventPacket)) {
        // FIX Punkt 4: MAC-Filter — erstes gültiges C6 binden, fremde verwerfen
        if (!peerMACBound) {
            memcpy(boundPeerMAC, mac, 6);
            peerMACBound = true;
        } else if (memcmp(mac, boundPeerMAC, 6) != 0) {
            return; // Fremdes Gerät — Paket verwerfen
        }

        ProbeEventPacket pkt;
        memcpy(&pkt, incomingData, sizeof(pkt));
        
        // Sequenz-Tracking (Packet Loss Detection)
        if (pkt.seq > expectedSeq && expectedSeq > 0) {
            totalDroppedPackets += (pkt.seq - expectedSeq);
            wirelessNextionRefreshPending = true; // NEXTION UPDATE wird im Task-Kontext nachgezogen
        }
        expectedSeq = pkt.seq + 1;
        
        portENTER_CRITICAL_ISR(&luxMux);
        lastRemotePacketMs = millis();
        
        // FIX Punkt 6: Jedes gueltige Paket bestaetigt Verbindung
        if (!probeConnected) {
            probeConnected = true;
            wirelessNextionRefreshPending = true; // NEXTION UPDATE wird im Task-Kontext nachgezogen
        }
        
        if (pkt.event_type == EVT_LUX_DATA) {
            // Mess-Antwort vom Flash-Handshake
            probeLuxG0 = pkt.lux_raw_g0;
            probeLuxG5 = pkt.lux_raw_g5;
            // Auch remoteLux aktualisieren (Kompatibilität)
            float maxLux = (pkt.lux_raw_g0 > pkt.lux_raw_g5) ? pkt.lux_raw_g0 : pkt.lux_raw_g5;
            if (maxLux > 0.0f) remoteLux = (double)maxLux;
            // Kritischer Bugfix: Die Statemachine im Main-Loop konsumiert nur FIFO-Events.
            // Ohne dieses Queue-Event bleibt EVT_LUX_DATA unsichtbar und der Handshake läuft in Timeout.
            pushProbeEvent(EVT_LUX_DATA);
        }
        else if (pkt.event_type == EVT_HEARTBEAT) {
            probeConnected = true;
        }
        else if (pkt.event_type != EVT_NONE) {
            // FIX K3: EVT_T1_CLICK (Remote Cancel) auch während Messung durchlassen
            if (lightOperationActive || isMeasuring) {
                if (pkt.event_type == EVT_T1_CLICK) {
                    pushProbeEvent(pkt.event_type);
                }
                portEXIT_CRITICAL_ISR(&luxMux);
                return;
            }
            // Button/Encoder-Event -> FIFO fuer Main Loop
            pushProbeEvent(pkt.event_type);
        }
        
        portEXIT_CRITICAL_ISR(&luxMux);
        return;
    }
    
    // --- Legacy Protokoll: WirelessPacket (12 Bytes) ---
    if (len == sizeof(WirelessPacket)) {
        WirelessPacket pkt;
        memcpy(&pkt, incomingData, sizeof(pkt));
        
        portENTER_CRITICAL_ISR(&luxMux);
        remoteLux = (double)pkt.lux;
        lastRemotePacketMs = millis();
        if (!probeConnected) {
            probeConnected = true;
            wirelessNextionRefreshPending = true; // NEXTION UPDATE wird im Task-Kontext nachgezogen
        }
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

// =============================================================================
// PFLICHTENHEFT FIX (Modus 9.2): Modusspezifische Render-Pakete für C6
// VORHER: sendRenderPacketToC6() sendete immer nur Metering-Informationen
//         (Lichter/Schatten Header, timer_base_seconds, statische Gradation 2.5).
//         Laut Pflichtenheft Abschnitt 9.2 muss das OLED des Handgeräts
//         je nach aktivem S3-Modus unterschiedliche Informationen anzeigen:
//         - BW/SG Metering: Zonen-Histogramm, Zeit, Gradation
//         - Burn: "[ BURN n ARMED ]", berechnete Zeit und Gradation
//         - Kalibrierung: "[ CALIBRATE ]", Messanweisung
//         - Densitometer: "[ DENSITOMETER ]", Lux und Dichte
// NEU: Dynamische Auswahl des Render-Inhalts basierend auf currentMode.
//      Gradation wird jetzt dynamisch aus grade_bw berechnet (war vorher
//      fest auf 2.5 codiert – PFLICHTENHEFT-VERLETZUNG).
// =============================================================================
void sendRenderPacketToC6() {
    ProbeRenderPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = 0xD4;
    pkt.command = CMD_RENDER;

    // PFLICHTENHEFT FIX: Modusspezifische Render-Logik
    switch (currentMode) {
        case MODE_BW:
        case MODE_SG: {
            // --- A) Mess-Modus (BW & Splitgrade - Zonen-Messung) ---
            if (currentMeasureFocus == FOCUS_HIGHLIGHTS) {
                strncpy(pkt.header_text, "[ LICHTER ]", sizeof(pkt.header_text) - 1);
            } else {
                strncpy(pkt.header_text, "[ SCHATTEN ]", sizeof(pkt.header_text) - 1);
            }
            pkt.header_text[sizeof(pkt.header_text) - 1] = '\0';

            // Bugfix Zeitanzeige: timer_base_seconds wird im aktuellen Codepfad nicht
            // zyklisch gepflegt und fuehrte zu statischer/alter Anzeige auf dem C6.
            // Deshalb wird hier die tatsaechlich aktive Zeitquelle je Modus verwendet:
            // - BW: time_bw
            // - SG: je nach Fokus Soft/Hard
            float renderTimeSeconds = time_bw;
            if (currentMode == MODE_SG) {
                renderTimeSeconds = (currentMeasureFocus == FOCUS_HIGHLIGHTS) ? time_soft : time_hard;
            }
            snprintf(pkt.line1_text, sizeof(pkt.line1_text), "Zeit: %.1fs", renderTimeSeconds);

            // PFLICHTENHEFT FIX: Dynamische Gradation statt statisch "2.5"
            // VORHER: snprintf(pkt.line2_text, ..., "Grad: %.1f", 2.5);
            // NEU: Echten Gradationswert aus grade_bw verwenden
            snprintf(pkt.line2_text, sizeof(pkt.line2_text), "Grad: %.1f", grade_bw);

            // Histogramm aus der Measurement-Logik kopieren
            memcpy(pkt.zone_histogram, currentZoneHistogram, 11);
            pkt.display_mode = (currentMode == MODE_BW) ? PMODE_METER_BW : PMODE_METER_SG;
            break;
        }

        case MODE_BURN: {
            // --- B) Burn-Modus (Nachbelichten Remote) ---
            // Pflichtenheft 9.2 B): OLED zeigt "[ BURN 1 ]", Zeit und Gradation
            strncpy(pkt.header_text, "[ BURN ARMED ]", sizeof(pkt.header_text) - 1);
            extern double getEffectiveBurnTime();
            snprintf(pkt.line1_text, sizeof(pkt.line1_text), "Zeit: %.1fs",
                     (float)getEffectiveBurnTime());
            snprintf(pkt.line2_text, sizeof(pkt.line2_text), "+%.1fEV G:%.1f",
                     burnEv, burnGrade);
            pkt.display_mode = PMODE_BURN;
            break;
        }

        case MODE_TIMER: {
            // Bugfix MODE_TIMER: Dieser Modus darf nicht in die generische Standby-Anzeige
            // fallen, sonst sieht das C6 keine laufenden Zeit-/Gradationsaenderungen.
            strncpy(pkt.header_text, "[ TIMER ]", sizeof(pkt.header_text) - 1);
            snprintf(pkt.line1_text, sizeof(pkt.line1_text), "Zeit: %.1fs", time_bw);
            snprintf(pkt.line2_text, sizeof(pkt.line2_text), "Grad: %.1f", grade_bw);
            pkt.display_mode = PMODE_METER_BW;
            break;
        }

        case MODE_CALIB: {
            // --- C) Kalibrierungs-Modus (Stouffer) ---
            // Pflichtenheft 9.2 C): OLED zeigt "[ CALIBRATE ]" und Messanweisung
            strncpy(pkt.header_text, "[ CALIBRATE ]", sizeof(pkt.header_text) - 1);
            strncpy(pkt.line1_text,  "Measure Base", sizeof(pkt.line1_text) - 1);
            strncpy(pkt.line2_text,  "T2=Messen", sizeof(pkt.line2_text) - 1);
            pkt.display_mode = PMODE_CALIBRATE;
            break;
        }

        case MODE_DENS: {
            // --- D) Densitometer-Modus ---
            // Pflichtenheft 9.2 D): OLED zeigt "[ DENSITOMETER ]"
            strncpy(pkt.header_text, "[ DENSITOM ]", sizeof(pkt.header_text) - 1);
            strncpy(pkt.line1_text,  "T1=Ref T2=Mess", sizeof(pkt.line1_text) - 1);
            strncpy(pkt.line2_text,  "", sizeof(pkt.line2_text) - 1);
            pkt.display_mode = PMODE_DENSITOM;
            break;
        }

        default: {
            // IDLE / Setup / Teststrip: Generische Standby-Anzeige
            strncpy(pkt.header_text, "[ STANDBY ]", sizeof(pkt.header_text) - 1);
            snprintf(pkt.line1_text, sizeof(pkt.line1_text), "Zeit: %.1fs", time_bw);
            strncpy(pkt.line2_text,  "", sizeof(pkt.line2_text) - 1);
            pkt.display_mode = PMODE_IDLE;
            break;
        }
    }

    // PFLICHTENHEFT FIX: Haptisches Feedback nur bei passivem Render-Update entfernt.
    // VORHER: pkt.haptic_feedback = 1 (löste bei JEDEM 250ms-Update einen Vibrationsmotor-Klick aus!)
    // VORHER: pkt.display_mode wurde hier pauschal überschrieben (ignorierte die modusspezifische Zuweisung oben)
    // NEU: haptic_feedback bleibt 0 für passive Updates. Aktives Feedback wird gezielt
    //      von den Mess-/Burn-Handlern in Logic_Measurement.cpp gesetzt.
    // ALT:
    // pkt.haptic_feedback = 1;
    // pkt.display_mode = (currentMode == MODE_BW) ? PMODE_METER_BW : PMODE_METER_SG;

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

// =============================================================================
// PROBE DISPLAY UPDATE (K06 Fix: fehlende Definition)
// =============================================================================
void updateProbeDisplay() {
    if (wirelessNextionRefreshPending) {
        wirelessNextionRefreshPending = false;
        updateNextionUI(true);
    }
    if (!probeConnected) return;
    sendRenderPacketToC6();
}

// =============================================================================
// PFLICHTENHEFT ERGÄNZUNG (Modus 9.2): Modusspezifisches Probe-Event-Routing
// =============================================================================
// Diese Funktion wird aus vTaskRealtime (main.cpp) aufgerufen, wenn ein
// Probe-Event NICHT von der Spektral-Statemachine konsumiert wurde.
// Sie implementiert die "eiserne Logik-Regel" des Pflichtenhefts:
//
// A) Mess-Modus (BW/SG Idle):
//    - T2 (Messen): Startet den spektralen Flash-Handshake
//    - T1 (Undo):   Löscht den letzten Messpunkt
//    - ENC Drehen:  Wechselt Soft/Hard Fokus (nur SG)
//    - ENC Klick:   Schließt die Mess-Session ab
//
// B) Burn-Modus:
//    - T2 (Feuer):  Fernauslöser für die aktuelle Nachbelichtung
//    - T1 (Abort):  Bricht die laufende Nachbelichtung ab
//    - ENC Drehen:  Blättert durch Burn-Steps (nicht implementiert: nur 1 Step)
//
// C) Kalibrierungs-Modus:
//    - T2 (Messen): Injiziert EVT_START_PRESSED in die Queue (triggert Wizard)
//
// D) Densitometer-Modus:
//    - T1 (Referenz): Injiziert EVT_START_PRESSED (referenzmessung)
//    - T2 (Messen):   Injiziert EVT_ENTER_PRESSED (Dichtemessung)
// =============================================================================

// Externe Funktionen aus Logic_Measurement.cpp
extern bool triggerSpectralMeasurement();
extern void undoLastMeasurement();
extern void finalizeMeteringSession();
extern void startMeteringSession();
extern bool isMeasurementActive();

// FIX Punkt 2: Diagnose-Counter für Queue-Backpressure.
// Vorher wurde der Rückgabewert von xQueueSend() ignoriert. Bei vollem Queue
// gingen Remote-Events (START/ENTER/BACK vom C6) still verloren.
volatile uint32_t inputQueueDrops = 0;

// Hilfsfunktion: Event in die xInputQueue injizieren (wie Nextion-Touch)
static void injectInputEvent(InputEventType type, int32_t value = 0) {
    InputEvent evt;
    evt.type = type;
    evt.value = value;
    evt.timestamp = millis();
    if (xQueueSend(xInputQueue, &evt, 0) != pdTRUE) {
        inputQueueDrops++;
        Serial.printf("[WIRELESS] InputQueue voll! Event 0x%02X verworfen (drops=%lu)\n",
                      (unsigned)type, (unsigned long)inputQueueDrops);
    }
}

void handleProbeEventForCurrentMode(uint8_t evt, float luxG0, float luxG5) {
    (void)luxG0; // Lux-Daten werden nur im Flash-Handshake gebraucht
    (void)luxG5;
    
    switch (currentMode) {
        // =====================================================================
        // A) MESS-MODUS (BW & Splitgrade - Zonen-Messung)
        // Pflichtenheft 9.2 A): "Status: S3 ist im Idle, Nutzer will einmessen"
        // =====================================================================
        case MODE_BW:
        case MODE_SG:
            switch (evt) {
                case EVT_T2_CLICK:
                    // Pflichtenheft: "Taster 2 (Messen): Startet den spektralen
                    // Mess-Handshake (Grün/Blau)."
                    if (!isMeasurementActive()) {
                        triggerSpectralMeasurement();
                        Serial.println("[PROBE] T2 -> Spektralmessung gestartet");
                    }
                    break;

                case EVT_ENC_LONG:
                    // C6 Enc3 Lang soll dieselbe Start-Aktion ausloesen wie lokaler EVT_GRADE_LONG.
                    if (!isMeasurementActive()) {
                        triggerSpectralMeasurement();
                        Serial.println("[PROBE] ENC_LONG -> Spektralmessung gestartet");
                    }
                    break;
                    
                case EVT_T1_CLICK:
                    // Pflichtenheft: "Taster 1 (Undo): Löscht den zuletzt
                    // gemessenen Spot"
                    undoLastMeasurement();
                    Serial.println("[PROBE] T1 -> Letzten Spot gelöscht (Undo)");
                    break;
                    
                case EVT_ENC_UP:
                case EVT_ENC_DOWN:
                    // Pflichtenheft: "Encoder Drehen: Wechselt im SG-Modus den
                    // Fokus zwischen [ Soft Lichter ] und [ Hard Schatten ]"
                    if (currentMode == MODE_SG) {
                        currentMeasureFocus = (currentMeasureFocus == FOCUS_HIGHLIGHTS)
                            ? FOCUS_SHADOWS : FOCUS_HIGHLIGHTS;
                        sendRenderPacketToC6(); // Sofort neues Render-Paket senden
                        uiTriggerBeep(SND_NAV);
                        Serial.printf("[PROBE] ENC -> Fokus gewechselt: %s\n",
                            currentMeasureFocus == FOCUS_HIGHLIGHTS ? "LICHTER" : "SCHATTEN");
                    }
                    break;
                    
                case EVT_ENC_CLICK:
                    // Pflichtenheft: "Encoder Klick (Abschließen): Die Mess-Session
                    // wird abgeschlossen und die Werte gehen fest in den Haupt-Timer."
                    // BETA-FIX: Schritt 4 - Remote-Apply fuer BW.
                    // Wenn im BW-Modus ein Pending-Vorschlag vorliegt, wird die
                    // Dosis unter Mutex-Schutz uebernommen und unmittelbar ein
                    // haptischer Rueckkanal-Klick an das C6 gesendet.
                    // Der Rueckkanal ist nicht blockierend: Ein Funkfehler darf
                    // das S3-System nicht anhalten oder den Eventfluss blockieren.
                    if (currentMode == MODE_BW) {
                        bool didApplyPending = false;
                        if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                            if (bwAutoPending) {
                                dose_bw = target_dose;
                                bwAutoPending = false;
                                didApplyPending = true;
                            }
                            xSemaphoreGive(gTimerMutex);
                        }

                        if (didApplyPending) {
                            // BETA-FIX: Schritt 4 - Lokale Anzeige sofort nachziehen,
                            // damit LCD/Nextion den uebernommenen Dosiswert direkt zeigen.
                            refreshDisplayVariables();

                            // BETA-FIX: Schritt 4 - Haptik-Rueckkanal zum Handgeraet.
                            // haptic_feedback = 1 entspricht kurzem Klick auf dem C6,
                            // der die erfolgreiche Uebernahme direkt am Messkopf bestaetigt.
                            // Kein Einfluss auf SG, da strikt auf MODE_BW begrenzt.
                            sendProbeRender("[ BW APPLY ]", "Dose updated", "", currentZoneHistogram, 1, PMODE_METER_BW);
                        }
                    }

                    finalizeMeteringSession();
                    uiTriggerBeep(SND_DONE);
                    sendRenderPacketToC6();
                    Serial.println("[PROBE] ENC_CLICK -> Mess-Session abgeschlossen");
                    break;
                    
                default: break;
            }
            break;
            
        // =====================================================================
        // B) BURN-MODUS (Nachbelichten Remote)
        // Pflichtenheft 9.2 B): "S3 steht auf Burn Armed. User positioniert Pappe."
        // =====================================================================
        case MODE_BURN:
            switch (evt) {
                case EVT_T2_CLICK:
                    // Pflichtenheft: "Taster 2 (Feuer): Fungiert als Fernauslöser.
                    // Startet genau diesen einen Burn-Step am S3."
                    injectInputEvent(EVT_START_PRESSED);
                    Serial.println("[PROBE] T2 -> Burn Fernauslöser (START injiziert)");
                    break;
                    
                case EVT_T1_CLICK:
                    // Pflichtenheft: "Taster 1 (Abbruch): Bricht den laufenden Burn ab."
                    injectInputEvent(EVT_GRADE_PRESSED); // Enc 3 Kurz = Burn-Abbruch
                    Serial.println("[PROBE] T1 -> Burn Abbruch (GRADE injiziert)");
                    break;
                    
                case EVT_ENC_UP:
                case EVT_ENC_DOWN:
                    // Bugfix Burn-Encoder: Vorher gab es nur akustisches Feedback ohne Wirkung.
                    // Damit das C6 im Burn-Modus direkt reagiert, wird die Drehung als
                    // Encoder-Soft-Event in die normale Input-Queue injiziert. Die bestehende
                    // Burn-Logik verarbeitet EVT_ENC_SOFT bereits als Burn-EV-Aenderung.
                    injectInputEvent(EVT_ENC_SOFT, (evt == EVT_ENC_UP) ? 1 : -1);
                    uiTriggerBeep(SND_NAV);
                    break;
                    
                default: break;
            }
            break;

        case MODE_TIMER:
            switch (evt) {
                case EVT_T2_CLICK:
                    // Start/Stop ueber denselben Eventpfad wie lokale Start-Taste.
                    injectInputEvent(EVT_START_PRESSED);
                    Serial.println("[PROBE] T2 -> TIMER Start/Stop (START injiziert)");
                    break;

                case EVT_T1_CLICK:
                    // Kurz-BACK bleibt im bestehenden System der EV-Step-Toggle.
                    injectInputEvent(EVT_BACK_PRESSED);
                    Serial.println("[PROBE] T1 -> TIMER Step-Toggle (BACK injiziert)");
                    break;

                case EVT_ENC_UP:
                case EVT_ENC_DOWN:
                    // Bugfix MODE_TIMER Encoder: Direkte EV-Aenderung wie im BW-Zeitpfad.
                    // Dadurch reagiert der Modus auch dann, wenn kein spezielles Mode-Routing
                    // fuer MODE_TIMER im lokalen Input-Delegator existiert.
                    modifyExposureByEV((evt == EVT_ENC_UP) ? (1.0 / 3.0) : (-1.0 / 3.0));
                    sendRenderPacketToC6();
                    uiTriggerBeep(SND_NAV);
                    Serial.printf("[PROBE] ENC -> TIMER EV %s\n", (evt == EVT_ENC_UP) ? "+1/3" : "-1/3");
                    break;

                case EVT_ENC_CLICK:
                    // ENTER-Funktion konsistent ueber vorhandenen Queue-Pfad.
                    injectInputEvent(EVT_ENTER_PRESSED);
                    Serial.println("[PROBE] ENC_CLICK -> TIMER ENTER injiziert");
                    break;

                default:
                    break;
            }
            break;
            
        // =====================================================================
        // C) KALIBRIERUNGS-MODUS (Modus 5 - Stouffer)
        // Pflichtenheft 9.2 C): "Taster 2 (Messen): Triggert die Messung"
        // =====================================================================
        case MODE_CALIB:
            if (evt == EVT_T2_CLICK) {
                injectInputEvent(EVT_START_PRESSED);
                Serial.println("[PROBE] T2 -> Kalibrier-Messung (START injiziert)");
            }
            break;
            
        // =====================================================================
        // D) DENSITOMETER-MODUS (Modus 7)
        // Pflichtenheft 9.2 D):
        //   T1 (Referenz): "Speichert das Leerlicht als Referenz"
        //   T2 (Messen):   "Misst den Punkt durch den Film"
        // =====================================================================
        case MODE_DENS:
            if (evt == EVT_T1_CLICK) {
                // Pflichtenheft: "Taster 1 (Referenz): Leere Bühne.
                // Da dies nur selten passiert, liegt es auf dem oberen Knopf."
                injectInputEvent(EVT_START_PRESSED);
                Serial.println("[PROBE] T1 -> Densitometer Referenz (START injiziert)");
            }
            else if (evt == EVT_T2_CLICK) {
                // Pflichtenheft: "Taster 2 (Messen): Ein Klick misst den Punkt"
                injectInputEvent(EVT_ENTER_PRESSED);
                Serial.println("[PROBE] T2 -> Densitometer Messung (ENTER injiziert)");
            }
            break;
            
        default:
            break;
    }
    
    // NEXTION: Update triggern, um Statusänderung (z.B. neue Messwerte) auf Seite 9 anzuzeigen
    updateNextionUI(true); 
}