#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_TSL2591.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

#include "DukatimerProtocol.h"
#include "config.h"
#include "input.h"
#include "output.h"

using dukatimer::protocol::RemoteDisplayPayload;
using dukatimer::protocol::DiagnosticPayload;
using dukatimer::protocol::RemoteHapticFeedback;
using dukatimer::protocol::TeensyCommandKind;
using dukatimer::protocol::TeensyCommandPayload;
using dukatimer::protocol::WirelessRemotePayload;

// =============================================================================
// GLOBALE OBJEKTE, MUTEX UND STATUS
// =============================================================================
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
SemaphoreHandle_t xI2CMutex = NULL; // Schutz für alle I2C-Operationen
static portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

WirelessRemotePayload outPayload = {};
RemoteDisplayPayload inPayload = {};
DiagnosticPayload pendingDiagnostic = {};
TeensyCommandPayload pendingCommand = {};
volatile uint32_t lastRemoteRxMs = 0;
static volatile uint32_t lastRemoteCommandMs = 0;
static volatile bool remoteDisplayDirty = false;
static volatile bool remoteDiagnosticPending = false;
static volatile bool remoteCommandPending = false;
static WirelessRemotePayload lastSentPayload = {};
static bool sentPayloadInitialized = false;
static uint32_t lastSendAttemptMs = 0;
static uint32_t lastSuccessfulTxMs = 0;
static uint32_t lastAcceptedCommandSequence = 0;
static uint32_t lastAppliedCommandSequence = 0;
static uint32_t lastAcceptedRenderSequence = 0;
static volatile uint32_t staleRenderDropCount = 0;
static volatile uint32_t renderTimeoutCount = 0;
static bool renderTimeoutActive = false;

static volatile uint32_t txFailCount = 0;
static volatile bool lastTxSuccess = true;
static unsigned long lastDisplayUpdateMs = 0;
static constexpr uint32_t SENSOR_POLL_INTERVAL = 800;
static constexpr uint32_t REMOTE_COMMAND_TIMEOUT_MS = REMOTE_SLAVE_TIMEOUT_MS;

bool tslAvailable = false;
static tsl2591Gain_t currentGain = TSL2591_GAIN_MED;
static bool remoteMeasurementActive = false;
static bool remoteMeasurementHardPhase = false;

namespace {

bool receiverAddressIsBroadcast() {
    for (const uint8_t octet : REMOTE_RECEIVER_MAC) {
        if (octet != 0xFFu) {
            return false;
        }
    }

    return true;
}

bool senderMatchesReceiver(const uint8_t* senderMac) {
    return receiverAddressIsBroadcast() ||
           (senderMac != nullptr && memcmp(senderMac, REMOTE_RECEIVER_MAC, sizeof(REMOTE_RECEIVER_MAC)) == 0);
}

bool isSequenceNewer(uint32_t sequence, uint32_t reference) {
    if (sequence == 0u) {
        return false;
    }

    return static_cast<int32_t>(sequence - reference) > 0;
}

bool payloadNeedsSend(const WirelessRemotePayload& payload, bool keepAliveDue) {
    if (!sentPayloadInitialized) {
        return true;
    }

    return payload.encoderDelta != 0 || payload.activeButtons != lastSentPayload.activeButtons ||
           fabsf(payload.activeLux - lastSentPayload.activeLux) >= 0.01f ||
           payload.activeLuxAgeMs < lastSentPayload.activeLuxAgeMs ||
           payload.commandAckSequence != lastSentPayload.commandAckSequence ||
           payload.renderStatusFlags != lastSentPayload.renderStatusFlags ||
           payload.staleRenderCount != lastSentPayload.staleRenderCount ||
           payload.renderTimeoutCount != lastSentPayload.renderTimeoutCount || keepAliveDue;
}

RemoteDisplayPayload copyIncomingDisplayPayload(bool* dirtyOut = nullptr) {
    RemoteDisplayPayload payload;
    portENTER_CRITICAL(&packetMux);
    payload = inPayload;
    if (dirtyOut != nullptr) {
        *dirtyOut = remoteDisplayDirty;
    }
    remoteDisplayDirty = false;
    portEXIT_CRITICAL(&packetMux);
    return payload;
}

bool copyPendingCommand(TeensyCommandPayload* commandOut) {
    if (commandOut == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&packetMux);
    if (!remoteCommandPending) {
        portEXIT_CRITICAL(&packetMux);
        return false;
    }

    *commandOut = pendingCommand;
    pendingCommand = {};
    remoteCommandPending = false;
    portEXIT_CRITICAL(&packetMux);
    return true;
}

bool copyPendingDiagnostic(DiagnosticPayload* diagnosticOut) {
    if (diagnosticOut == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&packetMux);
    if (!remoteDiagnosticPending) {
        portEXIT_CRITICAL(&packetMux);
        return false;
    }

    *diagnosticOut = pendingDiagnostic;
    pendingDiagnostic = {};
    remoteDiagnosticPending = false;
    portEXIT_CRITICAL(&packetMux);
    return true;
}

void noteStaleRenderDrop(uint32_t sequence, uint32_t reference) {
    uint32_t total = 0;
    portENTER_CRITICAL(&packetMux);
    staleRenderDropCount = staleRenderDropCount + 1u;
    total = staleRenderDropCount;
    portEXIT_CRITICAL(&packetMux);
    Serial.printf("[C6 RENDER] stale seq=%lu ref=%lu total=%lu\n",
                 static_cast<unsigned long>(sequence),
                 static_cast<unsigned long>(reference),
                 static_cast<unsigned long>(total));
}

void updateRenderTimeoutState(bool timeoutActiveNow) {
    bool timeoutStarted = false;
    uint32_t total = 0;
    portENTER_CRITICAL(&packetMux);
    if (timeoutActiveNow && !renderTimeoutActive) {
        renderTimeoutCount = renderTimeoutCount + 1u;
        timeoutStarted = true;
    }
    renderTimeoutActive = timeoutActiveNow;
    total = renderTimeoutCount;
    portEXIT_CRITICAL(&packetMux);
    if (timeoutStarted) {
        Serial.printf("[C6 RENDER] timeout total=%lu\n", static_cast<unsigned long>(total));
    }
}

void copyRenderDiagnostics(uint16_t* renderStatusFlagsOut,
                         uint32_t* staleRenderCountOut,
                         uint32_t* renderTimeoutCountOut,
                         bool* renderTimeoutActiveOut) {
    portENTER_CRITICAL(&packetMux);
    if (renderStatusFlagsOut != nullptr) {
        *renderStatusFlagsOut = renderTimeoutActive ? dukatimer::protocol::kWirelessRenderStatusTimeoutActive
                                                   : dukatimer::protocol::kWirelessRenderStatusNone;
    }
    if (staleRenderCountOut != nullptr) {
        *staleRenderCountOut = staleRenderDropCount;
    }
    if (renderTimeoutCountOut != nullptr) {
        *renderTimeoutCountOut = renderTimeoutCount;
    }
    if (renderTimeoutActiveOut != nullptr) {
        *renderTimeoutActiveOut = renderTimeoutActive;
    }
    portEXIT_CRITICAL(&packetMux);
}

void queueIncomingCommand(const TeensyCommandPayload& command, uint32_t receivedAtMs) {
    portENTER_CRITICAL(&packetMux);
    if (!isSequenceNewer(command.commandSequence, lastAcceptedCommandSequence) &&
        command.commandSequence != 0u) {
        portEXIT_CRITICAL(&packetMux);
        return;
    }

    // Kommandos werden nur gequeued und spaeter im Loop verarbeitet. Dadurch
    // bleibt der ESP-NOW-Callback kurz, und der lokale Mess-Task laeuft ohne
    // blockierende Remote-Nebenpfade weiter.
    pendingCommand = command;
    remoteCommandPending = true;
    lastAcceptedCommandSequence = command.commandSequence;
    lastRemoteCommandMs = receivedAtMs;
    portEXIT_CRITICAL(&packetMux);
}

void queueIncomingDiagnostic(const DiagnosticPayload& diagnostic) {
    portENTER_CRITICAL(&packetMux);
    pendingDiagnostic = diagnostic;
    remoteDiagnosticPending = true;
    portEXIT_CRITICAL(&packetMux);
}

void applyPendingRemoteCommand(const TeensyCommandPayload& command, uint32_t nowMs) {
    wakeDisplay();
    bool commandApplied = false;

    switch (static_cast<TeensyCommandKind>(command.commandKind)) {
        case TeensyCommandKind::RemoteMeasurementStart:
            // Das C6 bleibt Mess-Terminal mit lokaler Sensorakquise. Der Remote-
            // Start schaltet deshalb nur den sichtbaren Kontext um, nicht den
            // Mess-Task selbst.
            remoteMeasurementActive = true;
            remoteMeasurementHardPhase = command.argument0 != 0u;
            lastRemoteCommandMs = nowMs;
            commandApplied = true;
            break;

        case TeensyCommandKind::RemoteMeasurementCancel:
            remoteMeasurementActive = false;
            remoteMeasurementHardPhase = false;
            lastRemoteCommandMs = nowMs;
            commandApplied = true;
            break;

        case TeensyCommandKind::RemoteHaptic: {
            RemoteHapticFeedback feedback = RemoteHapticFeedback::None;
            switch (static_cast<RemoteHapticFeedback>(command.argument0)) {
                case RemoteHapticFeedback::Click:
                case RemoteHapticFeedback::Error:
                case RemoteHapticFeedback::Done:
                case RemoteHapticFeedback::None:
                    feedback = static_cast<RemoteHapticFeedback>(command.argument0);
                    break;
            }

            triggerHaptic(feedback);
            lastRemoteCommandMs = nowMs;
            commandApplied = true;
            break;
        }

        case TeensyCommandKind::Ping:
            lastRemoteCommandMs = nowMs;
            commandApplied = true;
            break;

        case TeensyCommandKind::RemoteRender:
        case TeensyCommandKind::None:
            break;
    }

    if (commandApplied && command.commandSequence != 0u) {
        // Das Ack darf erst nach der echten Loop-Verarbeitung steigen. So
        // bestaetigt der Rueckkanal Ausfuehrung statt blosser Funkannahme.
        lastAppliedCommandSequence = command.commandSequence;
    }
}

void serviceRemoteCommandState(uint32_t nowMs) {
    TeensyCommandPayload command = {};
    if (copyPendingCommand(&command)) {
        applyPendingRemoteCommand(command, nowMs);
    }

    DiagnosticPayload diagnostic = {};
    if (copyPendingDiagnostic(&diagnostic)) {
        observeRemoteDiagnostic(diagnostic);
        Serial.printf("[C6 DIAG] code=%u detail=%u counter=%lu\n",
                     static_cast<unsigned>(diagnostic.code),
                     static_cast<unsigned>(diagnostic.detail),
                     static_cast<unsigned long>(diagnostic.counter));
    }

    const bool slaveModeActive = lastRemoteRxMs > 0u && (nowMs - lastRemoteRxMs) < REMOTE_SLAVE_TIMEOUT_MS;
    const bool commandStillFresh =
        lastRemoteCommandMs > 0u && (nowMs - lastRemoteCommandMs) < REMOTE_COMMAND_TIMEOUT_MS;
    if (!slaveModeActive && !commandStillFresh) {
        remoteMeasurementActive = false;
        remoteMeasurementHardPhase = false;
    }
}

void sendRemoteStateIfDue(uint32_t nowMs) {
    const bool sendWindowDue = !sentPayloadInitialized || (nowMs - lastSendAttemptMs) >= REMOTE_SEND_INTERVAL_MS;
    const bool keepAliveDue = !sentPayloadInitialized || (nowMs - lastSuccessfulTxMs) >= REMOTE_KEEPALIVE_MS;
    if (!sendWindowDue && !keepAliveDue) {
        return;
    }

    WirelessRemotePayload payload = {};
    payload.encoderDelta = getAndClearEncoderDelta();
    payload.activeButtons = getActiveButtonMask();
    payload.activeLux = getLuxValue();
    payload.activeLuxAgeMs = getLuxSampleAgeMs(nowMs);
    payload.commandAckSequence = lastAppliedCommandSequence;
    copyRenderDiagnostics(&payload.renderStatusFlags, &payload.staleRenderCount,
                         &payload.renderTimeoutCount, nullptr);
    if (!payloadNeedsSend(payload, keepAliveDue)) {
        if (payload.encoderDelta != 0) {
            restoreEncoderDelta(payload.encoderDelta);
        }
        return;
    }

    payload.sequenceNumber = outPayload.sequenceNumber + 1u;
    lastSendAttemptMs = nowMs;
    const esp_err_t result =
        esp_now_send(REMOTE_RECEIVER_MAC, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
    if (result != ESP_OK) {
        if (payload.encoderDelta != 0) {
            restoreEncoderDelta(payload.encoderDelta);
        }
        txFailCount = txFailCount + 1u;
        Serial.printf("[C6 TX] send failed err=%d total=%lu\n", static_cast<int>(result),
                     static_cast<unsigned long>(txFailCount));
        return;
    }

    outPayload = payload;
    lastSentPayload = payload;
    lastSentPayload.encoderDelta = 0;
    sentPayloadInitialized = true;
    lastSuccessfulTxMs = nowMs;
}

}  // namespace

// =============================================================================
// ESP-NOW CALLBACKS
// =============================================================================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void OnDataSentC6(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
static void OnDataSentC6(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
    lastTxSuccess = (status == ESP_NOW_SEND_SUCCESS);
    if (status != ESP_NOW_SEND_SUCCESS) {
        txFailCount = txFailCount + 1u;
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    const uint8_t *senderMAC = info != nullptr ? info->src_addr : nullptr;
#else
    const uint8_t *senderMAC = mac_addr;
#endif
    if (data == nullptr || !senderMatchesReceiver(senderMAC)) {
        return;
    }

    const uint32_t receivedAtMs = millis();

    if (len == static_cast<int>(sizeof(RemoteDisplayPayload))) {
        RemoteDisplayPayload payload = {};
        memcpy(&payload, data, sizeof(payload));
        // Render-Frische wird direkt am C6 begrenzt. Veraltete oder replayte
        // Bildpakete duerfen weder das sichtbare OLED noch den Slave-Timeout
        // verlaengern.
        if (!isSequenceNewer(payload.sequenceNumber, lastAcceptedRenderSequence)) {
            noteStaleRenderDrop(payload.sequenceNumber, lastAcceptedRenderSequence);
            return;
        }

        portENTER_CRITICAL(&packetMux);
        inPayload = payload;
        lastRemoteRxMs = receivedAtMs;
        lastAcceptedRenderSequence = payload.sequenceNumber;
        remoteDisplayDirty = true;
        portEXIT_CRITICAL(&packetMux);
        return;
    }

    if (len == static_cast<int>(sizeof(TeensyCommandPayload))) {
        TeensyCommandPayload command = {};
        memcpy(&command, data, sizeof(command));
        queueIncomingCommand(command, receivedAtMs);
        return;
    }

    if (len == static_cast<int>(sizeof(DiagnosticPayload))) {
        DiagnosticPayload diagnostic = {};
        memcpy(&diagnostic, data, sizeof(diagnostic));
        queueIncomingDiagnostic(diagnostic);
        return;
    }
}

// =============================================================================
// I2C RECOVERY & SENSOR ACCESS
// =============================================================================
void recoverI2CBus() {
    if (xI2CMutex != NULL) {
        // I2C-Recovery setzt die Leitung rückstandsfrei zurück und braucht dazu exklusiven Zugriff
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    }
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
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (xI2CMutex != NULL) {
        xSemaphoreGive(xI2CMutex);
    }
}

bool initTSL2591() {
    bool res = false;
    if (xI2CMutex != NULL) {
        // Der Sensor muss exklusiv konfiguriert werden, damit Gain- und Timing-Register konsistent bleiben
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    }
    res = tsl.begin();
    if (res) {
        currentGain = TSL2591_GAIN_MED;
        tsl.setGain(currentGain);
        tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);
    }
    if (xI2CMutex != NULL) {
        xSemaphoreGive(xI2CMutex);
    }
    return res;
}

float measureLux() {
    if (!tslAvailable) return -1.0f;
    uint32_t lum = 0;
    if (xI2CMutex == NULL) return -1.0f;
    // Die volle Messintegration darf nicht durch parallele I2C-Zugriffe gestört werden
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(800)) != pdTRUE) return -1.0f;
    lum = tsl.getFullLuminosity();
    xSemaphoreGive(xI2CMutex);

    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;

    if (full > 36000 || ir > 36000) {
        if (currentGain > TSL2591_GAIN_LOW) {
            currentGain = (tsl2591Gain_t)((int)currentGain - 0x10);
            if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Gain-Reduktion benötigt kurzfristigen exklusiven Zugriff auf den Bus
                tsl.setGain(currentGain);
                xSemaphoreGive(xI2CMutex);
            }
            return -1.0f;
        }
    }
    if (full < 300 && full > 0 && currentGain < TSL2591_GAIN_MAX) {
        currentGain = (tsl2591Gain_t)((int)currentGain + 0x10);
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Verstärkungserhöhung braucht ebenfalls kurzfristige Bus-Sperre
            tsl.setGain(currentGain);
            xSemaphoreGive(xI2CMutex);
        }
        return -1.0f;
    }
    float lux = tsl.calculateLux(full, ir);
    if (isnan(lux) || isinf(lux) || lux < 0.0f) return 0.0f;
    return lux;
}

void measurementTask(void *pvParameters) {
    for (;;) {
		if (tslAvailable) {
			float lux = measureLux();
			if (lux >= 0.0f) {
                publishLuxValue(lux, millis());
			}
		}
		vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL));
	}
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    xI2CMutex = xSemaphoreCreateMutex();
    recoverI2CBus();

    initOutput();
    showStartup();

    byte error, address;
    int nDevices = 0;
    bool tslFound = false;
    if (xSemaphoreTake(xI2CMutex, portMAX_DELAY) == pdTRUE) {
        // Der I2C-Scan darf nicht neben anderen I2C-Zugriffen laufen
        for (address = 1; address < 127; address++ ) {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0) {
                nDevices++;
                if (address == 0x29) tslFound = true;
            }
        }
        xSemaphoreGive(xI2CMutex);
    }

    if (nDevices == 0 || !tslFound) {
        showError(nDevices == 0 ? "No I2C HW!" : "TSL missing!");
        while (1) {
            digitalWrite(PIN_VIB_MOTOR, HIGH);
            delay(100);
            digitalWrite(PIN_VIB_MOTOR, LOW);
            delay(500);
        }
    }

    initInput();
    tslAvailable = initTSL2591();
    publishLuxValue(0.0f);

    if (tslAvailable) {
        xTaskCreate(measurementTask, "meas", 4096, NULL, 1, NULL);
    }

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        showError("ESP-NOW Fail");
        delay(2000);
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSentC6);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, REMOTE_RECEIVER_MAC, sizeof(REMOTE_RECEIVER_MAC));
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    const uint32_t now = millis();

    serviceRemoteCommandState(now);
	sendRemoteStateIfDue(now);

	const bool slaveModeActive = lastRemoteRxMs > 0u && (now - lastRemoteRxMs) < REMOTE_SLAVE_TIMEOUT_MS;
    updateRenderTimeoutState(lastAcceptedRenderSequence != 0u && !slaveModeActive);
    uint32_t localStaleRenderCount = 0;
    uint32_t localRenderTimeoutCount = 0;
    bool localRenderTimeoutActive = false;
    copyRenderDiagnostics(nullptr, &localStaleRenderCount, &localRenderTimeoutCount, &localRenderTimeoutActive);
    observeLocalRenderDiagnostics(localStaleRenderCount, localRenderTimeoutCount, localRenderTimeoutActive);
	if (slaveModeActive) {
		bool dirty = false;
		const RemoteDisplayPayload payload = copyIncomingDisplayPayload(&dirty);
		if (dirty || (now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
			renderSlaveMode(payload);
			lastDisplayUpdateMs = now;
		}
    } else if (remoteMeasurementActive && (now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
        renderRemoteMeasurementMode(getLuxValue(), remoteMeasurementHardPhase);
        lastDisplayUpdateMs = now;
    } else if (lastAcceptedRenderSequence != 0u && (now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
        // Nach einer echten Remote-Session zeigt das Terminal bei Render-Timeout
        // bewusst einen Offline-Fallback statt still den letzten Slave-Screen zu
        // konservieren oder unmarkiert in lokale Standalone-Anzeige zu kippen.
        renderRemoteOfflineMode(getLuxValue());
        lastDisplayUpdateMs = now;
	} else if ((now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
		renderStandAloneMode(getLuxValue());
		lastDisplayUpdateMs = now;
	}

    serviceOutput();
	checkDisplaySleep();
	delay(5);
}