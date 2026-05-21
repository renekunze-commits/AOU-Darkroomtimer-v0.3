/*
 * EspLinkRuntimeStatus
 *
 * Version:
 * - Implementierungsschritt 5 der Inter-MCU-Basis
 * - eingefuehrt am 2026-04-24
 * - Schema-Version 1 fuer den sichtbaren ESP32-S3-Linkstatus am Teensy
 *
 * Zweck:
 * - macht den nicht-sicherheitskritischen Servicepfad zwischen ESP32-S3 und
 *   Teensy als eigene Laufzeitschicht sichtbar
 * - schafft einen festen Platz fuer Heartbeat, Encoder-4-Ereignisse,
 *   thermische Servicewerte und Wireless-2591-Gatewaystatus
 *
 * Hardwarebezug:
 * - die Board-zu-Board-Kommunikation laeuft ueber S3_TX/S3_RX
 * - der OneWire-/DS18B20-Master sitzt laut Schaltplan bewusst am ESP32-S3
 * - der Teensy konsumiert diese Thermikdaten deshalb als Servicepfad und nicht
 *   als lokalen OneWire-Bus
 */
#pragma once

#include <stdint.h>

#include <DukatimerProtocol.h>

namespace dukatimer {

constexpr uint16_t kEspLinkRuntimeStatusSchemaVersion = 9;

// Gesamtgesundheit des Service-Links aus Sicht des Teensy-Hauptsystems.
enum class EspLinkHealth : uint8_t {
	Unknown,
	Online,
	Stale,
	Lost,
};

// Sichtbarer Status der seriellen VFS-Bruecke. Erfasst Transferzustand und die
// letzte Rueckmeldung, nicht aber den eigentlichen Dateiinhalt.
struct VfsBridgeStatus {
	dukatimer::protocol::VfsStatusCode lastStatus = dukatimer::protocol::VfsStatusCode::None;
	bool transferActive = false;
	bool realtimeHoldActive = false;
	uint32_t transactionId = 0;
	uint32_t transferredBytes = 0;

	bool operator==(const VfsBridgeStatus& other) const {
		return lastStatus == other.lastStatus && transferActive == other.transferActive &&
		       realtimeHoldActive == other.realtimeHoldActive && transactionId == other.transactionId &&
		       transferredBytes == other.transferredBytes;
	}

	bool operator!=(const VfsBridgeStatus& other) const {
		return !(*this == other);
	}
};

// Letztes vom ESP geliefertes Remote-Eingabeereignis in sichtbarer Form.
struct EspRemoteInputStatus {
	dukatimer::protocol::RemoteInputSource source = dukatimer::protocol::RemoteInputSource::None;
	dukatimer::protocol::InputEventKind eventKind = dukatimer::protocol::InputEventKind::None;
	int16_t value = 0;
	uint32_t eventSequence = 0;
	uint32_t sourceTimestampMs = 0;
	bool pending = false;

	bool operator==(const EspRemoteInputStatus& other) const {
		return source == other.source && eventKind == other.eventKind && value == other.value &&
		       eventSequence == other.eventSequence && sourceTimestampMs == other.sourceTimestampMs &&
		       pending == other.pending;
	}

	bool operator!=(const EspRemoteInputStatus& other) const {
		return !(*this == other);
	}
};

// Service-Sensorwerte, die nicht direkt lokal am Teensy erhoben werden.
struct EspServiceSensorStatus {
	uint16_t sensorFlags = dukatimer::protocol::kServiceSensorNone;
	float ds18TemperatureCelsius = 0.0f;
	uint32_t ds18SampleAgeMs = 0;
	float ahtTemperatureCelsius = 0.0f;
	float ahtHumidityPercent = 0.0f;
	float bmpTemperatureCelsius = 0.0f;
	float bmpPressureHpa = 0.0f;

	bool operator==(const EspServiceSensorStatus& other) const {
		return sensorFlags == other.sensorFlags && ds18TemperatureCelsius == other.ds18TemperatureCelsius &&
		       ds18SampleAgeMs == other.ds18SampleAgeMs && ahtTemperatureCelsius == other.ahtTemperatureCelsius &&
		       ahtHumidityPercent == other.ahtHumidityPercent &&
		       bmpTemperatureCelsius == other.bmpTemperatureCelsius &&
		       bmpPressureHpa == other.bmpPressureHpa;
	}

	bool operator!=(const EspServiceSensorStatus& other) const {
		return !(*this == other);
	}
};

// Kompakte Recovery-Zusammenfassung fuer den vom ESP beobachteten PaperSlot-
// Produktdatenpfad. Der Teensy nutzt sie nur fuer Sichtbarkeit und explizite
// Nutzerhinweise, nicht fuer automatische Datenentscheidungen.
struct PaperSlotRecoveryStatus {
	uint16_t flags = dukatimer::protocol::kPaperSlotRecoveryNone;
	dukatimer::protocol::PaperSlotRecoveryRecommendation recommendation =
		dukatimer::protocol::PaperSlotRecoveryRecommendation::Unknown;
	dukatimer::protocol::VfsStatusCode activeVfsStatus = dukatimer::protocol::VfsStatusCode::None;
	uint8_t activeParseError = 0;
	dukatimer::protocol::VfsStatusCode backupVfsStatus = dukatimer::protocol::VfsStatusCode::None;
	uint8_t backupParseError = 0;

	bool operator==(const PaperSlotRecoveryStatus& other) const {
		return flags == other.flags && recommendation == other.recommendation &&
		       activeVfsStatus == other.activeVfsStatus && activeParseError == other.activeParseError &&
		       backupVfsStatus == other.backupVfsStatus && backupParseError == other.backupParseError;
	}

	bool operator!=(const PaperSlotRecoveryStatus& other) const {
		return !(*this == other);
	}
};

// Sichtbarer Zustand des Wireless-Messpeer-/Gatewaypfads.
struct WirelessGatewayStatus {
	dukatimer::protocol::WirelessPeerState peerState = dukatimer::protocol::WirelessPeerState::Unknown;
	uint8_t batteryPercent = 0;
	uint16_t flags = dukatimer::protocol::kWirelessSnapshotNone;
	uint32_t lastSeenAgeMs = 0;
	uint32_t sensorSampleAgeMs = 0;
	float lastLux = 0.0f;
	uint32_t measurementSequence = 0;
	uint32_t commandAckSequence = 0;
	uint16_t renderStatusFlags = dukatimer::protocol::kWirelessRenderStatusNone;
	uint32_t staleRenderCount = 0;
	uint32_t renderTimeoutCount = 0;

	bool operator==(const WirelessGatewayStatus& other) const {
		return peerState == other.peerState && batteryPercent == other.batteryPercent && flags == other.flags &&
		       lastSeenAgeMs == other.lastSeenAgeMs &&
		       sensorSampleAgeMs == other.sensorSampleAgeMs && lastLux == other.lastLux &&
		       measurementSequence == other.measurementSequence &&
		       commandAckSequence == other.commandAckSequence &&
		       renderStatusFlags == other.renderStatusFlags &&
		       staleRenderCount == other.staleRenderCount &&
		       renderTimeoutCount == other.renderTimeoutCount;
	}

	bool operator!=(const WirelessGatewayStatus& other) const {
		return !(*this == other);
	}
};

// Letzte explizite Diagnosemeldung aus dem ESP-Servicepfad.
struct EspDiagnosticStatus {
	dukatimer::protocol::DiagnosticCode code = dukatimer::protocol::DiagnosticCode::None;
	uint16_t detail = 0;
	uint32_t counter = 0;
	uint32_t timestampMs = 0;

	bool operator==(const EspDiagnosticStatus& other) const {
		return code == other.code && detail == other.detail && counter == other.counter &&
		       timestampMs == other.timestampMs;
	}

	bool operator!=(const EspDiagnosticStatus& other) const {
		return !(*this == other);
	}
};

// Sichtbarer Zustand des Teensy-seitigen UART-Ausgangspuffers. Damit bleibt
// nachvollziehbar, ob Safety-relevante Frames nur priorisiert werden mussten
// oder ob bereits echte Drops/Coalescing aufgetreten sind.
struct EspTxQueueStatus {
	uint8_t capacity = 0;
	uint8_t pendingFrameCount = 0;
	uint8_t maxObservedPendingFrameCount = 0;
	uint32_t evictedFrameCount = 0;
	uint32_t droppedHeartbeatCount = 0;
	uint32_t droppedRenderCount = 0;
	uint32_t droppedDiagnosticCount = 0;
	uint32_t droppedCommandCount = 0;
	uint32_t droppedVfsCount = 0;
	uint32_t coalescedRenderCount = 0;
	uint32_t coalescedDiagnosticCount = 0;

	bool operator==(const EspTxQueueStatus& other) const {
		return capacity == other.capacity && pendingFrameCount == other.pendingFrameCount &&
		       maxObservedPendingFrameCount == other.maxObservedPendingFrameCount &&
		       evictedFrameCount == other.evictedFrameCount &&
		       droppedHeartbeatCount == other.droppedHeartbeatCount &&
		       droppedRenderCount == other.droppedRenderCount &&
		       droppedDiagnosticCount == other.droppedDiagnosticCount &&
		       droppedCommandCount == other.droppedCommandCount &&
		       droppedVfsCount == other.droppedVfsCount &&
		       coalescedRenderCount == other.coalescedRenderCount &&
		       coalescedDiagnosticCount == other.coalescedDiagnosticCount;
	}

	bool operator!=(const EspTxQueueStatus& other) const {
		return !(*this == other);
	}
};

// Oberer Sammelzustand des kompletten ESP-Servicepfads fuer Snapshot und UI.
struct EspLinkRuntimeStatus {
	uint16_t schemaVersion = kEspLinkRuntimeStatusSchemaVersion;
	EspLinkHealth health = EspLinkHealth::Unknown;
	bool heartbeatSeen = false;
	uint32_t lastRxAgeMs = 0;
	uint32_t lastTxAgeMs = 0;
	uint32_t remoteUptimeMs = 0;
	uint32_t remoteCapabilityBits = 0;
	bool remoteFileTransactionActive = false;
	EspServiceSensorStatus serviceSensors = {};
	PaperSlotRecoveryStatus paperSlotRecovery = {};
	WirelessGatewayStatus wireless = {};
	EspRemoteInputStatus lastInputEvent = {};
	EspDiagnosticStatus diagnostic = {};
	EspTxQueueStatus txQueue = {};
	VfsBridgeStatus vfs = {};

	bool operator==(const EspLinkRuntimeStatus& other) const {
		return schemaVersion == other.schemaVersion && health == other.health &&
		       heartbeatSeen == other.heartbeatSeen && lastRxAgeMs == other.lastRxAgeMs &&
		       lastTxAgeMs == other.lastTxAgeMs && remoteUptimeMs == other.remoteUptimeMs &&
		       remoteCapabilityBits == other.remoteCapabilityBits &&
		       remoteFileTransactionActive == other.remoteFileTransactionActive &&
		       serviceSensors == other.serviceSensors &&
		       paperSlotRecovery == other.paperSlotRecovery && wireless == other.wireless &&
		       lastInputEvent == other.lastInputEvent && diagnostic == other.diagnostic &&
		       txQueue == other.txQueue &&
		       vfs == other.vfs;
	}

	bool operator!=(const EspLinkRuntimeStatus& other) const {
		return !(*this == other);
	}
};

inline EspLinkRuntimeStatus makeUnknownEspLinkRuntimeStatus() {
	return EspLinkRuntimeStatus{};
}

}  // namespace dukatimer