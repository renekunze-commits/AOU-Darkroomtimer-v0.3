/*
 * TeensyLinkService
 *
 * Version:
 * - Implementierungsschritt 7 der Inter-MCU-Basis
 * - eingefuehrt am 2026-04-24
 * - API-Version 1 fuer den ESP32-S3-Servicepfad zum Teensy
 *
 * Zweck:
 * - sendet Heartbeat, Service-Sensorwerte, Encoder-4-Ereignisse und
 *   Wireless-2591-Gatewaystatus an den Teensy
 * - empfaengt vorbereitete Teensy-Kommandos fuer spaetere Remote- und
 *   Servicefunktionen, ohne diese schon fachlich auszuwerten
 */
#pragma once

#include <Arduino.h>

#include <DukatimerProtocol.h>

#include "DukaEspServiceBoardPins.h"

namespace dukatimer {

struct TeensyLinkPendingInputBucket {
	protocol::RemoteInputSource source = protocol::RemoteInputSource::None;
	int16_t rotationDelta = 0;
	uint8_t pressCount = 0;
	uint8_t longPressCount = 0;
	uint8_t measureCount = 0;
	uint8_t undoCount = 0;
	uint32_t latestSequence = 0;
	uint32_t latestTimestampMs = 0;

	bool hasPending() const {
		return rotationDelta != 0 || pressCount != 0u || longPressCount != 0u ||
		       measureCount != 0u || undoCount != 0u;
	}

	void clear() {
		source = protocol::RemoteInputSource::None;
		rotationDelta = 0;
		pressCount = 0;
		longPressCount = 0;
		measureCount = 0;
		undoCount = 0;
		latestSequence = 0;
		latestTimestampMs = 0;
	}
};

/*
 * TeensyLinkService
 *
 * Zweck:
 * - bildet den ESP32-seitigen Gegenpart zum Teensy-EspServiceLink
 * - publiziert Heartbeat, Service-Snapshots, Remote-Input und Wireless-Status
 *   in Richtung Teensy
 * - kapselt die transaktionale Upload-Seite der seriellen VFS-Bruecke
 *
 * Architekturgrenze:
 * - der Dienst kennt das SharedProtocol und den seriellen Transport
 * - HTTP, Sensorquellen und spaetere Funklogik bleiben ausserhalb und speisen
 *   nur Daten oder Upload-Anfragen ein
 */
class TeensyLinkService {
public:
	explicit TeensyLinkService(HardwareSerial& serialPort);

	// begin()/tick() bilden den Lebenszyklus des nicht-blockierenden Servicepfads.
	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs);

	// Datenquellen fuer den autoritativen ESP-Servicezustand.
	void setDs18b20Sample(float temperatureCelsius, bool present, bool fault, bool oneWireOnline,
	                    uint32_t sampleTimestampMs = 0);
	void setEnvironmentSample(float ahtTemperatureCelsius,
	                       float ahtHumidityPercent,
	                       bool ahtPresent,
	                       bool ahtFault,
	                       float bmpTemperatureCelsius,
	                       float bmpPressureHpa,
	                       bool bmpPresent,
	                       bool bmpFault);
	void setWirelessPeerState(dukatimer::protocol::WirelessPeerState peerState, uint8_t batteryPercent,
	                       bool luxValid, float lastLux, uint32_t measurementSequence,
	                       uint32_t commandAckSequence, uint16_t renderStatusFlags,
	                       uint32_t staleRenderCount, uint32_t renderTimeoutCount,
	                       uint32_t lastSeenTimestampMs = 0,
	                       uint32_t lastLuxSampleTimestampMs = 0);
	void setPaperSlotRecoveryStatus(uint16_t flags,
	                             dukatimer::protocol::PaperSlotRecoveryRecommendation recommendation,
	                             dukatimer::protocol::VfsStatusCode activeVfsStatus,
	                             uint8_t activeParseError,
	                             dukatimer::protocol::VfsStatusCode backupVfsStatus,
	                             uint8_t backupParseError);
	void queueDiagnostic(dukatimer::protocol::DiagnosticCode code, uint16_t detail = 0,
	                  uint32_t timestampMs = 0);
	void queueInputEvent(dukatimer::protocol::RemoteInputSource source,
	                   dukatimer::protocol::InputEventKind eventKind, int16_t value,
	                   uint32_t sourceTimestampMs = 0);
	bool consumePendingCommand(dukatimer::protocol::TeensyCommandPayload* payloadOut);
	bool consumePendingRemoteDiagnostic(dukatimer::protocol::DiagnosticPayload* payloadOut);
	bool consumePendingRemoteRender(dukatimer::protocol::RemoteDisplayPayload* payloadOut);
	void setFileTransactionActive(bool active, uint32_t nowMs = 0);
	// Einfache Upload-API fuer die HTTP-VFS-Bruecke oberhalb des SharedProtocol.
	bool beginFileUpload(const char* path);
	bool streamFileChunk(const uint8_t* data, size_t length, bool finalize = false);
	bool finishFileUpload();
	bool abortFileUpload();
	// FileReadRequest bleibt bewusst chunkweise und request/response-basiert.
	// Dadurch kann der erste Exportpfad feste Produktdaten lesen, ohne eine zweite
	// langlaufende Stream-Session mit eigener Flow-Control einzufuehren.
	bool beginFileDownload(const char* path, uint32_t requestedOffset = 0u,
	                    uint16_t requestedLength = 0u);
	bool downloadInProgress() const;
	bool downloadCompletedSuccessfully() const;
	bool downloadFaulted() const;
	bool downloadChunkPending() const;
	bool consumeDownloadedChunk(dukatimer::protocol::VfsChunkPayload* payloadOut);
	uint32_t downloadFileSize() const;
	bool uploadReadyForNextChunk() const;
	bool uploadInProgress() const;
	bool uploadCompletedSuccessfully() const;
	bool uploadFaulted() const;
	uint32_t uploadedByteCount() const;
	bool teensyExposureActive(uint32_t nowMs) const;

	// Letzte empfangene Antworten bzw. Teensy-Kommandos fuer beobachtende Aufrufer.
	const dukatimer::protocol::TeensyCommandPayload& lastCommand() const;
	const dukatimer::protocol::VfsAckPayload& lastVfsAck() const;
	const dukatimer::protocol::VfsErrorPayload& lastVfsError() const;

private:
	enum class UploadPhase : uint8_t {
		Idle,
		WaitingOpenAck,
		Streaming,
		WaitingChunkAck,
		Completed,
		Fault,
	};

	enum class DownloadPhase : uint8_t {
		Idle,
		WaitingOpenAck,
		WaitingChunk,
		Completed,
		Fault,
	};

	static constexpr uint32_t kBaudRate = 1000000;
	static constexpr int kEspRxPin = esp_board::PIN_UART_RX;
	static constexpr int kEspTxPin = esp_board::PIN_UART_TX;
	static constexpr int kEspCtsPin = esp_board::PIN_UART_CTS;
	static constexpr int kEspRtsPin = esp_board::PIN_UART_RTS;
	static constexpr uint32_t kHeartbeatIntervalMs = 500;
	static constexpr uint32_t kServiceSnapshotIntervalMs = 1000;
	static constexpr uint32_t kWirelessSnapshotIntervalMs = 1000;
	static constexpr uint32_t kTeensyHeartbeatStaleTimeoutMs = 1500;
	static constexpr uint8_t kCommandQueueCapacity = 4;
	static constexpr uint8_t kInputEventBucketCount = 5;
	static constexpr size_t kFrameBufferSize = 2u + sizeof(dukatimer::protocol::FrameHeaderBody) +
	                                          dukatimer::protocol::kMaxPayloadSize + sizeof(uint16_t);

	HardwareSerial& serial_;
	bool initialized_ = false;
	uint32_t txSequence_ = 1;
	uint32_t inputEventSequence_ = 1;
	uint32_t nextVfsTransactionId_ = 1;
	uint32_t lastHeartbeatTxMs_ = 0;
	uint32_t lastServiceSnapshotTxMs_ = 0;
	uint32_t lastWirelessSnapshotTxMs_ = 0;
	uint32_t bootStartedMs_ = 0;
	uint32_t diagnosticCounter_ = 0;
	uint32_t lastTeensyHeartbeatMs_ = 0;
	bool teensyHeartbeatSeen_ = false;
	bool teensyExposureActive_ = false;
	bool fileTransactionActive_ = false;
	UploadPhase uploadPhase_ = UploadPhase::Idle;
	DownloadPhase downloadPhase_ = DownloadPhase::Idle;
	bool abortRequested_ = false;
	bool openRequestPending_ = false;
	bool chunkPending_ = false;
	bool downloadChunkPending_ = false;
	uint32_t activeVfsTransactionId_ = 0;
	uint32_t nextVfsOffset_ = 0;
	uint32_t inFlightChunkOffset_ = 0;
	uint16_t inFlightChunkLength_ = 0;
	bool inFlightChunkFinal_ = false;
	uint32_t activeDownloadFileSize_ = 0;
	uint32_t activeDownloadRequestedOffset_ = 0;
	uint16_t activeDownloadRequestedLength_ = 0;
	// decoder_ verdichtet den Bytestrom zu ganzen SharedProtocol-Frames, bevor die
	// eigentliche Service- oder Uploadlogik darauf reagiert.
	dukatimer::protocol::FrameDecoder decoder_;
	uint8_t frameBuffer_[kFrameBufferSize] = {};
	dukatimer::protocol::TeensyCommandPayload lastCommand_ = {};
	dukatimer::protocol::TeensyCommandPayload commandQueue_[kCommandQueueCapacity] = {};
	dukatimer::protocol::DiagnosticPayload lastRemoteDiagnostic_ = {};
	dukatimer::protocol::RemoteDisplayPayload lastRemoteRender_ = {};
	TeensyLinkPendingInputBucket inputEventBuckets_[kInputEventBucketCount] = {};
	dukatimer::protocol::VfsRequestPayload pendingOpenRequest_ = {};
	dukatimer::protocol::VfsChunkPayload pendingChunk_ = {};
	dukatimer::protocol::VfsChunkPayload lastDownloadChunk_ = {};
	dukatimer::protocol::VfsAckPayload lastVfsAck_ = {};
	dukatimer::protocol::VfsErrorPayload lastVfsError_ = {};
	dukatimer::protocol::DiagnosticPayload pendingDiagnostic_ = {};
	bool diagnosticPending_ = false;
	bool remoteDiagnosticPending_ = false;
	bool remoteRenderPending_ = false;
	uint8_t commandQueueHead_ = 0;
	uint8_t commandQueueCount_ = 0;
	uint8_t nextInputEventBucketCursor_ = 0;
	uint32_t droppedCommandCount_ = 0;
	uint32_t droppedInputEventCount_ = 0;
	uint32_t coalescedRemoteRenderCount_ = 0;
	bool ds18Present_ = false;
	bool ds18Fault_ = false;
	bool oneWireOnline_ = false;
	int16_t ds18TemperatureCentiC_ = 0;
	uint32_t ds18SampleTimestampMs_ = 0;
	bool ahtPresent_ = false;
	bool ahtFault_ = false;
	int16_t ahtTemperatureCentiC_ = 0;
	uint16_t ahtHumidityCentiPercent_ = 0;
	bool bmpPresent_ = false;
	bool bmpFault_ = false;
	int16_t bmpTemperatureCentiC_ = 0;
	uint32_t bmpPressureDeciHpa_ = 0;
	dukatimer::protocol::WirelessPeerState wirelessPeerState_ = dukatimer::protocol::WirelessPeerState::Offline;
	uint8_t wirelessBatteryPercent_ = 0;
	bool wirelessLuxValid_ = false;
	uint32_t wirelessLastLuxMilliLux_ = 0;
	uint32_t wirelessMeasurementSequence_ = 0;
	uint32_t wirelessCommandAckSequence_ = 0;
	uint16_t wirelessRenderStatusFlags_ = dukatimer::protocol::kWirelessRenderStatusNone;
	uint32_t wirelessStaleRenderCount_ = 0;
	uint32_t wirelessRenderTimeoutCount_ = 0;
	uint32_t wirelessLastSeenTimestampMs_ = 0;
	uint32_t wirelessLastLuxSampleTimestampMs_ = 0;
	uint16_t paperSlotRecoveryFlags_ = dukatimer::protocol::kPaperSlotRecoveryNone;
	dukatimer::protocol::PaperSlotRecoveryRecommendation paperSlotRecoveryRecommendation_ =
		dukatimer::protocol::PaperSlotRecoveryRecommendation::Unknown;
	dukatimer::protocol::VfsStatusCode paperSlotActiveVfsStatus_ =
		dukatimer::protocol::VfsStatusCode::None;
	uint8_t paperSlotActiveParseError_ = 0;
	dukatimer::protocol::VfsStatusCode paperSlotBackupVfsStatus_ =
		dukatimer::protocol::VfsStatusCode::None;
	uint8_t paperSlotBackupParseError_ = 0;

	void sendHeartbeat(uint32_t nowMs);
	void sendServiceSnapshot(uint32_t nowMs);
	void sendWirelessSnapshot(uint32_t nowMs);
	void sendPendingDiagnostic(uint32_t nowMs);
	void sendPendingInputEvent(uint32_t nowMs);
	void sendPendingOpenRequest(uint32_t nowMs);
	void sendPendingChunk(uint32_t nowMs);
	void readFrames(uint32_t nowMs);
	void handleFrame(const dukatimer::protocol::DecodedFrame& frame, uint32_t nowMs);
	void handleVfsChunk(const dukatimer::protocol::VfsChunkPayload& payload);
	void handleVfsAck(const dukatimer::protocol::VfsAckPayload& payload);
	void handleVfsError(const dukatimer::protocol::VfsErrorPayload& payload);
	void applyTeensyCommand(const dukatimer::protocol::TeensyCommandPayload& payload);
	void applyRemoteDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload);
	void applyRemoteRender(const dukatimer::protocol::RemoteDisplayPayload& payload);
	bool tryWriteFrame(const uint8_t* frameData, size_t frameSize);
	void maybeReportDropDiagnostic(dukatimer::protocol::DiagnosticCode code,
	                             uint32_t occurrenceCount,
	                             uint32_t nowMs);
};

}  // namespace dukatimer