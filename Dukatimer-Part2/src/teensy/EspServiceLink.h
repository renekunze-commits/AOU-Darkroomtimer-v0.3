/*
 * EspServiceLink
 *
 * Version:
 * - Implementierungsschritt 6 der Inter-MCU-Basis
 * - eingefuehrt am 2026-04-24
 * - API-Version 1 fuer den nicht-blockierenden ESP32-S3-Servicepfad am Teensy
 *
 * Zweck:
 * - kapselt die UART-Kommunikation zum ESP32-S3 ueber S3_TX/S3_RX
 * - empfängt Heartbeat, Thermik-/Servicewerte, Encoder-4-Ereignisse und
 *   Wireless-2591-Gatewaystatus ohne die Teensy-Kernlogik zu blockieren
 *
 * Sicherheitsgrenze:
 * - der Dienst ist strikt erweiternd
 * - ein Linkverlust darf keine Kernfunktion des Teensy blockieren
 */
#pragma once

#include <Arduino.h>

#include <array>

#include <DukatimerProtocol.h>

#include "EspLinkRuntimeStatus.h"
#include "TeensyStoragePolicy.h"

namespace dukatimer {

struct EspPendingInputBucket {
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
 * EspServiceLink
 *
 * Zweck:
 * - bildet den nicht-blockierenden Teensy-seitigen Zugang zum ESP32-S3-Service-MCU
 * - entkoppelt UART-Frames, Heartbeat-Zustand, Remote-Input, Wireless-Snapshots
 *   und den SD-VFS-Brueckenpfad vom restlichen System
 * - praesentiert dem Hauptloop nur letzte bekannte Runtime-Daten und konsumierbare
 *   Pending-Payloads
 */
class EspServiceLink {
public:
	explicit EspServiceLink(HardwareSerialIMXRT& serialPort, uint8_t uartCtsInputPin, uint8_t uartRtsOutputPin);

	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs);
	void attachStorage(TeensyStorageVolume* sdCard);
	void setRealtimeHold(bool holdActive);
	bool sendTeensyCommand(const dukatimer::protocol::TeensyCommandPayload& payload, uint32_t nowMs = 0);
	bool sendRemoteRender(const dukatimer::protocol::RemoteDisplayPayload& payload, uint32_t nowMs = 0);
	bool sendDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload, uint32_t nowMs = 0);

	bool consumeServiceSnapshot(dukatimer::protocol::EspServiceSnapshotPayload* payloadOut);
	bool consumeInputEvent(dukatimer::protocol::InputEventPayload* payloadOut);
	bool consumeWirelessSnapshot(dukatimer::protocol::WirelessSnapshotPayload* payloadOut);

	const EspLinkRuntimeStatus& state() const;

private:
	static constexpr uint32_t kBaudRate = 1000000;
	static constexpr uint32_t kHeartbeatIntervalMs = 500;
	static constexpr uint32_t kLinkStaleThresholdMs = 1500;
	static constexpr uint32_t kLinkLostThresholdMs = 5000;
	static constexpr size_t kSerialReadStorageSize = 4096;
	static constexpr size_t kSerialWriteStorageSize = 2048;
	static constexpr uint8_t kInputEventBucketCount = 5;
	static constexpr uint8_t kTxQueueCapacity = 6;
	static constexpr uint8_t kTxFlushFrameBudgetPerCall = 2;
	static constexpr size_t kFrameBufferSize = 2u + sizeof(dukatimer::protocol::FrameHeaderBody) +
	                                          dukatimer::protocol::kMaxPayloadSize + sizeof(uint16_t);

	enum class TxFrameKind : uint8_t {
		Heartbeat,
		TeensyCommand,
		RemoteRender,
		Diagnostic,
		VfsChunk,
		VfsAck,
		VfsError,
	};

	enum class TxFramePriority : uint8_t {
		Background,
		BestEffort,
		Normal,
		Important,
		Critical,
	};

	struct PendingTxFrame {
		TxFrameKind kind = TxFrameKind::Heartbeat;
		uint16_t size = 0;
		uint8_t data[kFrameBufferSize] = {};
	};

	HardwareSerialIMXRT& serial_;
	uint8_t uartCtsInputPin_ = 0;
	uint8_t uartRtsOutputPin_ = 0;
	bool initialized_ = false;
	bool realtimeHoldActive_ = false;
	bool storageMounted_ = false;
	bool writeSessionOpen_ = false;
	uint32_t txSequence_ = 1;
	uint32_t lastTxMs_ = 0;
	uint32_t lastRxMs_ = 0;
	uint32_t lastHeartbeatRxMs_ = 0;
	uint32_t activeVfsTransactionId_ = 0;
	uint32_t activeVfsBytesWritten_ = 0;
	// decoder_ sammelt serielle Bytes zu vollstaendigen SharedProtocol-Frames,
	// bevor irgendeine Fachlogik oder VFS-Behandlung ausgefuehrt wird.
	dukatimer::protocol::FrameDecoder decoder_;
	uint8_t serialReadStorage_[kSerialReadStorageSize] = {};
	uint8_t serialWriteStorage_[kSerialWriteStorageSize] = {};
	uint8_t frameBuffer_[kFrameBufferSize] = {};
	// Ausgehende Frames werden lokal gepuffert, damit ein voller UART-Ring oder
	// angezogener CTS den Teensy-Hauptloop nie in einen blockierenden write() zwingt.
	std::array<PendingTxFrame, kTxQueueCapacity> txQueue_ = {};
	uint8_t txQueueCount_ = 0;
	std::array<EspPendingInputBucket, kInputEventBucketCount> inputEventBuckets_ = {};
	char activeVfsPath_[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
	char activeVfsTmpPath_[dukatimer::protocol::kVfsMaxPathLength + 5] = {};
	// state_ ist das dauerhaft sichtbare Abbild des Links; Pending-Flags markieren,
	// welche Payloads seit dem letzten Consume neu angekommen sind.
	EspLinkRuntimeStatus state_ = makeUnknownEspLinkRuntimeStatus();
	bool serviceSnapshotPending_ = false;
	bool inputEventPending_ = false;
	bool wirelessSnapshotPending_ = false;
	uint8_t nextInputEventBucketCursor_ = 0;
	uint32_t droppedInputEventCount_ = 0;
	TeensyStorageVolume* sd_ = nullptr;
	TeensyStorageFile activeWriteFile_;
	dukatimer::protocol::EspServiceSnapshotPayload latestServiceSnapshot_ = {};
	dukatimer::protocol::WirelessSnapshotPayload latestWirelessSnapshot_ = {};
	uint32_t latestWirelessPeerSeenTimestampMs_ = 0;
	uint32_t latestWirelessSensorSampleTimestampMs_ = 0;

	void sendHeartbeat(uint32_t nowMs);
	void readFrames(uint32_t nowMs);
	void updateLinkHealth(uint32_t nowMs);
	void handleFrame(const dukatimer::protocol::DecodedFrame& frame, uint32_t nowMs);
	void applyHeartbeat(const dukatimer::protocol::HeartbeatPayload& payload, uint32_t nowMs);
	void applyServiceSnapshot(const dukatimer::protocol::EspServiceSnapshotPayload& payload);
	void applyInputEvent(const dukatimer::protocol::InputEventPayload& payload);
	void applyWirelessSnapshot(const dukatimer::protocol::WirelessSnapshotPayload& payload, uint32_t nowMs);
	void applyDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload);
	void flushPendingFrames(uint32_t nowMs);
	bool tryWriteFrame(const uint8_t* frameData, size_t frameSize);
	bool enqueuePendingFrame(TxFrameKind frameKind, const uint8_t* frameData, size_t frameSize);
	int8_t pendingFrameIndexForKind(TxFrameKind frameKind) const;
	int8_t pendingFrameIndexForLowerPriorityFrame(TxFramePriority incomingPriority) const;
	void removePendingFrameAt(uint8_t index);
	void noteDroppedFrame(TxFrameKind frameKind);
	void noteCoalescedFrame(TxFrameKind frameKind);
	static TxFramePriority priorityForFrameKind(TxFrameKind frameKind);
	static bool canCoalesceFrameKind(TxFrameKind frameKind);
	bool ensureStorageMounted();
	void resetWriteSession(bool removeTemporaryFile = false);
	void processVfsRequest(const dukatimer::protocol::VfsRequestPayload& payload, uint32_t nowMs);
	void processVfsChunk(const dukatimer::protocol::VfsChunkPayload& payload, uint32_t nowMs);
	void sendVfsAck(dukatimer::protocol::VfsOperation operation,
	               dukatimer::protocol::VfsStatusCode statusCode,
	               uint32_t transactionId,
	               uint32_t acknowledgedOffset,
	               uint16_t acknowledgedLength,
	               uint32_t detail,
	               uint32_t nowMs);
	void sendVfsError(dukatimer::protocol::VfsStatusCode statusCode,
	                 uint32_t transactionId,
	                 uint16_t detail,
	                 uint32_t fileOffset,
	                 uint32_t info,
	                 uint32_t nowMs);
	template <typename Payload>
	bool sendFrame(dukatimer::protocol::MessageType messageType,
	              TxFrameKind frameKind,
	              const Payload& payload,
	              uint32_t nowMs);
};

}  // namespace dukatimer