/*
 * EspServiceLink
 *
 * Diese Datei kapselt den kompletten nicht-blockierenden Servicekanal zwischen
 * Teensy und ESP32-S3. Der Hauptloop sieht davon nur einen kleinen Satz aus
 * Runtime-Status und konsumierbaren Payloads; UART-, Frame- und VFS-Details
 * bleiben absichtlich innerhalb dieses Dienstes.
 */

#include "EspServiceLink.h"

#include "FirmwareVersion.h"
#include "ServiceLinkRuntimeFlags.h"

namespace dukatimer {

namespace {

// Die Service-Snapshots nutzen kompakte Ganzzahl-Encodings; die sichtbare
// Runtime-Schicht arbeitet dagegen konsequent mit Fliekommawerten in den
// eigentlichen physikalischen Einheiten.
constexpr uint32_t kLinkStaleThresholdMs = 1500;
constexpr uint32_t kLinkLostThresholdMs = 5000;

float centiCelsiusToFloat(int16_t value) {
	return static_cast<float>(value) / 100.0f;
}

float centiPercentToFloat(uint16_t value) {
	return static_cast<float>(value) / 100.0f;
}

float deciHpaToFloat(uint32_t value) {
	return static_cast<float>(value) / 10.0f;
}

float milliLuxToFloat(uint32_t value) {
	return static_cast<float>(value) / 1000.0f;
}

uint32_t timestampFromAge(uint32_t nowMs, uint32_t ageMs) {
	return ageMs <= nowMs ? (nowMs - ageMs) : 0u;
}

uint8_t pendingInputBucketIndexForSource(dukatimer::protocol::RemoteInputSource source) {
	switch (source) {
		case dukatimer::protocol::RemoteInputSource::Encoder4:
			return 0u;
		case dukatimer::protocol::RemoteInputSource::WirelessEncoder:
			return 1u;
		case dukatimer::protocol::RemoteInputSource::WirelessMeasureButton:
			return 2u;
		case dukatimer::protocol::RemoteInputSource::WirelessBackButton:
			return 3u;
		case dukatimer::protocol::RemoteInputSource::WirelessEncoderButton:
			return 4u;
		case dukatimer::protocol::RemoteInputSource::None:
			break;
	}

	return 0xFFu;
}

uint8_t saturatingIncrement(uint8_t value, uint8_t increment = 1u) {
	const uint16_t widened = static_cast<uint16_t>(value) + static_cast<uint16_t>(increment);
	return widened > 0xFFu ? 0xFFu : static_cast<uint8_t>(widened);
}

int16_t clampAccumulatedRotationValue(int32_t value) {
	if (value > 32767) {
		return 32767;
	}
	if (value < -32768) {
		return -32768;
	}
	return static_cast<int16_t>(value);
}

bool enqueuePendingInputEvent(EspPendingInputBucket& bucket,
	                          const dukatimer::protocol::InputEventPayload& payload) {
	bucket.source = static_cast<dukatimer::protocol::RemoteInputSource>(payload.source);
	bucket.latestSequence = payload.eventSequence;
	bucket.latestTimestampMs = payload.sourceTimestampMs;

	switch (static_cast<dukatimer::protocol::InputEventKind>(payload.eventKind)) {
		case dukatimer::protocol::InputEventKind::RotateLeft:
		case dukatimer::protocol::InputEventKind::RotateRight:
			bucket.rotationDelta = clampAccumulatedRotationValue(
				static_cast<int32_t>(bucket.rotationDelta) + static_cast<int32_t>(payload.value));
			return true;

		case dukatimer::protocol::InputEventKind::Press:
			bucket.pressCount = saturatingIncrement(bucket.pressCount);
			return true;

		case dukatimer::protocol::InputEventKind::LongPress:
			bucket.longPressCount = saturatingIncrement(bucket.longPressCount);
			return true;

		case dukatimer::protocol::InputEventKind::Measure:
			bucket.measureCount = saturatingIncrement(bucket.measureCount);
			return true;

		case dukatimer::protocol::InputEventKind::Undo:
			bucket.undoCount = saturatingIncrement(bucket.undoCount);
			return true;

		case dukatimer::protocol::InputEventKind::None:
			break;
	}

	return false;
}

template <size_t N>
bool hasPendingInputBuckets(const std::array<EspPendingInputBucket, N>& buckets) {
	for (const EspPendingInputBucket& bucket : buckets) {
		if (bucket.hasPending()) {
			return true;
		}
	}

	return false;
}

bool popPendingInputEvent(EspPendingInputBucket& bucket, dukatimer::protocol::InputEventPayload* payloadOut) {
	if (payloadOut == nullptr || !bucket.hasPending()) {
		return false;
	}

	payloadOut->source = static_cast<uint8_t>(bucket.source);
	payloadOut->eventSequence = bucket.latestSequence;
	payloadOut->sourceTimestampMs = bucket.latestTimestampMs;

	if (bucket.pressCount > 0u) {
		payloadOut->eventKind = static_cast<uint8_t>(dukatimer::protocol::InputEventKind::Press);
		payloadOut->value = 1;
		--bucket.pressCount;
		return true;
	}

	if (bucket.longPressCount > 0u) {
		payloadOut->eventKind = static_cast<uint8_t>(dukatimer::protocol::InputEventKind::LongPress);
		payloadOut->value = 1;
		--bucket.longPressCount;
		return true;
	}

	if (bucket.measureCount > 0u) {
		payloadOut->eventKind = static_cast<uint8_t>(dukatimer::protocol::InputEventKind::Measure);
		payloadOut->value = 1;
		--bucket.measureCount;
		return true;
	}

	if (bucket.undoCount > 0u) {
		payloadOut->eventKind = static_cast<uint8_t>(dukatimer::protocol::InputEventKind::Undo);
		payloadOut->value = 1;
		--bucket.undoCount;
		return true;
	}

	if (bucket.rotationDelta > 0) {
		payloadOut->eventKind = static_cast<uint8_t>(dukatimer::protocol::InputEventKind::RotateRight);
		payloadOut->value = 1;
		--bucket.rotationDelta;
		return true;
	}

	if (bucket.rotationDelta < 0) {
		payloadOut->eventKind = static_cast<uint8_t>(dukatimer::protocol::InputEventKind::RotateLeft);
		payloadOut->value = -1;
		++bucket.rotationDelta;
		return true;
	}

	bucket.clear();
	return false;
}

bool extractVfsPath(const dukatimer::protocol::VfsRequestPayload& payload, char* pathOut, size_t capacity) {
	if (pathOut == nullptr || capacity == 0u || payload.pathLength == 0u ||
	    payload.pathLength > dukatimer::protocol::kVfsMaxPathLength || payload.pathLength >= capacity) {
		return false;
	}

	memcpy(pathOut, payload.path, payload.pathLength);
	pathOut[payload.pathLength] = '\0';
	return true;
}

bool isAllowedVfsPath(const char* path) {
	if (path == nullptr || path[0] != '/') {
		return false;
	}

	return strstr(path, "..") == nullptr;
}

bool isFinalizeChunk(const dukatimer::protocol::VfsChunkPayload& payload) {
	return payload.dataLength == 0u || (payload.flags & dukatimer::protocol::kVfsChunkFinalize) != 0u;
}

bool buildVfsTemporaryPath(const char* finalPath, char* tempPathOut, size_t capacity) {
	if (finalPath == nullptr || tempPathOut == nullptr || capacity == 0u) {
		return false;
	}

	constexpr char kTemporarySuffix[] = ".tmp";
	const size_t finalLength = strlen(finalPath);
	const size_t suffixLength = sizeof(kTemporarySuffix) - 1u;
	if ((finalLength + suffixLength + 1u) > capacity) {
		return false;
	}

	memcpy(tempPathOut, finalPath, finalLength);
	memcpy(tempPathOut + finalLength, kTemporarySuffix, suffixLength + 1u);
	return true;
}

bool buildVfsBackupPath(const char* finalPath, char* backupPathOut, size_t capacity) {
	if (finalPath == nullptr || backupPathOut == nullptr || capacity == 0u) {
		return false;
	}

	constexpr char kBackupSuffix[] = ".bak";
	const size_t finalLength = strlen(finalPath);
	const size_t suffixLength = sizeof(kBackupSuffix) - 1u;
	if ((finalLength + suffixLength + 1u) > capacity) {
		return false;
	}

	memcpy(backupPathOut, finalPath, finalLength);
	memcpy(backupPathOut + finalLength, kBackupSuffix, suffixLength + 1u);
	return true;
}

bool replaceVfsTargetFile(SdFs* sd, const char* temporaryPath, const char* finalPath, const char* backupPath) {
	if (sd == nullptr || temporaryPath == nullptr || finalPath == nullptr || backupPath == nullptr) {
		return false;
	}

	if (sd->exists(backupPath) && !sd->remove(backupPath)) {
		return false;
	}

	const bool finalExists = sd->exists(finalPath);
	if (finalExists && !sd->rename(finalPath, backupPath)) {
		return false;
	}

	if (sd->rename(temporaryPath, finalPath)) {
		if (finalExists) {
			(void)sd->remove(backupPath);
		}
		return true;
	}

	if (finalExists) {
		(void)sd->rename(backupPath, finalPath);
	}

	return false;
}

EspLinkHealth linkHealthForAge(bool heartbeatSeen, uint32_t ageMs) {
	if (!heartbeatSeen) {
		return EspLinkHealth::Unknown;
	}

	if (ageMs <= kLinkStaleThresholdMs) {
		return EspLinkHealth::Online;
	}

	if (ageMs <= kLinkLostThresholdMs) {
		return EspLinkHealth::Stale;
	}

	return EspLinkHealth::Lost;
}

}  // namespace

EspServiceLink::EspServiceLink(HardwareSerialIMXRT& serialPort, uint8_t uartCtsInputPin, uint8_t uartRtsOutputPin)
	: serial_(serialPort), uartCtsInputPin_(uartCtsInputPin), uartRtsOutputPin_(uartRtsOutputPin) {}

void EspServiceLink::begin(uint32_t nowMs) {
	// begin() richtet Ringpuffer und Hardware-Handshake nur einmal ein. Danach
	// kann tick() ohne weitere Blockierinitialisierung im Hauptloop laufen.
	if (initialized_) {
		return;
	}

	serial_.addMemoryForRead(serialReadStorage_, sizeof(serialReadStorage_));
	serial_.addMemoryForWrite(serialWriteStorage_, sizeof(serialWriteStorage_));
	serial_.begin(kBaudRate);
	serial_.attachCts(uartCtsInputPin_);
	serial_.attachRts(uartRtsOutputPin_);
	initialized_ = true;
	state_ = makeUnknownEspLinkRuntimeStatus();
	state_.txQueue.capacity = kTxQueueCapacity;
	state_.vfs.realtimeHoldActive = false;
	for (EspPendingInputBucket& bucket : inputEventBuckets_) {
		bucket.clear();
	}
	for (PendingTxFrame& frame : txQueue_) {
		frame = PendingTxFrame{};
	}
	txQueueCount_ = 0;
	nextInputEventBucketCursor_ = 0;
	inputEventPending_ = false;
	droppedInputEventCount_ = 0;
	lastTxMs_ = nowMs;
	sendHeartbeat(nowMs);
}

void EspServiceLink::tick(uint32_t nowMs) {
	// Die Linkpflege bleibt absichtlich klein und nicht-blockierend:
	// - neue Frames aus dem UART ziehen
	// - Heartbeat bei Bedarf senden
	// - sichtbaren Linkzustand aktualisieren
	if (!initialized_) {
		return;
	}

	readFrames(nowMs);
	flushPendingFrames(nowMs);
	if ((nowMs - lastTxMs_) >= kHeartbeatIntervalMs) {
		sendHeartbeat(nowMs);
	}
	updateLinkHealth(nowMs);
}

bool EspServiceLink::sendTeensyCommand(const dukatimer::protocol::TeensyCommandPayload& payload, uint32_t nowMs) {
	return sendFrame(dukatimer::protocol::MessageType::TeensyCommand, TxFrameKind::TeensyCommand, payload, nowMs);
}

bool EspServiceLink::sendRemoteRender(const dukatimer::protocol::RemoteDisplayPayload& payload, uint32_t nowMs) {
	return sendFrame(dukatimer::protocol::MessageType::RemoteRender, TxFrameKind::RemoteRender, payload, nowMs);
}

bool EspServiceLink::sendDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload, uint32_t nowMs) {
	return sendFrame(dukatimer::protocol::MessageType::Diagnostic, TxFrameKind::Diagnostic, payload, nowMs);
}

bool EspServiceLink::consumeServiceSnapshot(dukatimer::protocol::EspServiceSnapshotPayload* payloadOut) {
	if (!serviceSnapshotPending_ || payloadOut == nullptr) {
		return false;
	}

	*payloadOut = latestServiceSnapshot_;
	serviceSnapshotPending_ = false;
	return true;
}

bool EspServiceLink::consumeInputEvent(dukatimer::protocol::InputEventPayload* payloadOut) {
	if (!inputEventPending_ || payloadOut == nullptr) {
		return false;
	}

	for (uint8_t offset = 0; offset < kInputEventBucketCount; ++offset) {
		const uint8_t bucketIndex = static_cast<uint8_t>((nextInputEventBucketCursor_ + offset) %
		                                               kInputEventBucketCount);
		EspPendingInputBucket& bucket = inputEventBuckets_[bucketIndex];
		if (!bucket.hasPending()) {
			continue;
		}

		if (popPendingInputEvent(bucket, payloadOut)) {
			nextInputEventBucketCursor_ = static_cast<uint8_t>((bucketIndex + 1u) % kInputEventBucketCount);
			if (!bucket.hasPending()) {
				bucket.clear();
			}
			inputEventPending_ = hasPendingInputBuckets(inputEventBuckets_);
			state_.lastInputEvent.pending = inputEventPending_;
			return true;
		}
	}

	inputEventPending_ = false;
	state_.lastInputEvent.pending = inputEventPending_;
	return false;
}

bool EspServiceLink::consumeWirelessSnapshot(dukatimer::protocol::WirelessSnapshotPayload* payloadOut) {
	if (!wirelessSnapshotPending_ || payloadOut == nullptr) {
		return false;
	}

	*payloadOut = latestWirelessSnapshot_;
	wirelessSnapshotPending_ = false;
	return true;
}

const EspLinkRuntimeStatus& EspServiceLink::state() const {
	return state_;
}

void EspServiceLink::attachStorage(TeensyStorageVolume* sdCard) {
	// Der SD-Zugriff wird bewusst spaet angebunden. So bleibt der Linkdienst auch
	// ohne Karte oder vor dem ersten VFS-Zugriff voll benutzbar.
	sd_ = sdCard;
	storageMounted_ = false;
	if (!writeSessionOpen_) {
		return;
	}

	resetWriteSession(true);
}

void EspServiceLink::setRealtimeHold(bool holdActive) {
	// Realtime-Hold priorisiert die laufende Belichtung ueber jeden Dateitransfer.
	// Ist der Hold aktiv, darf der ESP keine VFS-Schreibsession mehr weiterfuehren.
	state_.vfs.realtimeHoldActive = holdActive;
	if (!initialized_ || holdActive == realtimeHoldActive_) {
		realtimeHoldActive_ = holdActive;
		return;
	}

	if (holdActive) {
		pinMode(uartRtsOutputPin_, OUTPUT);
		digitalWrite(uartRtsOutputPin_, HIGH);
		if (writeSessionOpen_) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Busy;
			resetWriteSession(true);
		}
	} else {
		serial_.attachRts(uartRtsOutputPin_);
	}

	realtimeHoldActive_ = holdActive;
}

void EspServiceLink::sendHeartbeat(uint32_t nowMs) {
	dukatimer::protocol::HeartbeatPayload payload;
	payload.nodeRole = static_cast<uint8_t>(dukatimer::protocol::NodeRole::Teensy);
	payload.bootState = static_cast<uint8_t>(dukatimer::protocol::BootState::Ready);
	payload.firmwareMajor = static_cast<uint8_t>(build::kVersionMajor);
	payload.firmwareMinor = static_cast<uint8_t>(build::kVersionMinor);
	payload.firmwarePatch = static_cast<uint8_t>(build::kVersionPatch);
	payload.runtimeFlags = realtimeHoldActive_
		? dukatimer::protocol::kHeartbeatRuntimeExposureActive
		: dukatimer::protocol::kHeartbeatRuntimeNone;
	payload.uptimeMs = nowMs;
	payload.capabilityBits = dukatimer::protocol::kCapabilityRemoteRender |
	                      dukatimer::protocol::kCapabilityVfsBridge;

	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::Heartbeat,
	                                                        dukatimer::protocol::NodeRole::Teensy,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u) {
		return;
	}

	if (!enqueuePendingFrame(TxFrameKind::Heartbeat, frameBuffer_, frameSize)) {
		return;
	}

	flushPendingFrames(nowMs);
}

void EspServiceLink::readFrames(uint32_t nowMs) {
	// readFrames() drainiert den UART ohne Wartefenster. Vollstaendige Frames
	// werden sofort an handleFrame() uebergeben, unvollstaendige bleiben im Decoder.
	dukatimer::protocol::DecodedFrame frame;
	while (serial_.available() > 0) {
		const int value = serial_.read();
		if (value < 0) {
			break;
		}

		if (decoder_.push(static_cast<uint8_t>(value), &frame)) {
			lastRxMs_ = nowMs;
			handleFrame(frame, nowMs);
		}
	}
}

void EspServiceLink::updateLinkHealth(uint32_t nowMs) {
	// Linkgesundheit wird nur aus beobachteter Heartbeat-Age abgeleitet. Einzelne
	// Payloadtypen koennen also fehlen, ohne dass sofort der gesamte Link als lost gilt.
	if (lastTxMs_ > 0u) {
		state_.lastTxAgeMs = nowMs - lastTxMs_;
	}

	if (lastRxMs_ > 0u) {
		state_.lastRxAgeMs = nowMs - lastRxMs_;
	}

	const uint32_t heartbeatAgeMs = lastHeartbeatRxMs_ > 0u ? (nowMs - lastHeartbeatRxMs_) : 0u;
	state_.health = linkHealthForAge(state_.heartbeatSeen, heartbeatAgeMs);
	state_.wireless.lastSeenAgeMs =
		latestWirelessPeerSeenTimestampMs_ > 0u ? (nowMs - latestWirelessPeerSeenTimestampMs_) : 0u;
	state_.wireless.sensorSampleAgeMs = latestWirelessSensorSampleTimestampMs_ > 0u
		? (nowMs - latestWirelessSensorSampleTimestampMs_)
		: 0u;
	state_.vfs.realtimeHoldActive = realtimeHoldActive_;
}

void EspServiceLink::handleFrame(const dukatimer::protocol::DecodedFrame& frame, uint32_t nowMs) {
	// Der Frame-Dispatch ist bewusst streng nach MessageType getrennt. Jede
	// Payloadart aktualisiert nur ihren eigenen sichtbaren Teilzustand.
	switch (frame.messageType) {
		case dukatimer::protocol::MessageType::Heartbeat: {
			dukatimer::protocol::HeartbeatPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyHeartbeat(payload, nowMs);
			}
			break;
		}

		case dukatimer::protocol::MessageType::EspServiceSnapshot: {
			dukatimer::protocol::EspServiceSnapshotPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyServiceSnapshot(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::InputEvent: {
			dukatimer::protocol::InputEventPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyInputEvent(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::WirelessSnapshot: {
			dukatimer::protocol::WirelessSnapshotPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyWirelessSnapshot(payload, nowMs);
			}
			break;
		}

		case dukatimer::protocol::MessageType::VfsRequest: {
			dukatimer::protocol::VfsRequestPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				processVfsRequest(payload, nowMs);
			}
			break;
		}

		case dukatimer::protocol::MessageType::VfsChunk: {
			dukatimer::protocol::VfsChunkPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				processVfsChunk(payload, nowMs);
			}
			break;
		}

		case dukatimer::protocol::MessageType::VfsAck:
		case dukatimer::protocol::MessageType::VfsError:

		case dukatimer::protocol::MessageType::TeensyCommand:
		case dukatimer::protocol::MessageType::RemoteRender:
			break;

		case dukatimer::protocol::MessageType::Diagnostic: {
			dukatimer::protocol::DiagnosticPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyDiagnostic(payload);
			}
			break;
		}
	}
}

template <typename Payload>
bool EspServiceLink::sendFrame(dukatimer::protocol::MessageType messageType,
	                          TxFrameKind frameKind,
	                          const Payload& payload,
	                          uint32_t nowMs) {
	if (!initialized_) {
		return false;
	}

	const size_t frameSize = dukatimer::protocol::encodeFrame(messageType,
	                                                        dukatimer::protocol::NodeRole::Teensy,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u) {
		return false;
	}

	if (!enqueuePendingFrame(frameKind, frameBuffer_, frameSize)) {
		return false;
	}

	flushPendingFrames(nowMs);
	return true;
}

EspServiceLink::TxFramePriority EspServiceLink::priorityForFrameKind(TxFrameKind frameKind) {
	switch (frameKind) {
		case TxFrameKind::Diagnostic:
			return TxFramePriority::Background;

		case TxFrameKind::RemoteRender:
			return TxFramePriority::BestEffort;

		case TxFrameKind::Heartbeat:
			return TxFramePriority::Normal;

		case TxFrameKind::VfsChunk:
		case TxFrameKind::VfsAck:
		case TxFrameKind::VfsError:
			return TxFramePriority::Important;

		case TxFrameKind::TeensyCommand:
			return TxFramePriority::Critical;
	}

	return TxFramePriority::Background;
}

bool EspServiceLink::canCoalesceFrameKind(TxFrameKind frameKind) {
	switch (frameKind) {
		case TxFrameKind::Heartbeat:
		case TxFrameKind::RemoteRender:
		case TxFrameKind::Diagnostic:
			return true;

		case TxFrameKind::TeensyCommand:
		case TxFrameKind::VfsAck:
		case TxFrameKind::VfsChunk:
		case TxFrameKind::VfsError:
			return false;
	}

	return false;
}

int8_t EspServiceLink::pendingFrameIndexForKind(TxFrameKind frameKind) const {
	for (uint8_t index = 0; index < txQueueCount_; ++index) {
		if (txQueue_[index].kind == frameKind) {
			return static_cast<int8_t>(index);
		}
	}

	return -1;
}

int8_t EspServiceLink::pendingFrameIndexForLowerPriorityFrame(TxFramePriority incomingPriority) const {
	int8_t candidateIndex = -1;
	TxFramePriority candidatePriority = incomingPriority;
	for (uint8_t index = 0; index < txQueueCount_; ++index) {
		const TxFramePriority queuedPriority = priorityForFrameKind(txQueue_[index].kind);
		if (static_cast<uint8_t>(queuedPriority) >= static_cast<uint8_t>(incomingPriority)) {
			continue;
		}

		if (candidateIndex < 0 || static_cast<uint8_t>(queuedPriority) < static_cast<uint8_t>(candidatePriority)) {
			candidateIndex = static_cast<int8_t>(index);
			candidatePriority = queuedPriority;
		}
	}

	return candidateIndex;
}

void EspServiceLink::removePendingFrameAt(uint8_t index) {
	if (index >= txQueueCount_) {
		return;
	}

	for (uint8_t current = static_cast<uint8_t>(index + 1u); current < txQueueCount_; ++current) {
		txQueue_[current - 1u] = txQueue_[current];
	}

	txQueue_[txQueueCount_ - 1u] = PendingTxFrame{};
	--txQueueCount_;
	state_.txQueue.pendingFrameCount = txQueueCount_;
}

void EspServiceLink::noteDroppedFrame(TxFrameKind frameKind) {
	switch (frameKind) {
		case TxFrameKind::Heartbeat:
			++state_.txQueue.droppedHeartbeatCount;
			break;

		case TxFrameKind::RemoteRender:
			++state_.txQueue.droppedRenderCount;
			break;

		case TxFrameKind::Diagnostic:
			++state_.txQueue.droppedDiagnosticCount;
			break;

		case TxFrameKind::TeensyCommand:
			++state_.txQueue.droppedCommandCount;
			break;

		case TxFrameKind::VfsChunk:
		case TxFrameKind::VfsAck:
		case TxFrameKind::VfsError:
			++state_.txQueue.droppedVfsCount;
			break;
	}
}

void EspServiceLink::noteCoalescedFrame(TxFrameKind frameKind) {
	switch (frameKind) {
		case TxFrameKind::RemoteRender:
			++state_.txQueue.coalescedRenderCount;
			break;

		case TxFrameKind::Diagnostic:
			++state_.txQueue.coalescedDiagnosticCount;
			break;

		case TxFrameKind::Heartbeat:
		case TxFrameKind::TeensyCommand:
		case TxFrameKind::VfsChunk:
		case TxFrameKind::VfsAck:
		case TxFrameKind::VfsError:
			break;
	}
}

bool EspServiceLink::enqueuePendingFrame(TxFrameKind frameKind, const uint8_t* frameData, size_t frameSize) {
	if (frameData == nullptr || frameSize == 0u || frameSize > kFrameBufferSize) {
		return false;
	}

	const TxFramePriority incomingPriority = priorityForFrameKind(frameKind);

	if (canCoalesceFrameKind(frameKind)) {
		const int8_t existingIndex = pendingFrameIndexForKind(frameKind);
		if (existingIndex >= 0) {
			PendingTxFrame& frame = txQueue_[existingIndex];
			frame.size = static_cast<uint16_t>(frameSize);
			memcpy(frame.data, frameData, frameSize);
			noteCoalescedFrame(frameKind);
			return true;
		}
	}

	if (txQueueCount_ >= kTxQueueCapacity) {
		const int8_t lowerPriorityIndex = pendingFrameIndexForLowerPriorityFrame(incomingPriority);
		if (lowerPriorityIndex >= 0) {
			noteDroppedFrame(txQueue_[lowerPriorityIndex].kind);
			++state_.txQueue.evictedFrameCount;
			removePendingFrameAt(static_cast<uint8_t>(lowerPriorityIndex));
		}
	}

	if (txQueueCount_ >= kTxQueueCapacity) {
		noteDroppedFrame(frameKind);
		return false;
	}

	uint8_t insertIndex = txQueueCount_;
	for (uint8_t index = 0; index < txQueueCount_; ++index) {
		const TxFramePriority queuedPriority = priorityForFrameKind(txQueue_[index].kind);
		if (static_cast<uint8_t>(queuedPriority) < static_cast<uint8_t>(incomingPriority)) {
			insertIndex = index;
			break;
		}
	}

	for (uint8_t current = txQueueCount_; current > insertIndex; --current) {
		txQueue_[current] = txQueue_[current - 1u];
	}

	PendingTxFrame& frame = txQueue_[insertIndex];
	++txQueueCount_;
	frame.kind = frameKind;
	frame.size = static_cast<uint16_t>(frameSize);
	memcpy(frame.data, frameData, frameSize);
	state_.txQueue.pendingFrameCount = txQueueCount_;
	if (state_.txQueue.maxObservedPendingFrameCount < txQueueCount_) {
		state_.txQueue.maxObservedPendingFrameCount = txQueueCount_;
	}
	return true;
}

bool EspServiceLink::tryWriteFrame(const uint8_t* frameData, size_t frameSize) {
	if (frameData == nullptr || frameSize == 0u) {
		return false;
	}

	if (serial_.availableForWrite() < static_cast<int>(frameSize)) {
		return false;
	}

	return serial_.write(frameData, frameSize) == frameSize;
}

void EspServiceLink::flushPendingFrames(uint32_t nowMs) {
	// Mehrere fertige Frames duerfen in einen freien UART-Ring laufen, aber nur
	// in kleinem Budget pro Aufruf. So bleiben auch Producer-Pfade mit bereits
	// wartenden Frames lokal begrenzt, statt unter Backpressure den ganzen Stau
	// in einem fremden Call-Kontext abzuarbeiten.
	uint8_t framesFlushed = 0;
	while (txQueueCount_ > 0u && framesFlushed < kTxFlushFrameBudgetPerCall) {
		const PendingTxFrame& frame = txQueue_[0];
		if (!tryWriteFrame(frame.data, frame.size)) {
			return;
		}

		lastTxMs_ = nowMs;
		state_.lastTxAgeMs = 0;
		removePendingFrameAt(0u);
		++framesFlushed;
	}
}

void EspServiceLink::applyHeartbeat(const dukatimer::protocol::HeartbeatPayload& payload, uint32_t nowMs) {
	if (payload.nodeRole != static_cast<uint8_t>(dukatimer::protocol::NodeRole::Esp32Service)) {
		return;
	}

	state_.heartbeatSeen = true;
	lastHeartbeatRxMs_ = nowMs;
	state_.remoteUptimeMs = payload.uptimeMs;
	state_.remoteCapabilityBits = payload.capabilityBits;
	state_.remoteFileTransactionActive =
		dukatimer::service_link_runtime::hasFileTransactionActive(payload.runtimeFlags);
}

void EspServiceLink::applyServiceSnapshot(const dukatimer::protocol::EspServiceSnapshotPayload& payload) {
	// Der letzte Service-Snapshot bleibt als "latest" zwischengespeichert, bis
	// main.cpp ihn konsumiert und in SensorManager ueberfuehrt.
	latestServiceSnapshot_ = payload;
	serviceSnapshotPending_ = true;
	state_.serviceSensors.sensorFlags = payload.sensorFlags;
	state_.serviceSensors.ds18TemperatureCelsius = centiCelsiusToFloat(payload.ds18TemperatureCentiC);
	state_.serviceSensors.ds18SampleAgeMs = payload.ds18SampleAgeMs;
	state_.serviceSensors.ahtTemperatureCelsius = centiCelsiusToFloat(payload.ahtTemperatureCentiC);
	state_.serviceSensors.ahtHumidityPercent = centiPercentToFloat(payload.ahtHumidityCentiPercent);
	state_.serviceSensors.bmpTemperatureCelsius = centiCelsiusToFloat(payload.bmpTemperatureCentiC);
	state_.serviceSensors.bmpPressureHpa = deciHpaToFloat(payload.bmpPressureDeciHpa);
	state_.paperSlotRecovery.flags = payload.paperSlotRecoveryFlags;
	state_.paperSlotRecovery.recommendation =
		static_cast<dukatimer::protocol::PaperSlotRecoveryRecommendation>(
			payload.paperSlotRecoveryRecommendation);
	state_.paperSlotRecovery.activeVfsStatus =
		static_cast<dukatimer::protocol::VfsStatusCode>(payload.paperSlotActiveVfsStatus);
	state_.paperSlotRecovery.activeParseError = payload.paperSlotActiveParseError;
	state_.paperSlotRecovery.backupVfsStatus =
		static_cast<dukatimer::protocol::VfsStatusCode>(payload.paperSlotBackupVfsStatus);
	state_.paperSlotRecovery.backupParseError = payload.paperSlotBackupParseError;
}

void EspServiceLink::applyInputEvent(const dukatimer::protocol::InputEventPayload& payload) {
	// Remote-Input wird per Quelle verdichtet gehalten. Dadurch bleibt der Pfad
	// auch bei laengerer lokaler Ownership-Sperre oder hoher Eventrate begrenzt.
	const auto source = static_cast<dukatimer::protocol::RemoteInputSource>(payload.source);
	const uint8_t bucketIndex = pendingInputBucketIndexForSource(source);
	if (bucketIndex >= kInputEventBucketCount) {
		return;
	}

	EspPendingInputBucket& bucket = inputEventBuckets_[bucketIndex];
	if (!enqueuePendingInputEvent(bucket, payload)) {
		return;
	}

	inputEventPending_ = true;
	state_.lastInputEvent.source = static_cast<dukatimer::protocol::RemoteInputSource>(payload.source);
	state_.lastInputEvent.eventKind = static_cast<dukatimer::protocol::InputEventKind>(payload.eventKind);
	state_.lastInputEvent.value = payload.value;
	state_.lastInputEvent.eventSequence = payload.eventSequence;
	state_.lastInputEvent.sourceTimestampMs = payload.sourceTimestampMs;
	state_.lastInputEvent.pending = true;
}

void EspServiceLink::applyWirelessSnapshot(const dukatimer::protocol::WirelessSnapshotPayload& payload,
	                                      uint32_t nowMs) {
	// Wireless-Telemetrie bleibt vom eigentlichen Service-Snapshot getrennt, damit
	// spaetere Mess- oder Gatewaypfade nicht mit dem Kernlink vermischt werden.
	latestWirelessSnapshot_ = payload;
	latestWirelessPeerSeenTimestampMs_ = timestampFromAge(nowMs, payload.lastSeenAgeMs);
	latestWirelessSensorSampleTimestampMs_ =
		(payload.flags & dukatimer::protocol::kWirelessSnapshotLuxValid) != 0u
			? timestampFromAge(nowMs, payload.sensorSampleAgeMs)
			: 0u;
	wirelessSnapshotPending_ = true;
	state_.wireless.peerState = static_cast<dukatimer::protocol::WirelessPeerState>(payload.peerState);
	state_.wireless.batteryPercent = payload.batteryPercent;
	state_.wireless.flags = payload.flags;
	state_.wireless.lastSeenAgeMs = payload.lastSeenAgeMs;
	state_.wireless.sensorSampleAgeMs = payload.sensorSampleAgeMs;
	state_.wireless.lastLux = milliLuxToFloat(payload.lastLuxMilliLux);
	state_.wireless.measurementSequence = payload.measurementSequence;
	state_.wireless.commandAckSequence = payload.commandAckSequence;
	state_.wireless.renderStatusFlags = payload.renderStatusFlags;
	state_.wireless.staleRenderCount = payload.staleRenderCount;
	state_.wireless.renderTimeoutCount = payload.renderTimeoutCount;
}

void EspServiceLink::applyDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload) {
	state_.diagnostic.code = static_cast<dukatimer::protocol::DiagnosticCode>(payload.code);
	state_.diagnostic.detail = payload.detail;
	state_.diagnostic.counter = payload.counter;
	state_.diagnostic.timestampMs = payload.timestampMs;
}

bool EspServiceLink::ensureStorageMounted() {
	// Das Dateisystem wird lazy gemountet. Ein fehlender oder spaet eingesteckter
	// Datentraeger soll den restlichen Servicekanal nicht beeinflussen.
	if (sd_ == nullptr) {
		return false;
	}

	if (storageMounted_) {
		return true;
	}

	storageMounted_ = beginTeensyStorageVolume(*sd_);
	return storageMounted_;
}

FLASHMEM void EspServiceLink::resetWriteSession(bool removeTemporaryFile) {
	// Jede VFS-Schreibsession ist transaktional gedacht: temp-Datei schliessen,
	// optional aufraeumen und den sichtbaren Transferzustand vollstaendig leeren.
	if (activeWriteFile_) {
		activeWriteFile_.close();
	}

	if (removeTemporaryFile && sd_ != nullptr && activeVfsTmpPath_[0] != '\0') {
		sd_->remove(activeVfsTmpPath_);
	}

	writeSessionOpen_ = false;
	activeVfsTransactionId_ = 0;
	activeVfsBytesWritten_ = 0;
	activeVfsPath_[0] = '\0';
	activeVfsTmpPath_[0] = '\0';
	state_.vfs.transferActive = false;
	state_.vfs.transactionId = 0;
	state_.vfs.transferredBytes = 0;
}

FLASHMEM void EspServiceLink::processVfsRequest(const dukatimer::protocol::VfsRequestPayload& payload, uint32_t nowMs) {
	// Der Request oeffnet nur dann eine Session, wenn Echtzeitbetrieb, SD-Status,
	// Pfadvalidierung und Transaktionszustand konsistent sind. Geschrieben wird
	// stets zuerst in eine .tmp-Datei, nie direkt in die Zieldatei.
	if (realtimeHoldActive_) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Busy;
		sendVfsError(dukatimer::protocol::VfsStatusCode::Busy, payload.transactionId, payload.operation, 0u, 0u,
		             nowMs);
		return;
	}

	if (payload.operation != static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileWriteRequest)) {
		if (payload.operation != static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileReadRequest)) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Unsupported;
			sendVfsError(dukatimer::protocol::VfsStatusCode::Unsupported, payload.transactionId, payload.operation, 0u,
			             0u, nowMs);
			return;
		}
	}

	if (writeSessionOpen_) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Busy;
		sendVfsError(dukatimer::protocol::VfsStatusCode::Busy, payload.transactionId, 0u, activeVfsBytesWritten_,
		             activeVfsTransactionId_, nowMs);
		return;
	}

	if (!ensureStorageMounted()) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::SdUnavailable;
		sendVfsError(dukatimer::protocol::VfsStatusCode::SdUnavailable, payload.transactionId, 0u, 0u, 0u, nowMs);
		return;
	}

	char requestedPath[dukatimer::protocol::kVfsMaxPathLength + 1] = {};
	if (!extractVfsPath(payload, requestedPath, sizeof(requestedPath)) || !isAllowedVfsPath(requestedPath)) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::InvalidPath;
		sendVfsError(dukatimer::protocol::VfsStatusCode::InvalidPath, payload.transactionId, payload.pathLength, 0u, 0u,
		             nowMs);
		return;
	}

	if (payload.operation == static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileReadRequest)) {
		// FileReadRequest liefert bewusst genau einen angefragten Dateislice zurueck.
		// Damit kann der ESP Produktdaten kontrolliert exportieren oder fuer eine
		// Recovery lokal zwischenspeichern, ohne hier einen zweiten Streampfad mit
		// eigener Session-Lebenszeit zu oeffnen.
		if (payload.flags != dukatimer::protocol::kVfsRequestNone ||
		    payload.requestedLength > dukatimer::protocol::kVfsChunkDataSize) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::ProtocolRejected;
			sendVfsError(dukatimer::protocol::VfsStatusCode::ProtocolRejected, payload.transactionId,
			             payload.requestedLength, payload.requestedOffset,
			             dukatimer::protocol::kVfsChunkDataSize, nowMs);
			return;
		}

		TeensyStorageFile sourceFile;
		if (!sourceFile.open(requestedPath, O_RDONLY)) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::OpenFailed;
			sendVfsError(dukatimer::protocol::VfsStatusCode::OpenFailed, payload.transactionId, 0u,
			             payload.requestedOffset, 0u, nowMs);
			return;
		}

		const uint32_t fileSize = static_cast<uint32_t>(sourceFile.size());
		if (payload.requestedOffset > fileSize) {
			sourceFile.close();
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::ProtocolRejected;
			sendVfsError(dukatimer::protocol::VfsStatusCode::ProtocolRejected, payload.transactionId,
			             payload.requestedLength, payload.requestedOffset, fileSize, nowMs);
			return;
		}

		const uint16_t requestedLength = payload.requestedLength == 0u
			? dukatimer::protocol::kVfsChunkDataSize
			: payload.requestedLength;
		if (!sourceFile.seek(payload.requestedOffset)) {
			sourceFile.close();
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::ReadFailed;
			sendVfsError(dukatimer::protocol::VfsStatusCode::ReadFailed, payload.transactionId, 0u,
			             payload.requestedOffset, fileSize, nowMs);
			return;
		}

		dukatimer::protocol::VfsChunkPayload chunkPayload = {};
		chunkPayload.transactionId = payload.transactionId;
		chunkPayload.fileOffset = payload.requestedOffset;
		const uint32_t remainingBytes = fileSize - payload.requestedOffset;
		const uint16_t bytesToRead = static_cast<uint16_t>(remainingBytes < requestedLength
			? remainingBytes
			: requestedLength);
		if (bytesToRead > 0u) {
			const int readBytes = sourceFile.read(chunkPayload.data, bytesToRead);
			if (readBytes < 0 || static_cast<uint16_t>(readBytes) != bytesToRead) {
				sourceFile.close();
				state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::ReadFailed;
				sendVfsError(dukatimer::protocol::VfsStatusCode::ReadFailed, payload.transactionId,
				             readBytes < 0 ? 0u : static_cast<uint16_t>(readBytes), payload.requestedOffset,
				             bytesToRead, nowMs);
				return;
			}
			chunkPayload.dataLength = static_cast<uint16_t>(readBytes);
		}

		sourceFile.close();
		if ((payload.requestedOffset + chunkPayload.dataLength) >= fileSize) {
			chunkPayload.flags |= dukatimer::protocol::kVfsChunkFinalize;
		}

		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Ok;
		state_.vfs.transferActive = false;
		state_.vfs.transactionId = payload.transactionId;
		state_.vfs.transferredBytes = payload.requestedOffset + chunkPayload.dataLength;
		sendVfsAck(dukatimer::protocol::VfsOperation::FileReadRequest,
		          dukatimer::protocol::VfsStatusCode::Ok,
		          payload.transactionId,
		          payload.requestedOffset,
		          chunkPayload.dataLength,
		          fileSize,
		          nowMs);
		if (!sendFrame(dukatimer::protocol::MessageType::VfsChunk,
		              TxFrameKind::VfsChunk,
		              chunkPayload,
		              nowMs)) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::ReadFailed;
			sendVfsError(dukatimer::protocol::VfsStatusCode::ReadFailed, payload.transactionId,
			             chunkPayload.dataLength, payload.requestedOffset, fileSize, nowMs);
		}
		return;
	}

	char temporaryPath[dukatimer::protocol::kVfsMaxPathLength + 5] = {};
	if (!buildVfsTemporaryPath(requestedPath, temporaryPath, sizeof(temporaryPath))) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::InvalidPath;
		sendVfsError(dukatimer::protocol::VfsStatusCode::InvalidPath, payload.transactionId, payload.pathLength, 0u, 0u,
		             nowMs);
		return;
	}

	// Eventuelle Altlasten eines abgebrochenen Transfers entfernen.
	(void)sd_->remove(temporaryPath);

	if (!activeWriteFile_.open(temporaryPath, O_WRONLY | O_CREAT | O_TRUNC)) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::OpenFailed;
		sendVfsError(dukatimer::protocol::VfsStatusCode::OpenFailed, payload.transactionId, 0u, 0u, 0u, nowMs);
		return;
	}

	writeSessionOpen_ = true;
	activeVfsTransactionId_ = payload.transactionId;
	activeVfsBytesWritten_ = 0;
	memcpy(activeVfsPath_, requestedPath, strlen(requestedPath) + 1u);
	memcpy(activeVfsTmpPath_, temporaryPath, strlen(temporaryPath) + 1u);
	state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Ok;
	state_.vfs.transferActive = true;
	state_.vfs.transactionId = payload.transactionId;
	state_.vfs.transferredBytes = 0;
	sendVfsAck(dukatimer::protocol::VfsOperation::FileWriteRequest, dukatimer::protocol::VfsStatusCode::Ok,
	          payload.transactionId, 0u, 0u, 0u, nowMs);
}

FLASHMEM void EspServiceLink::processVfsChunk(const dukatimer::protocol::VfsChunkPayload& payload, uint32_t nowMs) {
	// Chunks muessen strikt fortlaufend und transaktionsgebunden ankommen. Damit
	// wird aus dem seriellen VFS-Pfad ein einfacher, robust nachvollziehbarer
	// Stream ohne versteckte Random-Access- oder Reorder-Semantik.
	if (realtimeHoldActive_) {
		const uint32_t acknowledgedBytes = activeVfsBytesWritten_;
		if (writeSessionOpen_) {
			resetWriteSession(true);
		}

		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Busy;
		sendVfsError(dukatimer::protocol::VfsStatusCode::Busy, payload.transactionId, payload.flags,
		             payload.fileOffset, acknowledgedBytes, nowMs);
		return;
	}

	if (!writeSessionOpen_) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::InvalidState;
		sendVfsError(dukatimer::protocol::VfsStatusCode::InvalidState, payload.transactionId, 0u, payload.fileOffset,
		             0u, nowMs);
		return;
	}

	if (payload.transactionId != activeVfsTransactionId_) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::TransactionMismatch;
		sendVfsError(dukatimer::protocol::VfsStatusCode::TransactionMismatch, payload.transactionId, 0u,
		             payload.fileOffset, activeVfsTransactionId_, nowMs);
		return;
	}

	if (payload.fileOffset != activeVfsBytesWritten_ || payload.dataLength > dukatimer::protocol::kVfsChunkDataSize) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::ProtocolRejected;
		sendVfsError(dukatimer::protocol::VfsStatusCode::ProtocolRejected, payload.transactionId, payload.dataLength,
		             payload.fileOffset, activeVfsBytesWritten_, nowMs);
		return;
	}

	if ((payload.flags & dukatimer::protocol::kVfsChunkAbort) != 0u) {
		state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::InvalidState;
		resetWriteSession(true);
		sendVfsError(dukatimer::protocol::VfsStatusCode::InvalidState, payload.transactionId, payload.flags,
		             payload.fileOffset, 0u, nowMs);
		return;
	}

	if (payload.dataLength > 0u) {
		const size_t bytesWritten = activeWriteFile_.write(payload.data, payload.dataLength);
		if (bytesWritten != payload.dataLength) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::WriteFailed;
			resetWriteSession(true);
			sendVfsError(dukatimer::protocol::VfsStatusCode::WriteFailed, payload.transactionId,
			             static_cast<uint16_t>(bytesWritten), payload.fileOffset, payload.dataLength, nowMs);
			return;
		}

		activeVfsBytesWritten_ += payload.dataLength;
		state_.vfs.transferredBytes = activeVfsBytesWritten_;
	}

	const bool finalize = isFinalizeChunk(payload);
	if (finalize) {
		// Finalisierung erfolgt atomar ueber rename() der temp-Datei. Misslingt das,
		// bleibt die Session fehlerhaft und die alte Datei wird nicht stillschweigend
		// halb ueberschrieben.
		activeWriteFile_.close();

		bool renameOk = false;
		if (sd_ != nullptr && activeVfsTmpPath_[0] != '\0' && activeVfsPath_[0] != '\0') {
			char backupPath[dukatimer::protocol::kVfsMaxPathLength + 5] = {};
			renameOk = buildVfsBackupPath(activeVfsPath_, backupPath, sizeof(backupPath)) &&
			           replaceVfsTargetFile(sd_, activeVfsTmpPath_, activeVfsPath_, backupPath);
		}

		if (!renameOk) {
			state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::WriteFailed;
			sendVfsError(dukatimer::protocol::VfsStatusCode::WriteFailed, payload.transactionId, 0u,
			             payload.fileOffset, activeVfsBytesWritten_, nowMs);
			resetWriteSession(true);
			return;
		}
	}

	state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Ok;
	sendVfsAck(dukatimer::protocol::VfsOperation::FileWriteRequest, dukatimer::protocol::VfsStatusCode::Ok,
	          payload.transactionId, payload.fileOffset, payload.dataLength, activeVfsBytesWritten_, nowMs);

	if (finalize) {
		resetWriteSession(false);
	}
}

FLASHMEM void EspServiceLink::sendVfsAck(dukatimer::protocol::VfsOperation operation,
	                            dukatimer::protocol::VfsStatusCode statusCode,
	                            uint32_t transactionId,
	                            uint32_t acknowledgedOffset,
	                            uint16_t acknowledgedLength,
	                            uint32_t detail,
	                            uint32_t nowMs) {
	// Ack und Error laufen ueber denselben Framingpfad wie Heartbeats und Events.
	// Damit bleibt das serielle Protokoll in beide Richtungen einheitlich.
	dukatimer::protocol::VfsAckPayload payload;
	payload.transactionId = transactionId;
	payload.acknowledgedOffset = acknowledgedOffset;
	payload.acknowledgedLength = acknowledgedLength;
	payload.statusCode = static_cast<uint8_t>(statusCode);
	payload.operation = static_cast<uint8_t>(operation);
	payload.detail = detail;

	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::VfsAck,
	                                                        dukatimer::protocol::NodeRole::Teensy,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u) {
		return;
	}

	if (!enqueuePendingFrame(TxFrameKind::VfsAck, frameBuffer_, frameSize)) {
		return;
	}

	flushPendingFrames(nowMs);
}

FLASHMEM void EspServiceLink::sendVfsError(dukatimer::protocol::VfsStatusCode statusCode,
	                              uint32_t transactionId,
	                              uint16_t detail,
	                              uint32_t fileOffset,
	                              uint32_t info,
	                              uint32_t nowMs) {
	dukatimer::protocol::VfsErrorPayload payload;
	payload.transactionId = transactionId;
	payload.statusCode = static_cast<uint16_t>(statusCode);
	payload.detail = detail;
	payload.fileOffset = fileOffset;
	payload.info = info;

	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::VfsError,
	                                                        dukatimer::protocol::NodeRole::Teensy,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u) {
		return;
	}

	if (!enqueuePendingFrame(TxFrameKind::VfsError, frameBuffer_, frameSize)) {
		return;
	}

	flushPendingFrames(nowMs);
}

}  // namespace dukatimer