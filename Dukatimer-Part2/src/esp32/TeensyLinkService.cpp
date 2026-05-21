/*
 * TeensyLinkService
 *
 * Diese Datei kapselt den kompletten ESP32-seitigen Link zum Teensy. Sie hält
 * periodische Statuspublikation, Remote-Eingaben und den Upload-Zustandsautomaten
 * fuer die serielle VFS-Bruecke an einem Ort zusammen.
 */

#include "TeensyLinkService.h"

#include "FirmwareVersion.h"
#include "ServiceLinkRuntimeFlags.h"

namespace dukatimer {

namespace {

// Die kompakten SharedProtocol-Payloads arbeiten mit Ganzzahlskalen, waehrend
// lokale Dienste auf ESP-Seite bequem mit Fließkommawerten fuettern duerfen.
int16_t clampCentiCelsius(float temperatureCelsius) {
	const float scaled = temperatureCelsius * 100.0f;
	if (scaled > 32767.0f) {
		return 32767;
	}
	if (scaled < -32768.0f) {
		return -32768;
	}
	return static_cast<int16_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

uint16_t clampCentiPercent(float humidityPercent) {
	float scaled = humidityPercent * 100.0f;
	if (scaled < 0.0f) {
		scaled = 0.0f;
	}
	if (scaled > 65535.0f) {
		scaled = 65535.0f;
	}
	return static_cast<uint16_t>(scaled + 0.5f);
}

uint32_t clampDeciHpa(float pressureHpa) {
	float scaled = pressureHpa * 10.0f;
	if (scaled < 0.0f) {
		scaled = 0.0f;
	}
	if (scaled > 4294967295.0f) {
		scaled = 4294967295.0f;
	}
	return static_cast<uint32_t>(scaled + 0.5f);
}

uint32_t clampMilliLux(float lux) {
	float scaled = lux * 1000.0f;
	if (scaled < 0.0f) {
		scaled = 0.0f;
	}
	if (scaled > 4294967295.0f) {
		scaled = 4294967295.0f;
	}
	return static_cast<uint32_t>(scaled + 0.5f);
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

bool enqueuePendingInputEvent(TeensyLinkPendingInputBucket& bucket,
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

bool popPendingInputEvent(TeensyLinkPendingInputBucket& bucket,
	                     dukatimer::protocol::InputEventPayload* payloadOut) {
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

bool hasPendingInputBuckets(const TeensyLinkPendingInputBucket* buckets, uint8_t count) {
	for (uint8_t index = 0; index < count; ++index) {
		if (buckets[index].hasPending()) {
			return true;
		}
	}

	return false;
}

bool isAllowedVfsPath(const char* path) {
	if (path == nullptr || path[0] != '/') {
		return false;
	}

	return strstr(path, "..") == nullptr;
}

}  // namespace

TeensyLinkService::TeensyLinkService(HardwareSerial& serialPort) : serial_(serialPort) {}

void TeensyLinkService::begin(uint32_t nowMs) {
	// Der Linkdienst richtet den UART nur einmal ein und sendet danach sofort die
	// erste Sichtbarkeit fuer Heartbeat, Service- und Wirelessstatus.
	if (initialized_) {
		return;
	}

	serial_.setRxBufferSize(4096);
	serial_.setTxBufferSize(4096);
	serial_.begin(kBaudRate, SERIAL_8N1, kEspRxPin, kEspTxPin);
	serial_.setPins(kEspRxPin, kEspTxPin, kEspCtsPin, kEspRtsPin);
	serial_.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS);
	initialized_ = true;
	for (uint8_t bucketIndex = 0; bucketIndex < kInputEventBucketCount; ++bucketIndex) {
		inputEventBuckets_[bucketIndex].clear();
	}
	nextInputEventBucketCursor_ = 0;
	bootStartedMs_ = nowMs;
	teensyExposureActive_ = false;
	sendHeartbeat(nowMs);
	sendServiceSnapshot(nowMs);
	sendWirelessSnapshot(nowMs);
}

void TeensyLinkService::tick(uint32_t nowMs) {
	// tick() arbeitet den Link konsequent nicht-blockierend ab:
	// - eingehende Frames verarbeiten
	// - ausstehende Open-/Chunk-Frames senden
	// - periodische Heartbeat- und Snapshot-Telemetrie aussenden
	if (!initialized_) {
		return;
	}

	readFrames(nowMs);
	if (openRequestPending_) {
		sendPendingOpenRequest(nowMs);
	}
	if (chunkPending_) {
		sendPendingChunk(nowMs);
	}
	if ((nowMs - lastHeartbeatTxMs_) >= kHeartbeatIntervalMs) {
		sendHeartbeat(nowMs);
	}
	if ((nowMs - lastServiceSnapshotTxMs_) >= kServiceSnapshotIntervalMs) {
		sendServiceSnapshot(nowMs);
	}
	if ((nowMs - lastWirelessSnapshotTxMs_) >= kWirelessSnapshotIntervalMs) {
		sendWirelessSnapshot(nowMs);
	}
	if (diagnosticPending_) {
		sendPendingDiagnostic(nowMs);
	}
	if (hasPendingInputBuckets(inputEventBuckets_, kInputEventBucketCount)) {
		sendPendingInputEvent(nowMs);
	}
}

void TeensyLinkService::setDs18b20Sample(float temperatureCelsius, bool present, bool fault, bool oneWireOnline,
	                                     uint32_t sampleTimestampMs) {
	if (sampleTimestampMs == 0u) {
		sampleTimestampMs = millis();
	}

	ds18Present_ = present;
	ds18Fault_ = fault;
	oneWireOnline_ = oneWireOnline;
	ds18TemperatureCentiC_ = clampCentiCelsius(temperatureCelsius);
	ds18SampleTimestampMs_ = sampleTimestampMs;
}

void TeensyLinkService::setEnvironmentSample(float ahtTemperatureCelsius,
	                                       float ahtHumidityPercent,
	                                       bool ahtPresent,
	                                       bool ahtFault,
	                                       float bmpTemperatureCelsius,
	                                       float bmpPressureHpa,
	                                       bool bmpPresent,
	                                       bool bmpFault) {
	ahtPresent_ = ahtPresent;
	ahtFault_ = ahtFault;
	ahtTemperatureCentiC_ = clampCentiCelsius(ahtTemperatureCelsius);
	ahtHumidityCentiPercent_ = clampCentiPercent(ahtHumidityPercent);
	bmpPresent_ = bmpPresent;
	bmpFault_ = bmpFault;
	bmpTemperatureCentiC_ = clampCentiCelsius(bmpTemperatureCelsius);
	bmpPressureDeciHpa_ = clampDeciHpa(bmpPressureHpa);
}

void TeensyLinkService::setFileTransactionActive(bool active, uint32_t nowMs) {
	if (fileTransactionActive_ == active) {
		return;
	}

	fileTransactionActive_ = active;
	if (!initialized_) {
		return;
	}

	if (nowMs == 0u) {
		nowMs = millis();
	}

	// Dateitransaktionen sollen am Teensy moeglichst sofort sichtbar werden,
	// damit der Belichtungsstart nicht bis zum naechsten periodischen Heartbeat
	// durchrutschen kann.
	sendHeartbeat(nowMs);
}

void TeensyLinkService::setWirelessPeerState(dukatimer::protocol::WirelessPeerState peerState, uint8_t batteryPercent,
	                                        bool luxValid, float lastLux, uint32_t measurementSequence,
	                                        uint32_t commandAckSequence, uint16_t renderStatusFlags,
	                                        uint32_t staleRenderCount, uint32_t renderTimeoutCount,
	                                        uint32_t lastSeenTimestampMs,
	                                        uint32_t lastLuxSampleTimestampMs) {
	if (lastSeenTimestampMs == 0u && peerState != dukatimer::protocol::WirelessPeerState::Offline) {
		lastSeenTimestampMs = millis();
	}
	if (lastLuxSampleTimestampMs == 0u && luxValid && lastSeenTimestampMs != 0u) {
		lastLuxSampleTimestampMs = lastSeenTimestampMs;
	}

	wirelessPeerState_ = peerState;
	wirelessBatteryPercent_ = batteryPercent;
	wirelessLuxValid_ = luxValid;
	wirelessLastLuxMilliLux_ = clampMilliLux(lastLux);
	wirelessMeasurementSequence_ = measurementSequence;
	wirelessCommandAckSequence_ = commandAckSequence;
	wirelessRenderStatusFlags_ = renderStatusFlags;
	wirelessStaleRenderCount_ = staleRenderCount;
	wirelessRenderTimeoutCount_ = renderTimeoutCount;
	wirelessLastSeenTimestampMs_ = lastSeenTimestampMs;
	wirelessLastLuxSampleTimestampMs_ = lastLuxSampleTimestampMs;
}

void TeensyLinkService::setPaperSlotRecoveryStatus(
	uint16_t flags,
	dukatimer::protocol::PaperSlotRecoveryRecommendation recommendation,
	dukatimer::protocol::VfsStatusCode activeVfsStatus,
	uint8_t activeParseError,
	dukatimer::protocol::VfsStatusCode backupVfsStatus,
	uint8_t backupParseError) {
	paperSlotRecoveryFlags_ = flags;
	paperSlotRecoveryRecommendation_ = recommendation;
	paperSlotActiveVfsStatus_ = activeVfsStatus;
	paperSlotActiveParseError_ = activeParseError;
	paperSlotBackupVfsStatus_ = backupVfsStatus;
	paperSlotBackupParseError_ = backupParseError;
}

void TeensyLinkService::queueDiagnostic(dukatimer::protocol::DiagnosticCode code, uint16_t detail,
	                                   uint32_t timestampMs) {
	if (timestampMs == 0u) {
		timestampMs = millis();
	}

	pendingDiagnostic_.code = static_cast<uint16_t>(code);
	pendingDiagnostic_.detail = detail;
	pendingDiagnostic_.counter = ++diagnosticCounter_;
	pendingDiagnostic_.timestampMs = timestampMs;
	diagnosticPending_ = true;
}

void TeensyLinkService::queueInputEvent(dukatimer::protocol::RemoteInputSource source,
	                                   dukatimer::protocol::InputEventKind eventKind, int16_t value,
	                                   uint32_t sourceTimestampMs) {
	if (sourceTimestampMs == 0u) {
		sourceTimestampMs = millis();
	}

	dukatimer::protocol::InputEventPayload payload = {};
	payload.source = static_cast<uint8_t>(source);
	payload.eventKind = static_cast<uint8_t>(eventKind);
	payload.value = value;
	payload.eventSequence = inputEventSequence_++;
	payload.sourceTimestampMs = sourceTimestampMs;

	const uint8_t bucketIndex = pendingInputBucketIndexForSource(source);
	if (bucketIndex >= kInputEventBucketCount) {
		return;
	}

	(void)enqueuePendingInputEvent(inputEventBuckets_[bucketIndex], payload);
}

bool TeensyLinkService::consumePendingCommand(dukatimer::protocol::TeensyCommandPayload* payloadOut) {
	if (commandQueueCount_ == 0u || payloadOut == nullptr) {
		return false;
	}

	*payloadOut = commandQueue_[commandQueueHead_];
	commandQueueHead_ = static_cast<uint8_t>((commandQueueHead_ + 1u) % kCommandQueueCapacity);
	--commandQueueCount_;
	return true;
}

bool TeensyLinkService::consumePendingRemoteDiagnostic(dukatimer::protocol::DiagnosticPayload* payloadOut) {
	if (!remoteDiagnosticPending_ || payloadOut == nullptr) {
		return false;
	}

	*payloadOut = lastRemoteDiagnostic_;
	remoteDiagnosticPending_ = false;
	return true;
}

bool TeensyLinkService::consumePendingRemoteRender(dukatimer::protocol::RemoteDisplayPayload* payloadOut) {
	if (!remoteRenderPending_ || payloadOut == nullptr) {
		return false;
	}

	*payloadOut = lastRemoteRender_;
	remoteRenderPending_ = false;
	return true;
}

bool TeensyLinkService::beginFileUpload(const char* path) {
	// Ein Upload startet nur aus einem ruhigen Uploadzustand und mit gueltigem
	// absoluten Zielpfad. Die eigentliche Dateibedeutung bleibt dem Teensy-VFS ueberlassen.
	if (!initialized_ || path == nullptr || !isAllowedVfsPath(path)) {
		return false;
	}

	const size_t pathLength = strnlen(path, dukatimer::protocol::kVfsMaxPathLength + 1u);
	if (pathLength == 0u || pathLength > dukatimer::protocol::kVfsMaxPathLength) {
		return false;
	}

	if (downloadChunkPending_ || (downloadPhase_ != DownloadPhase::Idle &&
	                           downloadPhase_ != DownloadPhase::Completed &&
	                           downloadPhase_ != DownloadPhase::Fault)) {
		return false;
	}

	if (uploadPhase_ != UploadPhase::Idle && uploadPhase_ != UploadPhase::Completed &&
	    uploadPhase_ != UploadPhase::Fault) {
		return false;
	}

	activeVfsTransactionId_ = nextVfsTransactionId_++;
	nextVfsOffset_ = 0;
	inFlightChunkOffset_ = 0;
	inFlightChunkLength_ = 0;
	inFlightChunkFinal_ = false;
	abortRequested_ = false;
	lastVfsAck_ = {};
	lastVfsError_ = {};
	pendingOpenRequest_ = {};
	pendingOpenRequest_.operation = static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileWriteRequest);
	pendingOpenRequest_.flags = dukatimer::protocol::kVfsRequestNone;
	pendingOpenRequest_.pathLength = static_cast<uint16_t>(pathLength);
	pendingOpenRequest_.transactionId = activeVfsTransactionId_;
	memcpy(pendingOpenRequest_.path, path, pathLength);
	openRequestPending_ = true;
	chunkPending_ = false;
	downloadChunkPending_ = false;
	lastDownloadChunk_ = {};
	activeDownloadFileSize_ = 0u;
	activeDownloadRequestedOffset_ = 0u;
	activeDownloadRequestedLength_ = 0u;
	downloadPhase_ = DownloadPhase::Idle;
	uploadPhase_ = UploadPhase::WaitingOpenAck;
	return true;
}

bool TeensyLinkService::streamFileChunk(const uint8_t* data, size_t length, bool finalize) {
	// Chunks werden streng sequentiell vorbereitet. Solange ein Chunk oder Open-Request
	// noch in Flight ist, darf kein weiterer Block in die Transfermaschine gelangen.
	if (!initialized_ || uploadPhase_ != UploadPhase::Streaming || chunkPending_ ||
	    length > dukatimer::protocol::kVfsChunkDataSize) {
		return false;
	}

	pendingChunk_ = {};
	pendingChunk_.transactionId = activeVfsTransactionId_;
	pendingChunk_.fileOffset = nextVfsOffset_;
	pendingChunk_.dataLength = static_cast<uint16_t>(length);
	pendingChunk_.flags = finalize || length == 0u ? dukatimer::protocol::kVfsChunkFinalize : dukatimer::protocol::kVfsChunkNone;
	if (length > 0u && data != nullptr) {
		memcpy(pendingChunk_.data, data, length);
	}

	inFlightChunkOffset_ = nextVfsOffset_;
	inFlightChunkLength_ = static_cast<uint16_t>(length);
	inFlightChunkFinal_ = finalize || length == 0u;
	chunkPending_ = true;
	uploadPhase_ = UploadPhase::WaitingChunkAck;
	return true;
}

bool TeensyLinkService::finishFileUpload() {
	return streamFileChunk(nullptr, 0u, true);
}

bool TeensyLinkService::beginFileDownload(const char* path, uint32_t requestedOffset,
	                                   uint16_t requestedLength) {
	// Downloads bleiben fuer den ersten Produktdatenvertrag ein bewusst kleiner
	// Request/Response-Pfad: genau ein angeforderter Slice pro Request, kein
	// zweiter langlaufender Stream-Automat neben dem bestehenden Uploadpfad.
	if (!initialized_ || path == nullptr || !isAllowedVfsPath(path) ||
	    requestedLength > dukatimer::protocol::kVfsChunkDataSize) {
		return false;
	}

	const size_t pathLength = strnlen(path, dukatimer::protocol::kVfsMaxPathLength + 1u);
	if (pathLength == 0u || pathLength > dukatimer::protocol::kVfsMaxPathLength) {
		return false;
	}

	if (chunkPending_ || openRequestPending_ ||
	    (uploadPhase_ != UploadPhase::Idle && uploadPhase_ != UploadPhase::Completed &&
	     uploadPhase_ != UploadPhase::Fault) ||
	    downloadChunkPending_ ||
	    (downloadPhase_ != DownloadPhase::Idle && downloadPhase_ != DownloadPhase::Completed &&
	     downloadPhase_ != DownloadPhase::Fault)) {
		return false;
	}

	activeVfsTransactionId_ = nextVfsTransactionId_++;
	abortRequested_ = false;
	lastVfsAck_ = {};
	lastVfsError_ = {};
	lastDownloadChunk_ = {};
	pendingOpenRequest_ = {};
	pendingOpenRequest_.operation = static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileReadRequest);
	pendingOpenRequest_.flags = dukatimer::protocol::kVfsRequestNone;
	pendingOpenRequest_.pathLength = static_cast<uint16_t>(pathLength);
	pendingOpenRequest_.transactionId = activeVfsTransactionId_;
	pendingOpenRequest_.requestedOffset = requestedOffset;
	pendingOpenRequest_.requestedLength = requestedLength;
	memcpy(pendingOpenRequest_.path, path, pathLength);
	openRequestPending_ = true;
	chunkPending_ = false;
	uploadPhase_ = UploadPhase::Idle;
	downloadPhase_ = DownloadPhase::WaitingOpenAck;
	downloadChunkPending_ = false;
	activeDownloadFileSize_ = 0u;
	activeDownloadRequestedOffset_ = requestedOffset;
	activeDownloadRequestedLength_ = requestedLength;
	return true;
}

bool TeensyLinkService::abortFileUpload() {
	// Abbruch behandelt sowohl den Sonderfall eines noch nicht bestaetigten Open-Requests
	// als auch einen bereits laufenden Chunk-Stream.
	if (!initialized_ || activeVfsTransactionId_ == 0u) {
		return false;
	}

	if (openRequestPending_) {
		openRequestPending_ = false;
		abortRequested_ = false;
		uploadPhase_ = UploadPhase::Fault;
		activeVfsTransactionId_ = 0u;
		nextVfsOffset_ = 0u;
		inFlightChunkOffset_ = 0u;
		inFlightChunkLength_ = 0u;
		inFlightChunkFinal_ = false;
		return true;
	}

	if (uploadPhase_ == UploadPhase::Idle || uploadPhase_ == UploadPhase::Completed ||
	    uploadPhase_ == UploadPhase::Fault) {
		return false;
	}

	pendingChunk_ = {};
	pendingChunk_.transactionId = activeVfsTransactionId_;
	pendingChunk_.fileOffset = nextVfsOffset_;
	pendingChunk_.dataLength = 0u;
	pendingChunk_.flags = dukatimer::protocol::kVfsChunkAbort;
	inFlightChunkOffset_ = nextVfsOffset_;
	inFlightChunkLength_ = 0u;
	inFlightChunkFinal_ = false;
	chunkPending_ = true;
	abortRequested_ = true;
	uploadPhase_ = UploadPhase::WaitingChunkAck;
	return true;
}

bool TeensyLinkService::uploadReadyForNextChunk() const {
	return uploadPhase_ == UploadPhase::Streaming && !chunkPending_ && !openRequestPending_;
}

bool TeensyLinkService::downloadInProgress() const {
	return downloadPhase_ == DownloadPhase::WaitingOpenAck ||
	       downloadPhase_ == DownloadPhase::WaitingChunk;
}

bool TeensyLinkService::downloadCompletedSuccessfully() const {
	return downloadPhase_ == DownloadPhase::Completed;
}

bool TeensyLinkService::downloadFaulted() const {
	return downloadPhase_ == DownloadPhase::Fault;
}

bool TeensyLinkService::downloadChunkPending() const {
	return downloadChunkPending_;
}

bool TeensyLinkService::consumeDownloadedChunk(dukatimer::protocol::VfsChunkPayload* payloadOut) {
	if (!downloadChunkPending_ || payloadOut == nullptr) {
		return false;
	}

	*payloadOut = lastDownloadChunk_;
	downloadChunkPending_ = false;
	lastDownloadChunk_ = {};
	if (downloadPhase_ == DownloadPhase::Completed) {
		downloadPhase_ = DownloadPhase::Idle;
	}
	return true;
}

uint32_t TeensyLinkService::downloadFileSize() const {
	return activeDownloadFileSize_;
}

bool TeensyLinkService::uploadInProgress() const {
	return uploadPhase_ == UploadPhase::WaitingOpenAck || uploadPhase_ == UploadPhase::Streaming ||
	       uploadPhase_ == UploadPhase::WaitingChunkAck;
}

bool TeensyLinkService::uploadCompletedSuccessfully() const {
	return uploadPhase_ == UploadPhase::Completed;
}

bool TeensyLinkService::uploadFaulted() const {
	return uploadPhase_ == UploadPhase::Fault;
}

uint32_t TeensyLinkService::uploadedByteCount() const {
	return nextVfsOffset_;
}

bool TeensyLinkService::teensyExposureActive(uint32_t nowMs) const {
	if (!teensyHeartbeatSeen_) {
		return false;
	}

	return teensyExposureActive_ && (nowMs - lastTeensyHeartbeatMs_) <= kTeensyHeartbeatStaleTimeoutMs;
}

const dukatimer::protocol::TeensyCommandPayload& TeensyLinkService::lastCommand() const {
	return lastCommand_;
}

const dukatimer::protocol::VfsAckPayload& TeensyLinkService::lastVfsAck() const {
	return lastVfsAck_;
}

const dukatimer::protocol::VfsErrorPayload& TeensyLinkService::lastVfsError() const {
	return lastVfsError_;
}

void TeensyLinkService::sendHeartbeat(uint32_t nowMs) {
	dukatimer::protocol::HeartbeatPayload payload;
	payload.nodeRole = static_cast<uint8_t>(dukatimer::protocol::NodeRole::Esp32Service);
	payload.bootState = static_cast<uint8_t>(dukatimer::protocol::BootState::Ready);
	payload.firmwareMajor = static_cast<uint8_t>(build::kVersionMajor);
	payload.firmwareMinor = static_cast<uint8_t>(build::kVersionMinor);
	payload.firmwarePatch = static_cast<uint8_t>(build::kVersionPatch);
	payload.runtimeFlags = fileTransactionActive_
		? dukatimer::service_link_runtime::kFileTransactionActive
		: dukatimer::protocol::kHeartbeatRuntimeNone;
	payload.uptimeMs = nowMs - bootStartedMs_;
	payload.capabilityBits = dukatimer::protocol::kCapabilityEncoder4 |
	                      dukatimer::protocol::kCapabilityOneWireThermal |
	                      dukatimer::protocol::kCapabilityWirelessGateway |
	                      dukatimer::protocol::kCapabilityAmbientSensor |
	                      dukatimer::protocol::kCapabilityVfsBridge;

	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::Heartbeat,
	                                                        dukatimer::protocol::NodeRole::Esp32Service,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
		return;
	}

	lastHeartbeatTxMs_ = nowMs;
}

void TeensyLinkService::sendServiceSnapshot(uint32_t nowMs) {
	// Der Service-Snapshot ist das autoritative ESP-Zwischenmodell fuer Thermik,
	// AHT20/BMP280 und den Wireless-Gatewayzustand. Der Teensy interpretiert diese
	// Flags spaeter weiter, liest die Sensoren aber nicht selbst.
	dukatimer::protocol::EspServiceSnapshotPayload payload;
	payload.serviceUptimeMs = nowMs - bootStartedMs_;
	payload.ds18TemperatureCentiC = ds18TemperatureCentiC_;
	payload.ds18SampleAgeMs = (ds18Present_ && ds18SampleTimestampMs_ > 0u) ? (nowMs - ds18SampleTimestampMs_) : 0u;
	payload.ahtTemperatureCentiC = ahtTemperatureCentiC_;
	payload.ahtHumidityCentiPercent = ahtHumidityCentiPercent_;
	payload.bmpTemperatureCentiC = bmpTemperatureCentiC_;
	payload.bmpPressureDeciHpa = bmpPressureDeciHpa_;
	payload.paperSlotRecoveryFlags = paperSlotRecoveryFlags_;
	payload.paperSlotRecoveryRecommendation =
		static_cast<uint8_t>(paperSlotRecoveryRecommendation_);
	payload.paperSlotActiveVfsStatus = static_cast<uint8_t>(paperSlotActiveVfsStatus_);
	payload.paperSlotActiveParseError = paperSlotActiveParseError_;
	payload.paperSlotBackupVfsStatus = static_cast<uint8_t>(paperSlotBackupVfsStatus_);
	payload.paperSlotBackupParseError = paperSlotBackupParseError_;

	if (ds18Present_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorDs18Present;
	}
	if (ds18Fault_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorDs18Fault;
	}
	if (oneWireOnline_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorOneWireOnline;
	}
	if (ahtPresent_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorAhtPresent;
	}
	if (ahtFault_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorAhtFault;
	}
	if (bmpPresent_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorBmpPresent;
	}
	if (bmpFault_) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorBmpFault;
	}
	if (wirelessPeerState_ == dukatimer::protocol::WirelessPeerState::Online ||
	    wirelessPeerState_ == dukatimer::protocol::WirelessPeerState::Measuring) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorWirelessOnline;
	}
	if (wirelessPeerState_ == dukatimer::protocol::WirelessPeerState::Fault) {
		payload.sensorFlags |= dukatimer::protocol::kServiceSensorWirelessFault;
	}

	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::EspServiceSnapshot,
	                                                        dukatimer::protocol::NodeRole::Esp32Service,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
		return;
	}

	lastServiceSnapshotTxMs_ = nowMs;
}

void TeensyLinkService::sendWirelessSnapshot(uint32_t nowMs) {
	dukatimer::protocol::WirelessSnapshotPayload payload;
	payload.peerState = static_cast<uint8_t>(wirelessPeerState_);
	payload.batteryPercent = wirelessBatteryPercent_;
	payload.measurementSequence = wirelessMeasurementSequence_;
	payload.commandAckSequence = wirelessCommandAckSequence_;
	payload.renderStatusFlags = wirelessRenderStatusFlags_;
	payload.staleRenderCount = wirelessStaleRenderCount_;
	payload.renderTimeoutCount = wirelessRenderTimeoutCount_;
	payload.lastLuxMilliLux = wirelessLastLuxMilliLux_;
	payload.lastSeenAgeMs = wirelessLastSeenTimestampMs_ > 0u ? (nowMs - wirelessLastSeenTimestampMs_) : 0u;
	payload.sensorSampleAgeMs =
		wirelessLastLuxSampleTimestampMs_ > 0u ? (nowMs - wirelessLastLuxSampleTimestampMs_) : 0u;
	if (wirelessLuxValid_) {
		payload.flags |= dukatimer::protocol::kWirelessSnapshotLuxValid;
	}
	if (hasPendingInputBuckets(inputEventBuckets_, kInputEventBucketCount)) {
		payload.flags |= dukatimer::protocol::kWirelessSnapshotInputPending;
	}
	if (wirelessBatteryPercent_ > 0u) {
		payload.flags |= dukatimer::protocol::kWirelessSnapshotBatteryKnown;
	}

	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::WirelessSnapshot,
	                                                        dukatimer::protocol::NodeRole::Esp32Service,
	                                                        txSequence_++, payload, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
		return;
	}

	lastWirelessSnapshotTxMs_ = nowMs;
}

void TeensyLinkService::sendPendingDiagnostic(uint32_t nowMs) {
	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::Diagnostic,
	                                                        dukatimer::protocol::NodeRole::Esp32Service,
	                                                        txSequence_++, pendingDiagnostic_, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
		return;
	}

	diagnosticPending_ = false;
	(void)nowMs;
}

void TeensyLinkService::sendPendingInputEvent(uint32_t nowMs) {
	// Remote-Input wird pro Quelle begrenzt gesammelt und round-robin versendet.
	// Damit bleibt die Warteschlange mathematisch begrenzt, ohne dass eine
	// einzelne Quelle den Linkpfad monopolisiert.
	if (!hasPendingInputBuckets(inputEventBuckets_, kInputEventBucketCount)) {
		return;
	}

	for (uint8_t offset = 0; offset < kInputEventBucketCount; ++offset) {
		const uint8_t bucketIndex = static_cast<uint8_t>((nextInputEventBucketCursor_ + offset) %
		                                               kInputEventBucketCount);
		TeensyLinkPendingInputBucket& bucket = inputEventBuckets_[bucketIndex];
		if (!bucket.hasPending()) {
			continue;
		}

		dukatimer::protocol::InputEventPayload pendingInputEvent = {};
		if (!popPendingInputEvent(bucket, &pendingInputEvent)) {
			continue;
		}

		const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::InputEvent,
		                                                        dukatimer::protocol::NodeRole::Esp32Service,
		                                                        txSequence_++, pendingInputEvent, frameBuffer_, sizeof(frameBuffer_));
		if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
			(void)enqueuePendingInputEvent(bucket, pendingInputEvent);
			return;
		}

		if (!bucket.hasPending()) {
			bucket.clear();
		}
		nextInputEventBucketCursor_ = static_cast<uint8_t>((bucketIndex + 1u) % kInputEventBucketCount);
		(void)nowMs;
		return;
	}
}

void TeensyLinkService::sendPendingOpenRequest(uint32_t nowMs) {
	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::VfsRequest,
	                                                        dukatimer::protocol::NodeRole::Esp32Service,
	                                                        txSequence_++, pendingOpenRequest_, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
		return;
	}

	openRequestPending_ = false;
	(void)nowMs;
}

void TeensyLinkService::sendPendingChunk(uint32_t nowMs) {
	const size_t frameSize = dukatimer::protocol::encodeFrame(dukatimer::protocol::MessageType::VfsChunk,
	                                                        dukatimer::protocol::NodeRole::Esp32Service,
	                                                        txSequence_++, pendingChunk_, frameBuffer_, sizeof(frameBuffer_));
	if (frameSize == 0u || !tryWriteFrame(frameBuffer_, frameSize)) {
		return;
	}

	chunkPending_ = false;
	(void)nowMs;
}

void TeensyLinkService::readFrames(uint32_t nowMs) {
	// Wie auf Teensy-Seite wird der serielle Eingangsring ohne Wartefenster entleert;
	// vollstaendige Frames werden sofort in den Zustandsautomaten eingespeist.
	dukatimer::protocol::DecodedFrame frame;
	while (serial_.available() > 0) {
		const int value = serial_.read();
		if (value < 0) {
			break;
		}

		if (decoder_.push(static_cast<uint8_t>(value), &frame)) {
			handleFrame(frame, nowMs);
		}
	}
}

void TeensyLinkService::handleFrame(const dukatimer::protocol::DecodedFrame& frame, uint32_t nowMs) {
	switch (frame.messageType) {
		case dukatimer::protocol::MessageType::Heartbeat: {
			dukatimer::protocol::HeartbeatPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload) &&
			    payload.nodeRole == static_cast<uint8_t>(dukatimer::protocol::NodeRole::Teensy)) {
				teensyHeartbeatSeen_ = true;
				lastTeensyHeartbeatMs_ = nowMs;
				teensyExposureActive_ =
					(payload.runtimeFlags & dukatimer::protocol::kHeartbeatRuntimeExposureActive) != 0u;
			}
			break;
		}

		case dukatimer::protocol::MessageType::TeensyCommand: {
			dukatimer::protocol::TeensyCommandPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyTeensyCommand(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::RemoteRender: {
			dukatimer::protocol::RemoteDisplayPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyRemoteRender(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::VfsChunk: {
			dukatimer::protocol::VfsChunkPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				handleVfsChunk(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::VfsAck: {
			dukatimer::protocol::VfsAckPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				handleVfsAck(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::VfsError: {
			dukatimer::protocol::VfsErrorPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				handleVfsError(payload);
			}
			break;
		}

		case dukatimer::protocol::MessageType::EspServiceSnapshot:
		case dukatimer::protocol::MessageType::InputEvent:
		case dukatimer::protocol::MessageType::WirelessSnapshot:
		case dukatimer::protocol::MessageType::Diagnostic:
			dukatimer::protocol::DiagnosticPayload payload;
			if (dukatimer::protocol::extractPayload(frame, &payload)) {
				applyRemoteDiagnostic(payload);
			}
			break;
	}
}

void TeensyLinkService::applyTeensyCommand(const dukatimer::protocol::TeensyCommandPayload& payload) {
	lastCommand_ = payload;
	if (commandQueueCount_ >= kCommandQueueCapacity) {
		++droppedCommandCount_;
		maybeReportDropDiagnostic(dukatimer::protocol::DiagnosticCode::LinkCommandDropped,
		                        droppedCommandCount_, millis());
		return;
	}

	const uint8_t tailIndex = static_cast<uint8_t>((commandQueueHead_ + commandQueueCount_) %
	                                             kCommandQueueCapacity);
	commandQueue_[tailIndex] = payload;
	++commandQueueCount_;
}

void TeensyLinkService::applyRemoteDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload) {
	// Remote-Diagnosen vom Teensy werden fuer den Wireless-Gatewaypfad nur als
	// letztes bekanntes Ereignis gehalten. Die Zaehler im Payload bleiben kumulativ,
	// daher bleibt selbst bei Coalescing der aktuelle Counterstand sichtbar.
	lastRemoteDiagnostic_ = payload;
	remoteDiagnosticPending_ = true;
}

	void TeensyLinkService::applyRemoteRender(const dukatimer::protocol::RemoteDisplayPayload& payload) {
	if (remoteRenderPending_) {
		++coalescedRemoteRenderCount_;
		maybeReportDropDiagnostic(dukatimer::protocol::DiagnosticCode::LinkRenderCoalesced,
		                        coalescedRemoteRenderCount_, millis());
	}

	lastRemoteRender_ = payload;
	remoteRenderPending_ = true;
}

bool TeensyLinkService::tryWriteFrame(const uint8_t* frameData, size_t frameSize) {
	if (frameData == nullptr || frameSize == 0u) {
		return false;
	}

	if (serial_.availableForWrite() < static_cast<int>(frameSize)) {
		return false;
	}

	return serial_.write(frameData, frameSize) == frameSize;
}

void TeensyLinkService::maybeReportDropDiagnostic(dukatimer::protocol::DiagnosticCode code,
	                                             uint32_t occurrenceCount,
	                                             uint32_t nowMs) {
	if (occurrenceCount != 1u && (occurrenceCount % 16u) != 0u) {
		return;
	}

	const uint16_t detail = occurrenceCount > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(occurrenceCount);
	queueDiagnostic(code, detail, nowMs);
}

void TeensyLinkService::handleVfsChunk(const dukatimer::protocol::VfsChunkPayload& payload) {
	// Ein Download-Request erwartet genau einen Chunk fuer den angefragten Slice.
	// Abweichende Offsets oder Laengen sind deshalb kein stiller Sonderfall,
	// sondern ein klarer Protokollfehler im ersten Exportpfad.
	if (downloadPhase_ != DownloadPhase::WaitingChunk || payload.transactionId != activeVfsTransactionId_) {
		return;
	}

	const uint16_t expectedLength = activeDownloadRequestedLength_ == 0u
		? dukatimer::protocol::kVfsChunkDataSize
		: activeDownloadRequestedLength_;
	const bool offsetMatches = payload.fileOffset == activeDownloadRequestedOffset_;
	const bool lengthMatches = payload.dataLength <= expectedLength;
	const bool zeroLengthIsFinal = payload.dataLength != 0u ||
		((payload.flags & dukatimer::protocol::kVfsChunkFinalize) != 0u);
	const bool withinFile = payload.fileOffset <= activeDownloadFileSize_ &&
		(payload.fileOffset + payload.dataLength) <= activeDownloadFileSize_;
	if (!offsetMatches || !lengthMatches || !zeroLengthIsFinal || !withinFile) {
		lastVfsError_ = {};
		lastVfsError_.transactionId = payload.transactionId;
		lastVfsError_.statusCode = static_cast<uint16_t>(dukatimer::protocol::VfsStatusCode::ProtocolRejected);
		lastVfsError_.detail = payload.dataLength;
		lastVfsError_.fileOffset = payload.fileOffset;
		lastVfsError_.info = activeDownloadRequestedOffset_;
		downloadPhase_ = DownloadPhase::Fault;
		return;
	}

	lastDownloadChunk_ = payload;
	downloadChunkPending_ = true;
	downloadPhase_ = DownloadPhase::Completed;
}

void TeensyLinkService::handleVfsAck(const dukatimer::protocol::VfsAckPayload& payload) {
	// Acks treiben den Upload-Zustandsautomaten voran. Nur die aktive Transaktion
	// darf den lokalen Uploadzustand veraendern.
	if (payload.transactionId != activeVfsTransactionId_) {
		return;
	}

	lastVfsAck_ = payload;
	if (payload.statusCode != static_cast<uint8_t>(dukatimer::protocol::VfsStatusCode::Ok)) {
		abortRequested_ = false;
		if (downloadPhase_ == DownloadPhase::WaitingOpenAck ||
		    downloadPhase_ == DownloadPhase::WaitingChunk) {
			downloadPhase_ = DownloadPhase::Fault;
		} else {
			uploadPhase_ = UploadPhase::Fault;
		}
		return;
	}

	if (abortRequested_) {
		if (payload.operation == static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileWriteRequest)) {
			nextVfsOffset_ = payload.acknowledgedOffset + payload.acknowledgedLength;
		}
		return;
	}

	if (uploadPhase_ == UploadPhase::WaitingOpenAck &&
	    payload.operation == static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileWriteRequest)) {
		uploadPhase_ = UploadPhase::Streaming;
		return;
	}

	if (uploadPhase_ == UploadPhase::WaitingChunkAck) {
		nextVfsOffset_ = payload.acknowledgedOffset + payload.acknowledgedLength;
		uploadPhase_ = inFlightChunkFinal_ ? UploadPhase::Completed : UploadPhase::Streaming;
		return;
	}

	if (downloadPhase_ == DownloadPhase::WaitingOpenAck &&
	    payload.operation == static_cast<uint8_t>(dukatimer::protocol::VfsOperation::FileReadRequest)) {
		activeDownloadFileSize_ = payload.detail;
		downloadPhase_ = DownloadPhase::WaitingChunk;
	}
}

void TeensyLinkService::handleVfsError(const dukatimer::protocol::VfsErrorPayload& payload) {
	// Jeder Fehler der aktiven Transaktion fuehrt den Uploadpfad hart in Fault.
	if (payload.transactionId != activeVfsTransactionId_) {
		return;
	}

	lastVfsError_ = payload;
	abortRequested_ = false;
	if (downloadPhase_ == DownloadPhase::WaitingOpenAck ||
	    downloadPhase_ == DownloadPhase::WaitingChunk) {
		downloadPhase_ = DownloadPhase::Fault;
		return;
	}

	uploadPhase_ = UploadPhase::Fault;
}

}  // namespace dukatimer