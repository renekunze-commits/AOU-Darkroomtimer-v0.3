/*
 * DukatimerProtocol
 *
 * Gemeinsames serielles Transport- und Payloadmodell fuer Teensy, ESP32-S3 und
 * moegliche weitere Knoten wie das Wireless-Remote. Diese Datei definiert sowohl
 * den Byte-Rahmen eines Frames als auch die fachlichen Payloadstrukturen der
 * Inter-MCU- und Gateway-Kommunikation.
 *
 * Architekturgrenze:
 * - das Protokoll beschreibt Transport, Layout und kompakte Uebergabedaten
 * - es enthaelt keine Workflowlogik, keine UI-Semantik und keine direkte Hardware-
 *   steuerung oberhalb des reinen Nachrichtenverkehrs
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <array>
#include <type_traits>

namespace dukatimer::protocol {

// Basisparameter des seriellen Rahmens. kMaxPayloadSize muss gross genug fuer
// den groessten Transportfall (derzeit VFS-Chunk) bleiben.
constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kFrameSync0 = 0xAA;
constexpr uint8_t kFrameSync1 = 0x55;
constexpr size_t kMaxPayloadSize = 1056;
constexpr uint16_t kVfsMaxPathLength = 240;
constexpr uint16_t kVfsChunkDataSize = 1024;

// Nachrichtentypen des SharedProtocol. Jeder Typ belegt genau einen Frame und
// wird spaeter von den Linkdiensten strikt getrennt dispatcht.
enum class MessageType : uint8_t {
	Heartbeat = 1,
	EspServiceSnapshot = 2,
	InputEvent = 3,
	WirelessSnapshot = 4,
	TeensyCommand = 5,
	Diagnostic = 6,
	VfsRequest = 7,
	VfsChunk = 8,
	VfsAck = 9,
	VfsError = 10,
	RemoteRender = 11,
};

// Rolle des sendenden Knotens. Dieselbe Frame- und Payloadsprache kann dadurch
// fuer mehrere MCU- oder Remote-Typen wiederverwendet werden.
enum class NodeRole : uint8_t {
	Unknown = 0,
	Teensy = 1,
	Esp32Service = 2,
	WirelessRemote = 3,
};

// Grober Boot-/Betriebszustand eines Knotens fuer Heartbeats.
enum class BootState : uint8_t {
	Booting = 0,
	Ready = 1,
	Fault = 2,
};

// Herkunft entfernter Eingaben, bevor sie auf Teensy-Seite normalisiert werden.
enum class RemoteInputSource : uint8_t {
	None = 0,
	Encoder4 = 1,
	WirelessEncoder = 2,
	WirelessMeasureButton = 3,
	WirelessBackButton = 4,
	WirelessEncoderButton = 5,
};

// Grundgestalt eines uebertragenen Remote-Eingabeereignisses.
enum class InputEventKind : uint8_t {
	None = 0,
	RotateLeft = 1,
	RotateRight = 2,
	Press = 3,
	LongPress = 4,
	Measure = 5,
	Undo = 6,
};

// Zusammengefasster Zustand des Wireless-Messpartners/Gateways.
enum class WirelessPeerState : uint8_t {
	Unknown = 0,
	Offline = 1,
	Online = 2,
	Measuring = 3,
	Fault = 4,
};

// Vorbereitete Teensy-Zielkommandos fuer spaetere Remote-/Service-Erweiterungen.
enum class TeensyCommandKind : uint8_t {
	None = 0,
	Ping = 1,
	RemoteRender = 2,
	RemoteHaptic = 3,
	RemoteMeasurementStart = 4,
	RemoteMeasurementCancel = 5,
};

// Anzeige- und Haptikzustand fuer das Wireless-TSL2591-Handgeraet.
enum class RemoteRenderDisplayMode : uint8_t {
	Idle = 0,
	MeterBw = 1,
	MeterSg = 2,
	Burn = 3,
	Calibrate = 4,
	Densitometer = 5,
};

enum class RemoteHapticFeedback : uint8_t {
	None = 0,
	Click = 1,
	Error = 2,
	Done = 3,
};

// Verdichtete Diagnosecodes fuer sichtbare Fehlertelemetrie entlang des
// ESP-Service-, Sensor- und Wireless-Gatewaypfads.
enum class DiagnosticCode : uint16_t {
	None = 0,
	ServiceDs18Fault = 1,
	ServiceAhtFault = 2,
	WirelessGatewayInitFailed = 3,
	WirelessMeasurementTimeout = 4,
	WirelessSendFailed = 5,
	LinkRenderCoalesced = 6,
	LinkCommandDropped = 7,
	LinkInputDropped = 8,
	RemoteCommandRetry = 9,
	RemoteCommandTrackerSaturated = 10,
	RemoteCommandTimeout = 11,
	WirelessRenderStale = 12,
	WirelessRenderTimeout = 13,
	ServiceBmpFault = 14,
};

// Dateisystemoperationen der seriellen VFS-Bruecke.
enum class VfsOperation : uint8_t {
	None = 0,
	FileWriteRequest = 1,
	FileReadRequest = 2,
	DirectoryCreate = 3,
	FileDelete = 4,
	TransferClose = 5,
};

// Verdichtete Rueckmeldungen des VFS-Brueckenpfads.
enum class VfsStatusCode : uint8_t {
	None = 0,
	Ok = 1,
	Busy = 2,
	InvalidPath = 3,
	SdUnavailable = 4,
	Unsupported = 5,
	OpenFailed = 6,
	WriteFailed = 7,
	ReadFailed = 8,
	InvalidState = 9,
	TransactionMismatch = 10,
	StorageNotReady = 11,
	FlowHeld = 12,
	ProtocolRejected = 13,
};

// Verdichtete Recovery-Empfehlung fuer den PaperSlot-Produktdatenpfad. Die
// Empfehlung bleibt rein diagnostisch; der Teensy darf daraus Hinweise ableiten,
// aber keine automatische Datenumschaltung vornehmen.
enum class PaperSlotRecoveryRecommendation : uint8_t {
	Unknown = 0,
	RetryInspection = 1,
	NoAction = 2,
	KeepActive = 3,
	OfferRestoreBackup = 4,
	RecoverViaUpload = 5,
};

// CapabilityBits beschreiben die durch einen Heartbeat beworbenen Rollen und
// Zusatzfaehigkeiten eines Knotens.
enum CapabilityBits : uint32_t {
	kCapabilityEncoder4 = 1u << 0,
	kCapabilityOneWireThermal = 1u << 1,
	kCapabilityWirelessGateway = 1u << 2,
	kCapabilityAmbientSensor = 1u << 3,
	kCapabilityAudioHaptic = 1u << 4,
	kCapabilityRemoteRender = 1u << 5,
	kCapabilityVfsBridge = 1u << 6,
};

// Sensor- und Serviceflags des ESP32-S3-Snapshoteintrags.
enum ServiceSensorFlags : uint16_t {
	kServiceSensorNone = 0,
	kServiceSensorDs18Present = 1u << 0,
	kServiceSensorDs18Fault = 1u << 1,
	kServiceSensorOneWireOnline = 1u << 2,
	kServiceSensorAhtPresent = 1u << 3,
	kServiceSensorAhtFault = 1u << 4,
	kServiceSensorWirelessOnline = 1u << 5,
	kServiceSensorWirelessFault = 1u << 6,
	kServiceSensorBmpPresent = 1u << 7,
	kServiceSensorBmpFault = 1u << 8,
};

// Zusatzflags fuer den Wireless-Snapshot.
enum WirelessSnapshotFlags : uint16_t {
	kWirelessSnapshotNone = 0,
	kWirelessSnapshotLuxValid = 1u << 0,
	kWirelessSnapshotInputPending = 1u << 1,
	kWirelessSnapshotBatteryKnown = 1u << 2,
};

// Zusatzzustand des Wireless-Renderpfads vom C6-Terminal aus gesehen.
enum WirelessRenderStatusFlags : uint16_t {
	kWirelessRenderStatusNone = 0,
	kWirelessRenderStatusTimeoutActive = 1u << 0,
};

enum PaperSlotRecoveryFlags : uint16_t {
	kPaperSlotRecoveryNone = 0,
	kPaperSlotRecoveryDecisionStable = 1u << 0,
	kPaperSlotRecoveryActiveFetchSucceeded = 1u << 1,
	kPaperSlotRecoveryActiveBlobValid = 1u << 2,
	kPaperSlotRecoveryBackupFetchSucceeded = 1u << 3,
	kPaperSlotRecoveryBackupBlobValid = 1u << 4,
	kPaperSlotRecoveryRestoreWouldChangeActive = 1u << 5,
	kPaperSlotRecoverySamePayloadSignature = 1u << 6,
	kPaperSlotRecoveryManualRecoverUploadSuggested = 1u << 7,
};

enum VfsRequestFlags : uint8_t {
	kVfsRequestNone = 0,
	kVfsRequestCreateParents = 1u << 0,
};

enum VfsChunkFlags : uint16_t {
	kVfsChunkNone = 0,
	kVfsChunkFinalize = 1u << 0,
	kVfsChunkAbort = 1u << 1,
};

enum WirelessRemoteButtonBits : uint8_t {
	kWirelessRemoteButtonNone = 0,
	kWirelessRemoteButtonMeasure = 1u << 0,
	kWirelessRemoteButtonBack = 1u << 1,
	kWirelessRemoteButtonEncoder = 1u << 2,
};

enum HeartbeatRuntimeFlags : uint8_t {
	kHeartbeatRuntimeNone = 0,
	kHeartbeatRuntimeExposureActive = 1u << 0,
};

#pragma pack(push, 1)

// Fester Frame-Header nach den beiden Sync-Bytes. Header und Payload werden
// gemeinsam durch die abschliessende CRC abgesichert.
struct FrameHeaderBody {
	uint8_t version = kProtocolVersion;
	uint8_t messageType = 0;
	uint16_t payloadSize = 0;
	uint32_t sequence = 0;
	uint8_t sourceRole = 0;
	uint8_t reserved = 0;
};

// HeartbeatPayload macht Identitaet, Softwarestand, Runtimeflags und beworbene
// Faehigkeiten eines Knotens periodisch sichtbar.
struct HeartbeatPayload {
	uint8_t nodeRole = static_cast<uint8_t>(NodeRole::Unknown);
	uint8_t bootState = static_cast<uint8_t>(BootState::Booting);
	uint8_t firmwareMajor = 0;
	uint8_t firmwareMinor = 0;
	uint8_t firmwarePatch = 0;
	uint8_t runtimeFlags = kHeartbeatRuntimeNone;
	uint32_t uptimeMs = 0;
	uint32_t capabilityBits = 0;
};

// Autoritativer ESP-Service-Snapshot fuer Thermik-, AHT20-, BMP280- und Gatewaydaten.
struct EspServiceSnapshotPayload {
	uint16_t sensorFlags = kServiceSensorNone;
	int16_t ds18TemperatureCentiC = 0;
	uint32_t ds18SampleAgeMs = 0;
	int16_t ahtTemperatureCentiC = 0;
	uint16_t ahtHumidityCentiPercent = 0;
	int16_t bmpTemperatureCentiC = 0;
	uint32_t bmpPressureDeciHpa = 0;
	uint32_t serviceUptimeMs = 0;
	uint16_t paperSlotRecoveryFlags = kPaperSlotRecoveryNone;
	uint8_t paperSlotRecoveryRecommendation =
		static_cast<uint8_t>(PaperSlotRecoveryRecommendation::Unknown);
	uint8_t paperSlotActiveVfsStatus = static_cast<uint8_t>(VfsStatusCode::None);
	uint8_t paperSlotActiveParseError = 0;
	uint8_t paperSlotBackupVfsStatus = static_cast<uint8_t>(VfsStatusCode::None);
	uint8_t paperSlotBackupParseError = 0;
};

// Ein einzelnes entfernt erzeugtes Eingabeereignis.
struct InputEventPayload {
	uint8_t source = static_cast<uint8_t>(RemoteInputSource::None);
	uint8_t eventKind = static_cast<uint8_t>(InputEventKind::None);
	int16_t value = 0;
	uint32_t eventSequence = 0;
	uint32_t sourceTimestampMs = 0;
};

// Kompakter Gateway-/Wireless-Zustand fuer Mess- und Peerstatus.
struct WirelessSnapshotPayload {
	uint8_t peerState = static_cast<uint8_t>(WirelessPeerState::Unknown);
	uint8_t batteryPercent = 0;
	uint16_t flags = kWirelessSnapshotNone;
	uint32_t lastSeenAgeMs = 0;
	uint32_t sensorSampleAgeMs = 0;
	uint32_t lastLuxMilliLux = 0;
	uint32_t measurementSequence = 0;
	uint32_t commandAckSequence = 0;
	uint16_t renderStatusFlags = kWirelessRenderStatusNone;
	uint32_t staleRenderCount = 0;
	uint32_t renderTimeoutCount = 0;
};

// Aggregierter Statusrahmen des Wireless-Terminals fuer ESP-NOW inklusive
// zuletzt im Loop wirklich angewendetem Kommando-Ack sowie lokaler
// Renderverlust-Diagnose vom C6.
struct WirelessRemotePayload {
	uint32_t sequenceNumber = 0;
	int16_t encoderDelta = 0;
	uint8_t activeButtons = kWirelessRemoteButtonNone;
	float activeLux = 0.0f;
	uint32_t activeLuxAgeMs = 0;
	uint32_t commandAckSequence = 0;
	uint16_t renderStatusFlags = kWirelessRenderStatusNone;
	uint32_t staleRenderCount = 0;
	uint32_t renderTimeoutCount = 0;
};

// Reservierter Kommandokanal vom ESP- oder Remote-Pfad in Richtung Teensy.
struct TeensyCommandPayload {
	uint8_t commandKind = static_cast<uint8_t>(TeensyCommandKind::None);
	uint8_t target = 0;
	uint16_t argument0 = 0;
	uint32_t argument1 = 0;
	uint32_t argument2 = 0;
	uint32_t commandSequence = 0;
};

// Datengetriebene Fernanzeige fuer das Wireless-TSL2591-Handgeraet.
struct RemoteDisplayPayload {
	uint32_t sequenceNumber = 0;
	uint8_t viewType = 0;
	char line1[24] = {};
	char line2[24] = {};
	char line3[24] = {};
	uint8_t progressPercent = 255;
};

// Kleiner Diagnoserahmen fuer spaetere Debug- oder Fehlerkanäle.
struct DiagnosticPayload {
	uint16_t code = 0;
	uint16_t detail = 0;
	uint32_t counter = 0;
	uint32_t timestampMs = 0;
};

// Oeffnet oder beschreibt eine VFS-Operation auf dem Zielknoten.
struct VfsRequestPayload {
	uint8_t operation = static_cast<uint8_t>(VfsOperation::None);
	uint8_t flags = kVfsRequestNone;
	uint16_t pathLength = 0;
	uint32_t transactionId = 0;
	uint32_t requestedOffset = 0;
	uint32_t requestedLength = 0;
	char path[kVfsMaxPathLength] = {};
};

// Trägt einen Datenblock einer laufenden VFS-Transaktion.
struct VfsChunkPayload {
	uint32_t transactionId = 0;
	uint32_t fileOffset = 0;
	uint16_t dataLength = 0;
	uint16_t flags = kVfsChunkNone;
	uint8_t data[kVfsChunkDataSize] = {};
};

// Positive Bestaetigung einer VFS-Operation oder eines Chunks.
struct VfsAckPayload {
	uint32_t transactionId = 0;
	uint32_t acknowledgedOffset = 0;
	uint16_t acknowledgedLength = 0;
	uint8_t statusCode = static_cast<uint8_t>(VfsStatusCode::None);
	uint8_t operation = static_cast<uint8_t>(VfsOperation::None);
	uint32_t detail = 0;
};

// Fehlerantwort einer VFS-Operation inklusive Zusatzkontext.
struct VfsErrorPayload {
	uint32_t transactionId = 0;
	uint16_t statusCode = static_cast<uint16_t>(VfsStatusCode::None);
	uint16_t detail = 0;
	uint32_t fileOffset = 0;
	uint32_t info = 0;
};

#pragma pack(pop)

static_assert(sizeof(FrameHeaderBody) == 10, "FrameHeaderBody layout changed");
static_assert(sizeof(HeartbeatPayload) <= kMaxPayloadSize, "Heartbeat payload too large");
static_assert(sizeof(EspServiceSnapshotPayload) <= kMaxPayloadSize, "ESP service payload too large");
static_assert(sizeof(InputEventPayload) <= kMaxPayloadSize, "Input event payload too large");
static_assert(sizeof(WirelessSnapshotPayload) <= kMaxPayloadSize, "Wireless payload too large");
static_assert(sizeof(WirelessRemotePayload) <= kMaxPayloadSize, "Wireless remote payload too large");
static_assert(sizeof(WirelessRemotePayload) == 29, "Wireless remote ESP-NOW ABI changed");
static_assert(sizeof(TeensyCommandPayload) <= kMaxPayloadSize, "Teensy command payload too large");
static_assert(sizeof(RemoteDisplayPayload) <= kMaxPayloadSize, "Remote display payload too large");
static_assert(sizeof(RemoteDisplayPayload) == 78, "Remote display ESP-NOW ABI changed");
static_assert(sizeof(DiagnosticPayload) <= kMaxPayloadSize, "Diagnostic payload too large");
static_assert(sizeof(VfsRequestPayload) <= kMaxPayloadSize, "VFS request payload too large");
static_assert(sizeof(VfsChunkPayload) <= kMaxPayloadSize, "VFS chunk payload too large");
static_assert(sizeof(VfsAckPayload) <= kMaxPayloadSize, "VFS ack payload too large");
static_assert(sizeof(VfsErrorPayload) <= kMaxPayloadSize, "VFS error payload too large");

struct DecodedFrame {
	MessageType messageType = MessageType::Heartbeat;
	NodeRole sourceRole = NodeRole::Unknown;
	uint32_t sequence = 0;
	uint16_t payloadSize = 0;
	std::array<uint8_t, kMaxPayloadSize> payload = {};
};

// CRC16-CCITT sichert Header und Payload eines Frames ab. Die zweistufige
// Variante erlaubt, Header und Payload ohne Zwischenkopie fortlaufend zu hashen.
inline uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
	uint16_t crc = 0xFFFFu;
	for (size_t index = 0; index < length; ++index) {
		crc ^= static_cast<uint16_t>(data[index]) << 8;
		for (uint8_t bit = 0; bit < 8; ++bit) {
			if ((crc & 0x8000u) != 0u) {
				crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
			} else {
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}
	return crc;
}

inline uint16_t crc16Ccitt(const uint8_t* data, size_t length, uint16_t initialCrc) {
	uint16_t crc = initialCrc;
	for (size_t index = 0; index < length; ++index) {
		crc ^= static_cast<uint16_t>(data[index]) << 8;
		for (uint8_t bit = 0; bit < 8; ++bit) {
			if ((crc & 0x8000u) != 0u) {
				crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
			} else {
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}
	return crc;
}

inline size_t encodeFrameRaw(MessageType messageType, NodeRole sourceRole, uint32_t sequence,
	                         const void* payload, uint16_t payloadSize, uint8_t* outputBuffer,
	                         size_t outputCapacity) {
	// encodeFrameRaw schreibt den kompletten Transportrahmen inklusive Sync-Bytes,
	// Header, Payload und CRC in einen vom Aufrufer gestellten Puffer.
	if (outputBuffer == nullptr || payloadSize > kMaxPayloadSize) {
		return 0;
	}

	const size_t totalSize = 2u + sizeof(FrameHeaderBody) + payloadSize + sizeof(uint16_t);
	if (outputCapacity < totalSize) {
		return 0;
	}

	FrameHeaderBody header;
	header.version = kProtocolVersion;
	header.messageType = static_cast<uint8_t>(messageType);
	header.payloadSize = payloadSize;
	header.sequence = sequence;
	header.sourceRole = static_cast<uint8_t>(sourceRole);

	outputBuffer[0] = kFrameSync0;
	outputBuffer[1] = kFrameSync1;
	memcpy(outputBuffer + 2u, &header, sizeof(header));
	if (payloadSize > 0u && payload != nullptr) {
		memcpy(outputBuffer + 2u + sizeof(header), payload, payloadSize);
	}

	const uint16_t crc = crc16Ccitt(outputBuffer + 2u, sizeof(header) + payloadSize);
	outputBuffer[totalSize - 2u] = static_cast<uint8_t>(crc & 0xFFu);
	outputBuffer[totalSize - 1u] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
	return totalSize;
}

template <typename PayloadT>
inline size_t encodeFrame(MessageType messageType, NodeRole sourceRole, uint32_t sequence,
	                      const PayloadT& payload, uint8_t* outputBuffer,
	                      size_t outputCapacity) {
	static_assert(std::is_trivially_copyable<PayloadT>::value, "Payload must be trivially copyable");
	return encodeFrameRaw(messageType, sourceRole, sequence, &payload, static_cast<uint16_t>(sizeof(PayloadT)),
	                      outputBuffer, outputCapacity);
}

template <typename PayloadT>
inline bool extractPayload(const DecodedFrame& frame, PayloadT* payloadOut) {
	static_assert(std::is_trivially_copyable<PayloadT>::value, "Payload must be trivially copyable");
	// extractPayload erzwingt, dass Frametyp und Zielstruktur in ihrer Groesse exakt
	// zusammenpassen. Dadurch bleibt die Protokollgrenze streng layoutgebunden.
	if (payloadOut == nullptr || frame.payloadSize != sizeof(PayloadT)) {
		return false;
	}

	memcpy(payloadOut, frame.payload.data(), sizeof(PayloadT));
	return true;
}

class FrameDecoder {
public:
	// push() verarbeitet genau ein neues Byte des seriellen Datenstroms und liefert
	// erst bei einem vollstaendigen, CRC-gueltigen Frame ein Ergebnis zurueck.
	bool push(uint8_t byte, DecodedFrame* decodedFrame) {
		switch (state_) {
			case State::Sync0:
				if (byte == kFrameSync0) {
					state_ = State::Sync1;
				}
				return false;

			case State::Sync1:
				if (byte == kFrameSync1) {
					state_ = State::Header;
					headerIndex_ = 0;
					payloadIndex_ = 0;
					return false;
				}
				state_ = (byte == kFrameSync0) ? State::Sync1 : State::Sync0;
				return false;

			case State::Header:
				headerBuffer_[headerIndex_++] = byte;
				if (headerIndex_ < sizeof(FrameHeaderBody)) {
					return false;
				}

				memcpy(&header_, headerBuffer_.data(), sizeof(header_));
				if (header_.version != kProtocolVersion || header_.payloadSize > kMaxPayloadSize) {
					reset();
					return false;
				}

				state_ = header_.payloadSize == 0u ? State::Crc0 : State::Payload;
				payloadIndex_ = 0;
				return false;

			case State::Payload:
				payloadBuffer_[payloadIndex_++] = byte;
				if (payloadIndex_ >= header_.payloadSize) {
					state_ = State::Crc0;
				}
				return false;

			case State::Crc0:
				receivedCrc_ = byte;
				state_ = State::Crc1;
				return false;

			case State::Crc1:
				receivedCrc_ |= static_cast<uint16_t>(byte) << 8;
				return finalize(decodedFrame);
		}

		reset();
		return false;
	}

	void reset() {
		state_ = State::Sync0;
		headerIndex_ = 0;
		payloadIndex_ = 0;
		receivedCrc_ = 0;
		header_ = FrameHeaderBody{};
	}

private:
	enum class State : uint8_t {
		Sync0,
		Sync1,
		Header,
		Payload,
		Crc0,
		Crc1,
	};

	// finalize() prueft nach vollstaendiger Header-/Payload-Aufnahme die CRC und
	// materialisiert erst dann den sichtbaren DecodedFrame.
	bool finalize(DecodedFrame* decodedFrame) {
		uint16_t expectedCrc = crc16Ccitt(reinterpret_cast<const uint8_t*>(&header_), sizeof(header_));
		if (header_.payloadSize > 0u) {
			expectedCrc = crc16Ccitt(payloadBuffer_.data(), header_.payloadSize, expectedCrc);
		}

		const bool crcMatches = expectedCrc == receivedCrc_;
		if (crcMatches && decodedFrame != nullptr) {
			decodedFrame->messageType = static_cast<MessageType>(header_.messageType);
			decodedFrame->sourceRole = static_cast<NodeRole>(header_.sourceRole);
			decodedFrame->sequence = header_.sequence;
			decodedFrame->payloadSize = header_.payloadSize;
			decodedFrame->payload.fill(0);
			if (header_.payloadSize > 0u) {
				memcpy(decodedFrame->payload.data(), payloadBuffer_.data(), header_.payloadSize);
			}
		}

		reset();
		return crcMatches;
	}

	State state_ = State::Sync0;
	FrameHeaderBody header_ = {};
	std::array<uint8_t, sizeof(FrameHeaderBody)> headerBuffer_ = {};
	std::array<uint8_t, kMaxPayloadSize> payloadBuffer_ = {};
	size_t headerIndex_ = 0;
	size_t payloadIndex_ = 0;
	uint16_t receivedCrc_ = 0;
};

}  // namespace dukatimer::protocol
