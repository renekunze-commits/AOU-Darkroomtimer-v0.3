#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kNormalizedInputEventSchemaVersion = 1;

// Herkunft eines bereits hardwareverdichteten Eingabeereignisses.
enum class NormalizedInputSource : uint8_t {
	None,
	LocalEncoder1,
	LocalEncoder2,
	LocalEncoder3,
	LocalStartButton,
	LocalModalButton,
	RemoteEncoder4,
	WirelessEncoder,
	WirelessMeasureButton,
	WirelessBackButton,
	WirelessEncoderButton,
};

// Grobe Gestalt des Eingabeereignisses, noch ohne Workflowbedeutung.
enum class NormalizedInputEventKind : uint8_t {
	None,
	RotateLeft,
	RotateRight,
	Press,
	LongPress,
	RepeatPress,
	Measure,
	Undo,
};

// NormalizedInputEvent ist die zentrale, hardwareunabhaengige Rohsprache der
// Eingabeschicht. Erst InputNormalizer leitet daraus semantische Aktionen ab.
struct NormalizedInputEvent {
	uint16_t schemaVersion = kNormalizedInputEventSchemaVersion;
	NormalizedInputSource source = NormalizedInputSource::None;
	NormalizedInputEventKind eventKind = NormalizedInputEventKind::None;
	int16_t value = 0;
	uint32_t sourceTimestampMs = 0;
	uint32_t sequence = 0;
	bool valid = false;

	bool operator==(const NormalizedInputEvent& other) const {
		return schemaVersion == other.schemaVersion && source == other.source &&
		       eventKind == other.eventKind && value == other.value &&
		       sourceTimestampMs == other.sourceTimestampMs && sequence == other.sequence &&
		       valid == other.valid;
	}

	bool operator!=(const NormalizedInputEvent& other) const {
		return !(*this == other);
	}

	bool isMeaningful() const {
		return valid && source != NormalizedInputSource::None &&
		       eventKind != NormalizedInputEventKind::None;
	}
	};

// Hilfsfabrik fuer konsistente Eventerzeugung inklusive Sequenz und Quellzeit.
inline NormalizedInputEvent makeNormalizedInputEvent(NormalizedInputSource source,
	                                                 NormalizedInputEventKind eventKind,
	                                                 int16_t value,
	                                                 uint32_t sourceTimestampMs,
	                                                 uint32_t sequence) {
	NormalizedInputEvent event;
	event.source = source;
	event.eventKind = eventKind;
	event.value = value;
	event.sourceTimestampMs = sourceTimestampMs;
	event.sequence = sequence;
	event.valid = true;
	return event;
}

}  // namespace dukatimer