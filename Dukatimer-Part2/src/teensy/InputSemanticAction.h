#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kInputSemanticActionSchemaVersion = 1;

// Fachabsicht, die aus einem normierten Eingabeereignis abgeleitet wurde.
enum class InputSemanticActionKind : uint8_t {
	None,
	AdjustPrimaryDecrease,
	AdjustPrimaryIncrease,
	AdjustSecondaryDecrease,
	AdjustSecondaryIncrease,
	ContextPrevious,
	ContextNext,
	Confirm,
	Start,
	Measure,
	Pause,
	Resume,
	Undo,
};

// InputSemanticAction ist die zweite, fachlichere Sprache der Eingabekette.
// Workflows reagieren auf diese Aktion, nicht auf konkrete Hardwarequellen.
struct InputSemanticAction {
	uint16_t schemaVersion = kInputSemanticActionSchemaVersion;
	InputSemanticActionKind kind = InputSemanticActionKind::None;
	int16_t value = 0;
	uint32_t sourceTimestampMs = 0;
	uint32_t sequence = 0;
	bool valid = false;

	bool operator==(const InputSemanticAction& other) const {
		return schemaVersion == other.schemaVersion && kind == other.kind &&
		       value == other.value && sourceTimestampMs == other.sourceTimestampMs &&
		       sequence == other.sequence && valid == other.valid;
	}

	bool operator!=(const InputSemanticAction& other) const {
		return !(*this == other);
	}

	bool isMeaningful() const {
		return valid && kind != InputSemanticActionKind::None;
	}
};

// Hilfsfabrik fuer konsistente semantische Aktionen mit erhaltener Eventherkunft.
inline InputSemanticAction makeInputSemanticAction(InputSemanticActionKind kind, int16_t value,
	                                              uint32_t sourceTimestampMs,
	                                              uint32_t sequence) {
	InputSemanticAction action;
	action.kind = kind;
	action.value = value;
	action.sourceTimestampMs = sourceTimestampMs;
	action.sequence = sequence;
	action.valid = true;
	return action;
}

}  // namespace dukatimer