#pragma once

#include <stdint.h>

class Encoder;

namespace dukatimer {

struct RotaryEncoderRuntimeStatus {
	long logicalPosition = 0;
	uint32_t detentCount = 0;
	uint32_t partialReverseCount = 0;
	uint32_t phaseMismatchCount = 0;
	bool buttonActive = false;
	bool buttonChanged = false;

	bool operator==(const RotaryEncoderRuntimeStatus& other) const {
		return logicalPosition == other.logicalPosition && detentCount == other.detentCount &&
		       partialReverseCount == other.partialReverseCount &&
		       phaseMismatchCount == other.phaseMismatchCount &&
		       buttonActive == other.buttonActive && buttonChanged == other.buttonChanged;
	}

	bool operator!=(const RotaryEncoderRuntimeStatus& other) const {
		return !(*this == other);
	}
};

/*
 * RotaryEncoderDriver
 *
 * Dieser Treiber behaelt `Encoder.h` als performanten Interrupt-Unterbau,
 * modelliert darueber aber explizit ganze Rastungen und eine einfache
 * Phasenplausibilitaet. Damit bleibt der bestehende Hardwarepfad stabil,
 * waehrend die Applikationsschicht nicht mehr direkt positionsbasiert lebt.
 */
class RotaryEncoderDriver {
public:
	void begin(Encoder& encoderDriver,
	          uint8_t phaseAPin,
	          uint8_t phaseBPin,
	          uint8_t buttonPin,
	          long encoderCountsPerDetent,
	          uint32_t nowMs = 0);
	bool update(uint32_t nowMs);

	long logicalPosition() const;
	bool buttonChanged() const;
	bool buttonActive() const;
	const RotaryEncoderRuntimeStatus& status() const;

private:
	static constexpr uint32_t kButtonDebounceMs = 20;

	Encoder* driver_ = nullptr;
	uint8_t phaseAPin_ = 0;
	uint8_t phaseBPin_ = 0;
	uint8_t buttonPin_ = 0;
	long countsPerDetent_ = 1;
	long rawPosition_ = 0;
	long pendingSubstepDelta_ = 0;
	int8_t pendingDirection_ = 0;
	int8_t phaseAnchorIndex_ = 0;
	long phaseAnchorRawPosition_ = 0;
	int8_t phaseDirectionSign_ = 0;
	bool buttonStableState_ = false;
	bool buttonLastRawState_ = false;
	uint32_t buttonLastChangeMs_ = 0;
	RotaryEncoderRuntimeStatus status_ = {};

	bool readButtonRaw() const;
	int8_t readPhaseIndex() const;
	void updateButtonState(uint32_t nowMs);
	void applyRawDelta(long rawDelta);
	void validateQuadraturePhase(long rawPosition);
	void reanchorPhaseModel(long rawPosition, int8_t currentPhaseIndex);
	static int8_t advancePhaseIndex(int8_t startIndex, long delta);
};

}  // namespace dukatimer