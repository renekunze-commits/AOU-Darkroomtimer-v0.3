#include "RotaryEncoderDriver.h"

#include <Arduino.h>
#include <Encoder.h>

namespace dukatimer {

namespace {

int8_t directionFromDelta(long delta) {
	if (delta > 0) {
		return 1;
	}
	if (delta < 0) {
		return -1;
	}
	return 0;
}

}  // namespace

void RotaryEncoderDriver::begin(Encoder& encoderDriver,
	                            uint8_t phaseAPin,
	                            uint8_t phaseBPin,
	                            uint8_t buttonPin,
	                            long encoderCountsPerDetent,
	                            uint32_t nowMs) {
	driver_ = &encoderDriver;
	phaseAPin_ = phaseAPin;
	phaseBPin_ = phaseBPin;
	buttonPin_ = buttonPin;
	countsPerDetent_ = encoderCountsPerDetent > 0 ? encoderCountsPerDetent : 1;
	if (nowMs == 0u) {
		nowMs = millis();
	}

	// Der Taster bleibt lokal entprellt. Die A/B-Phasen werden weiterhin vom
	// bewaehrten Teensy-Encoder-Unterbau erfasst und hier nur plausibilisiert.
	pinMode(buttonPin_, INPUT_PULLUP);
	rawPosition_ = driver_->read();
	status_ = {};
	status_.logicalPosition = rawPosition_ / countsPerDetent_;
	pendingSubstepDelta_ = rawPosition_ - (status_.logicalPosition * countsPerDetent_);
	pendingDirection_ = directionFromDelta(pendingSubstepDelta_);
	buttonStableState_ = readButtonRaw();
	buttonLastRawState_ = buttonStableState_;
	buttonLastChangeMs_ = nowMs;
	status_.buttonActive = buttonStableState_;
	reanchorPhaseModel(rawPosition_, readPhaseIndex());
}

bool RotaryEncoderDriver::update(uint32_t nowMs) {
	status_.buttonChanged = false;
	updateButtonState(nowMs);

	const long newRawPosition = driver_ != nullptr ? driver_->read() : rawPosition_;
	if (newRawPosition == rawPosition_) {
		return status_.buttonChanged;
	}

	const long rawDelta = newRawPosition - rawPosition_;
	rawPosition_ = newRawPosition;
	applyRawDelta(rawDelta);
	validateQuadraturePhase(rawPosition_);
	return true;
}

long RotaryEncoderDriver::logicalPosition() const {
	return status_.logicalPosition;
}

bool RotaryEncoderDriver::buttonChanged() const {
	return status_.buttonChanged;
}

bool RotaryEncoderDriver::buttonActive() const {
	return status_.buttonActive;
}

const RotaryEncoderRuntimeStatus& RotaryEncoderDriver::status() const {
	return status_;
}

bool RotaryEncoderDriver::readButtonRaw() const {
	return digitalRead(buttonPin_) == LOW;
}

int8_t RotaryEncoderDriver::readPhaseIndex() const {
	const uint8_t stateBits = static_cast<uint8_t>((digitalRead(phaseAPin_) == HIGH ? 0x02u : 0u) |
	                                              (digitalRead(phaseBPin_) == HIGH ? 0x01u : 0u));
	switch (stateBits) {
		case 0x00u: return 0;
		case 0x01u: return 1;
		case 0x03u: return 2;
		case 0x02u: return 3;
	}

	return 0;
}

void RotaryEncoderDriver::updateButtonState(uint32_t nowMs) {
	const bool rawState = readButtonRaw();
	if (rawState != buttonLastRawState_) {
		buttonLastRawState_ = rawState;
		buttonLastChangeMs_ = nowMs;
	}

	if ((nowMs - buttonLastChangeMs_) >= kButtonDebounceMs && buttonStableState_ != rawState) {
		buttonStableState_ = rawState;
		status_.buttonActive = rawState;
		status_.buttonChanged = true;
	}
}

void RotaryEncoderDriver::applyRawDelta(long rawDelta) {
	const int8_t direction = directionFromDelta(rawDelta);
	if (direction != 0 && pendingDirection_ != 0 && direction != pendingDirection_ &&
	    pendingSubstepDelta_ != 0) {
		// Ein Richtungswechsel mitten in einer angebrochenen Rastung ist die
		// sichtbarste Laufzeitspur fuer Prellen oder halb abgebrochene Bedienung.
		++status_.partialReverseCount;
	}

	pendingSubstepDelta_ += rawDelta;
	pendingDirection_ = directionFromDelta(pendingSubstepDelta_);

	while (pendingSubstepDelta_ >= countsPerDetent_) {
		pendingSubstepDelta_ -= countsPerDetent_;
		++status_.logicalPosition;
		++status_.detentCount;
	}

	while (pendingSubstepDelta_ <= -countsPerDetent_) {
		pendingSubstepDelta_ += countsPerDetent_;
		--status_.logicalPosition;
		++status_.detentCount;
	}

	pendingDirection_ = directionFromDelta(pendingSubstepDelta_);
}

void RotaryEncoderDriver::validateQuadraturePhase(long rawPosition) {
	const int8_t currentPhaseIndex = readPhaseIndex();
	const long rawOffset = rawPosition - phaseAnchorRawPosition_;
	if (rawOffset == 0) {
		if (currentPhaseIndex != phaseAnchorIndex_) {
			++status_.phaseMismatchCount;
			reanchorPhaseModel(rawPosition, currentPhaseIndex);
		}
		return;
	}

	if (phaseDirectionSign_ == 0) {
		const int8_t forwardExpected = advancePhaseIndex(phaseAnchorIndex_, rawOffset);
		const int8_t reverseExpected = advancePhaseIndex(phaseAnchorIndex_, -rawOffset);
		const bool forwardMatches = currentPhaseIndex == forwardExpected;
		const bool reverseMatches = currentPhaseIndex == reverseExpected;
		if (forwardMatches != reverseMatches) {
			phaseDirectionSign_ = forwardMatches ? 1 : -1;
		} else if (!forwardMatches) {
			++status_.phaseMismatchCount;
			reanchorPhaseModel(rawPosition, currentPhaseIndex);
			return;
		} else {
			return;
		}
	}

	const int8_t expectedPhaseIndex =
		advancePhaseIndex(phaseAnchorIndex_, phaseDirectionSign_ * rawOffset);
	if (currentPhaseIndex != expectedPhaseIndex) {
		++status_.phaseMismatchCount;
		reanchorPhaseModel(rawPosition, currentPhaseIndex);
	}
}

void RotaryEncoderDriver::reanchorPhaseModel(long rawPosition, int8_t currentPhaseIndex) {
	phaseAnchorRawPosition_ = rawPosition;
	phaseAnchorIndex_ = currentPhaseIndex;
	phaseDirectionSign_ = 0;
}

int8_t RotaryEncoderDriver::advancePhaseIndex(int8_t startIndex, long delta) {
	int8_t index = static_cast<int8_t>((startIndex + (delta % 4l)) % 4l);
	if (index < 0) {
		index = static_cast<int8_t>(index + 4);
	}
	return index;
}

}  // namespace dukatimer