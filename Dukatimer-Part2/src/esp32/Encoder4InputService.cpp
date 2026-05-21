#include "Encoder4InputService.h"

#include <ESP32Encoder.h>

#include "TeensyLinkService.h"

namespace dukatimer {

namespace {

ESP32Encoder encoder4;

int16_t clampRotationDelta(int64_t value) {
	if (value > 32767ll) {
		return 32767;
	}
	if (value < -32768ll) {
		return -32768;
	}
	return static_cast<int16_t>(value);
}

}  // namespace

void Encoder4InputService::begin(uint32_t nowMs) {
	if (initialized_) {
		return;
	}

	if (nowMs == 0u) {
		nowMs = millis();
	}

	pinMode(kEncoderSwitchPin, INPUT_PULLUP);
	ESP32Encoder::useInternalWeakPullResistors = UP;
	encoder4.attachHalfQuad(kEncoderAPin, kEncoderBPin);
	encoder4.clearCount();
	encoder_ = &encoder4;
	lastLogicalPosition_ = encoder_->getCount() / kEncoderCountsPerDetent;
	buttonPrevRawPressed_ = digitalRead(kEncoderSwitchPin) == LOW;
	buttonStablePressed_ = buttonPrevRawPressed_;
	buttonPressStartedMs_ = buttonStablePressed_ ? nowMs : 0u;
	buttonLongPressSent_ = false;
	initialized_ = true;
}

void Encoder4InputService::tick(TeensyLinkService& teensyLink, uint32_t nowMs) {
	if (!initialized_) {
		begin(nowMs);
	}

	const int64_t logicalPosition = encoder_->getCount() / kEncoderCountsPerDetent;
	const int64_t logicalStepDelta = logicalPosition - lastLogicalPosition_;
	if (logicalStepDelta != 0) {
		emitRotation(teensyLink, logicalStepDelta, nowMs);
		lastLogicalPosition_ = logicalPosition;
	}

	tickButton(teensyLink, nowMs);
}

void Encoder4InputService::emitRotation(TeensyLinkService& teensyLink,
	                                    int64_t logicalStepDelta,
	                                    uint32_t nowMs) {
	const int16_t clampedDelta = clampRotationDelta(logicalStepDelta);
	if (clampedDelta > 0) {
		teensyLink.queueInputEvent(protocol::RemoteInputSource::Encoder4,
		                         protocol::InputEventKind::RotateRight,
		                         clampedDelta,
		                         nowMs);
		return;
	}

	if (clampedDelta < 0) {
		teensyLink.queueInputEvent(protocol::RemoteInputSource::Encoder4,
		                         protocol::InputEventKind::RotateLeft,
		                         clampedDelta,
		                         nowMs);
	}
}

void Encoder4InputService::tickButton(TeensyLinkService& teensyLink, uint32_t nowMs) {
	const bool rawPressed = digitalRead(kEncoderSwitchPin) == LOW;
	if (rawPressed != buttonPrevRawPressed_) {
		buttonPrevRawPressed_ = rawPressed;
		buttonLastEdgeMs_ = nowMs;
	}

	if (buttonLastEdgeMs_ != 0u && (nowMs - buttonLastEdgeMs_) >= kButtonDebounceMs) {
		if (rawPressed != buttonStablePressed_) {
			buttonStablePressed_ = rawPressed;
			buttonLastEdgeMs_ = 0u;
			if (buttonStablePressed_) {
				buttonPressStartedMs_ = nowMs;
				buttonLongPressSent_ = false;
			} else {
				if (!buttonLongPressSent_) {
					teensyLink.queueInputEvent(protocol::RemoteInputSource::Encoder4,
					                         protocol::InputEventKind::Press,
					                         1,
					                         nowMs);
				}
				buttonPressStartedMs_ = 0u;
				buttonLongPressSent_ = false;
			}
		}
	}

	if (buttonStablePressed_ && !buttonLongPressSent_ && buttonPressStartedMs_ != 0u &&
	    (nowMs - buttonPressStartedMs_) >= kButtonLongPressMs) {
		// LongPress wird bereits als eigener Eventtyp publiziert, damit spaetere
		// Menue- oder Servicefunktionen ihn nutzen koennen, ohne den Eingabedienst
		// noch einmal anfassen zu muessen.
		teensyLink.queueInputEvent(protocol::RemoteInputSource::Encoder4,
		                         protocol::InputEventKind::LongPress,
		                         1,
		                         nowMs);
		buttonLongPressSent_ = true;
	}
}

}  // namespace dukatimer