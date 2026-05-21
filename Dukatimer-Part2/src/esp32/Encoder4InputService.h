#pragma once

#include <Arduino.h>

#include "DukaEspServiceBoardPins.h"

class ESP32Encoder;

namespace dukatimer {

class TeensyLinkService;

class Encoder4InputService {
public:
	void begin(uint32_t nowMs = 0);
	void tick(TeensyLinkService& teensyLink, uint32_t nowMs);

private:
	static constexpr uint8_t kEncoderAPin = esp_board::PIN_ENC4_A;
	static constexpr uint8_t kEncoderBPin = esp_board::PIN_ENC4_B;
	static constexpr uint8_t kEncoderSwitchPin = esp_board::PIN_ENC4_SW;
	static constexpr int64_t kEncoderCountsPerDetent = 2;
	static constexpr uint32_t kButtonDebounceMs = 50;
	static constexpr uint32_t kButtonLongPressMs = 600;

	bool initialized_ = false;
	ESP32Encoder* encoder_ = nullptr;
	int64_t lastLogicalPosition_ = 0;
	bool buttonStablePressed_ = false;
	bool buttonPrevRawPressed_ = false;
	uint32_t buttonLastEdgeMs_ = 0;
	uint32_t buttonPressStartedMs_ = 0;
	bool buttonLongPressSent_ = false;

	void emitRotation(TeensyLinkService& teensyLink, int64_t logicalStepDelta, uint32_t nowMs);
	void tickButton(TeensyLinkService& teensyLink, uint32_t nowMs);
};

}  // namespace dukatimer