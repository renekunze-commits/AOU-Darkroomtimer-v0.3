#pragma once

#include <Arduino.h>

#include <DukatimerProtocol.h>

#include "DukaEspServiceBoardPins.h"

namespace dukatimer {

class TeensyLinkService;

class ServiceSensorHub {
public:
	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs, bool exposureActive);
	void publishTo(TeensyLinkService& teensyLink, uint32_t nowMs);

private:
	static constexpr uint8_t kOneWirePin = esp_board::PIN_ONEWIRE;
	static constexpr uint8_t kEnvironmentSdaPin = esp_board::PIN_AHT_SDA;
	static constexpr uint8_t kEnvironmentSclPin = esp_board::PIN_AHT_SCL;
	static constexpr uint32_t kDs18PollIntervalMs = 5000;
	static constexpr uint32_t kDs18RetryIntervalMs = 2000;
	static constexpr uint32_t kDs18ConversionTimeMs = 200;
	static constexpr uint32_t kAhtPollIntervalMs = 30000;
	static constexpr uint32_t kBmpPollIntervalMs = 30000;
	static constexpr uint32_t kEnvironmentRetryIntervalMs = 2000;

	bool initialized_ = false;
	bool ds18Present_ = false;
	bool ds18Fault_ = false;
	bool oneWireOnline_ = false;
	bool ds18ConversionInFlight_ = false;
	float ds18TemperatureCelsius_ = 0.0f;
	uint32_t ds18SampleTimestampMs_ = 0;
	uint32_t nextDs18PollMs_ = 0;
	uint32_t ds18ConversionReadyAtMs_ = 0;
	bool environmentBusStarted_ = false;
	bool ahtPresent_ = false;
	bool ahtFault_ = false;
	float ahtTemperatureCelsius_ = 0.0f;
	float ahtHumidityPercent_ = 0.0f;
	uint32_t nextAhtPollMs_ = 0;
	bool bmpPresent_ = false;
	bool bmpFault_ = false;
	float bmpTemperatureCelsius_ = 0.0f;
	float bmpPressureHpa_ = 0.0f;
	uint32_t nextBmpPollMs_ = 0;
	bool diagnosticPending_ = false;
	dukatimer::protocol::DiagnosticCode pendingDiagnosticCode_ = dukatimer::protocol::DiagnosticCode::None;
	uint16_t pendingDiagnosticDetail_ = 0;
	uint32_t pendingDiagnosticTimestampMs_ = 0;

	void beginDs18(uint32_t nowMs);
	void tickDs18(uint32_t nowMs);
	void beginEnvironmentBus();
	void beginAht(uint32_t nowMs);
	void tickAht(uint32_t nowMs);
	void beginBmp(uint32_t nowMs);
	void tickBmp(uint32_t nowMs);
	bool initializeBmp280();
	void queueDiagnostic(dukatimer::protocol::DiagnosticCode code, uint16_t detail, uint32_t nowMs);
};

}  // namespace dukatimer