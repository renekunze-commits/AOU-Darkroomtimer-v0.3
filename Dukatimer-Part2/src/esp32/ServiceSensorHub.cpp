/*
 * ServiceSensorHub
 *
 * Nicht-blockierender ESP-Sensorpfad fuer DS18B20 (OneWire) sowie AHT20 und
 * BMP280 auf dem gemeinsamen I2C-Bus von U-SENS1. Der Hub publiziert nur
 * verdichteten Servicezustand an den Teensy-Link und erzeugt Diagnoseereignisse
 * bei echten Fault-Uebergaengen.
 */

#include "ServiceSensorHub.h"

#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>

#include "TeensyLinkService.h"

namespace dukatimer {

namespace {

OneWire oneWire(esp_board::PIN_ONEWIRE);
DallasTemperature ds18(&oneWire);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp280;

constexpr uint8_t kBmpPrimaryAddress = 0x76u;
constexpr uint8_t kBmpSecondaryAddress = 0x77u;

bool isValidTemperature(float value) {
	return !isnan(value) && value > -100.0f && value < 150.0f;
}

bool isValidHumidity(float value) {
	return !isnan(value) && value >= 0.0f && value <= 100.0f;
}

bool isValidPressureHpa(float value) {
	return !isnan(value) && value >= 100.0f && value <= 1500.0f;
}

}  // namespace

void ServiceSensorHub::begin(uint32_t nowMs) {
	if (initialized_) {
		return;
	}

	initialized_ = true;
	beginDs18(nowMs);
	beginAht(nowMs);
	beginBmp(nowMs);
}

void ServiceSensorHub::tick(uint32_t nowMs, bool exposureActive) {
	if (!initialized_) {
		begin(nowMs);
	}

	tickDs18(nowMs);
	if (!exposureActive) {
		tickAht(nowMs);
		tickBmp(nowMs);
	}
}

void ServiceSensorHub::publishTo(TeensyLinkService& teensyLink, uint32_t nowMs) {
	// publishTo() ist der einzige Austrittspunkt des Hubs in Richtung Linkdienst.
	// Dadurch bleiben Sensor-Timing und Transport entkoppelt.
	teensyLink.setDs18b20Sample(ds18TemperatureCelsius_, ds18Present_, ds18Fault_, oneWireOnline_,
	                         ds18SampleTimestampMs_);
	teensyLink.setEnvironmentSample(ahtTemperatureCelsius_, ahtHumidityPercent_, ahtPresent_, ahtFault_,
	                             bmpTemperatureCelsius_, bmpPressureHpa_, bmpPresent_, bmpFault_);
	if (diagnosticPending_) {
		teensyLink.queueDiagnostic(pendingDiagnosticCode_, pendingDiagnosticDetail_, pendingDiagnosticTimestampMs_);
		diagnosticPending_ = false;
	}
	(void)nowMs;
}

void ServiceSensorHub::beginDs18(uint32_t nowMs) {
	ds18.begin();
	ds18.setWaitForConversion(false);
	ds18.setResolution(10);
	oneWireOnline_ = true;
	ds18ConversionInFlight_ = false;
	nextDs18PollMs_ = nowMs;
	ds18Fault_ = false;
	ds18Present_ = ds18.getDeviceCount() > 0;
	if (!ds18Present_) {
		nextDs18PollMs_ = nowMs + kDs18RetryIntervalMs;
	}
}

void ServiceSensorHub::tickDs18(uint32_t nowMs) {
	if (!oneWireOnline_) {
		beginDs18(nowMs);
	}

	if (!ds18ConversionInFlight_) {
		if (nowMs < nextDs18PollMs_) {
			return;
		}

		ds18Present_ = ds18.getDeviceCount() > 0;
		if (!ds18Present_) {
			ds18Fault_ = false;
			nextDs18PollMs_ = nowMs + kDs18RetryIntervalMs;
			return;
		}

		// Konvertierung wird asynchron gestartet; die Fertigzeit wird ueber
		// ds18ConversionReadyAtMs_ im Tickpfad abgefragt.
		ds18.requestTemperatures();
		ds18ConversionInFlight_ = true;
		ds18ConversionReadyAtMs_ = nowMs + kDs18ConversionTimeMs;
		return;
	}

	if (nowMs < ds18ConversionReadyAtMs_) {
		return;
	}

	ds18ConversionInFlight_ = false;
	const float temperatureCelsius = ds18.getTempCByIndex(0);
	if (!isValidTemperature(temperatureCelsius)) {
		const bool wasFault = ds18Fault_;
		ds18Fault_ = true;
		ds18Present_ = ds18.getDeviceCount() > 0;
		nextDs18PollMs_ = nowMs + kDs18RetryIntervalMs;
		if (!wasFault) {
			queueDiagnostic(dukatimer::protocol::DiagnosticCode::ServiceDs18Fault,
			             static_cast<uint16_t>(ds18Present_ ? 1u : 0u), nowMs);
		}
		return;
	}

	ds18TemperatureCelsius_ = temperatureCelsius;
	ds18SampleTimestampMs_ = nowMs;
	ds18Fault_ = false;
	ds18Present_ = true;
	nextDs18PollMs_ = nowMs + kDs18PollIntervalMs;
}

void ServiceSensorHub::beginEnvironmentBus() {
	if (environmentBusStarted_) {
		return;
	}

	// AHT20 und BMP280 teilen sich auf dem Serviceboard denselben I2C-Bus.
	// Der Bus wird daher genau einmal zentral gestartet und anschliessend von
	// beiden Treibern gemeinsam genutzt.
	Wire.begin(kEnvironmentSdaPin, kEnvironmentSclPin, 400000u);
	environmentBusStarted_ = true;
}

void ServiceSensorHub::beginAht(uint32_t nowMs) {
	beginEnvironmentBus();
	ahtPresent_ = aht.begin(&Wire);
	ahtFault_ = false;
	nextAhtPollMs_ = nowMs + (ahtPresent_ ? kAhtPollIntervalMs : kEnvironmentRetryIntervalMs);
}

void ServiceSensorHub::tickAht(uint32_t nowMs) {
	if (nowMs < nextAhtPollMs_) {
		return;
	}

	if (!ahtPresent_) {
		beginAht(nowMs);
		return;
	}

	sensors_event_t humidityEvent;
	sensors_event_t temperatureEvent;
	aht.getEvent(&humidityEvent, &temperatureEvent);
	if (!isValidTemperature(temperatureEvent.temperature) ||
	    !isValidHumidity(humidityEvent.relative_humidity)) {
		const bool wasFault = ahtFault_;
		ahtFault_ = true;
		nextAhtPollMs_ = nowMs + kEnvironmentRetryIntervalMs;
		ahtPresent_ = false;
		if (!wasFault) {
			queueDiagnostic(dukatimer::protocol::DiagnosticCode::ServiceAhtFault, 1u, nowMs);
		}
		return;
	}

	ahtTemperatureCelsius_ = temperatureEvent.temperature;
	ahtHumidityPercent_ = humidityEvent.relative_humidity;
	ahtFault_ = false;
	ahtPresent_ = true;
	nextAhtPollMs_ = nowMs + kAhtPollIntervalMs;
}

bool ServiceSensorHub::initializeBmp280() {
	return bmp280.begin(kBmpPrimaryAddress) || bmp280.begin(kBmpSecondaryAddress);
}

void ServiceSensorHub::beginBmp(uint32_t nowMs) {
	beginEnvironmentBus();
	bmpPresent_ = initializeBmp280();
	bmpFault_ = false;
	nextBmpPollMs_ = nowMs + (bmpPresent_ ? kBmpPollIntervalMs : kEnvironmentRetryIntervalMs);
}

void ServiceSensorHub::tickBmp(uint32_t nowMs) {
	if (nowMs < nextBmpPollMs_) {
		return;
	}

	if (!bmpPresent_) {
		beginBmp(nowMs);
		return;
	}

	const float bmpTemperatureCelsius = bmp280.readTemperature();
	const float bmpPressureHpa = bmp280.readPressure() / 100.0f;
	if (!isValidTemperature(bmpTemperatureCelsius) || !isValidPressureHpa(bmpPressureHpa)) {
		const bool wasFault = bmpFault_;
		bmpFault_ = true;
		bmpPresent_ = false;
		nextBmpPollMs_ = nowMs + kEnvironmentRetryIntervalMs;
		if (!wasFault) {
			queueDiagnostic(dukatimer::protocol::DiagnosticCode::ServiceBmpFault, 1u, nowMs);
		}
		return;
	}

	bmpTemperatureCelsius_ = bmpTemperatureCelsius;
	bmpPressureHpa_ = bmpPressureHpa;
	bmpFault_ = false;
	bmpPresent_ = true;
	nextBmpPollMs_ = nowMs + kBmpPollIntervalMs;
}

void ServiceSensorHub::queueDiagnostic(dukatimer::protocol::DiagnosticCode code, uint16_t detail, uint32_t nowMs) {
	// Pro Hub-Tick wird nur das letzte Ereignis gehalten; das reduziert Rauschen
	// und passt zum verdichteten Runtime-Diagnosemodell.
	pendingDiagnosticCode_ = code;
	pendingDiagnosticDetail_ = detail;
	pendingDiagnosticTimestampMs_ = nowMs;
	diagnosticPending_ = true;
}

}  // namespace dukatimer