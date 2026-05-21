/*
 * SensorRuntimeStatus
 *
 * Version:
 * - Implementierungsschritt 2 der Laufzeitspezifikation
 * - eingefuehrt am 2026-04-24
 * - Schema-Version 2 fuer den sichtbaren Laufzeitzustand der harten Sensorpfade
 *
 * Zweck:
 * - definiert einen neutralen Statusraum fuer den lokalen TSL2561- und den
 *   thermischen DS18B20-Pfad des aktuellen Hardwarestands
 * - schafft frueh einen festen Platz fuer Gueltigkeit, Frische, Health und
 *   Thermik, bevor echte Sensorzugriffe implementiert werden
 *
 * Integrationsgrenze dieses Schritts:
 * - noch keine echte Sensorinitialisierung
 * - noch keine Watchdog-Logik
 * - der TSL2561 wird spaeter lokal am Teensy gelesen, der DS18B20 im aktuellen
 *   Part2-Hardwarestand ueber den ESP32-S3-Servicepfad eingespeist
 * - nur formale Sichtbarkeit im Snapshot und Platzhalter-UI
 */
#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kSensorRuntimeStatusSchemaVersion = 2;

// Einheitliche Gesundheitsklassen fuer lokale wie entfernte Sensorsubsysteme.
enum class SensorHealth : uint8_t {
	Unknown,
	Ok,
	Stale,
	Fault,
};

// Bewertet nicht den Sensor insgesamt, sondern nur die Verwendbarkeit des
// zuletzt publizierten Lux-Samples.
enum class LuxSampleValidity : uint8_t {
	Unknown,
	Valid,
	Invalid,
	Fault,
};

// Grobe fachliche Verdichtung des thermischen Zustands fuer Snapshot und UI.
enum class ThermalState : uint8_t {
	Unknown,
	Normal,
	Derating,
	Critical,
	Fault,
};

enum class Tsl2561DiagnosticReason : uint8_t {
	None,
	InitializationFailed,
	ReadFailure,
	InterruptWatchdog,
	SaturatedSample,
	InvalidSample,
	StaleSample,
};

enum class Ds18b20DiagnosticReason : uint8_t {
	None,
	UpstreamFault,
	InvalidSample,
};

// Sichtbarer Status des lokalen TSL-Pfads, getrennt in Health, Gueltigkeit,
// Freshness, Diagnose und physikalischen Messwert.
struct Tsl2561RuntimeStatus {
	SensorHealth health = SensorHealth::Unknown;
	LuxSampleValidity sampleValidity = LuxSampleValidity::Unknown;
	Tsl2561DiagnosticReason diagnosticReason = Tsl2561DiagnosticReason::None;
	bool initialized = false;
	bool sampleFresh = false;
	uint32_t sampleAgeMs = 0;
	float lux = 0.0f;

	bool operator==(const Tsl2561RuntimeStatus& other) const {
		return health == other.health && sampleValidity == other.sampleValidity &&
		       diagnosticReason == other.diagnosticReason &&
		       initialized == other.initialized && sampleFresh == other.sampleFresh &&
		       sampleAgeMs == other.sampleAgeMs && lux == other.lux;
	}

	bool operator!=(const Tsl2561RuntimeStatus& other) const {
		return !(*this == other);
	}
};

// Sichtbarer Status des thermischen DS18B20-Pfads. Die Temperatur selbst bleibt
// als Messwert erhalten; ThermalState ist nur die grobe Ableitung darueber.
struct Ds18b20RuntimeStatus {
	SensorHealth health = SensorHealth::Unknown;
	ThermalState thermalState = ThermalState::Unknown;
	Ds18b20DiagnosticReason diagnosticReason = Ds18b20DiagnosticReason::None;
	bool initialized = false;
	float temperatureCelsius = 0.0f;

	bool operator==(const Ds18b20RuntimeStatus& other) const {
		return health == other.health && thermalState == other.thermalState &&
		       diagnosticReason == other.diagnosticReason && initialized == other.initialized &&
		       temperatureCelsius == other.temperatureCelsius;
	}

	bool operator!=(const Ds18b20RuntimeStatus& other) const {
		return !(*this == other);
	}
};

// Oberer Sammelstatus fuer alle aktuell sichtbaren Sensorpfade des Systems.
struct SensorRuntimeStatus {
	uint16_t schemaVersion = kSensorRuntimeStatusSchemaVersion;
	Tsl2561RuntimeStatus tsl2561 = {};
	Ds18b20RuntimeStatus ds18b20 = {};

	bool operator==(const SensorRuntimeStatus& other) const {
		return schemaVersion == other.schemaVersion && tsl2561 == other.tsl2561 &&
		       ds18b20 == other.ds18b20;
	}

	bool operator!=(const SensorRuntimeStatus& other) const {
		return !(*this == other);
	}
};

inline SensorRuntimeStatus makeUnknownSensorRuntimeStatus() {
	return SensorRuntimeStatus{};
}

}  // namespace dukatimer