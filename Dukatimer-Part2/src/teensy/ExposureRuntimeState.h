/*
 * ExposureRuntimeState
 *
 * Version:
 * - Implementierungsschritt 1 der Laufzeitspezifikation
 * - eingefuehrt am 2026-04-24
 * - Schema-Version 1 fuer den sichtbaren Exposure-Laufzeitzustand
 *
 * Zweck:
 * - formt den minimalen, UI- und Engine-tauglichen Laufzeitzustand fuer die
 *   spaetere ExposureEngine aus
 * - schafft einen eindeutigen Platz fuer Phase, Modus, latched Fault und die
 *   zentralen Telemetrie-Groessen der Belichtung
 *
 * Integrationsgrenze dieses Schritts:
 * - noch keine aktive Belichtungslogik
 * - noch kein Sensor- oder Timerbesitz
 * - nur der formale Zustand und seine Sichtbarkeit im Snapshot
 */
#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kExposureRuntimeStateSchemaVersion = 4;

// Sichtbare Hauptphase der physischen Belichtung. Diese Phase wird von der
// ExposureEngine gesetzt und spaeter nur noch gelesen, nicht von UI oder
// Workflows eigenmaechtig umgedeutet.
enum class ExposurePhase : uint8_t {
	Idle,
	PreWait,
	Exposing,
	Paused,
	PostWait,
	Done,
	Fault,
};

// Steuerart der Belichtung. Die Runtime bleibt bewusst bei den physischen Modi
// Zeit und Dosis; EV- oder workflowbezogene Bedeutungen liegen darueber.
enum class ExposureControlMode : uint8_t {
	None,
	Time,
	Dose,
};

// FaultReason benennt den technischen bzw. betrieblichen Grund eines Stopps.
// Die spaetere UI kann daraus kuerzere oder gruppierte Labels ableiten.
enum class ExposureFaultReason : uint8_t {
	None,
	StartBlocked,
	UserAbort,
	SensorWatchdog,
	SensorPlausibility,
	ThermalHardStop,
	InternalFault,
};

// ExposureRuntimeState ist das zentrale, sichtbare Datenmodell der laufenden
// Belichtung. Es enthaelt nur Runtime-Fakten in physischen Einheiten und keine
// modusspezifische Bearbeitungs- oder Darstellungslogik.
struct ExposureRuntimeState {
	uint16_t schemaVersion = kExposureRuntimeStateSchemaVersion;
	ExposurePhase phase = ExposurePhase::Idle;
	ExposureControlMode controlMode = ExposureControlMode::None;
	ExposureFaultReason faultReason = ExposureFaultReason::None;
	// sensorFallbackActive kennzeichnet den Dose-Notbetrieb: Die Live-Dosisregelung
	// hat den TSL2561 verloren und die Engine laeuft den Rest ueber eine aus dem
	// letzten plausiblen Messwert abgeleitete harte Zeit zu Ende.
	bool sensorFallbackActive = false;
	ExposureFaultReason sensorFallbackReason = ExposureFaultReason::None;
	bool faultLatched = false;
	bool thermalDeratingActive = false;
	uint32_t phaseAgeMs = 0;
	float targetDose = 0.0f;
	float currentDose = 0.0f;
	float measuredLux = 0.0f;
	float remainingDose = 0.0f;
	float remainingTimeSeconds = 0.0f;
	float runtimeOutputLimit = 1.0f;
	// Sichtbare technische Telemetrie fuer die adaptive Head-Latenz. Dieser Wert
	// bleibt bewusst in Millisekunden und ist keine fotografische Messgroesse.
	uint32_t runtimeHeadBusLatencyMs = 0;

	bool operator==(const ExposureRuntimeState& other) const {
		return schemaVersion == other.schemaVersion && phase == other.phase &&
		       controlMode == other.controlMode && faultReason == other.faultReason &&
		       sensorFallbackActive == other.sensorFallbackActive &&
		       sensorFallbackReason == other.sensorFallbackReason &&
		       faultLatched == other.faultLatched &&
		       thermalDeratingActive == other.thermalDeratingActive &&
		       phaseAgeMs == other.phaseAgeMs &&
		       targetDose == other.targetDose && currentDose == other.currentDose &&
		       measuredLux == other.measuredLux && remainingDose == other.remainingDose &&
		       remainingTimeSeconds == other.remainingTimeSeconds &&
		       runtimeOutputLimit == other.runtimeOutputLimit &&
		       runtimeHeadBusLatencyMs == other.runtimeHeadBusLatencyMs;
	}

	bool operator!=(const ExposureRuntimeState& other) const {
		return !(*this == other);
	}
};

inline ExposureRuntimeState makeIdleExposureRuntimeState() {
	return ExposureRuntimeState{};
}

}  // namespace dukatimer