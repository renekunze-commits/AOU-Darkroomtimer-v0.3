/*
 * UiSemantics
 *
 * Diese Datei haelt die sichtbaren Kurzbezeichnungen fuer Runtime- und Sensor-
 * zustande zentral zusammen. Das ist bewusst keine Internationalisierungsschicht,
 * sondern eine kompakte eingebettete Darstellungssemantik fuer Debug- und
 * Hauptansichten.
 */

#include "UiSemantics.h"

#include <Arduino.h>

namespace dukatimer {
namespace ui_semantics {

FLASHMEM const char* exposurePhaseLabel(ExposurePhase phase) {
	// Die Labels bleiben absichtlich sehr kurz, damit sie in LVGL-Headern,
	// Overlays und Diagnosezeilen mit wenig Platz konstant darstellbar sind.
	switch (phase) {
		case ExposurePhase::Idle:
			return "IDLE";
		case ExposurePhase::PreWait:
			return "PRE";
		case ExposurePhase::Exposing:
			return "RUN";
		case ExposurePhase::Paused:
			return "PAUSE";
		case ExposurePhase::PostWait:
			return "POST";
		case ExposurePhase::Done:
			return "DONE";
		case ExposurePhase::Fault:
			return "FAULT";
	}

	return "UNK";
}

FLASHMEM const char* exposureModeLabel(ExposureControlMode controlMode) {
	switch (controlMode) {
		case ExposureControlMode::None:
			return "NONE";
		case ExposureControlMode::Time:
			return "TIME";
		case ExposureControlMode::Dose:
			return "DOSE";
	}

	return "UNK";
}

FLASHMEM const char* exposureFaultLabel(ExposureFaultReason faultReason) {
	switch (faultReason) {
		case ExposureFaultReason::None:
			return "NONE";
		case ExposureFaultReason::StartBlocked:
			return "START";
		case ExposureFaultReason::UserAbort:
			return "ABORT";
		case ExposureFaultReason::SensorWatchdog:
			return "WDOG";
		case ExposureFaultReason::SensorPlausibility:
			return "SENSE";
		case ExposureFaultReason::ThermalHardStop:
			return "THERM";
		case ExposureFaultReason::InternalFault:
			return "INT";
	}

	return "UNK";
}

FLASHMEM const char* exposureFaultDomainLabel(ExposureFaultReason faultReason) {
	switch (faultReason) {
		case ExposureFaultReason::None:
			return "NONE";
		case ExposureFaultReason::StartBlocked:
			return "START";
		case ExposureFaultReason::UserAbort:
			return "USER";
		case ExposureFaultReason::SensorWatchdog:
		case ExposureFaultReason::SensorPlausibility:
			return "SENSOR";
		case ExposureFaultReason::ThermalHardStop:
			return "THERMAL";
		case ExposureFaultReason::InternalFault:
			return "INTERNAL";
	}

	return "UNK";
}

FLASHMEM const char* faultLatchLabel(bool faultLatched) {
	return faultLatched ? "LATCH" : "CLEAR";
}

FLASHMEM const char* onOffLabel(bool enabled) {
	return enabled ? "ON" : "OFF";
}

FLASHMEM const char* sensorHealthLabel(SensorHealth health) {
	switch (health) {
		case SensorHealth::Unknown:
			return "UNK";
		case SensorHealth::Ok:
			return "OK";
		case SensorHealth::Stale:
			return "STALE";
		case SensorHealth::Fault:
			return "FAULT";
	}

	return "UNK";
}

FLASHMEM const char* luxValidityLabel(LuxSampleValidity sampleValidity) {
	switch (sampleValidity) {
		case LuxSampleValidity::Unknown:
			return "UNK";
		case LuxSampleValidity::Valid:
			return "VALID";
		case LuxSampleValidity::Invalid:
			return "INV";
		case LuxSampleValidity::Fault:
			return "FAULT";
	}

	return "UNK";
}

FLASHMEM const char* thermalStateLabel(ThermalState thermalState) {
	switch (thermalState) {
		case ThermalState::Unknown:
			return "UNK";
		case ThermalState::Normal:
			return "NORMAL";
		case ThermalState::Derating:
			return "DERATE";
		case ThermalState::Critical:
			return "CRIT";
		case ThermalState::Fault:
			return "FAULT";
	}

	return "UNK";
}

FLASHMEM const char* sampleFreshLabel(bool sampleFresh) {
	return sampleFresh ? "FRESH" : "STALE";
}

FLASHMEM const char* tslDiagnosticReasonLabel(Tsl2561DiagnosticReason diagnosticReason) {
	// Array-basierte Tabellen halten die Diagnosekürzel klein und zentral. Die
	// Enum-Reihenfolge ist damit Teil der stillschweigenden Vertragsgrenze.
	static constexpr const char* kLabels[] = {
		"NONE", "INIT", "READ", "INTWD", "SAT", "INV", "STALE",
	};
	const size_t index = static_cast<size_t>(diagnosticReason);
	if (index < (sizeof(kLabels) / sizeof(kLabels[0]))) {
		return kLabels[index];
	}

	return "UNK";
}

FLASHMEM const char* ds18DiagnosticReasonLabel(Ds18b20DiagnosticReason diagnosticReason) {
	static constexpr const char* kLabels[] = {
		"NONE", "UPSTR", "INV",
	};
	const size_t index = static_cast<size_t>(diagnosticReason);
	if (index < (sizeof(kLabels) / sizeof(kLabels[0]))) {
		return kLabels[index];
	}

	return "UNK";
}

}  // namespace ui_semantics
}  // namespace dukatimer
