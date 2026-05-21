/*
 * SystemSnapshot
 *
 * Das Snapshot-Modell ist die Einbahnstrasse von Laufzeitdiensten zur UI.
 * Es sammelt die bereits aufbereiteten Sichtdaten aus Input, Modus, Belichtung,
 * ESP-Link, Sensorik, Licht und Touch in einem transportfreundlichen Objekt.
 */

#pragma once

#include <stdint.h>

#include "EspLinkRuntimeStatus.h"
#include "ExposureRuntimeState.h"
#include "LightController.h"
#include "MeasurementRuntimeStatus.h"
#include "ModeRuntimeState.h"
#include "PaperExposureProfile.h"
#include "PaperSlotBank.h"
#include "RemoteCommandTracker.h"
#include "RotaryEncoderDriver.h"
#include "SensorRuntimeStatus.h"
#include "TouchSampler.h"

namespace dukatimer {

struct PaperSlotUiSummary {
	bool available = false;
	bool calibrated = false;
	PaperGradeMode gradeMode = PaperGradeMode::Multigrade;
	bool useIsoMath = false;
	float fixedGradeValue = 2.5f;
	char name[kPaperProfileNameCapacity] = {};
};

struct PaperProfileUiDetail {
	bool available = false;
	bool calibrated = false;
	PaperGradeMode gradeMode = PaperGradeMode::Multigrade;
	bool useIsoMath = false;
	float fixedGradeValue = 2.5f;
	float isoP = 100.0f;
	float isoR = 100.0f;
	char name[kPaperProfileNameCapacity] = {};
};

// Die UI liest ausschliesslich dieses Sammelobjekt und greift nicht selbst auf
// Hardwareobjekte oder Dienstinstanzen zurueck.
struct SystemSnapshot {
	long encoder1Position = 0;
	long encoder2Position = 0;
	long encoder3Position = 0;
	RotaryEncoderRuntimeStatus encoder1Status;
	RotaryEncoderRuntimeStatus encoder2Status;
	RotaryEncoderRuntimeStatus encoder3Status;
	bool startButtonActive = false;
	// Diagnosewerte fuer die Paper-Slot-Persistenz werden nur sichtbar exportiert
	// und beeinflussen keine Laufzeitentscheidung.
	uint8_t paperSlotStorageErrorCode = 0;
	uint32_t paperSlotStorageErrorDetail = 0;
	uint8_t paperActiveSlot = 0;
	uint8_t paperSlotCount = 0;
	bool paperActiveSlotCalibrated = false;
	PaperGradeMode paperActiveGradeMode = PaperGradeMode::Multigrade;
	char paperActiveSlotName[kPaperProfileNameCapacity] = {};
	PaperProfileUiDetail paperActiveProfile;
	PaperSlotUiSummary paperSlotSummaries[kPaperSlotCount] = {};
	uint8_t inputFocusDomainCode = 0;
	uint8_t inputModalStateCode = 0;
	uint8_t inputEventGuardStateCode = 0;
	uint8_t inputControlOwnerCode = 0;
	uint32_t inputFocusAgeMs = 0;
	uint32_t inputModalAgeMs = 0;
	uint32_t inputEventGuardAgeMs = 0;
	uint32_t inputControlOwnerAgeMs = 0;
	bool localDoseControlForcedTime = false;
	bool localDoseControlWatchdogResetLatched = false;
	uint8_t localDoseControlDiagnosticReasonCode = 0;
	ModeRuntimeState modeState;
	ExposureRuntimeState exposureState;
	MeasurementRuntimeStatus measurementStatus;
	EspLinkRuntimeStatus espLinkStatus;
	RemoteCommandTrackerStatus remoteCommandTrackerStatus;
	SensorRuntimeStatus sensorStatus;
	LightState lightState;
	TouchState touchState;
};

}  // namespace dukatimer