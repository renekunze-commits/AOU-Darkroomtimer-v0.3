#pragma once

#include <stdint.h>

#include "ExposureRuntimeState.h"
#include "NormalizedInputEvent.h"
#include "PaperExposureProfile.h"
#include "SystemSettings.h"

namespace dukatimer {

constexpr uint16_t kModeRuntimeStateSchemaVersion = 9;

// ModeId beschreibt den aktuell sichtbaren Workflowraum des Systems. Weitere
// Modi werden hier zentral ergaenzt, nicht als verstreute String- oder Zahlencodes.
enum class ModeId : uint8_t {
	None,
	Splitgrade,
	BlackWhite,
	Setup,
	Paper,
};

enum class PaperWorkspacePanel : uint8_t {
	Inactive,
	Select,
	Calibrate,
};

enum class PaperCalibrationItem : uint8_t {
	GradeMode,
	FixedGradeValue,
	IsoMath,
	IsoP,
	IsoR,
	KBw,
	KSoft,
	KHard,
	Calibrated,
	// Visuelle Schwellenwert-Methode (Methode 1): Der Bediener zaehlt die
	// belichteten Stufen des Graukeils und gibt Weiss- und Schwarzpunkt-Stufe
	// direkt ein. ISO-R, ISO-P und k_Bw werden daraus exakt berechnet.
	StepWhite, // Weissunkt N (1..21); muss groesser als StepBlack sein
	StepBlack, // Schwarzpunkt M (1..21); muss kleiner als StepWhite sein
	Apply,
	Discard,
	Count,
};

enum class SetupMenuItem : uint8_t {
	SoundMode,
	SoundVolume,
	Vibration,
	MaxHeadBrightness,
	HeadTimingDiagnostics,
	ThermalDeratingStart,
	ThermalHardStop,
	Apply,
	Discard,
	SafetyDefaults,
	Count,
};

// SplitgradePanel beschreibt den lokalen Eingabefokus innerhalb des SG-Modus.
enum class SplitgradePanel : uint8_t {
	Inactive,
	SplitTargets,
	Grade,
	ControlMode,
	Measurement,
};

// Fachlicher SG-Ablauf oberhalb der generischen ExposureEngine.
enum class SplitgradeExecutionState : uint8_t {
	Inactive,
	IdleConfig,
	ArmingSoft,
	ExposingSoft,
	WaitForFilter,
	ArmingHard,
	ExposingHard,
	Completed,
	Aborted,
	Fault,
};

// Rohzustand des SG-Workflows fuer Snapshot und UI. Sichtbare Texte oder EV-
// Formatierung werden ausdruecklich nicht hier erzeugt.
struct SplitgradeModeRuntimeState {
	SplitgradePanel panel = SplitgradePanel::Inactive;
	SplitgradeExecutionState executionState = SplitgradeExecutionState::Inactive;
	ExposureControlMode controlMode = ExposureControlMode::Time;
	float softTarget = 10.0f;
	float hardTarget = 10.0f;
	float grade = 2.5f;
	bool parametersDirty = false;
	uint32_t panelAgeMs = 0;
	uint32_t executionStateAgeMs = 0;

	bool operator==(const SplitgradeModeRuntimeState& other) const {
		return panel == other.panel && executionState == other.executionState &&
		       controlMode == other.controlMode &&
		       softTarget == other.softTarget && hardTarget == other.hardTarget &&
		       grade == other.grade && parametersDirty == other.parametersDirty &&
		       panelAgeMs == other.panelAgeMs &&
		       executionStateAgeMs == other.executionStateAgeMs;
	}

	bool operator!=(const SplitgradeModeRuntimeState& other) const {
		return !(*this == other);
	}
};

inline SplitgradeModeRuntimeState makeInactiveSplitgradeModeRuntimeState() {
	return SplitgradeModeRuntimeState{};
}

struct PaperCalibrationRuntimeState {
	bool available = false;
	bool calibrated = false;
	PaperGradeMode gradeMode = PaperGradeMode::Multigrade;
	bool useIsoMath = false;
	float fixedGradeValue = 2.5f;
	float isoP = 100.0f;
	float isoR = 100.0f;
	float kBw = 0.0f;
	float kSoft = 0.0f;
	float kHard = 0.0f;

	bool operator==(const PaperCalibrationRuntimeState& other) const {
		return available == other.available && calibrated == other.calibrated &&
		       gradeMode == other.gradeMode && useIsoMath == other.useIsoMath &&
		       fixedGradeValue == other.fixedGradeValue && isoP == other.isoP &&
		       isoR == other.isoR && kBw == other.kBw && kSoft == other.kSoft &&
		       kHard == other.kHard;
	}

	bool operator!=(const PaperCalibrationRuntimeState& other) const {
		return !(*this == other);
	}
};

inline PaperCalibrationRuntimeState makeInactivePaperCalibrationRuntimeState() {
	return PaperCalibrationRuntimeState{};
}

struct PaperModeRuntimeState {
	PaperWorkspacePanel panel = PaperWorkspacePanel::Inactive;
	uint8_t selectedSlot = 0u;
	bool selectionDirty = false;
	PaperCalibrationItem selectedItem = PaperCalibrationItem::GradeMode;
	uint8_t itemCount = 0u;
	bool editingActive = false;
	bool parametersDirty = false;
	bool persistFailed = false;
	uint32_t panelAgeMs = 0u;
	uint32_t selectedSlotAgeMs = 0u;
	uint32_t selectedItemAgeMs = 0u;
	uint8_t storageErrorCode = 0u;
	uint32_t storageErrorDetail = 0u;
	// Visuelle Schwellenwert-Methode (Methode 1): Stufeneingaben werden im
	// Snapshot mitgefuehrt, damit UiPresenter sie ohne Rueckgriff auf den
	// Workflow direkt darstellen kann.
	uint8_t stepWhite = 15u;
	uint8_t stepBlack = 8u;
	PaperCalibrationRuntimeState stagedProfile = makeInactivePaperCalibrationRuntimeState();

	bool operator==(const PaperModeRuntimeState& other) const {
		return panel == other.panel && selectedSlot == other.selectedSlot &&
		       selectionDirty == other.selectionDirty && selectedItem == other.selectedItem &&
		       itemCount == other.itemCount && editingActive == other.editingActive &&
		       parametersDirty == other.parametersDirty &&
		       persistFailed == other.persistFailed &&
		       panelAgeMs == other.panelAgeMs &&
		       selectedSlotAgeMs == other.selectedSlotAgeMs &&
		       selectedItemAgeMs == other.selectedItemAgeMs &&
		       storageErrorCode == other.storageErrorCode &&
		       storageErrorDetail == other.storageErrorDetail &&
		       stepWhite == other.stepWhite &&
		       stepBlack == other.stepBlack &&
		       stagedProfile == other.stagedProfile;
	}

	bool operator!=(const PaperModeRuntimeState& other) const {
		return !(*this == other);
	}
};

inline PaperModeRuntimeState makeInactivePaperModeRuntimeState() {
	return PaperModeRuntimeState{};
}

struct SetupModeRuntimeState {
	SetupMenuItem selectedItem = SetupMenuItem::SoundMode;
	uint8_t itemCount = 0u;
	bool editingActive = false;
	bool parametersDirty = false;
	bool persistFailed = false;
	bool headTimingDiagnosticsEnabled = false;
	uint32_t selectedItemAgeMs = 0u;
	uint8_t storageErrorCode = 0u;
	uint32_t storageErrorDetail = 0u;
	SystemSettings stagedSettings = makeDefaultSystemSettings();

	bool operator==(const SetupModeRuntimeState& other) const {
		return selectedItem == other.selectedItem && itemCount == other.itemCount &&
		       editingActive == other.editingActive &&
		       parametersDirty == other.parametersDirty &&
		       persistFailed == other.persistFailed &&
		       headTimingDiagnosticsEnabled == other.headTimingDiagnosticsEnabled &&
		       selectedItemAgeMs == other.selectedItemAgeMs &&
		       storageErrorCode == other.storageErrorCode &&
		       storageErrorDetail == other.storageErrorDetail &&
		       stagedSettings == other.stagedSettings;
	}

	bool operator!=(const SetupModeRuntimeState& other) const {
		return !(*this == other);
	}
};

inline SetupModeRuntimeState makeInactiveSetupModeRuntimeState() {
	return SetupModeRuntimeState{};
}

// BwPanel beschreibt den lokalen Eingabefokus innerhalb des BW-Modus.
enum class BwPanel : uint8_t {
	Inactive,
	Target,
	Grade,
	ControlMode,
	Measurement,
};

// Fachlicher BW-Ablauf oberhalb der generischen ExposureEngine.
// Einfacher als SG: eine einzelne Belichtung, kein Filterwechsel.
enum class BwExecutionState : uint8_t {
	Inactive,
	IdleConfig,
	Arming,
	Exposing,
	Completed,
	Aborted,
	Fault,
};

// Rohzustand des BW-Workflows fuer Snapshot und UI.
struct BwModeRuntimeState {
	BwPanel panel = BwPanel::Inactive;
	BwExecutionState executionState = BwExecutionState::Inactive;
	ExposureControlMode controlMode = ExposureControlMode::Time;
	float targetValue = 10.0f;
	float grade = 2.5f;
	bool gradeEditable = true;
	bool whiteLight = false;
	float softMix = 0.5f;
	float hardMix = 0.5f;
	bool parametersDirty = false;
	uint32_t panelAgeMs = 0u;
	uint32_t executionStateAgeMs = 0u;

	bool operator==(const BwModeRuntimeState& other) const {
		return panel == other.panel && executionState == other.executionState &&
		       controlMode == other.controlMode && targetValue == other.targetValue &&
		       grade == other.grade && gradeEditable == other.gradeEditable &&
		       whiteLight == other.whiteLight && softMix == other.softMix &&
		       hardMix == other.hardMix && parametersDirty == other.parametersDirty &&
		       panelAgeMs == other.panelAgeMs &&
		       executionStateAgeMs == other.executionStateAgeMs;
	}

	bool operator!=(const BwModeRuntimeState& other) const {
		return !(*this == other);
	}
};

inline BwModeRuntimeState makeInactiveBwModeRuntimeState() {
	return BwModeRuntimeState{};
}

// ModeRuntimeState ist das oberste, workflowbezogene Sichtmodell des Systems.
// Der Coordinator baut es pro Tick neu aus dem aktiven Workflow auf.
struct ModeRuntimeState {
	uint16_t schemaVersion = kModeRuntimeStateSchemaVersion;
	ModeId activeMode = ModeId::None;
	ModeId requestedMode = ModeId::None;
	bool transitionPending = false;
	uint32_t activeModeAgeMs = 0;
	uint32_t lastInputAgeMs = 0;
	NormalizedInputEvent lastInputEvent = {};
	SplitgradeModeRuntimeState splitgrade = makeInactiveSplitgradeModeRuntimeState();
	PaperModeRuntimeState paper = makeInactivePaperModeRuntimeState();
	SetupModeRuntimeState setup = makeInactiveSetupModeRuntimeState();
	BwModeRuntimeState bw = makeInactiveBwModeRuntimeState();

	bool operator==(const ModeRuntimeState& other) const {
		return schemaVersion == other.schemaVersion && activeMode == other.activeMode &&
		       requestedMode == other.requestedMode &&
		       transitionPending == other.transitionPending &&
		       activeModeAgeMs == other.activeModeAgeMs &&
		       lastInputAgeMs == other.lastInputAgeMs &&
		       lastInputEvent == other.lastInputEvent && splitgrade == other.splitgrade &&
		       paper == other.paper &&
		       setup == other.setup &&
		       bw == other.bw;
	}

	bool operator!=(const ModeRuntimeState& other) const {
		return !(*this == other);
	}
};

inline ModeRuntimeState makeUnknownModeRuntimeState() {
	return ModeRuntimeState{};
}

}  // namespace dukatimer