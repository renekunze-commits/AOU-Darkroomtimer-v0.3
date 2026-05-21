
#include "UiPresenter.h"
#include <cstdio>
#include <Arduino.h>
#include "FirmwareVersion.h"
#include "InputRouterPolicy.h"
#include "MeasurementValueFormatter.h"
#include "PaperSlotStorage.h"
#include "SystemSettingsStorage.h"
#include "UiSemantics.h"

namespace dukatimer {
namespace {
const char* exposurePhaseLabel(ExposurePhase phase) { return ui_semantics::exposurePhaseLabel(phase); }
const char* modeIdLabel(ModeId modeId) {
switch (modeId) {
case ModeId::None: return "NONE";
case ModeId::Splitgrade: return "SG";
case ModeId::BlackWhite: return "BW";
case ModeId::Setup: return "SETUP";
case ModeId::Paper: return "PAPER";
}
return "UNK";
}
const char* setupMenuItemLabel(SetupMenuItem item) {
switch (item) {
case SetupMenuItem::SoundMode: return "SOUND MODE";
case SetupMenuItem::SoundVolume: return "VOLUME";
case SetupMenuItem::Vibration: return "VIBRATION";
case SetupMenuItem::MaxHeadBrightness: return "HEAD CAP";
case SetupMenuItem::HeadTimingDiagnostics: return "HEAD DIAG";
case SetupMenuItem::ThermalDeratingStart: return "TEMP REDUCE";
case SetupMenuItem::ThermalHardStop: return "TEMP STOP";
case SetupMenuItem::Apply: return "APPLY";
case SetupMenuItem::Discard: return "DISCARD";
case SetupMenuItem::SafetyDefaults: return "SAFE DEFAULTS";
case SetupMenuItem::Count: return "COUNT";
}
return "UNK";
}
const char* soundFeedbackModeLabel(SoundFeedbackMode mode) {
switch (mode) {
case SoundFeedbackMode::Off: return "AUS";
case SoundFeedbackMode::FaultsOnly: return "FAULTS";
case SoundFeedbackMode::Normal: return "NORMAL";
}
return "UNK";
}
const char* soundVolumeLabel(SoundVolumeLevel level) {
switch (level) {
case SoundVolumeLevel::Low: return "LOW";
case SoundVolumeLevel::Medium: return "MED";
case SoundVolumeLevel::High: return "HIGH";
}
return "UNK";
}
bool setupMenuItemIsAction(SetupMenuItem item) {
	return item == SetupMenuItem::Apply || item == SetupMenuItem::Discard ||
	       item == SetupMenuItem::SafetyDefaults ||
	       item == SetupMenuItem::HeadTimingDiagnostics;
}
bool paperCalibrationItemIsAction(PaperCalibrationItem item) {
	return item == PaperCalibrationItem::Apply || item == PaperCalibrationItem::Discard;
}
const char* systemSettingsStorageErrorLabel(SystemSettingsStorageError error) {
switch (error) {
case SystemSettingsStorageError::None: return "OK";
case SystemSettingsStorageError::StorageUnavailable: return "NO-SD";
case SystemSettingsStorageError::MountFailed: return "MOUNT";
case SystemSettingsStorageError::FileNotFound: return "MISS";
case SystemSettingsStorageError::OpenFailed: return "OPEN";
case SystemSettingsStorageError::ReadFailed: return "READ";
case SystemSettingsStorageError::WriteFailed: return "WRITE";
case SystemSettingsStorageError::InvalidBlob: return "BLOB";
case SystemSettingsStorageError::UnsupportedFormatVersion: return "FMT";
case SystemSettingsStorageError::InvalidSettings: return "DATA";
case SystemSettingsStorageError::BufferAllocationFailed: return "MEM";
case SystemSettingsStorageError::RenameFailed: return "RENAME";
}
return "UNK";
}
void formatSetupValue(char* destination, size_t capacity, const SetupModeRuntimeState& setup) {
	if (destination == nullptr || capacity == 0u) {
		return;
	}

	switch (setup.selectedItem) {
	case SetupMenuItem::SoundMode:
		std::snprintf(destination, capacity, "%s", soundFeedbackModeLabel(setup.stagedSettings.soundMode));
		break;

	case SetupMenuItem::SoundVolume:
		std::snprintf(destination, capacity, "%s", soundVolumeLabel(setup.stagedSettings.soundVolume));
		break;

	case SetupMenuItem::Vibration:
		std::snprintf(destination, capacity, "%s",
			          ui_semantics::onOffLabel(setup.stagedSettings.vibrationEnabled != 0u));
		break;

	case SetupMenuItem::MaxHeadBrightness:
		std::snprintf(destination, capacity, "%u%%",
			          static_cast<unsigned>(setup.stagedSettings.maxHeadBrightnessPercent));
		break;

	case SetupMenuItem::HeadTimingDiagnostics:
		std::snprintf(destination, capacity, "%s",
		          setup.headTimingDiagnosticsEnabled ? "PAT+LOG ACTIVE"
		                                          : "CONFIRM -> PAT+LOG");
		break;

	case SetupMenuItem::ThermalDeratingStart:
		std::snprintf(destination, capacity, "%.1f C",
			          setup.stagedSettings.thermalProtection.deratingStartCelsius);
		break;

	case SetupMenuItem::ThermalHardStop:
		std::snprintf(destination, capacity, "%.1f C",
			          setup.stagedSettings.thermalProtection.hardStopCelsius);
		break;

	case SetupMenuItem::Apply:
		std::snprintf(destination, capacity, "CONFIRM -> SD");
		break;

	case SetupMenuItem::Discard:
		std::snprintf(destination, capacity, "CONFIRM -> REVERT");
		break;

	case SetupMenuItem::SafetyDefaults:
		std::snprintf(destination, capacity, "CONFIRM -> 50/60C");
		break;

	case SetupMenuItem::Count:
		std::snprintf(destination, capacity, "-");
		break;
	}
}
const char* paperCalibrationItemLabel(PaperCalibrationItem item) {
	switch (item) {
	case PaperCalibrationItem::GradeMode: return "GRADE MODE";
	case PaperCalibrationItem::FixedGradeValue: return "FIXED GRADE";
	case PaperCalibrationItem::IsoMath: return "ISO MATH";
	case PaperCalibrationItem::IsoP: return "ISO-P";
	case PaperCalibrationItem::IsoR: return "ISO-R";
	case PaperCalibrationItem::KBw: return "K-BW";
	case PaperCalibrationItem::KSoft: return "K-SOFT";
	case PaperCalibrationItem::KHard: return "K-HARD";
	case PaperCalibrationItem::Calibrated: return "CALIBRATED";
	case PaperCalibrationItem::StepWhite: return "WEISSPUNKT (N)";
	case PaperCalibrationItem::StepBlack: return "SCHWARZPUNKT (M)";
	case PaperCalibrationItem::Apply: return "APPLY";
	case PaperCalibrationItem::Discard: return "DISCARD";
	case PaperCalibrationItem::Count: return "COUNT";
	}
	return "UNK";
}
void formatPaperCalibrationValue(char* destination, size_t capacity, const PaperModeRuntimeState& paper) {
	if (destination == nullptr || capacity == 0u) {
		return;
	}

	switch (paper.selectedItem) {
	case PaperCalibrationItem::GradeMode:
		std::snprintf(destination, capacity, "%s",
			          paper.stagedProfile.gradeMode == PaperGradeMode::FixedGrade ? "FG" : "MG");
		break;

	case PaperCalibrationItem::FixedGradeValue:
		std::snprintf(destination, capacity, "%.1f", paper.stagedProfile.fixedGradeValue);
		break;

	case PaperCalibrationItem::IsoMath:
		std::snprintf(destination, capacity, "%s",
			          ui_semantics::onOffLabel(paper.stagedProfile.useIsoMath));
		break;

	case PaperCalibrationItem::IsoP:
		std::snprintf(destination, capacity, "%.0f", paper.stagedProfile.isoP);
		break;

	case PaperCalibrationItem::IsoR:
		std::snprintf(destination, capacity, "%.0f", paper.stagedProfile.isoR);
		break;

	case PaperCalibrationItem::KBw:
		std::snprintf(destination, capacity, "%.1f", paper.stagedProfile.kBw);
		break;

	case PaperCalibrationItem::KSoft:
		std::snprintf(destination, capacity, "%.1f", paper.stagedProfile.kSoft);
		break;

	case PaperCalibrationItem::KHard:
		std::snprintf(destination, capacity, "%.1f", paper.stagedProfile.kHard);
		break;

	case PaperCalibrationItem::Calibrated:
		std::snprintf(destination, capacity, "%s",
			          paper.stagedProfile.calibrated ? "CAL" : "RAW");
		break;

	case PaperCalibrationItem::StepWhite:
		std::snprintf(destination, capacity, "Stufe %u",
		              static_cast<unsigned>(paper.stepWhite));
		break;

	case PaperCalibrationItem::StepBlack:
		std::snprintf(destination, capacity, "Stufe %u",
		              static_cast<unsigned>(paper.stepBlack));
		break;

	case PaperCalibrationItem::Apply:
		std::snprintf(destination, capacity, "CONFIRM -> SD");
		break;

	case PaperCalibrationItem::Discard:
		std::snprintf(destination, capacity, "CONFIRM -> REVERT");
		break;

	case PaperCalibrationItem::Count:
		std::snprintf(destination, capacity, "-");
		break;
	}
}
const char* splitgradePanelLabel(SplitgradePanel panel) {
switch (panel) {
case SplitgradePanel::Inactive: return "INACTIVE";
case SplitgradePanel::SplitTargets: return "TARGETS";
case SplitgradePanel::Grade: return "GRADE";
case SplitgradePanel::ControlMode: return "MODE";
	case SplitgradePanel::Measurement: return "MEAS";
}
return "UNK";
}
const char* splitgradeExecutionStateLabel(SplitgradeExecutionState executionState) {
switch (executionState) {
case SplitgradeExecutionState::Inactive: return "INACTIVE";
case SplitgradeExecutionState::IdleConfig: return "IDLE";
case SplitgradeExecutionState::ArmingSoft: return "ARM-S";
case SplitgradeExecutionState::ExposingSoft: return "RUN-S";
case SplitgradeExecutionState::WaitForFilter: return "WAIT-F";
case SplitgradeExecutionState::ArmingHard: return "ARM-H";
case SplitgradeExecutionState::ExposingHard: return "RUN-H";
case SplitgradeExecutionState::Completed: return "DONE";
case SplitgradeExecutionState::Aborted: return "ABORT";
case SplitgradeExecutionState::Fault: return "FAULT";
}
return "UNK";
}

const char* bwPanelLabel(BwPanel panel) {
switch (panel) {
case BwPanel::Inactive:    return "INACTIVE";
case BwPanel::Target:      return "TARGET";
case BwPanel::Grade:       return "GRADE";
case BwPanel::ControlMode: return "MODE";
case BwPanel::Measurement: return "MEAS";
}
return "UNK";
}

const char* bwExecutionStateLabel(BwExecutionState executionState) {
switch (executionState) {
case BwExecutionState::Inactive:   return "INACTIVE";
case BwExecutionState::IdleConfig: return "IDLE";
case BwExecutionState::Arming:     return "ARM";
case BwExecutionState::Exposing:   return "RUN";
case BwExecutionState::Completed:  return "DONE";
case BwExecutionState::Aborted:    return "ABORT";
case BwExecutionState::Fault:      return "FAULT";
}
return "UNK";
}
const char* normalizedInputSourceLabel(NormalizedInputSource source) {
switch (source) {
case NormalizedInputSource::None: return "NONE";
case NormalizedInputSource::LocalEncoder1: return "ENC1";
case NormalizedInputSource::LocalEncoder2: return "ENC2";
case NormalizedInputSource::LocalEncoder3: return "ENC3";
case NormalizedInputSource::LocalStartButton: return "START";
case NormalizedInputSource::LocalModalButton: return "MODAL";
case NormalizedInputSource::RemoteEncoder4: return "ENC4";
case NormalizedInputSource::WirelessEncoder: return "W-ENC";
case NormalizedInputSource::WirelessMeasureButton: return "W-MEA";
case NormalizedInputSource::WirelessBackButton: return "W-BACK";
case NormalizedInputSource::WirelessEncoderButton: return "W-BTN";
}
return "UNK";
}
const char* normalizedInputEventKindLabel(NormalizedInputEventKind eventKind) {
switch (eventKind) {
case NormalizedInputEventKind::None: return "NONE";
case NormalizedInputEventKind::RotateLeft: return "LEFT";
case NormalizedInputEventKind::RotateRight: return "RIGHT";
case NormalizedInputEventKind::Press: return "PRESS";
case NormalizedInputEventKind::LongPress: return "LONG";
case NormalizedInputEventKind::RepeatPress: return "REPEAT";
case NormalizedInputEventKind::Measure: return "MEASURE";
case NormalizedInputEventKind::Undo: return "UNDO";
}
return "UNK";
}
const char* exposureModeLabel(ExposureControlMode controlMode) { return ui_semantics::exposureModeLabel(controlMode); }
const char* exposureFaultLabel(ExposureFaultReason faultReason) { return ui_semantics::exposureFaultLabel(faultReason); }
const char* exposureFaultDomainLabel(ExposureFaultReason faultReason) { return ui_semantics::exposureFaultDomainLabel(faultReason); }
const char* faultLatchLabel(bool faultLatched) { return ui_semantics::faultLatchLabel(faultLatched); }
const char* onOffLabel(bool enabled) { return ui_semantics::onOffLabel(enabled); }
const char* sensorHealthLabel(SensorHealth health) { return ui_semantics::sensorHealthLabel(health); }
const char* luxValidityLabel(LuxSampleValidity sampleValidity) { return ui_semantics::luxValidityLabel(sampleValidity); }
const char* thermalStateLabel(ThermalState thermalState) { return ui_semantics::thermalStateLabel(thermalState); }
const char* sampleFreshLabel(bool sampleFresh) { return ui_semantics::sampleFreshLabel(sampleFresh); }
const char* tslDiagnosticReasonLabel(Tsl2561DiagnosticReason diagnosticReason) { return ui_semantics::tslDiagnosticReasonLabel(diagnosticReason); }
const char* ds18DiagnosticReasonLabel(Ds18b20DiagnosticReason diagnosticReason) { return ui_semantics::ds18DiagnosticReasonLabel(diagnosticReason); }
const char* paperGradeModeShortLabel(PaperGradeMode gradeMode) {
switch (gradeMode) {
case PaperGradeMode::Multigrade: return "MG";
case PaperGradeMode::FixedGrade: return "FG";
}
return "UNK";
}
const char* paperCalibrationShortLabel(bool calibrated) {
return calibrated ? "CAL" : "RAW";
}
const char* paperWorkspacePanelLabel(PaperWorkspacePanel panel) {
switch (panel) {
case PaperWorkspacePanel::Inactive: return "INACTIVE";
case PaperWorkspacePanel::Select: return "SELECT";
case PaperWorkspacePanel::Calibrate: return "CAL";
}
return "UNK";
}
const PaperSlotUiSummary* resolvePaperSlotSummary(const SystemSnapshot& snapshot, uint8_t slotIndex) {
	const uint8_t slotCount = snapshot.paperSlotCount <= kPaperSlotCount
		? snapshot.paperSlotCount
		: kPaperSlotCount;
	if (slotCount == 0u || slotIndex >= slotCount) {
		return nullptr;
	}

	const PaperSlotUiSummary& summary = snapshot.paperSlotSummaries[slotIndex];
	return summary.available ? &summary : nullptr;
}
const char* localDoseControlLabel(const SystemSnapshot& snapshot) {
if (!snapshot.localDoseControlForcedTime) { return "OFF"; }
if (snapshot.localDoseControlWatchdogResetLatched) { return "WDOG"; }
return tslDiagnosticReasonLabel(static_cast<Tsl2561DiagnosticReason>(snapshot.localDoseControlDiagnosticReasonCode));
}
const char* exposureRegulationLabel(const SystemSnapshot& snapshot) {
if (snapshot.exposureState.sensorFallbackActive) { return "EST"; }
switch (snapshot.exposureState.controlMode) {
case ExposureControlMode::Dose: return "LOOP";
case ExposureControlMode::Time: return "TIME";
case ExposureControlMode::None: return "NONE";
}
return "UNK";
}
const char* espLinkHealthLabel(EspLinkHealth health) {
switch (health) {
case EspLinkHealth::Unknown: return "UNK";
case EspLinkHealth::Online: return "ONLINE";
case EspLinkHealth::Stale: return "STALE";
case EspLinkHealth::Lost: return "LOST";
}
return "UNK";
}
const char* remoteInputSourceLabel(protocol::RemoteInputSource source) {
switch (source) {
case protocol::RemoteInputSource::None: return "NONE";
case protocol::RemoteInputSource::Encoder4: return "ENC4";
case protocol::RemoteInputSource::WirelessEncoder: return "W-ENC";
case protocol::RemoteInputSource::WirelessMeasureButton: return "W-MEA";
case protocol::RemoteInputSource::WirelessBackButton: return "W-BACK";
case protocol::RemoteInputSource::WirelessEncoderButton: return "W-BTN";
}
return "NONE";
}
const char* inputEventKindLabel(protocol::InputEventKind eventKind) {
switch (eventKind) {
case protocol::InputEventKind::None: return "NONE";
case protocol::InputEventKind::RotateLeft: return "LEFT";
case protocol::InputEventKind::RotateRight: return "RIGHT";
case protocol::InputEventKind::Press: return "PRESS";
case protocol::InputEventKind::LongPress: return "LONG";
case protocol::InputEventKind::Measure: return "MEASURE";
case protocol::InputEventKind::Undo: return "UNDO";
}
return "NONE";
}
const char* wirelessPeerStateLabel(protocol::WirelessPeerState state) {
switch (state) {
case protocol::WirelessPeerState::Unknown: return "UNK";
case protocol::WirelessPeerState::Offline: return "OFF";
case protocol::WirelessPeerState::Online: return "ON";
case protocol::WirelessPeerState::Measuring: return "MEAS";
case protocol::WirelessPeerState::Fault: return "FAULT";
}
return "UNK";
}
const char* diagnosticCodeLabel(protocol::DiagnosticCode code) {
	switch (code) {
	case protocol::DiagnosticCode::None: return "OK";
	case protocol::DiagnosticCode::ServiceDs18Fault: return "DS18";
	case protocol::DiagnosticCode::ServiceAhtFault: return "AHT";
	case protocol::DiagnosticCode::WirelessGatewayInitFailed: return "RAD-BOOT";
	case protocol::DiagnosticCode::WirelessMeasurementTimeout: return "RAD-TMO";
	case protocol::DiagnosticCode::WirelessSendFailed: return "RAD-TX";
	case protocol::DiagnosticCode::LinkRenderCoalesced: return "LINK-R";
	case protocol::DiagnosticCode::LinkCommandDropped: return "LINK-C";
	case protocol::DiagnosticCode::LinkInputDropped: return "LINK-I";
	case protocol::DiagnosticCode::RemoteCommandRetry: return "RMT-RET";
	case protocol::DiagnosticCode::RemoteCommandTrackerSaturated: return "RMT-SAT";
	case protocol::DiagnosticCode::RemoteCommandTimeout: return "RMT-TMO";
	case protocol::DiagnosticCode::WirelessRenderStale: return "RAD-RST";
	case protocol::DiagnosticCode::WirelessRenderTimeout: return "RAD-RTO";
	case protocol::DiagnosticCode::ServiceBmpFault: return "BMP";
	}
	return "UNK";
}

const char* vfsStatusShortLabel(protocol::VfsStatusCode status) {
	switch (status) {
	case protocol::VfsStatusCode::None: return "NONE";
	case protocol::VfsStatusCode::Ok: return "OK";
	case protocol::VfsStatusCode::Busy: return "BUSY";
	case protocol::VfsStatusCode::InvalidPath: return "PATH";
	case protocol::VfsStatusCode::SdUnavailable: return "NO-SD";
	case protocol::VfsStatusCode::Unsupported: return "UNSUP";
	case protocol::VfsStatusCode::OpenFailed: return "OPEN";
	case protocol::VfsStatusCode::WriteFailed: return "WRITE";
	case protocol::VfsStatusCode::ReadFailed: return "READ";
	case protocol::VfsStatusCode::InvalidState: return "STATE";
	case protocol::VfsStatusCode::TransactionMismatch: return "TXID";
	case protocol::VfsStatusCode::StorageNotReady: return "STOR";
	case protocol::VfsStatusCode::FlowHeld: return "HOLD";
	case protocol::VfsStatusCode::ProtocolRejected: return "PROTO";
	}
	return "UNK";
}

const char* paperSlotRecoveryRecommendationLabel(protocol::PaperSlotRecoveryRecommendation recommendation) {
	switch (recommendation) {
	case protocol::PaperSlotRecoveryRecommendation::Unknown: return "UNK";
	case protocol::PaperSlotRecoveryRecommendation::RetryInspection: return "RETRY";
	case protocol::PaperSlotRecoveryRecommendation::NoAction: return "OK";
	case protocol::PaperSlotRecoveryRecommendation::KeepActive: return "KEEP";
	case protocol::PaperSlotRecoveryRecommendation::OfferRestoreBackup: return "RESTORE";
	case protocol::PaperSlotRecoveryRecommendation::RecoverViaUpload: return "UPLOAD";
	}
	return "UNK";
}

const char* paperSlotParseCodeLabel(uint8_t code) {
	switch (code) {
	case 0u: return "OK";
	case 1u: return "SMALL";
	case 2u: return "MAGIC";
	case 3u: return "FMT";
	case 4u: return "SIZE";
	case 5u: return "BLOB";
	case 6u: return "CRC";
	case 7u: return "BANK";
	}
	return "UNK";
}

const char* wirelessRenderStatusLabel(uint16_t flags) {
	return (flags & protocol::kWirelessRenderStatusTimeoutActive) != 0u ? "TMO" : "OK";
}
const char* paperSlotStorageErrorLabel(PaperSlotStorageError error) {
switch (error) {
case PaperSlotStorageError::None: return "OK";
case PaperSlotStorageError::StorageUnavailable: return "NO-SD";
case PaperSlotStorageError::MountFailed: return "MOUNT";
case PaperSlotStorageError::FileNotFound: return "MISS";
case PaperSlotStorageError::OpenFailed: return "OPEN";
case PaperSlotStorageError::ReadFailed: return "READ";
case PaperSlotStorageError::WriteFailed: return "WRITE";
case PaperSlotStorageError::InvalidBlob: return "BLOB";
case PaperSlotStorageError::UnsupportedFormatVersion: return "FMT";
case PaperSlotStorageError::InvalidBank: return "BANK";
case PaperSlotStorageError::BufferAllocationFailed: return "MEM";
case PaperSlotStorageError::RenameFailed: return "RENAME";
}
return "UNK";
}
const char* inputFocusDomainLabel(InputFocusDomain focusDomain) {
switch (focusDomain) {
case InputFocusDomain::Encoder: return "ENC";
case InputFocusDomain::Touch: return "TOUCH";
case InputFocusDomain::Modal: return "MODAL";
}
return "UNK";
}
const char* inputModalStateLabel(InputModalState modalState) {
switch (modalState) {
case InputModalState::None: return "NONE";
case InputModalState::WorkflowConfirm: return "CONF";
case InputModalState::WorkflowWait: return "WAIT";
case InputModalState::WorkflowFault: return "FAULT";
}
return "UNK";
}
const char* inputControlOwnerLabel(InputControlOwner controlOwner) {
switch (controlOwner) {
case InputControlOwner::None: return "NONE";
case InputControlOwner::Local: return "LOCAL";
case InputControlOwner::Remote: return "REMOTE";
}
return "UNK";
}
const char* inputEventGuardLabel(InputEventGuardState eventGuardState) {
switch (eventGuardState) {
case InputEventGuardState::Open: return "OPEN";
case InputEventGuardState::TouchPriority: return "TOUCH";
case InputEventGuardState::ConfirmLocked: return "CONF";
case InputEventGuardState::WaitLocked: return "WAIT";
case InputEventGuardState::FaultLocked: return "FAULT";
}
return "UNK";
}
const char* measurementLuxSourceLabel(MeasurementLuxSource source) {
switch (source) {
case MeasurementLuxSource::None: return "NONE";
case MeasurementLuxSource::LocalTsl2561: return "TSL";
case MeasurementLuxSource::WirelessGateway: return "WIRE";
}
return "UNK";
}
} // namespace

// Diagnostic string buffers live in PSRAM (EXTMEM). They are written by
// FLASHMEM presenter methods and read by the LVGL render path in the main loop.
// Moving them out of DTCM saves 3,168 bytes of RAM1.
static EXTMEM char subtitleBuffer_[96];
static EXTMEM char modeInfoBuffer_[384];
static EXTMEM char exposureInfoBuffer_[192];
static EXTMEM char sensorInfoBuffer_[256];
static EXTMEM char remoteCommandInfoBuffer_[192];
static EXTMEM char gatewayInfoBuffer_[768];
static EXTMEM char encoderInfoBuffer_[192];
static EXTMEM char switchInfoBuffer_[96];
static EXTMEM char touchInfoBuffer_[64];
static EXTMEM char measurementSourcesBuffer_[128];
static EXTMEM char measurementMainBuffer_[64];
static EXTMEM char measurementMetaBuffer_[96];
static EXTMEM char measurementReferenceBuffer_[96];
static EXTMEM char measurementRangeBuffer_[96];
static EXTMEM char measurementStatusLeftBuffer_[64];
static EXTMEM char measurementStatusCenterBuffer_[96];
static EXTMEM char measurementStatusRightBuffer_[96];
static EXTMEM char measurementUndoChipBuffer_[32];
static EXTMEM char dmaInfoBuffer_[160];
static EXTMEM char pageMeasLocalLuxBuffer_[24];
static EXTMEM char pageMeasWirelessLuxBuffer_[24];
static EXTMEM char pageMeasLuxMainBuffer_[24];
static EXTMEM char pageMeasLuxAgeBuffer_[24];
static EXTMEM char pageMeasRefLuxBuffer_[32];
static EXTMEM char pageMeasEvDiffBuffer_[16];
static EXTMEM char pageMeasSessionBuffer_[64];

const char* UiPresenter::getTitle(const SystemSnapshot& snapshot) {
	std::snprintf(titleBuffer_, sizeof(titleBuffer_), "Dukatimer v%s", build::kFirmwareVersion);
	return titleBuffer_;
}

FLASHMEM const char* UiPresenter::getSubtitle(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode == ModeId::Paper) {
		if (snapshot.modeState.paper.panel == PaperWorkspacePanel::Calibrate) {
			std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_),
					  "Paper CAL - Slotprofil lokal editieren und speichern");
		} else {
			std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_),
					  "Paper Workspace - E1 +/-1  E2 +/-5  E3 CAL");
		}
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite && snapshot.paperActiveProfile.available) {
		const BwModeRuntimeState& bw = snapshot.modeState.bw;
		const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
		const unsigned activeSlot = slotCount > 0u
		                        ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u
		                        : 0u;
		const bool bwSubDose = (bw.controlMode == ExposureControlMode::Dose);
		if (bw.whiteLight) {
			if (bwSubDose) {
				std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_),
						  "BW WEISSLICHT G%.1f  Ziel %.2f lx\xB7s  Paper %u/%u %.12s",
						  static_cast<double>(bw.grade),
						  static_cast<double>(bw.targetValue),
						  activeSlot, slotCount, snapshot.paperActiveProfile.name);
			} else {
				std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_),
						  "BW WEISSLICHT G%.1f  Ziel %.1f s  Paper %u/%u %.12s",
						  static_cast<double>(bw.grade),
						  static_cast<double>(bw.targetValue),
						  activeSlot, slotCount, snapshot.paperActiveProfile.name);
			}
		} else {
			if (bwSubDose) {
				std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_),
						  "BW MG G%.1f  S%.0f%%/H%.0f%%  Ziel %.2f lx\xB7s  Paper %u/%u %.12s",
						  static_cast<double>(bw.grade),
						  static_cast<double>(bw.softMix * 100.0f),
						  static_cast<double>(bw.hardMix * 100.0f),
						  static_cast<double>(bw.targetValue),
						  activeSlot, slotCount, snapshot.paperActiveProfile.name);
			} else {
				std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_),
						  "BW MG G%.1f  S%.0f%%/H%.0f%%  Ziel %.1f s  Paper %u/%u %.12s",
						  static_cast<double>(bw.grade),
						  static_cast<double>(bw.softMix * 100.0f),
						  static_cast<double>(bw.hardMix * 100.0f),
						  static_cast<double>(bw.targetValue),
						  activeSlot, slotCount, snapshot.paperActiveProfile.name);
			}
		}
	} else {
		std::snprintf(subtitleBuffer_, sizeof(subtitleBuffer_), "System Snapshot - %lu ms",
				  static_cast<unsigned long>(millis()));
	}
	return subtitleBuffer_;
}

const char* UiPresenter::getSgHeader(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode == ModeId::Splitgrade) {
		if (snapshot.modeState.splitgrade.panel == SplitgradePanel::Measurement) {
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "MEAS %s  EXEC %s",
					  measurementLuxSourceLabel(snapshot.measurementStatus.activeLux.source),
					  splitgradeExecutionStateLabel(snapshot.modeState.splitgrade.executionState));
		} else {
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "SG %s  EXEC %s  CTRL %s  P %s",
					  splitgradePanelLabel(snapshot.modeState.splitgrade.panel),
					  splitgradeExecutionStateLabel(snapshot.modeState.splitgrade.executionState),
					  exposureModeLabel(snapshot.modeState.splitgrade.controlMode),
					  paperGradeModeShortLabel(snapshot.paperActiveGradeMode));
		}
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite) {
		const BwModeRuntimeState& bw = snapshot.modeState.bw;
		const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
		const unsigned activeSlot = slotCount > 0u
		                        ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u
		                        : 0u;
		if (bw.panel == BwPanel::Measurement) {
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_),
					  "BW MEAS %s  EXEC %s",
					  measurementLuxSourceLabel(snapshot.measurementStatus.activeLux.source),
					  bwExecutionStateLabel(bw.executionState));
		} else if (snapshot.paperActiveProfile.available) {
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "BW %s  EXEC %s  CTRL %s  SLOT %u/%u",
					  bwPanelLabel(bw.panel),
					  bwExecutionStateLabel(bw.executionState),
					  exposureModeLabel(bw.controlMode),
					  activeSlot, slotCount);
		} else {
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "BW  NO PAPER PROFILE");
		}
	} else if (snapshot.modeState.activeMode == ModeId::Paper) {
		const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
		const unsigned selectedSlot = slotCount > 0u
			? static_cast<unsigned>(snapshot.modeState.paper.selectedSlot) + 1u
			: 0u;
		const unsigned activeSlot = slotCount > 0u
			? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u
			: 0u;
		if (snapshot.modeState.paper.panel == PaperWorkspacePanel::Calibrate) {
			const char* stateLabel = snapshot.modeState.paper.editingActive
				? "EDIT"
				: (snapshot.modeState.paper.parametersDirty ? "DIRTY" : "NAV");
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "PAPER CAL %u/%u  %s  %s",
					  selectedSlot,
					  slotCount,
					  paperCalibrationItemLabel(snapshot.modeState.paper.selectedItem),
					  stateLabel);
		} else {
			std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "PAPER %s  SEL %u/%u  ACT %u",
					  paperWorkspacePanelLabel(snapshot.modeState.paper.panel),
					  selectedSlot,
					  slotCount,
					  activeSlot);
		}
	} else if (snapshot.modeState.activeMode == ModeId::Setup) {
		const char* stateLabel = snapshot.modeState.setup.editingActive
			? "EDIT"
			: (snapshot.modeState.setup.parametersDirty ? "DIRTY" : "NAV");
		std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "SETUP %s  %s",
					  setupMenuItemLabel(snapshot.modeState.setup.selectedItem), stateLabel);
	} else {
		std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "MODE %s",
					  modeIdLabel(snapshot.modeState.activeMode));
	}

	return sgHeaderBuffer_;
}

const char* UiPresenter::getSgTargets(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode == ModeId::Splitgrade) {
		if (snapshot.modeState.splitgrade.panel == SplitgradePanel::Measurement) {
			const MeasurementLuxSample& localLux = snapshot.measurementStatus.localLux;
			const MeasurementLuxSample& wirelessLux = snapshot.measurementStatus.wirelessLux;
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "LOCAL %.3f  WIRE %.3f  U%u",
					  localLux.valid ? localLux.lux : 0.0f,
					  wirelessLux.valid ? wirelessLux.lux : 0.0f,
					  static_cast<unsigned>(snapshot.measurementStatus.session.undoDepth));
		} else {
			const char* dirtyLabel = snapshot.modeState.splitgrade.parametersDirty ? "DIRTY" : "CLEAN";
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "TARGET S %.1f  H %.1f  G %.1f  %s",
					  snapshot.modeState.splitgrade.softTarget,
					  snapshot.modeState.splitgrade.hardTarget,
					  snapshot.modeState.splitgrade.grade, dirtyLabel);
		}
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite) {
		const BwModeRuntimeState& bw = snapshot.modeState.bw;
		if (!snapshot.paperActiveProfile.available) {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "PAPER PROFILE FEHLT");
		} else {
			const char* dirtyMark = bw.parametersDirty ? " DIRTY" : "";
			if (bw.whiteLight) {
				if (bw.controlMode == ExposureControlMode::Dose) {
					std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
							  "BW G%.1f WEISS  Ziel %.2f lx\xB7s%s",
							  static_cast<double>(bw.grade),
							  static_cast<double>(bw.targetValue), dirtyMark);
				} else {
					std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
							  "BW G%.1f WEISS  Ziel %.1f s%s",
							  static_cast<double>(bw.grade),
							  static_cast<double>(bw.targetValue), dirtyMark);
				}
			} else {
				if (bw.controlMode == ExposureControlMode::Dose) {
					std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
							  "BW G%.1f  S%.0f%%/H%.0f%%  Ziel %.2f lx\xB7s%s",
							  static_cast<double>(bw.grade),
							  static_cast<double>(bw.softMix * 100.0f),
							  static_cast<double>(bw.hardMix * 100.0f),
							  static_cast<double>(bw.targetValue), dirtyMark);
				} else {
					std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
							  "BW G%.1f  S%.0f%%/H%.0f%%  Ziel %.1f s%s",
							  static_cast<double>(bw.grade),
							  static_cast<double>(bw.softMix * 100.0f),
							  static_cast<double>(bw.hardMix * 100.0f),
							  static_cast<double>(bw.targetValue), dirtyMark);
				}
			}
		}
	} else if (snapshot.modeState.activeMode == ModeId::Paper) {
		const PaperSlotUiSummary* selectedSummary =
			resolvePaperSlotSummary(snapshot, snapshot.modeState.paper.selectedSlot);
		if (snapshot.modeState.paper.panel == PaperWorkspacePanel::Calibrate) {
			char valueBuffer[48] = {};
			formatPaperCalibrationValue(valueBuffer, sizeof(valueBuffer), snapshot.modeState.paper);
			if (selectedSummary == nullptr) {
				std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "%s: %s",
						  paperCalibrationItemLabel(snapshot.modeState.paper.selectedItem), valueBuffer);
			} else {
				std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "%.12s %s/%s  %s",
						  selectedSummary->name,
						  paperCalibrationShortLabel(snapshot.modeState.paper.stagedProfile.calibrated),
						  paperGradeModeShortLabel(snapshot.modeState.paper.stagedProfile.gradeMode),
						  valueBuffer);
			}
		} else if (selectedSummary == nullptr) {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "SLOT LEER ODER UNGUELTIG");
		} else if (isFixedGradeMode(selectedSummary->gradeMode)) {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "%.16s  %s/%s  G %.1f",
					  selectedSummary->name,
					  paperCalibrationShortLabel(selectedSummary->calibrated),
					  paperGradeModeShortLabel(selectedSummary->gradeMode),
					  selectedSummary->fixedGradeValue);
		} else {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "%.16s  %s/%s  %s",
					  selectedSummary->name,
					  paperCalibrationShortLabel(selectedSummary->calibrated),
					  paperGradeModeShortLabel(selectedSummary->gradeMode),
					  selectedSummary->useIsoMath ? "ISO" : "LUT");
		}
	} else if (snapshot.modeState.activeMode == ModeId::Setup) {
		char valueBuffer[48] = {};
		formatSetupValue(valueBuffer, sizeof(valueBuffer), snapshot.modeState.setup);
		std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "%s: %s",
					  setupMenuItemLabel(snapshot.modeState.setup.selectedItem), valueBuffer);
	} else {
		std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "REQ %s",
					  modeIdLabel(snapshot.modeState.requestedMode));
	}

	return sgTargetsBuffer_;
}

const char* UiPresenter::getSgExposureMain(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode == ModeId::Splitgrade) {
		if (snapshot.modeState.splitgrade.panel == SplitgradePanel::Measurement) {
			const MeasurementLuxSample& activeLux = snapshot.measurementStatus.activeLux;
			const MeasurementReferenceStatus& activeReference = snapshot.measurementStatus.activeReference;
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "A %.3f lux  REF %.3f  dEV %+.2f",
					  activeLux.valid ? activeLux.lux : 0.0f,
					  activeReference.valid ? activeReference.lux : 0.0f,
					  snapshot.measurementStatus.activeRelativeEvValid
						  ? snapshot.measurementStatus.activeRelativeEvStops
						  : 0.0f);
		} else {
			MeasurementValueFormatter::formatSplitgradeDoseTelemetry(
				sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
				snapshot.exposureState.currentDose, snapshot.exposureState.targetDose,
				snapshot.exposureState.remainingTimeSeconds,
				snapshot.exposureState.measuredLux);
		}
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite) {
		const BwModeRuntimeState& bw = snapshot.modeState.bw;
		if (bw.panel == BwPanel::Measurement) {
			const MeasurementLuxSample& activeLux = snapshot.measurementStatus.activeLux;
			const MeasurementReferenceStatus& activeReference = snapshot.measurementStatus.activeReference;
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "A %.3f lux  REF %.3f  dEV %+.2f",
					  activeLux.valid ? static_cast<double>(activeLux.lux) : 0.0,
					  activeReference.valid ? static_cast<double>(activeReference.lux) : 0.0,
					  snapshot.measurementStatus.activeRelativeEvValid
						  ? static_cast<double>(snapshot.measurementStatus.activeRelativeEvStops)
						  : 0.0);
		} else {
			MeasurementValueFormatter::formatSplitgradeDoseTelemetry(
				sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
				snapshot.exposureState.currentDose, snapshot.exposureState.targetDose,
				snapshot.exposureState.remainingTimeSeconds,
				snapshot.exposureState.measuredLux);
		}
	} else if (snapshot.modeState.activeMode == ModeId::Paper) {
		if (snapshot.modeState.paper.panel == PaperWorkspacePanel::Calibrate) {
			if (snapshot.modeState.paper.persistFailed) {
				std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
						  "SAVE FEHLER %s/%lu",
						  paperSlotStorageErrorLabel(
							  static_cast<PaperSlotStorageError>(snapshot.modeState.paper.storageErrorCode)),
						  static_cast<unsigned long>(snapshot.modeState.paper.storageErrorDetail));
			} else if (paperCalibrationItemIsAction(snapshot.modeState.paper.selectedItem)) {
				std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
						  "CONFIRM AKTION  DIRTY %s",
						  snapshot.modeState.paper.parametersDirty ? "ON" : "OFF");
			} else {
				std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
						  "E3 ITEM  E1/E2 WERT  CONFIRM %s",
						  snapshot.modeState.paper.editingActive ? "OK" : "EDIT");
			}
		} else if (snapshot.modeState.paper.selectionDirty) {
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "CONFIRM -> AKTIV  UNDO -> ACTIVE  E1/E2 SLOT");
		} else {
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "AKTIV  E1/E2 SLOT  E3 VIEW  ENC2 BTN -> PRINT");
		}
	} else if (snapshot.modeState.activeMode == ModeId::Setup) {
		if (snapshot.modeState.setup.persistFailed) {
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "SAVE FEHLER %s/%lu",
					  systemSettingsStorageErrorLabel(
						  static_cast<SystemSettingsStorageError>(snapshot.modeState.setup.storageErrorCode)),
					  static_cast<unsigned long>(snapshot.modeState.setup.storageErrorDetail));
		} else if (setupMenuItemIsAction(snapshot.modeState.setup.selectedItem)) {
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "CONFIRM AKTION  DIRTY %s",
					  snapshot.modeState.setup.parametersDirty ? "ON" : "OFF");
		} else {
			std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
					  "E3 ITEM  E1/E2 WERT  CONFIRM %s",
					  snapshot.modeState.setup.editingActive ? "OK" : "EDIT");
		}
	} else {
		sgExposureMainBuffer_[0] = '\0';
	}

	return sgExposureMainBuffer_;
}

// ---------- BW-specific getters (SCREEN_ID_PAGE_SPLITGRADE BW reuse + future PageBlackWhite) ----

const char* UiPresenter::getBwHeader(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode != ModeId::BlackWhite) {
		sgHeaderBuffer_[0] = '\0';
		return sgHeaderBuffer_;
	}
	const BwModeRuntimeState& bw = snapshot.modeState.bw;
	const unsigned slotCount  = static_cast<unsigned>(snapshot.paperSlotCount);
	const unsigned activeSlot = slotCount > 0u
	                        ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u
	                        : 0u;
	if (bw.panel == BwPanel::Measurement) {
		std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "BW MEAS %s  EXEC %s",
				  measurementLuxSourceLabel(snapshot.measurementStatus.activeLux.source),
				  bwExecutionStateLabel(bw.executionState));
	} else if (snapshot.paperActiveProfile.available) {
		std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "BW %s  EXEC %s  CTRL %s  SLOT %u/%u",
				  bwPanelLabel(bw.panel),
				  bwExecutionStateLabel(bw.executionState),
				  exposureModeLabel(bw.controlMode),
				  activeSlot, slotCount);
	} else {
		std::snprintf(sgHeaderBuffer_, sizeof(sgHeaderBuffer_), "SCHWARZWEISS  KEIN PAPIERPROFIL");
	}
	return sgHeaderBuffer_;
}

const char* UiPresenter::getBwTargets(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode != ModeId::BlackWhite) {
		sgTargetsBuffer_[0] = '\0';
		return sgTargetsBuffer_;
	}
	const BwModeRuntimeState& bw = snapshot.modeState.bw;
	if (!snapshot.paperActiveProfile.available) {
		std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_), "PAPIERPROFIL FEHLT");
		return sgTargetsBuffer_;
	}
	if (bw.panel == BwPanel::Measurement) {
		const MeasurementLuxSample& localLux    = snapshot.measurementStatus.localLux;
		const MeasurementLuxSample& wirelessLux = snapshot.measurementStatus.wirelessLux;
		std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
				  "LOCAL %.3f  WIRE %.3f  U%u",
				  localLux.valid    ? static_cast<double>(localLux.lux)    : 0.0,
				  wirelessLux.valid ? static_cast<double>(wirelessLux.lux) : 0.0,
				  static_cast<unsigned>(snapshot.measurementStatus.session.undoDepth));
		return sgTargetsBuffer_;
	}
	const char* dirtyMark = bw.parametersDirty ? " DIRTY" : "";
	if (bw.whiteLight) {
		if (bw.controlMode == ExposureControlMode::Dose) {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "BW G%.1f WEISS  ZIEL %.2f lx\xB7s%s",
					  static_cast<double>(bw.grade),
					  static_cast<double>(bw.targetValue), dirtyMark);
		} else {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "BW G%.1f WEISS  ZIEL %.1f s%s",
					  static_cast<double>(bw.grade),
					  static_cast<double>(bw.targetValue), dirtyMark);
		}
	} else {
		if (bw.controlMode == ExposureControlMode::Dose) {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "BW G%.1f  S%.0f%%/H%.0f%%  ZIEL %.2f lx\xB7s%s",
					  static_cast<double>(bw.grade),
					  static_cast<double>(bw.softMix * 100.0f),
					  static_cast<double>(bw.hardMix * 100.0f),
					  static_cast<double>(bw.targetValue), dirtyMark);
		} else {
			std::snprintf(sgTargetsBuffer_, sizeof(sgTargetsBuffer_),
					  "BW G%.1f  S%.0f%%/H%.0f%%  ZIEL %.1f s%s",
					  static_cast<double>(bw.grade),
					  static_cast<double>(bw.softMix * 100.0f),
					  static_cast<double>(bw.hardMix * 100.0f),
					  static_cast<double>(bw.targetValue), dirtyMark);
		}
	}
	return sgTargetsBuffer_;
}

const char* UiPresenter::getBwExposureMain(const SystemSnapshot& snapshot) {
	if (snapshot.modeState.activeMode != ModeId::BlackWhite) {
		sgExposureMainBuffer_[0] = '\0';
		return sgExposureMainBuffer_;
	}
	const BwModeRuntimeState& bw = snapshot.modeState.bw;
	if (bw.panel == BwPanel::Measurement) {
		const MeasurementLuxSample& activeLux = snapshot.measurementStatus.activeLux;
		const MeasurementReferenceStatus& activeReference = snapshot.measurementStatus.activeReference;
		std::snprintf(sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
				  "A %.3f lux  REF %.3f  dEV %+.2f",
				  activeLux.valid    ? static_cast<double>(activeLux.lux)    : 0.0,
				  activeReference.valid ? static_cast<double>(activeReference.lux) : 0.0,
				  snapshot.measurementStatus.activeRelativeEvValid
					  ? static_cast<double>(snapshot.measurementStatus.activeRelativeEvStops)
					  : 0.0);
	} else {
		MeasurementValueFormatter::formatSplitgradeDoseTelemetry(
			sgExposureMainBuffer_, sizeof(sgExposureMainBuffer_),
			snapshot.exposureState.currentDose, snapshot.exposureState.targetDose,
			snapshot.exposureState.remainingTimeSeconds,
			snapshot.exposureState.measuredLux);
	}
	return sgExposureMainBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementSources(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatMeasurementSources(
		measurementSourcesBuffer_, sizeof(measurementSourcesBuffer_),
		snapshot.measurementStatus.localLux, snapshot.measurementStatus.wirelessLux,
		snapshot.measurementStatus.activeLux.source);
	return measurementSourcesBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementMain(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatMeasurementMain(
		measurementMainBuffer_, sizeof(measurementMainBuffer_),
		snapshot.measurementStatus.activeLux);
	return measurementMainBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementMeta(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatMeasurementMeta(
		measurementMetaBuffer_, sizeof(measurementMetaBuffer_),
		snapshot.measurementStatus.activeLux);
	return measurementMetaBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementReference(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatMeasurementReference(
		measurementReferenceBuffer_, sizeof(measurementReferenceBuffer_),
		snapshot.measurementStatus.activeReference,
		snapshot.measurementStatus.session.mode,
		snapshot.measurementStatus.activeRelativeEvValid,
		snapshot.measurementStatus.activeRelativeEvStops);
	return measurementReferenceBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementRange(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatMeasurementRange(
		measurementRangeBuffer_, sizeof(measurementRangeBuffer_),
		snapshot.measurementStatus.session);
	return measurementRangeBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementStatusLeft(const SystemSnapshot& snapshot) {
	const MeasurementSessionStatus& session = snapshot.measurementStatus.session;
	char zoneBuffer[8] = {};
	if (session.sampleCount == 0u) {
		// Ohne sichtbare Session-History darf die Statuszeile keinen impliziten
		// Zonenindex 0 vorspiegeln. `Z--` markiert explizit den Leerlauf, waehrend
		// die Zaehlerwerte weiter sichtbar bleiben.
		std::snprintf(zoneBuffer, sizeof(zoneBuffer), "--");
	} else {
		std::snprintf(zoneBuffer,
		              sizeof(zoneBuffer),
		              "%u",
		              static_cast<unsigned>(session.latestSample.zoneIndex));
	}
	std::snprintf(measurementStatusLeftBuffer_, sizeof(measurementStatusLeftBuffer_),
				  "CUR %lu  CAP %lu  DR %lu  Z%s",
				  static_cast<unsigned long>(session.sampleCount),
				  static_cast<unsigned long>(session.capturedSampleCount),
				  static_cast<unsigned long>(session.droppedSampleCount),
				  zoneBuffer);
	return measurementStatusLeftBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementStatusCenter(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatMeasurementControls(
		measurementStatusCenterBuffer_, sizeof(measurementStatusCenterBuffer_),
		snapshot.measurementStatus.session);
	return measurementStatusCenterBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementStatusRight(const SystemSnapshot& snapshot) {
	std::snprintf(measurementStatusRightBuffer_, sizeof(measurementStatusRightBuffer_),
				  "AHT %.1fC %.0f%%  BMP %.0fhPa",
				  snapshot.espLinkStatus.serviceSensors.ahtTemperatureCelsius,
				  snapshot.espLinkStatus.serviceSensors.ahtHumidityPercent,
				  snapshot.espLinkStatus.serviceSensors.bmpPressureHpa);
	return measurementStatusRightBuffer_;
}

FLASHMEM const char* UiPresenter::getMeasurementUndoChip(const SystemSnapshot& snapshot) {
	const MeasurementSessionStatus& session = snapshot.measurementStatus.session;
	std::snprintf(measurementUndoChipBuffer_, sizeof(measurementUndoChipBuffer_),
				  "UNDO %u",
				  static_cast<unsigned>(session.undoDepth));
	return measurementUndoChipBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasLocalLux(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasLocalLux(
		pageMeasLocalLuxBuffer_, sizeof(pageMeasLocalLuxBuffer_),
		snapshot.measurementStatus.localLux);
	return pageMeasLocalLuxBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasWirelessLux(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasWirelessLux(
		pageMeasWirelessLuxBuffer_, sizeof(pageMeasWirelessLuxBuffer_),
		snapshot.measurementStatus.wirelessLux);
	return pageMeasWirelessLuxBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasSourceChip(const SystemSnapshot& snapshot) {
	return MeasurementValueFormatter::formatPageMeasSourceChip(
		snapshot.measurementStatus.activeLux.source,
		snapshot.measurementStatus.activeLux.valid);
}

FLASHMEM const char* UiPresenter::getPageMeasLuxMain(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasLuxMain(
		pageMeasLuxMainBuffer_, sizeof(pageMeasLuxMainBuffer_),
		snapshot.measurementStatus.activeLux);
	return pageMeasLuxMainBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasLuxAge(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasLuxAge(
		pageMeasLuxAgeBuffer_, sizeof(pageMeasLuxAgeBuffer_),
		snapshot.measurementStatus.activeLux);
	return pageMeasLuxAgeBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasRefLux(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasRefLux(
		pageMeasRefLuxBuffer_, sizeof(pageMeasRefLuxBuffer_),
		snapshot.measurementStatus.activeReference);
	return pageMeasRefLuxBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasEvDiff(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasEvDiff(
		pageMeasEvDiffBuffer_, sizeof(pageMeasEvDiffBuffer_),
		snapshot.measurementStatus.activeRelativeEvValid,
		snapshot.measurementStatus.activeRelativeEvStops);
	return pageMeasEvDiffBuffer_;
}

FLASHMEM const char* UiPresenter::getPageMeasSession(const SystemSnapshot& snapshot) {
	MeasurementValueFormatter::formatPageMeasSession(
		pageMeasSessionBuffer_, sizeof(pageMeasSessionBuffer_),
		snapshot.measurementStatus.session);
	return pageMeasSessionBuffer_;
}

const char* UiPresenter::getOverlayText(const SystemSnapshot& snapshot) {
	const InputModalState modalState = static_cast<InputModalState>(snapshot.inputModalStateCode);
	if (modalState == InputModalState::WorkflowFault) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "FEHLER %s - UNDO ODER MEASURE ZUM QUITTIEREN",
					  exposureFaultLabel(snapshot.exposureState.faultReason));
		return overlayBuffer_;
	}

	if (modalState == InputModalState::WorkflowConfirm) {
		if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
		    snapshot.modeState.splitgrade.executionState == SplitgradeExecutionState::WaitForFilter) {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "CONFIRM: FILTER WECHSELN, DANN CONFIRM ODER START");
		} else {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "CONFIRM: BEREIT ZUR BETAETIGUNG ODER UNDO");
		}
		return overlayBuffer_;
	}

	if (modalState == InputModalState::WorkflowWait) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "WAIT %s - NUR UNDO ZUM ABBRUCH AKTIV",
					  exposurePhaseLabel(snapshot.exposureState.phase));
		return overlayBuffer_;
	}

	if (snapshot.modeState.activeMode == ModeId::Setup) {
		if (snapshot.modeState.setup.persistFailed) {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "SETUP SAVE FEHLER %s - PRUEFE SD ODER DATEN",
					  systemSettingsStorageErrorLabel(
						  static_cast<SystemSettingsStorageError>(snapshot.modeState.setup.storageErrorCode)));
		} else if (snapshot.modeState.setup.editingActive) {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "SETUP EDIT - CONFIRM ODER UNDO");
		} else if (snapshot.modeState.setup.parametersDirty) {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "SETUP DIRTY - APPLY ODER DISCARD");
		} else {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "SETUP BEREIT - ENC3 LANG ZURUECK");
		}
		return overlayBuffer_;
	}

	if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
	    snapshot.modeState.splitgrade.panel == SplitgradePanel::Measurement) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
				  "MESSUNG BEREIT - START LOKAL, C6-BTN SAMPLE, UNDO ZUM LOESCHEN");
		return overlayBuffer_;
	}

	if (snapshot.modeState.activeMode == ModeId::BlackWhite &&
	    snapshot.modeState.bw.panel == BwPanel::Measurement) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
				  "MESSUNG BEREIT - START LOKAL, C6-BTN SAMPLE, UNDO ZUM LOESCHEN");
		return overlayBuffer_;
	}

	if (snapshot.localDoseControlForcedTime && !snapshot.exposureState.sensorFallbackActive) {
		if (snapshot.localDoseControlWatchdogResetLatched) {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "TSL BUS RESET - NUR TIMER BIS NEUSTART");
		} else {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "TSL %s - NUR TIMER BIS SENSOR OK",
					  localDoseControlLabel(snapshot));
		}
		return overlayBuffer_;
	}

	if (snapshot.exposureState.sensorFallbackActive) {
		if (snapshot.exposureState.phase == ExposurePhase::Exposing ||
			snapshot.exposureState.phase == ExposurePhase::Paused) {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "TSL FEHLER %s - CLOSED LOOP AUS, TIMER %.1fs, ERGEBNIS UNSICHER",
						  exposureFaultLabel(snapshot.exposureState.sensorFallbackReason),
						  snapshot.exposureState.remainingTimeSeconds);
		} else {
			std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
					  "TSL FAILSAFE %s - NUR TIMER, ERGEBNIS NICHT VERTRAUENSWUERDIG",
						  exposureFaultLabel(snapshot.exposureState.sensorFallbackReason));
		}
	} else if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
			   snapshot.modeState.splitgrade.executionState == SplitgradeExecutionState::Completed) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "BELICHTUNG FERTIG");
	} else if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
			   snapshot.modeState.splitgrade.executionState == SplitgradeExecutionState::Aborted) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "BELICHTUNG ABGEBROCHEN");
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite &&
			   snapshot.modeState.bw.executionState == BwExecutionState::Completed) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "BELICHTUNG FERTIG");
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite &&
			   snapshot.modeState.bw.executionState == BwExecutionState::Aborted) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "BELICHTUNG ABGEBROCHEN");
	} else if (snapshot.espLinkStatus.health == EspLinkHealth::Lost ||
			   snapshot.espLinkStatus.health == EspLinkHealth::Stale) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "REMOTE %s",
					  espLinkHealthLabel(snapshot.espLinkStatus.health));
	} else if (snapshot.modeState.activeMode == ModeId::Paper &&
		       snapshot.espLinkStatus.paperSlotRecovery.recommendation ==
			       protocol::PaperSlotRecoveryRecommendation::OfferRestoreBackup &&
		       (snapshot.espLinkStatus.paperSlotRecovery.flags &
			        protocol::kPaperSlotRecoveryDecisionStable) != 0u) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
				  "PAPER RECOVERY - BACKUP ZUM RESTORE VERFUEGBAR");
	} else if (snapshot.modeState.activeMode == ModeId::Paper &&
		       snapshot.espLinkStatus.paperSlotRecovery.recommendation ==
			       protocol::PaperSlotRecoveryRecommendation::RecoverViaUpload &&
		       (snapshot.espLinkStatus.paperSlotRecovery.flags &
			        protocol::kPaperSlotRecoveryDecisionStable) != 0u) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_),
				  "PAPER RECOVERY - HOST-UPLOAD FUER SLOT-DATEN NOETIG");
	} else if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
			   snapshot.modeState.splitgrade.parametersDirty) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "PARAMETER GEAENDERT");
	} else if (snapshot.modeState.activeMode == ModeId::Splitgrade) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "BEREIT: START ZUM BELICHTEN");
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite &&
			   snapshot.modeState.bw.parametersDirty) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "PARAMETER GEAENDERT");
	} else if (snapshot.modeState.activeMode == ModeId::BlackWhite) {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "BEREIT: START ZUM BELICHTEN");
	} else {
		std::snprintf(overlayBuffer_, sizeof(overlayBuffer_), "MODE %s BEREIT",
				  modeIdLabel(snapshot.modeState.activeMode));
	}
	return overlayBuffer_;
}

FLASHMEM const char* UiPresenter::getModeInfo(const SystemSnapshot& snapshot) {
	const InputFocusDomain focusDomain = static_cast<InputFocusDomain>(snapshot.inputFocusDomainCode);
	const InputModalState modalState = static_cast<InputModalState>(snapshot.inputModalStateCode);
	const InputEventGuardState eventGuardState =
		static_cast<InputEventGuardState>(snapshot.inputEventGuardStateCode);
	const InputControlOwner controlOwner = static_cast<InputControlOwner>(snapshot.inputControlOwnerCode);
	const char* dirtyLabel = snapshot.modeState.splitgrade.parametersDirty ? "DIRTY" : "CLEAN";
	if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
		snapshot.modeState.splitgrade.panel != SplitgradePanel::Inactive) {
		if (snapshot.modeState.lastInputEvent.isMeaningful()) {
			std::snprintf(modeInfoBuffer_, sizeof(modeInfoBuffer_),
					  "SG %s  EXEC %s  CTRL %s  S %.1f  H %.1f  G %.1f  %s  IN %s/%s %d  %lums  F %s/%lums M %s/%lums G %s/%lums O %s/%lums",
						  splitgradePanelLabel(snapshot.modeState.splitgrade.panel),
						  splitgradeExecutionStateLabel(snapshot.modeState.splitgrade.executionState),
						  exposureModeLabel(snapshot.modeState.splitgrade.controlMode),
						  snapshot.modeState.splitgrade.softTarget,
						  snapshot.modeState.splitgrade.hardTarget,
						  snapshot.modeState.splitgrade.grade, dirtyLabel,
						  normalizedInputSourceLabel(snapshot.modeState.lastInputEvent.source),
						  normalizedInputEventKindLabel(snapshot.modeState.lastInputEvent.eventKind),
						  static_cast<int>(snapshot.modeState.lastInputEvent.value),
						  static_cast<unsigned long>(snapshot.modeState.lastInputAgeMs),
						  inputFocusDomainLabel(focusDomain),
						  static_cast<unsigned long>(snapshot.inputFocusAgeMs),
						  inputModalStateLabel(modalState),
						  static_cast<unsigned long>(snapshot.inputModalAgeMs),
						  inputEventGuardLabel(eventGuardState),
						  static_cast<unsigned long>(snapshot.inputEventGuardAgeMs),
						  inputControlOwnerLabel(controlOwner),
						  static_cast<unsigned long>(snapshot.inputControlOwnerAgeMs));
		} else {
			std::snprintf(modeInfoBuffer_, sizeof(modeInfoBuffer_),
						  "SG %s  EXEC %s  CTRL %s  S %.1f  H %.1f  G %.1f  %s  P%lums E%lums  F %s/%lums M %s/%lums G %s/%lums O %s/%lums",
						  splitgradePanelLabel(snapshot.modeState.splitgrade.panel),
						  splitgradeExecutionStateLabel(snapshot.modeState.splitgrade.executionState),
						  exposureModeLabel(snapshot.modeState.splitgrade.controlMode),
						  snapshot.modeState.splitgrade.softTarget,
						  snapshot.modeState.splitgrade.hardTarget,
						  snapshot.modeState.splitgrade.grade, dirtyLabel,
						  static_cast<unsigned long>(snapshot.modeState.splitgrade.panelAgeMs),
						  static_cast<unsigned long>(snapshot.modeState.splitgrade.executionStateAgeMs),
						  inputFocusDomainLabel(focusDomain),
						  static_cast<unsigned long>(snapshot.inputFocusAgeMs),
						  inputModalStateLabel(modalState),
						  static_cast<unsigned long>(snapshot.inputModalAgeMs),
						  inputEventGuardLabel(eventGuardState),
						  static_cast<unsigned long>(snapshot.inputEventGuardAgeMs),
						  inputControlOwnerLabel(controlOwner),
						  static_cast<unsigned long>(snapshot.inputControlOwnerAgeMs));
		}
	} else if (snapshot.modeState.lastInputEvent.isMeaningful()) {
		std::snprintf(modeInfoBuffer_, sizeof(modeInfoBuffer_),
						  "MODE %s  REQ %s  INPUT %s/%s %d  %lums  F %s/%lums M %s/%lums G %s/%lums O %s/%lums",
					  modeIdLabel(snapshot.modeState.activeMode),
					  modeIdLabel(snapshot.modeState.requestedMode),
					  normalizedInputSourceLabel(snapshot.modeState.lastInputEvent.source),
					  normalizedInputEventKindLabel(snapshot.modeState.lastInputEvent.eventKind),
					  static_cast<int>(snapshot.modeState.lastInputEvent.value),
					  static_cast<unsigned long>(snapshot.modeState.lastInputAgeMs),
					  inputFocusDomainLabel(focusDomain),
					  static_cast<unsigned long>(snapshot.inputFocusAgeMs),
					  inputModalStateLabel(modalState),
					  static_cast<unsigned long>(snapshot.inputModalAgeMs),
					  inputEventGuardLabel(eventGuardState),
					  static_cast<unsigned long>(snapshot.inputEventGuardAgeMs),
					  inputControlOwnerLabel(controlOwner),
					  static_cast<unsigned long>(snapshot.inputControlOwnerAgeMs));
	} else {
		std::snprintf(modeInfoBuffer_, sizeof(modeInfoBuffer_),
					  "MODE %s  REQ %s  INPUT none  F %s/%lums M %s/%lums G %s/%lums O %s/%lums",
					  modeIdLabel(snapshot.modeState.activeMode),
					  modeIdLabel(snapshot.modeState.requestedMode),
					  inputFocusDomainLabel(focusDomain),
					  static_cast<unsigned long>(snapshot.inputFocusAgeMs),
					  inputModalStateLabel(modalState),
					  static_cast<unsigned long>(snapshot.inputModalAgeMs),
					  inputEventGuardLabel(eventGuardState),
					  static_cast<unsigned long>(snapshot.inputEventGuardAgeMs),
					  inputControlOwnerLabel(controlOwner),
					  static_cast<unsigned long>(snapshot.inputControlOwnerAgeMs));
	}

	return modeInfoBuffer_;
}
FLASHMEM const char* UiPresenter::getExposureInfo(const SystemSnapshot& snapshot) {
	std::snprintf(
		exposureInfoBuffer_, sizeof(exposureInfoBuffer_),
		"EXP P:%s A:%lums M:%s F:%s FD:%s L:%s DRT:%s BUS:%lums FB:%s DG:%s REG:%s",
		exposurePhaseLabel(snapshot.exposureState.phase),
		static_cast<unsigned long>(snapshot.exposureState.phaseAgeMs),
		exposureModeLabel(snapshot.exposureState.controlMode),
		exposureFaultLabel(snapshot.exposureState.faultReason),
		exposureFaultDomainLabel(snapshot.exposureState.faultReason),
		faultLatchLabel(snapshot.exposureState.faultLatched),
		onOffLabel(snapshot.exposureState.thermalDeratingActive),
		static_cast<unsigned long>(snapshot.exposureState.runtimeHeadBusLatencyMs),
		snapshot.exposureState.sensorFallbackActive
			? exposureFaultLabel(snapshot.exposureState.sensorFallbackReason)
			: "OFF",
		localDoseControlLabel(snapshot),
		exposureRegulationLabel(snapshot));
	return exposureInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getSensorInfo(const SystemSnapshot& snapshot) {
	const uint16_t serviceSensorFlags = snapshot.espLinkStatus.serviceSensors.sensorFlags;
	std::snprintf(sensorInfoBuffer_, sizeof(sensorInfoBuffer_),
				  "SENS TSL[H:%s V:%s F:%s D:%s A:%lums] DS[H:%s T:%s D:%s C:%.1f] AHT[O:%s F:%s T:%.1f H:%.1f] BMP[O:%s F:%s T:%.1f P:%.1f]",
				  sensorHealthLabel(snapshot.sensorStatus.tsl2561.health),
				  luxValidityLabel(snapshot.sensorStatus.tsl2561.sampleValidity),
				  sampleFreshLabel(snapshot.sensorStatus.tsl2561.sampleFresh),
				  tslDiagnosticReasonLabel(snapshot.sensorStatus.tsl2561.diagnosticReason),
				  static_cast<unsigned long>(snapshot.sensorStatus.tsl2561.sampleAgeMs),
				  sensorHealthLabel(snapshot.sensorStatus.ds18b20.health),
				  thermalStateLabel(snapshot.sensorStatus.ds18b20.thermalState),
				  ds18DiagnosticReasonLabel(snapshot.sensorStatus.ds18b20.diagnosticReason),
				  snapshot.sensorStatus.ds18b20.temperatureCelsius,
				  onOffLabel((serviceSensorFlags & protocol::kServiceSensorAhtPresent) != 0u),
				  onOffLabel((serviceSensorFlags & protocol::kServiceSensorAhtFault) != 0u),
				  snapshot.espLinkStatus.serviceSensors.ahtTemperatureCelsius,
				  snapshot.espLinkStatus.serviceSensors.ahtHumidityPercent,
				  onOffLabel((serviceSensorFlags & protocol::kServiceSensorBmpPresent) != 0u),
				  onOffLabel((serviceSensorFlags & protocol::kServiceSensorBmpFault) != 0u),
				  snapshot.espLinkStatus.serviceSensors.bmpTemperatureCelsius,
				  snapshot.espLinkStatus.serviceSensors.bmpPressureHpa);
	return sensorInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getRemoteCommandInfo(const SystemSnapshot& snapshot) {
	const RemoteCommandTrackerStatus& tracker = snapshot.remoteCommandTrackerStatus;
	std::snprintf(remoteCommandInfoBuffer_, sizeof(remoteCommandInfoBuffer_),
				  "RMT IN %u/%u ACK#%lu TRK#%lu RET %lu(#%lu) SAT %lu(#%lu) TMO %lu(#%lu)",
				  static_cast<unsigned>(tracker.inFlightCount),
				  static_cast<unsigned>(RemoteCommandTracker::kMaxInFlightCommands),
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.commandAckSequence),
				  static_cast<unsigned long>(tracker.lastTrackedSequence),
				  static_cast<unsigned long>(tracker.retryCount),
				  static_cast<unsigned long>(tracker.lastRetrySequence),
				  static_cast<unsigned long>(tracker.saturationCount),
				  static_cast<unsigned long>(tracker.lastSaturatedSequence),
				  static_cast<unsigned long>(tracker.timeoutCount),
				  static_cast<unsigned long>(tracker.lastTimedOutSequence));
	return remoteCommandInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getGatewayInfo(const SystemSnapshot& snapshot) {
	const PaperSlotStorageError storageError =
		static_cast<PaperSlotStorageError>(snapshot.paperSlotStorageErrorCode);
	const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
	const unsigned activeSlot = slotCount > 0u
	                        ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u
	                        : 0u;
	const MeasurementLuxSample& activeLux = snapshot.measurementStatus.activeLux;
	const MeasurementReferenceStatus& activeReference = snapshot.measurementStatus.activeReference;
	const MeasurementSessionStatus& measurementSession = snapshot.measurementStatus.session;
	const MeasurementSessionSample& latestMeasurement = measurementSession.latestSample;
	const MeasurementSessionSample& shadowMeasurement = measurementSession.shadowSample;
	const MeasurementSessionSample& highlightMeasurement = measurementSession.highlightSample;
	const EspTxQueueStatus& txQueue = snapshot.espLinkStatus.txQueue;
	const PaperSlotRecoveryStatus& paperRecovery = snapshot.espLinkStatus.paperSlotRecovery;
	const unsigned long txDropCount = static_cast<unsigned long>(txQueue.droppedHeartbeatCount) +
	                               static_cast<unsigned long>(txQueue.droppedRenderCount) +
	                               static_cast<unsigned long>(txQueue.droppedDiagnosticCount) +
	                               static_cast<unsigned long>(txQueue.droppedCommandCount) +
	                               static_cast<unsigned long>(txQueue.droppedVfsCount);
	const unsigned long txCoalescedCount = static_cast<unsigned long>(txQueue.coalescedRenderCount) +
	                                    static_cast<unsigned long>(txQueue.coalescedDiagnosticCount);
	std::snprintf(gatewayInfoBuffer_, sizeof(gatewayInfoBuffer_),
				  "ESP %s  RX %lums  TX %lums  TXQ %u/%u D%lu E%lu C%lu  EV %s/%s %d  WR %s %.3flux S%lu P%lu #%lu ACK#%lu B%u RD %s %lu/%lu  REC %s %04x A %s/%s B %s/%s  M %s %.3flux %lums #%lu dEV %+.2f REF %s %.3flux  MS %lu U%u Z%u RNG %+.2f..%+.2f dS %.2f dEV %+.2f #%lu  SV %04x  AHT %.1fC %.1f%%  BMP %.1fC %.1fhPa  DG %s %u/%lu  PS %s(%u/%lu)  SLOT %u/%u %s/%s %.16s",
				  espLinkHealthLabel(snapshot.espLinkStatus.health),
				  static_cast<unsigned long>(snapshot.espLinkStatus.lastRxAgeMs),
				  static_cast<unsigned long>(snapshot.espLinkStatus.lastTxAgeMs),
				  static_cast<unsigned>(txQueue.pendingFrameCount),
				  static_cast<unsigned>(txQueue.capacity),
				  txDropCount,
				  static_cast<unsigned long>(txQueue.evictedFrameCount),
				  txCoalescedCount,
				  remoteInputSourceLabel(snapshot.espLinkStatus.lastInputEvent.source),
				  inputEventKindLabel(snapshot.espLinkStatus.lastInputEvent.eventKind),
				  static_cast<int>(snapshot.espLinkStatus.lastInputEvent.value),
				  wirelessPeerStateLabel(snapshot.espLinkStatus.wireless.peerState),
				  snapshot.espLinkStatus.wireless.lastLux,
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.sensorSampleAgeMs),
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.lastSeenAgeMs),
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.measurementSequence),
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.commandAckSequence),
				  static_cast<unsigned>(snapshot.espLinkStatus.wireless.batteryPercent),
				  wirelessRenderStatusLabel(snapshot.espLinkStatus.wireless.renderStatusFlags),
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.staleRenderCount),
				  static_cast<unsigned long>(snapshot.espLinkStatus.wireless.renderTimeoutCount),
				  paperSlotRecoveryRecommendationLabel(paperRecovery.recommendation),
				  static_cast<unsigned>(paperRecovery.flags),
				  vfsStatusShortLabel(paperRecovery.activeVfsStatus),
				  paperSlotParseCodeLabel(paperRecovery.activeParseError),
				  vfsStatusShortLabel(paperRecovery.backupVfsStatus),
				  paperSlotParseCodeLabel(paperRecovery.backupParseError),
				  measurementLuxSourceLabel(activeLux.source),
				  activeLux.valid ? activeLux.lux : 0.0f,
				  static_cast<unsigned long>(activeLux.ageMs),
				  static_cast<unsigned long>(activeLux.sequence),
				  snapshot.measurementStatus.activeRelativeEvValid
					  ? snapshot.measurementStatus.activeRelativeEvStops
					  : 0.0f,
				  measurementLuxSourceLabel(activeReference.source),
				  activeReference.valid ? activeReference.lux : 0.0f,
				  static_cast<unsigned long>(measurementSession.sampleCount),
				  static_cast<unsigned>(measurementSession.undoDepth),
				  static_cast<unsigned>(latestMeasurement.zoneIndex),
				  measurementSession.rangeValid ? shadowMeasurement.relativeEvStops : 0.0f,
				  measurementSession.rangeValid ? highlightMeasurement.relativeEvStops : 0.0f,
				  measurementSession.rangeValid ? measurementSession.relativeEvSpanStops : 0.0f,
				  latestMeasurement.relativeEvStops,
				  static_cast<unsigned long>(latestMeasurement.sequence),
				  static_cast<unsigned>(snapshot.espLinkStatus.serviceSensors.sensorFlags),
				  snapshot.espLinkStatus.serviceSensors.ahtTemperatureCelsius,
				  snapshot.espLinkStatus.serviceSensors.ahtHumidityPercent,
				  snapshot.espLinkStatus.serviceSensors.bmpTemperatureCelsius,
				  snapshot.espLinkStatus.serviceSensors.bmpPressureHpa,
				  diagnosticCodeLabel(snapshot.espLinkStatus.diagnostic.code),
				  static_cast<unsigned>(snapshot.espLinkStatus.diagnostic.detail),
				  static_cast<unsigned long>(snapshot.espLinkStatus.diagnostic.counter),
				  paperSlotStorageErrorLabel(storageError),
				  static_cast<unsigned>(snapshot.paperSlotStorageErrorCode),
				  static_cast<unsigned long>(snapshot.paperSlotStorageErrorDetail),
				  activeSlot,
				  slotCount,
				  snapshot.paperActiveSlotCalibrated ? "CAL" : "RAW",
				  paperGradeModeShortLabel(snapshot.paperActiveGradeMode),
				  snapshot.paperActiveSlotName);
	return gatewayInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getEncoderInfo(const SystemSnapshot& snapshot) {
	std::snprintf(encoderInfoBuffer_,
	              sizeof(encoderInfoBuffer_),
	              "E1 %ld R%lu M%lu  E2 %ld R%lu M%lu  E3 %ld R%lu M%lu  ST %s",
	              snapshot.encoder1Position,
	              static_cast<unsigned long>(snapshot.encoder1Status.partialReverseCount),
	              static_cast<unsigned long>(snapshot.encoder1Status.phaseMismatchCount),
	              snapshot.encoder2Position,
	              static_cast<unsigned long>(snapshot.encoder2Status.partialReverseCount),
	              static_cast<unsigned long>(snapshot.encoder2Status.phaseMismatchCount),
	              snapshot.encoder3Position,
	              static_cast<unsigned long>(snapshot.encoder3Status.partialReverseCount),
	              static_cast<unsigned long>(snapshot.encoder3Status.phaseMismatchCount),
	              onOffLabel(snapshot.startButtonActive));
	return encoderInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getSwitchInfo(const SystemSnapshot& snapshot) {
	std::snprintf(switchInfoBuffer_, sizeof(switchInfoBuffer_), "SW FOCUS:%s SAVE:%s ROOM:%s LATCH:%s",
				  onOffLabel(snapshot.lightState.focusSwitchRaw),
				  onOffLabel(snapshot.lightState.saveSwitchRaw),
				  onOffLabel(snapshot.lightState.roomSwitchRaw),
				  onOffLabel(snapshot.lightState.saveLatchActive));
	return switchInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getTouchInfo(const SystemSnapshot& snapshot) {
	if (snapshot.touchState.active) {
		std::snprintf(touchInfoBuffer_, sizeof(touchInfoBuffer_), "TOUCH raw %d / %d / %d",
					  snapshot.touchState.rawX, snapshot.touchState.rawY,
					  snapshot.touchState.rawZ);
	} else {
		std::snprintf(touchInfoBuffer_, sizeof(touchInfoBuffer_), "TOUCH idle");
	}

	return touchInfoBuffer_;
}

FLASHMEM const char* UiPresenter::getDmaInfo(const SystemSnapshot& snapshot,
									bool fbEnabled,
									bool dmaSupported,
									bool dmaActive) {
	const PaperSlotStorageError storageError =
		static_cast<PaperSlotStorageError>(snapshot.paperSlotStorageErrorCode);
	const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
	const unsigned activeSlot = slotCount > 0u
	                        ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u
	                        : 0u;
	std::snprintf(dmaInfoBuffer_, sizeof(dmaInfoBuffer_),
				  "Flush %s  DMA %s  pending %s  PS %s e:%u d:%lu  Slot %u/%u %s/%s %.16s",
				  fbEnabled ? "FULLFB" : "PARTIAL", dmaSupported ? "YES" : "NO",
				  dmaActive ? "YES" : "NO",
				  paperSlotStorageErrorLabel(storageError),
				  static_cast<unsigned>(snapshot.paperSlotStorageErrorCode),
				  static_cast<unsigned long>(snapshot.paperSlotStorageErrorDetail),
				  activeSlot,
				  slotCount,
				  paperCalibrationShortLabel(snapshot.paperActiveSlotCalibrated),
				  paperGradeModeShortLabel(snapshot.paperActiveGradeMode),
				  snapshot.paperActiveSlotName);
	return dmaInfoBuffer_;
}
} // namespace dukatimer
