#include "BlackWhiteWorkflow.h"

#include <Arduino.h>
#include "ExposureValueMath.h"
#include "SplitgradeMath.h"

namespace dukatimer {

/*
 * BlackWhiteWorkflow
 *
 * Realisiert eine einzelne BW/SW-Belichtung auf Basis des aktiven Papierprofils.
 * - FixedGrade: Weißlicht, Gradation durch Papierfiltereinsatz physisch bestimmt
 * - Multigrade: Soft/Hard-Mix nach Gradationseinstellung (kein Filterwechsel)
 *
 * Architektur: Analog zu SplitgradeWorkflow, jedoch vereinfacht:
 * - eine Belichtungsphase statt Soft+Hard-Sequenz
 * - keine Override-Tabelle fuer Gradationsbrueche
 * - Soft/Hard-Mix als Ausgabewert fuer den Wiring-Layer (NeoPixel-Farbe)
 */

namespace {

using namespace dukatimer::splitgrade_math;

bool isExposureActivePhase(ExposurePhase phase) {
	return phase == ExposurePhase::PreWait || phase == ExposurePhase::Exposing ||
	       phase == ExposurePhase::Paused  || phase == ExposurePhase::PostWait;
}

PaperExposureProfile makeBwDefaultPaperProfile() {
	PaperExposureProfile profile = {};
	profile.schemaVersion = kPaperExposureProfileSchemaVersion;
	profile.calibrated    = false;
	profile.gradeMode     = PaperGradeMode::Multigrade;
	profile.useIsoMath    = false;
	profile.fixedGradeValue = 2.5f;
	profile.isoP = 100.0f;
	profile.isoR = 100.0f;
	profile.kBw  = 0.0f;
	profile.kSoft = 10.0f;
	profile.kHard = 10.0f;
	for (uint8_t i = 0; i < kSplitgradeStepCount; ++i) {
		const float hardFactor = static_cast<float>(i) / 10.0f;
		profile.gradeKSoft[i] = profile.kSoft * (1.0f - hardFactor);
		profile.gradeKHard[i] = profile.kHard * hardFactor;
	}
	return profile;
}

}  // namespace

// ---------------------------------------------------------------------------
// IModeWorkflow interface
// ---------------------------------------------------------------------------

ModeId BlackWhiteWorkflow::modeId() const { return ModeId::BlackWhite; }

const char* BlackWhiteWorkflow::shortName() const { return "BW"; }

FLASHMEM void BlackWhiteWorkflow::onEnter(uint32_t nowMs) {
	if (!initialized_ || shouldReloadPaperDrivenState()) {
		initialized_ = true;
		initializePaperDrivenState();
	}

	active_              = true;
	panel_               = BwPanel::Target;
	enteredAtMs_         = nowMs;
	panelChangedAtMs_    = nowMs;
	pendingCommand_      = {};
	observedExposurePhase_ = ExposurePhase::Idle;
	setExecutionState(BwExecutionState::IdleConfig, nowMs);
}

FLASHMEM void BlackWhiteWorkflow::onExit(uint32_t nowMs) {
	if (executionState_ == BwExecutionState::Arming ||
	    executionState_ == BwExecutionState::Exposing) {
		queueCommand(BlackWhiteExecutionCommandKind::AbortExposure);
	}

	active_              = false;
	panel_               = BwPanel::Inactive;
	enteredAtMs_         = 0;
	panelChangedAtMs_    = 0;
	observedExposurePhase_ = ExposurePhase::Idle;
	setExecutionState(BwExecutionState::Inactive, nowMs);
}

void BlackWhiteWorkflow::onTick(uint32_t nowMs) {
	(void)nowMs;
}

FLASHMEM void BlackWhiteWorkflow::onInputEvent(const NormalizedInputEvent& event,
                                                const InputSemanticAction& action,
                                                uint32_t nowMs) {
	if (!active_) return;

	switch (action.kind) {
		case InputSemanticActionKind::AdjustPrimaryIncrease:
			adjustPrimaryValue(+1, nowMs);
			return;
		case InputSemanticActionKind::AdjustPrimaryDecrease:
			adjustPrimaryValue(-1, nowMs);
			return;
		case InputSemanticActionKind::AdjustSecondaryIncrease:
			// Enc2 auf Grade-Panel: Gradation erhoehen
			if (panel_ == BwPanel::Grade && gradeEditable_) {
				grade_ = normalizeWorkflowGrade(grade_ + 0.5f);
				if (grade_ > 5.0f) grade_ = 5.0f;
				refreshMixFractions();
				parametersDirty_ = true;
			} else if (panel_ == BwPanel::Target) {
				toggleControlMode();
			}
			return;
		case InputSemanticActionKind::AdjustSecondaryDecrease:
			if (panel_ == BwPanel::Grade && gradeEditable_) {
				grade_ = normalizeWorkflowGrade(grade_ - 0.5f);
				if (grade_ < 0.0f) grade_ = 0.0f;
				refreshMixFractions();
				parametersDirty_ = true;
			} else if (panel_ == BwPanel::Target) {
				toggleControlMode();
			}
			return;
		case InputSemanticActionKind::ContextNext:
			cyclePanel(+1, nowMs);
			return;
		case InputSemanticActionKind::ContextPrevious:
			cyclePanel(-1, nowMs);
			return;
		default:
			break;
	}

	(void)handleExecutionAction(event, action, nowMs);
}

FLASHMEM void BlackWhiteWorkflow::populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const {
	state.bw.panel          = active_ ? panel_ : BwPanel::Inactive;
	state.bw.executionState = active_ ? executionState_ : BwExecutionState::Inactive;
	state.bw.controlMode    = controlMode_;
	state.bw.targetValue    = targetValue_;
	state.bw.grade          = grade_;
	state.bw.gradeEditable  = gradeEditable_;
	state.bw.whiteLight     = whiteLight_;
	state.bw.softMix        = softMix_;
	state.bw.hardMix        = hardMix_;
	state.bw.parametersDirty = parametersDirty_;
	if (!active_ || panel_ == BwPanel::Inactive || panelChangedAtMs_ == 0) {
		state.bw.panelAgeMs = 0;
	} else {
		state.bw.panelAgeMs = nowMs - panelChangedAtMs_;
	}
	if (!active_ || executionStateChangedAtMs_ == 0 ||
	    executionState_ == BwExecutionState::Inactive) {
		state.bw.executionStateAgeMs = 0;
	} else {
		state.bw.executionStateAgeMs = nowMs - executionStateChangedAtMs_;
	}
	(void)enteredAtMs_;
}

// ---------------------------------------------------------------------------
// Public non-interface methods
// ---------------------------------------------------------------------------

FLASHMEM void BlackWhiteWorkflow::observeExposureState(
        const ExposureRuntimeState& exposureState, uint32_t nowMs) {
	if (!active_) return;

	observedExposurePhase_ = exposureState.phase;

	if (exposureState.phase == ExposurePhase::Fault) {
		setExecutionState(BwExecutionState::Fault, nowMs);
		return;
	}

	switch (executionState_) {
		case BwExecutionState::Arming:
			if (isExposureActivePhase(exposureState.phase)) {
				setExecutionState(BwExecutionState::Exposing, nowMs);
			} else if (exposureState.phase == ExposurePhase::Done ||
			           ((nowMs - executionStateChangedAtMs_) > 200 &&
			            exposureState.phase == ExposurePhase::Idle)) {
				setExecutionState(BwExecutionState::Completed, nowMs);
			}
			break;

		case BwExecutionState::Exposing:
			if (exposureState.phase == ExposurePhase::Done ||
			    exposureState.phase == ExposurePhase::Idle) {
				setExecutionState(BwExecutionState::Completed, nowMs);
			}
			break;

		case BwExecutionState::Completed:
			if (exposureState.phase == ExposurePhase::Idle) {
				setExecutionState(BwExecutionState::IdleConfig, nowMs);
			}
			break;

		case BwExecutionState::Fault:
			if (exposureState.phase == ExposurePhase::Idle && !exposureState.faultLatched) {
				setExecutionState(BwExecutionState::Aborted, nowMs);
			}
			break;

		case BwExecutionState::IdleConfig:
		case BwExecutionState::Aborted:
		case BwExecutionState::Inactive:
			break;
	}
}

bool BlackWhiteWorkflow::consumeExecutionCommand(BlackWhiteExecutionCommand& commandOut) {
	if (!pendingCommand_.isMeaningful()) return false;
	commandOut = pendingCommand_;
	pendingCommand_ = {};
	return true;
}

FLASHMEM void BlackWhiteWorkflow::setDoseControlForcedTime(bool forcedTime) {
	if (doseControlForcedTime_ == forcedTime) return;

	doseControlForcedTime_ = forcedTime;
	if (!doseControlForcedTime_) return;

	controlMode_ = ExposureControlMode::Time;
	if (pendingCommand_.kind == BlackWhiteExecutionCommandKind::Start) {
		pendingCommand_.controlMode = ExposureControlMode::Time;
	}
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

FLASHMEM void BlackWhiteWorkflow::cyclePanel(int8_t direction, uint32_t nowMs) {
	if (!active_ || direction == 0) return;

	int next = static_cast<int>(panel_);
	if (next < static_cast<int>(BwPanel::Target) ||
	    next > static_cast<int>(BwPanel::Measurement)) {
		next = static_cast<int>(BwPanel::Target);
	}

	next += (direction > 0) ? 1 : -1;
	if (next > static_cast<int>(BwPanel::Measurement)) {
		next = static_cast<int>(BwPanel::Target);
	}
	if (next < static_cast<int>(BwPanel::Target)) {
		next = static_cast<int>(BwPanel::Measurement);
	}

	panel_            = static_cast<BwPanel>(next);
	panelChangedAtMs_ = nowMs;
}

FLASHMEM void BlackWhiteWorkflow::adjustPrimaryValue(int8_t direction, uint32_t nowMs) {
	if (direction == 0) return;

	switch (panel_) {
		case BwPanel::Target:
			targetValue_ = clampTargetValue(
			    ExposureValueMath::applyStepDirection(targetValue_, direction));
			parametersDirty_ = true;
			break;
		case BwPanel::Grade:
			if (gradeEditable_) {
				grade_ = normalizeWorkflowGrade(grade_ + direction * 0.5f);
				if (grade_ < 0.0f) grade_ = 0.0f;
				if (grade_ > 5.0f) grade_ = 5.0f;
				refreshMixFractions();
				parametersDirty_ = true;
			}
			break;
		case BwPanel::ControlMode:
			toggleControlMode();
			break;
		case BwPanel::Measurement: {
			const ModeWorkflowServices* boundServices = services();
			if (boundServices != nullptr && boundServices->measurementCommands != nullptr) {
				(void)boundServices->measurementCommands->cyclePendingCaptureRole(direction);
			}
			break;
		}
		case BwPanel::Inactive:
			break;
	}
	(void)nowMs;
}

FLASHMEM bool BlackWhiteWorkflow::shouldReloadPaperDrivenState() const {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfiles == nullptr) {
		return !paperProfileLoaded_;
	}
	const PaperExposureProfile* activeProfile = boundServices->paperProfiles->activeProfile();
	if (activeProfile == nullptr) {
		return !paperProfileLoaded_;
	}
	return !paperProfileLoaded_ || *activeProfile != paperProfile_;
}

FLASHMEM void BlackWhiteWorkflow::initializePaperDrivenState() {
	controlMode_         = ExposureControlMode::Time;
	paperProfileLoaded_  = false;

	if (const ModeWorkflowServices* boundServices = services();
	    boundServices != nullptr && boundServices->paperProfiles != nullptr) {
		if (const PaperExposureProfile* activeProfile =
		        boundServices->paperProfiles->activeProfile();
		    activeProfile != nullptr) {
			paperProfile_       = *activeProfile;
			paperProfileLoaded_ = true;
		}
	}

	if (!paperProfileLoaded_) {
		paperProfile_       = makeBwDefaultPaperProfile();
		paperProfileLoaded_ = true;
	}

	// E17-Entflechtung: kBw ist ein dimensionsloser Transmissionsfaktor und
	// darf nicht als Belichtungszeit interpretiert werden.
	targetValue_ = kDefaultTargetValue;

	if (isFixedGradeMode(paperProfile_.gradeMode)) {
		// FixedGrade: Gradation ist durch das Papier / die Filtereinstellung
		// vorgegeben. Weißlicht-Modus, da keine Multigrade-Trennung stattfindet.
		grade_        = normalizeWorkflowGrade(paperProfile_.fixedGradeValue);
		gradeEditable_ = false;
		whiteLight_   = true;
		softMix_      = 0.0f;
		hardMix_      = 0.0f;
	} else {
		// Multigrade: Soft/Hard-Mix nach Gradationseinstellung.
		grade_        = normalizeWorkflowGrade(kDefaultGrade);
		gradeEditable_ = true;
		whiteLight_   = false;
		refreshMixFractions();
	}

	parametersDirty_ = false;
}

FLASHMEM void BlackWhiteWorkflow::refreshMixFractions() {
	if (isFixedGradeMode(paperProfile_.gradeMode)) {
		softMix_ = 0.0f;
		hardMix_ = 0.0f;
		return;
	}

	const uint8_t gradeIndex = static_cast<uint8_t>(gradeIndexFromFloat(grade_));
	const float softFraction = profileLutSoftFraction(paperProfile_, gradeIndex);
	softMix_ = clampUnit(softFraction);
	hardMix_ = clampUnit(1.0f - softFraction);
}

FLASHMEM bool BlackWhiteWorkflow::startCurrentExecution(uint32_t nowMs) {
	if (targetValue_ < kMinTargetValue) {
		return false;
	}
	queueCommand(BlackWhiteExecutionCommandKind::Start);
	setExecutionState(BwExecutionState::Arming, nowMs);
	return true;
}

FLASHMEM bool BlackWhiteWorkflow::handleExecutionAction(const NormalizedInputEvent& event,
                                                         const InputSemanticAction& action,
                                                         uint32_t nowMs) {
	switch (action.kind) {
		case InputSemanticActionKind::Start:
			if (panel_ == BwPanel::Measurement) {
				const ModeWorkflowServices* boundServices = services();
				if (boundServices != nullptr && boundServices->measurementCommands != nullptr) {
					(void)boundServices->measurementCommands->captureLocalSample(nowMs);
				}
				return true;
			}
			if (executionState_ == BwExecutionState::Arming ||
			    executionState_ == BwExecutionState::Exposing) {
				if (observedExposurePhase_ == ExposurePhase::Exposing) {
					queueCommand(BlackWhiteExecutionCommandKind::PauseExposure);
					return true;
				}
				if (observedExposurePhase_ == ExposurePhase::Paused) {
					queueCommand(BlackWhiteExecutionCommandKind::ResumeExposure);
					return true;
				}
			}
			if (executionState_ == BwExecutionState::IdleConfig ||
			    executionState_ == BwExecutionState::Aborted    ||
			    executionState_ == BwExecutionState::Completed) {
				(void)startCurrentExecution(nowMs);
				return true;
			}
			if (executionState_ == BwExecutionState::Fault) {
				queueCommand(BlackWhiteExecutionCommandKind::ClearFault);
				setExecutionState(BwExecutionState::Aborted, nowMs);
				return true;
			}
			return true;

		case InputSemanticActionKind::Measure:
			if (executionState_ == BwExecutionState::Fault) {
				queueCommand(BlackWhiteExecutionCommandKind::ClearFault);
				setExecutionState(BwExecutionState::Aborted, nowMs);
				return true;
			}
			if (panel_ == BwPanel::Measurement) {
				if (event.source == NormalizedInputSource::WirelessMeasureButton) {
					return true;
				}
				const ModeWorkflowServices* boundServices = services();
				if (boundServices != nullptr && boundServices->measurementCommands != nullptr) {
					(void)boundServices->measurementCommands->captureLocalSample(nowMs);
				}
				return true;
			}
			return true;

		case InputSemanticActionKind::Confirm:
			if (executionState_ == BwExecutionState::Completed) {
				queueCommand(BlackWhiteExecutionCommandKind::AcknowledgeDone);
				setExecutionState(BwExecutionState::IdleConfig, nowMs);
				return true;
			}
			break;

		case InputSemanticActionKind::Pause:
			if (executionState_ == BwExecutionState::Arming ||
			    executionState_ == BwExecutionState::Exposing) {
				queueCommand(BlackWhiteExecutionCommandKind::PauseExposure);
				return true;
			}
			break;

		case InputSemanticActionKind::Resume:
			if (executionState_ == BwExecutionState::Arming ||
			    executionState_ == BwExecutionState::Exposing) {
				queueCommand(BlackWhiteExecutionCommandKind::ResumeExposure);
				return true;
			}
			break;

		case InputSemanticActionKind::Undo:
			if (executionState_ == BwExecutionState::Arming ||
			    executionState_ == BwExecutionState::Exposing) {
				queueCommand(BlackWhiteExecutionCommandKind::AbortExposure);
				setExecutionState(BwExecutionState::Aborted, nowMs);
				return true;
			}
			if (executionState_ == BwExecutionState::Completed ||
			    executionState_ == BwExecutionState::Aborted) {
				setExecutionState(BwExecutionState::IdleConfig, nowMs);
				return true;
			}
			if (executionState_ == BwExecutionState::Fault) {
				queueCommand(BlackWhiteExecutionCommandKind::ClearFault);
				setExecutionState(BwExecutionState::Aborted, nowMs);
				return true;
			}
			break;

		default:
			break;
	}

	return false;
}

void BlackWhiteWorkflow::setExecutionState(BwExecutionState newState, uint32_t nowMs) {
	if (executionState_ == newState) return;
	executionState_              = newState;
	executionStateChangedAtMs_  = nowMs;
}

void BlackWhiteWorkflow::queueCommand(BlackWhiteExecutionCommandKind kind) {
	pendingCommand_.kind         = kind;
	pendingCommand_.controlMode  = controlMode_;
	pendingCommand_.targetValue  = targetValue_;
	pendingCommand_.valid        = true;
}

FLASHMEM void BlackWhiteWorkflow::toggleControlMode() {
	if (doseControlForcedTime_) {
		controlMode_ = ExposureControlMode::Time;
		return;
	}
	controlMode_ = (controlMode_ == ExposureControlMode::Dose)
	    ? ExposureControlMode::Time
	    : ExposureControlMode::Dose;
	parametersDirty_ = true;
}

/*static*/ float BlackWhiteWorkflow::clampTargetValue(float value) {
	if (value < kMinTargetValue) return kMinTargetValue;
	if (value > kMaxTargetValue) return kMaxTargetValue;
	return value;
}

}  // namespace dukatimer
