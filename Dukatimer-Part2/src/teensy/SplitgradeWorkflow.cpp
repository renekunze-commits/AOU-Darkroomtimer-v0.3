#include "SplitgradeWorkflow.h"

#include "ExposureValueMath.h"
#include "SplitgradeMath.h"

namespace dukatimer {

/*
 * SplitgradeWorkflow
 *
 * Historische Einordnung:
 * - uebernimmt den SG-Bedienkern, der in den historischen Versionen stark an
 *   globale Zustands- und Timerlogik gekoppelt war
 * - trennt in Part2 erstmals sauber zwischen SG-Fachzustand, normierten
 *   Eingabeaktionen und der spaeteren physischen Belichtungsausfuehrung
 *
 * Wichtige Invariante:
 * - diese Datei darf SG-Fachlogik besitzen, aber keine Hardware schalten,
 *   keine UI-Texte aufbauen und keine Sensorwerte direkt lesen
 */


namespace {

// Gemeinsame Konstanten und Basisfunktionen werden aus SplitgradeMath.h bezogen.
using namespace dukatimer::splitgrade_math;

bool isExposureActivePhase(ExposurePhase phase) {
// Fuer die Rueckkopplung vom ExposureRuntimeState in den SG-Ablauf ist nur
// relevant, ob die Engine tatsaechlich in einer aktiven Belichtungsphase ist.
	return phase == ExposurePhase::PreWait || phase == ExposurePhase::Exposing ||
	       phase == ExposurePhase::Paused || phase == ExposurePhase::PostWait;
}

PaperExposureProfile makeDefaultPaperProfile() {
	PaperExposureProfile profile = {};
	profile.schemaVersion = kPaperExposureProfileSchemaVersion;
	profile.calibrated = false;
	profile.gradeMode = PaperGradeMode::Multigrade;
	profile.useIsoMath = false;
	profile.fixedGradeValue = 2.5f;
	profile.isoP = 100.0f;
	profile.isoR = 100.0f;
	// kBw ist ein dimensionsloser Faktor; 0.0f markiert "nicht kalibriert". (E17)
	profile.kBw = 0.0f;
	profile.kSoft = 10.0f;
	profile.kHard = 10.0f;
	for (uint8_t gradeIndex = 0; gradeIndex < kSplitgradeStepCount; ++gradeIndex) {
		const float hardFactor = static_cast<float>(gradeIndex) / 10.0f;
		profile.gradeKSoft[gradeIndex] = profile.kSoft * (1.0f - hardFactor);
		profile.gradeKHard[gradeIndex] = profile.kHard * hardFactor;
	}
	return profile;
}

}  // namespace

	// Beim ersten Eintritt werden nur die persistent gedachten SG-Defaults gesetzt.
	// Spaetere Wiedereintritte sollen dagegen den zuletzt aufgebauten SG-Zustand
	// erhalten und nur den sichtbaren Bedienkontext zuruecksetzen.
ModeId SplitgradeWorkflow::modeId() const {
	return ModeId::Splitgrade;
}

const char* SplitgradeWorkflow::shortName() const {
	return "SG";
}

void SplitgradeWorkflow::onEnter(uint32_t nowMs) {
	if (!initialized_ || shouldReloadPaperDrivenState()) {
	// Ein Moduswechsel darf keine haengende SG-Belichtung zuruecklassen. Der
	// Workflow fordert deshalb aktiv einen Abort an, falls die Engine noch einen
	// Soft- oder Hard-Teil ausfuehrt.
		initialized_ = true;
		initializePaperDrivenState();
	}

	active_ = true;
	panel_ = SplitgradePanel::SplitTargets;
	// SG reagiert ausschliesslich auf semantische Aktionen. Welche Hardware diese
	// ausgelost hat, ist hier absichtlich egal: Encoder 1-3, Touch oder Remote
	// sollen denselben SG-Fachpfad nutzen.
	enteredAtMs_ = nowMs;
	panelChangedAtMs_ = nowMs;
	pendingCommand_ = {};
	observedExposurePhase_ = ExposurePhase::Idle;
	setExecutionState(SplitgradeExecutionState::IdleConfig, nowMs);
}

bool SplitgradeWorkflow::shouldReloadPaperDrivenState() const {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfiles == nullptr) {
		return !paperProfileLoaded_;
	}

	// SG soll seinen lokal aufgebauten Parameterkontext behalten, solange das
	// zugrunde liegende Papierprofil unveraendert blieb. Wenn PAPER aber einen
	// anderen Slot aktiviert hat, muss PRINT beim Wiedereintritt dieselbe neue
	// fotografische Basis sehen wie Snapshot und UI.
	const PaperExposureProfile* activeProfile = boundServices->paperProfiles->activeProfile();
	if (activeProfile == nullptr) {
		return !paperProfileLoaded_;
	}

	return !paperProfileLoaded_ || *activeProfile != paperProfile_;
}
	// Der Workflow spiegelt den physischen Engine-Zustand in seinen eigenen,
	// fachlich verstehbaren SG-Ablauf zurueck. Erst dadurch kann die UI zwischen
	// "Soft laeuft", "Filterwechsel", "Hard laeuft", "Done" und "Fault"
	// unterscheiden, ohne die Engine-Details selbst auszuwerten.

void SplitgradeWorkflow::onExit(uint32_t nowMs) {
	(void)flushPendingPaperProfilePersist(nowMs, true);

	if (executionState_ == SplitgradeExecutionState::ArmingSoft ||
	    executionState_ == SplitgradeExecutionState::ExposingSoft ||
	    executionState_ == SplitgradeExecutionState::ArmingHard ||
	// RuntimeState bleibt bewusst roh und formatterfrei. Die UI oder spaetere
	// Presenter sollen aus diesen Feldern sichtbare Texte ableiten, nicht der
	// Workflow selbst.
	    executionState_ == SplitgradeExecutionState::ExposingHard) {
		queueCommand(SplitgradeExecutionCommandKind::AbortExposure);
	}

	active_ = false;
	panel_ = SplitgradePanel::Inactive;
	enteredAtMs_ = 0;
	panelChangedAtMs_ = 0;
	observedExposurePhase_ = ExposurePhase::Idle;
	setExecutionState(SplitgradeExecutionState::Inactive, nowMs);
	// Die SG-Bedienoberflaeche bildet einen kleinen Ring aus konfigurierbaren
	// Panels. Dadurch bleibt das Navigationsmodell fuer lokale und spaetere
	// Remote-Eingaben identisch.
}

void SplitgradeWorkflow::onTick(uint32_t nowMs) {
	(void)flushPendingPaperProfilePersist(nowMs, false);
}
	// Beim Start wird der aktuell sichtbare Soft- oder Hard-Target-Wert in ein
	// explizites Kommando materialisiert. Damit bleibt spaeter nachvollziehbar,
	// welche Zielgroesse die Engine fuer genau diesen Teilstart erhalten hat.

void SplitgradeWorkflow::onInputEvent(const NormalizedInputEvent& event,
	                                const InputSemanticAction& action,
	                                uint32_t nowMs) {
	if (!active_ || !event.isMeaningful() || !action.isMeaningful()) {
		return;
	// Diese kleine Zustandsmaschine uebersetzt semantische Bedienabsichten in die
	// SG-spezifische Abfolge aus Soft-Start, Filterwechsel, Hard-Start, Abort,
	// Done-Quittierung und Fault-Clear. Genau hier lebt damit der eigentliche
	// fotografische SG-Workflow oberhalb der generischen ExposureEngine.
	}

	if (handleExecutionAction(event, action, nowMs)) {
	// Parameter duerfen nur veraendert werden, solange keine laufende oder
	// wartende Belichtung den aktuellen Satz bereits "verbraucht" hat.
		return;
	}

	if (!isEditingAllowed()) {
		return;
	}

	switch (action.kind) {
		case InputSemanticActionKind::AdjustPrimaryDecrease:
			adjustPrimaryValue(-1, nowMs);
			break;
		case InputSemanticActionKind::AdjustPrimaryIncrease:
			adjustPrimaryValue(1, nowMs);
			break;
		case InputSemanticActionKind::AdjustSecondaryDecrease:
			adjustSecondaryValue(-1, nowMs);
			break;
		case InputSemanticActionKind::AdjustSecondaryIncrease:
			adjustSecondaryValue(1, nowMs);
			break;
		case InputSemanticActionKind::ContextPrevious:
			cyclePanel(-1, nowMs);
			break;
		case InputSemanticActionKind::ContextNext:
		case InputSemanticActionKind::Confirm:
			cyclePanel(1, nowMs);
			break;
		case InputSemanticActionKind::Start:
		case InputSemanticActionKind::Measure:
			case InputSemanticActionKind::Pause:
			case InputSemanticActionKind::Resume:
		case InputSemanticActionKind::Undo:
		case InputSemanticActionKind::None:
			break;
	}
}

void SplitgradeWorkflow::observeExposureState(const ExposureRuntimeState& exposureState, uint32_t nowMs) {
	if (!active_) {
		return;
	}

	observedExposurePhase_ = exposureState.phase;

	if (exposureState.phase == ExposurePhase::Fault) {
		setExecutionState(SplitgradeExecutionState::Fault, nowMs);
		return;
	}

	switch (executionState_) {
		case SplitgradeExecutionState::ArmingSoft:
			if (isExposureActivePhase(exposureState.phase)) {
				setExecutionState(SplitgradeExecutionState::ExposingSoft, nowMs);
			} else if (exposureState.phase == ExposurePhase::Done ||
			           ((nowMs - executionStateChangedAtMs_) > 200 &&
			            exposureState.phase == ExposurePhase::Idle)) {
				setExecutionState(resolveSoftPhaseCompletionState(), nowMs);
			}
			break;

		case SplitgradeExecutionState::ExposingSoft:
			if (exposureState.phase == ExposurePhase::Done || exposureState.phase == ExposurePhase::Idle) {
				setExecutionState(resolveSoftPhaseCompletionState(), nowMs);
			}
			break;

		case SplitgradeExecutionState::ArmingHard:
			if (isExposureActivePhase(exposureState.phase)) {
				setExecutionState(SplitgradeExecutionState::ExposingHard, nowMs);
			} else if (exposureState.phase == ExposurePhase::Done || exposureState.phase == ExposurePhase::Idle) {
				setExecutionState(SplitgradeExecutionState::Completed, nowMs);
			}
			break;

		case SplitgradeExecutionState::ExposingHard:
			if (exposureState.phase == ExposurePhase::Done || exposureState.phase == ExposurePhase::Idle) {
				setExecutionState(SplitgradeExecutionState::Completed, nowMs);
			}
			break;

		case SplitgradeExecutionState::Completed:
			if (exposureState.phase == ExposurePhase::Idle) {
				setExecutionState(SplitgradeExecutionState::IdleConfig, nowMs);
			}
			break;

		case SplitgradeExecutionState::Fault:
			if (exposureState.phase == ExposurePhase::Idle && !exposureState.faultLatched) {
				setExecutionState(SplitgradeExecutionState::Aborted, nowMs);
			}
			break;

		case SplitgradeExecutionState::IdleConfig:
		case SplitgradeExecutionState::WaitForFilter:
		case SplitgradeExecutionState::Aborted:
		case SplitgradeExecutionState::Inactive:
			break;
	}
}

bool SplitgradeWorkflow::consumeExecutionCommand(SplitgradeExecutionCommand& commandOut) {
	if (!pendingCommand_.isMeaningful()) {
		return false;
	}

	commandOut = pendingCommand_;
	pendingCommand_ = {};
	return true;
}

void SplitgradeWorkflow::populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const {
	state.splitgrade.panel = active_ ? panel_ : SplitgradePanel::Inactive;
	state.splitgrade.executionState = active_ ? executionState_ : SplitgradeExecutionState::Inactive;
	state.splitgrade.controlMode = controlMode_;
	state.splitgrade.softTarget = softTarget_;
	state.splitgrade.hardTarget = hardTarget_;
	state.splitgrade.grade = grade_;
	state.splitgrade.parametersDirty = parametersDirty_;
	if (!active_ || panel_ == SplitgradePanel::Inactive || panelChangedAtMs_ == 0) {
		state.splitgrade.panelAgeMs = 0;
	} else {
		state.splitgrade.panelAgeMs = nowMs - panelChangedAtMs_;
	}
	if (!active_ || executionStateChangedAtMs_ == 0 ||
	    executionState_ == SplitgradeExecutionState::Inactive) {
		state.splitgrade.executionStateAgeMs = 0;
	} else {
		state.splitgrade.executionStateAgeMs = nowMs - executionStateChangedAtMs_;
	}
	(void)enteredAtMs_;
}

void SplitgradeWorkflow::cyclePanel(int8_t direction, uint32_t nowMs) {
	if (!active_ || direction == 0) {
		return;
	}

	int nextPanel = static_cast<int>(panel_);
	if (nextPanel < static_cast<int>(SplitgradePanel::SplitTargets) ||
	    nextPanel > static_cast<int>(SplitgradePanel::Measurement)) {
		nextPanel = static_cast<int>(SplitgradePanel::SplitTargets);
	}

	nextPanel += (direction > 0) ? 1 : -1;
	if (nextPanel > static_cast<int>(SplitgradePanel::Measurement)) {
		nextPanel = static_cast<int>(SplitgradePanel::SplitTargets);
	}
	if (nextPanel < static_cast<int>(SplitgradePanel::SplitTargets)) {
		nextPanel = static_cast<int>(SplitgradePanel::Measurement);
	}

	panel_ = static_cast<SplitgradePanel>(nextPanel);
	panelChangedAtMs_ = nowMs;
}

void SplitgradeWorkflow::adjustPrimaryValue(int8_t direction, uint32_t nowMs) {
	if (direction == 0) {
		return;
	}

	switch (panel_) {
		case SplitgradePanel::SplitTargets:
			// EV-Skalierung liegt zentral in ExposureValueMath, damit weitere Modi
			// dieselbe historische Schrittsemantik nutzen koennen.
			softTarget_ = clampTargetValue(ExposureValueMath::applyStepDirection(softTarget_, direction));
			writeBackCurrentTargets(nowMs);
			parametersDirty_ = true;
			break;
		case SplitgradePanel::Grade:
			adjustGrade(direction * 0.5f);
			break;
		case SplitgradePanel::ControlMode:
			toggleControlMode();
			break;
		case SplitgradePanel::Measurement:
		case SplitgradePanel::Inactive:
			break;
	}
}

void SplitgradeWorkflow::adjustSecondaryValue(int8_t direction, uint32_t nowMs) {
	if (direction == 0) {
		return;
	}

	switch (panel_) {
		case SplitgradePanel::SplitTargets:
			hardTarget_ = clampTargetValue(ExposureValueMath::applyStepDirection(hardTarget_, direction));
			writeBackCurrentTargets(nowMs);
			parametersDirty_ = true;
			break;
		case SplitgradePanel::Grade:
			adjustGrade(direction * 0.5f);
			break;
		case SplitgradePanel::ControlMode:
			toggleControlMode();
			break;
		case SplitgradePanel::Measurement: {
			const ModeWorkflowServices* boundServices = services();
			if (boundServices != nullptr && boundServices->measurementCommands != nullptr) {
				(void)boundServices->measurementCommands->cyclePendingCaptureRole(direction);
			}
			break;
		}
		case SplitgradePanel::Inactive:
			break;
	}
}

void SplitgradeWorkflow::adjustGrade(float delta) {
	if (paperProfileLoaded_ && isFixedGradeMode(paperProfile_.gradeMode)) {
		grade_ = normalizeWorkflowGrade(clampGrade(paperProfile_.fixedGradeValue));
		return;
	}

	const float nextGrade = normalizeWorkflowGrade(clampGrade(grade_ + delta));
	if (nextGrade == grade_) {
		return;
	}

	grade_ = nextGrade;
	refreshPaperDrivenTargets();
	parametersDirty_ = true;
}

void SplitgradeWorkflow::toggleControlMode() {
	// Ein gesetzter Sensor-Failsafe darf nicht durch Bedienung wieder auf Dose
	// umgestellt werden. Die Rueckkehr zu Dose ist eine Laufzeitentscheidung des
	// Wiring-Layers, nicht des Workflows.
	if (doseControlForcedTime_) {
		controlMode_ = ExposureControlMode::Time;
		return;
	}

	controlMode_ = (controlMode_ == ExposureControlMode::Dose) ? ExposureControlMode::Time
	                                                          : ExposureControlMode::Dose;
	parametersDirty_ = true;
}

void SplitgradeWorkflow::setDoseControlForcedTime(bool forcedTime) {
	if (doseControlForcedTime_ == forcedTime) {
		return;
	}

	doseControlForcedTime_ = forcedTime;
	if (!doseControlForcedTime_) {
		return;
	}

	// Der Workflow bleibt fachlich derselbe, aber neue Starts muessen jetzt als
	// harte Zeitbelichtung laufen. Deshalb werden sowohl der sichtbare Modus als
	// auch ein eventuell schon vorbereiteter Start lokal auf TIME gezogen.
	controlMode_ = ExposureControlMode::Time;
	if (pendingCommand_.kind == SplitgradeExecutionCommandKind::StartSoft ||
	    pendingCommand_.kind == SplitgradeExecutionCommandKind::StartHard) {
		pendingCommand_.controlMode = ExposureControlMode::Time;
	}
}

void SplitgradeWorkflow::clearGradeSplitOverrides() {
	for (uint8_t gradeIndex = 0; gradeIndex < kSplitgradeStepCount; ++gradeIndex) {
		gradeSoftFractionOverrides_[gradeIndex] = 0u;
		gradeSoftFractionOverrideValid_[gradeIndex] = false;
	}
}

void SplitgradeWorkflow::initializePaperDrivenState() {
	controlMode_ = ExposureControlMode::Time;
	paperProfileLoaded_ = false;
	clearGradeSplitOverrides();
	if (const ModeWorkflowServices* boundServices = services();
	    boundServices != nullptr && boundServices->paperProfiles != nullptr) {
		if (const PaperExposureProfile* activeProfile = boundServices->paperProfiles->activeProfile();
		    activeProfile != nullptr) {
			paperProfile_ = *activeProfile;
			paperProfileLoaded_ = true;
		}
	}

	if (!paperProfileLoaded_) {
		paperProfile_ = makeDefaultPaperProfile();
		paperProfileLoaded_ = true;
	}

	// E17-Entflechtung: kBw ist ein dimensionsloser physikalischer
	// Transmissionsfaktor (10^-D_N) und darf nicht als Belichtungszeit geladen
	// werden. Der Workflow-eigene Basiszielwert startet immer mit dem
	// konsistenten Standard-Default.
	baseTarget_ = kDefaultBaseTarget;
	grade_ = isFixedGradeMode(paperProfile_.gradeMode)
	       ? normalizeWorkflowGrade(clampGrade(paperProfile_.fixedGradeValue))
	       : normalizeWorkflowGrade(kDefaultGrade);
	refreshPaperDrivenTargets();
	parametersDirty_ = false;
	paperProfilePersistPending_ = false;
	paperProfileChangedAtMs_ = 0;
}

void SplitgradeWorkflow::refreshPaperDrivenTargets() {
	if (paperProfileLoaded_ && isFixedGradeMode(paperProfile_.gradeMode)) {
		grade_ = normalizeWorkflowGrade(clampGrade(paperProfile_.fixedGradeValue));
		softTarget_ = clampTargetValue(baseTarget_);
		hardTarget_ = 0.0f;
		return;
	}

	const float softFraction = resolveSplitFraction(false);
	const float hardFraction = resolveSplitFraction(true);
	const float scaledBaseTarget = baseTarget_ * isoTargetScaleForProfile(paperProfile_);
	const float softTarget = scaledBaseTarget * softFraction;
	const float hardTarget = scaledBaseTarget * hardFraction;

	softTarget_ = (softTarget > kSplitTargetPresenceEpsilon) ? clampTargetValue(softTarget) : 0.0f;
	hardTarget_ = (hardTarget > kSplitTargetPresenceEpsilon) ? clampTargetValue(hardTarget) : 0.0f;
}

void SplitgradeWorkflow::writeBackCurrentTargets(uint32_t nowMs) {
	const float currentSoft = hasMeaningfulSoftTarget() ? softTarget_ : 0.0f;
	const float currentHard = hasMeaningfulHardTarget() ? hardTarget_ : 0.0f;
	const float totalTarget = currentSoft + currentHard;
	if (totalTarget > kSplitTargetPresenceEpsilon) {
		const float isoScale = isoTargetScaleForProfile(paperProfile_);
		baseTarget_ = clampTargetValue(totalTarget / isoScale);
	}

	if (isFixedGradeMode(paperProfile_.gradeMode) || paperProfile_.useIsoMath ||
	    totalTarget <= kSplitTargetPresenceEpsilon) {
		return;
	}

	const uint8_t gradeIndex = currentGradeIndex();
	gradeSoftFractionOverrides_[gradeIndex] = encodeSoftFractionOverride(currentSoft, totalTarget);
	gradeSoftFractionOverrideValid_[gradeIndex] = true;
	if (!paperProfileLoaded_) {
		return;
	}

	const float softFraction = decodeSoftFractionOverride(gradeSoftFractionOverrides_[gradeIndex]);
	const float hardFraction = 1.0f - softFraction;
	paperProfile_.gradeKSoft[gradeIndex] = (paperProfile_.kSoft > kSplitTargetPresenceEpsilon)
	                                     ? (paperProfile_.kSoft * softFraction)
	                                     : softFraction;
	paperProfile_.gradeKHard[gradeIndex] = (paperProfile_.kHard > kSplitTargetPresenceEpsilon)
	                                     ? (paperProfile_.kHard * hardFraction)
	                                     : hardFraction;
	schedulePaperProfilePersist(nowMs);
}

void SplitgradeWorkflow::schedulePaperProfilePersist(uint32_t nowMs) {
	if (!paperProfileLoaded_) {
		return;
	}

	if (const ModeWorkflowServices* boundServices = services();
	    boundServices == nullptr || boundServices->paperProfileCommands == nullptr) {
		return;
	}

	paperProfilePersistPending_ = true;
	paperProfileChangedAtMs_ = nowMs;
}

bool SplitgradeWorkflow::flushPendingPaperProfilePersist(uint32_t nowMs, bool force) {
	if (!paperProfilePersistPending_) {
		return true;
	}

	if (!force && (nowMs - paperProfileChangedAtMs_) < kPaperProfilePersistDebounceMs) {
		return false;
	}

	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->paperProfileCommands == nullptr) {
		return false;
	}

	if (!boundServices->paperProfileCommands->saveActiveProfile(paperProfile_)) {
		return false;
	}

	paperProfilePersistPending_ = false;
	return true;
}

bool SplitgradeWorkflow::startCurrentExecution(uint32_t nowMs) {
	if (hasMeaningfulSoftTarget()) {
		queueStartCommand(false);
		setExecutionState(SplitgradeExecutionState::ArmingSoft, nowMs);
		return true;
	}

	if (hasMeaningfulHardTarget()) {
		queueStartCommand(true);
		setExecutionState(SplitgradeExecutionState::ArmingHard, nowMs);
		return true;
	}

	return false;
}

void SplitgradeWorkflow::continueExecutionAfterFilterConfirm(uint32_t nowMs) {
	if (hasMeaningfulHardTarget()) {
		queueStartCommand(true);
		setExecutionState(SplitgradeExecutionState::ArmingHard, nowMs);
		return;
	}

	setExecutionState(SplitgradeExecutionState::Completed, nowMs);
}

SplitgradeExecutionState SplitgradeWorkflow::resolveSoftPhaseCompletionState() const {
	return hasMeaningfulHardTarget() ? SplitgradeExecutionState::WaitForFilter
	                                 : SplitgradeExecutionState::Completed;
}

void SplitgradeWorkflow::setExecutionState(SplitgradeExecutionState newState, uint32_t nowMs) {
	if (executionState_ == newState) {
		return;
	}

	executionState_ = newState;
	executionStateChangedAtMs_ = nowMs;
}

void SplitgradeWorkflow::queueCommand(SplitgradeExecutionCommandKind kind, float targetValue) {
	pendingCommand_.kind = kind;
	pendingCommand_.controlMode = controlMode_;
	pendingCommand_.targetValue = targetValue;
	pendingCommand_.valid = true;
}

void SplitgradeWorkflow::queueStartCommand(bool hardPhase) {
	queueCommand(hardPhase ? SplitgradeExecutionCommandKind::StartHard
	                     : SplitgradeExecutionCommandKind::StartSoft,
	             clampTargetValue(hardPhase ? hardTarget_ : softTarget_));
}

bool SplitgradeWorkflow::handleExecutionAction(const NormalizedInputEvent& event,
	                                           const InputSemanticAction& action,
	                                           uint32_t nowMs) {
	switch (action.kind) {
		case InputSemanticActionKind::Start:
			if (panel_ == SplitgradePanel::Measurement) {
				const ModeWorkflowServices* boundServices = services();
				if (boundServices != nullptr && boundServices->measurementCommands != nullptr) {
					(void)boundServices->measurementCommands->captureLocalSample(nowMs);
				}
				return true;
			}
			if (executionState_ == SplitgradeExecutionState::WaitForFilter) {
				continueExecutionAfterFilterConfirm(nowMs);
				return true;
			}
			if (executionState_ == SplitgradeExecutionState::ArmingSoft ||
			    executionState_ == SplitgradeExecutionState::ExposingSoft ||
			    executionState_ == SplitgradeExecutionState::ArmingHard ||
			    executionState_ == SplitgradeExecutionState::ExposingHard) {
				// Die lokale Start-Taste bleibt die blinde Paritaetsroute zum Busy-
				// Overlay: waehrend laufender Belichtung pausiert sie, im Paused-State
				// startet dieselbe Taste wieder. Der Workflow braucht dafuer die zuletzt
				// beobachtete Engine-Phase, weil sein fachlicher SG-State zwischen Run und
				// Paused absichtlich nicht doppelt unterscheidet.
				if (observedExposurePhase_ == ExposurePhase::Exposing) {
					queueCommand(SplitgradeExecutionCommandKind::PauseExposure);
					return true;
				}
				if (observedExposurePhase_ == ExposurePhase::Paused) {
					queueCommand(SplitgradeExecutionCommandKind::ResumeExposure);
					return true;
				}
			}
			if (executionState_ == SplitgradeExecutionState::IdleConfig ||
			    executionState_ == SplitgradeExecutionState::Aborted ||
			    executionState_ == SplitgradeExecutionState::Completed) {
				(void)startCurrentExecution(nowMs);
				return true;
			}
			if (executionState_ == SplitgradeExecutionState::Fault) {
				queueCommand(SplitgradeExecutionCommandKind::ClearFault);
				setExecutionState(SplitgradeExecutionState::Aborted, nowMs);
				return true;
			}
			return true;

		case InputSemanticActionKind::Measure:
			if (executionState_ == SplitgradeExecutionState::Fault) {
				queueCommand(SplitgradeExecutionCommandKind::ClearFault);
				setExecutionState(SplitgradeExecutionState::Aborted, nowMs);
				return true;
			}
			if (panel_ == SplitgradePanel::Measurement) {
				if (event.source == NormalizedInputSource::WirelessMeasureButton) {
					// Wireless-Session-Captures laufen ueber measurementSequence des C6.
					// Auf der Messseite darf das Measure-Event deshalb kein lokales
					// Duplikat erzeugen und auch keine SG-Belichtung starten.
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
			if (executionState_ == SplitgradeExecutionState::WaitForFilter) {
				continueExecutionAfterFilterConfirm(nowMs);
				return true;
			}
			if (executionState_ == SplitgradeExecutionState::Completed) {
				queueCommand(SplitgradeExecutionCommandKind::AcknowledgeDone);
				setExecutionState(SplitgradeExecutionState::IdleConfig, nowMs);
				return true;
			}
			break;

		case InputSemanticActionKind::Pause:
			if (executionState_ == SplitgradeExecutionState::ArmingSoft ||
			    executionState_ == SplitgradeExecutionState::ExposingSoft ||
			    executionState_ == SplitgradeExecutionState::ArmingHard ||
			    executionState_ == SplitgradeExecutionState::ExposingHard) {
				// Pause ist eine globale Laufzeitaktion und wird deshalb nicht ueber
				// lokale Screen-Quellnamen, sondern ueber eine eigene Semantik bis zur
				// Engine-Command-Grenze getragen.
				queueCommand(SplitgradeExecutionCommandKind::PauseExposure);
				return true;
			}
			break;

		case InputSemanticActionKind::Resume:
			if (executionState_ == SplitgradeExecutionState::ArmingSoft ||
			    executionState_ == SplitgradeExecutionState::ExposingSoft ||
			    executionState_ == SplitgradeExecutionState::ArmingHard ||
			    executionState_ == SplitgradeExecutionState::ExposingHard) {
				queueCommand(SplitgradeExecutionCommandKind::ResumeExposure);
				return true;
			}
			break;

		case InputSemanticActionKind::Undo:
			if (panel_ == SplitgradePanel::Measurement &&
			    (executionState_ == SplitgradeExecutionState::IdleConfig ||
			     executionState_ == SplitgradeExecutionState::Aborted ||
			     executionState_ == SplitgradeExecutionState::Completed)) {
				const ModeWorkflowServices* boundServices = services();
				if (boundServices != nullptr && boundServices->measurementCommands != nullptr) {
					(void)boundServices->measurementCommands->undoLastSessionSample();
				}
				return true;
			}

			switch (executionState_) {
				case SplitgradeExecutionState::ArmingSoft:
				case SplitgradeExecutionState::ExposingSoft:
				case SplitgradeExecutionState::ArmingHard:
				case SplitgradeExecutionState::ExposingHard:
					queueCommand(SplitgradeExecutionCommandKind::AbortExposure);
					setExecutionState(SplitgradeExecutionState::Aborted, nowMs);
					return true;

				case SplitgradeExecutionState::WaitForFilter:
					queueCommand(SplitgradeExecutionCommandKind::AcknowledgeDone);
					setExecutionState(SplitgradeExecutionState::Aborted, nowMs);
					return true;

				case SplitgradeExecutionState::Completed:
					queueCommand(SplitgradeExecutionCommandKind::AcknowledgeDone);
					setExecutionState(SplitgradeExecutionState::IdleConfig, nowMs);
					return true;

				case SplitgradeExecutionState::Fault:
					queueCommand(SplitgradeExecutionCommandKind::ClearFault);
					setExecutionState(SplitgradeExecutionState::Aborted, nowMs);
					return true;

				case SplitgradeExecutionState::Aborted:
					setExecutionState(SplitgradeExecutionState::IdleConfig, nowMs);
					return true;

				case SplitgradeExecutionState::IdleConfig:
				case SplitgradeExecutionState::Inactive:
					break;
			}
			break;

		case InputSemanticActionKind::None:
		case InputSemanticActionKind::AdjustPrimaryDecrease:
		case InputSemanticActionKind::AdjustPrimaryIncrease:
		case InputSemanticActionKind::AdjustSecondaryDecrease:
		case InputSemanticActionKind::AdjustSecondaryIncrease:
		case InputSemanticActionKind::ContextPrevious:
		case InputSemanticActionKind::ContextNext:
			break;
	}

	return false;
}

bool SplitgradeWorkflow::isEditingAllowed() const {
	return executionState_ == SplitgradeExecutionState::IdleConfig ||
	       executionState_ == SplitgradeExecutionState::Aborted;
}

bool SplitgradeWorkflow::hasMeaningfulSoftTarget() const {
	return softTarget_ > kSplitTargetPresenceEpsilon;
}

bool SplitgradeWorkflow::hasMeaningfulHardTarget() const {
	return hardTarget_ > kSplitTargetPresenceEpsilon;
}

uint8_t SplitgradeWorkflow::currentGradeIndex() const {
	return static_cast<uint8_t>(gradeIndexFromFloat(grade_));
}

float SplitgradeWorkflow::resolveSplitFraction(bool hardPhase) const {
	return resolveSplitFractionForGradeIndex(currentGradeIndex(), hardPhase);
}

float SplitgradeWorkflow::resolveSplitFractionForGradeIndex(uint8_t gradeIndex, bool hardPhase) const {
	if (paperProfileLoaded_ && isFixedGradeMode(paperProfile_.gradeMode)) {
		return hardPhase ? 0.0f : 1.0f;
	}

	const float softFraction = resolveProfileSoftFraction(gradeIndex);
	return hardPhase ? (1.0f - softFraction) : softFraction;
}

bool SplitgradeWorkflow::hasSplitFractionOverride(uint8_t gradeIndex) const {
	return !isFixedGradeMode(paperProfile_.gradeMode) && !paperProfile_.useIsoMath &&
	       gradeIndex < kSplitgradeStepCount && gradeSoftFractionOverrideValid_[gradeIndex];
}

float SplitgradeWorkflow::resolveProfileSoftFraction(uint8_t gradeIndex) const {
	if (hasSplitFractionOverride(gradeIndex)) {
		return decodeSoftFractionOverride(gradeSoftFractionOverrides_[gradeIndex]);
	}

	const uint8_t clampedGradeIndex = gradeIndex < kSplitgradeStepCount
	                                ? gradeIndex
	                                : static_cast<uint8_t>(kSplitgradeStepCount - 1u);
	const float clampedGrade = static_cast<float>(clampedGradeIndex) * 0.5f;
	if (paperProfileLoaded_ && paperProfile_.useIsoMath) {
		const float hardFraction = isoHardFractionForGrade(paperProfile_, clampedGrade);
		return 1.0f - hardFraction;
	}

	float softFactor = 0.0f;
	float hardFactor = 0.0f;
	if (paperProfileLoaded_) {
		softFactor = paperProfile_.gradeKSoft[clampedGradeIndex];
		hardFactor = paperProfile_.gradeKHard[clampedGradeIndex];
		if (paperProfile_.kSoft > kSplitTargetPresenceEpsilon) {
			softFactor /= paperProfile_.kSoft;
		}
		if (paperProfile_.kHard > kSplitTargetPresenceEpsilon) {
			hardFactor /= paperProfile_.kHard;
		}
	}

	if (softFactor <= kSplitTargetPresenceEpsilon && hardFactor <= kSplitTargetPresenceEpsilon) {
		hardFactor = clampedGrade / 5.0f;
		softFactor = 1.0f - hardFactor;
	}

	const float totalFactor = softFactor + hardFactor;
	if (totalFactor <= kSplitTargetPresenceEpsilon) {
		const float hardFraction = clampedGrade / 5.0f;
		return 1.0f - hardFraction;
	}

	float normalizedSoft = softFactor / totalFactor;
	if (normalizedSoft < 0.0f) {
		normalizedSoft = 0.0f;
	} else if (normalizedSoft > 1.0f) {
		normalizedSoft = 1.0f;
	}

	return normalizedSoft;
}

uint16_t SplitgradeWorkflow::encodeSoftFractionOverride(float softTarget, float totalTarget) {
	if (totalTarget <= kSplitTargetPresenceEpsilon) {
		return 0u;
	}

	float softFraction = softTarget / totalTarget;
	if (softFraction < 0.0f) {
		softFraction = 0.0f;
	} else if (softFraction > 1.0f) {
		softFraction = 1.0f;
	}

	return static_cast<uint16_t>((softFraction * static_cast<float>(kSplitFractionOverrideScale)) + 0.5f);
}

float SplitgradeWorkflow::decodeSoftFractionOverride(uint16_t encodedFraction) {
	return static_cast<float>(encodedFraction) / static_cast<float>(kSplitFractionOverrideScale);
}

float SplitgradeWorkflow::clampTargetValue(float value) {
	return (value < 0.1f) ? 0.1f : value;
}

float SplitgradeWorkflow::clampGrade(float value) {
	if (value < 0.0f) {
		return 0.0f;
	}
	if (value > 5.0f) {
		return 5.0f;
	}
	return value;
}

}  // namespace dukatimer