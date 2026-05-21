#pragma once

#include <stdint.h>

#include "ExposureRuntimeState.h"
#include "IModeWorkflow.h"
#include "PaperExposureProfile.h"

namespace dukatimer {

enum class SplitgradeExecutionCommandKind : uint8_t {
	None,
	StartSoft,
	StartHard,
	PauseExposure,
	ResumeExposure,
	AbortExposure,
	AcknowledgeDone,
	ClearFault,
};

struct SplitgradeExecutionCommand {
	SplitgradeExecutionCommandKind kind = SplitgradeExecutionCommandKind::None;
	ExposureControlMode controlMode = ExposureControlMode::None;
	float targetValue = 0.0f;
	bool valid = false;

	bool isMeaningful() const {
		return valid && kind != SplitgradeExecutionCommandKind::None;
	}
};

class SplitgradeWorkflow : public IModeWorkflow {
public:
	ModeId modeId() const override;
	const char* shortName() const override;
	void onEnter(uint32_t nowMs) override;
	void onExit(uint32_t nowMs) override;
	void onTick(uint32_t nowMs) override;
	void onInputEvent(const NormalizedInputEvent& event,
	               const InputSemanticAction& action,
	               uint32_t nowMs) override;
	void populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const override;
	void observeExposureState(const ExposureRuntimeState& exposureState, uint32_t nowMs);
	bool consumeExecutionCommand(SplitgradeExecutionCommand& commandOut);
	void setDoseControlForcedTime(bool forcedTime);

private:
	static constexpr float kDefaultSoftTarget = 10.0f;
	static constexpr float kDefaultHardTarget = 10.0f;
	static constexpr float kDefaultBaseTarget = 10.0f;
	static constexpr float kDefaultGrade = 2.5f;
	static constexpr uint16_t kSplitFractionOverrideScale = 65535u;
	static constexpr uint32_t kPaperProfilePersistDebounceMs = 800u;

	bool active_ = false;
	bool initialized_ = false;
// SplitgradeExecutionCommand ist die schmale Bruecke zwischen fachlichem
// SG-Workflow und physischer ExposureEngine. Der Workflow beschreibt nur, was
// passieren soll; main.cpp entscheidet, wie das als lokaler Exposure-Start,
// Abort oder Fault-Reset konkret ausgefuehrt wird.
	SplitgradePanel panel_ = SplitgradePanel::Inactive;
	ExposureControlMode controlMode_ = ExposureControlMode::Time;
	bool doseControlForcedTime_ = false;
	float baseTarget_ = kDefaultBaseTarget;
	float softTarget_ = kDefaultSoftTarget;
	float hardTarget_ = kDefaultHardTarget;
	float grade_ = kDefaultGrade;
	PaperExposureProfile paperProfile_ = {};
	bool paperProfileLoaded_ = false;
	uint16_t gradeSoftFractionOverrides_[kSplitgradeStepCount] = {};
	bool gradeSoftFractionOverrideValid_[kSplitgradeStepCount] = {};
	bool parametersDirty_ = false;
	bool paperProfilePersistPending_ = false;
	uint32_t enteredAtMs_ = 0;
	uint32_t panelChangedAtMs_ = 0;
	uint32_t paperProfileChangedAtMs_ = 0;
	SplitgradeExecutionState executionState_ = SplitgradeExecutionState::Inactive;
	ExposurePhase observedExposurePhase_ = ExposurePhase::Idle;
	uint32_t executionStateChangedAtMs_ = 0;
	SplitgradeExecutionCommand pendingCommand_ = {};

/*
 * SplitgradeWorkflow
 *
 * Zweck:
 * - bildet den lokalen SG-Bedien- und Ausfuehrungsablauf oberhalb der
 *   physischen ExposureEngine ab
 * - besitzt SG-spezifische Parameter wie Targets, Grade, Panel-Fokus und
 *   Ausfuehrungszustand
 * - kennt keine Hardware, keine direkte Sensorik und keine UI-Strings
 *
 * Architekturgrenze:
 * - der Workflow erzeugt nur fachliche Kommandos und Runtime-State
 * - main.cpp und die Laufzeitdienste setzen diese Kommandos spaeter um
 */
	void cyclePanel(int8_t direction, uint32_t nowMs);
	void adjustPrimaryValue(int8_t direction, uint32_t nowMs);
	void adjustSecondaryValue(int8_t direction, uint32_t nowMs);
	bool shouldReloadPaperDrivenState() const;
	void adjustGrade(float delta);
	void toggleControlMode();
	void clearGradeSplitOverrides();
	void initializePaperDrivenState();
	void refreshPaperDrivenTargets();
	void writeBackCurrentTargets(uint32_t nowMs);
	void schedulePaperProfilePersist(uint32_t nowMs);
	bool flushPendingPaperProfilePersist(uint32_t nowMs, bool force);
	bool startCurrentExecution(uint32_t nowMs);
	void continueExecutionAfterFilterConfirm(uint32_t nowMs);
	SplitgradeExecutionState resolveSoftPhaseCompletionState() const;
	void setExecutionState(SplitgradeExecutionState newState, uint32_t nowMs);
	void queueCommand(SplitgradeExecutionCommandKind kind, float targetValue = 0.0f);
	void queueStartCommand(bool hardPhase);
	bool handleExecutionAction(const NormalizedInputEvent& event,
	                         const InputSemanticAction& action,
	                         uint32_t nowMs);
	bool isEditingAllowed() const;
	bool hasMeaningfulSoftTarget() const;
	bool hasMeaningfulHardTarget() const;
	uint8_t currentGradeIndex() const;
	float resolveSplitFraction(bool hardPhase) const;
	float resolveSplitFractionForGradeIndex(uint8_t gradeIndex, bool hardPhase) const;
	bool hasSplitFractionOverride(uint8_t gradeIndex) const;
	float resolveProfileSoftFraction(uint8_t gradeIndex) const;
	static uint16_t encodeSoftFractionOverride(float softTarget, float totalTarget);
	static float decodeSoftFractionOverride(uint16_t encodedFraction);

	static float clampTargetValue(float value);
	static float clampGrade(float value);
};

}  // namespace dukatimer