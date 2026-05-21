#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../../src/teensy/ModeWorkflowServices.h"
#include "../../src/teensy/SplitgradeWorkflow.h"

namespace {

using dukatimer::InputSemanticAction;
using dukatimer::InputSemanticActionKind;
using dukatimer::ModeId;
using dukatimer::ModeRuntimeState;
using dukatimer::NormalizedInputEvent;
using dukatimer::NormalizedInputEventKind;
using dukatimer::NormalizedInputSource;
using dukatimer::PaperExposureProfile;
using dukatimer::PaperProfileQueryPort;
using dukatimer::SplitgradeExecutionCommand;
using dukatimer::SplitgradeExecutionCommandKind;
using dukatimer::SplitgradeExecutionState;
using dukatimer::SplitgradeModeRuntimeState;
using dukatimer::SplitgradePanel;
using dukatimer::SplitgradeWorkflow;

constexpr float kFloatTolerance = 0.02f;
constexpr float kDriftTolerance = 0.005f;

[[noreturn]] void fail(const char* message) {
	std::fprintf(stderr, "FAIL: %s\n", message);
	std::exit(1);
}

void expectTrue(bool condition, const char* message) {
	if (!condition) {
		fail(message);
	}
}

void expectNear(float actual, float expected, float tolerance, const char* message) {
	if (std::fabs(actual - expected) > tolerance) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=%.4f expected=%.4f tolerance=%.4f)\n",
		             message,
		             actual,
		             expected,
		             tolerance);
		std::exit(1);
	}
}

struct FakePaperProfiles final : PaperProfileQueryPort {
	PaperExposureProfile active = {};

	uint8_t slotCount() const override {
		return 1u;
	}

	uint8_t activeSlotIndex() const override {
		return 0u;
	}

	const PaperExposureProfile* activeProfile() const override {
		return &active;
	}

	const PaperExposureProfile* profileAt(uint8_t slotIndex) const override {
		return slotIndex == 0u ? &active : nullptr;
	}

	bool hasCalibratedActiveProfile() const override {
		return active.calibrated;
	}

	uint8_t storageErrorCode() const override {
		return 0u;
	}

	uint32_t storageErrorDetail() const override {
		return 0u;
	}
};

// E17: baseTarget-Parameter entfernt. kBw ist ein dimensionsloser
// Transmissionsfaktor und darf nicht als Zeitanker im Profil stehen.
// Der SG-Workflow startet immer mit kDefaultBaseTarget.
PaperExposureProfile makeLinearProfile(float softK = 12.0f,
	                                   float hardK = 12.0f) {
	PaperExposureProfile profile = {};
	profile.schemaVersion = dukatimer::kPaperExposureProfileSchemaVersion;
	profile.calibrated = true;
	profile.gradeMode = dukatimer::PaperGradeMode::Multigrade;
	profile.useIsoMath = false;
	profile.fixedGradeValue = 2.5f;
	profile.isoP = 100.0f;
	profile.isoR = 100.0f;
	profile.kBw = 0.0f;  // E17: kein Zeitanker; 0.0f = unkalibriert
	profile.kSoft = softK;
	profile.kHard = hardK;
	for (uint8_t gradeIndex = 0; gradeIndex < dukatimer::kSplitgradeStepCount; ++gradeIndex) {
		const float hardFraction = static_cast<float>(gradeIndex) / 10.0f;
		profile.gradeKSoft[gradeIndex] = softK * (1.0f - hardFraction);
		profile.gradeKHard[gradeIndex] = hardK * hardFraction;
	}
	return profile;
}

PaperExposureProfile makeFixedGradeProfile(float fixedGrade = 3.0f) {
	PaperExposureProfile profile = makeLinearProfile();
	profile.gradeMode = dukatimer::PaperGradeMode::FixedGrade;
	profile.fixedGradeValue = fixedGrade;
	return profile;
}

PaperExposureProfile makeIsoProfile(float isoP = 100.0f, float isoR = 100.0f) {
	PaperExposureProfile profile = makeLinearProfile();
	profile.useIsoMath = true;
	profile.isoP = isoP;
	profile.isoR = isoR;
	return profile;
}

NormalizedInputEvent makeEvent(NormalizedInputEventKind eventKind, uint32_t nowMs, uint32_t sequence) {
	return dukatimer::makeNormalizedInputEvent(NormalizedInputSource::LocalEncoder1,
	                                         eventKind,
	                                         0,
	                                         nowMs,
	                                         sequence);
}

InputSemanticAction makeAction(InputSemanticActionKind kind, uint32_t nowMs, uint32_t sequence) {
	return dukatimer::makeInputSemanticAction(kind, 0, nowMs, sequence);
}

void dispatch(SplitgradeWorkflow& workflow,
	          InputSemanticActionKind kind,
	          uint32_t nowMs,
	          uint32_t sequence = 1u,
	          NormalizedInputEventKind eventKind = NormalizedInputEventKind::Press) {
	workflow.onInputEvent(makeEvent(eventKind, nowMs, sequence), makeAction(kind, nowMs, sequence), nowMs);
}

SplitgradeModeRuntimeState stateAt(SplitgradeWorkflow& workflow, uint32_t nowMs) {
	ModeRuntimeState state;
	state.activeMode = ModeId::Splitgrade;
	workflow.populateRuntimeState(state, nowMs);
	return state.splitgrade;
}

SplitgradeExecutionCommand consumeExpectedCommand(SplitgradeWorkflow& workflow,
	                                              SplitgradeExecutionCommandKind expectedKind,
	                                              const char* message) {
	SplitgradeExecutionCommand command;
	expectTrue(workflow.consumeExecutionCommand(command), message);
	expectTrue(command.kind == expectedKind, message);
	return command;
}

void switchToGradePanel(SplitgradeWorkflow& workflow, uint32_t nowMs) {
	dispatch(workflow, InputSemanticActionKind::Confirm, nowMs);
	expectTrue(stateAt(workflow, nowMs).panel == SplitgradePanel::Grade,
	          "SG panel should switch to Grade");
}

void driveGradeTo(SplitgradeWorkflow& workflow, float targetGrade, uint32_t& nowMs, uint32_t& sequence) {
	SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	while (state.grade > targetGrade + 0.01f) {
		dispatch(workflow,
		       InputSemanticActionKind::AdjustPrimaryDecrease,
		       ++nowMs,
		       ++sequence,
		       NormalizedInputEventKind::RotateLeft);
		state = stateAt(workflow, nowMs);
	}
	while (state.grade < targetGrade - 0.01f) {
		dispatch(workflow,
		       InputSemanticActionKind::AdjustPrimaryIncrease,
		       ++nowMs,
		       ++sequence,
		       NormalizedInputEventKind::RotateRight);
		state = stateAt(workflow, nowMs);
	}
}

dukatimer::ExposureRuntimeState makeIdleExposureState() {
	return dukatimer::makeIdleExposureRuntimeState();
}

void runGradeZeroCase() {
	FakePaperProfiles profiles;
	profiles.active = makeLinearProfile();

	dukatimer::ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	SplitgradeWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs = 1000u;
	uint32_t sequence = 1u;
	workflow.onEnter(nowMs);
	switchToGradePanel(workflow, ++nowMs);
	driveGradeTo(workflow, 0.0f, nowMs, sequence);

	const SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	expectNear(state.grade, 0.0f, kFloatTolerance, "Grade-0 case should reach grade 0");
	expectNear(state.softTarget, 10.0f, kFloatTolerance, "Grade-0 case should become soft-only");
	expectNear(state.hardTarget, 0.0f, kFloatTolerance, "Grade-0 case should clear hard target");

	dispatch(workflow, InputSemanticActionKind::Start, ++nowMs, ++sequence);
	const SplitgradeExecutionCommand command = consumeExpectedCommand(
		workflow,
		SplitgradeExecutionCommandKind::StartSoft,
		"Grade-0 case should start in soft phase");
	expectNear(command.targetValue, 10.0f, kFloatTolerance, "Grade-0 soft target should stay at base");
	workflow.observeExposureState(makeIdleExposureState(), nowMs + 250u);
	expectTrue(stateAt(workflow, nowMs + 250u).executionState == SplitgradeExecutionState::Completed,
	          "Grade-0 case should complete without WaitForFilter");
	dispatch(workflow, InputSemanticActionKind::Confirm, nowMs + 251u, ++sequence);
	consumeExpectedCommand(workflow,
	                     SplitgradeExecutionCommandKind::AcknowledgeDone,
	                     "Grade-0 case should finish via Done acknowledgement, not a hidden hard phase");
	expectTrue(stateAt(workflow, nowMs + 251u).executionState == SplitgradeExecutionState::IdleConfig,
	          "Grade-0 case should return to idle after acknowledge");
}

void runGradeFiveCase() {
	FakePaperProfiles profiles;
	profiles.active = makeLinearProfile();

	dukatimer::ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	SplitgradeWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs = 2000u;
	uint32_t sequence = 1u;
	workflow.onEnter(nowMs);
	switchToGradePanel(workflow, ++nowMs);
	driveGradeTo(workflow, 5.0f, nowMs, sequence);

	const SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	expectNear(state.grade, 5.0f, kFloatTolerance, "Grade-5 case should reach grade 5");
	expectNear(state.softTarget, 0.0f, kFloatTolerance, "Grade-5 case should clear soft target");
	expectNear(state.hardTarget, 10.0f, kFloatTolerance, "Grade-5 case should become hard-only");

	dispatch(workflow, InputSemanticActionKind::Start, ++nowMs, ++sequence);
	const SplitgradeExecutionCommand command = consumeExpectedCommand(
		workflow,
		SplitgradeExecutionCommandKind::StartHard,
		"Grade-5 case should start directly in hard phase");
	expectNear(command.targetValue, 10.0f, kFloatTolerance, "Grade-5 hard target should stay at base");
	workflow.observeExposureState(makeIdleExposureState(), nowMs + 250u);
	expectTrue(stateAt(workflow, nowMs + 250u).executionState == SplitgradeExecutionState::Completed,
	          "Grade-5 case should complete from hard phase");
	dispatch(workflow, InputSemanticActionKind::Confirm, nowMs + 251u, ++sequence);
	consumeExpectedCommand(workflow,
	                     SplitgradeExecutionCommandKind::AcknowledgeDone,
	                     "Grade-5 case should finish via Done acknowledgement");
	expectTrue(stateAt(workflow, nowMs + 251u).executionState == SplitgradeExecutionState::IdleConfig,
	          "Grade-5 case should return to idle after acknowledge");
}

void runFixedGradeCase() {
	FakePaperProfiles profiles;
	profiles.active = makeFixedGradeProfile();

	dukatimer::ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	SplitgradeWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs = 3000u;
	uint32_t sequence = 1u;
	workflow.onEnter(nowMs);
	SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	expectNear(state.grade, 3.0f, kFloatTolerance, "Fixed-grade case should adopt profile grade");
	expectNear(state.softTarget, 10.0f, kFloatTolerance, "Fixed-grade case should start at kDefaultBaseTarget");
	expectNear(state.hardTarget, 0.0f, kFloatTolerance, "Fixed-grade case should clear hard target");

	switchToGradePanel(workflow, ++nowMs);
	dispatch(workflow,
	       InputSemanticActionKind::AdjustPrimaryIncrease,
	       ++nowMs,
	       ++sequence,
	       NormalizedInputEventKind::RotateRight);
	state = stateAt(workflow, nowMs);
	expectNear(state.grade, 3.0f, kFloatTolerance, "Fixed-grade case should ignore grade edits");
	expectNear(state.softTarget, 10.0f, kFloatTolerance, "Fixed-grade case should keep kDefaultBaseTarget after ignored grade edit");

	dispatch(workflow, InputSemanticActionKind::Start, ++nowMs, ++sequence);
	consumeExpectedCommand(workflow,
	                     SplitgradeExecutionCommandKind::StartSoft,
	                     "Fixed-grade case should still start soft");
	workflow.observeExposureState(makeIdleExposureState(), nowMs + 250u);
	expectTrue(stateAt(workflow, nowMs + 250u).executionState == SplitgradeExecutionState::Completed,
	          "Fixed-grade case should complete without WaitForFilter");
}

void runManualEditThenGradeChangeCase() {
	FakePaperProfiles profiles;
	profiles.active = makeLinearProfile();

	dukatimer::ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	SplitgradeWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs = 4000u;
	uint32_t sequence = 1u;
	workflow.onEnter(nowMs);

	SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	expectNear(state.softTarget, 5.0f, kFloatTolerance, "Manual-edit case should start with balanced soft target");
	expectNear(state.hardTarget, 5.0f, kFloatTolerance, "Manual-edit case should start with balanced hard target");

	dispatch(workflow,
	       InputSemanticActionKind::AdjustPrimaryIncrease,
	       ++nowMs,
	       ++sequence,
	       NormalizedInputEventKind::RotateRight);
	state = stateAt(workflow, nowMs);
	const float editedSoftTarget = state.softTarget;
	const float editedHardTarget = state.hardTarget;
	const float editedBaseTarget = editedSoftTarget + editedHardTarget;
	expectTrue(editedSoftTarget > 5.0f, "Manual-edit case should increase soft target via EV math");

	switchToGradePanel(workflow, ++nowMs);
	driveGradeTo(workflow, 0.0f, nowMs, sequence);
	state = stateAt(workflow, nowMs);
	expectNear(state.softTarget,
	          editedBaseTarget,
	          kFloatTolerance,
	          "Manual-edit case should carry edited base target into grade 0");
	expectNear(state.hardTarget, 0.0f, kFloatTolerance, "Manual-edit case should still clear hard at grade 0");

	driveGradeTo(workflow, 2.5f, nowMs, sequence);
	state = stateAt(workflow, nowMs);
	expectNear(state.softTarget,
	          editedSoftTarget,
	          kFloatTolerance,
	          "Manual-edit case should preserve edited soft ratio when returning to the original grade");
	expectNear(state.hardTarget,
	          editedHardTarget,
	          kFloatTolerance,
	          "Manual-edit case should preserve edited hard ratio when returning to the original grade");

	for (uint8_t roundTrip = 0u; roundTrip < 8u; ++roundTrip) {
		driveGradeTo(workflow, 0.0f, nowMs, sequence);
		state = stateAt(workflow, nowMs);
		expectNear(state.softTarget,
		          editedBaseTarget,
		          kDriftTolerance,
		          "Manual-edit case should not drift while round-tripping to grade 0");
		expectNear(state.hardTarget,
		          0.0f,
		          kDriftTolerance,
		          "Manual-edit case should keep the hard phase empty at grade 0 across round-trips");

		driveGradeTo(workflow, 2.5f, nowMs, sequence);
		state = stateAt(workflow, nowMs);
		expectNear(state.softTarget,
		          editedSoftTarget,
		          kDriftTolerance,
		          "Manual-edit case should keep the edited soft target stable across grade round-trips");
		expectNear(state.hardTarget,
		          editedHardTarget,
		          kDriftTolerance,
		          "Manual-edit case should keep the edited hard target stable across grade round-trips");
	}
}

void runHalfGradeLutCase() {
	FakePaperProfiles profiles;
	profiles.active = makeLinearProfile(10.0f, 10.0f);
	profiles.active.gradeKSoft[5] = 9.0f;
	profiles.active.gradeKHard[5] = 1.0f;
	profiles.active.gradeKSoft[6] = 1.0f;
	profiles.active.gradeKHard[6] = 9.0f;

	dukatimer::ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	SplitgradeWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs = 5000u;
	uint32_t sequence = 1u;
	workflow.onEnter(nowMs);

	SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	expectNear(state.grade, 2.5f, kFloatTolerance, "Half-grade LUT case should start at grade 2.5");
	expectNear(state.softTarget, 9.0f, kFloatTolerance, "Half-grade LUT case should use index 5 soft value");
	expectNear(state.hardTarget, 1.0f, kFloatTolerance, "Half-grade LUT case should use index 5 hard value");

	switchToGradePanel(workflow, ++nowMs);
	dispatch(workflow,
	       InputSemanticActionKind::AdjustPrimaryIncrease,
	       ++nowMs,
	       ++sequence,
	       NormalizedInputEventKind::RotateRight);
	state = stateAt(workflow, nowMs);
	expectNear(state.grade, 3.0f, kFloatTolerance, "Half-grade LUT case should step to grade 3.0");
	expectNear(state.softTarget, 1.0f, kFloatTolerance, "Half-grade LUT case should use index 6 soft value");
	expectNear(state.hardTarget, 9.0f, kFloatTolerance, "Half-grade LUT case should use index 6 hard value");
}

void runIsoSpeedAndRangeCase() {
	FakePaperProfiles profiles;
	profiles.active = makeIsoProfile(200.0f, 100.0f);

	dukatimer::ModeWorkflowServices services;
	services.paperProfiles = &profiles;

	SplitgradeWorkflow workflow;
	workflow.bindServices(services);

	uint32_t nowMs = 6000u;
	workflow.onEnter(nowMs);

	SplitgradeModeRuntimeState state = stateAt(workflow, nowMs);
	expectNear(state.grade, 2.5f, kFloatTolerance, "ISO case should start at grade 2.5");
	expectNear(state.softTarget, 2.5f, kFloatTolerance, "ISO-P case should compensate soft target by paper speed");
	expectNear(state.hardTarget, 2.5f, kFloatTolerance, "ISO-P case should compensate hard target by paper speed");

	profiles.active = makeIsoProfile(100.0f, 200.0f);
	SplitgradeWorkflow softRangeWorkflow;
	softRangeWorkflow.bindServices(services);
	softRangeWorkflow.onEnter(nowMs + 100u);
	state = stateAt(softRangeWorkflow, nowMs + 100u);
	expectNear(state.softTarget, 10.0f / 3.0f, kFloatTolerance, "ISO-R case should bias soft paper toward less soft exposure");
	expectNear(state.hardTarget, 20.0f / 3.0f, kFloatTolerance, "ISO-R case should bias soft paper toward more hard exposure");

	profiles.active = makeIsoProfile(0.0f, 0.0f);
	SplitgradeWorkflow fallbackWorkflow;
	fallbackWorkflow.bindServices(services);
	fallbackWorkflow.onEnter(nowMs + 200u);
	state = stateAt(fallbackWorkflow, nowMs + 200u);
	expectNear(state.softTarget, 5.0f, kFloatTolerance, "ISO fallback case should default soft target safely");
	expectNear(state.hardTarget, 5.0f, kFloatTolerance, "ISO fallback case should default hard target safely");
}

}  // namespace

int main() {
	runGradeZeroCase();
	runGradeFiveCase();
	runFixedGradeCase();
	runManualEditThenGradeChangeCase();
	runHalfGradeLutCase();
	runIsoSpeedAndRangeCase();
	std::puts("AP-07 acceptance harness passed.");
	return 0;
}