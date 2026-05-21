/*
 * ModeCoordinator
 *
 * Diese Datei bildet die kleinste moegliche Laufzeitverwaltung fuer Modi.
 * Sie entscheidet nicht ueber fotografische Inhalte, sondern nur darueber,
 * welcher Workflow aktiv ist, wohin Eingaben gehen und welcher Runtime-State
 * in den Snapshot geschrieben wird.
 */

#include "ModeCoordinator.h"

namespace dukatimer {

ModeCoordinator::ModeCoordinator(IModeWorkflow* const* workflows, size_t workflowCount) {
	for (size_t index = 0u; index < workflowCount && index < kMaxWorkflowCount; ++index) {
		if (workflows == nullptr || workflows[index] == nullptr) {
			continue;
		}

		workflows_[workflowCount_] = workflows[index];
		++workflowCount_;
	}

	state_.requestedMode = workflowCount_ > 0u ? workflows_[0]->modeId() : ModeId::None;
}

void ModeCoordinator::bindServices(const ModeWorkflowServices& services) {
	// Der Coordinator bleibt der eine Verdrahtungspunkt fuer Workflow-Dienste.
	// So muessen kuenftige Modi nicht einzeln aus main.cpp heraus mit globalen
	// Dienstinstanzen gekoppelt werden.
	for (size_t index = 0; index < workflowCount_; ++index) {
		if (workflows_[index] != nullptr) {
			workflows_[index]->bindServices(services);
		}
	}
}

void ModeCoordinator::begin(uint32_t nowMs, ModeId initialMode) {
	if (activeWorkflow_ != nullptr) {
		updateDerivedState(nowMs);
		return;
	}

	if (!requestMode(initialMode, nowMs)) {
		for (size_t index = 0; index < workflowCount_; ++index) {
			if (workflows_[index] != nullptr) {
				requestMode(workflows_[index]->modeId(), nowMs);
				break;
			}
		}
	}

	updateDerivedState(nowMs);
}

void ModeCoordinator::tick(uint32_t nowMs) {
	if (activeWorkflow_ != nullptr) {
		activeWorkflow_->onTick(nowMs);
	}

	updateDerivedState(nowMs);
}

bool ModeCoordinator::requestMode(ModeId modeId, uint32_t nowMs) {
	// Moduswechsel passieren immer als geordneter Exit/Enter-Uebergang. Dadurch
	// koennen Workflows lokale Ressourcen oder offene Ausfuehrungen sauber
	// freigeben, bevor der neue Modus sichtbar wird.
	IModeWorkflow* nextWorkflow = findWorkflow(modeId);
	if (nextWorkflow == nullptr) {
		return false;
	}

	state_.requestedMode = modeId;
	if (activeWorkflow_ == nextWorkflow) {
		updateDerivedState(nowMs);
		return true;
	}

	state_.transitionPending = true;
	leaveActiveWorkflow(nowMs);
	enterWorkflow(*nextWorkflow, nowMs);
	state_.transitionPending = false;
	updateDerivedState(nowMs);
	return true;
}

void ModeCoordinator::handleInputEvent(const NormalizedInputEvent& event,
	                                  const InputSemanticAction& action,
	                                  uint32_t nowMs) {
	// Der Coordinator bleibt bewusst duenn: Er reicht das bereits normierte Event
	// zusammen mit der semantischen Aktion an genau einen aktiven Workflow weiter.
	// Jegliche Moduslogik bleibt damit hinter der Workflow-Schnittstelle.
	if (!event.isMeaningful()) {
		return;
	}

	state_.lastInputEvent = event;
	lastInputReceivedMs_ = nowMs;
	if (activeWorkflow_ != nullptr) {
		activeWorkflow_->onInputEvent(event, action, nowMs);
	}

	updateDerivedState(nowMs);
}

const ModeRuntimeState& ModeCoordinator::state() const {
	return state_;
}

IModeWorkflow* ModeCoordinator::findWorkflow(ModeId modeId) const {
	for (size_t index = 0; index < workflowCount_; ++index) {
		if (workflows_[index] != nullptr && workflows_[index]->modeId() == modeId) {
			return workflows_[index];
		}
	}

	return nullptr;
}

void ModeCoordinator::leaveActiveWorkflow(uint32_t nowMs) {
	if (activeWorkflow_ == nullptr) {
		return;
	}

	activeWorkflow_->onExit(nowMs);
	activeWorkflow_ = nullptr;
	activeModeStartedMs_ = 0;
	state_.activeMode = ModeId::None;
	state_.activeModeAgeMs = 0;
}

void ModeCoordinator::enterWorkflow(IModeWorkflow& workflow, uint32_t nowMs) {
	activeWorkflow_ = &workflow;
	activeWorkflow_->onEnter(nowMs);
	activeModeStartedMs_ = nowMs;
	state_.activeMode = workflow.modeId();
	state_.requestedMode = workflow.modeId();
}

void ModeCoordinator::updateDerivedState(uint32_t nowMs) {
	// state_ ist der sichtbare Rohzustand des Modussystems. Die Struktur wird in
	// jedem Tick neu aus dem aktiven Workflow aufgebaut, damit keine veralteten
	// Felder aus frueheren Modi stehen bleiben.
	state_.splitgrade = makeInactiveSplitgradeModeRuntimeState();
	state_.paper = makeInactivePaperModeRuntimeState();
	state_.setup = makeInactiveSetupModeRuntimeState();
	state_.bw = makeInactiveBwModeRuntimeState();

	if (activeWorkflow_ == nullptr) {
		state_.activeModeAgeMs = 0;
		state_.activeMode = ModeId::None;
	} else {
		state_.activeModeAgeMs = nowMs - activeModeStartedMs_;
		state_.activeMode = activeWorkflow_->modeId();
		activeWorkflow_->populateRuntimeState(state_, nowMs);
	}

	if (!state_.lastInputEvent.isMeaningful()) {
		state_.lastInputAgeMs = 0;
	} else {
		state_.lastInputAgeMs = nowMs - lastInputReceivedMs_;
	}
}

}  // namespace dukatimer