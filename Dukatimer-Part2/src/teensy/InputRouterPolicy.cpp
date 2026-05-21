/*
 * InputRouterPolicy
 *
 * Diese Datei kapselt die kleine, aber zentrale Vor-Dispatch-Logik fuer Eingaben.
 * Sie verhindert, dass Touch-Interaktionen und Encoder-Drehungen gleichzeitig an
 * denselben Workflowpfad gelangen, und blockt Drehereignisse in modalen SG-
 * Zustanden wie Filterwechsel oder Fault.
 */

#include "InputRouterPolicy.h"

namespace dukatimer {

void InputRouterPolicy::begin(uint32_t nowMs) {
	// Der Router startet bewusst im Encoder-Fokus. Touch darf den Fokus spaeter
	// temporär uebernehmen, muss ihn aber nach Inaktivitaet wieder freigeben.
	state_ = {};
	state_.focusDomain = InputFocusDomain::Encoder;
	state_.modalState = InputModalState::None;
	state_.eventGuardState = InputEventGuardState::Open;
	state_.controlOwner = InputControlOwner::None;
	state_.focusAgeMs = 0;
	state_.modalAgeMs = 0;
	state_.eventGuardAgeMs = 0;
	state_.controlOwnerAgeMs = 0;
	state_.lastTouchSeenMs = nowMs;
	state_.touchActive = false;
	focusChangedAtMs_ = nowMs;
	modalChangedAtMs_ = nowMs;
	eventGuardChangedAtMs_ = nowMs;
	controlOwnerChangedAtMs_ = nowMs;
}

void InputRouterPolicy::observeTouchState(const TouchState& touchState, uint32_t nowMs) {
	// Touch wirkt hier nur als Fokusquelle, nicht als eigener Workflow-Dispatch.
	// Solange Touch aktiv oder gerade erst losgelassen wurde, sollen Rotations-
	// Events nicht parallel dieselbe UI ueberfahren.
	state_.touchActive = touchState.active;
	if (touchState.active) {
		state_.lastTouchSeenMs = nowMs;
		if (state_.modalState != InputModalState::None) {
			updateAges(nowMs);
			refreshEventGuardState(nowMs);
			return;
		}
		setFocusDomain(InputFocusDomain::Touch, nowMs);
		updateAges(nowMs);
		refreshEventGuardState(nowMs);
		return;
	}

	if (state_.modalState != InputModalState::None) {
		updateAges(nowMs);
		refreshEventGuardState(nowMs);
		return;
	}

	if (state_.focusDomain == InputFocusDomain::Touch &&
	    (nowMs - state_.lastTouchSeenMs) >= kTouchFocusHoldMs) {
		setFocusDomain(InputFocusDomain::Encoder, nowMs);
	}

	updateAges(nowMs);
	refreshEventGuardState(nowMs);
}

void InputRouterPolicy::observeRuntimeState(const ModeRuntimeState& modeState,
	                                        const ExposureRuntimeState& exposureState,
	                                        uint32_t nowMs) {
	// Modale Sperren werden aus dem sichtbaren Runtime-State abgeleitet, nicht aus
	// verstecktem Workflowwissen. Dadurch bleibt die Policy austauschbar und klein.
	setModalState(resolveModalState(modeState, exposureState), nowMs);
	if (state_.modalState != InputModalState::None) {
		setFocusDomain(InputFocusDomain::Modal, nowMs);
	} else if (state_.focusDomain == InputFocusDomain::Modal) {
		const bool touchHasPriority = state_.touchActive ||
		                            ((nowMs - state_.lastTouchSeenMs) < kTouchFocusHoldMs);
		setFocusDomain(touchHasPriority ? InputFocusDomain::Touch : InputFocusDomain::Encoder,
		              nowMs);
	}
	updateAges(nowMs);
	refreshEventGuardState(nowMs);
}

bool InputRouterPolicy::shouldDispatch(const NormalizedInputEvent& event,
	                                   const InputSemanticAction& action) const {
	// shouldDispatch() ist die letzte zentrale Schranke vor dem Workflow. Alles,
	// was hier verworfen wird, soll gar nicht erst in Moduslogik oder UI landen.
	if (!event.isMeaningful() || !action.isMeaningful()) {
		return false;
	}

	if (modalStateBlocksEvent(event, action)) {
		return false;
	}

	if (state_.focusDomain == InputFocusDomain::Touch &&
	    isEncoderLikeSource(event.source) &&
	    isRotateEvent(event.eventKind)) {
		return false;
	}

	if (shouldDeferRemoteEvent(event)) {
		return false;
	}

	return true;
}

bool InputRouterPolicy::shouldDeferRemoteEvent(const NormalizedInputEvent& event) const {
	if (!event.isMeaningful()) {
		return false;
	}

	return controlOwnerForSource(event.source) == InputControlOwner::Remote &&
	       state_.controlOwner == InputControlOwner::Local &&
	       state_.controlOwnerAgeMs < kLocalControlOwnershipHoldMs;
}

void InputRouterPolicy::observeDispatchedEvent(const NormalizedInputEvent& event, uint32_t nowMs) {
	if (!event.isMeaningful()) {
		return;
	}

	if (state_.modalState == InputModalState::None &&
	    (isEncoderLikeSource(event.source) ||
	     event.source == NormalizedInputSource::LocalStartButton)) {
		setFocusDomain(InputFocusDomain::Encoder, nowMs);
	}

	const InputControlOwner controlOwner = controlOwnerForSource(event.source);
	if (controlOwner != InputControlOwner::None) {
		setControlOwner(controlOwner, nowMs);
	}

	updateAges(nowMs);
	refreshEventGuardState(nowMs);
}

const InputRouterRuntimeState& InputRouterPolicy::state() const {
	return state_;
}

bool InputRouterPolicy::isEncoderLikeSource(NormalizedInputSource source) {
	return source == NormalizedInputSource::LocalEncoder1 ||
	       source == NormalizedInputSource::LocalEncoder2 ||
	       source == NormalizedInputSource::LocalEncoder3 ||
	       source == NormalizedInputSource::RemoteEncoder4 ||
	       source == NormalizedInputSource::WirelessEncoder ||
	       source == NormalizedInputSource::WirelessEncoderButton;
}

bool InputRouterPolicy::isRotateEvent(NormalizedInputEventKind eventKind) {
	return eventKind == NormalizedInputEventKind::RotateLeft ||
	       eventKind == NormalizedInputEventKind::RotateRight;
}

InputControlOwner InputRouterPolicy::controlOwnerForSource(NormalizedInputSource source) {
	switch (source) {
		case NormalizedInputSource::LocalEncoder1:
		case NormalizedInputSource::LocalEncoder2:
		case NormalizedInputSource::LocalEncoder3:
		case NormalizedInputSource::LocalStartButton:
		case NormalizedInputSource::LocalModalButton:
			return InputControlOwner::Local;

		case NormalizedInputSource::RemoteEncoder4:
		case NormalizedInputSource::WirelessEncoder:
		case NormalizedInputSource::WirelessMeasureButton:
		case NormalizedInputSource::WirelessBackButton:
		case NormalizedInputSource::WirelessEncoderButton:
			return InputControlOwner::Remote;

		case NormalizedInputSource::None:
			break;
	}

	return InputControlOwner::None;
}

bool InputRouterPolicy::isAdjustmentOrNavigationAction(InputSemanticActionKind actionKind) {
	return actionKind == InputSemanticActionKind::AdjustPrimaryDecrease ||
	       actionKind == InputSemanticActionKind::AdjustPrimaryIncrease ||
	       actionKind == InputSemanticActionKind::AdjustSecondaryDecrease ||
	       actionKind == InputSemanticActionKind::AdjustSecondaryIncrease ||
	       actionKind == InputSemanticActionKind::ContextPrevious ||
	       actionKind == InputSemanticActionKind::ContextNext;
}

InputModalState InputRouterPolicy::resolveModalState(const ModeRuntimeState& modeState,
	                                                  const ExposureRuntimeState& exposureState) {
	// AP-06-Modallogik in Prioritaetsreihenfolge:
	// - Fault: globales Fehlermodal
	// - WaitForFilter: bestaetigendes Modal
	// - aktive Belichtungsphase: globales Wait-Modal
	if (exposureState.faultReason != ExposureFaultReason::None ||
	    exposureState.faultLatched ||
	    modeState.splitgrade.executionState == SplitgradeExecutionState::Fault) {
		return InputModalState::WorkflowFault;
	}

	if (modeState.activeMode == ModeId::Splitgrade &&
	    modeState.splitgrade.executionState == SplitgradeExecutionState::WaitForFilter) {
		return InputModalState::WorkflowConfirm;
	}

	if (exposureState.phase == ExposurePhase::PreWait ||
	    exposureState.phase == ExposurePhase::Exposing ||
	    exposureState.phase == ExposurePhase::Paused ||
	    exposureState.phase == ExposurePhase::PostWait) {
		return InputModalState::WorkflowWait;
	}

	return InputModalState::None;
}

InputEventGuardState InputRouterPolicy::resolveEventGuardState(InputFocusDomain focusDomain,
	                                                            InputModalState modalState) {
	if (modalState == InputModalState::WorkflowFault) {
		return InputEventGuardState::FaultLocked;
	}

	if (modalState == InputModalState::WorkflowWait) {
		return InputEventGuardState::WaitLocked;
	}

	if (modalState == InputModalState::WorkflowConfirm) {
		return InputEventGuardState::ConfirmLocked;
	}

	if (focusDomain == InputFocusDomain::Touch) {
		return InputEventGuardState::TouchPriority;
	}

	return InputEventGuardState::Open;
}

bool InputRouterPolicy::modalStateBlocksEvent(const NormalizedInputEvent& event,
	                                          const InputSemanticAction& action) const {
	if (state_.modalState == InputModalState::None) {
		return false;
	}

	if (state_.modalState == InputModalState::WorkflowWait) {
		// Im globalen Wait-Modal bleiben nur die expliziten Laufzeitaktionen offen:
		// Sicherheitsausstieg, Pause und Resume. Zusaetzlich darf der lokale Start-
		// Taster denselben Laufzeitpfad nutzen, damit Busy- und Blindbedienung dieselbe
		// Pause/Resume-Semantik teilen, ohne den Guard fuer Navigation aufzuweichen.
		const bool waitModalActionAllowed =
			action.kind == InputSemanticActionKind::Undo ||
			action.kind == InputSemanticActionKind::Pause ||
			action.kind == InputSemanticActionKind::Resume ||
			(action.kind == InputSemanticActionKind::Start &&
			 event.source == NormalizedInputSource::LocalStartButton);
		return !waitModalActionAllowed;
	}

	if (state_.modalState == InputModalState::WorkflowConfirm ||
	    state_.modalState == InputModalState::WorkflowFault) {
		// Confirm-/Fault-Modal: Parameternavigation ist gesperrt, bestaetigende
		// und quittierende Aktionen bleiben zulaessig.
		if (isAdjustmentOrNavigationAction(action.kind)) {
			return true;
		}

		return false;
	}

	return isRotateEvent(event.eventKind);
}

void InputRouterPolicy::setFocusDomain(InputFocusDomain focusDomain, uint32_t nowMs) {
	if (state_.focusDomain == focusDomain) {
		return;
	}

	state_.focusDomain = focusDomain;
	focusChangedAtMs_ = nowMs;
	state_.focusAgeMs = 0;
}

void InputRouterPolicy::setControlOwner(InputControlOwner controlOwner, uint32_t nowMs) {
	if (state_.controlOwner == controlOwner) {
		return;
	}

	state_.controlOwner = controlOwner;
	controlOwnerChangedAtMs_ = nowMs;
	state_.controlOwnerAgeMs = 0;
}

void InputRouterPolicy::setModalState(InputModalState modalState, uint32_t nowMs) {
	if (state_.modalState == modalState) {
		return;
	}

	state_.modalState = modalState;
	modalChangedAtMs_ = nowMs;
	state_.modalAgeMs = 0;
}


void InputRouterPolicy::refreshEventGuardState(uint32_t nowMs) {
	const InputEventGuardState nextGuardState =
		resolveEventGuardState(state_.focusDomain, state_.modalState);
	if (state_.eventGuardState != nextGuardState) {
		state_.eventGuardState = nextGuardState;
		eventGuardChangedAtMs_ = nowMs;
		state_.eventGuardAgeMs = 0;
		return;
	}

	state_.eventGuardAgeMs = nowMs - eventGuardChangedAtMs_;
}

void InputRouterPolicy::updateAges(uint32_t nowMs) {
	state_.focusAgeMs = nowMs - focusChangedAtMs_;
	state_.modalAgeMs = nowMs - modalChangedAtMs_;
	state_.eventGuardAgeMs = nowMs - eventGuardChangedAtMs_;
	state_.controlOwnerAgeMs = nowMs - controlOwnerChangedAtMs_;
}

}  // namespace dukatimer
