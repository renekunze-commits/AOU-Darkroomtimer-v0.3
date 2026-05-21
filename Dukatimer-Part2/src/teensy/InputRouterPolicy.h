#pragma once

#include <stdint.h>

#include "ExposureRuntimeState.h"
#include "InputSemanticAction.h"
#include "ModeRuntimeState.h"
#include "NormalizedInputEvent.h"
#include "TouchSampler.h"

namespace dukatimer {

enum class InputFocusDomain : uint8_t {
	Encoder,
	Touch,
	Modal,
};

enum class InputModalState : uint8_t {
	None,
	WorkflowConfirm,
	WorkflowWait,
	WorkflowFault,
};

enum class InputEventGuardState : uint8_t {
	Open,
	TouchPriority,
	ConfirmLocked,
	WaitLocked,
	FaultLocked,
};

enum class InputControlOwner : uint8_t {
	None,
	Local,
	Remote,
};

struct InputRouterRuntimeState {
	InputFocusDomain focusDomain = InputFocusDomain::Encoder;
	InputModalState modalState = InputModalState::None;
	InputEventGuardState eventGuardState = InputEventGuardState::Open;
	InputControlOwner controlOwner = InputControlOwner::None;
	uint32_t focusAgeMs = 0;
	uint32_t modalAgeMs = 0;
	uint32_t eventGuardAgeMs = 0;
	uint32_t controlOwnerAgeMs = 0;
	uint32_t lastTouchSeenMs = 0;
	bool touchActive = false;
};

/*
 * InputRouterPolicy
 *
 * Zweck:
 * - entscheidet vor dem Workflow-Dispatch, ob ein normiertes Event derzeit
 *   ueberhaupt zugelassen ist
 * - verwaltet den Fokus zwischen Touch- und Encoder-Domaene
 * - bildet einfache modale Sperren fuer Confirm-/Fault-Situationen ab
 *
 * Architekturgrenze:
 * - veraendert keine Workflowdaten und mappt keine Semantik
 * - beantwortet nur die Frage: "Darf dieses Event jetzt durch?"
 */
class InputRouterPolicy {
public:
	void begin(uint32_t nowMs);
	void observeTouchState(const TouchState& touchState, uint32_t nowMs);
	void observeRuntimeState(const ModeRuntimeState& modeState,
	                        const ExposureRuntimeState& exposureState,
	                        uint32_t nowMs);
	bool shouldDispatch(const NormalizedInputEvent& event,
	                  const InputSemanticAction& action) const;
	bool shouldDeferRemoteEvent(const NormalizedInputEvent& event) const;
	void observeDispatchedEvent(const NormalizedInputEvent& event, uint32_t nowMs);

	const InputRouterRuntimeState& state() const;

private:
	static constexpr uint32_t kTouchFocusHoldMs = 900;
	static constexpr uint32_t kLocalControlOwnershipHoldMs = 350;

	InputRouterRuntimeState state_ = {};
	uint32_t focusChangedAtMs_ = 0;
	uint32_t modalChangedAtMs_ = 0;
	uint32_t eventGuardChangedAtMs_ = 0;
	uint32_t controlOwnerChangedAtMs_ = 0;

	static bool isEncoderLikeSource(NormalizedInputSource source);
	static bool isRotateEvent(NormalizedInputEventKind eventKind);
	static bool isAdjustmentOrNavigationAction(InputSemanticActionKind actionKind);
	static InputControlOwner controlOwnerForSource(NormalizedInputSource source);
	static InputModalState resolveModalState(const ModeRuntimeState& modeState,
	                                        const ExposureRuntimeState& exposureState);
	static InputEventGuardState resolveEventGuardState(InputFocusDomain focusDomain,
	                                                  InputModalState modalState);
	void setModalState(InputModalState modalState, uint32_t nowMs);
	bool modalStateBlocksEvent(const NormalizedInputEvent& event,
	                         const InputSemanticAction& action) const;
	void setFocusDomain(InputFocusDomain focusDomain, uint32_t nowMs);
	void setControlOwner(InputControlOwner controlOwner, uint32_t nowMs);
	void refreshEventGuardState(uint32_t nowMs);
	void updateAges(uint32_t nowMs);
};

}  // namespace dukatimer
