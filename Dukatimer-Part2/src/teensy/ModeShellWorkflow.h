#pragma once

#include <stdint.h>

#include "IModeWorkflow.h"

namespace dukatimer {

/*
 * ModeShellWorkflow
 *
 * Zweck:
 * - stellt einen bewusst fachleeren Workflow fuer Modi bereit, die in Part2
 *   noch keinen eigenen Bedien- und Laufzeitkern besitzen
 * - erlaubt dem ModeCoordinator bereits jetzt saubere Moduswechsel und einen
 *   konsistenten Platzhalter im Runtime-Modell
 */
class ModeShellWorkflow : public IModeWorkflow {
public:
	ModeShellWorkflow(ModeId modeId, const char* shortName);

	ModeId modeId() const override;
	const char* shortName() const override;
	void onEnter(uint32_t nowMs) override;
	void onExit(uint32_t nowMs) override;
	void onTick(uint32_t nowMs) override;
	void onInputEvent(const NormalizedInputEvent& event,
	               const InputSemanticAction& action,
	               uint32_t nowMs) override;
	void populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const override;

private:
	ModeId modeId_ = ModeId::None;
	const char* shortName_ = "-";
	bool active_ = false;
	uint32_t enteredAtMs_ = 0;
	uint32_t lastInputReceivedMs_ = 0;
	NormalizedInputEvent lastInputEvent_ = {};
};

}  // namespace dukatimer