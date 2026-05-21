#pragma once

#include <stddef.h>
#include <stdint.h>

#include "IModeWorkflow.h"
#include "ModeRuntimeState.h"

namespace dukatimer {

/*
 * ModeCoordinator
 *
 * Zweck:
 * - haelt die kleine Menge aktiver Workflows und den aktuell sichtbaren Modus
 * - verteilt normierte Eingabeereignisse an genau einen aktiven Workflow
 * - baut aus dem aktiven Workflow den rohen ModeRuntimeState fuer Snapshot/UI
 *
 * Architekturgrenze:
 * - der Coordinator kennt keine Modusdetails; er schaltet nur zwischen
 *   Workflows um und ruft deren wohldefinierte Schnittstelle auf
 */
class ModeCoordinator {
public:
	ModeCoordinator(IModeWorkflow* const* workflows, size_t workflowCount);

	void bindServices(const ModeWorkflowServices& services);
	void begin(uint32_t nowMs, ModeId initialMode = ModeId::Splitgrade);
	void tick(uint32_t nowMs);
	bool requestMode(ModeId modeId, uint32_t nowMs);
	void handleInputEvent(const NormalizedInputEvent& event,
	                    const InputSemanticAction& action,
	                    uint32_t nowMs);

	const ModeRuntimeState& state() const;

private:
	static constexpr size_t kMaxWorkflowCount = 6;

	IModeWorkflow* workflows_[kMaxWorkflowCount] = {};
	size_t workflowCount_ = 0u;
	IModeWorkflow* activeWorkflow_ = nullptr;
	ModeRuntimeState state_ = makeUnknownModeRuntimeState();
	uint32_t activeModeStartedMs_ = 0;
	uint32_t lastInputReceivedMs_ = 0;

	IModeWorkflow* findWorkflow(ModeId modeId) const;
	void leaveActiveWorkflow(uint32_t nowMs);
	void enterWorkflow(IModeWorkflow& workflow, uint32_t nowMs);
	void updateDerivedState(uint32_t nowMs);
};

}  // namespace dukatimer