#pragma once

#include <stdint.h>

#include "InputSemanticAction.h"
#include "ModeWorkflowServices.h"
#include "ModeRuntimeState.h"
#include "NormalizedInputEvent.h"

namespace dukatimer {

/*
 * IModeWorkflow
 *
 * Gemeinsamer Vertragsrand aller Modi gegenueber ModeCoordinator. Jeder Modus
 * kapselt seinen eigenen Bedien- und Laufzeitkern hinter diesem minimalen Satz
 * aus Lifecycle-, Input- und RuntimeState-Funktionen.
 */
class IModeWorkflow {
public:
	virtual ~IModeWorkflow() = default;

	// Workflows erhalten gemeinsame Dienste einmalig ueber diesen festen Rand.
	// AP-04 nutzt ihn zunaechst nur als Skelett; konkrete Ports bleiben vorerst
	// unverdrahtet oder null, damit kein Laufzeitverhalten geaendert wird.
	virtual void bindServices(const ModeWorkflowServices& services) {
		services_ = &services;
	}

	virtual ModeId modeId() const = 0;
	virtual const char* shortName() const = 0;
	virtual void onEnter(uint32_t nowMs) = 0;
	virtual void onExit(uint32_t nowMs) = 0;
	virtual void onTick(uint32_t nowMs) = 0;
	virtual void onInputEvent(const NormalizedInputEvent& event,
	                        const InputSemanticAction& action,
	                        uint32_t nowMs) = 0;
	virtual void populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const = 0;

protected:
	const ModeWorkflowServices* services() const {
		return services_;
	}

private:
	const ModeWorkflowServices* services_ = nullptr;
};

}  // namespace dukatimer