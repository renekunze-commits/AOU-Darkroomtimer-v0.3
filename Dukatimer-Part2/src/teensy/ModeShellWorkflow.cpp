/*
 * ModeShellWorkflow
 *
 * Diese Datei ist absichtlich klein: Sie zeigt, wie ein Modus an den Coordinator
 * angeschlossen wird, ohne schon eigene Fachlogik zu besitzen. Damit bleibt der
 * noch nicht ausgebaute BW-Pfad sauber im Modussystem vertreten, ohne falsche
 * Scheinfunktionalitaet zu verstecken.
 */

#include "ModeShellWorkflow.h"

namespace dukatimer {

ModeShellWorkflow::ModeShellWorkflow(ModeId modeId, const char* shortName)
	: modeId_(modeId), shortName_(shortName != nullptr ? shortName : "-") {}

ModeId ModeShellWorkflow::modeId() const {
	return modeId_;
}

const char* ModeShellWorkflow::shortName() const {
	return shortName_;
}

void ModeShellWorkflow::onEnter(uint32_t nowMs) {
	// Beim Eintritt wird nur der minimale Shell-Zustand zurueckgesetzt. Ein echter
	// Fachworkflow wuerde hier spaeter eigene Runtime- oder UI-Initialisierung tun.
	active_ = true;
	enteredAtMs_ = nowMs;
	lastInputReceivedMs_ = 0;
	lastInputEvent_ = {};
}

void ModeShellWorkflow::onExit(uint32_t nowMs) {
	(void)nowMs;
	active_ = false;
	enteredAtMs_ = 0;
	lastInputReceivedMs_ = 0;
	lastInputEvent_ = {};
}

void ModeShellWorkflow::onTick(uint32_t nowMs) {
	(void)nowMs;
}

void ModeShellWorkflow::onInputEvent(const NormalizedInputEvent& event,
	                               const InputSemanticAction& action,
	                               uint32_t nowMs) {
	// Die Shell protokolliert Eingaben derzeit nur implizit ueber ihre lokalen
	// Felder. Sie fuehrt bewusst noch keine fachlichen Reaktionen aus.
	if (!active_ || !event.isMeaningful()) {
		return;
	}

	lastInputEvent_ = event;
	lastInputReceivedMs_ = nowMs;
	(void)action;
	(void)enteredAtMs_;
	(void)lastInputReceivedMs_;
	(void)lastInputEvent_;
}

void ModeShellWorkflow::populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const {
	// Noch kein eigener Shell-State: Der Coordinator traegt bereits activeMode und
	// Altersdaten ein. Weitere Felder folgen erst mit einem echten Modusausbau.
	(void)state;
	(void)nowMs;
}

}  // namespace dukatimer