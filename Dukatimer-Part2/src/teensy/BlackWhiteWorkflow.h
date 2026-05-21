#pragma once

/*
 * BlackWhiteWorkflow.h
 *
 * BW-Workflow fuer Dukatimer-Part2. Bildet eine einzelne SW/BW-Belichtung ab:
 * - FixedGrade-Papier: Weißlicht (alle Kanaele), Gradation durch Papier bestimmt
 * - Multigrade-Papier: Soft/Hard-Mix nach Gradationseinstellung
 *
 * Einfacher als SplitgradeWorkflow: eine Belichtungsphase, kein Filterwechsel.
 *
 * Pattern und Architektur folgen SplitgradeWorkflow.
 */

#include <stdint.h>

#include "ExposureRuntimeState.h"
#include "IModeWorkflow.h"
#include "ModeRuntimeState.h"
#include "PaperExposureProfile.h"

namespace dukatimer {

// Schmale Kommandobruecke zwischen BlackWhiteWorkflow und dem Wiring-Layer.
// Der Workflow darf keine Hardware schalten; er aeussert nur Kommandos.
enum class BlackWhiteExecutionCommandKind : uint8_t {
	None,
	Start,
	PauseExposure,
	ResumeExposure,
	AbortExposure,
	AcknowledgeDone,
	ClearFault,
};

struct BlackWhiteExecutionCommand {
	BlackWhiteExecutionCommandKind kind = BlackWhiteExecutionCommandKind::None;
	ExposureControlMode controlMode = ExposureControlMode::Time;
	float targetValue = 0.0f;
	bool valid = false;

	bool isMeaningful() const {
		return valid && kind != BlackWhiteExecutionCommandKind::None;
	}
};

class BlackWhiteWorkflow : public IModeWorkflow {
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

	// Spiegelt den physischen Engine-Zustand in den BW-Ablauf zurueck.
	void observeExposureState(const ExposureRuntimeState& exposureState, uint32_t nowMs);

	// Gibt ein anstehendes Kommando an den Wiring-Layer ab. Gibt true zurueck
	// wenn ein Kommando verfuegbar war.
	bool consumeExecutionCommand(BlackWhiteExecutionCommand& commandOut);

	// Setzt die Laufzeit-Override-Flag fuer erzwungene Zeit-Belichtung
	// (wenn Dosismessung durch Sensor-Fallback gesperrt ist).
	void setDoseControlForcedTime(bool forcedTime);

private:
	static constexpr float kDefaultTargetValue = 10.0f;
	static constexpr float kDefaultGrade = 2.5f;
	static constexpr float kMinTargetValue = 0.1f;
	static constexpr float kMaxTargetValue = 9999.0f;

	bool active_ = false;
	bool initialized_ = false;
	BwPanel panel_ = BwPanel::Inactive;
	BwExecutionState executionState_ = BwExecutionState::Inactive;
	ExposureControlMode controlMode_ = ExposureControlMode::Time;
	bool doseControlForcedTime_ = false;
	float targetValue_ = kDefaultTargetValue;
	float grade_ = kDefaultGrade;
	bool gradeEditable_ = true;
	bool whiteLight_ = false;
	float softMix_ = 0.5f;
	float hardMix_ = 0.5f;
	PaperExposureProfile paperProfile_ = {};
	bool paperProfileLoaded_ = false;
	bool parametersDirty_ = false;
	uint32_t enteredAtMs_ = 0;
	uint32_t panelChangedAtMs_ = 0;
	ExposurePhase observedExposurePhase_ = ExposurePhase::Idle;
	uint32_t executionStateChangedAtMs_ = 0;
	BlackWhiteExecutionCommand pendingCommand_ = {};

	void cyclePanel(int8_t direction, uint32_t nowMs);
	void adjustPrimaryValue(int8_t direction, uint32_t nowMs);
	bool shouldReloadPaperDrivenState() const;
	void initializePaperDrivenState();
	void refreshMixFractions();
	bool startCurrentExecution(uint32_t nowMs);
	bool handleExecutionAction(const NormalizedInputEvent& event,
	                           const InputSemanticAction& action,
	                           uint32_t nowMs);
	void setExecutionState(BwExecutionState newState, uint32_t nowMs);
	void queueCommand(BlackWhiteExecutionCommandKind kind);
	void toggleControlMode();
	static float clampTargetValue(float value);
};

}  // namespace dukatimer
