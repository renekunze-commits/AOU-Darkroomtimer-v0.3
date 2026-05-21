#pragma once

#include <stdint.h>

#include "IModeWorkflow.h"
#include "PaperExposureProfile.h"

namespace dukatimer {

/*
 * PaperWorkflow
 *
 * Zweck:
 * - macht die PAPER-Familie erstmals als echten Runtime-Workflow sichtbar
 * - trennt Slot-Auswahl, aktiven Slot-Wechsel und spaetere Kalibrierpfade vom
 *   PRINT-Kontext, statt diese nur implizit im Snapshot zu dokumentieren
 * - haelt `SELECT` und einen ersten echten lokalen `CAL`-Editor produktiv,
 *   ohne bereits einen messgetriebenen Papierkalibrier-Wizard zu behaupten
 */
class PaperWorkflow : public IModeWorkflow {
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

private:
	static constexpr int8_t kSecondarySlotStep = 5;
	static constexpr float kFixedGradeStep = 0.5f;
	static constexpr float kFixedGradeCoarseStep = 1.0f;
	static constexpr float kIsoStep = 5.0f;
	static constexpr float kIsoCoarseStep = 25.0f;
	static constexpr float kKFactorStep = 0.5f;
	static constexpr float kKFactorCoarseStep = 2.0f;
	static constexpr float kMinimumFixedGrade = 0.0f;
	static constexpr float kMaximumFixedGrade = 5.0f;
	static constexpr float kMinimumIso = 10.0f;
	static constexpr float kMaximumIso = 400.0f;
	static constexpr float kMinimumKFactor = 0.0f;
	static constexpr float kMaximumKFactor = 999.0f;
	// Graukeil-Stufeneingaben fuer Methode 1 (visuelle Schwellenwert-Methode).
	// Diese Werte sind rein lokal und werden nicht persistiert; sie dienen nur
	// als Eingabequelle fuer recalculateFromSteps().
	static constexpr uint8_t kStepMin = 1u;
	static constexpr uint8_t kStepMax = 21u;
	static constexpr uint8_t kStepWhiteDefault = 15u;
	static constexpr uint8_t kStepBlackDefault = 8u;

	bool active_ = false;
	bool editingActive_ = false;
	bool parametersDirty_ = false;
	bool persistFailed_ = false;
	// Der Workflow speichert nur den lokalen Bedienkontext. Die eigentlichen
	// Papierdaten bleiben autoritativ in der PaperSlotBank und werden ueber die
	// Query-/Command-Ports gelesen oder umgeschaltet. Fuer CAL werden zunaechst
	// nur die lokal bearbeiteten Basisfelder gespiegelt, nicht die ganze LUT.
	PaperWorkspacePanel panel_ = PaperWorkspacePanel::Inactive;
	uint8_t selectedSlot_ = 0u;
	PaperCalibrationItem selectedItem_ = PaperCalibrationItem::GradeMode;
	uint32_t enteredAtMs_ = 0u;
	uint32_t panelChangedAtMs_ = 0u;
	uint32_t selectedSlotChangedAtMs_ = 0u;
	uint32_t selectedItemChangedAtMs_ = 0u;
	PaperExposureProfile storedProfile_ = {};
	PaperExposureProfile stagedProfile_ = {};
	PaperExposureProfile editBackupProfile_ = {};
	// Stufeneingaben der visuellen Schwellenwert-Methode. Werden beim Eintritt
	// in den CAL-Editor auf sichere Defaultwerte zurueckgesetzt.
	uint8_t stepWhite_ = kStepWhiteDefault;
	uint8_t stepBlack_ = kStepBlackDefault;

	uint8_t currentSlotCount() const;
	uint8_t currentActiveSlot() const;
	const PaperExposureProfile* currentSelectedProfile() const;
	void syncSelectedSlotFromActive(uint32_t nowMs, bool forceAgeReset);
	void enterCalibrationPanel(uint32_t nowMs);
	void leaveCalibrationPanel(uint32_t nowMs);
	void refreshSelectedProfile(bool overwriteStaged);
	void cycleSelectedSlot(int8_t direction, uint32_t nowMs);
	void cycleSelectedItem(int8_t direction, uint32_t nowMs);
	void adjustSelectedItem(int8_t direction, bool coarseStep, uint32_t nowMs);
	void activateSelectedSlot();
	void handleCalibrationConfirm(uint32_t nowMs);
	void handleCalibrationUndo(uint32_t nowMs);
	void executeCalibrationAction(PaperCalibrationItem item, uint32_t nowMs);
	void updateDirtyState();
	// Berechnet ISO-R, ISO-P und k_Bw aus den Stufeneingaben (Methode 1) und
	// schreibt die Ergebnisse direkt in stagedProfile_.
	void recalculateFromSteps();

	static bool isActionItem(PaperCalibrationItem item);
	static PaperCalibrationItem wrapSelectedItem(int nextIndex);
	static float clampFixedGrade(float value);
	static float clampIso(float value);
	static float clampKFactor(float value);
};

}  // namespace dukatimer