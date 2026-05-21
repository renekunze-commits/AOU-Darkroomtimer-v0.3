#pragma once

#include <stdint.h>

#include "IModeWorkflow.h"

namespace dukatimer {

class SetupWorkflow : public IModeWorkflow {
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
	static constexpr uint8_t kBrightnessStepPercent = 5u;
	static constexpr uint8_t kMinimumHeadBrightnessPercent = 10u;
	static constexpr float kThermalStepCelsius = 1.0f;

	bool active_ = false;
	bool editingActive_ = false;
	bool parametersDirty_ = false;
	bool persistFailed_ = false;
	SetupMenuItem selectedItem_ = SetupMenuItem::SoundMode;
	uint32_t enteredAtMs_ = 0u;
	uint32_t selectedItemChangedAtMs_ = 0u;
	SystemSettings storedSettings_ = makeDefaultSystemSettings();
	SystemSettings stagedSettings_ = makeDefaultSystemSettings();
	SystemSettings editBackupSettings_ = makeDefaultSystemSettings();

	void refreshStoredSettings(bool overwriteStaged);
	void cycleSelectedItem(int8_t delta, uint32_t nowMs);
	void adjustSelectedItem(int8_t delta, uint32_t nowMs);
	void handleConfirm(uint32_t nowMs);
	void handleUndo();
	void executeActionItem(SetupMenuItem item, uint32_t nowMs);
	void setSelectedItem(SetupMenuItem item, uint32_t nowMs);
	void updateDirtyState();

	static bool isActionItem(SetupMenuItem item);
	static SetupMenuItem wrapSelectedItem(int nextIndex);
};

}  // namespace dukatimer