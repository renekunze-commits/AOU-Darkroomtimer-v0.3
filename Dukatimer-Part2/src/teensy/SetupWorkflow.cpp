/*
 * SetupWorkflow
 *
 * Dieser Workflow kapselt die globale Systemkonfiguration getrennt von Papier,
 * Kalibrierung und Servicewerkzeugen. Er arbeitet nur gegen den neuen
 * SystemSettings-Port und bleibt damit frei von direkten Storage- oder UI-Globals.
 */

#include "SetupWorkflow.h"

#include "ModeWorkflowServices.h"

namespace dukatimer {

namespace {

constexpr uint8_t kSoundModeCount = 3u;
constexpr uint8_t kSoundVolumeCount = 3u;
constexpr float kMinimumThermalGapCelsius = 5.0f;

}  // namespace

ModeId SetupWorkflow::modeId() const {
	return ModeId::Setup;
}

const char* SetupWorkflow::shortName() const {
	return "SETUP";
}

void SetupWorkflow::onEnter(uint32_t nowMs) {
	active_ = true;
	editingActive_ = false;
	parametersDirty_ = false;
	persistFailed_ = false;
	selectedItem_ = SetupMenuItem::SoundMode;
	enteredAtMs_ = nowMs;
	selectedItemChangedAtMs_ = nowMs;
	refreshStoredSettings(true);
}

void SetupWorkflow::onExit(uint32_t nowMs) {
	(void)nowMs;
	active_ = false;
	editingActive_ = false;
	parametersDirty_ = false;
	persistFailed_ = false;
	enteredAtMs_ = 0u;
	selectedItemChangedAtMs_ = 0u;
	refreshStoredSettings(true);
}

void SetupWorkflow::onTick(uint32_t nowMs) {
	(void)nowMs;
	if (!active_ || editingActive_) {
		return;
	}

	refreshStoredSettings(false);
}

void SetupWorkflow::onInputEvent(const NormalizedInputEvent& event,
	                            const InputSemanticAction& action,
	                            uint32_t nowMs) {
	if (!active_ || !event.isMeaningful() || !action.isMeaningful()) {
		return;
	}

	switch (action.kind) {
		case InputSemanticActionKind::ContextPrevious:
			if (!editingActive_) {
				cycleSelectedItem(-1, nowMs);
			}
			break;

		case InputSemanticActionKind::ContextNext:
			if (!editingActive_) {
				cycleSelectedItem(1, nowMs);
			}
			break;

		case InputSemanticActionKind::AdjustPrimaryDecrease:
		case InputSemanticActionKind::AdjustSecondaryDecrease:
			if (editingActive_) {
				adjustSelectedItem(-1, nowMs);
			}
			break;

		case InputSemanticActionKind::AdjustPrimaryIncrease:
		case InputSemanticActionKind::AdjustSecondaryIncrease:
			if (editingActive_) {
				adjustSelectedItem(1, nowMs);
			}
			break;

		case InputSemanticActionKind::Confirm:
			handleConfirm(nowMs);
			break;

		case InputSemanticActionKind::Undo:
			handleUndo();
			break;

		case InputSemanticActionKind::Start:
		case InputSemanticActionKind::Measure:
		case InputSemanticActionKind::Pause:
		case InputSemanticActionKind::Resume:
		case InputSemanticActionKind::None:
			break;
	}
}

void SetupWorkflow::populateRuntimeState(ModeRuntimeState& state, uint32_t nowMs) const {
	if (!active_) {
		return;
	}

	state.setup.selectedItem = selectedItem_;
	state.setup.itemCount = static_cast<uint8_t>(SetupMenuItem::Count);
	state.setup.editingActive = editingActive_;
	state.setup.parametersDirty = parametersDirty_;
	state.setup.persistFailed = persistFailed_;
	state.setup.selectedItemAgeMs = nowMs - selectedItemChangedAtMs_;
	state.setup.stagedSettings = stagedSettings_;

	const ModeWorkflowServices* boundServices = services();
	if (boundServices != nullptr && boundServices->headTimingDiagnosticsQuery != nullptr) {
		state.setup.headTimingDiagnosticsEnabled =
			boundServices->headTimingDiagnosticsQuery->headTimingDiagnosticsEnabled();
	}

	if (boundServices != nullptr && boundServices->systemSettingsQuery != nullptr) {
		state.setup.storageErrorCode = boundServices->systemSettingsQuery->storageErrorCode();
		state.setup.storageErrorDetail = boundServices->systemSettingsQuery->storageErrorDetail();
	}
}

void SetupWorkflow::refreshStoredSettings(bool overwriteStaged) {
	const ModeWorkflowServices* boundServices = services();
	if (boundServices == nullptr || boundServices->systemSettingsQuery == nullptr) {
		storedSettings_ = makeDefaultSystemSettings();
		if (overwriteStaged) {
			stagedSettings_ = storedSettings_;
			parametersDirty_ = false;
		}
		return;
	}

	storedSettings_ = boundServices->systemSettingsQuery->currentSettings();
	if (overwriteStaged) {
		stagedSettings_ = storedSettings_;
		parametersDirty_ = false;
	}
}

void SetupWorkflow::cycleSelectedItem(int8_t delta, uint32_t nowMs) {
	setSelectedItem(wrapSelectedItem(static_cast<int>(selectedItem_) + static_cast<int>(delta)), nowMs);
	persistFailed_ = false;
}

void SetupWorkflow::adjustSelectedItem(int8_t delta, uint32_t nowMs) {
	(void)nowMs;
	persistFailed_ = false;

	switch (selectedItem_) {
		case SetupMenuItem::SoundMode: {
			int mode = static_cast<int>(stagedSettings_.soundMode) + static_cast<int>(delta);
			if (mode < 0) {
				mode = static_cast<int>(kSoundModeCount) - 1;
			} else if (mode >= static_cast<int>(kSoundModeCount)) {
				mode = 0;
			}
			stagedSettings_.soundMode = static_cast<SoundFeedbackMode>(mode);
			break;
		}

		case SetupMenuItem::SoundVolume: {
			int volume = static_cast<int>(stagedSettings_.soundVolume) + static_cast<int>(delta);
			if (volume < 0) {
				volume = static_cast<int>(kSoundVolumeCount) - 1;
			} else if (volume >= static_cast<int>(kSoundVolumeCount)) {
				volume = 0;
			}
			stagedSettings_.soundVolume = static_cast<SoundVolumeLevel>(volume);
			break;
		}

		case SetupMenuItem::Vibration:
			stagedSettings_.vibrationEnabled = stagedSettings_.vibrationEnabled == 0u ? 1u : 0u;
			break;

		case SetupMenuItem::MaxHeadBrightness: {
			int next = static_cast<int>(stagedSettings_.maxHeadBrightnessPercent) +
			           (static_cast<int>(delta) * static_cast<int>(kBrightnessStepPercent));
			if (next < static_cast<int>(kMinimumHeadBrightnessPercent)) {
				next = static_cast<int>(kMinimumHeadBrightnessPercent);
			}
			if (next > 100) {
				next = 100;
			}
			stagedSettings_.maxHeadBrightnessPercent = static_cast<uint8_t>(next);
			break;
		}

		case SetupMenuItem::HeadTimingDiagnostics:
			break;

		case SetupMenuItem::ThermalDeratingStart: {
			float next = stagedSettings_.thermalProtection.deratingStartCelsius +
			             (static_cast<float>(delta) * kThermalStepCelsius);
			const float maxAllowed = stagedSettings_.thermalProtection.hardStopCelsius -
			                         kMinimumThermalGapCelsius;
			if (next < 30.0f) {
				next = 30.0f;
			}
			if (next > maxAllowed) {
				next = maxAllowed;
			}
			stagedSettings_.thermalProtection.deratingStartCelsius = next;
			break;
		}

		case SetupMenuItem::ThermalHardStop: {
			float next = stagedSettings_.thermalProtection.hardStopCelsius +
			             (static_cast<float>(delta) * kThermalStepCelsius);
			const float minAllowed = stagedSettings_.thermalProtection.deratingStartCelsius +
			                         kMinimumThermalGapCelsius;
			if (next < minAllowed) {
				next = minAllowed;
			}
			if (next > 90.0f) {
				next = 90.0f;
			}
			stagedSettings_.thermalProtection.hardStopCelsius = next;
			break;
		}

		case SetupMenuItem::Apply:
		case SetupMenuItem::Discard:
		case SetupMenuItem::SafetyDefaults:
		case SetupMenuItem::Count:
			break;
	}

	updateDirtyState();
}

void SetupWorkflow::handleConfirm(uint32_t nowMs) {
	if (editingActive_) {
		editingActive_ = false;
		updateDirtyState();
		persistFailed_ = false;
		return;
	}

	if (isActionItem(selectedItem_)) {
		executeActionItem(selectedItem_, nowMs);
		return;
	}

	editBackupSettings_ = stagedSettings_;
	editingActive_ = true;
	persistFailed_ = false;
}

void SetupWorkflow::handleUndo() {
	if (editingActive_) {
		stagedSettings_ = editBackupSettings_;
		editingActive_ = false;
		updateDirtyState();
		persistFailed_ = false;
		return;
	}

	if (parametersDirty_) {
		stagedSettings_ = storedSettings_;
		parametersDirty_ = false;
		persistFailed_ = false;
	}
}

void SetupWorkflow::executeActionItem(SetupMenuItem item, uint32_t nowMs) {
	(void)nowMs;
	persistFailed_ = false;

	const ModeWorkflowServices* boundServices = services();
	switch (item) {
		case SetupMenuItem::HeadTimingDiagnostics:
			if (boundServices == nullptr || boundServices->headTimingDiagnosticsCommands == nullptr ||
			    boundServices->headTimingDiagnosticsQuery == nullptr) {
				return;
			}

			// Die Head-Timing-Diagnose bleibt bewusst runtime-only. Dieser
			// Setup-Eintrag toggelt nur das feste Bring-up-Muster plus
			// Serial-Reporting und schreibt keine persistenten Produktwerte.
			boundServices->headTimingDiagnosticsCommands->setHeadTimingDiagnosticsEnabled(
				!boundServices->headTimingDiagnosticsQuery->headTimingDiagnosticsEnabled());
			break;

		case SetupMenuItem::Apply:
			if (boundServices == nullptr || boundServices->systemSettingsCommands == nullptr) {
				persistFailed_ = true;
				return;
			}

			if (!boundServices->systemSettingsCommands->saveSettings(stagedSettings_)) {
				persistFailed_ = true;
				return;
			}

			refreshStoredSettings(true);
			persistFailed_ = false;
			break;

		case SetupMenuItem::Discard:
			stagedSettings_ = storedSettings_;
			parametersDirty_ = false;
			editingActive_ = false;
			break;

		case SetupMenuItem::SafetyDefaults:
			stagedSettings_.thermalProtection = makeDefaultSystemSettings().thermalProtection;
			updateDirtyState();
			break;

		case SetupMenuItem::SoundMode:
		case SetupMenuItem::SoundVolume:
		case SetupMenuItem::Vibration:
		case SetupMenuItem::MaxHeadBrightness:
		case SetupMenuItem::ThermalDeratingStart:
		case SetupMenuItem::ThermalHardStop:
		case SetupMenuItem::Count:
			break;
	}
}

void SetupWorkflow::setSelectedItem(SetupMenuItem item, uint32_t nowMs) {
	if (selectedItem_ == item) {
		return;
	}

	selectedItem_ = item;
	selectedItemChangedAtMs_ = nowMs;
}

void SetupWorkflow::updateDirtyState() {
	parametersDirty_ = stagedSettings_ != storedSettings_;
}

bool SetupWorkflow::isActionItem(SetupMenuItem item) {
	return item == SetupMenuItem::Apply || item == SetupMenuItem::Discard ||
	       item == SetupMenuItem::SafetyDefaults ||
	       item == SetupMenuItem::HeadTimingDiagnostics;
}

SetupMenuItem SetupWorkflow::wrapSelectedItem(int nextIndex) {
	const int itemCount = static_cast<int>(SetupMenuItem::Count);
	if (itemCount <= 0) {
		return SetupMenuItem::SoundMode;
	}

	while (nextIndex < 0) {
		nextIndex += itemCount;
	}

	while (nextIndex >= itemCount) {
		nextIndex -= itemCount;
	}

	return static_cast<SetupMenuItem>(nextIndex);
}

}  // namespace dukatimer