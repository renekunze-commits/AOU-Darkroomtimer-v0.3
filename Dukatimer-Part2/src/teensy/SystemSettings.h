#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kSystemSettingsSchemaVersion = 1;

enum class SoundFeedbackMode : uint8_t {
	Off = 0,
	FaultsOnly = 1,
	Normal = 2,
};

enum class SoundVolumeLevel : uint8_t {
	Low = 0,
	Medium = 1,
	High = 2,
};

struct ThermalProtectionSettings {
	float deratingStartCelsius = 50.0f;
	float hardStopCelsius = 60.0f;

	bool operator==(const ThermalProtectionSettings& other) const {
		return deratingStartCelsius == other.deratingStartCelsius &&
		       hardStopCelsius == other.hardStopCelsius;
	}

	bool operator!=(const ThermalProtectionSettings& other) const {
		return !(*this == other);
	}
};

struct SystemSettings {
	uint16_t schemaVersion = kSystemSettingsSchemaVersion;
	SoundFeedbackMode soundMode = SoundFeedbackMode::Normal;
	SoundVolumeLevel soundVolume = SoundVolumeLevel::Medium;
	uint8_t vibrationEnabled = 1u;
	uint8_t maxHeadBrightnessPercent = 100u;
	ThermalProtectionSettings thermalProtection = {};

	bool operator==(const SystemSettings& other) const {
		return schemaVersion == other.schemaVersion && soundMode == other.soundMode &&
		       soundVolume == other.soundVolume && vibrationEnabled == other.vibrationEnabled &&
		       maxHeadBrightnessPercent == other.maxHeadBrightnessPercent &&
		       thermalProtection == other.thermalProtection;
	}

	bool operator!=(const SystemSettings& other) const {
		return !(*this == other);
	}
};

inline SystemSettings makeDefaultSystemSettings() {
	SystemSettings settings = {};
	settings.schemaVersion = kSystemSettingsSchemaVersion;
	settings.soundMode = SoundFeedbackMode::Normal;
	settings.soundVolume = SoundVolumeLevel::Medium;
	settings.vibrationEnabled = 1u;
	settings.maxHeadBrightnessPercent = 100u;
	settings.thermalProtection.deratingStartCelsius = 50.0f;
	settings.thermalProtection.hardStopCelsius = 60.0f;
	return settings;
}

}  // namespace dukatimer