#pragma once

#include "SystemSettingsCommandPort.h"
#include "SystemSettingsQueryPort.h"
#include "SystemSettingsStorage.h"

namespace dukatimer {

class SystemSettingsCommandAdapter : public SystemSettingsCommandPort {
public:
	SystemSettingsCommandAdapter(SystemSettings& settings, SystemSettingsStorage& storage)
		: settings_(&settings), storage_(&storage) {}

	bool saveSettings(const SystemSettings& settings) override {
		if (settings_ == nullptr || storage_ == nullptr) {
			return false;
		}

		SystemSettings nextSettings = settings;
		if (!storage_->save(nextSettings)) {
			return false;
		}

		*settings_ = nextSettings;
		return true;
	}

private:
	SystemSettings* settings_ = nullptr;
	SystemSettingsStorage* storage_ = nullptr;
};

class SystemSettingsQueryAdapter : public SystemSettingsQueryPort {
public:
	SystemSettingsQueryAdapter(const SystemSettings& settings, const SystemSettingsStorage& storage)
		: settings_(&settings), storage_(&storage) {}

	const SystemSettings& currentSettings() const override {
		static const SystemSettings kDefaultSettings = makeDefaultSystemSettings();
		return settings_ != nullptr ? *settings_ : kDefaultSettings;
	}

	uint8_t storageErrorCode() const override {
		if (storage_ == nullptr) {
			return static_cast<uint8_t>(SystemSettingsStorageError::StorageUnavailable);
		}

		return static_cast<uint8_t>(storage_->status().lastError);
	}

	uint32_t storageErrorDetail() const override {
		if (storage_ == nullptr) {
			return 0u;
		}

		return storage_->status().detail;
	}

private:
	const SystemSettings* settings_ = nullptr;
	const SystemSettingsStorage* storage_ = nullptr;
};

}  // namespace dukatimer