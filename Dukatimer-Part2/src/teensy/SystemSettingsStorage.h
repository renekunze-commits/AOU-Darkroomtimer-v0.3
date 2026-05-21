#pragma once

#include "SystemSettings.h"
#include "TeensyStoragePolicy.h"

namespace dukatimer {

enum class SystemSettingsStorageError : uint8_t {
	None,
	StorageUnavailable,
	MountFailed,
	FileNotFound,
	OpenFailed,
	ReadFailed,
	WriteFailed,
	InvalidBlob,
	UnsupportedFormatVersion,
	InvalidSettings,
	BufferAllocationFailed,
	RenameFailed,
};

struct SystemSettingsStorageStatus {
	SystemSettingsStorageError lastError = SystemSettingsStorageError::None;
	uint32_t detail = 0u;
};

class SystemSettingsStorage {
public:
	explicit SystemSettingsStorage(TeensyStorageVolume* sdCard = nullptr);

	void attachStorage(TeensyStorageVolume* sdCard);
	bool load(SystemSettings& outSettings);
	bool save(const SystemSettings& settings);

	const SystemSettingsStorageStatus& status() const;
	const char* path() const;

private:
	static constexpr const char* kStoragePath = "/systemsettings.bin";
	static constexpr const char* kTemporaryPath = "/systemsettings.bin.tmp";

	TeensyStorageVolume* sd_ = nullptr;
	bool storageMounted_ = false;
	SystemSettingsStorageStatus status_ = {};

	bool ensureStorageMounted();
	void setError(SystemSettingsStorageError error, uint32_t detail = 0u);
};

}  // namespace dukatimer