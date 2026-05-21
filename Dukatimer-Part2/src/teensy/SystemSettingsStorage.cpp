/*
 * SystemSettingsStorage
 *
 * Dieser SD-Rand kapselt Laden und Speichern der globalen Systemsettings.
 * Er spiegelt bewusst die robuste Paper-Slot-Persistenz, bleibt aber getrennt,
 * damit globale Produktkonfiguration nicht im Papierpfad mitlaeuft.
 */

#include "SystemSettingsStorage.h"

#include <Arduino.h>
#include <string.h>

#include "SystemSettingsPersistenceCodec.h"

namespace dukatimer {

namespace {

EXTMEM uint8_t gSystemSettingsBlobIoBuffer[SystemSettingsPersistenceCodec::blobSize()];
constexpr uint8_t kLoadAttemptCount = 3u;
constexpr const char* kBackupPath = "/systemsettings.bin.bak";

bool replaceFileWithBackup(TeensyStorageVolume* sd,
	                      const char* temporaryPath,
	                      const char* finalPath,
	                      const char* backupPath) {
	if (sd == nullptr || temporaryPath == nullptr || finalPath == nullptr || backupPath == nullptr) {
		return false;
	}

	if (sd->exists(backupPath) && !sd->remove(backupPath)) {
		return false;
	}

	const bool finalExists = sd->exists(finalPath);
	if (finalExists && !sd->rename(finalPath, backupPath)) {
		return false;
	}

	if (sd->rename(temporaryPath, finalPath)) {
		if (finalExists) {
			(void)sd->remove(backupPath);
		}
		return true;
	}

	if (finalExists) {
		(void)sd->rename(backupPath, finalPath);
	}

	return false;
}

}  // namespace

FLASHMEM SystemSettingsStorage::SystemSettingsStorage(TeensyStorageVolume* sdCard) {
	attachStorage(sdCard);
}

FLASHMEM void SystemSettingsStorage::attachStorage(TeensyStorageVolume* sdCard) {
	sd_ = sdCard;
	storageMounted_ = false;
	setError(SystemSettingsStorageError::None, 0u);
}

FLASHMEM bool SystemSettingsStorage::load(SystemSettings& outSettings) {
	if (!ensureStorageMounted()) {
		return false;
	}

	const size_t expectedBlobSize = SystemSettingsPersistenceCodec::blobSize();
	uint8_t* buffer = gSystemSettingsBlobIoBuffer;
	SystemSettingsStorageError finalError = SystemSettingsStorageError::None;
	uint32_t finalDetail = 0u;

	for (uint8_t attempt = 0u; attempt < kLoadAttemptCount; ++attempt) {
		TeensyStorageFile file;
		if (!file.open(kStoragePath, O_RDONLY)) {
			setError(sd_->exists(kStoragePath) ? SystemSettingsStorageError::OpenFailed
			                                 : SystemSettingsStorageError::FileNotFound,
			         0u);
			return false;
		}

		size_t bytesRead = 0u;
		while (bytesRead < expectedBlobSize) {
			const int chunkRead = file.read(buffer + bytesRead, expectedBlobSize - bytesRead);
			if (chunkRead <= 0) {
				break;
			}

			bytesRead += static_cast<size_t>(chunkRead);
		}

		uint8_t trailingByte = 0u;
		const int trailingRead = file.read(&trailingByte, 1u);
		file.close();

		if (bytesRead != expectedBlobSize || trailingRead > 0) {
			finalError = SystemSettingsStorageError::ReadFailed;
			finalDetail = static_cast<uint32_t>(bytesRead);
			continue;
		}

		SystemSettingsBlobHeader observedHeader = {};
		memcpy(&observedHeader, buffer, sizeof(SystemSettingsBlobHeader));

		SystemSettings loadedSettings = {};
		SystemSettingsBlobParseError parseError = SystemSettingsBlobParseError::None;
		if (!SystemSettingsPersistenceCodec::parseBlobWithError(buffer, expectedBlobSize, loadedSettings,
		                                                   &parseError)) {
			if (parseError == SystemSettingsBlobParseError::UnsupportedFormatVersion) {
				setError(SystemSettingsStorageError::UnsupportedFormatVersion,
				         static_cast<uint32_t>(observedHeader.formatVersion));
				return false;
			}

			if (parseError == SystemSettingsBlobParseError::InvalidSettings) {
				setError(SystemSettingsStorageError::InvalidSettings,
				         static_cast<uint32_t>(parseError));
				return false;
			}

			finalError = SystemSettingsStorageError::InvalidBlob;
			finalDetail = static_cast<uint32_t>(parseError);
			continue;
		}

		outSettings = loadedSettings;
		setError(SystemSettingsStorageError::None, 0u);
		return true;
	}

	setError(finalError, finalDetail);
	return false;
}

FLASHMEM bool SystemSettingsStorage::save(const SystemSettings& settings) {
	if (!SystemSettingsPersistenceCodec::validateSettings(settings)) {
		setError(SystemSettingsStorageError::InvalidSettings, 0u);
		return false;
	}

	if (!ensureStorageMounted()) {
		return false;
	}

	const size_t expectedBlobSize = SystemSettingsPersistenceCodec::blobSize();
	uint8_t* buffer = gSystemSettingsBlobIoBuffer;

	size_t blobBytes = 0u;
	if (!SystemSettingsPersistenceCodec::buildBlob(settings, buffer, expectedBlobSize, blobBytes)) {
		setError(SystemSettingsStorageError::InvalidSettings, 0u);
		return false;
	}

	(void)sd_->remove(kTemporaryPath);

	TeensyStorageFile tempFile;
	if (!tempFile.open(kTemporaryPath, O_WRONLY | O_CREAT | O_TRUNC)) {
		setError(SystemSettingsStorageError::OpenFailed, 0u);
		return false;
	}

	const size_t writtenBytes = tempFile.write(buffer, blobBytes);
	tempFile.close();

	if (writtenBytes != blobBytes) {
		(void)sd_->remove(kTemporaryPath);
		setError(SystemSettingsStorageError::WriteFailed, static_cast<uint32_t>(writtenBytes));
		return false;
	}

	if (!replaceFileWithBackup(sd_, kTemporaryPath, kStoragePath, kBackupPath)) {
		(void)sd_->remove(kTemporaryPath);
		setError(SystemSettingsStorageError::RenameFailed, 0u);
		return false;
	}

	setError(SystemSettingsStorageError::None, 0u);
	return true;
}

FLASHMEM const SystemSettingsStorageStatus& SystemSettingsStorage::status() const {
	return status_;
}

FLASHMEM const char* SystemSettingsStorage::path() const {
	return kStoragePath;
}

FLASHMEM bool SystemSettingsStorage::ensureStorageMounted() {
	if (sd_ == nullptr) {
		setError(SystemSettingsStorageError::StorageUnavailable, 0u);
		return false;
	}

	if (storageMounted_) {
		return true;
	}

	storageMounted_ = beginTeensyStorageVolume(*sd_);
	if (!storageMounted_) {
		setError(SystemSettingsStorageError::MountFailed, 0u);
	}

	return storageMounted_;
}

FLASHMEM void SystemSettingsStorage::setError(SystemSettingsStorageError error, uint32_t detail) {
	status_.lastError = error;
	status_.detail = detail;
}

}  // namespace dukatimer