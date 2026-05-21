/*
 * PaperSlotStorage
 *
 * Diese Datei bildet den robusten SD-Rand fuer die Paper-Slot-Persistenz.
 * Sie liest und schreibt immer den kompletten Bankzustand, nutzt einen festen
 * IO-Puffer und behandelt SD-/Dateifehler als klaren Status statt als verstreute
 * Nebenwirkung im Rest des Systems.
 */

#include "PaperSlotStorage.h"

#include <Arduino.h>
#include <new>
#include <string.h>

#include <ProductDataStoragePaths.h>

#include "PaperSlotPersistenceCodec.h"

namespace dukatimer {

namespace {

// Der Blob-Puffer liegt dauerhaft in EXTMEM, damit Persistenz-I/O keinen grossen
// transienten Stack- oder Heapbedarf im Hauptpfad erzeugt.
EXTMEM uint8_t gPaperSlotBlobIoBuffer[PaperSlotPersistenceCodec::blobSize()];
constexpr uint8_t kLoadAttemptCount = 3u;
constexpr const char* kBackupPath = dukatimer::product_data::kPaperSlotsBackupPath;

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

FLASHMEM PaperSlotStorage::PaperSlotStorage(TeensyStorageVolume* sdCard) {
	attachStorage(sdCard);
}

FLASHMEM void PaperSlotStorage::attachStorage(TeensyStorageVolume* sdCard) {
	sd_ = sdCard;
	storageMounted_ = false;
	setError(PaperSlotStorageError::None, 0u);
}

FLASHMEM bool PaperSlotStorage::load(PaperSlotBank& outBank) {
	// load() akzeptiert nur genau einen vollstaendigen, formatgueltigen Blob.
	// Zu kurze, zu lange oder inhaltlich ungueltige Dateien werden strikt verworfen.
	if (!ensureStorageMounted()) {
		return false;
	}

	const size_t expectedBlobSize = PaperSlotPersistenceCodec::blobSize();
	uint8_t* buffer = gPaperSlotBlobIoBuffer;
	PaperSlotStorageError finalError = PaperSlotStorageError::None;
	uint32_t finalDetail = 0u;

	for (uint8_t attempt = 0u; attempt < kLoadAttemptCount; ++attempt) {
		TeensyStorageFile file;
		if (!file.open(kStoragePath, O_RDONLY)) {
			setError(sd_->exists(kStoragePath) ? PaperSlotStorageError::OpenFailed
			                                : PaperSlotStorageError::FileNotFound,
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
			finalError = PaperSlotStorageError::ReadFailed;
			finalDetail = static_cast<uint32_t>(bytesRead);
			continue;
		}

		PaperSlotBlobHeader observedHeader = {};
		memcpy(&observedHeader, buffer, sizeof(PaperSlotBlobHeader));

		PaperSlotBank loadedBank;
		PaperSlotBlobParseError parseError = PaperSlotBlobParseError::None;
		if (!PaperSlotPersistenceCodec::parseBlobWithError(buffer, expectedBlobSize, loadedBank, &parseError)) {
			if (parseError == PaperSlotBlobParseError::UnsupportedFormatVersion) {
				setError(PaperSlotStorageError::UnsupportedFormatVersion,
				         static_cast<uint32_t>(observedHeader.formatVersion));
				return false;
			}

			if (parseError == PaperSlotBlobParseError::InvalidBank) {
				setError(PaperSlotStorageError::InvalidBank,
				         static_cast<uint32_t>(parseError));
				return false;
			}

			finalError = PaperSlotStorageError::InvalidBlob;
			finalDetail = static_cast<uint32_t>(parseError);
			continue;
		}

		outBank = loadedBank;
		setError(PaperSlotStorageError::None, 0u);
		return true;
	}

	setError(finalError, finalDetail);
	return false;
}

FLASHMEM bool PaperSlotStorage::save(const PaperSlotBank& bank) {
	// save() ist transaktional gedacht: validieren, Blob bauen, in .tmp schreiben,
	// dann per rename() atomar zur Zieldatei machen.
	if (!PaperSlotPersistenceCodec::validateBank(bank)) {
		setError(PaperSlotStorageError::InvalidBank, 0u);
		return false;
	}

	if (!ensureStorageMounted()) {
		return false;
	}

	const size_t expectedBlobSize = PaperSlotPersistenceCodec::blobSize();
	uint8_t* buffer = gPaperSlotBlobIoBuffer;

	size_t blobBytes = 0u;
	if (!PaperSlotPersistenceCodec::buildBlob(bank, buffer, expectedBlobSize, blobBytes)) {
		setError(PaperSlotStorageError::InvalidBank, 0u);
		return false;
	}

	(void)sd_->remove(kTemporaryPath);

	TeensyStorageFile tempFile;
	if (!tempFile.open(kTemporaryPath, O_WRONLY | O_CREAT | O_TRUNC)) {
		setError(PaperSlotStorageError::OpenFailed, 0u);
		return false;
	}

	const size_t writtenBytes = tempFile.write(buffer, blobBytes);
	tempFile.close();

	if (writtenBytes != blobBytes) {
		(void)sd_->remove(kTemporaryPath);
		setError(PaperSlotStorageError::WriteFailed, static_cast<uint32_t>(writtenBytes));
		return false;
	}

	const bool renamed = replaceFileWithBackup(sd_, kTemporaryPath, kStoragePath, kBackupPath);

	if (!renamed) {
		(void)sd_->remove(kTemporaryPath);
		setError(PaperSlotStorageError::RenameFailed, 0u);
		return false;
	}

	setError(PaperSlotStorageError::None, 0u);
	return true;
}

FLASHMEM const PaperSlotStorageStatus& PaperSlotStorage::status() const {
	return status_;
}

FLASHMEM const char* PaperSlotStorage::path() const {
	return kStoragePath;
}

FLASHMEM bool PaperSlotStorage::ensureStorageMounted() {
	// Das SD-Mount bleibt lazy. Persistenz darf fehlschlagen, ohne dass Bring-up,
	// UI oder der restliche Teensy-Hauptpfad daran blockieren.
	if (sd_ == nullptr) {
		setError(PaperSlotStorageError::StorageUnavailable, 0u);
		return false;
	}

	if (storageMounted_) {
		return true;
	}

	storageMounted_ = beginTeensyStorageVolume(*sd_);
	if (!storageMounted_) {
		setError(PaperSlotStorageError::MountFailed, 0u);
	}

	return storageMounted_;
}

FLASHMEM void PaperSlotStorage::setError(PaperSlotStorageError error, uint32_t detail) {
	status_.lastError = error;
	status_.detail = detail;
}

}  // namespace dukatimer
