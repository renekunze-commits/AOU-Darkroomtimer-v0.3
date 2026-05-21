#pragma once

#include <ProductDataStoragePaths.h>

#include "PaperSlotBank.h"
#include "TeensyStoragePolicy.h"

namespace dukatimer {

enum class PaperSlotStorageError : uint8_t {
	None,
	StorageUnavailable,
	MountFailed,
	FileNotFound,
	OpenFailed,
	ReadFailed,
	WriteFailed,
	InvalidBlob,
	UnsupportedFormatVersion,
	InvalidBank,
	BufferAllocationFailed,
	RenameFailed,
};

struct PaperSlotStorageStatus {
	PaperSlotStorageError lastError = PaperSlotStorageError::None;
	uint32_t detail = 0;
};

/*
 * PaperSlotStorage
 *
 * Zweck:
 * - kapselt das Laden und Speichern der gesamten PaperSlotBank auf SD
 * - fuehrt die Blob-Serialisierung nicht selbst aus, sondern delegiert diese an
 *   den zentralen PersistenceCodec
 * - schreibt stets ueber eine temporaere Datei, um halbgeschriebene Profile nach
 *   Stromverlust oder Abbruch zu vermeiden
 */
class PaperSlotStorage {
public:
	explicit PaperSlotStorage(TeensyStorageVolume* sdCard = nullptr);

	void attachStorage(TeensyStorageVolume* sdCard);
	bool load(PaperSlotBank& outBank);
	bool save(const PaperSlotBank& bank);

	const PaperSlotStorageStatus& status() const;
	const char* path() const;

private:
	static constexpr const char* kStoragePath = dukatimer::product_data::kPaperSlotsPath;
	static constexpr const char* kTemporaryPath = "/paperslots.bin.tmp";

	TeensyStorageVolume* sd_ = nullptr;
	bool storageMounted_ = false;
	PaperSlotStorageStatus status_ = {};

	bool ensureStorageMounted();
	void setError(PaperSlotStorageError error, uint32_t detail = 0u);
};

}  // namespace dukatimer
