#pragma once

#include <stdint.h>

#include "PaperProfileQueryPort.h"
#include "PaperSlotBank.h"
#include "PaperSlotStorage.h"

namespace dukatimer {

/*
 * PaperSlotBankQueryAdapter
 *
 * Uebersetzt den aktuellen Laufzeitzustand von PaperSlotBank/PaperSlotStorage
 * auf den stabilen Query-Port fuer Workflows. Der Adapter ist read-only und
 * enthaelt keine Persistenz- oder Migrationslogik.
 */
class PaperSlotBankQueryAdapter : public PaperProfileQueryPort {
public:
	PaperSlotBankQueryAdapter(const PaperSlotBank& bank, const PaperSlotStorage& storage)
		: bank_(&bank), storage_(&storage) {}

	uint8_t slotCount() const override {
		if (bank_ == nullptr || bank_->slotCount > kPaperSlotCount) {
			return kPaperSlotCount;
		}

		return bank_->slotCount;
	}

	uint8_t activeSlotIndex() const override {
		if (bank_ == nullptr) {
			return 0u;
		}

		const uint8_t count = slotCount();
		if (count == 0u) {
			return 0u;
		}

		return bank_->activeSlot < count ? bank_->activeSlot : 0u;
	}

	const PaperExposureProfile* activeProfile() const override {
		return profileAt(activeSlotIndex());
	}

	const PaperExposureProfile* profileAt(uint8_t slotIndex) const override {
		if (bank_ == nullptr) {
			return nullptr;
		}

		const uint8_t count = slotCount();
		if (slotIndex >= count || slotIndex >= kPaperSlotCount) {
			return nullptr;
		}

		return &bank_->slots[slotIndex];
	}

	bool hasCalibratedActiveProfile() const override {
		const PaperExposureProfile* profile = activeProfile();
		return profile != nullptr && profile->calibrated;
	}

	uint8_t storageErrorCode() const override {
		if (storage_ == nullptr) {
			return static_cast<uint8_t>(PaperSlotStorageError::StorageUnavailable);
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
	const PaperSlotBank* bank_ = nullptr;
	const PaperSlotStorage* storage_ = nullptr;
};

}  // namespace dukatimer
