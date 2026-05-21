#pragma once

#include <stdint.h>

#include "PaperProfileCommandPort.h"
#include "PaperSlotBank.h"
#include "PaperSlotStorage.h"

namespace dukatimer {

/*
 * PaperSlotBankCommandAdapter
 *
 * Uebersetzt Workflow-Schreiboperationen auf die aktive PaperSlotBank samt
 * transaktionalem Storage-Writeback. Der Adapter mutiert den sichtbaren RAM-
 * Zustand erst, nachdem die SD-Persistenz erfolgreich geschrieben wurde.
 */
class PaperSlotBankCommandAdapter : public PaperProfileCommandPort {
public:
	PaperSlotBankCommandAdapter(PaperSlotBank& bank, PaperSlotStorage& storage)
		: bank_(&bank), storage_(&storage) {}

	bool selectActiveSlot(uint8_t slotIndex) override {
		// Der sichtbare RAM-Zustand folgt absichtlich erst nach erfolgreichem
		// Storage-Writeback. Ein abgebrochener SD-Schreibpfad darf den aktiven
		// Papierkontext nicht lokal umschalten und spaeter wieder verlieren.
		if (bank_ == nullptr || storage_ == nullptr) {
			return false;
		}

		const uint8_t slotCount = bank_->slotCount <= kPaperSlotCount ? bank_->slotCount : kPaperSlotCount;
		if (slotCount == 0u || slotIndex >= slotCount) {
			return false;
		}

		if (bank_->activeSlot == slotIndex) {
			return true;
		}

		PaperSlotBank nextBank = *bank_;
		nextBank.activeSlot = slotIndex;
		if (!storage_->save(nextBank)) {
			return false;
		}

		*bank_ = nextBank;
		return true;
	}

	bool saveProfileAt(uint8_t slotIndex, const PaperExposureProfile& profile) override {
		// Auch profilbezogene CAL-Aenderungen folgen derselben Storage-Policy wie
		// die aktive Auswahl: erst erfolgreich persistieren, dann den sichtbaren
		// RAM-Zustand umschalten.
		return saveProfileToSlot(slotIndex, profile);
	}

	bool saveActiveProfile(const PaperExposureProfile& profile) override {
		if (bank_ == nullptr || storage_ == nullptr) {
			return false;
		}

		const uint8_t slotCount = bank_->slotCount <= kPaperSlotCount ? bank_->slotCount : kPaperSlotCount;
		if (slotCount == 0u) {
			return false;
		}

		const uint8_t activeSlot = bank_->activeSlot < slotCount ? bank_->activeSlot : 0u;
		return saveProfileToSlot(activeSlot, profile);
	}

private:
	bool saveProfileToSlot(uint8_t slotIndex, const PaperExposureProfile& profile) {
		if (bank_ == nullptr || storage_ == nullptr) {
			return false;
		}

		const uint8_t slotCount = bank_->slotCount <= kPaperSlotCount ? bank_->slotCount : kPaperSlotCount;
		if (slotCount == 0u || slotIndex >= slotCount) {
			return false;
		}

		PaperSlotBank nextBank = *bank_;
		nextBank.slots[slotIndex] = profile;
		if (!storage_->save(nextBank)) {
			return false;
		}

		*bank_ = nextBank;
		return true;
	}

	PaperSlotBank* bank_ = nullptr;
	PaperSlotStorage* storage_ = nullptr;
};

}  // namespace dukatimer