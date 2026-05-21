/*
 * PaperSlotBank
 *
 * Oberes Sammelmodell aller lokal gespeicherten Papierprofile. Die Bank bildet
 * den persistenten Satz auswählbarer Slots, waehrend einzelne Profilinhalte in
 * PaperExposureProfile beschrieben werden.
 */

#pragma once

#include <stdint.h>

#include "PaperExposureProfile.h"

namespace dukatimer {

constexpr uint16_t kPaperSlotBankSchemaVersion = 1;
constexpr uint8_t kPaperSlotCount = 20;

// Vollstaendige Sammlung aller Papier-Slots inklusive aktivem Index.
struct PaperSlotBank {
	uint16_t schemaVersion = kPaperSlotBankSchemaVersion;
	uint8_t activeSlot = 1;
	uint8_t slotCount = kPaperSlotCount;
	PaperExposureProfile slots[kPaperSlotCount] = {};

	bool operator==(const PaperSlotBank& other) const {
		if (schemaVersion != other.schemaVersion || activeSlot != other.activeSlot ||
		    slotCount != other.slotCount) {
			return false;
		}

		for (uint8_t index = 0; index < kPaperSlotCount; ++index) {
			if (slots[index] != other.slots[index]) {
				return false;
			}
		}

		return true;
	}

	bool operator!=(const PaperSlotBank& other) const {
		return !(*this == other);
	}
};

}  // namespace dukatimer
