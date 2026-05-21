#pragma once

#include <stdint.h>

#include "PaperExposureProfile.h"

namespace dukatimer {

/*
 * PaperProfileQueryPort
 *
 * Fester Lesepfad fuer Papier-, Slot- und Profilabfragen in Workflows.
 * AP-05 liefert den ersten konkreten Query-Umfang, damit Workflows Daten aus
 * Slot-Bank und Persistenzstatus lesen koennen, ohne Storage-Objekte direkt
 * zu kennen.
 */
class PaperProfileQueryPort {
public:
	virtual ~PaperProfileQueryPort() = default;

	virtual uint8_t slotCount() const = 0;
	virtual uint8_t activeSlotIndex() const = 0;
	virtual const PaperExposureProfile* activeProfile() const = 0;
	virtual const PaperExposureProfile* profileAt(uint8_t slotIndex) const = 0;
	virtual bool hasCalibratedActiveProfile() const = 0;
	virtual uint8_t storageErrorCode() const = 0;
	virtual uint32_t storageErrorDetail() const = 0;
};

}  // namespace dukatimer