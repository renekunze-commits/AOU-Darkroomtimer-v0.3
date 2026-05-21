#pragma once

#include "PaperExposureProfile.h"

namespace dukatimer {

/*
 * PaperProfileCommandPort
 *
 * Schreibender Workflow-Zugang zur aktiven Papierprofil-Persistenz. Workflows
 * sollen Profilwerte nicht mehr nur lokal im RAM mutieren, sondern ueber diese
 * Schnittstelle kontrolliert in die zentrale PaperSlotBank zurueckschreiben.
 */
class PaperProfileCommandPort {
public:
	virtual ~PaperProfileCommandPort() = default;

	// Die aktive Slot-Auswahl ist ein eigener Fachschritt neben dem Speichern des
	// aktiven Profils. Dadurch kann die PAPER-Familie den Benutzerkontext sauber
	// umschalten, ohne ein komplettes Profil neu schreiben zu muessen.
	virtual bool selectActiveSlot(uint8_t slotIndex) = 0;
	// Die PAPER-Kalibrierung darf einen gewaehlten Slot bearbeiten, ohne ihn
	// vorher still zum aktiven Druckprofil machen zu muessen. Deshalb braucht der
	// Command-Port einen expliziten, slotbezogenen Save-Pfad.
	virtual bool saveProfileAt(uint8_t slotIndex, const PaperExposureProfile& profile) = 0;
	virtual bool saveActiveProfile(const PaperExposureProfile& profile) = 0;
};

}  // namespace dukatimer