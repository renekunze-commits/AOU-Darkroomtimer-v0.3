#pragma once

#include "HeadLightCommand.h"

namespace dukatimer {

/*
 * HeadLightArbiter
 *
 * Zweck:
 * - fuehrt lokale Lichtwuensche und den uebergeordneten Belichtungspfad zu
 *   genau einem sichtbaren HeadLightCommand zusammen
 * - erzwingt dabei die Regel, dass eine aktive Belichtung lokale Kopflicht-
 *   Signale immer uebersteuern darf
 *
 * Architekturgrenze:
 * - der Arbiter kennt keine Hardware und keine Moduslogik
 * - er entscheidet nur Prioritaet und Resultat zwischen zwei Quellen
 */
class HeadLightArbiter {
public:
	void setLocalSource(const HeadLightSourceState& source) {
		localSource_ = source;
	}

	void setExposureSource(const HeadLightSourceState& source) {
		exposureSource_ = source;
	}

	const HeadLightSourceState& localSource() const {
		return localSource_;
	}

	const HeadLightSourceState& exposureSource() const {
		return exposureSource_;
	}

	bool exposureOverrideActive() const {
		return exposureSource_.active;
	}

	HeadLightCommand resolvedCommand() const {
		// Uebergeordnete Belichtungszustande muessen lokale Schalterausgaenge hart
		// uebersteuern koennen. Das aktive Bit ist deshalb bewusst getrennt vom
		// eigentlichen Kommando, damit auch ein aktives OFF Exposure > Local sauber
		// abbildbar bleibt.
		if (exposureSource_.active) {
			return exposureSource_.command;
		}

		if (localSource_.active) {
			return localSource_.command;
		}

		return makeHeadLightOff();
	}

private:
	HeadLightSourceState localSource_ = {};
	HeadLightSourceState exposureSource_ = {};
};

}  // namespace dukatimer