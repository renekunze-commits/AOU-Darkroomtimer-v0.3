/*
 * LightController
 *
 * Kapselt die lokale Licht- und Umfeldsteuerung des Teensy:
 * - Fokus / Save / Room
 * - SaveLatch-Schutz gegen ungewolltes weisses Wiederanspringen
 * - physische Room-Ausgabe auf SSR
 * - lokale Lichtentscheidung fuer den separaten NeoPixel-Lichtkopf
 */
#pragma once

#include <Arduino.h>
#include "HeadLightCommand.h"

namespace dukatimer {

struct LightState {
	// Rohschalterzustaende aus der lokalen Eingabeschicht.
	bool focusSwitchRaw = false;
	bool saveSwitchRaw = false;
	bool roomSwitchRaw = false;
	// SaveLatch bleibt aktiv, bis Fokus physisch einmal auf AUS gefallen ist.
	bool saveLatchActive = false;
	// Abgeleitete Ausgaenge fuer nachgelagerte Schichten und physische Treiber.
	bool derivedFocusOutput = false;
	bool derivedSaveOutput = false;
	bool derivedRoomOutput = false;
	bool previousFocusSwitchRaw = false;

	void update(bool focusRaw, bool saveRaw, bool roomRaw) {
		// Erst ein bewusstes Loslassen von Fokus loescht den Latch wieder.
		if (!focusRaw && previousFocusSwitchRaw) {
			saveLatchActive = false;
		}

		// Save bei gleichzeitig aktivem Fokus sperrt das weisse Licht, bis Fokus
		// einmal physisch aus- und spaeter wieder eingeschaltet wurde.
		if (saveRaw && focusRaw) {
			saveLatchActive = true;
		}

		focusSwitchRaw = focusRaw;
		saveSwitchRaw = saveRaw;
		roomSwitchRaw = roomRaw;

		derivedRoomOutput = roomSwitchRaw;
		derivedSaveOutput = saveSwitchRaw;
		derivedFocusOutput = focusSwitchRaw && !saveSwitchRaw && !saveLatchActive;

		previousFocusSwitchRaw = focusSwitchRaw;
	}
};

class LightController {
public:
	explicit LightController(uint8_t ssrRoomPin);

	// Begin initialisiert nur die lokale Umfeldausgabe. Der eigentliche
	// NeoPixel-Lichtkopf haengt bewusst an einer getrennten Schicht.
	void begin(bool focusRaw, bool saveRaw, bool roomRaw);
	// Setzt nur die logische Eingangssicht. Room und Kopflicht werden getrennt
	// in nachgelagerten Schichten angewendet.
	void setLocalSwitches(bool focusRaw, bool saveRaw, bool roomRaw);
	void applyRoomOutput();
	HeadLightSourceState localHeadSource() const;
	// Liefert den zentralen, bereits abgeleiteten Lichtzustand fuer UI und spaeter
	// fuer weitere Fachschichten.
	const LightState& state() const;

private:
	uint8_t ssrRoomPin_;
	LightState state_;
};

}  // namespace dukatimer