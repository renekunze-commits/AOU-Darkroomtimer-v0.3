/*
 * TouchSampler
 *
 * Liest den XPT2046 kontinuierlich und stellt immer den letzten gueltigen
 * Rohzustand bereit. Kalibrierung und Touch-Zonen folgen spaeter in einer
 * hoeheren Schicht.
 */
#pragma once

#include <Arduino.h>
#include <XPT2046_Touchscreen.h>

namespace dukatimer {

struct TouchState {
	// active zeigt an, dass der zuletzt gelesene Punkt die Mindestdruckschwelle
	// erreicht hat und deshalb als verwertbarer Touch gilt.
	bool active = false;
	int16_t rawX = 0;
	int16_t rawY = 0;
	int16_t rawZ = 0;
};

/*
 * TouchSampler
 *
 * Zweck:
 * - liest den Touchcontroller roh aus und cached immer genau den letzten
 *   verwertbaren Rohpunkt
 * - trennt damit Hardwarezugriff und Mindestdruckschwelle von spaeterer
 *   Kalibrierung, Fokuslogik und UI-Gesteninterpretation
 */
class TouchSampler {
public:
	TouchSampler(uint8_t chipSelectPin, uint8_t irqPin, uint8_t rotation, uint16_t minPressure);

	// Initialisiert den Controller und uebernimmt die feste Rotationsvorgabe.
	void begin();
	// Update liest hoechstens einen aktuellen Rohpunkt ein und cached ihn in
	// state(). Die Auswertung der Touch-Zonen passiert nicht hier.
	void update();
	const TouchState& state() const;

private:
	XPT2046_Touchscreen touch_;
	uint8_t rotation_;
	uint16_t minPressure_;
	TouchState state_;
};

}  // namespace dukatimer