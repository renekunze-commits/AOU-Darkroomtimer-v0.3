/*
 * TouchSampler.cpp
 *
 * Diese Implementierung kapselt nur das Rohsampling. Sie trifft absichtlich
 * noch keine Aussage ueber UI-Gesten, Kalibrierung oder Touch-Mapping.
 */
/*
 * TouchSampler
 *
 * Diese Implementierung ist bewusst roh gehalten. Sie liefert nur einen stabilen
 * letzten Rohzustand; jede weitere Bedeutung von Touch bleibt in hoeheren
 * Schichten wie InputRouterPolicy oder LvglUi.
 */

#include "TouchSampler.h"

namespace dukatimer {

TouchSampler::TouchSampler(uint8_t chipSelectPin, uint8_t irqPin, uint8_t rotation, uint16_t minPressure)
	: touch_(chipSelectPin, irqPin), rotation_(rotation), minPressure_(minPressure) {}

void TouchSampler::begin() {
	// Rotation wird hier fest auf den Controller gelegt, damit alle spaeteren
	// Touch-Verbraucher bereits dieselbe Rohachsenlage sehen.
	touch_.begin();
	touch_.setRotation(rotation_);
}

void TouchSampler::update() {
	// Ohne gueltigen Touch wird der Cache aktiv auf "kein Touch" zurueckgesetzt,
	// damit die aufrufende Schicht keinen veralteten Punkt weiterzeichnet.
	state_.active = false;

	if (!touch_.touched()) {
		return;
	}

	const TS_Point point = touch_.getPoint();
	if (point.z < minPressure_) {
		return;
	}

	state_.active = true;
	state_.rawX = point.x;
	state_.rawY = point.y;
	state_.rawZ = point.z;
}

const TouchState& TouchSampler::state() const {
	return state_;
}

}  // namespace dukatimer