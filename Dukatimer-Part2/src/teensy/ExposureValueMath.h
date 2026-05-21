#pragma once

#include <stdint.h>

namespace dukatimer {

/*
 * ExposureValueMath
 *
 * Zentrale Mathematikschicht fuer EV-/F-Stop-nahe Belichtungsanpassungen.
 * Workflows sollen fotografische EV-Skalierung nur ueber diese API ausfuehren.
 */
class ExposureValueMath {
public:
	// Historischer Standardschritt aus v0.3/v0.9 fuer interaktive Aenderungen.
	static constexpr float kDefaultEvStepStops = 1.0f / 3.0f;

	// Messpfade duerfen Lux nur ueber eine explizite positive Referenz in relative
	// EV-Abstaende umrechnen. So bleibt klar, dass eine Zone immer von einem
	// Session-/Kalibrieranker und nicht von einer versteckten Global-Konstante lebt.
	static float relativeEvFromLux(float lux, float referenceLux);
	static float evDeltaToMultiplier(float evDeltaStops);
	static float applyEvDeltaStops(float baseValue, float evDeltaStops);
	static float applyStepDirection(float baseValue,
	                               int8_t direction,
	                               float evStepStops = kDefaultEvStepStops);
};

}  // namespace dukatimer
