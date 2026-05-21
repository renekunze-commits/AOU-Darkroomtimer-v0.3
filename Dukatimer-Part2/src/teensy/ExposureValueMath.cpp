#include "ExposureValueMath.h"

#include <cmath>

namespace dukatimer {

float ExposureValueMath::relativeEvFromLux(float lux, float referenceLux) {
	if (lux <= 0.0f || referenceLux <= 0.0f || !std::isfinite(lux) || !std::isfinite(referenceLux)) {
		return NAN;
	}

	return std::log2(lux / referenceLux);
}

float ExposureValueMath::evDeltaToMultiplier(float evDeltaStops) {
	return powf(2.0f, evDeltaStops);
}

float ExposureValueMath::applyEvDeltaStops(float baseValue, float evDeltaStops) {
	return baseValue * evDeltaToMultiplier(evDeltaStops);
}

float ExposureValueMath::applyStepDirection(float baseValue, int8_t direction, float evStepStops) {
	if (direction == 0) {
		return baseValue;
	}

	return applyEvDeltaStops(baseValue, static_cast<float>(direction) * evStepStops);
}

}  // namespace dukatimer
