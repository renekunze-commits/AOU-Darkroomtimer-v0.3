#pragma once

#include <array>
#include <stdint.h>

#include "MeasurementQueryPort.h"

namespace dukatimer {

constexpr uint16_t kMeasurementRuntimeStatusSchemaVersion = 9;

struct MeasurementRuntimeStatus {
	uint16_t schemaVersion = kMeasurementRuntimeStatusSchemaVersion;
	MeasurementLuxSample activeLux = {};
	MeasurementReferenceStatus activeReference = {};
	bool activeRelativeEvValid = false;
	float activeRelativeEvStops = 0.0f;
	MeasurementLuxSample localLux = {};
	MeasurementLuxSample wirelessLux = {};
	MeasurementSessionStatus session = {};

	bool operator==(const MeasurementRuntimeStatus& other) const {
		return schemaVersion == other.schemaVersion && activeLux == other.activeLux &&
		       activeReference == other.activeReference &&
		       activeRelativeEvValid == other.activeRelativeEvValid &&
		       activeRelativeEvStops == other.activeRelativeEvStops &&
		       localLux == other.localLux && wirelessLux == other.wirelessLux &&
		       session == other.session;
	}

	bool operator!=(const MeasurementRuntimeStatus& other) const {
		return !(*this == other);
	}
};

inline MeasurementRuntimeStatus makeUnknownMeasurementRuntimeStatus() {
	return MeasurementRuntimeStatus{};
}

}  // namespace dukatimer
