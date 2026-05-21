#pragma once

#include <stdint.h>

#include "MeasurementQueryPort.h"

namespace dukatimer {

/*
 * MeasurementCommandPort
 *
 * Schreibender Zugriff fuer Mess-Workflows auf die zentrale Session-Schicht.
 * Die konkrete Session- und Histogrammlogik bleibt im Measurement-Domain-
 * Dienst, statt spaeter pro Workflow eigene Undo- oder Reset-Pfade wachsen zu
 * lassen.
 */
class MeasurementCommandPort {
public:
	virtual ~MeasurementCommandPort() = default;

	virtual bool captureLocalSample(uint32_t nowMs = 0) = 0;
	virtual MeasurementSampleRole cyclePendingCaptureRole(int8_t direction) = 0;
	virtual bool undoLastSessionSample() = 0;
	virtual void resetSession() = 0;
};

}  // namespace dukatimer