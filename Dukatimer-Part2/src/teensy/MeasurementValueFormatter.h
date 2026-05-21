#pragma once

#include <stddef.h>

#include "MeasurementQueryPort.h"

namespace dukatimer {

/*
 * MeasurementValueFormatter
 *
 * Zentrale Formatter-Schicht fuer Mess- und Belichtungswerte in UI-/Render-Texten.
 * Measurement-spezifische Lux-, EV-, Source- und Validity-Texte werden hier
 * quellenbewusst und ohne versteckte Null-Platzhalter aufgebaut, damit
 * Praezision und Ungueltigkeit nicht pro Presenter-Pfad neu interpretiert
 * werden.
 */
class MeasurementValueFormatter {
public:
	static void formatSplitgradeDoseTelemetry(char* output,
	                                         size_t outputCapacity,
	                                         float currentDose,
	                                         float targetDose,
	                                         float remainingTimeSeconds,
	                                         float measuredLux);
	static void formatMeasurementSources(char* output,
	                                   size_t outputCapacity,
	                                   const MeasurementLuxSample& localLux,
	                                   const MeasurementLuxSample& wirelessLux,
	                                   MeasurementLuxSource activeSource);
	static void formatMeasurementMain(char* output,
	                                size_t outputCapacity,
	                                const MeasurementLuxSample& activeLux);
	static void formatMeasurementMeta(char* output,
	                                size_t outputCapacity,
	                                const MeasurementLuxSample& activeLux);
	static void formatMeasurementReference(char* output,
	                                     size_t outputCapacity,
	                                     const MeasurementReferenceStatus& activeReference,
	                                     MeasurementSessionMode sessionMode,
	                                     bool activeRelativeEvValid,
	                                     float activeRelativeEvStops);
	static void formatMeasurementRange(char* output,
	                                 size_t outputCapacity,
	                                 const MeasurementSessionStatus& session);
	static void formatMeasurementControls(char* output,
	                                    size_t outputCapacity,
	                                    const MeasurementSessionStatus& session);

	// PageMeasurement widget-specific formatters
	// (separate from the multi-line diagnostic formatters above)
	static void formatPageMeasLocalLux(char* output,
	                                   size_t outputCapacity,
	                                   const MeasurementLuxSample& localLux);
	static void formatPageMeasWirelessLux(char* output,
	                                      size_t outputCapacity,
	                                      const MeasurementLuxSample& wirelessLux);
	static const char* formatPageMeasSourceChip(MeasurementLuxSource source, bool valid);
	static void formatPageMeasLuxMain(char* output,
	                                  size_t outputCapacity,
	                                  const MeasurementLuxSample& activeLux);
	static void formatPageMeasLuxAge(char* output,
	                                 size_t outputCapacity,
	                                 const MeasurementLuxSample& activeLux);
	static void formatPageMeasRefLux(char* output,
	                                 size_t outputCapacity,
	                                 const MeasurementReferenceStatus& activeReference);
	static void formatPageMeasEvDiff(char* output,
	                                 size_t outputCapacity,
	                                 bool activeRelativeEvValid,
	                                 float activeRelativeEvStops);
	static void formatPageMeasSession(char* output,
	                                  size_t outputCapacity,
	                                  const MeasurementSessionStatus& session);
};

}  // namespace dukatimer
