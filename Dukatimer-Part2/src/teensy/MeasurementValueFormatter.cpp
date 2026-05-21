#include "MeasurementValueFormatter.h"

#include <cmath>
#include <cstdio>

namespace dukatimer {

namespace {

const char* measurementLuxSourceLabel(MeasurementLuxSource source) {
	switch (source) {
	case MeasurementLuxSource::None: return "NONE";
	case MeasurementLuxSource::LocalTsl2561: return "TSL";
	case MeasurementLuxSource::WirelessGateway: return "WIRE";
	}

	return "UNK";
}

const char* measurementSessionModeLabel(MeasurementSessionMode mode) {
	switch (mode) {
	case MeasurementSessionMode::None: return "NONE";
	case MeasurementSessionMode::RelativeSpot: return "REL";
	case MeasurementSessionMode::PaperCalibration: return "CAL";
	case MeasurementSessionMode::ProposalPreview: return "PROP";
	}

	return "UNK";
}

const char* measurementCorrectionLabel(const MeasurementCorrectionStatus& correction) {
	switch (correction.state) {
	case MeasurementCorrectionState::NotRequired: return "RAW";
	case MeasurementCorrectionState::PendingCalibration:
		return correction.model == MeasurementCorrectionModel::DarkOffset ? "DARK?" : "PEND";
	case MeasurementCorrectionState::Configured:
		return correction.model == MeasurementCorrectionModel::DarkOffset ? "DARKSET" : "SET";
	case MeasurementCorrectionState::Applied:
		return correction.model == MeasurementCorrectionModel::DarkOffset ? "DARKON" : "ON";
	case MeasurementCorrectionState::Mixed: return "MIX";
	case MeasurementCorrectionState::Unknown:
	default: return "UNK";
	}
}

const char* measurementExplicitRoleLabel(MeasurementSampleRole role) {
	switch (role) {
	case MeasurementSampleRole::None: return "NONE";
	case MeasurementSampleRole::Shadow: return "SHAD";
	case MeasurementSampleRole::Highlight: return "HIGH";
	case MeasurementSampleRole::Midtone: return "MID";
	case MeasurementSampleRole::Dark: return "DARK";
	case MeasurementSampleRole::NoNegativeReference: return "NNREF";
	case MeasurementSampleRole::Calibration: return "CAL";
	case MeasurementSampleRole::PaperWhite: return "PWHITE";
	case MeasurementSampleRole::PaperBlack: return "PBLACK";
	case MeasurementSampleRole::Reference:
	case MeasurementSampleRole::RangeLow:
	case MeasurementSampleRole::RangeHigh:
	default: return "AUTO";
	}
}

bool usesIntegerLuxPrecision(MeasurementLuxSource source) {
	return source == MeasurementLuxSource::LocalTsl2561;
}

void formatLuxValue(char* output,
	               size_t outputCapacity,
	               MeasurementLuxSource source,
	               bool valid,
	               float lux,
	               const char* invalidText) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	if (!valid || !std::isfinite(lux) || lux < 0.0f) {
		std::snprintf(output, outputCapacity, "%s", invalidText);
		return;
	}

	// Die Quellenpraezision ist Teil der sichtbaren Messsemantik: lokaler
	// Kopf-TSL2561 bleibt ehrlich grob, der Wireless-TSL2591 darf seine
	// milli-lux-taugliche Papierpraezision sichtbar behalten.
	if (usesIntegerLuxPrecision(source)) {
		std::snprintf(output, outputCapacity, "%.0f lux", lux);
		return;
	}

	std::snprintf(output, outputCapacity, "%.3f lux", lux);
}

void formatRelativeEv(char* output, size_t outputCapacity, bool valid, float relativeEvStops) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	if (!valid || !std::isfinite(relativeEvStops)) {
		std::snprintf(output, outputCapacity, "--.--");
		return;
	}

	std::snprintf(output, outputCapacity, "%+.2f", relativeEvStops);
}

}  // namespace

void MeasurementValueFormatter::formatSplitgradeDoseTelemetry(char* output,
	                                                          size_t outputCapacity,
	                                                          float currentDose,
	                                                          float targetDose,
	                                                          float remainingTimeSeconds,
	                                                          float measuredLux) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	std::snprintf(output,
	              outputCapacity,
	              "DOSE %.2f / %.2f  REM %.1fs  LUX %.1f",
	              currentDose,
	              targetDose,
	              remainingTimeSeconds,
	              measuredLux);
}

void MeasurementValueFormatter::formatMeasurementSources(char* output,
	                                                    size_t outputCapacity,
	                                                    const MeasurementLuxSample& localLux,
	                                                    const MeasurementLuxSample& wirelessLux,
	                                                    MeasurementLuxSource activeSource) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	char localBuffer[24] = {};
	char wirelessBuffer[24] = {};
	formatLuxValue(localBuffer,
	             sizeof(localBuffer),
	             MeasurementLuxSource::LocalTsl2561,
	             localLux.valid,
	             localLux.lux,
	             "NO SAMPLE");
	formatLuxValue(wirelessBuffer,
	             sizeof(wirelessBuffer),
	             MeasurementLuxSource::WirelessGateway,
	             wirelessLux.valid,
	             wirelessLux.lux,
	             "NO SAMPLE");

	std::snprintf(output,
	              outputCapacity,
	              "LOCAL  %s\nWIRE   %s\nAKTIV  %s",
	              localBuffer,
	              wirelessBuffer,
	              measurementLuxSourceLabel(activeSource));
}

void MeasurementValueFormatter::formatMeasurementMain(char* output,
	                                                 size_t outputCapacity,
	                                                 const MeasurementLuxSample& activeLux) {
	formatLuxValue(output,
	             outputCapacity,
	             activeLux.source,
	             activeLux.valid,
	             activeLux.lux,
	             "NO SAMPLE");
}

void MeasurementValueFormatter::formatMeasurementMeta(char* output,
	                                                 size_t outputCapacity,
	                                                 const MeasurementLuxSample& activeLux) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	if (!activeLux.valid) {
		std::snprintf(output, outputCapacity, "NO SAMPLE");
		return;
	}

	std::snprintf(output,
	              outputCapacity,
	              "AGE %lums  SEQ #%lu  COR %s",
	              static_cast<unsigned long>(activeLux.ageMs),
	              static_cast<unsigned long>(activeLux.sequence),
	              measurementCorrectionLabel(activeLux.correction));
}

void MeasurementValueFormatter::formatMeasurementReference(char* output,
	                                                      size_t outputCapacity,
	                                                      const MeasurementReferenceStatus& activeReference,
	                                                      MeasurementSessionMode sessionMode,
	                                                      bool activeRelativeEvValid,
	                                                      float activeRelativeEvStops) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	char referenceBuffer[24] = {};
	char evBuffer[12] = {};
	formatLuxValue(referenceBuffer,
	             sizeof(referenceBuffer),
	             activeReference.source,
	             activeReference.valid,
	             activeReference.lux,
	             "NO REF");
	formatRelativeEv(evBuffer, sizeof(evBuffer), activeRelativeEvValid, activeRelativeEvStops);
	std::snprintf(output,
	              outputCapacity,
	              "REF %s  dEV %s  MOD %s",
	              referenceBuffer,
	              evBuffer,
	              measurementSessionModeLabel(sessionMode));
}

void MeasurementValueFormatter::formatMeasurementRange(char* output,
	                                                  size_t outputCapacity,
	                                                  const MeasurementSessionStatus& session) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	const char* sourceLabel = session.mixedSources
		? "MIX"
		: measurementLuxSourceLabel(session.commonSource);
	const char* correctionLabel = measurementCorrectionLabel(session.correction);
	const bool proposalReady = session.rangeUsableForProposal && session.correctionReadyForProposal;
	if (!session.rangeValid) {
		// `NO RNG` markiert explizit, dass noch keine belastbare Session-Range
		// vorliegt. Quelle, Korrekturstatus und Proposal-Sperre bleiben dabei
		// sichtbar, ohne eine falsche LO/HI-Nullsemantik vorzutäuschen.
		std::snprintf(output,
		              outputCapacity,
		              "NO RNG  SRC %s  COR %s  PROP %s",
		              sourceLabel,
		              correctionLabel,
		              proposalReady ? "OK" : "NO");
		return;
	}

	char shadowBuffer[12] = {};
	char highlightBuffer[12] = {};
	char spanBuffer[12] = {};
	formatRelativeEv(shadowBuffer,
	               sizeof(shadowBuffer),
	               session.rangeValid,
	               session.shadowSample.relativeEvStops);
	formatRelativeEv(highlightBuffer,
	               sizeof(highlightBuffer),
	               session.rangeValid,
	               session.highlightSample.relativeEvStops);
	if (session.rangeValid && std::isfinite(session.relativeEvSpanStops)) {
		std::snprintf(spanBuffer, sizeof(spanBuffer), "%.2f", session.relativeEvSpanStops);
	} else {
		std::snprintf(spanBuffer, sizeof(spanBuffer), "--.--");
	}
	// `LO/HI` benennt hier bewusst nur technische Range-Extrema. Die spaeteren
	// fotografischen Rollen Shadow/Highlight bleiben damit frei fuer einen
	// expliziten Workflow statt still aus blossen EV-Extrema abgeleitet zu werden.
	// `PROP` ist sichtbar erst dann frei, wenn sowohl Range- als auch Korrektur-
	// Voraussetzungen der Session gleichzeitig tragen.
	std::snprintf(output,
	              outputCapacity,
	              "LO %s  HI %s  SPAN %s  SRC %s  COR %s  PROP %s",
	              shadowBuffer,
	              highlightBuffer,
	              spanBuffer,
	              sourceLabel,
	              correctionLabel,
	              proposalReady ? "OK" : "NO");
}

void MeasurementValueFormatter::formatMeasurementControls(char* output,
	                                                     size_t outputCapacity,
	                                                     const MeasurementSessionStatus& session) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}

	std::snprintf(output,
	              outputCapacity,
	              "START SAMPLE  E2/C6 ROLE %s",
	              measurementExplicitRoleLabel(session.pendingCaptureRole));
}

void MeasurementValueFormatter::formatPageMeasLocalLux(char* output,
	                                                   size_t outputCapacity,
	                                                   const MeasurementLuxSample& localLux) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (localLux.valid) {
		std::snprintf(output, outputCapacity, "LOK %.2f lx", static_cast<double>(localLux.lux));
	} else {
		std::snprintf(output, outputCapacity, "LOK ---");
	}
}

void MeasurementValueFormatter::formatPageMeasWirelessLux(char* output,
	                                                      size_t outputCapacity,
	                                                      const MeasurementLuxSample& wirelessLux) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (wirelessLux.valid) {
		std::snprintf(output, outputCapacity, "FUNK %.2f lx", static_cast<double>(wirelessLux.lux));
	} else {
		std::snprintf(output, outputCapacity, "FUNK ---");
	}
}

const char* MeasurementValueFormatter::formatPageMeasSourceChip(MeasurementLuxSource source,
	                                                             bool valid) {
	if (!valid) {
		return "KEIN";
	}
	return (source == MeasurementLuxSource::WirelessGateway) ? "WIRELESS" : "LOCAL";
}

void MeasurementValueFormatter::formatPageMeasLuxMain(char* output,
	                                                  size_t outputCapacity,
	                                                  const MeasurementLuxSample& activeLux) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (activeLux.valid) {
		std::snprintf(output, outputCapacity, "%.2f lx", static_cast<double>(activeLux.lux));
	} else {
		std::snprintf(output, outputCapacity, "---");
	}
}

void MeasurementValueFormatter::formatPageMeasLuxAge(char* output,
	                                                 size_t outputCapacity,
	                                                 const MeasurementLuxSample& activeLux) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (activeLux.valid && activeLux.ageMs < 60000u) {
		std::snprintf(output, outputCapacity, "vor %.1f s",
		              static_cast<double>(activeLux.ageMs) / 1000.0);
	} else {
		std::snprintf(output, outputCapacity, "---");
	}
}

void MeasurementValueFormatter::formatPageMeasRefLux(char* output,
	                                                 size_t outputCapacity,
	                                                 const MeasurementReferenceStatus& activeReference) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (activeReference.valid) {
		std::snprintf(output, outputCapacity, "REF %.2f lx",
		              static_cast<double>(activeReference.lux));
	} else {
		std::snprintf(output, outputCapacity, "REF ---");
	}
}

void MeasurementValueFormatter::formatPageMeasEvDiff(char* output,
	                                                 size_t outputCapacity,
	                                                 bool activeRelativeEvValid,
	                                                 float activeRelativeEvStops) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (activeRelativeEvValid) {
		std::snprintf(output, outputCapacity, "%+.2f EV",
		              static_cast<double>(activeRelativeEvStops));
	} else {
		std::snprintf(output, outputCapacity, "--- EV");
	}
}

void MeasurementValueFormatter::formatPageMeasSession(char* output,
	                                                  size_t outputCapacity,
	                                                  const MeasurementSessionStatus& session) {
	if (output == nullptr || outputCapacity == 0u) {
		return;
	}
	if (session.sampleCount == 0u) {
		std::snprintf(output, outputCapacity, "Keine Proben  E1=STARTEN");
	} else {
		std::snprintf(output, outputCapacity, "n=%u%s%s",
		              static_cast<unsigned>(session.sampleCount),
		              session.correctionReadyForProposal ? "  KORR.BEREIT" : "",
		              session.canUndo ? "  E1-LONG=UNDO" : "");
	}
}

}  // namespace dukatimer
