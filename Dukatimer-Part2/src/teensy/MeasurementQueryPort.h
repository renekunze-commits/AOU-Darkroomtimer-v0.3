#pragma once

#include <array>
#include <stddef.h>
#include <stdint.h>

namespace dukatimer {

constexpr size_t kMeasurementHistogramBucketCount = 11;
constexpr uint8_t kMeasurementHistogramBucketStep = 20;
constexpr uint8_t kMeasurementHistogramBucketMax = 240;
constexpr size_t kMeasurementSessionRecentCapacity = 8;

using MeasurementZoneHistogram = std::array<uint8_t, kMeasurementHistogramBucketCount>;
using MeasurementSampleRoleMask = uint16_t;

enum class MeasurementLuxSource : uint8_t {
	None = 0,
	LocalTsl2561 = 1,
	WirelessGateway = 2,
};

enum class MeasurementSessionMode : uint8_t {
	None = 0,
	RelativeSpot = 1,
	PaperCalibration = 2,
	ProposalPreview = 3,
};

enum class MeasurementCorrectionModel : uint8_t {
	None = 0,
	DarkOffset = 1,
};

enum class MeasurementCorrectionState : uint8_t {
	Unknown = 0,
	NotRequired = 1,
	PendingCalibration = 2,
	Configured = 3,
	Applied = 4,
	Mixed = 5,
};

/*
 * Dark-/Offset-Korrektur bleibt damit als expliziter Vertragsbestandteil
 * sichtbar, auch solange die Domain noch keinen numerischen Abzug auf die
 * aktiven Luxwerte anwendet. So kann Proposal spaeter auf bekannten statt
 * still impliziten Korrekturannahmen aufbauen.
 */
struct MeasurementCorrectionStatus {
	MeasurementCorrectionModel model = MeasurementCorrectionModel::None;
	MeasurementCorrectionState state = MeasurementCorrectionState::Unknown;

	bool operator==(const MeasurementCorrectionStatus& other) const {
		return model == other.model && state == other.state;
	}

	bool operator!=(const MeasurementCorrectionStatus& other) const {
		return !(*this == other);
	}
};

enum class MeasurementSampleRole : MeasurementSampleRoleMask {
	None = 0u,
	Reference = 1u << 0u,
	RangeLow = 1u << 1u,
	RangeHigh = 1u << 2u,
	Shadow = 1u << 3u,
	Highlight = 1u << 4u,
	Midtone = 1u << 5u,
	Dark = 1u << 6u,
	NoNegativeReference = 1u << 7u,
	Calibration = 1u << 8u,
	PaperWhite = 1u << 9u,
	PaperBlack = 1u << 10u,
};

constexpr MeasurementSampleRoleMask measurementSampleRoleBit(MeasurementSampleRole role) {
	return static_cast<MeasurementSampleRoleMask>(role);
}

constexpr uint8_t kMeasurementSourceMaskNone = 0u;
constexpr uint8_t kMeasurementSourceMaskLocalTsl2561 = 1u << 0u;
constexpr uint8_t kMeasurementSourceMaskWirelessGateway = 1u << 1u;
constexpr MeasurementSampleRoleMask kMeasurementSampleRoleMaskNone =
	measurementSampleRoleBit(MeasurementSampleRole::None);
constexpr MeasurementSampleRoleMask kMeasurementSampleRoleMaskReference =
	measurementSampleRoleBit(MeasurementSampleRole::Reference);
constexpr MeasurementSampleRoleMask kMeasurementSampleRoleMaskRangeLow =
	measurementSampleRoleBit(MeasurementSampleRole::RangeLow);
constexpr MeasurementSampleRoleMask kMeasurementSampleRoleMaskRangeHigh =
	measurementSampleRoleBit(MeasurementSampleRole::RangeHigh);
constexpr MeasurementSampleRoleMask kMeasurementDerivedSampleRoleMask =
	kMeasurementSampleRoleMaskReference |
	kMeasurementSampleRoleMaskRangeLow |
	kMeasurementSampleRoleMaskRangeHigh;

constexpr bool measurementSampleRoleMaskHas(MeasurementSampleRoleMask roleMask,
	                                        MeasurementSampleRole role) {
	return (roleMask & measurementSampleRoleBit(role)) != 0u;
}

struct MeasurementLuxSample {
	MeasurementLuxSource source = MeasurementLuxSource::None;
	bool valid = false;
	float lux = 0.0f;
	uint32_t ageMs = 0;
	uint32_t sequence = 0;
	MeasurementCorrectionStatus correction = {};

	bool operator==(const MeasurementLuxSample& other) const {
		return source == other.source && valid == other.valid && lux == other.lux &&
		       ageMs == other.ageMs && sequence == other.sequence &&
		       correction == other.correction;
	}

	bool operator!=(const MeasurementLuxSample& other) const {
		return !(*this == other);
	}
};

/*
 * Expliziter Session-Anker fuer relative EV-/Zonenwerte.
 * Damit bleibt sichtbar, auf welchen Messpunkt sich eine Session-Auswertung
 * gerade bezieht, statt eine versteckte Global-Konstante zu unterstellen.
 */
struct MeasurementReferenceStatus {
	MeasurementLuxSource source = MeasurementLuxSource::None;
	bool valid = false;
	float lux = 0.0f;
	uint32_t sequence = 0;
	uint32_t capturedAtMs = 0;
	MeasurementCorrectionStatus correction = {};

	bool operator==(const MeasurementReferenceStatus& other) const {
		return source == other.source && valid == other.valid && lux == other.lux &&
		       sequence == other.sequence && capturedAtMs == other.capturedAtMs &&
		       correction == other.correction;
	}

	bool operator!=(const MeasurementReferenceStatus& other) const {
		return !(*this == other);
	}
};

struct MeasurementSessionSample {
	MeasurementLuxSource source = MeasurementLuxSource::None;
	float lux = 0.0f;
	uint32_t sequence = 0;
	uint32_t capturedAtMs = 0;
	float referenceLux = 0.0f;
	float relativeEvStops = 0.0f;
	uint8_t zoneIndex = 0;
	uint8_t histogramWeight = 0;
	MeasurementCorrectionStatus correction = {};
	// `roleMask` trennt technische Session-Anker von spaeteren fotografischen
	// Nutzerrollen. Ein Sample kann damit gleichzeitig Referenz und Range-Low
	// sein, ohne dass die Domain kuenstlich Prioritaeten erfinden muss.
	MeasurementSampleRoleMask roleMask = kMeasurementSampleRoleMaskNone;

	bool operator==(const MeasurementSessionSample& other) const {
		return source == other.source && lux == other.lux && sequence == other.sequence &&
		       capturedAtMs == other.capturedAtMs && referenceLux == other.referenceLux &&
		       relativeEvStops == other.relativeEvStops && zoneIndex == other.zoneIndex &&
		       histogramWeight == other.histogramWeight &&
		       correction == other.correction && roleMask == other.roleMask;
	}

	bool operator!=(const MeasurementSessionSample& other) const {
		return !(*this == other);
	}
};

struct MeasurementSessionStatus {
	MeasurementSessionMode mode = MeasurementSessionMode::None;
	// `sampleCount` benennt bewusst nur die aktuell sichtbare Session-History.
	// Lifetime-Captures und durch Overflow verworfene Samples werden getrennt
	// gehalten, damit UI und Proposal-Vorstufe keine gemischte Zaehlersemantik
	// lesen muessen.
	uint32_t sampleCount = 0;
	uint32_t capturedSampleCount = 0;
	uint32_t droppedSampleCount = 0;
	MeasurementSampleRole pendingCaptureRole = MeasurementSampleRole::None;
	MeasurementCorrectionStatus correction = {};
	bool correctionReadyForProposal = false;
	MeasurementSampleRoleMask roleMask = kMeasurementSampleRoleMaskNone;
	bool hasExplicitRoles = false;
	MeasurementSessionSample latestSample = {};
	bool rangeValid = false;
	bool rangeUsableForProposal = false;
	MeasurementSessionSample shadowSample = {};
	MeasurementSessionSample highlightSample = {};
	float relativeEvSpanStops = 0.0f;
	MeasurementZoneHistogram zoneHistogram = {};
	std::array<MeasurementSessionSample, kMeasurementSessionRecentCapacity> recentSamples = {};
	uint8_t recentSampleCount = 0;
	uint8_t undoDepth = 0;
	bool canUndo = false;
	uint8_t sourceMask = kMeasurementSourceMaskNone;
	uint8_t sourceCount = 0u;
	MeasurementLuxSource commonSource = MeasurementLuxSource::None;
	bool mixedSources = false;

	bool operator==(const MeasurementSessionStatus& other) const {
		return mode == other.mode && sampleCount == other.sampleCount &&
		       capturedSampleCount == other.capturedSampleCount &&
		       droppedSampleCount == other.droppedSampleCount &&
		       pendingCaptureRole == other.pendingCaptureRole &&
		       correction == other.correction &&
		       correctionReadyForProposal == other.correctionReadyForProposal &&
		       roleMask == other.roleMask && hasExplicitRoles == other.hasExplicitRoles &&
		       latestSample == other.latestSample && rangeValid == other.rangeValid &&
		       rangeUsableForProposal == other.rangeUsableForProposal &&
		       shadowSample == other.shadowSample &&
		       highlightSample == other.highlightSample &&
		       relativeEvSpanStops == other.relativeEvSpanStops &&
		       zoneHistogram == other.zoneHistogram &&
		       recentSamples == other.recentSamples && recentSampleCount == other.recentSampleCount &&
		       undoDepth == other.undoDepth && canUndo == other.canUndo &&
		       sourceMask == other.sourceMask && sourceCount == other.sourceCount &&
		       commonSource == other.commonSource && mixedSources == other.mixedSources;
	}

	bool operator!=(const MeasurementSessionStatus& other) const {
		return !(*this == other);
	}
};

/*
 * MeasurementQueryPort
 *
 * Fester Lesepfad fuer Messdaten und Messstatus in Workflows.
 * Die Measurement-Domain haelt dabei die Herkunft (lokal/wireless) und
 * Sample-Age konsistent, statt dass jeder Workflow eigene Heuristiken baut.
 */
class MeasurementQueryPort {
public:
	virtual ~MeasurementQueryPort() = default;

	virtual MeasurementLuxSample activeLuxSample() const = 0;
	virtual MeasurementLuxSample localLuxSample() const = 0;
	virtual MeasurementLuxSample wirelessLuxSample() const = 0;
	virtual const MeasurementSessionStatus& sessionStatus() const = 0;
};

}  // namespace dukatimer