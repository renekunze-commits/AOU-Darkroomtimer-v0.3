#include "MeasurementDomainService.h"

#include <Arduino.h>

#include <cmath>

#include <DukatimerProtocol.h>

#include "ExposureValueMath.h"

namespace dukatimer {

namespace {

constexpr long kMeasurementHistogramCenterZone = static_cast<long>(kMeasurementHistogramBucketCount / 2u);

uint8_t sourceMaskForMeasurementSource(MeasurementLuxSource source) {
	switch (source) {
	case MeasurementLuxSource::LocalTsl2561: return kMeasurementSourceMaskLocalTsl2561;
	case MeasurementLuxSource::WirelessGateway: return kMeasurementSourceMaskWirelessGateway;
	case MeasurementLuxSource::None:
	default: return kMeasurementSourceMaskNone;
	}
}

uint8_t countMeasurementSources(uint8_t sourceMask) {
	uint8_t sourceCount = 0u;
	if ((sourceMask & kMeasurementSourceMaskLocalTsl2561) != 0u) {
		++sourceCount;
	}
	if ((sourceMask & kMeasurementSourceMaskWirelessGateway) != 0u) {
		++sourceCount;
	}
	return sourceCount;
}

MeasurementLuxSource commonMeasurementSource(uint8_t sourceMask) {
	switch (sourceMask) {
	case kMeasurementSourceMaskLocalTsl2561: return MeasurementLuxSource::LocalTsl2561;
	case kMeasurementSourceMaskWirelessGateway: return MeasurementLuxSource::WirelessGateway;
	case kMeasurementSourceMaskNone:
	default: return MeasurementLuxSource::None;
	}
}

MeasurementSampleRoleMask explicitMeasurementSampleRoles(MeasurementSampleRoleMask roleMask) {
	return static_cast<MeasurementSampleRoleMask>(roleMask & ~kMeasurementDerivedSampleRoleMask);
}

void clearDerivedMeasurementSampleRoles(MeasurementSessionSample& sample) {
	sample.roleMask =
		static_cast<MeasurementSampleRoleMask>(sample.roleMask & ~kMeasurementDerivedSampleRoleMask);
}

MeasurementCorrectionStatus correctionStatusForMeasurementSource(MeasurementLuxSource source) {
	switch (source) {
	case MeasurementLuxSource::LocalTsl2561:
		// Der lokale TSL2561 ist aktuell Kopf-/Closed-Loop-Sensor. Fuer diesen
		// Runtime-Pfad ist Raw-Lux die beabsichtigte Semantik, nicht ein spaeter
		// noch fehlender Papier-/Dunkeloffset-Abzug.
		return {MeasurementCorrectionModel::None, MeasurementCorrectionState::NotRequired};
	case MeasurementLuxSource::WirelessGateway:
		// Der Wireless-TSL2591 sitzt im Papier-/Spotpfad. Solange der aktive
		// Part2-Workflow keinen dokumentierten Dunkelabgleich transportiert,
		// bleibt dieser Pfad fachlich korrekt als Dark-Offset-pending markiert.
		return {MeasurementCorrectionModel::DarkOffset,
		        MeasurementCorrectionState::PendingCalibration};
	case MeasurementLuxSource::None:
	default: return {};
	}
}

bool correctionStatusReadyForProposal(MeasurementCorrectionState state) {
	return state == MeasurementCorrectionState::NotRequired ||
	       state == MeasurementCorrectionState::Applied;
}

bool isPaperCalibrationRole(MeasurementSampleRole role) {
	switch (role) {
	case MeasurementSampleRole::Dark:
	case MeasurementSampleRole::NoNegativeReference:
	case MeasurementSampleRole::Calibration:
	case MeasurementSampleRole::PaperWhite:
	case MeasurementSampleRole::PaperBlack: return true;
	case MeasurementSampleRole::None:
	case MeasurementSampleRole::Reference:
	case MeasurementSampleRole::RangeLow:
	case MeasurementSampleRole::RangeHigh:
	case MeasurementSampleRole::Shadow:
	case MeasurementSampleRole::Highlight:
	case MeasurementSampleRole::Midtone:
	default: return false;
	}
}

MeasurementSessionMode sessionModeForPendingCaptureRole(MeasurementSampleRole role) {
	return isPaperCalibrationRole(role) ? MeasurementSessionMode::PaperCalibration
	                                   : MeasurementSessionMode::RelativeSpot;
}

MeasurementSessionMode sessionModeForRoleMask(MeasurementSampleRoleMask roleMask) {
	return measurementSampleRoleMaskHas(roleMask, MeasurementSampleRole::Dark) ||
	               measurementSampleRoleMaskHas(roleMask, MeasurementSampleRole::NoNegativeReference) ||
	               measurementSampleRoleMaskHas(roleMask, MeasurementSampleRole::Calibration) ||
	               measurementSampleRoleMaskHas(roleMask, MeasurementSampleRole::PaperWhite) ||
	               measurementSampleRoleMaskHas(roleMask, MeasurementSampleRole::PaperBlack)
	           ? MeasurementSessionMode::PaperCalibration
	           : MeasurementSessionMode::RelativeSpot;
}

MeasurementSampleRole cycleSelectableMeasurementRole(MeasurementSampleRole role, int8_t direction) {
	constexpr MeasurementSampleRole kSelectableRoles[] = {
		MeasurementSampleRole::None,
		MeasurementSampleRole::Shadow,
		MeasurementSampleRole::Highlight,
		MeasurementSampleRole::Midtone,
		MeasurementSampleRole::Dark,
		MeasurementSampleRole::NoNegativeReference,
		MeasurementSampleRole::Calibration,
		MeasurementSampleRole::PaperWhite,
		MeasurementSampleRole::PaperBlack,
	};

	if (direction == 0) {
		return role;
	}

	int currentIndex = 0;
	for (size_t index = 0; index < (sizeof(kSelectableRoles) / sizeof(kSelectableRoles[0])); ++index) {
		if (kSelectableRoles[index] == role) {
			currentIndex = static_cast<int>(index);
			break;
		}
	}

	currentIndex += (direction > 0) ? 1 : -1;
	if (currentIndex < 0) {
		currentIndex = static_cast<int>((sizeof(kSelectableRoles) / sizeof(kSelectableRoles[0])) - 1u);
	} else if (currentIndex >= static_cast<int>(sizeof(kSelectableRoles) / sizeof(kSelectableRoles[0]))) {
		currentIndex = 0;
	}

	return kSelectableRoles[currentIndex];
}

}  // namespace

void MeasurementDomainService::begin(uint32_t nowMs) {
	if (initialized_) {
		return;
	}

	if (nowMs == 0) {
		nowMs = millis();
	}

	state_ = makeUnknownMeasurementRuntimeStatus();
	state_.activeLux.ageMs = 0;
	state_.localLux.ageMs = 0;
	state_.wirelessLux.ageMs = 0;
	state_.session = {};
	localSampleTimestampMs_ = nowMs;
	wirelessSampleTimestampMs_ = nowMs;
	lastWirelessHandledSequence_ = 0;
	wirelessUndoBarrierSequence_ = 0;
	pendingCaptureRole_ = MeasurementSampleRole::None;
	sessionUndoHistory_ = {};
	sessionUndoHistoryCount_ = 0;
	clearSessionReferences();
	wirelessPeerState_ = protocol::WirelessPeerState::Unknown;
	linkHealth_ = EspLinkHealth::Unknown;
	wirelessCaptureArmed_ = false;
	initialized_ = true;
}

void MeasurementDomainService::tick(uint32_t nowMs) {
	if (!initialized_) {
		return;
	}

	refreshSampleAges(nowMs);
	chooseActiveSample();
}

void MeasurementDomainService::setWirelessCaptureArmed(bool armed) {
	// Wireless-Measure darf nur dann still in die Session laufen, wenn der
	// sichtbare Workflow tatsaechlich im Measurement-Kontext steht. Der Guard
	// liegt hier im Domain-Dienst, damit Gateway-Sequenzen ausserhalb dieses
	// Kontexts nicht spaeter nachtraeglich in die Session kippen.
	wirelessCaptureArmed_ = armed;
}

void MeasurementDomainService::observeLocalSensor(const SensorRuntimeStatus& sensorStatus, uint32_t nowMs) {
	if (!initialized_) {
		return;
	}

	if (nowMs == 0) {
		nowMs = millis();
	}

	state_.localLux.source = MeasurementLuxSource::LocalTsl2561;
	state_.localLux.valid = false;
	state_.localLux.lux = 0.0f;
	state_.localLux.sequence = 0;
	state_.localLux.correction =
		correctionStatusForMeasurementSource(MeasurementLuxSource::LocalTsl2561);

	const Tsl2561RuntimeStatus& tsl = sensorStatus.tsl2561;
	if (!tsl.initialized || tsl.health != SensorHealth::Ok ||
	    tsl.sampleValidity != LuxSampleValidity::Valid) {
		return;
	}

	localSampleTimestampMs_ = sanitizeTimestamp(nowMs, tsl.sampleAgeMs);
	state_.localLux.valid = true;
	state_.localLux.lux = tsl.lux;
	state_.localLux.sequence = 0;
}

void MeasurementDomainService::observeWirelessGateway(const WirelessGatewayStatus& wirelessStatus,
	                                                  EspLinkHealth linkHealth,
	                                                  uint32_t nowMs) {
	if (!initialized_) {
		return;
	}

	if (nowMs == 0) {
		nowMs = millis();
	}

	linkHealth_ = linkHealth;
	wirelessPeerState_ = wirelessStatus.peerState;
	state_.wirelessLux.source = MeasurementLuxSource::WirelessGateway;
	state_.wirelessLux.valid = false;
	state_.wirelessLux.lux = 0.0f;
	state_.wirelessLux.sequence = wirelessStatus.measurementSequence;
	state_.wirelessLux.correction =
		correctionStatusForMeasurementSource(MeasurementLuxSource::WirelessGateway);

	if ((wirelessStatus.flags & protocol::kWirelessSnapshotLuxValid) == 0u) {
		return;
	}

	wirelessSampleTimestampMs_ = sanitizeTimestamp(nowMs, wirelessStatus.sensorSampleAgeMs);
	state_.wirelessLux.valid = true;
	state_.wirelessLux.lux = wirelessStatus.lastLux;
	recordWirelessMeasurementSample(nowMs);
}

MeasurementLuxSample MeasurementDomainService::activeLuxSample() const {
	return state_.activeLux;
}

MeasurementLuxSample MeasurementDomainService::localLuxSample() const {
	return state_.localLux;
}

MeasurementLuxSample MeasurementDomainService::wirelessLuxSample() const {
	return state_.wirelessLux;
}

const MeasurementSessionStatus& MeasurementDomainService::sessionStatus() const {
	return state_.session;
}

bool MeasurementDomainService::captureLocalSample(uint32_t nowMs) {
	if (!initialized_ || !isLocalSampleUsable()) {
		return false;
	}

	if (nowMs == 0) {
		nowMs = millis();
	}

	return appendSessionSample(state_.localLux, nowMs);
}

MeasurementSampleRole MeasurementDomainService::cyclePendingCaptureRole(int8_t direction) {
	if (!initialized_) {
		return MeasurementSampleRole::None;
	}

	pendingCaptureRole_ = cycleSelectableMeasurementRole(pendingCaptureRole_, direction);
	syncSessionStatusFromHistory();
	return pendingCaptureRole_;
}

bool MeasurementDomainService::undoLastSessionSample() {
	if (!initialized_ || sessionUndoHistoryCount_ == 0u) {
		return false;
	}

	const MeasurementSessionSample& sample = sessionUndoHistory_[sessionUndoHistoryCount_ - 1u];
	if (sample.source == MeasurementLuxSource::WirelessGateway) {
		const uint32_t visibleSequence = state_.wirelessLux.valid ? state_.wirelessLux.sequence : 0u;
		const uint32_t latestBlockedSequence = visibleSequence > sample.sequence ? visibleSequence : sample.sequence;
		if (isSequenceNewer(latestBlockedSequence, wirelessUndoBarrierSequence_)) {
			wirelessUndoBarrierSequence_ = latestBlockedSequence;
		}
	}
	applyHistogramDelta(sample.zoneIndex, -static_cast<int>(sample.histogramWeight));
	sessionUndoHistory_[sessionUndoHistoryCount_ - 1u] = {};
	--sessionUndoHistoryCount_;
	syncSessionStatusFromHistory();
	return true;
}

void MeasurementDomainService::resetSession() {
	state_.session = {};
	sessionUndoHistory_ = {};
	sessionUndoHistoryCount_ = 0u;
	pendingCaptureRole_ = MeasurementSampleRole::None;
	lastWirelessHandledSequence_ = state_.wirelessLux.valid ? state_.wirelessLux.sequence : 0u;
	wirelessUndoBarrierSequence_ = 0u;
	clearSessionReferences();
}

const MeasurementRuntimeStatus& MeasurementDomainService::state() const {
	return state_;
}

uint32_t MeasurementDomainService::sanitizeTimestamp(uint32_t nowMs, uint32_t ageMs) const {
	if (ageMs >= nowMs) {
		return 0;
	}

	return nowMs - ageMs;
}

bool MeasurementDomainService::isSequenceNewer(uint32_t sequence, uint32_t reference) {
	if (sequence == 0u) {
		return false;
	}

	return static_cast<int32_t>(sequence - reference) > 0;
}

void MeasurementDomainService::refreshSampleAges(uint32_t nowMs) {
	if (state_.localLux.valid) {
		state_.localLux.ageMs = nowMs - localSampleTimestampMs_;
	} else {
		state_.localLux.ageMs = 0;
	}

	if (state_.wirelessLux.valid) {
		state_.wirelessLux.ageMs = nowMs - wirelessSampleTimestampMs_;
	} else {
		state_.wirelessLux.ageMs = 0;
	}
}

void MeasurementDomainService::recordWirelessMeasurementSample(uint32_t nowMs) {
	if (!state_.wirelessLux.valid ||
	    !isSequenceNewer(state_.wirelessLux.sequence, lastWirelessHandledSequence_)) {
		return;
	}

	// Jede sichtbare Wireless-Measure-Sequenz wird genau einmal verbraucht,
	// auch wenn der aktuelle Workflow-Kontext die Sessionaenderung ablehnt.
	// Dadurch kann ein ausserhalb des Messpanels gedrueckter C6-Measure-Button
	// spaeter beim Betreten des Messpanels nicht nachtraeglich in die Session
	// einsickern.
	lastWirelessHandledSequence_ = state_.wirelessLux.sequence;

	if (!wirelessCaptureArmed_) {
		return;
	}

	if (wirelessUndoBarrierSequence_ != 0u &&
	    !isSequenceNewer(state_.wirelessLux.sequence, wirelessUndoBarrierSequence_)) {
		return;
	}

	if (appendSessionSample(state_.wirelessLux, nowMs)) {
		if (wirelessUndoBarrierSequence_ != 0u &&
		    isSequenceNewer(state_.wirelessLux.sequence, wirelessUndoBarrierSequence_)) {
			wirelessUndoBarrierSequence_ = 0u;
		}
	}
}

bool MeasurementDomainService::appendSessionSample(const MeasurementLuxSample& sample, uint32_t nowMs) {
	if (!sample.valid || sample.lux <= 0.0f || !std::isfinite(sample.lux)) {
		return false;
	}

	if (sessionUndoHistoryCount_ == sessionUndoHistory_.size()) {
		const MeasurementSessionSample droppedSample = sessionUndoHistory_[0u];
		applyHistogramDelta(droppedSample.zoneIndex, -static_cast<int>(droppedSample.histogramWeight));
		for (size_t index = 1u; index < sessionUndoHistory_.size(); ++index) {
			sessionUndoHistory_[index - 1u] = sessionUndoHistory_[index];
		}
		sessionUndoHistoryCount_ = static_cast<uint8_t>(sessionUndoHistory_.size() - 1u);
		state_.session.droppedSampleCount += 1u;
		rebuildSessionReferencesFromHistory();
	}

	const MeasurementReferenceStatus reference = establishSessionReference(sample, nowMs);
	if (!reference.valid) {
		return false;
	}

	MeasurementSessionSample sessionSample = buildSessionSample(sample, reference, nowMs);

	sessionUndoHistory_[sessionUndoHistoryCount_] = sessionSample;
	++sessionUndoHistoryCount_;
	state_.session.capturedSampleCount += 1u;
	applyHistogramDelta(sessionSample.zoneIndex, static_cast<int>(sessionSample.histogramWeight));
	syncSessionStatusFromHistory();
	return true;
}

MeasurementSessionSample MeasurementDomainService::buildSessionSample(const MeasurementLuxSample& sample,
	                                                                 const MeasurementReferenceStatus& reference,
	                                                                 uint32_t nowMs) const {
	MeasurementSessionSample sessionSample = {};
	sessionSample.source = sample.source;
	sessionSample.lux = sample.lux;
	sessionSample.sequence = sample.sequence;
	sessionSample.capturedAtMs = nowMs;
	sessionSample.referenceLux = reference.lux;
	sessionSample.relativeEvStops = ExposureValueMath::relativeEvFromLux(sample.lux, reference.lux);
	sessionSample.zoneIndex = zoneIndexFromRelativeEv(sessionSample.relativeEvStops);
	sessionSample.correction = sample.correction;
	sessionSample.roleMask = measurementSampleRoleBit(pendingCaptureRole_);
	const uint8_t currentBucket = state_.session.zoneHistogram[sessionSample.zoneIndex];
	if (currentBucket >= kMeasurementHistogramBucketMax) {
		sessionSample.histogramWeight = 0u;
	} else if ((currentBucket + kMeasurementHistogramBucketStep) > kMeasurementHistogramBucketMax) {
		sessionSample.histogramWeight =
			static_cast<uint8_t>(kMeasurementHistogramBucketMax - currentBucket);
	} else {
		sessionSample.histogramWeight = kMeasurementHistogramBucketStep;
	}
	return sessionSample;
}

MeasurementReferenceStatus MeasurementDomainService::establishSessionReference(const MeasurementLuxSample& sample,
	                                                                          uint32_t nowMs) {
	MeasurementReferenceStatus* reference = sessionReferenceForSource(sample.source);
	if (reference == nullptr || !sample.valid || sample.lux <= 0.0f || !std::isfinite(sample.lux)) {
		return {};
	}

	if (!reference->valid) {
		// Die erste gueltige Messung pro Quelle dient bewusst als bodenstaendiger
		// Session-Anker. Das beseitigt den versteckten 1-Lux-Nullpunkt, ohne schon
		// eine groessere Papier-/Hardware-Kalibrierarchitektur vorwegzunehmen.
		reference->source = sample.source;
		reference->valid = true;
		reference->lux = sample.lux;
		reference->sequence = sample.sequence;
		reference->capturedAtMs = nowMs;
		reference->correction = sample.correction;
	}

	return *reference;
}

MeasurementReferenceStatus* MeasurementDomainService::sessionReferenceForSource(MeasurementLuxSource source) {
	switch (source) {
	case MeasurementLuxSource::LocalTsl2561: return &localSessionReference_;
	case MeasurementLuxSource::WirelessGateway: return &wirelessSessionReference_;
	case MeasurementLuxSource::None:
	default: return nullptr;
	}
}

const MeasurementReferenceStatus* MeasurementDomainService::sessionReferenceForSource(MeasurementLuxSource source) const {
	switch (source) {
	case MeasurementLuxSource::LocalTsl2561: return &localSessionReference_;
	case MeasurementLuxSource::WirelessGateway: return &wirelessSessionReference_;
	case MeasurementLuxSource::None:
	default: return nullptr;
	}
}

void MeasurementDomainService::clearSessionReferences() {
	localSessionReference_ = {};
	wirelessSessionReference_ = {};
	state_.activeReference = {};
	state_.activeRelativeEvValid = false;
	state_.activeRelativeEvStops = 0.0f;
}

void MeasurementDomainService::rebuildSessionReferencesFromHistory() {
	localSessionReference_ = {};
	wirelessSessionReference_ = {};

	for (size_t index = 0u; index < sessionUndoHistoryCount_; ++index) {
		const MeasurementSessionSample& sample = sessionUndoHistory_[index];
		MeasurementReferenceStatus* reference = sessionReferenceForSource(sample.source);
		if (reference == nullptr || reference->valid) {
			continue;
		}

		// Die sichtbare History bleibt der einzige gueltige Ursprung der
		// Session-Referenzen. Nach Undo oder Overflow darf keine bereits
		// herausgefallene Probe als stiller Anker im Hintergrund weiterleben.
		reference->source = sample.source;
		reference->valid = true;
		reference->lux = sample.lux;
		reference->sequence = sample.sequence;
		reference->capturedAtMs = sample.capturedAtMs;
		reference->correction = sample.correction;
	}
}

void MeasurementDomainService::refreshActiveReference() {
	state_.activeReference = {};
	state_.activeRelativeEvValid = false;
	state_.activeRelativeEvStops = 0.0f;

	if (!state_.activeLux.valid) {
		return;
	}

	const MeasurementReferenceStatus* reference = sessionReferenceForSource(state_.activeLux.source);
	if (reference == nullptr || !reference->valid) {
		return;
	}

	state_.activeReference = *reference;
	const float relativeEvStops = ExposureValueMath::relativeEvFromLux(state_.activeLux.lux, reference->lux);
	if (!std::isfinite(relativeEvStops)) {
		return;
	}

	state_.activeRelativeEvValid = true;
	state_.activeRelativeEvStops = relativeEvStops;
}

uint8_t MeasurementDomainService::zoneIndexFromRelativeEv(float relativeEvStops) const {
	// Das Histogramm ist jetzt explizit session-relativ: die Mitte bedeutet den
	// aufgenommenen Referenzpunkt der jeweiligen Quelle, nicht mehr einen festen
	// globalen Lux-Wert. Damit bleiben Spot-Abstaende nachvollziehbar, ohne eine
	// nicht belegte absolute Papierzone vorzutäuschen.
	if (!std::isfinite(relativeEvStops)) {
		return 0u;
	}

	long zoneIndex = std::lround(relativeEvStops) + kMeasurementHistogramCenterZone;
	if (zoneIndex < 0l) {
		return 0u;
	}
	if (zoneIndex >= static_cast<long>(kMeasurementHistogramBucketCount)) {
		return static_cast<uint8_t>(kMeasurementHistogramBucketCount - 1u);
	}
	return static_cast<uint8_t>(zoneIndex);
}

void MeasurementDomainService::applyHistogramDelta(uint8_t zoneIndex, int delta) {
	if (zoneIndex >= state_.session.zoneHistogram.size() || delta == 0) {
		return;
	}

	const int currentValue = static_cast<int>(state_.session.zoneHistogram[zoneIndex]);
	int nextValue = currentValue + delta;
	if (nextValue < 0) {
		nextValue = 0;
	}
	if (nextValue > static_cast<int>(kMeasurementHistogramBucketMax)) {
		nextValue = static_cast<int>(kMeasurementHistogramBucketMax);
	}
	state_.session.zoneHistogram[zoneIndex] = static_cast<uint8_t>(nextValue);
}

void MeasurementDomainService::syncSessionStatusFromHistory() {
	state_.session.mode = MeasurementSessionMode::None;
	state_.session.sampleCount = 0u;
	state_.session.latestSample = {};
	state_.session.pendingCaptureRole = pendingCaptureRole_;
	state_.session.correction = {};
	state_.session.correctionReadyForProposal = false;
	state_.session.roleMask = kMeasurementSampleRoleMaskNone;
	state_.session.hasExplicitRoles = false;
	state_.session.rangeValid = false;
	state_.session.rangeUsableForProposal = false;
	state_.session.shadowSample = {};
	state_.session.highlightSample = {};
	state_.session.relativeEvSpanStops = 0.0f;
	state_.session.recentSamples = {};
	state_.session.recentSampleCount = 0u;
	state_.session.undoDepth = sessionUndoHistoryCount_;
	state_.session.canUndo = sessionUndoHistoryCount_ > 0u;
	state_.session.sourceMask = kMeasurementSourceMaskNone;
	state_.session.sourceCount = 0u;
	state_.session.commonSource = MeasurementLuxSource::None;
	state_.session.mixedSources = false;

	if (sessionUndoHistoryCount_ == 0u) {
		state_.session.mode = pendingCaptureRole_ == MeasurementSampleRole::None
			? MeasurementSessionMode::None
			: sessionModeForPendingCaptureRole(pendingCaptureRole_);
		clearSessionReferences();
		return;
	}

	rebuildSessionReferencesFromHistory();
	state_.session.mode = sessionModeForPendingCaptureRole(pendingCaptureRole_);

	state_.session.sampleCount = static_cast<uint32_t>(sessionUndoHistoryCount_);
	bool localReferenceAssigned = false;
	bool wirelessReferenceAssigned = false;
	size_t shadowIndex = 0u;
	size_t highlightIndex = 0u;
	uint8_t sourceMask = kMeasurementSourceMaskNone;
	MeasurementCorrectionStatus correctionSummary = {};
	bool correctionSummaryAssigned = false;
	bool correctionMixed = false;
	for (size_t index = 0u; index < sessionUndoHistoryCount_; ++index) {
		MeasurementSessionSample& sample = sessionUndoHistory_[index];
		clearDerivedMeasurementSampleRoles(sample);
		if (!correctionSummaryAssigned) {
			correctionSummary = sample.correction;
			correctionSummaryAssigned = true;
		} else if (sample.correction != correctionSummary) {
			correctionMixed = true;
		}
		if (sample.source == MeasurementLuxSource::LocalTsl2561 && !localReferenceAssigned) {
			sample.roleMask = static_cast<MeasurementSampleRoleMask>(
				sample.roleMask | kMeasurementSampleRoleMaskReference);
			localReferenceAssigned = true;
		} else if (sample.source == MeasurementLuxSource::WirelessGateway &&
		           !wirelessReferenceAssigned) {
			sample.roleMask = static_cast<MeasurementSampleRoleMask>(
				sample.roleMask | kMeasurementSampleRoleMaskReference);
			wirelessReferenceAssigned = true;
		}
		sourceMask |= sourceMaskForMeasurementSource(sample.source);
		if (sample.relativeEvStops < sessionUndoHistory_[shadowIndex].relativeEvStops) {
			shadowIndex = index;
		}
		if (sample.relativeEvStops > sessionUndoHistory_[highlightIndex].relativeEvStops) {
			highlightIndex = index;
		}
	}
	// Rollen fuer Referenz und Range-Extrema werden jedes Mal aus der sichtbaren
	// History neu aufgebaut. Dadurch koennen Undo, Reset und Overflow keine
	// semantisch stale Anker im Sessionvertrag hinterlassen.
	sessionUndoHistory_[shadowIndex].roleMask = static_cast<MeasurementSampleRoleMask>(
		sessionUndoHistory_[shadowIndex].roleMask | kMeasurementSampleRoleMaskRangeLow);
	sessionUndoHistory_[highlightIndex].roleMask = static_cast<MeasurementSampleRoleMask>(
		sessionUndoHistory_[highlightIndex].roleMask | kMeasurementSampleRoleMaskRangeHigh);
	if (correctionMixed) {
		correctionSummary.model = MeasurementCorrectionModel::None;
		correctionSummary.state = MeasurementCorrectionState::Mixed;
	}
	MeasurementSampleRoleMask sessionRoleMask = kMeasurementSampleRoleMaskNone;
	for (size_t index = 0u; index < sessionUndoHistoryCount_; ++index) {
		sessionRoleMask = static_cast<MeasurementSampleRoleMask>(
			sessionRoleMask | sessionUndoHistory_[index].roleMask);
	}
	state_.session.latestSample = sessionUndoHistory_[sessionUndoHistoryCount_ - 1u];
	MeasurementSessionSample shadowSample = sessionUndoHistory_[shadowIndex];
	MeasurementSessionSample highlightSample = sessionUndoHistory_[highlightIndex];
	// Schatten/Lichter bleiben bewusst im bereits eingefuehrten
	// session-relativen EV-Raum. Dadurch entstehen keine neuen absoluten
	// Papierzonen-Claims, aber spaetere Vorschlags- und Messscreens koennen die
	// aktuell aufgespannte Spreizung sichtbar und konsistent nutzen.
	state_.session.correction = correctionSummary;
	// Die Proposal-Vorstufe braucht nicht nur Range-Konsistenz, sondern auch
	// einen bekannten Korrekturstatus. Lokaler Kopf-Raw-Pfad ist bewusst sofort
	// zulaessig, der Wireless-Papierpfad bleibt bis zu einem echten Dark-Offset-
	// Workflow explizit gesperrt statt still raw-vertrauenswuerdig zu wirken.
	state_.session.correctionReadyForProposal =
		!correctionMixed && !state_.session.mixedSources &&
		correctionStatusReadyForProposal(correctionSummary.state);
	state_.session.roleMask = sessionRoleMask;
	state_.session.hasExplicitRoles =
		explicitMeasurementSampleRoles(sessionRoleMask) != kMeasurementSampleRoleMaskNone;
	if (state_.session.hasExplicitRoles || pendingCaptureRole_ != MeasurementSampleRole::None) {
		state_.session.mode = sessionModeForRoleMask(static_cast<MeasurementSampleRoleMask>(
			sessionRoleMask | measurementSampleRoleBit(pendingCaptureRole_)));
	}
	state_.session.rangeValid = true;
	state_.session.shadowSample = shadowSample;
	state_.session.highlightSample = highlightSample;
	state_.session.relativeEvSpanStops =
		highlightSample.relativeEvStops - shadowSample.relativeEvStops;
	state_.session.sourceMask = sourceMask;
	state_.session.sourceCount = countMeasurementSources(sourceMask);
	state_.session.commonSource = commonMeasurementSource(sourceMask);
	state_.session.mixedSources = state_.session.sourceCount > 1u;
	// `rangeValid` bleibt die technische Extrema-Sicht auf die aktuelle History.
	// Fuer Proposal darf dieselbe Range erst benutzt werden, wenn mindestens zwei
	// sichtbare Samples vorliegen und die Session auf genau einer Quelle basiert.
	state_.session.rangeUsableForProposal =
		sessionUndoHistoryCount_ >= 2u && !state_.session.mixedSources &&
		state_.session.commonSource != MeasurementLuxSource::None;
	const size_t recentCount =
		sessionUndoHistoryCount_ < kMeasurementSessionRecentCapacity
			? sessionUndoHistoryCount_
			: kMeasurementSessionRecentCapacity;
	const size_t historyStart = sessionUndoHistoryCount_ - recentCount;
	for (size_t index = 0u; index < recentCount; ++index) {
		state_.session.recentSamples[index] = sessionUndoHistory_[historyStart + index];
	}
	state_.session.recentSampleCount = static_cast<uint8_t>(recentCount);
	refreshActiveReference();
}

bool MeasurementDomainService::isLocalSampleUsable() const {
	return state_.localLux.valid && state_.localLux.ageMs <= kLocalLuxStaleMs;
}

bool MeasurementDomainService::isWirelessSampleUsable() const {
	if (!state_.wirelessLux.valid || state_.wirelessLux.ageMs > kWirelessLuxStaleMs) {
		return false;
	}

	if (wirelessPeerState_ != protocol::WirelessPeerState::Online &&
	    wirelessPeerState_ != protocol::WirelessPeerState::Measuring) {
		return false;
	}

	return linkHealth_ != EspLinkHealth::Lost;
}

void MeasurementDomainService::chooseActiveSample() {
	state_.activeLux = {};

	if (isWirelessSampleUsable()) {
		state_.activeLux = state_.wirelessLux;
		refreshActiveReference();
		return;
	}

	if (isLocalSampleUsable()) {
		state_.activeLux = state_.localLux;
		refreshActiveReference();
		return;
	}

	state_.activeLux.source = MeasurementLuxSource::None;
	state_.activeLux.valid = false;
	state_.activeLux.lux = 0.0f;
	state_.activeLux.ageMs = 0;
	state_.activeLux.sequence = 0;
	refreshActiveReference();
}

}  // namespace dukatimer
