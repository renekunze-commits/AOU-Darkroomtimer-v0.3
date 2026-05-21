#pragma once

#include <array>
#include <stdint.h>

#include "EspLinkRuntimeStatus.h"
#include "MeasurementCommandPort.h"
#include "MeasurementRuntimeStatus.h"
#include "MeasurementQueryPort.h"
#include "SensorRuntimeStatus.h"

namespace dukatimer {

class MeasurementDomainService : public MeasurementQueryPort, public MeasurementCommandPort {
public:
	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs);
	void setWirelessCaptureArmed(bool armed);
	void observeLocalSensor(const SensorRuntimeStatus& sensorStatus, uint32_t nowMs = 0);
	void observeWirelessGateway(const WirelessGatewayStatus& wirelessStatus,
	                          EspLinkHealth linkHealth,
	                          uint32_t nowMs = 0);

	MeasurementLuxSample activeLuxSample() const override;
	MeasurementLuxSample localLuxSample() const override;
	MeasurementLuxSample wirelessLuxSample() const override;
	const MeasurementSessionStatus& sessionStatus() const override;
	bool captureLocalSample(uint32_t nowMs = 0) override;
	MeasurementSampleRole cyclePendingCaptureRole(int8_t direction) override;
	bool undoLastSessionSample() override;
	void resetSession() override;
	const MeasurementRuntimeStatus& state() const;

private:
	static constexpr uint32_t kLocalLuxStaleMs = 1500;
	static constexpr uint32_t kWirelessLuxStaleMs = 2200;
	static constexpr size_t kSessionUndoHistoryCapacity = 128;

	MeasurementRuntimeStatus state_ = makeUnknownMeasurementRuntimeStatus();
	uint32_t localSampleTimestampMs_ = 0;
	uint32_t wirelessSampleTimestampMs_ = 0;
	uint32_t lastWirelessHandledSequence_ = 0;
	uint32_t wirelessUndoBarrierSequence_ = 0;
	MeasurementSampleRole pendingCaptureRole_ = MeasurementSampleRole::None;
	std::array<MeasurementSessionSample, kSessionUndoHistoryCapacity> sessionUndoHistory_ = {};
	uint8_t sessionUndoHistoryCount_ = 0;
	MeasurementReferenceStatus localSessionReference_ = {};
	MeasurementReferenceStatus wirelessSessionReference_ = {};
	protocol::WirelessPeerState wirelessPeerState_ = protocol::WirelessPeerState::Unknown;
	EspLinkHealth linkHealth_ = EspLinkHealth::Unknown;
	bool wirelessCaptureArmed_ = false;
	bool initialized_ = false;

	uint32_t sanitizeTimestamp(uint32_t nowMs, uint32_t ageMs) const;
	static bool isSequenceNewer(uint32_t sequence, uint32_t reference);
	void refreshSampleAges(uint32_t nowMs);
	void recordWirelessMeasurementSample(uint32_t nowMs);
	bool appendSessionSample(const MeasurementLuxSample& sample, uint32_t nowMs);
	MeasurementSessionSample buildSessionSample(const MeasurementLuxSample& sample,
	                                         const MeasurementReferenceStatus& reference,
	                                         uint32_t nowMs) const;
	MeasurementReferenceStatus establishSessionReference(const MeasurementLuxSample& sample,
	                                                   uint32_t nowMs);
	MeasurementReferenceStatus* sessionReferenceForSource(MeasurementLuxSource source);
	const MeasurementReferenceStatus* sessionReferenceForSource(MeasurementLuxSource source) const;
	void clearSessionReferences();
	void rebuildSessionReferencesFromHistory();
	void refreshActiveReference();
	uint8_t zoneIndexFromRelativeEv(float relativeEvStops) const;
	void applyHistogramDelta(uint8_t zoneIndex, int delta);
	void syncSessionStatusFromHistory();
	bool isLocalSampleUsable() const;
	bool isWirelessSampleUsable() const;
	void chooseActiveSample();
};

}  // namespace dukatimer
