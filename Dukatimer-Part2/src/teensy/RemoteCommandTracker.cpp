#include "RemoteCommandTracker.h"

namespace dukatimer {
namespace {

constexpr RemoteCommandPolicy kFireAndForgetPolicy = {};

constexpr RemoteCommandPolicy kCriticalMeasurementPolicy = {
	RemoteCommandPolicyClass::Critical,
	RemoteCommandTracker::kCriticalAckTimeoutMs,
	RemoteCommandTracker::kCriticalMaxRetries,
	true,
};

}  // namespace

RemoteCommandPolicy RemoteCommandTracker::policyForCommand(protocol::TeensyCommandKind commandKind) {
	switch (commandKind) {
		case protocol::TeensyCommandKind::RemoteMeasurementStart:
		case protocol::TeensyCommandKind::RemoteMeasurementCancel:
			return kCriticalMeasurementPolicy;

		case protocol::TeensyCommandKind::None:
		case protocol::TeensyCommandKind::Ping:
		case protocol::TeensyCommandKind::RemoteHaptic:
		default:
			return kFireAndForgetPolicy;
	}
}

bool RemoteCommandTracker::isSequenceAcknowledged(uint32_t currentAckSequence, uint32_t commandSequence) {
	// Ack-Sequenzen koennen bei uint32 ueberlaufen; der signed Deltavergleich
	// behandelt deshalb auch Wrap-around weiterhin als monotone Ordnung.
	return static_cast<int32_t>(currentAckSequence - commandSequence) >= 0;
}

bool RemoteCommandTracker::pushCommand(const protocol::TeensyCommandPayload& payload,
	                                   uint32_t currentAckSequence,
	                                   uint32_t nowMs) {
	const RemoteCommandPolicy policy =
		policyForCommand(static_cast<protocol::TeensyCommandKind>(payload.commandKind));
	if (policy.policyClass == RemoteCommandPolicyClass::FireAndForget) {
		return true;
	}

	// Vor jedem Enqueue werden bereits kumulativ bestaetigte Eintraege entfernt,
	// damit spaet eintreffende Acks keine kuenstliche Queue-Saettigung erzeugen.
	acknowledgeThrough(currentAckSequence);
	if (inFlightCount_ >= kMaxInFlightCommands) {
		++status_.saturationCount;
		status_.lastSaturatedSequence = payload.commandSequence;
		return false;
	}

	InFlightCommand& entry = inFlight_[inFlightCount_++];
	entry.payload = payload;
	entry.policy = policy;
	entry.firstSentMs = nowMs;
	entry.lastSentMs = nowMs;
	entry.retryCount = 0u;
	status_.inFlightCount = inFlightCount_;
	status_.lastTrackedSequence = payload.commandSequence;
	return true;
}

void RemoteCommandTracker::acknowledgeThrough(uint32_t currentAckSequence) {
	// Das Ack ist kumulativ: sobald Sequenz N bestaetigt wurde, gelten auch alle
	// aelteren in-flight Kommandos als erledigt und koennen vorne entfernt werden.
	while (inFlightCount_ > 0u &&
	       isSequenceAcknowledged(currentAckSequence, inFlight_[0].payload.commandSequence)) {
		eraseAt(0u);
	}
}

void RemoteCommandTracker::eraseAt(uint8_t index) {
	if (index >= inFlightCount_) {
		return;
	}

	for (uint8_t current = static_cast<uint8_t>(index + 1u); current < inFlightCount_; ++current) {
		inFlight_[current - 1u] = inFlight_[current];
	}

	inFlight_[inFlightCount_ - 1u] = {};
	--inFlightCount_;
	status_.inFlightCount = inFlightCount_;
}

RemoteCommandTrackerTickResult RemoteCommandTracker::tick(uint32_t currentAckSequence, uint32_t nowMs) {
	RemoteCommandTrackerTickResult result = {};
	acknowledgeThrough(currentAckSequence);
	if (inFlightCount_ == 0u) {
		return result;
	}

	// Pro Loopiteration wird hoechstens ein Retry erneut ausgesendet, damit der
	// Link bei mehreren faelligen Kommandos nicht burstartig mit Wiederholungen flutet.
	for (uint8_t index = 0u; index < inFlightCount_; ++index) {
		InFlightCommand& entry = inFlight_[index];
		if ((nowMs - entry.lastSentMs) < entry.policy.ackTimeoutMs) {
			continue;
		}

		if (entry.retryCount < entry.policy.maxRetries) {
			++entry.retryCount;
			entry.lastSentMs = nowMs;
			++status_.retryCount;
			status_.lastRetrySequence = entry.payload.commandSequence;
			result.retryPayload = entry.payload;
			result.hasRetry = true;
			return result;
		}

		++status_.timeoutCount;
		status_.lastTimedOutSequence = entry.payload.commandSequence;
		result.timedOutPayload = entry.payload;
		result.fatalTimeout = entry.policy.fatalOnTimeout;
		eraseAt(index);
		return result;
	}

	return result;
}

const RemoteCommandTrackerStatus& RemoteCommandTracker::status() const {
	return status_;
}

}  // namespace dukatimer
