#pragma once

#include <array>
#include <stdint.h>

#include <DukatimerProtocol.h>

namespace dukatimer {

enum class RemoteCommandPolicyClass : uint8_t {
	FireAndForget = 0,
	Critical = 1,
};

struct RemoteCommandPolicy {
	RemoteCommandPolicyClass policyClass = RemoteCommandPolicyClass::FireAndForget;
	uint32_t ackTimeoutMs = 0;
	uint8_t maxRetries = 0;
	bool fatalOnTimeout = false;
};

struct InFlightCommand {
	protocol::TeensyCommandPayload payload = {};
	RemoteCommandPolicy policy = {};
	uint32_t firstSentMs = 0;
	uint32_t lastSentMs = 0;
	uint8_t retryCount = 0;
};

struct RemoteCommandTrackerStatus {
	uint8_t inFlightCount = 0;
	uint32_t retryCount = 0;
	uint32_t timeoutCount = 0;
	uint32_t saturationCount = 0;
	uint32_t lastTrackedSequence = 0;
	uint32_t lastRetrySequence = 0;
	uint32_t lastTimedOutSequence = 0;
	uint32_t lastSaturatedSequence = 0;
};

struct RemoteCommandTrackerTickResult {
	protocol::TeensyCommandPayload retryPayload = {};
	protocol::TeensyCommandPayload timedOutPayload = {};
	bool hasRetry = false;
	bool fatalTimeout = false;
};

class RemoteCommandTracker {
public:
	static constexpr uint32_t kCriticalAckTimeoutMs = 150;
	static constexpr uint8_t kCriticalMaxRetries = 4;
	static constexpr uint8_t kMaxInFlightCommands = 8;

	bool pushCommand(const protocol::TeensyCommandPayload& payload,
	                uint32_t currentAckSequence,
	                uint32_t nowMs);
	RemoteCommandTrackerTickResult tick(uint32_t currentAckSequence, uint32_t nowMs);
	const RemoteCommandTrackerStatus& status() const;

private:
	static RemoteCommandPolicy policyForCommand(protocol::TeensyCommandKind commandKind);
	static bool isSequenceAcknowledged(uint32_t currentAckSequence, uint32_t commandSequence);
	void acknowledgeThrough(uint32_t currentAckSequence);
	void eraseAt(uint8_t index);

	std::array<InFlightCommand, kMaxInFlightCommands> inFlight_ = {};
	uint8_t inFlightCount_ = 0;
	RemoteCommandTrackerStatus status_ = {};
};

}  // namespace dukatimer
