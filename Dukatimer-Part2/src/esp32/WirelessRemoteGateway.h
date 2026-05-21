#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include <DukatimerProtocol.h>

namespace dukatimer {

class TeensyLinkService;

// ESP-seitige Bruecke zwischen SharedProtocol-Link und historischem
// Wireless-TSL2591-Handgeraet (ESP-NOW Paket-ABI).
class WirelessRemoteGateway {
public:
	explicit WirelessRemoteGateway(TeensyLinkService& teensyLink);
	~WirelessRemoteGateway();

	void begin(uint32_t nowMs = 0);
	void tick(uint32_t nowMs);
	void end();

private:
	static constexpr uint32_t kOfflineTimeoutMs = 5000;
	static constexpr uint32_t kEncoderLongPressMs = 600;
	static constexpr uint8_t kTargetAny = 0;

	TeensyLinkService& teensyLink_;
	bool initialized_ = false;
	bool espNowReady_ = false;
	bool peerBound_ = false;
	uint8_t peerMac_[6] = {};
	uint32_t lastSeenMs_ = 0;
	uint32_t lastLuxSampleTimestampMs_ = 0;
	float lastLux_ = 0.0f;
	bool luxValid_ = false;
	uint32_t measurementSequence_ = 0;
	uint32_t lastCommandAckSequence_ = 0;
	uint16_t renderStatusFlags_ = dukatimer::protocol::kWirelessRenderStatusNone;
	uint32_t staleRenderCount_ = 0;
	uint32_t renderTimeoutCount_ = 0;
	uint32_t lastRemoteSequence_ = 0;
	uint8_t activeButtons_ = dukatimer::protocol::kWirelessRemoteButtonNone;
	bool encoderButtonLongPressSent_ = false;
	uint32_t encoderButtonPressedAtMs_ = 0;
	dukatimer::protocol::WirelessPeerState peerState_ = dukatimer::protocol::WirelessPeerState::Offline;
	dukatimer::protocol::RemoteDisplayPayload lastDisplayPayload_ = {};

	static WirelessRemoteGateway* s_instance_;
	static bool isSequenceNewer(uint32_t sequence, uint32_t reference);

	void handlePendingLinkMessages(uint32_t nowMs);
	void updatePeerState(uint32_t nowMs);
	void publishState(uint32_t nowMs);
	void applyRemoteDisplay(const dukatimer::protocol::RemoteDisplayPayload& payload);
	void applyRemoteDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload);
	void applyTeensyCommand(const dukatimer::protocol::TeensyCommandPayload& payload, uint32_t nowMs);
	void applyRenderDiagnostics(const dukatimer::protocol::WirelessRemotePayload& payload, uint32_t nowMs);
	void applyEncoderDelta(int16_t encoderDelta, uint32_t nowMs);
	void applyButtonStates(uint8_t activeButtons, uint32_t nowMs);
	void bindPeer(const uint8_t* macAddress);
	void onReceivePacket(const uint8_t* macAddress, const uint8_t* data, int length);
	void onSendStatus(bool success);

	static void handleReceiveThunk(
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		const esp_now_recv_info_t* info,
#else
		const uint8_t* macAddress,
#endif
		const uint8_t* data,
		int length);
	static void handleSendThunk(
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		const esp_now_send_info_t* info,
#else
		const uint8_t* macAddress,
#endif
		esp_now_send_status_t status);
};

}  // namespace dukatimer