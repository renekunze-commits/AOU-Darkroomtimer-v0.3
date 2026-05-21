/*
 * WirelessRemoteGateway
 *
 * Diese Datei uebersetzt zwischen zwei Welten:
 * - SharedProtocol auf dem ESP<->Teensy-Service-Link
 * - das aggregierte ESP-NOW Dual-Mode-Terminalprotokoll des C6-Handgeraets
 *
 * Ziel ist ein datengetriebener Brueckenpfad: der Teensy liefert formatierte
 * Textsichtdaten, das Wireless-Terminal sendet aggregierte Eingabe- und
 * Live-Luxdaten zurueck.
 */

#include "WirelessRemoteGateway.h"

#include <esp_now.h>
#include <WiFi.h>

#include <cmath>
#include <cstring>

#include "TeensyLinkService.h"

namespace dukatimer {

namespace {

bool macMatches(const uint8_t* left, const uint8_t* right) {
	return left != nullptr && right != nullptr && memcmp(left, right, 6) == 0;
}

bool ensureEspNowPeer(const uint8_t* macAddress) {
	if (macAddress == nullptr) {
		return false;
	}

	esp_now_peer_info_t peerInfo = {};
	memcpy(peerInfo.peer_addr, macAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;
	peerInfo.ifidx = WIFI_IF_STA;
	const esp_err_t result = esp_now_add_peer(&peerInfo);
	return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

uint32_t timestampFromAge(uint32_t nowMs, uint32_t ageMs) {
	return ageMs <= nowMs ? (nowMs - ageMs) : 0u;
}

}  // namespace

WirelessRemoteGateway* WirelessRemoteGateway::s_instance_ = nullptr;

WirelessRemoteGateway::WirelessRemoteGateway(TeensyLinkService& teensyLink) : teensyLink_(teensyLink) {}

WirelessRemoteGateway::~WirelessRemoteGateway() {
	end();
}

void WirelessRemoteGateway::end() {
	if (s_instance_ == this) {
		esp_now_unregister_recv_cb();
		esp_now_unregister_send_cb();
		s_instance_ = nullptr;
	}

	if (espNowReady_) {
		esp_now_deinit();
	}

	initialized_ = false;
	espNowReady_ = false;
	peerBound_ = false;
	memset(peerMac_, 0, sizeof(peerMac_));
	lastSeenMs_ = 0;
	lastLuxSampleTimestampMs_ = 0;
	lastLux_ = 0.0f;
	luxValid_ = false;
	measurementSequence_ = 0;
	lastCommandAckSequence_ = 0;
	renderStatusFlags_ = dukatimer::protocol::kWirelessRenderStatusNone;
	staleRenderCount_ = 0;
	renderTimeoutCount_ = 0;
	lastRemoteSequence_ = 0;
	activeButtons_ = dukatimer::protocol::kWirelessRemoteButtonNone;
	encoderButtonLongPressSent_ = false;
	encoderButtonPressedAtMs_ = 0;
	peerState_ = dukatimer::protocol::WirelessPeerState::Offline;
	lastDisplayPayload_ = {};
}

bool WirelessRemoteGateway::isSequenceNewer(uint32_t sequence, uint32_t reference) {
	if (sequence == 0u) {
		return false;
	}

	return static_cast<int32_t>(sequence - reference) > 0;
}

void WirelessRemoteGateway::begin(uint32_t nowMs) {
	if (initialized_) {
		return;
	}

	initialized_ = true;
	lastSeenMs_ = nowMs;
	if (WiFi.getMode() == WIFI_MODE_NULL) {
		WiFi.mode(WIFI_STA);
	} else if (WiFi.getMode() == WIFI_MODE_AP) {
		WiFi.mode(WIFI_AP_STA);
	}

	if (esp_now_init() != ESP_OK) {
		teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessGatewayInitFailed, 0u, nowMs);
		peerState_ = dukatimer::protocol::WirelessPeerState::Fault;
		publishState(nowMs);
		return;
	}

	s_instance_ = this;
	esp_now_register_recv_cb(handleReceiveThunk);
	esp_now_register_send_cb(handleSendThunk);
	static const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	ensureEspNowPeer(broadcastAddress);
	espNowReady_ = true;
	peerState_ = dukatimer::protocol::WirelessPeerState::Offline;
	publishState(nowMs);
}

void WirelessRemoteGateway::tick(uint32_t nowMs) {
	if (!initialized_) {
		begin(nowMs);
	}

	handlePendingLinkMessages(nowMs);
	updatePeerState(nowMs);
	publishState(nowMs);
}

void WirelessRemoteGateway::handlePendingLinkMessages(uint32_t nowMs) {
	dukatimer::protocol::RemoteDisplayPayload renderPayload;
	while (teensyLink_.consumePendingRemoteRender(&renderPayload)) {
		applyRemoteDisplay(renderPayload);
	}

	dukatimer::protocol::DiagnosticPayload diagnosticPayload = {};
	while (teensyLink_.consumePendingRemoteDiagnostic(&diagnosticPayload)) {
		applyRemoteDiagnostic(diagnosticPayload);
	}

	dukatimer::protocol::TeensyCommandPayload commandPayload;
	while (teensyLink_.consumePendingCommand(&commandPayload)) {
		applyTeensyCommand(commandPayload, nowMs);
	}
}

void WirelessRemoteGateway::updatePeerState(uint32_t nowMs) {
	if (!espNowReady_) {
		peerState_ = dukatimer::protocol::WirelessPeerState::Fault;
		return;
	}

	if (!peerBound_ || lastSeenMs_ == 0u || (nowMs - lastSeenMs_) > kOfflineTimeoutMs) {
		peerState_ = dukatimer::protocol::WirelessPeerState::Offline;
		return;
	}

	peerState_ = dukatimer::protocol::WirelessPeerState::Online;
}

void WirelessRemoteGateway::publishState(uint32_t nowMs) {
	teensyLink_.setWirelessPeerState(peerState_, 0u, luxValid_, lastLux_, measurementSequence_,
	                             lastCommandAckSequence_, renderStatusFlags_, staleRenderCount_,
	                             renderTimeoutCount_, peerBound_ ? lastSeenMs_ : 0u,
	                             luxValid_ ? lastLuxSampleTimestampMs_ : 0u);
	(void)nowMs;
}

void WirelessRemoteGateway::applyRemoteDisplay(const dukatimer::protocol::RemoteDisplayPayload& payload) {
	if (!espNowReady_) {
		return;
	}
	lastDisplayPayload_ = payload;

	const uint8_t* targetMac = peerBound_ ? peerMac_ : nullptr;
	if (targetMac == nullptr) {
		static const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		targetMac = broadcastAddress;
	}

	const esp_err_t sendResult = esp_now_send(targetMac, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
	if (sendResult != ESP_OK) {
		peerState_ = dukatimer::protocol::WirelessPeerState::Fault;
		teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessSendFailed,
		                        static_cast<uint16_t>(sendResult & 0xFFFFu), millis());
	}
}

void WirelessRemoteGateway::applyRemoteDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload) {
	if (!espNowReady_) {
		return;
	}

	const uint8_t* targetMac = peerBound_ ? peerMac_ : nullptr;
	if (targetMac == nullptr) {
		static const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		targetMac = broadcastAddress;
	}

	const esp_err_t sendResult = esp_now_send(targetMac, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
	if (sendResult != ESP_OK) {
		peerState_ = dukatimer::protocol::WirelessPeerState::Fault;
		teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessSendFailed,
		                        static_cast<uint16_t>(sendResult & 0xFFFFu), millis());
	}
}

void WirelessRemoteGateway::applyTeensyCommand(const dukatimer::protocol::TeensyCommandPayload& payload,
	                                         uint32_t nowMs) {
	(void)nowMs;
	if (!espNowReady_) {
		return;
	}

	if (payload.target != kTargetAny) {
		return;
	}

	switch (static_cast<dukatimer::protocol::TeensyCommandKind>(payload.commandKind)) {
		case dukatimer::protocol::TeensyCommandKind::Ping:
		case dukatimer::protocol::TeensyCommandKind::RemoteMeasurementStart:
		case dukatimer::protocol::TeensyCommandKind::RemoteMeasurementCancel:
		case dukatimer::protocol::TeensyCommandKind::RemoteHaptic: {
			// Der C6-Terminalpfad bleibt bewusst High-Level-Dumb-Terminal: Der Teensy
			// trifft die Fachentscheidung, der ESP-S3 leitet nur das vorbereitete
			// Kommando roh und ohne Umdeutung per ESP-NOW weiter.
			const uint8_t* targetMac = peerBound_ ? peerMac_ : nullptr;
			if (targetMac == nullptr) {
				static const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
				targetMac = broadcastAddress;
			}

			const esp_err_t sendResult =
				esp_now_send(targetMac, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
			if (sendResult != ESP_OK) {
				peerState_ = dukatimer::protocol::WirelessPeerState::Fault;
				teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessSendFailed,
				                        static_cast<uint16_t>(sendResult & 0xFFFFu), millis());
			}
			break;
		}

		case dukatimer::protocol::TeensyCommandKind::RemoteRender:
		case dukatimer::protocol::TeensyCommandKind::None:
			break;
	}
}

void WirelessRemoteGateway::applyRenderDiagnostics(const dukatimer::protocol::WirelessRemotePayload& payload,
	                                            uint32_t nowMs) {
	if (payload.staleRenderCount > staleRenderCount_) {
		const uint16_t detail = payload.staleRenderCount > 0xFFFFu ? 0xFFFFu
		                                                     : static_cast<uint16_t>(payload.staleRenderCount);
		teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessRenderStale, detail, nowMs);
	}

	if (payload.renderTimeoutCount > renderTimeoutCount_) {
		const uint16_t detail = payload.renderTimeoutCount > 0xFFFFu ? 0xFFFFu
		                                                       : static_cast<uint16_t>(payload.renderTimeoutCount);
		teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessRenderTimeout, detail, nowMs);
	}

	renderStatusFlags_ = payload.renderStatusFlags;
	staleRenderCount_ = payload.staleRenderCount;
	renderTimeoutCount_ = payload.renderTimeoutCount;
}

void WirelessRemoteGateway::applyEncoderDelta(int16_t encoderDelta, uint32_t nowMs) {
	if (encoderDelta > 0) {
		teensyLink_.queueInputEvent(dukatimer::protocol::RemoteInputSource::WirelessEncoder,
		                        dukatimer::protocol::InputEventKind::RotateRight, encoderDelta, nowMs);
	} else if (encoderDelta < 0) {
		teensyLink_.queueInputEvent(dukatimer::protocol::RemoteInputSource::WirelessEncoder,
		                        dukatimer::protocol::InputEventKind::RotateLeft, encoderDelta, nowMs);
	}
}

void WirelessRemoteGateway::applyButtonStates(uint8_t activeButtons, uint32_t nowMs) {
	const bool measurePressed = (activeButtons & dukatimer::protocol::kWirelessRemoteButtonMeasure) != 0u;
	const bool measureWasPressed = (activeButtons_ & dukatimer::protocol::kWirelessRemoteButtonMeasure) != 0u;
	if (measurePressed && !measureWasPressed) {
		teensyLink_.queueInputEvent(dukatimer::protocol::RemoteInputSource::WirelessMeasureButton,
		                        dukatimer::protocol::InputEventKind::Press, 1, nowMs);
		if (luxValid_) {
			++measurementSequence_;
		}
	}

	const bool backPressed = (activeButtons & dukatimer::protocol::kWirelessRemoteButtonBack) != 0u;
	const bool backWasPressed = (activeButtons_ & dukatimer::protocol::kWirelessRemoteButtonBack) != 0u;
	if (backPressed && !backWasPressed) {
		teensyLink_.queueInputEvent(dukatimer::protocol::RemoteInputSource::WirelessBackButton,
		                        dukatimer::protocol::InputEventKind::Press, 1, nowMs);
	}

	const bool encoderPressed = (activeButtons & dukatimer::protocol::kWirelessRemoteButtonEncoder) != 0u;
	const bool encoderWasPressed = (activeButtons_ & dukatimer::protocol::kWirelessRemoteButtonEncoder) != 0u;
	if (encoderPressed && !encoderWasPressed) {
		encoderButtonPressedAtMs_ = nowMs;
		encoderButtonLongPressSent_ = false;
	}

	if (encoderPressed && !encoderButtonLongPressSent_ && encoderButtonPressedAtMs_ != 0u &&
	    (nowMs - encoderButtonPressedAtMs_) >= kEncoderLongPressMs) {
		teensyLink_.queueInputEvent(dukatimer::protocol::RemoteInputSource::WirelessEncoderButton,
		                        dukatimer::protocol::InputEventKind::LongPress, 1, nowMs);
		encoderButtonLongPressSent_ = true;
	}

	if (!encoderPressed && encoderWasPressed) {
		if (!encoderButtonLongPressSent_) {
			teensyLink_.queueInputEvent(dukatimer::protocol::RemoteInputSource::WirelessEncoderButton,
			                        dukatimer::protocol::InputEventKind::Press, 1, nowMs);
		}
		encoderButtonPressedAtMs_ = 0u;
		encoderButtonLongPressSent_ = false;
	}

	activeButtons_ = activeButtons;
}

void WirelessRemoteGateway::bindPeer(const uint8_t* macAddress) {
	if (macAddress == nullptr || (peerBound_ && macMatches(peerMac_, macAddress))) {
		return;
	}

	if (ensureEspNowPeer(macAddress)) {
		memcpy(peerMac_, macAddress, sizeof(peerMac_));
		peerBound_ = true;
	}
}

void WirelessRemoteGateway::onReceivePacket(const uint8_t* macAddress, const uint8_t* data, int length) {
	if (data == nullptr || length != static_cast<int>(sizeof(dukatimer::protocol::WirelessRemotePayload))) {
		return;
	}

	dukatimer::protocol::WirelessRemotePayload packet = {};
	memcpy(&packet, data, sizeof(packet));
	if (!isSequenceNewer(packet.sequenceNumber, lastRemoteSequence_) && packet.sequenceNumber != 0u) {
		return;
	}

	bindPeer(macAddress);
	lastSeenMs_ = millis();
	lastRemoteSequence_ = packet.sequenceNumber;
	luxValid_ = std::isfinite(packet.activeLux) && packet.activeLux > 0.0f;
	lastLux_ = luxValid_ ? packet.activeLux : 0.0f;
	lastLuxSampleTimestampMs_ = luxValid_ ? timestampFromAge(lastSeenMs_, packet.activeLuxAgeMs) : 0u;
	// Der Gatewayzustand fuehrt den letzten bestaetigten Kommandozaehler
	// getrennt von der Eingabesequenz, damit der Teensy Sendung und Ausfuehrung
	// spaeter auseinanderhalten kann.
	if (isSequenceNewer(packet.commandAckSequence, lastCommandAckSequence_)) {
		lastCommandAckSequence_ = packet.commandAckSequence;
	}
	applyRenderDiagnostics(packet, lastSeenMs_);
	applyEncoderDelta(packet.encoderDelta, lastSeenMs_);
	applyButtonStates(packet.activeButtons, lastSeenMs_);
	peerState_ = dukatimer::protocol::WirelessPeerState::Online;
}

void WirelessRemoteGateway::onSendStatus(bool success) {
	if (!success) {
		teensyLink_.queueDiagnostic(dukatimer::protocol::DiagnosticCode::WirelessSendFailed, 0u, millis());
		peerState_ = dukatimer::protocol::WirelessPeerState::Fault;
	}
}

void WirelessRemoteGateway::handleReceiveThunk(
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	const esp_now_recv_info_t* info,
#else
	const uint8_t* macAddress,
#endif
	const uint8_t* data,
	int length) {
	if (s_instance_ == nullptr) {
		return;
	}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	const uint8_t* macAddress = info != nullptr ? info->src_addr : nullptr;
#endif
	s_instance_->onReceivePacket(macAddress, data, length);
}

void WirelessRemoteGateway::handleSendThunk(
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	const esp_now_send_info_t* info,
#else
	const uint8_t* macAddress,
#endif
	esp_now_send_status_t status) {
	if (s_instance_ == nullptr) {
		return;
	}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	(void)info;
#else
	(void)macAddress;
#endif

	s_instance_->onSendStatus(status == ESP_NOW_SEND_SUCCESS);
}

}  // namespace dukatimer