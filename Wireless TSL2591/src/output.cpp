#include "output.h"

#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_I2C_SCL, PIN_I2C_SDA);

static unsigned long lastDisplayUpdate = 0;
static bool isDisplaySleeping = false;

namespace {

struct RemoteDiagnosticState {
	uint32_t retryCount = 0;
	uint32_t saturationCount = 0;
	uint32_t timeoutCount = 0;
	dukatimer::protocol::DiagnosticCode lastCode = dukatimer::protocol::DiagnosticCode::None;
	uint32_t lastCounter = 0;
	uint16_t lastDetail = 0;
	uint32_t lastUpdateMs = 0;
};

struct HapticSequenceState {
	bool active = false;
	bool motorHigh = false;
	uint8_t pendingPulseStarts = 0;
	uint16_t onDurationMs = 0;
	uint16_t offDurationMs = 0;
	uint32_t nextTransitionMs = 0;
};

struct LocalRenderDiagnosticState {
	uint32_t staleRenderCount = 0;
	uint32_t renderTimeoutCount = 0;
	bool renderTimeoutActive = false;
	uint32_t lastUpdateMs = 0;
};

static HapticSequenceState gHapticSequence = {};
static RemoteDiagnosticState gRemoteDiagnosticState = {};
static LocalRenderDiagnosticState gLocalRenderDiagnosticState = {};

unsigned long footerCounterValue(uint32_t counter) {
	return static_cast<unsigned long>(counter > 999u ? 999u : counter);
}

void finishFrame() {
	u8g2.sendBuffer();
	lastDisplayUpdate = millis();
}

void drawTextLine(uint8_t y, const char* text) {
	u8g2.drawStr(0, y, text != nullptr ? text : "");
}

void drawProgressBar(uint8_t progressPercent) {
	if (progressPercent > 100u) {
		return;
	}

	u8g2.drawFrame(0, 54, 128, 10);
	const int width = (126 * static_cast<int>(progressPercent)) / 100;
	if (width > 0) {
		u8g2.drawBox(1, 55, width, 8);
	}
}

bool shouldDrawRemoteDiagnosticFooter() {
	return gRemoteDiagnosticState.retryCount != 0u || gRemoteDiagnosticState.saturationCount != 0u ||
	       gRemoteDiagnosticState.timeoutCount != 0u ||
	       (gRemoteDiagnosticState.lastUpdateMs != 0u && (millis() - gRemoteDiagnosticState.lastUpdateMs) < 5000u);
}

bool shouldDrawLocalRenderDiagnosticFooter() {
	return gLocalRenderDiagnosticState.renderTimeoutActive ||
	       gLocalRenderDiagnosticState.staleRenderCount != 0u ||
	       gLocalRenderDiagnosticState.renderTimeoutCount != 0u;
	}

void drawLocalRenderDiagnosticFooter(uint8_t baselineY) {
	char buffer[24];
	snprintf(buffer, sizeof(buffer), "RS%lu RT%lu%s",
	         footerCounterValue(gLocalRenderDiagnosticState.staleRenderCount),
	         footerCounterValue(gLocalRenderDiagnosticState.renderTimeoutCount),
	         gLocalRenderDiagnosticState.renderTimeoutActive ? " !" : "");
	u8g2.setFont(u8g2_font_5x8_tr);
	drawTextLine(baselineY, buffer);
}

void drawRemoteDiagnosticFooter(uint8_t baselineY) {
	if (!shouldDrawRemoteDiagnosticFooter()) {
		return;
	}

	char buffer[28];
	snprintf(buffer, sizeof(buffer), "R%lu S%lu T%lu",
	         static_cast<unsigned long>(gRemoteDiagnosticState.retryCount),
	         static_cast<unsigned long>(gRemoteDiagnosticState.saturationCount),
	         static_cast<unsigned long>(gRemoteDiagnosticState.timeoutCount));
	u8g2.setFont(u8g2_font_5x8_tr);
	drawTextLine(baselineY, buffer);
}

void drawStatusFooter(uint8_t baselineY) {
	if (shouldDrawLocalRenderDiagnosticFooter()) {
		drawLocalRenderDiagnosticFooter(baselineY);
		return;
	}

	drawRemoteDiagnosticFooter(baselineY);
}

void stopHapticMotor() {
	digitalWrite(PIN_VIB_MOTOR, LOW);
	gHapticSequence.active = false;
	gHapticSequence.motorHigh = false;
	gHapticSequence.pendingPulseStarts = 0;
	gHapticSequence.nextTransitionMs = 0;
}

void startHapticSequence(uint8_t pulseCount, uint16_t onDurationMs, uint16_t offDurationMs) {
	if (pulseCount == 0u) {
		stopHapticMotor();
		return;
	}

	// Haptik wird bewusst nicht-blockierend im Loop abgearbeitet, damit weder die
	// lokale Sensorakquise noch ESP-NOW-Sendezyklen fuer ein Vibrationsfeedback
	// angehalten werden.
	gHapticSequence.active = true;
	gHapticSequence.motorHigh = true;
	gHapticSequence.pendingPulseStarts = static_cast<uint8_t>(pulseCount - 1u);
	gHapticSequence.onDurationMs = onDurationMs;
	gHapticSequence.offDurationMs = offDurationMs;
	gHapticSequence.nextTransitionMs = millis() + onDurationMs;
	digitalWrite(PIN_VIB_MOTOR, HIGH);
}

}  // namespace

float calculateEV(float lux) {
	if (lux <= 0.01f) {
		return 0.0f;
	}

	return log2f((lux * 100.0f) / 250.0f);
}

void observeRemoteDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload) {
	gRemoteDiagnosticState.lastCode = static_cast<dukatimer::protocol::DiagnosticCode>(payload.code);
	gRemoteDiagnosticState.lastCounter = payload.counter;
	gRemoteDiagnosticState.lastDetail = payload.detail;
	gRemoteDiagnosticState.lastUpdateMs = millis();

	switch (gRemoteDiagnosticState.lastCode) {
		case dukatimer::protocol::DiagnosticCode::RemoteCommandRetry:
			gRemoteDiagnosticState.retryCount = payload.counter;
			break;

		case dukatimer::protocol::DiagnosticCode::RemoteCommandTrackerSaturated:
			gRemoteDiagnosticState.saturationCount = payload.counter;
			break;

		case dukatimer::protocol::DiagnosticCode::RemoteCommandTimeout:
			gRemoteDiagnosticState.timeoutCount = payload.counter;
			break;

		case dukatimer::protocol::DiagnosticCode::WirelessRenderStale:
		case dukatimer::protocol::DiagnosticCode::WirelessRenderTimeout:
			break;

		case dukatimer::protocol::DiagnosticCode::None:
		case dukatimer::protocol::DiagnosticCode::ServiceDs18Fault:
		case dukatimer::protocol::DiagnosticCode::ServiceAhtFault:
		case dukatimer::protocol::DiagnosticCode::WirelessGatewayInitFailed:
		case dukatimer::protocol::DiagnosticCode::WirelessMeasurementTimeout:
		case dukatimer::protocol::DiagnosticCode::WirelessSendFailed:
		case dukatimer::protocol::DiagnosticCode::LinkRenderCoalesced:
		case dukatimer::protocol::DiagnosticCode::LinkCommandDropped:
		case dukatimer::protocol::DiagnosticCode::LinkInputDropped:
			break;
	}
}

void observeLocalRenderDiagnostics(uint32_t staleRenderCount,
	                              uint32_t renderTimeoutCount,
	                              bool renderTimeoutActive) {
	if (gLocalRenderDiagnosticState.staleRenderCount == staleRenderCount &&
	    gLocalRenderDiagnosticState.renderTimeoutCount == renderTimeoutCount &&
	    gLocalRenderDiagnosticState.renderTimeoutActive == renderTimeoutActive) {
		return;
	}

	gLocalRenderDiagnosticState.staleRenderCount = staleRenderCount;
	gLocalRenderDiagnosticState.renderTimeoutCount = renderTimeoutCount;
	gLocalRenderDiagnosticState.renderTimeoutActive = renderTimeoutActive;
	gLocalRenderDiagnosticState.lastUpdateMs = millis();
}

void wakeDisplay() {
    if (isDisplaySleeping) {
		u8g2.setPowerSave(0);
		isDisplaySleeping = false;
	}
	lastDisplayUpdate = millis();
}

void checkDisplaySleep() {
    if (!isDisplaySleeping && (millis() - lastDisplayUpdate > DISPLAY_SLEEP_TIMEOUT)) {
		u8g2.setPowerSave(1);
		isDisplaySleeping = true;
	}
}

void initOutput() {
	Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
	u8g2.begin();
	u8g2.clearBuffer();
	u8g2.sendBuffer();
	pinMode(PIN_VIB_MOTOR, OUTPUT);
	digitalWrite(PIN_VIB_MOTOR, LOW);
	gHapticSequence = {};
	gLocalRenderDiagnosticState = {};
}

void showStartup() {
	wakeDisplay();
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_6x12_tr);
	drawTextLine(18, "Wireless Terminal");
	drawTextLine(34, "Initialisiere...");
	finishFrame();
}

void showError(const char* message) {
	wakeDisplay();
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_6x12_tr);
	drawTextLine(16, "Fehler");
	drawTextLine(34, message != nullptr ? message : "Unknown");
	finishFrame();
}

void renderSlaveMode(const dukatimer::protocol::RemoteDisplayPayload& data) {
	wakeDisplay();
	u8g2.clearBuffer();
	u8g2.setFont(data.viewType == 2u ? u8g2_font_6x12_tr : u8g2_font_5x8_tr);
	drawTextLine(14, data.line1);
	drawTextLine(30, data.line2);
	drawTextLine(46, data.line3);
	if (data.progressPercent <= 100u) {
		drawProgressBar(data.progressPercent);
	} else {
		drawStatusFooter(62);
	}
	finishFrame();
}

void renderRemoteOfflineMode(float lux) {
	wakeDisplay();
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_6x12_tr);
	drawTextLine(14, "Remote offline");

	char buffer[24];
	snprintf(buffer, sizeof(buffer), "%.2f lx", lux);
	drawTextLine(34, buffer);
	snprintf(buffer, sizeof(buffer), "EV %.1f lokal", calculateEV(lux));
	drawTextLine(52, buffer);
	drawStatusFooter(63);
	finishFrame();
}

void renderRemoteMeasurementMode(float lux, bool hardPhase) {
	wakeDisplay();
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_6x12_tr);
	drawTextLine(14, hardPhase ? "Remote Messung HART" : "Remote Messung SOFT");

	char buffer[24];
	snprintf(buffer, sizeof(buffer), "%.2f lx", lux);
	drawTextLine(34, buffer);
	snprintf(buffer, sizeof(buffer), "EV %.1f", calculateEV(lux));
	drawTextLine(52, buffer);
	drawStatusFooter(63);
	finishFrame();
}

void renderStandAloneMode(float lux) {
	wakeDisplay();
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_helvB12_tr);
	char buffer[24];
	snprintf(buffer, sizeof(buffer), "%.2f lx", lux);
	drawTextLine(24, buffer);
	snprintf(buffer, sizeof(buffer), "EV %.1f", calculateEV(lux));
	drawTextLine(48, buffer);
	drawStatusFooter(63);
	finishFrame();
}

void triggerHaptic(dukatimer::protocol::RemoteHapticFeedback feedback) {
	switch (feedback) {
		case dukatimer::protocol::RemoteHapticFeedback::Click:
			startHapticSequence(1u, 30u, 0u);
			break;

		case dukatimer::protocol::RemoteHapticFeedback::Done:
			startHapticSequence(2u, 45u, 55u);
			break;

		case dukatimer::protocol::RemoteHapticFeedback::Error:
			startHapticSequence(3u, 75u, 60u);
			break;

		case dukatimer::protocol::RemoteHapticFeedback::None:
			stopHapticMotor();
			break;
	}
}

void serviceOutput() {
	if (!gHapticSequence.active) {
		return;
	}

	const uint32_t nowMs = millis();
	if (static_cast<int32_t>(nowMs - gHapticSequence.nextTransitionMs) < 0) {
		return;
	}

	if (gHapticSequence.motorHigh) {
		digitalWrite(PIN_VIB_MOTOR, LOW);
		gHapticSequence.motorHigh = false;
		if (gHapticSequence.pendingPulseStarts == 0u) {
			gHapticSequence.active = false;
			return;
		}

		gHapticSequence.nextTransitionMs = nowMs + gHapticSequence.offDurationMs;
		return;
	}

	digitalWrite(PIN_VIB_MOTOR, HIGH);
	gHapticSequence.motorHigh = true;
	--gHapticSequence.pendingPulseStarts;
	gHapticSequence.nextTransitionMs = nowMs + gHapticSequence.onDurationMs;
}

void clickSound() {
	triggerHaptic(dukatimer::protocol::RemoteHapticFeedback::Click);
}