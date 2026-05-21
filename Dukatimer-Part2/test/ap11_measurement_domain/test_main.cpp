#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#include "../support/Arduino.h"

unsigned long gFakeMillis = 0;

unsigned long millis() {
	return gFakeMillis;
}

namespace {

void setFakeMillis(unsigned long nowMs) {
	gFakeMillis = nowMs;
}

}  // namespace

#include "../../src/teensy/ExposureValueMath.cpp"
#include "../../src/teensy/MeasurementDomainService.cpp"
#include "../../src/teensy/MeasurementValueFormatter.cpp"

namespace {

using dukatimer::EspLinkHealth;
using dukatimer::LuxSampleValidity;
using dukatimer::MeasurementDomainService;
using dukatimer::MeasurementLuxSource;
using dukatimer::MeasurementSampleRole;
using dukatimer::MeasurementSessionMode;
using dukatimer::MeasurementSessionStatus;
using dukatimer::MeasurementValueFormatter;
using dukatimer::SensorHealth;
using dukatimer::SensorRuntimeStatus;
using dukatimer::Tsl2561RuntimeStatus;
using dukatimer::WirelessGatewayStatus;
using dukatimer::protocol::WirelessPeerState;

constexpr float kFloatTolerance = 0.02f;

[[noreturn]] void fail(const char* message) {
	std::fprintf(stderr, "FAIL: %s\n", message);
	std::exit(1);
}

void expectTrue(bool condition, const char* message) {
	if (!condition) {
		fail(message);
	}
}

void expectEqualU32(uint32_t actual, uint32_t expected, const char* message) {
	if (actual != expected) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=%lu expected=%lu)\n",
		             message,
		             static_cast<unsigned long>(actual),
		             static_cast<unsigned long>(expected));
		std::exit(1);
	}
}

void expectEqualU8(uint8_t actual, uint8_t expected, const char* message) {
	if (actual != expected) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=%u expected=%u)\n",
		             message,
		             static_cast<unsigned>(actual),
		             static_cast<unsigned>(expected));
		std::exit(1);
	}
}

void expectNear(float actual, float expected, float tolerance, const char* message) {
	if (std::fabs(actual - expected) > tolerance) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=%.4f expected=%.4f tolerance=%.4f)\n",
		             message,
		             actual,
		             expected,
		             tolerance);
		std::exit(1);
	}
}

void expectContains(const char* actual, const char* needle, const char* message) {
	if (std::strstr(actual, needle) == nullptr) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=\"%s\" needle=\"%s\")\n",
		             message,
		             actual,
		             needle);
		std::exit(1);
	}
}

void expectNotContains(const char* actual, const char* needle, const char* message) {
	if (std::strstr(actual, needle) != nullptr) {
		std::fprintf(stderr,
		             "FAIL: %s (actual=\"%s\" needle=\"%s\")\n",
		             message,
		             actual,
		             needle);
		std::exit(1);
	}
}

SensorRuntimeStatus makeLocalStatus(float lux, uint32_t sampleAgeMs = 0u) {
	SensorRuntimeStatus status = dukatimer::makeUnknownSensorRuntimeStatus();
	status.tsl2561.health = SensorHealth::Ok;
	status.tsl2561.sampleValidity = LuxSampleValidity::Valid;
	status.tsl2561.initialized = true;
	status.tsl2561.sampleFresh = true;
	status.tsl2561.sampleAgeMs = sampleAgeMs;
	status.tsl2561.lux = lux;
	return status;
}

WirelessGatewayStatus makeWirelessStatus(float lux,
	                                     uint32_t measurementSequence,
	                                     uint32_t sensorSampleAgeMs = 0u) {
	WirelessGatewayStatus status = {};
	status.peerState = WirelessPeerState::Online;
	status.flags = dukatimer::protocol::kWirelessSnapshotLuxValid;
	status.lastLux = lux;
	status.measurementSequence = measurementSequence;
	status.sensorSampleAgeMs = sensorSampleAgeMs;
	return status;
}

void observeLocalAndCapture(MeasurementDomainService& service,
	                        float lux,
	                        uint32_t nowMs,
	                        uint32_t sampleAgeMs = 0u) {
	setFakeMillis(nowMs);
	service.observeLocalSensor(makeLocalStatus(lux, sampleAgeMs), nowMs);
	expectTrue(service.captureLocalSample(nowMs), "Local sample should capture successfully");
	service.tick(nowMs);
}

void observeWireless(MeasurementDomainService& service,
	                 float lux,
	                 uint32_t measurementSequence,
	                 uint32_t nowMs,
	                 bool armed = true,
	                 uint32_t sensorSampleAgeMs = 0u) {
	setFakeMillis(nowMs);
	service.setWirelessCaptureArmed(armed);
	service.observeWirelessGateway(makeWirelessStatus(lux, measurementSequence, sensorSampleAgeMs),
	                            EspLinkHealth::Online,
	                            nowMs);
	service.tick(nowMs);
}

void runLocalRangeAndUndoCase() {
	MeasurementDomainService service;
	service.begin(1000u);

	// Der Basistest prueft die sichtbare Sessionsemantik fuer einen reinen
	// lokalen Quellpfad: erster Punkt setzt nur Referenz, zweiter Punkt schaltet
	// die Proposal-Vorstufe frei, Undo zieht die Session wieder auf einen Punkt
	// zurueck.
	observeLocalAndCapture(service, 100.0f, 1100u);
	MeasurementSessionStatus session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 1u, "Single local sample should set sampleCount to 1");
	expectTrue(session.rangeValid, "Single local sample should keep technical rangeValid true");
	expectTrue(!session.rangeUsableForProposal,
	          "Single local sample must not be proposal-eligible");
	expectTrue(session.commonSource == MeasurementLuxSource::LocalTsl2561,
	          "Single local sample should keep local commonSource");

	observeLocalAndCapture(service, 400.0f, 1200u);
	session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 2u, "Second local sample should extend session history");
	expectTrue(session.rangeUsableForProposal,
	          "Two local samples should unlock proposal preconditions");
	expectTrue(!session.mixedSources, "Two local samples must stay single-source");
	expectNear(session.relativeEvSpanStops, 2.0f, kFloatTolerance,
	          "100 -> 400 lux should span about two stops");

	expectTrue(service.undoLastSessionSample(), "Undo should remove the last local sample");
	service.tick(1201u);
	session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 1u, "Undo should reduce the visible history");
	expectTrue(!session.rangeUsableForProposal,
	          "Undo back to one sample must clear proposal usability");
	expectEqualU32(session.capturedSampleCount, 2u,
	              "Lifetime capture counter must survive undo");
}

void runMixedSourceAndReferenceRebuildCase() {
	MeasurementDomainService service;
	service.begin(2000u);

	// Dieser Fall prueft zwei Audit-Landminen gleichzeitig: Mixed-Source-
	// Sperre fuer Proposal und sauberen Wireless-Reference-Rebuild nach Undo der
	// letzten Wireless-Teilserie.
	observeLocalAndCapture(service, 100.0f, 2100u);
	observeWireless(service, 0.5f, 1u, 2200u, true, 50u);

	MeasurementSessionStatus session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 2u, "Local + wireless should produce two visible samples");
	expectTrue(session.mixedSources, "Mixed local and wireless samples must be flagged");
	expectEqualU8(session.sourceCount, 2u, "Mixed session should export both sources");
	expectTrue(!session.rangeUsableForProposal,
	          "Mixed-source session must stay proposal-blocked");
	expectTrue(!session.correctionReadyForProposal,
	          "Wireless dark-offset pending state must keep proposal correction-blocked");

	expectTrue(service.undoLastSessionSample(), "Undo should remove the last wireless sample");
	service.tick(2201u);
	session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 1u, "Undo should remove wireless sample from visible history");
	expectTrue(!session.mixedSources, "Undo back to local-only should clear mixedSources");

	observeWireless(service, 1.0f, 2u, 2300u, true, 10u);
	session = service.sessionStatus();
	expectTrue(session.latestSample.source == MeasurementLuxSource::WirelessGateway,
	          "Latest sample should become wireless again");
	expectNear(session.latestSample.referenceLux, 1.0f, kFloatTolerance,
	          "New wireless run must rebuild its own reference from visible history");
	expectNear(session.latestSample.relativeEvStops, 0.0f, kFloatTolerance,
	          "Freshly rebuilt wireless reference should yield zero relative EV");
}

void runOverflowHistogramAndReferenceRebuildCase() {
	MeasurementDomainService service;
	service.begin(3000u);

	// Der Overflow-Fall muss drei Dinge gleichzeitig halten: sichtbare History
	// bleibt bei 128 Eintraegen gedeckelt, der herausfallende Histogrammbeitrag
	// verschwindet wirklich, und die aktive Referenz springt auf den ersten noch
	// sichtbaren Sample der Quelle.
	observeLocalAndCapture(service, 100.0f, 3100u);
	for (uint32_t index = 1u; index < 128u; ++index) {
		observeLocalAndCapture(service, 200.0f, 3100u + index);
	}

	MeasurementSessionStatus session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 128u, "History should fill to the hard capacity");
	expectEqualU8(session.zoneHistogram[5], 20u,
	            "First reference sample should contribute once to the center bucket");

	observeLocalAndCapture(service, 200.0f, 3300u);
	service.tick(3300u);
	session = service.sessionStatus();
	expectEqualU32(session.sampleCount, 128u, "Overflow must keep visible sampleCount capped");
	expectEqualU32(session.droppedSampleCount, 1u, "Overflow must report one dropped sample");
	expectEqualU8(session.zoneHistogram[5], 0u,
	            "Dropped center-reference bucket contribution must be removed on overflow");
	expectNear(service.state().activeReference.lux, 200.0f, kFloatTolerance,
	          "Active reference must rebuild to the first still-visible local sample");
}

void runPendingRoleModeCase() {
	MeasurementDomainService service;
	service.begin(4000u);

	// Die Rollenwahl aus P2.4 bleibt Teil der P3-Abdeckung, weil Proposal- und
	// Hardware-Checks sonst wieder auf einer impliziten Sessionsemantik laufen.
	expectTrue(service.cyclePendingCaptureRole(1) == MeasurementSampleRole::Shadow,
	          "First role cycle should enter shadow mode");
	expectTrue(service.sessionStatus().mode == MeasurementSessionMode::RelativeSpot,
	          "Shadow role should stay in relative-spot session mode");
	expectTrue(service.cyclePendingCaptureRole(3) == MeasurementSampleRole::Dark,
	          "Further role cycles should reach dark mode");
	expectTrue(service.sessionStatus().mode == MeasurementSessionMode::PaperCalibration,
	          "Dark role should flip the session into paper-calibration mode");

	observeLocalAndCapture(service, 50.0f, 4100u);
	const MeasurementSessionStatus session = service.sessionStatus();
	expectTrue((session.latestSample.roleMask & static_cast<uint16_t>(MeasurementSampleRole::Dark)) != 0u,
	          "Captured sample should inherit the pending explicit role");
}

void runInvalidRangeFormatterCase() {
	MeasurementSessionStatus session = {};
	char buffer[96] = {};

	MeasurementValueFormatter::formatMeasurementRange(buffer, sizeof(buffer), session);

	expectContains(buffer,
	              "NO RNG",
	              "Invalid range should be labelled explicitly instead of showing null-style range output");
	expectNotContains(buffer,
	                 "LO ",
	                 "Invalid range must not render LO/HI fields as if a technical range existed");
}

}  // namespace

int main() {
	runLocalRangeAndUndoCase();
	runMixedSourceAndReferenceRebuildCase();
	runOverflowHistogramAndReferenceRebuildCase();
	runPendingRoleModeCase();
	runInvalidRangeFormatterCase();
	std::puts("AP11 measurement domain harness passed");
	return 0;
}