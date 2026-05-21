/*
 * Dukatimer-Part2 - ESP32-S3 service bootstrap
 *
 * Zweck:
 * - verdrahtet den ESP32-S3 als Service-Co-Prozessor des Teensy
 * - startet den seriellen Teensy-Link und die optionale HTTP-VFS-Bridge
 * - stellt frueh einen autoritativen, wenn auch noch teilweise simulierten
 *   Servicezustand fuer Thermik-, Ambient- und Wireless-Pfade bereit
 *
 * Architekturgrenze:
 * - main.cpp auf der ESP-Seite bleibt reiner Wiring- und Tick-Einstieg
 * - Sensor-, Funk- und Dateibrueckenlogik leben in getrennten Diensten
 */

#include <Arduino.h>

#include <DukatimerProtocol.h>

#include "Encoder4InputService.h"
#include "FirmwareVersion.h"
#include "HttpVfsBridge.h"
#include "ServiceSensorHub.h"
#include "TeensyLinkService.h"
#include "WirelessRemoteGateway.h"

namespace {

static_assert(dukatimer::protocol::kProtocolVersion == 1, "Unexpected SharedProtocol version");

HardwareSerial interMcuSerial(0);
dukatimer::TeensyLinkService teensyLink(interMcuSerial);
dukatimer::HttpVfsBridge httpVfsBridge(teensyLink);
dukatimer::ServiceSensorHub serviceSensorHub;
dukatimer::Encoder4InputService encoder4InputService;
dukatimer::WirelessRemoteGateway wirelessRemoteGateway(teensyLink);

void serviceEspWorkWhileUploadWaits(uint32_t nowMs) {
	teensyLink.tick(nowMs);
	serviceSensorHub.tick(nowMs, teensyLink.teensyExposureActive(nowMs));
	serviceSensorHub.publishTo(teensyLink, nowMs);
	encoder4InputService.tick(teensyLink, nowMs);
	wirelessRemoteGateway.tick(nowMs);
}

}

void setup() {
	// setup() bringt die Servicepfade in der Reihenfolge hoch, in der der Teensy
	// moeglichst frueh einen konsistenten Heartbeat- und Servicezustand sehen soll.
	Serial.begin(115200);
	Serial.printf("[%s] v%s\n", dukatimer::build::kFirmwareName, dukatimer::build::kFirmwareVersion);
	teensyLink.begin(millis());
	serviceSensorHub.begin(millis());
	encoder4InputService.begin(millis());
	httpVfsBridge.setCooperativeTickHook(serviceEspWorkWhileUploadWaits);
	httpVfsBridge.begin(millis());
	wirelessRemoteGateway.begin(millis());
}

void loop() {
	// Der ESP-Hauptloop bleibt reines Wiring: Sensoren und Funk-Gateway liefern
	// ihren Zustand an den Teensy-Link, die HTTP-Bruecke bedient Netzwerk plus
	// seriellen Uploadpfad.
	const uint32_t nowMs = millis();
	teensyLink.tick(nowMs);
	serviceSensorHub.tick(nowMs, teensyLink.teensyExposureActive(nowMs));
	serviceSensorHub.publishTo(teensyLink, nowMs);
	encoder4InputService.tick(teensyLink, nowMs);
	httpVfsBridge.tick(nowMs);
	wirelessRemoteGateway.tick(nowMs);
}
