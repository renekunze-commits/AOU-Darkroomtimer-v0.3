/*
 * FirmwareVersion
 *
 * Zentrale Versions- und Identitaetskonstanten der ESP32-Service-Firmware.
 * Heartbeat, Serial-Ausgabe und Diagnosepfade greifen auf dieselben Werte zu.
 */

#pragma once

namespace dukatimer::build {

static constexpr const char* kFirmwareName = "Dukatimer Part2 ESP Service";
static constexpr const char* kFirmwareVersion = "0.1.14-dev";

static constexpr unsigned kVersionMajor = 0;
static constexpr unsigned kVersionMinor = 1;
static constexpr unsigned kVersionPatch = 14;

}  // namespace dukatimer::build