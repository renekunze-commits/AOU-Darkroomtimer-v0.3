/*
 * ServiceNetworkConfig
 *
 * Zentrale Build-/Bring-up-Konfiguration fuer den ESP32-HTTP-Servicepfad.
 * Diese Konstanten halten WLAN- und Timeoutvorgaben an einer Stelle zusammen,
 * statt sie in HttpVfsBridge oder main.cpp zu verstreuen.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dukatimer::service_config {

// Leere Station-Credentials erzwingen den AP-Fallback fuer lokales Bring-up.
constexpr const char* kStationSsid = "";
constexpr const char* kStationPassword = "";
constexpr const char* kAccessPointSsid = "Dukatimer-Part2-S3";
constexpr const char* kAccessPointPassword = "dukatimer42";
constexpr uint8_t kAccessPointChannel = 1;
constexpr uint16_t kHttpPort = 80;
constexpr size_t kHttpUploadReadSize = 512;
constexpr uint32_t kStationConnectTimeoutMs = 15000;
constexpr uint32_t kVfsAckTimeoutMs = 5000;
constexpr uint32_t kHttpBodyReadTimeoutMs = 5000;

}  // namespace dukatimer::service_config