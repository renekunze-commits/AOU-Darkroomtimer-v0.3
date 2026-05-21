#pragma once

#include <stdint.h>

namespace dukatimer::service_link_runtime {

// Dieses Zusatzbit nutzt das bestehende Heartbeat-`runtimeFlags`-Feld zwischen
// ESP32-S3 und Teensy, ohne das SharedProtocol-Layout zu vergroessern. Es
// signalisiert einen aktiven Datei- oder VFS-Transaktionskontext am ESP-Pfad,
// also insbesondere HTTP-Staging plus serielle Upload-/Download-Phasen.
constexpr uint8_t kFileTransactionActive = 1u << 1;

inline bool hasFileTransactionActive(uint8_t runtimeFlags) {
	return (runtimeFlags & kFileTransactionActive) != 0u;
}

}  // namespace dukatimer::service_link_runtime