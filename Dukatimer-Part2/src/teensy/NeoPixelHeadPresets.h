#pragma once

#include <stdint.h>

#include "NeoPixelHead.h"

namespace dukatimer {

/*
 * NeoPixelHeadTopologyPreset
 *
 * Beschreibt vordefinierte physische Topologien des Kopflichts. Diese Presets
 * legen nur Segmentanordnung und Pinbelegung fest, nicht die spaetere Nutzung
 * oder Belichtungslogik.
 */
enum class NeoPixelHeadTopologyPreset : uint8_t {
	SingleMatrix16x16,
	Quad8x8BringUp,
};

struct NeoPixelHeadPinout {
	uint8_t neo1Pin = 0xFF;
	uint8_t neo2Pin = 0xFF;
	uint8_t neo3Pin = 0xFF;
	uint8_t neo4Pin = 0xFF;
};

// Baut aus Topologie-Preset und konkreter Pinbelegung die logische Kopfkonfigu-
// ration fuer NeoPixelHead. Das eigentliche Segment-Mapping bleibt danach rein
// datengetrieben im Config-Objekt.
NeoPixelHeadConfig buildNeoPixelHeadConfig(NeoPixelHeadTopologyPreset preset, const NeoPixelHeadPinout& pinout);

}  // namespace dukatimer