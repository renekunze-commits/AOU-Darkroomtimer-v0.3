#pragma once

#include <stdint.h>

#include "NeoPixelHead.h"

namespace dukatimer {

/*
 * NeoPixelHeadTestPattern
 *
 * Kleine Bring-up-Muster fuer den Lichtkopf. Sie dienen ausschliesslich der
 * Verifikation von Segmentlage, Orientierung und Farbkanalzuordnung und sind
 * bewusst getrennt von der produktiven Belichtungslogik.
 */
enum class NeoPixelHeadTestPattern : uint8_t {
	Disabled,
	MatrixCorners,
	SegmentSolidColors,
	SegmentCornerMarkers,
};

// Rendert ein explizites Diagnosemuster direkt auf den logischen Kopflicht-
// Framebuffer und praesentiert es sofort. Der normale Head-Arbiter wird dabei
// bewusst umgangen.
void applyNeoPixelHeadTestPattern(NeoPixelHead& head, const NeoPixelHeadConfig& config, NeoPixelHeadTestPattern pattern);

}  // namespace dukatimer