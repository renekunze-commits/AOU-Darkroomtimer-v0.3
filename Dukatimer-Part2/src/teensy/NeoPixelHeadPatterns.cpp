/*
 * NeoPixelHeadPatterns
 *
 * Diese Datei enthaelt rein diagnostische Renderhilfen fuer den Lichtkopf.
 * Die Muster sind absichtlich simpel und deterministisch, damit Segmentgrenzen,
 * Matrixecken und Farbkanalzuordnung ohne Workflow- oder UI-Einfluss geprueft
 * werden koennen.
 */

#include "NeoPixelHeadPatterns.h"

namespace dukatimer {

namespace {

constexpr LightRgb kOff = {0, 0, 0};
constexpr LightRgb kWhite = {255, 255, 255};
constexpr LightRgb kRed = {255, 0, 0};
constexpr LightRgb kGreen = {0, 255, 0};
constexpr LightRgb kBlue = {0, 0, 255};
constexpr LightRgb kSegmentBaseColors[4] = {
	{32, 0, 0},
	{0, 32, 0},
	{0, 0, 32},
	{32, 16, 0},
};

// Fuelle den kompletten logischen Bereich eines Segments einfarbig. Dadurch
// werden Segmentlage und Segmentgroesse sichtbar, ohne dass das physische
// Pixel-Mapping hier nochmals dupliziert werden muss.
void fillSegment(NeoPixelHead& head, const NeoPixelSegmentConfig& segment, const LightRgb& color) {
	for (uint8_t y = 0; y < segment.height; ++y) {
		for (uint8_t x = 0; x < segment.width; ++x) {
			head.setPixel(static_cast<uint8_t>(segment.originX + x), static_cast<uint8_t>(segment.originY + y), color);
		}
	}
}

void markSegmentCorners(NeoPixelHead& head, const NeoPixelSegmentConfig& segment) {
	// Vier feste Eckfarben erlauben eine schnelle Sichtpruefung von Rotation,
	// Spiegelung und ZigZag-Annahme eines Segments.
	if (segment.width == 0 || segment.height == 0) {
		return;
	}

	const uint8_t left = segment.originX;
	const uint8_t top = segment.originY;
	const uint8_t right = static_cast<uint8_t>(segment.originX + segment.width - 1u);
	const uint8_t bottom = static_cast<uint8_t>(segment.originY + segment.height - 1u);

	head.setPixel(left, top, kWhite);
	head.setPixel(right, top, kRed);
	head.setPixel(right, bottom, kGreen);
	head.setPixel(left, bottom, kBlue);
}

}  // namespace

void applyNeoPixelHeadTestPattern(NeoPixelHead& head, const NeoPixelHeadConfig& config, NeoPixelHeadTestPattern pattern) {
	// Jedes Muster beginnt mit einem definierten leeren Framebuffer. So bleibt die
	// Diagnose frei von Resten eines vorangegangenen Belichtungs- oder Testzustands.
	head.clear();

	switch (pattern) {
	case NeoPixelHeadTestPattern::Disabled:
		break;
	case NeoPixelHeadTestPattern::MatrixCorners:
		if (head.width() == 0 || head.height() == 0) {
			break;
		}
		head.setPixel(0, 0, kWhite);
		head.setPixel(static_cast<uint8_t>(head.width() - 1u), 0, kRed);
		head.setPixel(static_cast<uint8_t>(head.width() - 1u), static_cast<uint8_t>(head.height() - 1u), kGreen);
		head.setPixel(0, static_cast<uint8_t>(head.height() - 1u), kBlue);
		break;
	case NeoPixelHeadTestPattern::SegmentSolidColors:
		for (uint8_t segmentIndex = 0; segmentIndex < config.segmentCount && segmentIndex < 4; ++segmentIndex) {
			fillSegment(head, config.segments[segmentIndex], kSegmentBaseColors[segmentIndex]);
		}
		break;
	case NeoPixelHeadTestPattern::SegmentCornerMarkers:
		for (uint8_t segmentIndex = 0; segmentIndex < config.segmentCount && segmentIndex < 4; ++segmentIndex) {
			fillSegment(head, config.segments[segmentIndex], kSegmentBaseColors[segmentIndex]);
			markSegmentCorners(head, config.segments[segmentIndex]);
		}
		break;
	}

	head.present();
}

}  // namespace dukatimer