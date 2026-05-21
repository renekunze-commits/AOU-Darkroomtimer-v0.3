/*
 * NeoPixelHeadPresets
 *
 * Diese Datei kapselt die bekannten Bring-up-Topologien des Lichtkopfs. Sie ist
 * bewusst klein gehalten, damit sich physische Segmentlayouts an einer Stelle
 * aendern lassen, ohne Render- oder Belichtungslogik anzufassen.
 */

#include "NeoPixelHeadPresets.h"

namespace dukatimer {

namespace {

NeoPixelSegmentConfig makeSegmentConfig(uint8_t pin, uint8_t originX, uint8_t originY, uint8_t width, uint8_t height) {
	// makeSegmentConfig() erzeugt eine einheitliche Basiskonfiguration fuer ein
	// Segment; Rotation und ZigZag sind explizit Teil des Datenmodells und nicht
	// als versteckte Verdrahtungsannahme im Hauptcode verstreut.
	NeoPixelSegmentConfig segment;
	segment.pin = pin;
	segment.originX = originX;
	segment.originY = originY;
	segment.width = width;
	segment.height = height;
	// Diese Bring-up-Voreinstellung ist nur ein Startpunkt fuer das Mapping.
	// Die reale Orientierung jedes Segments muss anschliessend mit den
	// Testmustern verifiziert und bei Bedarf angepasst werden.
	segment.zigZagRows = true;
	segment.rotation = NeoPixelRotation::Deg0;
	return segment;
}

}  // namespace

NeoPixelHeadConfig buildNeoPixelHeadConfig(NeoPixelHeadTopologyPreset preset, const NeoPixelHeadPinout& pinout) {
	// Das Ergebnis ist eine vollstaendige logische Matrixbeschreibung, die spaeter
	// von NeoPixelHead unabhaengig vom gewaehlten Bring-up-Preset verwendet wird.
	NeoPixelHeadConfig config;
	config.width = 16;
	config.height = 16;

	switch (preset) {
	case NeoPixelHeadTopologyPreset::SingleMatrix16x16:
		config.segmentCount = 1;
		config.segments[0] = makeSegmentConfig(pinout.neo1Pin, 0, 0, 16, 16);
		break;
	case NeoPixelHeadTopologyPreset::Quad8x8BringUp:
		config.segmentCount = 4;
		config.segments[0] = makeSegmentConfig(pinout.neo1Pin, 0, 0, 8, 8);
		config.segments[1] = makeSegmentConfig(pinout.neo2Pin, 8, 0, 8, 8);
		config.segments[2] = makeSegmentConfig(pinout.neo3Pin, 0, 8, 8, 8);
		config.segments[3] = makeSegmentConfig(pinout.neo4Pin, 8, 8, 8, 8);
		break;
	}

	return config;
}

}  // namespace dukatimer