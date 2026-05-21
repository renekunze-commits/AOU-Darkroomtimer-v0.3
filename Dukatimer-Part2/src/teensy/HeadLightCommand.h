/*
 * HeadLightCommand
 *
 * Kleines physisches Zielmodell fuer den aktuellen Kopflichtausgang. Diese
 * Struktur liegt unterhalb von Workflow, ExposureEngine und Semantikmodellen
 * wie HeadSpectrumCommand und wird direkt von Arbiter und NeoPixelHead genutzt.
 */

#pragma once

#include <stdint.h>

namespace dukatimer {

// Direkter 8-Bit-RGB-Ausgabewert fuer den Kopflichtpfad.
struct LightRgb {
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blue = 0;

	bool operator==(const LightRgb& other) const {
		return red == other.red && green == other.green && blue == other.blue;
	}

	bool operator!=(const LightRgb& other) const {
		return !(*this == other);
	}
};

// Kleinster aktuell unterstuetzter physischer Lichtmodus des Kopfes.
enum class HeadLightMode : uint8_t {
	Off,
	SolidColor,
};

// Konkret auszugebendes Kopflichtkommando nach Arbitration und Ableitung.
struct HeadLightCommand {
	HeadLightMode mode = HeadLightMode::Off;
	LightRgb color = {};

	bool operator==(const HeadLightCommand& other) const {
		return mode == other.mode && color == other.color;
	}

	bool operator!=(const HeadLightCommand& other) const {
		return !(*this == other);
	}
};

// Quelle eines Kopflichtkommandos inklusive explizitem Aktiv-Bit. Das erlaubt,
// dass ein aktiver OFF-Zustand weiterhin lokale Quellen uebersteuern kann.
struct HeadLightSourceState {
	bool active = false;
	HeadLightCommand command = {};

	bool operator==(const HeadLightSourceState& other) const {
		return active == other.active && command == other.command;
	}

	bool operator!=(const HeadLightSourceState& other) const {
		return !(*this == other);
	}
};

inline LightRgb makeLightRgb(uint8_t red, uint8_t green, uint8_t blue) {
	LightRgb color;
	color.red = red;
	color.green = green;
	color.blue = blue;
	return color;
}

inline HeadLightCommand makeHeadLightOff() {
	return HeadLightCommand{};
}

inline HeadLightCommand makeHeadLightSolid(const LightRgb& color) {
	HeadLightCommand command;
	command.mode = HeadLightMode::SolidColor;
	command.color = color;
	return command;
}

inline HeadLightSourceState makeInactiveHeadLightSource() {
	return HeadLightSourceState{};
}

inline HeadLightSourceState makeActiveHeadLightSource(const HeadLightCommand& command) {
	HeadLightSourceState source;
	source.active = true;
	source.command = command;
	return source;
}

}  // namespace dukatimer