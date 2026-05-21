/*
 * HeadSpectrumCommand
 *
 * Version:
 * - Architektur-Schritt 1, eingefuehrt am 2026-04-24
 * - Schema-Version 1 fuer das fachliche Spektrummodell
 *
 * Zweck:
 * - beschreibt das fachlich gewuenschte Licht des Kopfes
 * - bleibt vollstaendig unabhaengig von NeoPixel-Pinout, Matrix-Topologie
 *   und elektrischen RGB-Rohwerten
 *
 * Bewusste Nicht-Zustaendigkeit:
 * - keine Papier-Kalibrierung
 * - keine Kopf-Kalibrierung
 * - keine Zeit- oder Dosisfuehrung
 * - keine Treiber- oder Busentscheidungen
 *
 * Integrationsstatus:
 * - in Schritt 1 absichtlich nur als neutraler Modelltpy eingefuehrt
 * - noch nicht in LightController, ExposureEngine oder NeoPixelHead verdrahtet
 */
#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kHeadSpectrumCommandSchemaVersion = 1;

struct LogicalHeadChannels {
	float safelightRed = 0.0f;
	float focusWhite = 0.0f;
	float soft = 0.0f;
	float hard = 0.0f;

	bool operator==(const LogicalHeadChannels& other) const {
		return safelightRed == other.safelightRed && focusWhite == other.focusWhite &&
		       soft == other.soft && hard == other.hard;
	}

	bool operator!=(const LogicalHeadChannels& other) const {
		return !(*this == other);
	}
};

enum class HeadSpectrumSemantic : uint8_t {
	Off,
	LocalSafelight,
	LocalFocus,
	BwWhite,       // Volles Weißlicht fuer FixedGrade-Papiere (Soft+Hard gleichzeitig voll)
	BwGradeMix,    // Simultaner Soft/Hard-Mix gemaess Gradationseinstellung
	SplitgradeSoft,
	SplitgradeHard,
	Burn,
	TestStrip,
	Preflash,
	Calibration,
	DiagnosticPattern,
	CustomLogicalMix,
};

struct HeadSpectrumCommand {
	HeadSpectrumSemantic semantic = HeadSpectrumSemantic::Off;
	LogicalHeadChannels channels = {};
	float masterIntensity = 1.0f;
	float grade = 2.5f;

	bool operator==(const HeadSpectrumCommand& other) const {
		return semantic == other.semantic && channels == other.channels &&
		       masterIntensity == other.masterIntensity && grade == other.grade;
	}

	bool operator!=(const HeadSpectrumCommand& other) const {
		return !(*this == other);
	}
};

inline HeadSpectrumCommand makeHeadSpectrumOff() {
	return HeadSpectrumCommand{};
}

}  // namespace dukatimer