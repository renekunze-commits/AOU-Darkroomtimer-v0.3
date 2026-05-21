/*
 * HeadCalibrationProfile
 *
 * Version:
 * - Architektur-Schritt 1, eingefuehrt am 2026-04-24
 * - Schema-Version 1 fuer das hardwarebezogene Kalibriermodell
 *
 * Zweck:
 * - bildet fachliche Lichtkanaele auf korrigierte RGB-Ausgaenge des realen
 *   16x16-NeoPixel-Kopfes ab
 * - trennt globale Gamma- und RGB-Korrektur explizit vom Papiermodell
 *
 * Umfang in dieser ersten Fassung:
 * - gemeinsamer Gamma- oder Drive-Pfad fuer den 8-Bit-Ausgangsraum
 * - globaler spektraler RGB-Abgleich
 * - logische Kanalzuordnung fuer Safelight, Focus, Soft und Hard
 *
 * Bewusste Nicht-Zustaendigkeit:
 * - keine papierbezogenen K-Faktoren
 * - keine D-logH- oder Splitgrade-Regeln
 * - keine Laufzeit-Schutzentscheidung der ExposureEngine
 */
#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kHeadCalibrationProfileSchemaVersion = 1;
constexpr uint16_t kDriveCurveEntryCount = 256;

struct PhysicalRgbMix {
	float red = 0.0f;
	float green = 0.0f;
	float blue = 0.0f;

	bool operator==(const PhysicalRgbMix& other) const {
		return red == other.red && green == other.green && blue == other.blue;
	}

	bool operator!=(const PhysicalRgbMix& other) const {
		return !(*this == other);
	}
};

struct ChannelDriveCurve {
	uint8_t lut[kDriveCurveEntryCount] = {};

	ChannelDriveCurve() {
		for (uint16_t index = 0; index < kDriveCurveEntryCount; ++index) {
			lut[index] = static_cast<uint8_t>(index);
		}
	}

	bool operator==(const ChannelDriveCurve& other) const {
		for (uint16_t index = 0; index < kDriveCurveEntryCount; ++index) {
			if (lut[index] != other.lut[index]) {
				return false;
			}
		}
		return true;
	}

	bool operator!=(const ChannelDriveCurve& other) const {
		return !(*this == other);
	}
};

struct GlobalRgbCalibration {
	float redScale = 1.0f;
	float greenScale = 1.0f;
	float blueScale = 1.0f;

	bool operator==(const GlobalRgbCalibration& other) const {
		return redScale == other.redScale && greenScale == other.greenScale &&
		       blueScale == other.blueScale;
	}

	bool operator!=(const GlobalRgbCalibration& other) const {
		return !(*this == other);
	}
};

struct LogicalChannelCalibration {
	PhysicalRgbMix rgbMix = {};
	ChannelDriveCurve driveCurve = {};
	float maxNormalizedOutput = 1.0f;

	bool operator==(const LogicalChannelCalibration& other) const {
		return rgbMix == other.rgbMix && driveCurve == other.driveCurve &&
		       maxNormalizedOutput == other.maxNormalizedOutput;
	}

	bool operator!=(const LogicalChannelCalibration& other) const {
		return !(*this == other);
	}
};

struct HeadCalibrationProfile {
	uint16_t schemaVersion = kHeadCalibrationProfileSchemaVersion;
	bool calibrated = false;
	float globalOutputLimit = 1.0f;
	ChannelDriveCurve sharedGammaCurve = {};
	GlobalRgbCalibration globalRgb = {};

	LogicalChannelCalibration safelightRed = {};
	LogicalChannelCalibration focusWhite = {};
	LogicalChannelCalibration soft = {};
	LogicalChannelCalibration hard = {};

	bool operator==(const HeadCalibrationProfile& other) const {
		return schemaVersion == other.schemaVersion && calibrated == other.calibrated &&
		       globalOutputLimit == other.globalOutputLimit &&
		       sharedGammaCurve == other.sharedGammaCurve && globalRgb == other.globalRgb &&
		       safelightRed == other.safelightRed && focusWhite == other.focusWhite &&
		       soft == other.soft && hard == other.hard;
	}

	bool operator!=(const HeadCalibrationProfile& other) const {
		return !(*this == other);
	}
};

}  // namespace dukatimer