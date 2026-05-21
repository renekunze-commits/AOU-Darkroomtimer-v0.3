#include <Arduino.h>
#include <algorithm>

#include "HeadSpectrumMapper.h"

namespace dukatimer {

namespace {

// Klemmt einen float-Wert auf [0..255] und wandelt ihn in uint8_t um.
static inline uint8_t clampToU8(float v) {
	if (v <= 0.0f) return 0;
	if (v >= 255.0f) return 255;
	return static_cast<uint8_t>(v + 0.5f);
}

// Addiert zwei geklemmte uint8_t-Kanalwerte (saettigend).
static inline uint8_t saturatingAdd(uint8_t a, uint8_t b) {
	const uint16_t sum = static_cast<uint16_t>(a) + static_cast<uint16_t>(b);
	return sum > 255u ? uint8_t(255) : static_cast<uint8_t>(sum);
}

// Wendet eine LogicalChannelCalibration auf eine normierte Eingangsintensitaet [0..1]
// an und liefert den physischen RGB-Beitrag zurueck.
static LightRgb applyChannelCalibration(const LogicalChannelCalibration& cal,
                                        float intensity,
                                        const GlobalRgbCalibration& globalRgb,
                                        float globalOutputLimit) {
	if (intensity <= 0.0f) {
		return LightRgb{};
	}
	const float clamped = intensity > 1.0f ? 1.0f : intensity;

	// Drive-Kurve: Eingangsindex 0..255
	const uint8_t driveIndex = static_cast<uint8_t>(clamped * 255.0f + 0.5f);
	const float drivenOutput =
		static_cast<float>(cal.driveCurve.lut[driveIndex]) / 255.0f;

	const float limitedOutput = drivenOutput * cal.maxNormalizedOutput * globalOutputLimit;

	return LightRgb{
		clampToU8(limitedOutput * cal.rgbMix.red   * globalRgb.redScale   * 255.0f),
		clampToU8(limitedOutput * cal.rgbMix.green * globalRgb.greenScale * 255.0f),
		clampToU8(limitedOutput * cal.rgbMix.blue  * globalRgb.blueScale  * 255.0f),
	};
}

// Addiert zwei RGB-Betraege kanalweise mit Saettigung.
static LightRgb addRgb(const LightRgb& a, const LightRgb& b) {
	return LightRgb{
		saturatingAdd(a.red,   b.red),
		saturatingAdd(a.green, b.green),
		saturatingAdd(a.blue,  b.blue),
	};
}

// --- Identity-Fallback (profile.calibrated == false) ---

FLASHMEM static LightRgb mapIdentity(const HeadSpectrumCommand& cmd) {
	switch (cmd.semantic) {
		case HeadSpectrumSemantic::SplitgradeSoft:
			// Reines Gruen - identisches Verhalten wie vor Einfuehrung des Mappers.
			return makeLightRgb(0, 255, 0);

		case HeadSpectrumSemantic::SplitgradeHard:
			// Reines Blau - identisches Verhalten wie vor Einfuehrung des Mappers.
			return makeLightRgb(0, 0, 255);

		case HeadSpectrumSemantic::BwWhite:
			// FixedGrade-Weisslicht: Soft (Gruen) + Hard (Blau) beide voll.
			return makeLightRgb(0, 255, 255);

		case HeadSpectrumSemantic::BwGradeMix: {
			// Max-Normalisierung: staerkster Kanal auf 255, anderer proportional.
			// Erhaelt das Spektralverhaeltnis und vermeidet unnoetige Verdunkelung.
			const float soft = cmd.channels.soft;
			const float hard = cmd.channels.hard;
			const float maxMix = soft > hard ? soft : hard;
			if (maxMix < 1e-6f) {
				return LightRgb{};
			}
			const float scale = 255.0f / maxMix;
			return makeLightRgb(0, clampToU8(soft * scale), clampToU8(hard * scale));
		}

		case HeadSpectrumSemantic::LocalSafelight:
			return makeLightRgb(255, 0, 0);

		case HeadSpectrumSemantic::LocalFocus:
			return makeLightRgb(255, 255, 255);

		case HeadSpectrumSemantic::Off:
		case HeadSpectrumSemantic::Burn:
		case HeadSpectrumSemantic::TestStrip:
		case HeadSpectrumSemantic::Preflash:
		case HeadSpectrumSemantic::Calibration:
		case HeadSpectrumSemantic::DiagnosticPattern:
		case HeadSpectrumSemantic::CustomLogicalMix:
		default:
			return LightRgb{};
	}
}

// --- Kalibrierter Pfad (profile.calibrated == true) ---

FLASHMEM static LightRgb mapCalibrated(const HeadSpectrumCommand& cmd,
                                       const HeadCalibrationProfile& profile) {
	const GlobalRgbCalibration& globalRgb = profile.globalRgb;
	const float globalLimit = profile.globalOutputLimit;

	switch (cmd.semantic) {
		case HeadSpectrumSemantic::SplitgradeSoft:
			return applyChannelCalibration(profile.soft, cmd.channels.soft > 0.0f ? cmd.channels.soft : 1.0f,
			                              globalRgb, globalLimit);

		case HeadSpectrumSemantic::SplitgradeHard:
			return applyChannelCalibration(profile.hard, cmd.channels.hard > 0.0f ? cmd.channels.hard : 1.0f,
			                              globalRgb, globalLimit);

		case HeadSpectrumSemantic::BwWhite: {
			// Beide Kanaele voll, additiv gemischt.
			const LightRgb softColor =
				applyChannelCalibration(profile.soft, 1.0f, globalRgb, globalLimit);
			const LightRgb hardColor =
				applyChannelCalibration(profile.hard, 1.0f, globalRgb, globalLimit);
			return addRgb(softColor, hardColor);
		}

		case HeadSpectrumSemantic::BwGradeMix: {
			const LightRgb softColor =
				applyChannelCalibration(profile.soft, cmd.channels.soft, globalRgb, globalLimit);
			const LightRgb hardColor =
				applyChannelCalibration(profile.hard, cmd.channels.hard, globalRgb, globalLimit);
			return addRgb(softColor, hardColor);
		}

		case HeadSpectrumSemantic::LocalSafelight:
			return applyChannelCalibration(profile.safelightRed, 1.0f, globalRgb, globalLimit);

		case HeadSpectrumSemantic::LocalFocus:
			return applyChannelCalibration(profile.focusWhite, 1.0f, globalRgb, globalLimit);

		case HeadSpectrumSemantic::Off:
		case HeadSpectrumSemantic::Burn:
		case HeadSpectrumSemantic::TestStrip:
		case HeadSpectrumSemantic::Preflash:
		case HeadSpectrumSemantic::Calibration:
		case HeadSpectrumSemantic::DiagnosticPattern:
		case HeadSpectrumSemantic::CustomLogicalMix:
		default:
			return LightRgb{};
	}
}

}  // namespace

// --- Oeffentliche Schnittstelle ---

FLASHMEM LightRgb HeadSpectrumMapper::map(const HeadSpectrumCommand& cmd,
                                          const HeadCalibrationProfile& profile) {
	if (!profile.calibrated) {
		return mapIdentity(cmd);
	}
	return mapCalibrated(cmd, profile);
}

}  // namespace dukatimer
