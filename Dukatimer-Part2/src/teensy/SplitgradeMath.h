#pragma once

/*
 * SplitgradeMath.h
 *
 * Gemeinsame Inline-Mathematik fuer SplitgradeWorkflow und BlackWhiteWorkflow.
 * Alle Funktionen sind zustandslos und haengen nur vom Papierprofil und der
 * Gradation ab. FLASHMEM wird hier nicht gesetzt, da die Funktionen als
 * Inline-Helfer in FLASHMEM-dekorierten Aufrufer-Methoden eingebettet werden.
 */

#include <stdint.h>

#include "PaperExposureProfile.h"

namespace dukatimer {
namespace splitgrade_math {

constexpr float kSplitTargetPresenceEpsilon = 0.05f;
constexpr float kIsoReferencePaperSpeed    = 100.0f;
constexpr float kIsoReferenceRange         = 100.0f;
constexpr float kIsoMinimumUsableValue     = 10.0f;
constexpr float kIsoMaximumUsableValue     = 10000.0f;
constexpr float kIsoMinimumRangeBias       = 0.25f;
constexpr float kIsoMaximumRangeBias       = 4.0f;

// Liefert den ganzzahligen Gradationsindex (0..10) fuer einen Float-Gradwert.
// 0 = Grad 0.0, 10 = Grad 5.0, in 0.5er-Schritten.
inline int gradeIndexFromFloat(float grade) {
	const float clamped = (grade < 0.0f) ? 0.0f : (grade > 5.0f) ? 5.0f : grade;
	int index = static_cast<int>((clamped * 2.0f) + 0.1f);
	if (index < 0)  return 0;
	if (index > 10) return 10;
	return index;
}

// Quantisiert einen Float-Gradwert auf den naechsten 0.5er-Schritt (0.0..5.0).
inline float normalizeWorkflowGrade(float grade) {
	return static_cast<float>(gradeIndexFromFloat(grade)) * 0.5f;
}

inline float clampUnit(float value) {
	if (value < 0.0f) return 0.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

// Prueft ob ein ISO-Parameter plausibel ist; gibt `fallback` zurueck wenn nicht.
inline float sanitizeIsoParameter(float value, float fallback) {
	if (value == value && value > kIsoMinimumUsableValue && value < kIsoMaximumUsableValue) {
		return value;
	}
	return fallback;
}

// Liefert den ISO-Skalierungsfaktor fuer ein Profil (Papierspeed-Anpassung).
inline float isoTargetScaleForProfile(const PaperExposureProfile& profile) {
	if (!profile.useIsoMath || isFixedGradeMode(profile.gradeMode)) {
		return 1.0f;
	}
	const float paperSpeed = sanitizeIsoParameter(profile.isoP, kIsoReferencePaperSpeed);
	return kIsoReferencePaperSpeed / paperSpeed;
}

// Berechnet den Hard-Anteil fuer die gegebene Gradation gemaess ISO-Modell.
// Rueckgabewert: 0.0 (alle Soft) .. 1.0 (alle Hard).
inline float isoHardFractionForGrade(const PaperExposureProfile& profile, float grade) {
	const float linearHardFraction = clampUnit(grade / 5.0f);
	if (linearHardFraction <= 0.0f || linearHardFraction >= 1.0f) {
		return linearHardFraction;
	}

	const float paperRange = sanitizeIsoParameter(profile.isoR, kIsoReferenceRange);
	float rangeBias = paperRange / kIsoReferenceRange;
	if (rangeBias < kIsoMinimumRangeBias) rangeBias = kIsoMinimumRangeBias;
	else if (rangeBias > kIsoMaximumRangeBias) rangeBias = kIsoMaximumRangeBias;

	const float softWeight = 1.0f - linearHardFraction;
	const float hardWeight = linearHardFraction * rangeBias;
	const float totalWeight = softWeight + hardWeight;
	if (totalWeight <= kSplitTargetPresenceEpsilon) {
		return linearHardFraction;
	}
	return clampUnit(hardWeight / totalWeight);
}

// Liefert den Soft-Anteil aus der Profil-LUT fuer einen gegebenen Gradationsindex.
// Kein Override-Support (fuer SG-interne Override-Logik: SplitgradeWorkflow verwenden).
// Verwendbar fuer BW-Workflow und als Fallback ohne Override-Kontext.
inline float profileLutSoftFraction(const PaperExposureProfile& profile, uint8_t gradeIndex) {
	const uint8_t idx = (gradeIndex < kSplitgradeStepCount)
	    ? gradeIndex : static_cast<uint8_t>(kSplitgradeStepCount - 1u);

	if (profile.useIsoMath) {
		const float grade = static_cast<float>(idx) * 0.5f;
		return 1.0f - isoHardFractionForGrade(profile, grade);
	}

	float softFactor = profile.gradeKSoft[idx];
	float hardFactor = profile.gradeKHard[idx];
	if (profile.kSoft > kSplitTargetPresenceEpsilon) softFactor /= profile.kSoft;
	if (profile.kHard > kSplitTargetPresenceEpsilon) hardFactor /= profile.kHard;

	if (softFactor <= kSplitTargetPresenceEpsilon && hardFactor <= kSplitTargetPresenceEpsilon) {
		hardFactor = static_cast<float>(idx) / 10.0f;
		softFactor = 1.0f - hardFactor;
	}

	const float total = softFactor + hardFactor;
	if (total <= kSplitTargetPresenceEpsilon) {
		return 1.0f - (static_cast<float>(idx) / 10.0f);
	}
	return clampUnit(softFactor / total);
}

}  // namespace splitgrade_math
}  // namespace dukatimer
