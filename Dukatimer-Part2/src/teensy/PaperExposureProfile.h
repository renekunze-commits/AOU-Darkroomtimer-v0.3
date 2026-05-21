/*
 * PaperExposureProfile
 *
 * Version:
 * - Architektur-Schritt 1, eingefuehrt am 2026-04-24
 * - Schema-Version 1 fuer das papierbezogene Belichtungsmodell
 *
 * Zweck:
 * - haelt papierbezogene K-Faktoren, Splitgrade-Daten und Preflash-Parameter
 * - bleibt strikt getrennt von Kopfverdrahtung, Gamma-Korrektur und NeoPixel-
 *   Ausgabedetails
 *
 * Historische Einordnung:
 * - dies ist der Teil des Zielmodells, der am staerksten an v0.3 und v0.9
 *   anknuepft
 * - in Schritt 1 wird nur die Struktur eingezogen, noch keine Mathematik
 *   portiert
 */
#pragma once

#include <stdint.h>

namespace dukatimer {

constexpr uint16_t kPaperExposureProfileSchemaVersion = 1;
constexpr uint8_t kSplitgradeStepCount = 11;
constexpr uint8_t kPaperProfileNameCapacity = 24;

enum class PaperGradeMode : uint8_t {
	Multigrade = 0u,
	FixedGrade = 1u,
};

constexpr bool isValidPaperGradeMode(PaperGradeMode gradeMode) {
	return gradeMode == PaperGradeMode::Multigrade || gradeMode == PaperGradeMode::FixedGrade;
}

constexpr bool isFixedGradeMode(PaperGradeMode gradeMode) {
	return gradeMode == PaperGradeMode::FixedGrade;
}

struct PreflashSettings {
	bool calibrated = false;
	bool enabled = false;
	float thresholdSeconds = 0.0f;
	float factor = 1.0f;
	uint8_t level = 0;
	uint8_t colorMode = 0;

	bool operator==(const PreflashSettings& other) const {
		return calibrated == other.calibrated && enabled == other.enabled &&
		       thresholdSeconds == other.thresholdSeconds && factor == other.factor &&
		       level == other.level && colorMode == other.colorMode;
	}

	bool operator!=(const PreflashSettings& other) const {
		return !(*this == other);
	}
};

struct PaperExposureProfile {
	uint16_t schemaVersion = kPaperExposureProfileSchemaVersion;
	char name[kPaperProfileNameCapacity] = {};
	bool calibrated = false;
	// Dieser Vertrag benennt die Papierart explizit, damit Persistenz,
	// Workflow und Snapshot/UI denselben Typ fuer fixed/multigrade teilen.
	PaperGradeMode gradeMode = PaperGradeMode::Multigrade;
	bool useIsoMath = false;

	float fixedGradeValue = 2.5f;
	float isoP = 100.0f;
	float isoR = 100.0f;

	// kBw: Dimensionsloser, statischer Transmissionsfaktor des Weißpunkts
	// (10^-D_N, Wertebereich offen (0,1)). Kalibrierungskonstante des Profils.
	// DARF NICHT als temporaerer Speicher fuer die benutzerseitige
	// Belichtungszeit verwendet werden! (E17)
	float kBw = 0.0f;
	float kSoft = 0.0f;
	float kHard = 0.0f;

	float gradeKSoft[kSplitgradeStepCount] = {};
	float gradeKHard[kSplitgradeStepCount] = {};

	PreflashSettings preflash = {};

	bool operator==(const PaperExposureProfile& other) const {
		if (schemaVersion != other.schemaVersion || calibrated != other.calibrated ||
		    gradeMode != other.gradeMode || useIsoMath != other.useIsoMath ||
		    fixedGradeValue != other.fixedGradeValue || isoP != other.isoP ||
		    isoR != other.isoR || kBw != other.kBw || kSoft != other.kSoft ||
		    kHard != other.kHard || preflash != other.preflash) {
			return false;
		}

		for (uint8_t index = 0; index < kPaperProfileNameCapacity; ++index) {
			if (name[index] != other.name[index]) {
				return false;
			}
		}

		for (uint8_t index = 0; index < kSplitgradeStepCount; ++index) {
			if (gradeKSoft[index] != other.gradeKSoft[index] ||
			    gradeKHard[index] != other.gradeKHard[index]) {
				return false;
			}
		}

		return true;
	}

	bool operator!=(const PaperExposureProfile& other) const {
		return !(*this == other);
	}
};

}  // namespace dukatimer