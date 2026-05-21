/*
 * HeadSpectrumMapper
 *
 * Uebersetzt ein fachliches HeadSpectrumCommand in einen konkreten
 * LightRgb-Ausgabewert fuer den NeoPixel-Kopf.
 *
 * Zweck:
 *   Dieser Mapping-Rand trennt fotografische Semantik von physischen RGB-Werten.
 *   Er ist der einzige Ort, an dem HeadSpectrumCommand auf HeadCalibrationProfile
 *   trifft und ein physisch sendbarer Wert entsteht.
 *
 * Identity-Fallback (profile.calibrated == false):
 *   SplitgradeSoft  -> RGB(0, 255,   0)  reines Gruen, keine Regression
 *   SplitgradeHard  -> RGB(0,   0, 255)  reines Blau, keine Regression
 *   BwWhite         -> RGB(0, 255, 255)  Weisslicht FixedGrade
 *   BwGradeMix      -> Max-Normalisierung: staerkster Kanal = 255, anderer proportional
 *   LocalSafelight  -> RGB(255,   0,   0)
 *   LocalFocus      -> RGB(255, 255, 255)
 *   alle anderen    -> RGB(0, 0, 0)
 *
 * Kalibrierter Pfad (profile.calibrated == true):
 *   Nutzt LogicalChannelCalibration (driveCurve, rgbMix, maxNormalizedOutput)
 *   und GlobalRgbCalibration aus dem Profil. Identity-Abbildung bei Default-
 *   Profil bleibt erhalten, solange rgbMix die Kanalzuordnung korrekt spiegelt.
 *
 * Invarianten:
 *   - Ausgabewerte sind stets geklemmte uint8_t [0..255].
 *   - Bei degenerierten Mischverhaeltnissen (beide Kanaele 0) wird RGB(0,0,0)
 *     zurueckgegeben ohne Seiteneffekte.
 *   - SG-Sequenzfarben bleiben regressionsarm: SplitgradeSoft und SplitgradeHard
 *     produzieren im Identity-Fallback exakt dieselben Werte wie zuvor.
 */
#pragma once

#include "HeadCalibrationProfile.h"
#include "HeadLightCommand.h"
#include "HeadSpectrumCommand.h"

namespace dukatimer {

struct HeadSpectrumMapper {
	// Liefert den physischen LightRgb-Ausgabewert fuer das gegebene fachliche
	// Spektrumkommando unter Beruecksichtigung des Kopf-Kalibrierungsprofils.
	static LightRgb map(const HeadSpectrumCommand& cmd, const HeadCalibrationProfile& profile);
};

}  // namespace dukatimer
