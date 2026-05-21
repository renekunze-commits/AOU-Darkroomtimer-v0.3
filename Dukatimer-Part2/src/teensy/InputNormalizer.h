#pragma once

#include "InputSemanticAction.h"
#include "NormalizedInputEvent.h"

namespace dukatimer {

/*
 * InputNormalizer
 *
 * Zweck:
 * - uebersetzt bereits hardwareunabhaengige NormalizedInputEvents in die noch
 *   abstraktere Fachbedeutung fuer Workflows
 * - haelt die Zuordnung von Encoder-/Tasterquelle zu semantischer Aktion an
 *   genau einer Stelle zentral zusammen
 *
 * Architekturgrenze:
 * - kennt weder Moduszustand noch Hardwaredetails
 * - beantwortet nur die Frage: "Welche Absicht steckt in diesem Event?"
 */
class InputNormalizer {
public:
	InputSemanticAction normalize(const NormalizedInputEvent& event) const;
};

}  // namespace dukatimer