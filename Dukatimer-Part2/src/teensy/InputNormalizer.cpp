/*
 * InputNormalizer
 *
 * Diese Datei bildet die feste Zuordnung zwischen neutralen Eingabeereignissen
 * und Workflow-Aktionen. Dadurch muessen spaetere Modi nicht wissen, ob eine
 * Aktion von einem lokalen Encoder, einem Remote-Encoder oder einem Wireless-
 * Taster ausgeloest wurde.
 */

#include "InputNormalizer.h"

#include "UiModalAction.h"

namespace dukatimer {

InputSemanticAction InputNormalizer::normalize(const NormalizedInputEvent& event) const {
	if (!event.isMeaningful()) {
		return InputSemanticAction{};
	}

	// Zeitstempel und Sequenz des Eingabeevents werden bewusst in die semantische
	// Aktion uebernommen, damit nachgelagerte Workflows dieselbe zeitliche Herkunft
	// sehen wie im Rohereignis.
	const auto makeAction = [&](InputSemanticActionKind kind, int16_t value) {
		return makeInputSemanticAction(kind, value, event.sourceTimestampMs, event.sequence);
	};

	// Einige Ereignisse sind bereits semantisch eindeutig und werden direkt
	// durchgereicht, noch bevor quellspezifische Rollen geprueft werden.
	if (event.eventKind == NormalizedInputEventKind::Measure) {
		return makeAction(InputSemanticActionKind::Measure, event.value);
	}

	if (event.eventKind == NormalizedInputEventKind::Undo) {
		return makeAction(InputSemanticActionKind::Undo, event.value);
	}

	if (event.source == NormalizedInputSource::LocalStartButton) {
		// Der lokale Starttaster bildet immer dieselbe Fachabsicht ab, egal ob die
		// Geste als Press, LongPress oder RepeatPress hereinkommt.
		if (event.eventKind == NormalizedInputEventKind::Press ||
		    event.eventKind == NormalizedInputEventKind::LongPress ||
		    event.eventKind == NormalizedInputEventKind::RepeatPress) {
			return makeAction(InputSemanticActionKind::Start, 1);
		}
	}

	switch (event.source) {
		case NormalizedInputSource::LocalEncoder1:
			// Encoder 1 bedient immer den primaeren Wert der aktiven Eingabemaske.
			if (event.eventKind == NormalizedInputEventKind::RotateLeft) {
				return makeAction(InputSemanticActionKind::AdjustPrimaryDecrease, -1);
			}
			if (event.eventKind == NormalizedInputEventKind::RotateRight) {
				return makeAction(InputSemanticActionKind::AdjustPrimaryIncrease, 1);
			}
			break;

		case NormalizedInputSource::LocalEncoder2:
			// Encoder 2 ist der sekundaere Stellpfad und bleibt damit parallel zum
			// historischen Rollenbild von Soft/Hard- oder Unterwert-Anpassungen.
			if (event.eventKind == NormalizedInputEventKind::RotateLeft) {
				return makeAction(InputSemanticActionKind::AdjustSecondaryDecrease, -1);
			}
			if (event.eventKind == NormalizedInputEventKind::RotateRight) {
				return makeAction(InputSemanticActionKind::AdjustSecondaryIncrease, 1);
			}
			break;

		case NormalizedInputSource::LocalEncoder3:
		case NormalizedInputSource::RemoteEncoder4:
		case NormalizedInputSource::WirelessEncoder:
			// Lokaler Kontextencoder, entfernter Encoder 4 und Wireless-Encoder teilen
			// sich absichtlich dieselbe Semantik: Navigation und Bestaetigung statt
			// direkter Parameterveraenderung.
			if (event.eventKind == NormalizedInputEventKind::RotateLeft) {
				return makeAction(InputSemanticActionKind::ContextPrevious, -1);
			}
			if (event.eventKind == NormalizedInputEventKind::RotateRight) {
				return makeAction(InputSemanticActionKind::ContextNext, 1);
			}
			if (event.eventKind == NormalizedInputEventKind::Press ||
			    event.eventKind == NormalizedInputEventKind::RepeatPress) {
				return makeAction(InputSemanticActionKind::Confirm, 1);
			}
			break;

		case NormalizedInputSource::LocalModalButton:
			// Globale Modal-Buttons werden als ein einziger lokaler Rohkanal erfasst.
			// Die konkrete Fachabsicht steckt im Event-Value und bleibt so fuer kuenftige
			// Overlays wie Pause/Resume/Abort erweiterbar, ohne weitere Sonderquellen.
			if (event.eventKind == NormalizedInputEventKind::Press ||
			    event.eventKind == NormalizedInputEventKind::RepeatPress) {
				switch (decodeUiModalAction(event.value)) {
					case UiModalAction::Pause:
						return makeAction(InputSemanticActionKind::Pause, 1);

					case UiModalAction::Resume:
						return makeAction(InputSemanticActionKind::Resume, 1);

					case UiModalAction::Abort:
						// ABBRECHEN bleibt absichtlich auf der bewaehrten Undo-/Abort-
						// Schiene, damit globale Sicherheitsausstiege keinen zweiten
						// Workflowvertrag neben Undo aufmachen.
						return makeAction(InputSemanticActionKind::Undo, 1);

					case UiModalAction::None:
						break;
				}
			}
			break;

		case NormalizedInputSource::WirelessEncoderButton:
			// Der dedizierte Wireless-Taster hat keine eigene Workflowrolle, sondern
			// spiegelt Confirm bzw. Undo fuer den entfernten Bedienpfad.
			if (event.eventKind == NormalizedInputEventKind::Press ||
			    event.eventKind == NormalizedInputEventKind::RepeatPress) {
				return makeAction(InputSemanticActionKind::Confirm, 1);
			}
			if (event.eventKind == NormalizedInputEventKind::LongPress) {
				return makeAction(InputSemanticActionKind::Undo, 1);
			}
			break;

		case NormalizedInputSource::WirelessMeasureButton:
			if (event.eventKind == NormalizedInputEventKind::Press ||
			    event.eventKind == NormalizedInputEventKind::LongPress) {
				return makeAction(InputSemanticActionKind::Measure, 1);
			}
			break;

		case NormalizedInputSource::WirelessBackButton:
			if (event.eventKind == NormalizedInputEventKind::Press ||
			    event.eventKind == NormalizedInputEventKind::LongPress) {
				return makeAction(InputSemanticActionKind::Undo, 1);
			}
			break;

		case NormalizedInputSource::None:
		case NormalizedInputSource::LocalStartButton:
			break;
	}

	return InputSemanticAction{};
}

}  // namespace dukatimer