#pragma once

#include <stdint.h>

namespace dukatimer {

// UiModalAction beschreibt die fachliche Absicht eines globalen Modal-Buttons,
// unabhaengig davon, auf welchem konkreten LVGL-Screen der Button sitzt.
enum class UiModalAction : uint8_t {
	None = 0,
	Resume = 1,
	Abort = 2,
	Pause = 3,
};

inline int16_t encodeUiModalAction(UiModalAction action) {
	return static_cast<int16_t>(static_cast<uint8_t>(action));
}

inline UiModalAction decodeUiModalAction(int16_t encodedAction) {
	if (encodedAction < 0 ||
	    encodedAction > static_cast<int16_t>(static_cast<uint8_t>(UiModalAction::Pause))) {
		return UiModalAction::None;
	}

	return static_cast<UiModalAction>(static_cast<uint8_t>(encodedAction));
}

}  // namespace dukatimer