/*
 * LightController.cpp
 *
 * Die aktuelle Implementierung ist die kleinste funktionale Lichtbasis fuer das
 * Part2-Board. Sie haelt die SaveLatch-Regel zentral an einer Stelle.
 */
#include "LightController.h"

namespace dukatimer {

LightController::LightController(uint8_t ssrRoomPin) : ssrRoomPin_(ssrRoomPin) {}

void LightController::begin(bool focusRaw, bool saveRaw, bool roomRaw) {
	pinMode(ssrRoomPin_, OUTPUT);
	digitalWrite(ssrRoomPin_, HIGH);

	setLocalSwitches(focusRaw, saveRaw, roomRaw);
	applyRoomOutput();
}

void LightController::setLocalSwitches(bool focusRaw, bool saveRaw, bool roomRaw) {
	state_.update(focusRaw, saveRaw, roomRaw);
}

void LightController::applyRoomOutput() {
	// Room ist aktiv-low am SSR-Treiber verdrahtet und wird deshalb invertiert
	// ausgegeben.
	digitalWrite(ssrRoomPin_, state_.derivedRoomOutput ? LOW : HIGH);
}

HeadLightSourceState LightController::localHeadSource() const {
	if (state_.derivedSaveOutput) {
		return makeActiveHeadLightSource(makeHeadLightSolid(makeLightRgb(255, 0, 0)));
	}

	if (state_.derivedFocusOutput) {
		return makeActiveHeadLightSource(makeHeadLightSolid(makeLightRgb(255, 255, 255)));
	}

	return makeInactiveHeadLightSource();
}

const LightState& LightController::state() const {
	return state_;
}

}  // namespace dukatimer