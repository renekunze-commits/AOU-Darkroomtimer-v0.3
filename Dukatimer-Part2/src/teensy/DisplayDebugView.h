/*
 * DisplayDebugView
 *
 * Kleine, bewusst provisorische Render-Schicht fuer die aktuelle Bring-up-Phase.
 * Sie stellt nur einen Snapshot dar und besitzt keinerlei Fachlogik.
 */
#pragma once

#include <Arduino.h>
#include "LightController.h"
#include "TftDisplayHal.h"
#include "TouchSampler.h"

namespace dukatimer {

struct DisplayDebugSnapshot {
	// Der Snapshot wird ausserhalb der View aufgebaut, damit die View keine
	// Abhaengigkeit zu globalen Singletons oder Hardwareobjekten aufbauen muss.
	long encoder1Position = 0;
	long encoder2Position = 0;
	long encoder3Position = 0;
	bool startButtonActive = false;
	LightState lightState;
	TouchState touchState;
};

class DisplayDebugView {
public:
	DisplayDebugView(uint8_t chipSelectPin, uint8_t dcPin, uint8_t resetPin, uint8_t backlightPin, uint8_t rotation);

	// Initialisiert Backlight und Display-Controller und setzt den Screen in einen
	// bekannten schwarzen Grundzustand.
	void begin();
	// Rendern ist bewusst rein datengetrieben: dieselbe Funktion kann spaeter
	// leicht durch einen anderen Presenter oder LVGL-Screen ersetzt werden.
	void render(const DisplayDebugSnapshot& snapshot);

private:
	TftDisplayHal display_;
};

}  // namespace dukatimer