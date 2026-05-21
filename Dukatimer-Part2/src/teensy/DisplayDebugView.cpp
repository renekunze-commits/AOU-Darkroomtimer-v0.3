/*
 * DisplayDebugView.cpp
 *
 * Diese Datei rendert nur eine einfache Diagnoseoberflaeche fuer Input-, Touch-
 * und Lichtzustand. Sie ist kein Ersatz fuer die spaetere UI-Schichtung.
 */
#include "DisplayDebugView.h"

#include "FirmwareVersion.h"

namespace dukatimer {

DisplayDebugView::DisplayDebugView(uint8_t chipSelectPin, uint8_t dcPin, uint8_t resetPin, uint8_t backlightPin, uint8_t rotation)
	: display_({chipSelectPin, dcPin, resetPin, backlightPin, rotation, ILI9488_SPICLOCK, true}) {}

void DisplayDebugView::begin() {
	display_.begin();
}

void DisplayDebugView::render(const DisplayDebugSnapshot& snapshot) {
	display_.waitForPendingFlush();
	auto& tft = display_.driver();

	// Der Hintergrund wird nicht pro Frame geloescht, um sichtbares Flackern auf
	// dem SPI-TFT zu vermeiden. Einzelne Texte schreiben ihren Hintergrund selbst.
	tft.setTextColor(0xFFFF, 0x0000);
	tft.setTextSize(2);
	tft.setCursor(12, 12);
	tft.printf("%-20s", "Dukatimer Part2");

	tft.setTextSize(1);
	tft.setCursor(12, 40);
	tft.printf("v%s | %-24s", dukatimer::build::kFirmwareVersion, dukatimer::build::kFirmwareStage);
	tft.setCursor(12, 52);
	tft.printf("Display: %-24s", dukatimer::build::kDisplayBackend);

	tft.setCursor(12, 72);
	tft.printf("ENC1: %-8ld", snapshot.encoder1Position);
	tft.setCursor(12, 84);
	tft.printf("ENC2: %-8ld", snapshot.encoder2Position);
	tft.setCursor(12, 96);
	tft.printf("ENC3: %-8ld", snapshot.encoder3Position);

	tft.setCursor(12, 120);
	tft.printf("Focus: %-3s", snapshot.lightState.focusSwitchRaw ? "ON" : "OFF");
	tft.setCursor(12, 132);
	tft.printf("Save : %-3s", snapshot.lightState.saveSwitchRaw ? "ON" : "OFF");
	tft.setCursor(12, 144);
	tft.printf("Room : %-3s", snapshot.lightState.roomSwitchRaw ? "ON" : "OFF");
	tft.setCursor(12, 156);
	tft.printf("Latch: %-3s", snapshot.lightState.saveLatchActive ? "ON" : "OFF");
	tft.setCursor(12, 168);
	tft.printf("Start: %-3s", snapshot.startButtonActive ? "ON" : "OFF");

	tft.setCursor(12, 192);
	if (snapshot.touchState.active) {
		tft.printf("Touch raw: %-5d / %-5d / %-5d", snapshot.touchState.rawX, snapshot.touchState.rawY, snapshot.touchState.rawZ);
	} else {
		tft.printf("%-40s", "");
	}

	display_.present();
}

}  // namespace dukatimer