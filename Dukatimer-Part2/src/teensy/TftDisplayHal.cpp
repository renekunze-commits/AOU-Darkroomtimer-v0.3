/*
 * TftDisplayHal
 *
 * Diese Datei bildet die duenne Hardware-Abstraktion fuer das TFT. Sie besitzt
 * keine UI-Logik, sondern nur die Regeln fuer Initialisierung, Flush-Synchroni-
 * sation und die Wahl zwischen Framebuffer- und Rechteck-Transferpfad.
 */

#include "TftDisplayHal.h"

namespace dukatimer {

TftDisplayHal::TftDisplayHal(const TftDisplayConfig& config)
	: tft_(config.chipSelectPin, config.dcPin, config.resetPin), config_(config) {}

bool TftDisplayHal::begin() {
	// begin() initialisiert Backlight und Display in einer festen Reihenfolge.
	// Ein optionaler Framebuffer wird nur aktiviert, wenn der Treiber ihn tatsaechlich
	// bereitstellen kann.
	pinMode(config_.backlightPin, OUTPUT);
	digitalWrite(config_.backlightPin, HIGH);

	tft_.begin(config_.spiClockHz);
	tft_.setRotation(config_.rotation);
	frameBufferEnabled_ = false;

	if (config_.enableFrameBuffer) {
		frameBufferEnabled_ = tft_.useFrameBuffer(true) != 0;
	}

	tft_.fillScreen(ILI9488_BLACK);
	if (frameBufferEnabled_) {
		tft_.updateScreen();
	}

	return true;
}

void TftDisplayHal::waitForPendingFlush() {
	// Vor synchronen Schreibpfaden werden alle laufenden async-Transfers sauber
	// abgeschlossen, damit LVGL und der Treiber keine ueberlappenden Flushes sehen.
	if (frameBufferEnabled_ && tft_.asyncUpdateActive()) {
		tft_.waitUpdateAsyncComplete();
	}
	tft_.waitAsyncWriteRectComplete();
}

bool TftDisplayHal::present() {
	// present() existiert nur fuer den Vollbild-Framebuffer-Pfad. Ohne aktiven
	// Framebuffer ist LVGL auf writeRect()-Transfers angewiesen.
	if (!frameBufferEnabled_) {
		return false;
	}

	if (tft_.asyncUpdateActive()) {
		return false;
	}

	return tft_.updateScreenAsync(false);
}

void TftDisplayHal::presentBlocking() {
	if (!frameBufferEnabled_) {
		return;
	}

	waitForPendingFlush();
	tft_.updateScreen();
}

void TftDisplayHal::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels) {
	waitForPendingFlush();
	tft_.writeRect(x, y, w, h, pixels);
}

bool TftDisplayHal::writeRectAsync(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels,
	                               TransferCompleteCallback callback, void* context) {
	// Asynchrone Teilrechtecke werden nur gestartet, wenn kein konkurrierender
	// async-Fullscreen-Flush laeuft. Die HAL bleibt damit der einzige Ort, der
	// diese Treibernebenbedingung kennen muss.
	if (frameBufferEnabled_ && tft_.asyncUpdateActive()) {
		return false;
	}

	return tft_.writeRectAsync(x, y, w, h, pixels, callback, context);
}

bool TftDisplayHal::frameBufferEnabled() const {
	return frameBufferEnabled_;
}

bool TftDisplayHal::asyncFlushSupported() const {
#if defined(SPI_HAS_TRANSFER_ASYNC)
	return true;
#else
	return frameBufferEnabled_;
#endif
}

bool TftDisplayHal::asyncFlushActive() const {
	return (frameBufferEnabled_ && const_cast<ILI9488_t3_mm&>(tft_).asyncUpdateActive()) || tft_.asyncWriteRectActive();
}

RAFB* TftDisplayHal::frameBuffer() {
	return tft_.getFrameBuffer();
}

ILI9488_t3_mm& TftDisplayHal::driver() {
	return tft_;
}

const ILI9488_t3_mm& TftDisplayHal::driver() const {
	return tft_;
}

}  // namespace dukatimer