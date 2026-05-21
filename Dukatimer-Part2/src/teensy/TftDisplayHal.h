#pragma once

#include <Arduino.h>
#include <ILI9488_t3_mm.h>

namespace dukatimer {

struct TftDisplayConfig {
	uint8_t chipSelectPin;
	uint8_t dcPin;
	uint8_t resetPin;
	uint8_t backlightPin;
	uint8_t rotation;
	uint32_t spiClockHz;
	bool enableFrameBuffer;
};

/*
 * TftDisplayHal
 *
 * Zweck:
 * - kapselt den konkreten ILI9488-Treiber hinter einer kleinen, LVGL-tauglichen
 *   Anzeige-HAL
 * - versteckt Details zu Framebuffer-Nutzung, async writeRect und Backlight-
 *   Initialisierung vor der restlichen UI-Schicht
 * - laesst LvglUi dadurch gegen eine stabilere, engere Display-Schnittstelle
 *   arbeiten statt direkt gegen den Bibliothekstreiber
 */
class TftDisplayHal {
public:
	using TransferCompleteCallback = void (*)(void* context);

	explicit TftDisplayHal(const TftDisplayConfig& config);

	bool begin();
	void waitForPendingFlush();
	bool present();
	void presentBlocking();
	void writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels);
	bool writeRectAsync(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels,
	                   TransferCompleteCallback callback, void* context);

	bool frameBufferEnabled() const;
	bool asyncFlushSupported() const;
	bool asyncFlushActive() const;
	RAFB* frameBuffer();
	ILI9488_t3_mm& driver();
	const ILI9488_t3_mm& driver() const;

private:
	ILI9488_t3_mm tft_;
	TftDisplayConfig config_;
	bool frameBufferEnabled_ = false;
};

}  // namespace dukatimer