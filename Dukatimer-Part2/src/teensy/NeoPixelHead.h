#pragma once

#include <Arduino.h>
#include <NeoPixelBus.h>

#include "HeadTimingConstants.h"
#include "HeadLightCommand.h"

namespace dukatimer {

enum class NeoPixelRotation : uint8_t {
	Deg0,
	Deg90,
	Deg180,
	Deg270,
};

struct NeoPixelSegmentConfig {
	uint8_t pin = 0xFF;
	uint8_t originX = 0;
	uint8_t originY = 0;
	uint8_t width = 0;
	uint8_t height = 0;
	bool zigZagRows = true;
	NeoPixelRotation rotation = NeoPixelRotation::Deg0;
};

struct NeoPixelHeadConfig {
	uint8_t width = 16;
	uint8_t height = 16;
	uint8_t segmentCount = 0;
	NeoPixelSegmentConfig segments[4] = {};
};

struct NeoPixelHeadPresentStats {
	uint32_t presentCalls = 0;
	uint32_t appliedFrames = 0;
	uint32_t skippedFrames = 0;
	uint32_t lastDurationUs = 0;
	uint32_t averageDurationUs = 0;
	uint32_t maxDurationUs = 0;
};

/*
 * NeoPixelHead
 *
 * Zweck:
 * - bildet die logische Kopflichtflaeche als 2D-Framebuffer ueber bis zu vier
 *   physische NeoPixel-Segmente ab
 * - nimmt abstrakte HeadLightCommands entgegen und setzt sie in weiche Start-,
 *   Stop- und Crossfade-Uebergaenge um
 * - misst present()-Laufzeiten fuer spaetere praediktive Shutoff-Logik mit
 *
 * Architekturgrenze:
 * - diese Klasse kennt keine Belichtungsphasen und keine lokalen Schalter
 * - sie rendert nur den bereits entschiedenen Kopflichtbefehl auf Hardware
 */
class NeoPixelHead {
public:
	explicit NeoPixelHead(const NeoPixelHeadConfig& config);
	~NeoPixelHead();

	void begin();
	void clear();
	void fill(const LightRgb& color);
	void setPixel(uint8_t x, uint8_t y, const LightRgb& color);
	void applyCommand(const HeadLightCommand& command);
	void present();
	const NeoPixelHeadPresentStats& presentStats() const;
	void resetPresentStats();

	uint8_t width() const;
	uint8_t height() const;

private:
	using PixelBus = NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod>;

	static constexpr uint8_t kMaxSegments = 4;
	static constexpr uint16_t kMaxLogicalPixels = 256;
	static constexpr uint16_t kSoftStartMs = 64;
	static constexpr uint16_t kSoftStopMs = kHeadSoftStopMs;
	static constexpr uint16_t kCrossfadeMs = 40;
	static constexpr uint8_t kPresentDurationEmaShift = 3;

	static RgbColor toNeoPixelColor(const LightRgb& color);
	static LightRgb commandColor(const HeadLightCommand& command);
	static bool isDarkColor(const LightRgb& color);
	static uint8_t blendChannel(uint8_t from, uint8_t to, uint16_t progressPermille);
	static LightRgb blendColor(const LightRgb& fromColor, const LightRgb& toColor, uint16_t progressPermille);
	static uint16_t mapSegmentPixel(const NeoPixelSegmentConfig& segment, uint8_t localX, uint8_t localY);
	static uint16_t transitionDurationFor(const LightRgb& fromColor, const LightRgb& toColor);

	uint16_t logicalPixelCount() const;
	bool logicalIndex(uint8_t x, uint8_t y, uint16_t* index) const;
	void applySolidFrameColor(const LightRgb& color);
	void startTransition(const HeadLightCommand& command, uint32_t nowMs);
	void updateTransition(uint32_t nowMs);

	NeoPixelHeadConfig config_;
	PixelBus* segments_[kMaxSegments] = {};
	LightRgb frameBuffer_[kMaxLogicalPixels] = {};
	HeadLightCommand activeCommand_ = {};
	HeadLightCommand targetCommand_ = {};
	LightRgb renderedColor_ = {};
	LightRgb transitionFromColor_ = {};
	LightRgb transitionToColor_ = {};
	uint32_t transitionStartedMs_ = 0;
	uint16_t transitionDurationMs_ = 0;
	bool transitionActive_ = false;
	bool dirty_ = true;
	uint32_t averageDurationUsEmaQ8_ = 0;
	NeoPixelHeadPresentStats presentStats_ = {};
};

}  // namespace dukatimer