/*
 * NeoPixelHead
 *
 * Diese Datei kapselt die komplette Umsetzung vom abstrakten Kopflichtkommando
 * auf die tatsaechliche NeoPixel-Matrix. Das umfasst Segment-Mapping,
 * Uebergangsanimationen, Framebuffer-Aktualisierung und Laufzeitmessung des
 * physikalischen Show()-Pfads.
 */

#include "NeoPixelHead.h"

namespace {

constexpr uint8_t kInvalidPin = 0xFF;

// Jedes Segment beschreibt einen rechteckigen physikalischen Teilbereich der
// Gesamtmatrix. Die Pixelzahl wird daraus zentral abgeleitet.
uint16_t segmentPixelCount(const dukatimer::NeoPixelSegmentConfig& segment) {
	return static_cast<uint16_t>(segment.width) * static_cast<uint16_t>(segment.height);
}

}  // namespace

namespace dukatimer {

NeoPixelHead::NeoPixelHead(const NeoPixelHeadConfig& config) : config_(config) {}

NeoPixelHead::~NeoPixelHead() {
	for (uint8_t index = 0; index < kMaxSegments; ++index) {
		delete segments_[index];
		segments_[index] = nullptr;
	}
}

void NeoPixelHead::begin() {
	// begin() erzeugt und initialisiert nur die wirklich konfigurierten Segmente.
	// Der logische Framebuffer bleibt davon getrennt und wird anschliessend mit
	// clear()/present() in einen definierten OFF-Zustand gebracht.
	for (uint8_t index = 0; index < config_.segmentCount && index < kMaxSegments; ++index) {
		const NeoPixelSegmentConfig& segment = config_.segments[index];
		if (segment.pin == kInvalidPin || segmentPixelCount(segment) == 0) {
			continue;
		}

		if (segments_[index] == nullptr) {
			segments_[index] = new PixelBus(segmentPixelCount(segment), segment.pin);
		}

		segments_[index]->Begin();
		segments_[index]->ClearTo(RgbColor(0, 0, 0));
		segments_[index]->Show();
	}

	clear();
	present();
	resetPresentStats();
}

void NeoPixelHead::clear() {
	const LightRgb offColor = {};
	applySolidFrameColor(offColor);
	renderedColor_ = offColor;
	transitionFromColor_ = offColor;
	transitionToColor_ = offColor;
	transitionDurationMs_ = 0;
	transitionStartedMs_ = 0;
	transitionActive_ = false;
	activeCommand_ = makeHeadLightOff();
	targetCommand_ = activeCommand_;
}

void NeoPixelHead::fill(const LightRgb& color) {
	applySolidFrameColor(color);
	renderedColor_ = color;
	transitionFromColor_ = color;
	transitionToColor_ = color;
	transitionDurationMs_ = 0;
	transitionStartedMs_ = 0;
	transitionActive_ = false;
	activeCommand_ = makeHeadLightSolid(color);
	targetCommand_ = activeCommand_;
}

void NeoPixelHead::setPixel(uint8_t x, uint8_t y, const LightRgb& color) {
	uint16_t index = 0;
	if (!logicalIndex(x, y, &index) || frameBuffer_[index] == color) {
		return;
	}

	frameBuffer_[index] = color;
	dirty_ = true;
	transitionActive_ = false;
	activeCommand_ = makeHeadLightOff();
	targetCommand_ = activeCommand_;
}

void NeoPixelHead::applyCommand(const HeadLightCommand& command) {
	// applyCommand() nimmt neue Zielkommandos jederzeit an, fuehrt sie aber nie
	// direkt als harten Sprung aus. Stattdessen wird der laufende Renderzustand
	// ueber startTransition()/updateTransition() in den neuen Zielzustand ueberfuehrt.
	const uint32_t nowMs = millis();
	updateTransition(nowMs);

	if (command == targetCommand_ && !transitionActive_ && command == activeCommand_) {
		return;
	}

	if (command != targetCommand_) {
		startTransition(command, nowMs);
	}

	updateTransition(nowMs);
}

void NeoPixelHead::present() {
	// present() ist der einzige physische Ausgabezeitpunkt. Nur hier werden die
	// logischen Framebuffer-Farben auf die Segment-Busse geschrieben und die
	// resultierende Uebertragungsdauer fuer die Telemetrie gemessen.
	updateTransition(millis());
	++presentStats_.presentCalls;

	if (!dirty_) {
		++presentStats_.skippedFrames;
		return;
	}

	const uint32_t startedUs = micros();

	for (uint8_t segmentIndex = 0; segmentIndex < config_.segmentCount && segmentIndex < kMaxSegments; ++segmentIndex) {
		PixelBus* bus = segments_[segmentIndex];
		const NeoPixelSegmentConfig& segment = config_.segments[segmentIndex];
		if (bus == nullptr || segmentPixelCount(segment) == 0) {
			continue;
		}

		for (uint8_t localY = 0; localY < segment.height; ++localY) {
			for (uint8_t localX = 0; localX < segment.width; ++localX) {
				const uint8_t globalX = static_cast<uint8_t>(segment.originX + localX);
				const uint8_t globalY = static_cast<uint8_t>(segment.originY + localY);
				uint16_t frameIndex = 0;
				if (!logicalIndex(globalX, globalY, &frameIndex)) {
					continue;
				}

				const uint16_t pixelIndex = mapSegmentPixel(segment, localX, localY);
				if (pixelIndex >= segmentPixelCount(segment)) {
					continue;
				}

				bus->SetPixelColor(pixelIndex, toNeoPixelColor(frameBuffer_[frameIndex]));
			}
		}

		bus->Show();
	}

	const uint32_t elapsedUs = micros() - startedUs;
	++presentStats_.appliedFrames;
	presentStats_.lastDurationUs = elapsedUs;
	if (elapsedUs > presentStats_.maxDurationUs) {
		presentStats_.maxDurationUs = elapsedUs;
	}

	if (presentStats_.appliedFrames == 1) {
		averageDurationUsEmaQ8_ = elapsedUs << 8u;
		presentStats_.averageDurationUs = elapsedUs;
	} else {
		const uint32_t sampleQ8 = elapsedUs << 8u;
		const int32_t deltaQ8 = static_cast<int32_t>(sampleQ8) - static_cast<int32_t>(averageDurationUsEmaQ8_);
		averageDurationUsEmaQ8_ += static_cast<uint32_t>(deltaQ8 >> kPresentDurationEmaShift);
		presentStats_.averageDurationUs = (averageDurationUsEmaQ8_ + 128u) >> 8u;
	}

	dirty_ = false;
}

const NeoPixelHeadPresentStats& NeoPixelHead::presentStats() const {
	return presentStats_;
}

void NeoPixelHead::resetPresentStats() {
	presentStats_ = {};
	averageDurationUsEmaQ8_ = 0u;
}

uint8_t NeoPixelHead::width() const {
	return config_.width;
}

uint8_t NeoPixelHead::height() const {
	return config_.height;
}

RgbColor NeoPixelHead::toNeoPixelColor(const LightRgb& color) {
	return RgbColor(color.red, color.green, color.blue);
}

LightRgb NeoPixelHead::commandColor(const HeadLightCommand& command) {
	switch (command.mode) {
		case HeadLightMode::Off:
			return LightRgb{};
		case HeadLightMode::SolidColor:
			return command.color;
	}

	return LightRgb{};
}

bool NeoPixelHead::isDarkColor(const LightRgb& color) {
	return color.red == 0 && color.green == 0 && color.blue == 0;
}

uint8_t NeoPixelHead::blendChannel(uint8_t from, uint8_t to, uint16_t progressPermille) {
	if (progressPermille >= 1000) {
		return to;
	}

	const uint16_t inverse = static_cast<uint16_t>(1000 - progressPermille);
	const uint32_t mixed = (static_cast<uint32_t>(from) * inverse) +
	                     (static_cast<uint32_t>(to) * progressPermille);
	return static_cast<uint8_t>((mixed + 500u) / 1000u);
}

LightRgb NeoPixelHead::blendColor(const LightRgb& fromColor, const LightRgb& toColor,
	                               uint16_t progressPermille) {
	return makeLightRgb(blendChannel(fromColor.red, toColor.red, progressPermille),
	                   blendChannel(fromColor.green, toColor.green, progressPermille),
	                   blendChannel(fromColor.blue, toColor.blue, progressPermille));
}

uint16_t NeoPixelHead::mapSegmentPixel(const NeoPixelSegmentConfig& segment, uint8_t localX, uint8_t localY) {
	// mapSegmentPixel() entkoppelt die logische Matrixlage vom realen Verdrahtungs-
	// pfad des einzelnen Segments. Rotation und ZigZag werden ausschliesslich hier
	// behandelt, damit der restliche Code nur in logischen X/Y-Koordinaten denkt.
	uint8_t mappedX = localX;
	uint8_t mappedY = localY;
	uint8_t physicalWidth = segment.width;

	switch (segment.rotation) {
	case NeoPixelRotation::Deg0:
		mappedX = localX;
		mappedY = localY;
		physicalWidth = segment.width;
		break;
	case NeoPixelRotation::Deg90:
		mappedX = localY;
		mappedY = static_cast<uint8_t>(segment.width - 1u - localX);
		physicalWidth = segment.height;
		break;
	case NeoPixelRotation::Deg180:
		mappedX = static_cast<uint8_t>(segment.width - 1u - localX);
		mappedY = static_cast<uint8_t>(segment.height - 1u - localY);
		physicalWidth = segment.width;
		break;
	case NeoPixelRotation::Deg270:
		mappedX = static_cast<uint8_t>(segment.height - 1u - localY);
		mappedY = localX;
		physicalWidth = segment.height;
		break;
	}

	if (segment.zigZagRows && (mappedY % 2u) != 0u) {
		mappedX = static_cast<uint8_t>(physicalWidth - 1u - mappedX);
	}

	return static_cast<uint16_t>(mappedY) * physicalWidth + mappedX;
}

uint16_t NeoPixelHead::transitionDurationFor(const LightRgb& fromColor, const LightRgb& toColor) {
	// Einschalten, Ausschalten und Farbe-zu-Farbe-Wechsel nutzen bewusst
	// unterschiedliche Zeitkonstanten. Das vermeidet harte Lichtspruenge beim
	// Belichtungsstart und gibt dem Soft-Stop mehr Reserve fuer das Ausblenden.
	if (fromColor == toColor) {
		return 0;
	}

	if (isDarkColor(fromColor) && !isDarkColor(toColor)) {
		return kSoftStartMs;
	}

	if (!isDarkColor(fromColor) && isDarkColor(toColor)) {
		return kSoftStopMs;
	}

	return kCrossfadeMs;
}

uint16_t NeoPixelHead::logicalPixelCount() const {
	const uint16_t configuredPixelCount = static_cast<uint16_t>(config_.width) * static_cast<uint16_t>(config_.height);
	return configuredPixelCount < kMaxLogicalPixels ? configuredPixelCount : kMaxLogicalPixels;
}

bool NeoPixelHead::logicalIndex(uint8_t x, uint8_t y, uint16_t* index) const {
	if (x >= config_.width || y >= config_.height || index == nullptr) {
		return false;
	}

	*index = static_cast<uint16_t>(y) * static_cast<uint16_t>(config_.width) + x;
	return *index < logicalPixelCount();
}

void NeoPixelHead::applySolidFrameColor(const LightRgb& color) {
	bool changed = false;
	const uint16_t pixelCount = logicalPixelCount();
	for (uint16_t index = 0; index < pixelCount; ++index) {
		if (frameBuffer_[index] == color) {
			continue;
		}

		frameBuffer_[index] = color;
		changed = true;
	}

	if (changed) {
		dirty_ = true;
	}
}

void NeoPixelHead::startTransition(const HeadLightCommand& command, uint32_t nowMs) {
	// Ab hier gilt das neue Kommando als Zielzustand. Die sichtbare Farbe laeuft
	// aber weiter ueber renderedColor_ und den zeitbasierten Uebergang, bis das
	// Ziel erreicht ist.
	activeCommand_ = command;
	targetCommand_ = command;
	transitionFromColor_ = renderedColor_;
	transitionToColor_ = commandColor(command);
	transitionDurationMs_ = transitionDurationFor(transitionFromColor_, transitionToColor_);
	transitionStartedMs_ = nowMs;

	if (transitionDurationMs_ == 0 || transitionFromColor_ == transitionToColor_) {
		transitionActive_ = false;
		renderedColor_ = transitionToColor_;
		applySolidFrameColor(renderedColor_);
		return;
	}

	transitionActive_ = true;
}

void NeoPixelHead::updateTransition(uint32_t nowMs) {
	// updateTransition() arbeitet rein auf dem logischen Farbzustand. Hardware-
	// Ausgabe passiert erst im naechsten present()-Durchlauf.
	if (!transitionActive_) {
		return;
	}

	if (transitionDurationMs_ == 0) {
		transitionActive_ = false;
		renderedColor_ = transitionToColor_;
		applySolidFrameColor(renderedColor_);
		return;
	}

	const uint32_t elapsedMs = nowMs - transitionStartedMs_;
	if (elapsedMs >= transitionDurationMs_) {
		transitionActive_ = false;
		renderedColor_ = transitionToColor_;
		applySolidFrameColor(renderedColor_);
		return;
	}

	const uint16_t progressPermille = static_cast<uint16_t>((elapsedMs * 1000u) / transitionDurationMs_);
	const LightRgb blended = blendColor(transitionFromColor_, transitionToColor_, progressPermille);
	if (blended == renderedColor_) {
		return;
	}

	renderedColor_ = blended;
	applySolidFrameColor(renderedColor_);
}

}  // namespace dukatimer