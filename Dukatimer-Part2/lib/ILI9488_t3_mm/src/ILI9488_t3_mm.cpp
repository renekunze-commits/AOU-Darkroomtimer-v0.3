#include "ILI9488_t3_mm.h"

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint16_t kAsyncWriteBufferLines = 40;
constexpr size_t kAsyncWriteMaxPixels = static_cast<size_t>(ILI9488_TFTHEIGHT) * kAsyncWriteBufferLines;
constexpr size_t kAsyncWriteBytesPerPixel = 3;

DMAMEM static uint8_t sAsyncWriteBuffer[kAsyncWriteMaxPixels * kAsyncWriteBytesPerPixel];

}  // namespace

bool ILI9488_t3_mm::writeRectAsync(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels,
	                               AsyncTransferCompleteCallback callback, void* context) {
#if !defined(SPI_HAS_TRANSFER_ASYNC)
	(void)x;
	(void)y;
	(void)w;
	(void)h;
	(void)pixels;
	(void)callback;
	(void)context;
	return false;
#else
	if (asyncWriteActive_ || pixels == nullptr || w <= 0 || h <= 0 || spi_port == nullptr) {
		return false;
	}

	const size_t pixelCount = static_cast<size_t>(w) * static_cast<size_t>(h);
	if (pixelCount > kAsyncWriteMaxPixels) {
		return false;
	}

	if (asyncWriteResponder_.getContext() != this) {
		asyncWriteResponder_.attach(&ILI9488_t3_mm::handleAsyncWriteRectDone);
		asyncWriteResponder_.setContext(this);
	}

	convertRgb565ToRgb888(pixels, pixelCount);

	asyncWriteCompleteCallback_ = callback;
	asyncWriteCompleteContext_ = context;
	asyncWriteActive_ = true;

	beginSPITransaction();
	setAddr(x, y, x + w - 1, y + h - 1);
	writecommand_cont(ILI9488_RAMWR);
	waitTransmitComplete();

	// Nach dem RAMWR-Befehl bleiben CS und die Buskonfiguration erhalten; der
	// folgende DMA-Transfer uebernimmt nur die Pixeldaten im 8-Bit-Strom.
	maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
	if (!spi_port->transfer(sAsyncWriteBuffer, nullptr, pixelCount * kAsyncWriteBytesPerPixel, asyncWriteResponder_)) {
		asyncWriteActive_ = false;
		asyncWriteCompleteCallback_ = nullptr;
		asyncWriteCompleteContext_ = nullptr;
		endSPITransaction();
		return false;
	}

	return true;
#endif
}

bool ILI9488_t3_mm::asyncWriteRectActive() const {
	return asyncWriteActive_;
}

void ILI9488_t3_mm::waitAsyncWriteRectComplete() {
	while (asyncWriteActive_) {
		yield();
	}
}

void ILI9488_t3_mm::handleAsyncWriteRectDone(EventResponderRef eventResponder) {
	auto* self = static_cast<ILI9488_t3_mm*>(eventResponder.getContext());
	if (self != nullptr) {
		self->handleAsyncWriteRectDoneImpl();
	}
}

void ILI9488_t3_mm::handleAsyncWriteRectDoneImpl() {
	endSPITransaction();
	asyncWriteActive_ = false;

	AsyncTransferCompleteCallback callback = asyncWriteCompleteCallback_;
	void* callbackContext = asyncWriteCompleteContext_;
	asyncWriteCompleteCallback_ = nullptr;
	asyncWriteCompleteContext_ = nullptr;

	if (callback != nullptr) {
		callback(callbackContext);
	}
}

void ILI9488_t3_mm::convertRgb565ToRgb888(const uint16_t* pixels, size_t pixelCount) {
	uint8_t* output = sAsyncWriteBuffer;
	for (size_t index = 0; index < pixelCount; ++index) {
		const uint16_t color = pixels[index];

		uint8_t red = static_cast<uint8_t>((color & 0xF800u) >> 11);
		uint8_t green = static_cast<uint8_t>((color & 0x07E0u) >> 5);
		uint8_t blue = static_cast<uint8_t>(color & 0x001Fu);

		red = static_cast<uint8_t>((static_cast<uint16_t>(red) * 255u) / 31u);
		green = static_cast<uint8_t>((static_cast<uint16_t>(green) * 255u) / 63u);
		blue = static_cast<uint8_t>((static_cast<uint16_t>(blue) * 255u) / 31u);

		*output++ = red;
		*output++ = green;
		*output++ = blue;
	}
}