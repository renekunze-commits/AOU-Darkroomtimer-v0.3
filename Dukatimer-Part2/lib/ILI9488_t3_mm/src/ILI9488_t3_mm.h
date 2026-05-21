#pragma once

#include <EventResponder.h>
#include <ILI9488_t3.h>

/*
 * ILI9488_t3_mm compatibility shim
 *
 * Dieser lokale Header haelt den Projekttypnamen ILI9488_t3_mm stabil, solange
 * die exakte externe _mm-Quelle noch nicht als echte Abhaengigkeit vorliegt.
 * Sobald die Zielbibliothek verifiziert verfuegbar ist, soll dieser Shim durch
 * die echte Library ersetzt werden.
 */
class ILI9488_t3_mm : public ILI9488_t3 {
public:
	using AsyncTransferCompleteCallback = void (*)(void* context);

	using ILI9488_t3::ILI9488_t3;

	bool writeRectAsync(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels,
	                    AsyncTransferCompleteCallback callback, void* context);
	bool asyncWriteRectActive() const;
	void waitAsyncWriteRectComplete();

private:
	static void handleAsyncWriteRectDone(EventResponderRef eventResponder);
	void handleAsyncWriteRectDoneImpl();
	void convertRgb565ToRgb888(const uint16_t* pixels, size_t pixelCount);

	EventResponder asyncWriteResponder_;
	AsyncTransferCompleteCallback asyncWriteCompleteCallback_ = nullptr;
	void* asyncWriteCompleteContext_ = nullptr;
	volatile bool asyncWriteActive_ = false;
};