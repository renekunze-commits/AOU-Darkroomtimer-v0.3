#pragma once

#include <stdint.h>

namespace dukatimer {
namespace board {

// Spiegel der aktuellen Teensy-Pintabelle aus
// docs/hardware/duka-teen-schematic-overview.md. Wenn sich das PCB- oder
// Schaltplanrouting aendert, muss diese Datei bewusst mitgezogen werden, damit
// die produktive Verdrahtung nicht still in main.cpp auseinanderlaeuft.
constexpr uint8_t PIN_ENC1_A = 2;
constexpr uint8_t PIN_ENC1_B = 3;
constexpr uint8_t PIN_ENC1_SW = 4;
constexpr uint8_t PIN_ENC2_A = 5;
constexpr uint8_t PIN_TOUCH_CS = 6;
constexpr uint8_t PIN_S3_RTS = 7;
constexpr uint8_t PIN_S3_CTS = 8;
constexpr uint8_t PIN_TFT_DC = 9;
constexpr uint8_t PIN_TFT_CS = 10;
constexpr uint8_t PIN_SSR_ROOM = 24;
constexpr uint8_t PIN_ENC3_A = 25;
constexpr uint8_t PIN_ENC3_B = 26;
constexpr uint8_t PIN_ENC3_SW = 27;
constexpr uint8_t PIN_TOUCH_IRQ = 28;
constexpr uint8_t PIN_NEO_1 = 29;
constexpr uint8_t PIN_NEO_2 = 30;
constexpr uint8_t PIN_NEO_3 = 31;
constexpr uint8_t PIN_NEO_4 = 32;
constexpr uint8_t PIN_BTN_START = 33;
constexpr uint8_t PIN_BTN_RED = 36;
constexpr uint8_t PIN_BTN_WHITE = 37;
constexpr uint8_t PIN_BTN_ROOM = 38;
constexpr uint8_t PIN_TSL_INT = 39;
constexpr uint8_t PIN_S3_GPIO0 = 40;
constexpr uint8_t PIN_TFT_RST = 41;
constexpr uint8_t PIN_ENC2_SW = 20;
constexpr uint8_t PIN_ENC2_B = 21;
constexpr uint8_t PIN_TFT_BL = 22;

}  // namespace board
}  // namespace dukatimer