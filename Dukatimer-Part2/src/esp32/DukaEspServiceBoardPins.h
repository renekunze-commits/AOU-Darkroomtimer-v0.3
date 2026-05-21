#pragma once

#include <stdint.h>

namespace dukatimer {
namespace esp_board {

// Spiegel der aktuell dokumentierten ESP32-S3-Servicebelegung aus
// docs/hardware/duka-teen-schematic-overview.md. Wenn sich die KiCad-Revision
// aendert, muessen UART-, Sensor- und spaetere Encoder4-Pins hier gemeinsam
// nachgezogen werden statt verteilt in mehreren Diensten.
constexpr int PIN_UART_RX = 44;
constexpr int PIN_UART_TX = 43;
constexpr int PIN_UART_CTS = 5;
constexpr int PIN_UART_RTS = 4;

constexpr uint8_t PIN_ONEWIRE = 8;
// Der Umweltsensorbus auf U-SENS1 ist ein gemeinsamer I2C-Pfad fuer AHT20 und
// BMP280. Beide Sensoren teilen sich diese Leitungen und werden gemeinsam als
// Servicepfad zum Teensy publiziert.
constexpr uint8_t PIN_AHT_SDA = 11;
constexpr uint8_t PIN_AHT_SCL = 12;

// Encoder 4 liegt lokal auf dem ESP32-S3 und wird als stoerungsarmer
// Kontextencoder per PCNT erfasst. Die semantische Auswertung bleibt trotzdem
// auf Teensy-Seite ueber den bestehenden Remote-Input-Pfad gebuendelt.
constexpr uint8_t PIN_ENC4_A = 3;
constexpr uint8_t PIN_ENC4_B = 2;
constexpr uint8_t PIN_ENC4_SW = 1;

}  // namespace esp_board
}  // namespace dukatimer