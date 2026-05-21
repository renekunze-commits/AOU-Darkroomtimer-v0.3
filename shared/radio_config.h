#pragma once

#include <stdint.h>

// Gemeinsame 2.4-GHz-Konfiguration fuer WLAN/NTP und ESP-NOW.
// Der Sender nutzt die SSID nur zur Kanal-Erkennung und verbindet sich nicht mit dem AP.
static constexpr const char* BEWEME_WIFI_SSID = "FRITZ!Box Gastzugang";
static constexpr uint8_t BEWEME_ESPNOW_FALLBACK_CHANNEL = 1;

inline bool isValidBeWeMeWifiChannel(uint8_t channel) {
    return channel >= 1 && channel <= 13;
}