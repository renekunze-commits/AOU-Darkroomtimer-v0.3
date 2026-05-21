#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "AppManager.h"
#include "SensorManager.h"
#include "types.h"

/* =============================================================================
 * WirelessManager.h - DUKATIMER BETA (v0.917.5)
 * * Handgerät-Management via ESP-NOW.
 * * Kopplung und Fernsteuerung.
 * ========================================================================== */

class WirelessManager
{
public:
    WirelessManager(AppManager *appMgr, SensorManager *sensorMgr);
    ~WirelessManager() = default;

    bool init();
    void process();

    // Ausgehende Befehle zum Handgerät
    // Sendet das UI-Update-Paket an das Handgerät
    void sendRender(const char* header, const char* l1, const char* l2, 
                    const uint8_t* histogram, uint8_t haptic, ProbeDisplayMode displayMode);
    void sendMeasureCmd(bool isG5);

    bool isConnected() const { return _connected; }

private:
    AppManager *_appMgr;
    SensorManager *_sensorMgr;

    uint8_t _peerMac[6];
    bool _peerBound;
    volatile bool _pendingPeerAdd;
    bool _initialized;
    bool _connected;
    uint32_t _lastPacketMs;

    // ESP-NOW Callbacks
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    static void onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len);
#else
    static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len);
#endif

    void handleIncomingPacket(const uint8_t *mac, const uint8_t *data, int len);

    static WirelessManager *_instance;
};
