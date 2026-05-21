#include "WirelessManager.h"
#include <WiFi.h>
#include <cstring>
#include "SystemContext.h"

/* =============================================================================
 * WirelessManager.cpp - DUKATIMER BETA (v0.917.5)
 * * Implementierung der ESP-NOW Brücke.
 * * LANDMINE FREE FIXES (Deferred Peer Addition, No Blocking in CB).
 * ========================================================================== */

WirelessManager *WirelessManager::_instance = nullptr;
static const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

WirelessManager::WirelessManager(AppManager *appMgr, SensorManager *sensorMgr)
    : _appMgr(appMgr), _sensorMgr(sensorMgr), _peerBound(false),
      _pendingPeerAdd(false), _initialized(false), _connected(false), _lastPacketMs(0)
{
    _instance = this;
    std::memset(_peerMac, 0, 6);
}

bool WirelessManager::init()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);

    // Initialen Broadcast-Peer hinzufügen (für Discovery)
    esp_now_peer_info_t peerInfo;
    std::memset(&peerInfo, 0, sizeof(peerInfo));
    std::memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        return false;
    }

    _initialized = true;
    return true;
}

void WirelessManager::process()
{
    if (!_initialized)
        return;

    // LANDMINE DEFERRED REGISTRATION: Peer sicher im Main-Loop hinzufügen
    if (_pendingPeerAdd)
    {
        _pendingPeerAdd = false;

        esp_now_peer_info_t peerInfo;
        std::memset(&peerInfo, 0, sizeof(peerInfo));
        std::memcpy(peerInfo.peer_addr, _peerMac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (!esp_now_is_peer_exist(_peerMac))
        {
            esp_now_add_peer(&peerInfo);
        }
        _peerBound = true;
    }

    // Timeout-Check für Verbindung (3 Sekunden)
    if (_connected && (millis() - _lastPacketMs > 3000))
    {
        _connected = false;
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void WirelessManager::onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    if (_instance)
        _instance->handleIncomingPacket(esp_now_info->src_addr, data, len);
}
#else
void WirelessManager::onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (_instance)
        _instance->handleIncomingPacket(mac, data, len);
}
#endif

void WirelessManager::handleIncomingPacket(const uint8_t *mac, const uint8_t *data, int len)
{
    // LANDMINE QUEUE RACE: Nur verarbeiten wenn initialisiert
    if (!_initialized || len < (int)sizeof(ProbeEventPacket))
        return;

    ProbeEventPacket pkt;
    std::memcpy(&pkt, data, sizeof(ProbeEventPacket));

    if (pkt.magic != REMOTE_MAGIC)
        return;

    _lastPacketMs = millis();
    _connected = true;

    // Wenn noch nicht gebunden, MAC merken für process()
    if (!_peerBound)
    {
        std::memcpy(_peerMac, mac, 6);
        _pendingPeerAdd = true;
    }

    // Event-Mapping
    InputEvent ev = EV_NONE;
    switch (pkt.event_type)
    {
    case PRB_EVT_T2_CLICK:
        ev = EV_REMOTE_T2_CLICK;
        break;
    case PRB_EVT_T1_CLICK:
        ev = EV_REMOTE_T1_CLICK;
        break;
    case PRB_EVT_ENC_CLICK:
        ev = EV_REMOTE_ENC_CLICK;
        break;
    case PRB_EVT_ENC_UP:
        ev = EV_REMOTE_ENC_UP;
        break;
    case PRB_EVT_ENC_DOWN:
        ev = EV_REMOTE_ENC_DOWN;
        break;
    case PRB_EVT_ENC_LONG:
        ev = EV_REMOTE_ENC_LONG;
        break;

    case PRB_EVT_LUX_DATA:
        // LANDMINE SENSOR_MGR SIGNATUR: Aktuell nur G0 injizieren
        if (_sensorMgr)
        {
            _sensorMgr->injectWirelessLux(pkt.lux_raw_g0);
        }
        return; // Kein App-Event für reine Datenpakete

    case PRB_EVT_HEARTBEAT:
        return;

    default:
        break;
    }

    if (ev != EV_NONE && _appMgr)
    {
        _appMgr->postEvent(ev);
    }
}

void WirelessManager::sendRender(const char* header, const char* l1, const char* l2, 
                               const uint8_t* histogram, uint8_t haptic, ProbeDisplayMode displayMode)
{
    if (!_initialized) return;

    ProbeRenderPacket pkt;
    std::memset(&pkt, 0, sizeof(pkt));
    pkt.magic = REMOTE_MAGIC;
    pkt.command = PRB_CMD_RENDER;

    // Strikte Null-Terminierung
    if (header) { std::strncpy(pkt.header_text, header, 15); pkt.header_text[15] = '\0'; }
    if (l1) { std::strncpy(pkt.line1_text, l1, 15); pkt.line1_text[15] = '\0'; }
    if (l2) { std::strncpy(pkt.line2_text, l2, 15); pkt.line2_text[15] = '\0'; }
    
    // Histogramm wird sicher als Parameter übergeben, nicht per Hack aus dem Context geholt!
    if (histogram) {
        std::memcpy(pkt.zone_histogram, histogram, 11);
    }
    
    pkt.haptic_feedback = haptic;
    pkt.display_mode = static_cast<uint8_t>(displayMode);

    static const uint8_t broadcastMac_local[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    esp_now_send(_peerBound ? _peerMac : broadcastMac_local, (uint8_t*)&pkt, sizeof(pkt));
}

void WirelessManager::sendMeasureCmd(bool isG5)
{
    if (!_initialized)
        return;

    ProbeRenderPacket pkt;
    std::memset(&pkt, 0, sizeof(pkt));
    pkt.magic = REMOTE_MAGIC;
    pkt.command = isG5 ? PRB_CMD_MEASURE_G5 : PRB_CMD_MEASURE_G0;

    const uint8_t *target = _peerBound ? _peerMac : broadcastMac;
    esp_now_send(target, (uint8_t *)&pkt, sizeof(pkt));
}
