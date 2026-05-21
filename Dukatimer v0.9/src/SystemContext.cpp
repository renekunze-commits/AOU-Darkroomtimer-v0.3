#include "SystemContext.h"
#include <cstring>

/* =============================================================================
 * SystemContext.cpp - DUKATIMER BETA (v0.917.2)
 * * REVISION: Wireless Histogram & Low-Effort Undo Logic
 * ========================================================================== */

namespace
{
    class MutexGuard
    {
        SemaphoreHandle_t _mutex;
        bool _acquired;

    public:
        MutexGuard(SemaphoreHandle_t m, TickType_t t = pdMS_TO_TICKS(50))
            : _mutex(m), _acquired(xSemaphoreTake(m, t) == pdTRUE) {}

        ~MutexGuard()
        {
            if (_acquired)
            {
                xSemaphoreGive(_mutex);
            }
        }
        bool isAcquired() const { return _acquired; }
    };
}

struct __attribute__((packed)) SettingsBlob
{
    SystemPreferences prefs;
    uint16_t hash;
};

static uint16_t calculateHash16(const void *data, size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i)
    {
        h ^= bytes[i];
        h *= 16777619u;
    }
    return (uint16_t)((h >> 16) ^ (h & 0xFFFF));
}

static const TickType_t UI_WAIT = pdMS_TO_TICKS(30);
static const TickType_t CRITICAL_WAIT = pdMS_TO_TICKS(50);

SystemContext::SystemContext()
{
    _mutex = xSemaphoreCreateMutex();

    _exp.targetDoseBw = 12.0f;
    _exp.targetDoseSoft = 10.0f;
    _exp.targetDoseHard = 5.0f;
    _exp.grade = 2.5f;
    _exp.doseModeActive = true;

    _hw.headLux = 0.0f;
    _hw.baseLux = 0.0f;
    _hw.tempAlu = 25.0f;
    _hw.tempRoom = 22.0f;
    _hw.liveDose = 0.0f;
    _hw.liveTime = 0.0f;
    _hw.overheatActive = false;
    _hw.lastError = ERR_NONE;

    _flags.currentMode = MODE_BW;
    _flags.isExposureRunning = false;
    _flags.isMeasuring = false;
    _flags.measProgress = 0;
    _flags.bwAutoPending = false;
    _flags.sgAutoPending = false;
    _flags.isCalibrating = false;
    _flags.modePreviewActive = false;
    _flags.previewMode = MODE_BW;
    _flags.pendingDose = 0.0f;
    _flags.pendingGrade = 2.5f;
    
    // --- HISTOGRAMM INIT ---
    std::memset(_flags.zoneHistogram, 0, sizeof(_flags.zoneHistogram));
    _flags.lastMeasuredZone = -1;

    _flags.appState = AppSharedState{};
    _flags.appState.activeStateMode = MODE_BW;

    _prefs.pwmSafe = 100;
    _prefs.pwmFocus = 255;
    _prefs.pwmLcd = 100;
    _prefs.pwmMax = 255;
    _prefs.baseDarkLux = 0.0f;
    _prefs.probeDarkLux = 0.0f;
    _prefs.soundMode = 1;
    _prefs.stepMode = 2;
    _prefs.bwTargetZone = 8.0f;
    _prefs.useWirelessProbe = 1;
    _prefs.stdTime = 15.0f;
    _prefs.splitModeAuto = 1;

    _isDirty.store(false);
}

SystemContext::~SystemContext()
{
    if (_mutex) vSemaphoreDelete(_mutex);
}

// --- GETTER ---
bool SystemContext::getExposure(ExposureParams &out) { MutexGuard lock(_mutex, UI_WAIT); if (!lock.isAcquired()) return false; out = _exp; return true; }
bool SystemContext::getStatus(HardwareStatus &out) { MutexGuard lock(_mutex, UI_WAIT); if (!lock.isAcquired()) return false; out = _hw; return true; }
bool SystemContext::getFlags(WorkflowFlags &out) { MutexGuard lock(_mutex, UI_WAIT); if (!lock.isAcquired()) return false; out = _flags; return true; }
bool SystemContext::getPreferences(SystemPreferences &out) { MutexGuard lock(_mutex, UI_WAIT); if (!lock.isAcquired()) return false; out = _prefs; return true; }

// --- SETTER ---
bool SystemContext::setBWDose(float dose) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _exp.targetDoseBw = dose; return true; }
bool SystemContext::setSplitDoses(float soft, float hard) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _exp.targetDoseSoft = soft; _exp.targetDoseHard = hard; return true; }
bool SystemContext::setGrade(float grade) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _exp.grade = grade; return true; }
bool SystemContext::setDoseMode(bool active) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _exp.doseModeActive = active; return true; }
bool SystemContext::setPreferences(const SystemPreferences &prefs) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _prefs = prefs; _isDirty.store(true); return true; }
bool SystemContext::setAppState(const AppSharedState &state) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _flags.appState = state; _flags.appState.activeStateMode = _flags.currentMode; return true; }

bool SystemContext::setMode(Mode mode)
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return false;
    _flags.currentMode = mode;
    _flags.bwAutoPending = false;
    _flags.sgAutoPending = false;
    _flags.appState = AppSharedState{};
    _flags.appState.activeStateMode = mode;
    return true;
}

bool SystemContext::setModePreview(bool active, Mode preview) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _flags.modePreviewActive = active; _flags.previewMode = preview; return true; }
bool SystemContext::setExposureState(bool running) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _flags.isExposureRunning = running; return true; }
bool SystemContext::setMeasuringState(bool measuring) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _flags.isMeasuring = measuring; if (!measuring) _flags.measProgress = 0; return true; }
bool SystemContext::updateMeasuringProgress(uint8_t progress) { MutexGuard lock(_mutex, 0); if (!lock.isAcquired()) return false; _flags.measProgress = (progress > 100) ? 100 : progress; return true; }

bool SystemContext::setBWPending(bool pending, float suggestedDose)
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return false;
    _flags.bwAutoPending = pending;
    if (pending) {
        _flags.pendingDose = suggestedDose;
        // FIX: Im reinen Zeitmodus (MODE_BW) Dosisintegration abschalten
        if (_flags.currentMode == MODE_BW) {
            _exp.doseModeActive = false;
        }
    }
    return true;
}

bool SystemContext::setSGPending(bool pending, float suggestedDose, float suggestedGrade)
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return false;
    _flags.sgAutoPending = pending;
    if (pending) {
        _flags.pendingDose = suggestedDose;
        _flags.pendingGrade = suggestedGrade;
    }
    return true;
}

bool SystemContext::setLastError(SystemError err) { MutexGuard lock(_mutex, CRITICAL_WAIT); if (!lock.isAcquired()) return false; _hw.lastError = err; return true; }
bool SystemContext::updateSensors(float head, float base, float tempA, float tempR) { MutexGuard lock(_mutex, 0); if (!lock.isAcquired()) return false; _hw.headLux = head; _hw.baseLux = base; _hw.tempAlu = tempA; _hw.tempRoom = tempR; _hw.overheatActive = (tempA >= TEMP_MAX_ALU); return true; }
bool SystemContext::updateLiveDose(float currentDose) { MutexGuard lock(_mutex, 0); if (!lock.isAcquired()) return false; _hw.liveDose = currentDose; return true; }
bool SystemContext::updateLiveTime(float seconds) { MutexGuard lock(_mutex, 0); if (!lock.isAcquired()) return false; _hw.liveTime = seconds; return true; }

// --- IDataProvider ---
bool SystemContext::isDirty() const { return _isDirty.load(); }
void SystemContext::clearDirty() { _isDirty.store(false); }
void SystemContext::markDirty() { _isDirty.store(true); }

bool SystemContext::serialize(uint8_t **buffer, size_t *length, uint16_t *outHash)
{
    if (!buffer || !length || !outHash) return false;
    SettingsBlob blob;
    {
        MutexGuard lock(_mutex, CRITICAL_WAIT);
        if (!lock.isAcquired()) return false;
        blob.prefs = _prefs;
    }
    const size_t payloadLen = offsetof(SettingsBlob, hash);
    uint8_t *buf = (uint8_t *)malloc(payloadLen);
    if (!buf) return false;
    memcpy(buf, &blob.prefs, payloadLen);
    *buffer = buf; *length = payloadLen; *outHash = calculateHash16(buf, payloadLen);
    return true;
}

bool SystemContext::deserialize(const uint8_t *buffer, size_t length, uint16_t hash)
{
    if (!buffer) return false;
    const size_t payloadLen = offsetof(SettingsBlob, hash);
    if (length != payloadLen || calculateHash16(buffer, length) != hash) return false;
    SettingsBlob tmp; std::memset(&tmp, 0, sizeof(SettingsBlob)); memcpy(&tmp.prefs, buffer, length);
    {
        MutexGuard lock(_mutex, CRITICAL_WAIT);
        if (!lock.isAcquired()) return false;
        _prefs = tmp.prefs;
    }
    _isDirty.store(false);
    return true;
}

const char *SystemContext::getFileName() const { return "/settings.bin"; }

// =============================================================================
// BELICHTUNGS-STEUERUNG & HISTOGRAMM
// =============================================================================

bool SystemContext::setSGTimePending(float softS, float hardS)
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return false;
    _exp.targetDoseSoft = softS;
    _exp.targetDoseHard = hardS;
    _exp.doseModeActive = false;
    _flags.sgAutoPending = true;
    return true;
}

void SystemContext::triggerAbort()
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return;
    _flags.isExposureRunning = false;
    _flags.isMeasuring = false;
    _flags.bwAutoPending = false;
    _flags.sgAutoPending = false;
}

void SystemContext::clearHistogram()
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return;
    std::memset(_flags.zoneHistogram, 0, sizeof(_flags.zoneHistogram));
    _flags.lastMeasuredZone = -1;
}

void SystemContext::addHistogramSpot(uint8_t zone)
{
    if (zone > 10) return;
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return;
    
    // v0.3 Logik: Ein Spot entspricht 20 Einheiten (Anzeige-Skalierung)
    if (_flags.zoneHistogram[zone] <= 220) {
        _flags.zoneHistogram[zone] += 20;
    } else {
        _flags.zoneHistogram[zone] = 240; // Hard Cap
    }
    _flags.lastMeasuredZone = static_cast<int8_t>(zone);
}

void SystemContext::undoLastHistogramSpot()
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return;
    
    if (_flags.lastMeasuredZone >= 0 && _flags.lastMeasuredZone <= 10) {
        if (_flags.zoneHistogram[_flags.lastMeasuredZone] >= 20) {
            _flags.zoneHistogram[_flags.lastMeasuredZone] -= 20;
        } else {
            _flags.zoneHistogram[_flags.lastMeasuredZone] = 0;
        }
        _flags.lastMeasuredZone = -1; // Nur einmaliger Undo möglich
    }
}