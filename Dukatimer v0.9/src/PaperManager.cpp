#include "PaperManager.h"
#include <cmath>
#include <cstring>

/* =============================================================================
 * PaperManager.cpp - DUKATIMER BETA (v0.916)
 * * (+ FIX T1: Thread-sicheres Dirty-Flag Management via std::atomic)
 * * (+ FIX T7: NVS-Zugriff ohne const_cast durch mutable Member)
 * ========================================================================== */

namespace
{
    // Sicheres RAII-Pattern für FreeRTOS Mutexe
    class MutexGuard
    {
        SemaphoreHandle_t _mutex;
        bool _acquired;

    public:
        MutexGuard(SemaphoreHandle_t m, TickType_t t)
            : _mutex(m), _acquired(m != nullptr && xSemaphoreTake(m, t) == pdTRUE) {}
        ~MutexGuard()
        {
            if (_acquired)
                xSemaphoreGive(_mutex);
        }
        bool isAcquired() const { return _acquired; }
    };

    static const TickType_t DB_WAIT = pdMS_TO_TICKS(150);

    float clampFloat(float value, float minValue, float maxValue)
    {
        if (value < minValue)
            return minValue;
        if (value > maxValue)
            return maxValue;
        return value;
    }

    int gradeIndexFromFloat(float grade)
    {
        const float clampedGrade = clampFloat(grade, 0.0f, 5.0f);
        int index = static_cast<int>((clampedGrade * 2.0f) + 0.1f);
        if (index < 0)
            index = 0;
        if (index > 10)
            index = 10;
        return index;
    }

    void computeSplitFactors(const PaperProfile &profile, float grade, float &softFactor, float &hardFactor)
    {
        const float clampedGrade = clampFloat(grade, 0.0f, 5.0f);

        if (profile.useIsoMath)
        {
            hardFactor = clampedGrade / 5.0f;
            softFactor = 1.0f - hardFactor;
            return;
        }

        const int index = gradeIndexFromFloat(clampedGrade);
        softFactor = profile.gradeK_Soft[index];
        hardFactor = profile.gradeK_Hard[index];

        if (softFactor <= 0.0001f && hardFactor <= 0.0001f)
        {
            hardFactor = clampedGrade / 5.0f;
            softFactor = 1.0f - hardFactor;
        }
    }
}

// =============================================================================
// LEBENSZYKLUS & INIT
// =============================================================================

PaperManager::PaperManager(SystemContext *ctx)
    : _ctx(ctx), _bank(nullptr)
{
    _isDirty.store(false);
    _dbMutex = xSemaphoreCreateMutex();
}

PaperManager::~PaperManager()
{
    if (_dbMutex)
        vSemaphoreDelete(_dbMutex);
    if (_bank)
        free(_bank);
}

bool PaperManager::init()
{
    if (!_dbMutex)
        return false;

    // Speicher im SPIRAM (PSRAM) allokieren
    _bank = (PaperBank *)heap_caps_malloc(sizeof(PaperBank), MALLOC_CAP_SPIRAM);
    if (!_bank)
        return false;

    // 1. Initialer Grundzustand
    factoryReset();

    // 2. NVS Fallback (Lädt den letzten aktiven Papier-Index/Profil)
    _loadActivePaperFromNVS();

    // Nach dem Booten ist der Zustand sauber, bis der StorageManager
    // evtl. neuere Daten vom Flash liefert.
    _isDirty.store(false);
    return true;
}

bool PaperManager::_isWriteLocked() const
{
    if (!_ctx)
        return false;
    WorkflowFlags flags;
    if (_ctx->getFlags(flags))
    {
        return flags.isExposureRunning || flags.isMeasuring;
    }
    return true;
}

float PaperManager::_applyDLogH(float x) const
{
    float t = (x < 0.0f) ? 0.0f : ((x > 1.0f) ? 1.0f : x);
    return t * t * (3.0f - 2.0f * t);
}

uint16_t PaperManager::_calculateHash(const void *data, size_t len) const
{
    const uint8_t *p = static_cast<const uint8_t *>(data);
    uint16_t hash = 0x811C;
    for (size_t i = 0; i < len; i++)
    {
        hash ^= p[i];
        hash *= 0x0101;
    }
    return hash;
}

// =============================================================================
// SERIALISIERUNG & MIGRATION
// =============================================================================

bool PaperManager::serialize(uint8_t **buffer, size_t *length, uint16_t *outHash)
{
    if (!_bank)
        return false;

    MutexGuard lock(_dbMutex, DB_WAIT);
    if (!lock.isAcquired())
        return false;

    const size_t payloadLen = offsetof(PaperBank, hash);
    *buffer = (uint8_t *)malloc(payloadLen);
    if (!*buffer)
        return false;

    memcpy(*buffer, _bank, payloadLen);
    *length = payloadLen;
    *outHash = _calculateHash(*buffer, payloadLen);
    _bank->hash = *outHash;

    return true;
}

bool PaperManager::deserialize(const uint8_t *buffer, size_t length, uint16_t hash)
{
    if (!buffer || !_bank || length == 0)
        return false;

    const uint16_t expectedHash = _calculateHash(buffer, length);
    if (expectedHash != hash)
    {
        Serial.printf("[STORAGE] Hash mismatch: 0x%04x != 0x%04x\n", hash, expectedHash);
        return false;
    }

    const uint16_t version = *reinterpret_cast<const uint16_t *>(buffer);

    if (version == 0x0901)
    {
        return _migrateBankAndUpdate(buffer, length, version);
    }

    PaperBank *loaded = (PaperBank *)heap_caps_malloc(sizeof(PaperBank), MALLOC_CAP_SPIRAM);
    if (!loaded)
        return false;

    memset(loaded, 0, sizeof(PaperBank));
    const size_t currentPayloadLen = offsetof(PaperBank, hash);
    const size_t copyLen = (length < currentPayloadLen) ? length : currentPayloadLen;
    memcpy(loaded, buffer, copyLen);
    loaded->hash = hash;

    bool ok = restoreBankFromStorage(loaded);
    free(loaded);

    if (ok)
    {
        _saveActivePaperToNVS();
    }

    return ok;
}

namespace
{
    struct __attribute__((packed)) LegacyPaperProfile_v0901
    {
        char name[24];
        bool isFixedGrade;
        double fixedGradeValue;
        bool useIsoMath;
        double isoP;
        double isoR;
        double Kbw;
        double Ksoft;
        double Khard;
        double gradeK_Soft[11];
        double gradeK_Hard[11];
        bool calibrated;
        bool flashEnable;
        bool flashCalibrated;
        int flashLevel;
        int flashColor;
        double flashThreshS;
        double flashFactor;
    };

    struct __attribute__((packed)) LegacyPaperBank_v0901
    {
        uint16_t version;
        uint8_t activeIndex;
        LegacyPaperProfile_v0901 profiles[20];
        uint16_t hash;
    };
}

bool PaperManager::_migrateBankAndUpdate(const uint8_t *legacyBuffer, size_t length, uint16_t legacyVersion)
{
    const size_t expectedLen = sizeof(LegacyPaperBank_v0901) - offsetof(LegacyPaperBank_v0901, hash);
    if (length != expectedLen)
        return false;

    const auto *legacyBank = reinterpret_cast<const LegacyPaperBank_v0901 *>(legacyBuffer);

    MutexGuard lock(_dbMutex, pdMS_TO_TICKS(500));
    if (!lock.isAcquired())
        return false;

    std::memset(_bank, 0, sizeof(PaperBank));
    _bank->version = SW_VERSION_HEX;
    _bank->activeIndex = (legacyBank->activeIndex < 20) ? legacyBank->activeIndex : 0;

    for (int i = 0; i < 20; ++i)
    {
        const auto &lp = legacyBank->profiles[i];
        auto &np = _bank->profiles[i];

        std::memcpy(np.name, lp.name, 24);
        np.name[23] = '\0';

        np.isFixedGrade = lp.isFixedGrade;
        np.useIsoMath = lp.useIsoMath;
        np.calibrated = lp.calibrated;
        np.flashEnable = lp.flashEnable;
        np.flashCalibrated = lp.flashCalibrated;
        np.flashLevel = lp.flashLevel;
        np.flashColor = lp.flashColor;

        np.fixedGradeValue = static_cast<float>(lp.fixedGradeValue);
        np.isoP = static_cast<float>(lp.isoP);
        np.isoR = static_cast<float>(lp.isoR);
        np.Kbw = static_cast<float>(lp.Kbw);
        np.Ksoft = static_cast<float>(lp.Ksoft);
        np.Khard = static_cast<float>(lp.Khard);

        np.flashThreshD = static_cast<float>(lp.flashThreshS);
        np.flashFactor = static_cast<float>(lp.flashFactor);

        for (int j = 0; j < 11; j++)
        {
            np.gradeK_Soft[j] = static_cast<float>(lp.gradeK_Soft[j]);
            np.gradeK_Hard[j] = static_cast<float>(lp.gradeK_Hard[j]);
        }
    }

    _isDirty.store(true);
    return true;
}

// =============================================================================
// LESE-ZUGRIFF
// =============================================================================

bool PaperManager::getActiveProfileCopy(PaperProfile &outProfile) const
{
    if (!_bank)
        return false;
    MutexGuard lock(_dbMutex, DB_WAIT);
    if (!lock.isAcquired())
        return false;

    uint8_t idx = _bank->activeIndex;
    outProfile = _bank->profiles[idx < 20 ? idx : 0];
    return true;
}

bool PaperManager::isFixedGradeActive() const
{
    PaperProfile p;
    return getActiveProfileCopy(p) ? p.isFixedGrade : false;
}

float PaperManager::getActiveFixedGradeValue() const
{
    PaperProfile p;
    return getActiveProfileCopy(p) ? p.fixedGradeValue : 2.5f;
}

bool PaperManager::calculateSplitGradeTimes(float baseTime, float grade, float &softTime, float &hardTime) const
{
    softTime = clampFloat(baseTime, 0.0f, TIME_MAX_S);
    hardTime = 0.0f;

    PaperProfile profile;
    if (!getActiveProfileCopy(profile))
    {
        return false;
    }

    if (profile.isFixedGrade)
    {
        return true;
    }

    float softFactor = 0.0f;
    float hardFactor = 0.0f;
    computeSplitFactors(profile, grade, softFactor, hardFactor);

    float compensatedBaseTime = baseTime;
    if (profile.useIsoMath)
    {
        float speedP = (profile.isoP > 10.0f) ? profile.isoP : 100.0f;
        compensatedBaseTime = baseTime * (100.0f / speedP);
    }

    softTime = clampFloat(compensatedBaseTime * softFactor, 0.0f, TIME_MAX_S);
    hardTime = clampFloat(compensatedBaseTime * hardFactor, 0.0f, TIME_MAX_S);
    return true;
}

bool PaperManager::calculateSplitGradeDose(float baseDose, float grade, float &softDose, float &hardDose) const
{
    softDose = fmaxf(baseDose, 0.0f);
    hardDose = 0.0f;

    PaperProfile profile;
    if (!getActiveProfileCopy(profile))
    {
        return false;
    }

    if (profile.isFixedGrade)
    {
        return true;
    }

    float softFactor = 0.0f;
    float hardFactor = 0.0f;
    computeSplitFactors(profile, grade, softFactor, hardFactor);

    softDose = fmaxf(baseDose * softFactor, 0.0f);
    hardDose = fmaxf(baseDose * hardFactor, 0.0f);
    return true;
}

uint8_t PaperManager::getActiveIndex() const
{
    uint8_t idx = 0;
    MutexGuard lock(_dbMutex, DB_WAIT);
    if (lock.isAcquired() && _bank)
    {
        idx = _bank->activeIndex;
    }
    return idx;
}

// =============================================================================
// SCHREIB-ZUGRIFF
// =============================================================================

bool PaperManager::setActiveIndex(uint8_t index)
{
    if (index >= 20 || _isWriteLocked() || !_bank)
        return false;

    MutexGuard lock(_dbMutex, DB_WAIT);
    if (!lock.isAcquired())
        return false;

    if (_bank->activeIndex != index)
    {
        _bank->activeIndex = index;
        _isDirty.store(true);
    }
    return true;
}

bool PaperManager::updateActiveProfile(const PaperProfile &newProfile)
{
    if (_isWriteLocked() || !_bank)
        return false;

    MutexGuard lock(_dbMutex, DB_WAIT);
    if (!lock.isAcquired())
        return false;

    uint8_t idx = _bank->activeIndex;
    if (idx < 20)
    {
        _bank->profiles[idx] = newProfile;
        _isDirty.store(true);
    }
    return true;
}

// =============================================================================
// KALIBRIERUNG & PERSISTENZ
// =============================================================================

bool PaperManager::calculateAndSaveCalibration(int stepW0, int stepB0, int stepW5, int stepB5,
                                               float calibTime, float luxG0, float luxG5)
{
    if (_isWriteLocked() || !_bank)
        return false;
    if (luxG0 <= 0.0f || luxG5 <= 0.0f || calibTime <= 0.0f)
        return false;

    auto calcDose = [&](int step, float baseLux) -> float
    {
        float density = ((float)(step - 1) * 0.15f) + 0.05f;
        float transmission = powf(10.0f, -density);
        return baseLux * transmission * calibTime;
    };

    MutexGuard lock(_dbMutex, DB_WAIT);
    if (!lock.isAcquired())
        return false;

    PaperProfile &p = _bank->profiles[_bank->activeIndex];

    p.Ksoft = calcDose(stepW0, luxG0);
    p.Khard = calcDose(stepW5, luxG5);
    p.isFixedGrade = false;
    p.useIsoMath = false;

    for (int i = 0; i < 11; i++)
    {
        float sw = _applyDLogH((float)i / 10.0f);
        p.gradeK_Soft[i] = p.Ksoft * (1.0f - sw);
        p.gradeK_Hard[i] = p.Khard * sw;
    }

    p.calibrated = true;
    _isDirty.store(true);
    return true;
}

bool PaperManager::factoryReset()
{
    if (_isWriteLocked() || !_bank)
        return false;

    MutexGuard lock(_dbMutex, pdMS_TO_TICKS(500));
    if (!lock.isAcquired())
        return false;

    _bank->version = SW_VERSION_HEX;
    _bank->activeIndex = 1;

    for (int i = 0; i < 20; i++)
    {
        PaperProfile &p = _bank->profiles[i];
        memset(&p, 0, sizeof(PaperProfile));
        snprintf(p.name, sizeof(p.name), "Leer %d", i);
        p.isFixedGrade = false;
        p.fixedGradeValue = 2.5f;
        for (int g = 0; g < 11; g++)
        {
            p.gradeK_Soft[g] = 10.0f * (1.0f - ((float)g / 10.0f));
            p.gradeK_Hard[g] = 10.0f * ((float)g / 10.0f);
        }
    }

    PaperProfile &p1 = _bank->profiles[1];
    snprintf(p1.name, sizeof(p1.name), "Ilford MGIV RC");
    p1.Kbw = 15.0f;
    p1.Ksoft = 12.0f;
    p1.Khard = 12.0f;
    for (int g = 0; g < 11; g++)
    {
        float sw = _applyDLogH((float)g / 10.0f);
        p1.gradeK_Soft[g] = p1.Ksoft * (1.0f - sw);
        p1.gradeK_Hard[g] = p1.Khard * sw;
    }

    _isDirty.store(true);
    return true;
}

PaperBank *PaperManager::allocateBankCopyForStorage() const
{
    if (!_bank)
        return nullptr;
    PaperBank *copy = (PaperBank *)heap_caps_malloc(sizeof(PaperBank), MALLOC_CAP_SPIRAM);
    if (!copy)
        return nullptr;

    MutexGuard lock(_dbMutex, DB_WAIT);
    if (lock.isAcquired())
    {
        memcpy(copy, _bank, sizeof(PaperBank));
        return copy;
    }
    free(copy);
    return nullptr;
}

bool PaperManager::restoreBankFromStorage(const PaperBank *loadedData)
{
    if (_isWriteLocked() || !loadedData || !_bank)
        return false;

    MutexGuard lock(_dbMutex, pdMS_TO_TICKS(500));
    if (lock.isAcquired())
    {
        memcpy(_bank, loadedData, sizeof(PaperBank));
        _isDirty.store(false);
        return true;
    }
    return false;
}

// =============================================================================
// NVS FALLBACK FUNKTIONEN
// =============================================================================

void PaperManager::_saveActivePaperToNVS() const
{
    PaperProfile activePaper;
    // T7: Dank mutable _nvs ist kein const_cast mehr nötig. Bulletproof.
    if (getActiveProfileCopy(activePaper) && _nvs.begin(NVS_SPACE, false))
    {
        _nvs.putBytes("paper_active", &activePaper, sizeof(PaperProfile));
        _nvs.end();
    }
}

void PaperManager::_loadActivePaperFromNVS()
{
    if (_nvs.begin(NVS_SPACE, true))
    {
        if (_nvs.isKey("paper_active") && _nvs.getBytesLength("paper_active") == sizeof(PaperProfile))
        {
            PaperProfile nvsPaper;
            _nvs.getBytes("paper_active", &nvsPaper, sizeof(PaperProfile));
            updateActiveProfile(nvsPaper);
        }
        _nvs.end();
    }
}