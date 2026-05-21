#include "StorageManager.h"
#include "SystemContext.h" 

/* =============================================================================
 * StorageManager.cpp - DUKATIMER BETA (v0.912)
 * (+ HÄRTUNG: Flash-Write Interlock sperrt nun auch während isMeasuring)
 * ========================================================================== */

StorageManager::StorageManager(SystemContext *ctx)
    : _ctx(ctx),
      _providers{nullptr, nullptr, nullptr, nullptr},
      _dirtyLatched{false, false, false, false},
      _dirtySince{0, 0, 0, 0},
      _providerCount(0)
{
}

void StorageManager::registerProvider(IDataProvider *provider)
{
    if (!provider || _providerCount >= MAX_PROVIDERS) return;
    _providers[_providerCount] = provider;
    _dirtyLatched[_providerCount] = false;
    _dirtySince[_providerCount] = 0;
    _providerCount++;
}

bool StorageManager::init()
{
    if (!LittleFS.begin(true)) return false;

    for (size_t i = 0; i < _providerCount; ++i)
    {
        IDataProvider *provider = _providers[i];
        if (!provider) continue;

        File file = LittleFS.open(provider->getFileName(), "r");
        if (!file) continue;

        const size_t fileSize = (size_t)file.size();
        if (fileSize <= sizeof(uint16_t))
        {
            file.close();
            continue;
        }

        const size_t payloadLen = fileSize - sizeof(uint16_t);
        uint8_t *payload = (uint8_t *)malloc(payloadLen);
        if (!payload)
        {
            file.close();
            continue;
        }

        const size_t readPayload = file.read(payload, payloadLen);
        uint16_t storedHash = 0;
        const size_t readHash = file.read((uint8_t *)&storedHash, sizeof(uint16_t));
        file.close();

        if (readPayload == payloadLen && readHash == sizeof(uint16_t))
        {
            provider->deserialize(payload, payloadLen, storedHash);
        }
        free(payload);
    }
    return true;
}

void StorageManager::process()
{
    const unsigned long now = millis();

    // --- ABSOLUTER WRITE INTERLOCK ---
    // Jitter-Vermeidung: Weder bei Belichtung (Core 1) noch bei Sensor-Messung (Core 0)
    // darf das Dateisystem blockieren!
    bool lockOutWrites = false; 

    if (_ctx)
    {
        WorkflowFlags flags;
        if (_ctx->getFlags(flags))
        {
            lockOutWrites = flags.isExposureRunning || flags.isMeasuring;
        }
        else
        {
            // Fail-Closed: Bei Mutex-Timeout zur Sicherheit sperren.
            lockOutWrites = true; 
        }
    }

    for (size_t i = 0; i < _providerCount; ++i)
    {
        IDataProvider *provider = _providers[i];
        if (!provider) continue;

        const bool isDirtyNow = provider->isDirty();

        if (isDirtyNow && !_dirtyLatched[i])
        {
            _dirtyLatched[i] = true;
            _dirtySince[i] = now;
        }
        else if (!isDirtyNow)
        {
            _dirtyLatched[i] = false;
            _dirtySince[i] = 0;
            continue;
        }

        if ((now - _dirtySince[i]) < DEFER_DELAY_MS) continue;

        if (lockOutWrites) continue; // Wartet auf Ende der Belichtung/Messung

        provider->clearDirty(); 
        
        uint8_t *buffer = nullptr;
        size_t length = 0;
        uint16_t hash = 0;

        if (!provider->serialize(&buffer, &length, &hash))
        {
            provider->markDirty(); 
            continue;
        }

        bool writeOk = false;
        File file = LittleFS.open(provider->getFileName(), "w");
        if (file)
        {
            const size_t writtenPayload = file.write(buffer, length);
            const size_t writtenHash = file.write((const uint8_t *)&hash, sizeof(uint16_t));
            file.close();
            writeOk = (writtenPayload == length) && (writtenHash == sizeof(uint16_t));
        }

        if (buffer) free(buffer);

        if (!writeOk)
        {
            provider->markDirty(); 
            continue;
        }

        _dirtyLatched[i] = false;
        _dirtySince[i] = 0;
    }
}