#ifndef PAPER_MANAGER_H
#define PAPER_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <atomic>
#include "types.h"
#include "config.h"
#include "SystemContext.h"
#include "IDataProvider.h"

/* =============================================================================
 * PaperManager.h - DUKATIMER BETA (v0.916.3)
 * * PSRAM-basierte Verwaltung der Papierprofile.
 * (+ FIX: Doppelte Deklarationen von _isDirty und _nvs entfernt)
 * ========================================================================== */

struct __attribute__((packed)) PaperProfile
{
    char name[24];
    bool isFixedGrade;
    float fixedGradeValue;
    bool useIsoMath;
    float isoP;
    float isoR;
    float Kbw;
    float Ksoft;
    float Khard;
    float gradeK_Soft[11];
    float gradeK_Hard[11];
    bool calibrated;
    bool flashEnable;
    bool flashCalibrated;
    int flashLevel;
    int flashColor;
    float flashThreshD;
    float flashFactor;
};

struct __attribute__((packed)) PaperBank
{
    uint16_t version;
    uint8_t activeIndex;
    PaperProfile profiles[20];
    uint16_t hash;
};

class PaperManager : public IDataProvider
{
public:
    PaperManager(SystemContext *ctx);
    ~PaperManager();

    bool init();

    bool getActiveProfileCopy(PaperProfile &outProfile) const;
    uint8_t getActiveIndex() const;
    bool isFixedGradeActive() const;
    float getActiveFixedGradeValue() const;
    bool calculateSplitGradeTimes(float baseTime, float grade, float &softTime, float &hardTime) const;
    bool calculateSplitGradeDose(float baseDose, float grade, float &softDose, float &hardDose) const;

    bool setActiveIndex(uint8_t index);
    bool updateActiveProfile(const PaperProfile &newProfile);
    bool calculateAndSaveCalibration(int stepW0, int stepB0, int stepW5, int stepB5,
                                     float calibTime, float luxG0, float luxG5);
    bool factoryReset();

    bool isDirty() const override { return _isDirty.load(); }
    void clearDirty() override { _isDirty.store(false); }
    void markDirty() override { _isDirty.store(true); }

    bool serialize(uint8_t **buffer, size_t *length, uint16_t *outHash) override;
    bool deserialize(const uint8_t *buffer, size_t length, uint16_t hash) override;
    const char *getFileName() const override { return "/papers.bin"; }

    PaperBank *allocateBankCopyForStorage() const;
    bool restoreBankFromStorage(const PaperBank *loadedData);

private:
    SystemContext *_ctx;
    SemaphoreHandle_t _dbMutex;
    PaperBank *_bank;

    std::atomic<bool> _isDirty{false};
    mutable Preferences _nvs;

    bool _isWriteLocked() const;
    float _applyDLogH(float x) const;
    uint16_t _calculateHash(const void *data, size_t len) const;

    bool _migrateBankAndUpdate(const uint8_t *legacyBuffer, size_t length, uint16_t legacyVersion);

    void _saveActivePaperToNVS() const;
    void _loadActivePaperFromNVS();
};

static_assert(sizeof(PaperProfile) == 157, "PaperProfile size mismatch!");
static_assert(sizeof(PaperBank) == 3145, "PaperBank size mismatch!");

#endif