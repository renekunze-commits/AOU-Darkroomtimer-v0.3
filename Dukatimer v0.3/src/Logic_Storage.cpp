/*
  Logic_Storage.cpp - v0.5 Root Cause Edition
  
  Zentrales Management der Persistenz (EEPROM & NVS).
  Sorgt für das "Grounding" der Benutzerdaten über Power-Zyklen hinweg.
  
  Layout:
  - 0-511: SettingsObject (Systemkonfiguration)
  - 512-1999: PaperBank (20 Profile, via Logic_Papers)
  - 2000+: System Error Log
*/

#include <Arduino.h>
#include <LittleFS.h>
#include <FS.h>
#include <Preferences.h>
#include <nvs_flash.h> // Hinzugefügt für nvs_flash_init
#include <stddef.h> // Hinzugefügt für offsetof()
#include "Globals.h"
#include "Config.h"
#include "Logic_Papers.h"
#include "Logic_Storage.h"
#include "Logic_Math.h"

// Externe Abhängigkeiten für das Boot-Sequence-Grounding
extern void validateTimes();
extern void resetTracking();
extern void initializeDoseStateFromCurrentTimes();

// ROOT CAUSE FIX: Definition der in Globals.h versprochenen Variablen
volatile unsigned long lastSettingChange = 0;
volatile bool settingsDirty = false;

static Preferences preferences;
const char* NVS_NAMESPACE = "dukatimer";
const char* NVS_KEY_PAPER = "paper_active";
// BETA-FIX: [Phase 2] NVS-Key fuer BW-Zielzone (<=15 Zeichen wegen ESP32-NVS-Limit).
const char* NVS_KEY_BW_TARGET_ZONE = "bwTZone";

static const char* SETTINGS_FILE_PATH = "/settings.bin";
static const char* PAPERS_FILE_PATH = "/papers.bin";
static const char* ERROR_FILE_PATH = "/error.bin";

static const uint32_t MAGIC_SETTINGS = 0x44555354; // "DUST"
static const uint32_t MAGIC_PAPERS = 0x44555042;   // "DUPB"
static const uint32_t MAGIC_ERROR = 0x44554552;    // "DUER"

struct StorageBlobHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t payloadSize;
    uint16_t payloadCrc;
    uint16_t reserved;
};

// =============================================================================
// HELPER: INTEGRITÄTS-CHECK (FNV-1a 32-bit CRC)
// =============================================================================
static uint32_t fnv1a32(const uint8_t* data, size_t len) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        h ^= data[i];
        h *= 16777619u;
    }
    return h;
}

uint16_t calcCrc(const SettingsObject& s) {
    // ROOT CAUSE FIX: offsetof() verhindert Probleme mit Compiler-Padding
    // Berechnet CRC exakt bis zum Beginn des 'crc'-Feldes.
    uint32_t h = fnv1a32((const uint8_t*)&s, offsetof(SettingsObject, crc));
    // ROOT CAUSE FIX: Sauberer XOR-Fold von 32-Bit auf 16-Bit (Erhalt der Entropie)
    return (uint16_t)((h >> 16) ^ (h & 0xFFFF));
}

static uint16_t calcBlobCrc(const void* data, size_t len) {
    uint32_t h = fnv1a32((const uint8_t*)data, len);
    return (uint16_t)((h >> 16) ^ (h & 0xFFFF));
}

static bool writeBlob(const char* path,
                      uint32_t magic,
                      uint16_t version,
                      const void* payload,
                      size_t payloadSize,
                      uint16_t payloadCrc) {
    if (LittleFS.exists(path)) {
        LittleFS.remove(path);
    }

    File f = LittleFS.open(path, FILE_WRITE);
    if (!f) {
        Serial.printf("[STORAGE] Kann %s nicht zum Schreiben oeffnen.\n", path);
        return false;
    }

    StorageBlobHeader hdr;
    hdr.magic = magic;
    hdr.version = version;
    hdr.payloadSize = (uint16_t)payloadSize;
    hdr.payloadCrc = payloadCrc;
    hdr.reserved = 0;

    size_t wrote = 0;
    wrote += f.write((const uint8_t*)&hdr, sizeof(hdr));
    wrote += f.write((const uint8_t*)payload, payloadSize);
    f.close();

    if (wrote != sizeof(hdr) + payloadSize) {
        Serial.printf("[STORAGE] Schreibfehler bei %s.\n", path);
        return false;
    }
    return true;
}

static bool readBlob(const char* path,
                     uint32_t expectedMagic,
                     uint16_t expectedVersion,
                     void* payloadOut,
                     size_t expectedPayloadSize,
                     uint16_t* payloadCrcOut = nullptr) {
    if (!LittleFS.exists(path)) return false;

    File f = LittleFS.open(path, FILE_READ);
    if (!f) return false;

    if ((size_t)f.size() != sizeof(StorageBlobHeader) + expectedPayloadSize) {
        f.close();
        return false;
    }

    StorageBlobHeader hdr;
    if (f.read((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr)) {
        f.close();
        return false;
    }

    if (hdr.magic != expectedMagic || hdr.version != expectedVersion ||
        hdr.payloadSize != expectedPayloadSize) {
        f.close();
        return false;
    }

    if (f.read((uint8_t*)payloadOut, expectedPayloadSize) != (int)expectedPayloadSize) {
        f.close();
        return false;
    }

    f.close();

    if (payloadCrcOut) {
        *payloadCrcOut = hdr.payloadCrc;
    }
    return true;
}

void markDirty() {
    settingsDirty = true;
    lastSettingChange = millis();
}

static bool writeSettingsFile(const SettingsObject& s) {
    return writeBlob(
        SETTINGS_FILE_PATH,
        MAGIC_SETTINGS,
        SW_VERSION,
        &s,
        sizeof(SettingsObject),
        s.crc
    );
}

static bool writePapersFile() {
    uint16_t crc = calcBlobCrc(&paperBank, sizeof(PaperBank));
    return writeBlob(
        PAPERS_FILE_PATH,
        MAGIC_PAPERS,
        SW_VERSION,
        &paperBank,
        sizeof(PaperBank),
        crc
    );
}

// =============================================================================
// SETTINGS PERSISTENZ (EEPROM)
// =============================================================================

void saveSettings() {
    // Thread-Sicherheit: Wir nehmen den Mutex, um einen konsistenten Snapshot zu erhalten
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        validateTimes();
        
        globalSet.version = SW_VERSION;
        globalSet.t_s  = time_soft; 
        globalSet.t_h  = time_hard;
        globalSet.t_bw = time_bw;
        globalSet.g_bw = grade_bw;
        globalSet.burn_g = burnGrade;
        globalSet.base_dark_lux  = baseDarkLux;
        globalSet.probe_dark_lux = probeDarkLux;

        globalSet.pwm_safe  = set_safe;
        globalSet.pwm_focus = set_focus;
        globalSet.pwm_lcd   = set_lcd;
        globalSet.pwm_max   = set_max;
        globalSet.useWirelessProbe = useWirelessProbe; // Persistent speichern

        globalSet.stepMode = (uint8_t)globalStepMode;
        
        globalSet.crc = calcCrc(globalSet);
        
        if (!writeSettingsFile(globalSet)) {
            Serial.println("[STORAGE] Fehler beim Speichern von /settings.bin");
        }

        // BETA-FIX: [Phase 2] BW-Zielzone zusaetzlich in NVS persistieren,
        // damit der Wert auch bei Struktur-/Versionswechseln robust bleibt.
        if (preferences.begin(NVS_NAMESPACE, false)) {
            preferences.putFloat(NVS_KEY_BW_TARGET_ZONE, globalSet.bwTargetZone);
            preferences.end();
        } else {
            Serial.println("[STORAGE] NVS konnte fuer bwTZone nicht geoeffnet werden.");
        }
        
        xSemaphoreGive(gTimerMutex);
        
        // Papiere separat sichern
        savePapers();
        
        settingsDirty = false;
        Serial.println("[STORAGE] System-Settings permanent gespeichert.");
    }
}

void defaultsSettings() {
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&globalSet, 0, sizeof(globalSet));
        globalSet.version = SW_VERSION;
        
        time_soft = 6.0; 
        time_hard = 10.0; 
        time_bw = 8.0; 
        grade_bw = 2.5;
        burnGrade = 2.5;
        baseDarkLux  = 0.0;
        probeDarkLux = 0.0;

        set_safe = 100; 
        set_focus = 255; 
        set_lcd = 100; 
        set_max = 255;
        useWirelessProbe = true; // Standard bei Reset

        // M02 FIX: Standard-Zeit explizit setzen (war 0.0 durch memset!)
        globalSet.std_time = 8.0;
        
        globalStepMode = STEP_THIRD;
        globalSet.stepMode = (uint8_t)globalStepMode;

        // Sound-Standard auf NORMAL setzen (anstatt auf OFF durch memset)
        globalSet.soundMode = SOUND_NORMAL;

        globalSet.burn_g = burnGrade;
        globalSet.base_dark_lux  = baseDarkLux;
        globalSet.probe_dark_lux = probeDarkLux;
        // BETA-FIX: [Phase 2] Standard-Zielzone fuer Auto-BW ist 8.0
        // (Lichter-Prioritaet im Sinne des Zonensystems).
        globalSet.bwTargetZone = 8.0f;

        globalSet.crc = calcCrc(globalSet);
        
        xSemaphoreGive(gTimerMutex);

        if (!writeSettingsFile(globalSet)) {
            Serial.println("[STORAGE] Fehler beim Schreiben der Default-Settings.");
        }
        
        defaultPapers();
        savePapers();
        Serial.println("[STORAGE] Werkseinstellungen wiederhergestellt.");
    }
}

void loadSettings() {
    SettingsObject loaded;
    uint16_t fileCrc = 0;
    bool fileOk = readBlob(
        SETTINGS_FILE_PATH,
        MAGIC_SETTINGS,
        SW_VERSION,
        &loaded,
        sizeof(SettingsObject),
        &fileCrc
    );
    
    // Validierung: Version oder CRC falsch?
    if (!fileOk || loaded.version != SW_VERSION || calcCrc(loaded) != loaded.crc || loaded.crc != fileCrc) {
        Serial.println("[STORAGE] CRC-Fehler oder alte Version. Lade Defaults...");
        saveErrorState(ERR_STORAGE_CORRUPTED);
        defaultsSettings();
    } else {
        globalSet = loaded;
        // BETA-FIX: [Phase 2] Defensiver Fallback fuer alte Binaerstaende,
        // die das neue Feld noch nicht sinnvoll initialisiert haben koennten.
        if (globalSet.bwTargetZone < 4.0f || globalSet.bwTargetZone > 9.0f) {
            globalSet.bwTargetZone = 8.0f;
        }
        // Daten aus der geladenen Struct in die globalen Arbeitsvariablen mappen
        time_soft = globalSet.t_s; 
        time_hard = globalSet.t_h; 
        time_bw   = globalSet.t_bw;
        grade_bw  = globalSet.g_bw;
        burnGrade = globalSet.burn_g;
        baseDarkLux  = globalSet.base_dark_lux;
        probeDarkLux = globalSet.probe_dark_lux;
        
        set_safe  = globalSet.pwm_safe; 
        set_focus = globalSet.pwm_focus;
        set_lcd   = globalSet.pwm_lcd;   
        set_max   = globalSet.pwm_max;
        useWirelessProbe = globalSet.useWirelessProbe; // Laden aus Speicher

        // CODE_REVIEW FIX: soundMode-Validierung beim Laden.
        // Ohne Prüfung könnte ein korrupter EEPROM-Wert > 2 zu undefiniertem
        // Verhalten im Sound-System führen (Array-Out-of-Bounds in sMap[] etc.)
        if (globalSet.soundMode > SOUND_NORMAL) {
            globalSet.soundMode = SOUND_NORMAL;
        }
        
        // CODE_REVIEW FIX (Audit Kritisch #5): stepMode Validierung war fehlerhaft.
        // Alte Bedingung: globalSet.stepMode <= STEP_FULL prüfte nur auf 0 (STEP_FULL),
        // da STEP_FULL=0 ist. Alle anderen gültigen Werte (HALF=1, THIRD=2, SIXTH=3)
        // wurden fälschlich als ungültig verworfen und auf STEP_THIRD zurückgesetzt.
        // Fix: Prüfung gegen STEP_SIXTH (=3), den höchsten gültigen Enum-Wert.
        // if (globalSet.stepMode <= STEP_FULL) {  // FEHLERHAFT: Nur 0 war gültig
        if (globalSet.stepMode <= STEP_SIXTH) {
            globalStepMode = (StepSize)globalSet.stepMode; 
        } else {
            globalStepMode = STEP_THIRD;
        }

        // BETA-FIX: [Phase 2] Zielzone aus NVS laden (Key: bwTZone).
        // Falls der Key fehlt, bleibt/gesetzt wird der Default 8.0.
        if (preferences.begin(NVS_NAMESPACE, false)) {
            float loadedBwZone = preferences.getFloat(NVS_KEY_BW_TARGET_ZONE, 8.0f);
            preferences.end();
            if (loadedBwZone >= 4.0f && loadedBwZone <= 9.0f) {
                globalSet.bwTargetZone = loadedBwZone;
            } else {
                globalSet.bwTargetZone = 8.0f;
            }
        } else {
            globalSet.bwTargetZone = 8.0f;
        }
    }
    
    loadPapers();         // Papierdatenbank aus EEPROM laden
    validateTimes();      // Plausibilitäts-Check
    resetTracking();      // UI-Tracking nullen
    updateGradeMath();    // PWM-Werte für LEDs berechnen
    
    // WICHTIG: v0.5 Dosis-Engine synchronisieren
    initializeDoseStateFromCurrentTimes();
}

// =============================================================================
// ERROR LOGGING
// =============================================================================
void saveErrorState(SystemError err) {
    int errVal = (int)err;
    uint16_t crc = calcBlobCrc(&errVal, sizeof(errVal));
    if (!writeBlob(ERROR_FILE_PATH, MAGIC_ERROR, SW_VERSION, &errVal, sizeof(errVal), crc)) {
        Serial.println("[STORAGE] Fehler beim Speichern des Error-Status.");
    }
}

SystemError loadErrorState() {
    int errVal = 0;
    uint16_t fileCrc = 0;
    bool ok = readBlob(
        ERROR_FILE_PATH,
        MAGIC_ERROR,
        SW_VERSION,
        &errVal,
        sizeof(errVal),
        &fileCrc
    );
    if (!ok || calcBlobCrc(&errVal, sizeof(errVal)) != fileCrc) {
        return ERR_NONE;
    }
    if (errVal < (int)ERR_NONE || errVal > (int)ERR_MATH_INVALID) {
        return ERR_NONE;
    }
    return (SystemError)errVal;
}

void clearErrorState() {
    saveErrorState(ERR_NONE);
}

// =============================================================================
// NVS PREFERENCES (Redundante Papier-Sicherung)
// =============================================================================

void initStorage() {
    if (!LittleFS.begin(true)) {
        Serial.println("[STORAGE] LittleFS Mount fehlgeschlagen!");
    }
}

void saveActivePaperProfile() {
    preferences.begin(NVS_NAMESPACE, false); // false = Read/Write

    PaperProfile &activePaper = getActivePaper();
    preferences.putBytes(NVS_KEY_PAPER, &activePaper, sizeof(activePaper));

    preferences.end();
}

void loadActivePaperProfile() {
    // ROOT CAUSE FIX:
    // `Preferences.begin(..., true)` erzeugt bei einem noch nicht existierenden Namespace
    // auf ESP32 einen lauten NOT_FOUND-Fehler im Serial-Log. Das ist auf Erststart oder
    // nach frisch geloeschtem NVS kein echter Fehlerzustand, sondern nur "noch keine Daten".
    // Wir oeffnen deshalb im Read/Write-Modus. Dadurch wird der Namespace bei Bedarf sauber
    // angelegt, ohne den Boot-Log mit einem irrefuehrenden Fehler zu fluten.
    if (!preferences.begin(NVS_NAMESPACE, false)) {
        Serial.println("[STORAGE] NVS Namespace konnte nicht geoeffnet werden. Nutze RAM-Bank.");
        return;
    }

    if (!preferences.isKey(NVS_KEY_PAPER)) {
        paperBank.activeIndex = 1;
        updateGradeMath();
        Serial.println("[STORAGE] NVS paper_active fehlt. Nutze Default: Ilford MGIV RC.");
        preferences.end();
        return;
    }

    PaperProfile &activePaper = getActivePaper();
    if (preferences.getBytesLength(NVS_KEY_PAPER) == sizeof(activePaper)) {
        preferences.getBytes(NVS_KEY_PAPER, &activePaper, sizeof(activePaper));
    } else {
        // ROOT CAUSE FIX: Zerstörungs-Schutz
        // Wenn das NVS-Profil durch ein Firmware-Update ungültig wurde,
        // belassen wir die Struktur unangetastet. initPapers() hat
        // bereits die sauberen Defaults geladen!
        Serial.println("[STORAGE] NVS PaperProfile Größen-Mismatch (Update?). Nutze RAM-Bank.");
    }

    preferences.end();
}

// =============================================================================
// DEFERRED EEPROM COMMIT (K06 / H02 Fix: Async EEPROM ohne Blockade)
// =============================================================================
void processDeferredEEPROM() {
    if (!settingsDirty) return;
    // Erst nach 2 Sekunden ohne weitere Änderungen wirklich schreiben
    if (millis() - lastSettingChange < 2000) return;
    saveSettings();
    Serial.println("[STORAGE] Deferred Settings-Sync ausgefuehrt.");
}

void savePapers() {
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (!writePapersFile()) {
            Serial.println("[STORAGE] Fehler beim Speichern von /papers.bin");
        }
        xSemaphoreGive(gTimerMutex);
    }
}

void loadPapers() {
    uint16_t fileCrc = 0;

    if (!paperBankPtr) {
        Serial.println("[STORAGE] paperBankPtr ist NULL. Lade Defaults...");
        saveErrorState(ERR_STORAGE_CORRUPTED);
        defaultPapers();
        savePapers();
        return;
    }

    bool ok = readBlob(
        PAPERS_FILE_PATH,
        MAGIC_PAPERS,
        SW_VERSION,
        &paperBank,
        sizeof(PaperBank),
        &fileCrc
    );

    if (!ok || calcBlobCrc(&paperBank, sizeof(PaperBank)) != fileCrc) {
        Serial.println("[STORAGE] /papers.bin fehlt/korrupt. Lade Defaults...");
        saveErrorState(ERR_STORAGE_CORRUPTED);
        defaultPapers();
        savePapers();
        return;
    }

    if (paperBank.version != SW_VERSION || paperBank.activeIndex >= 20) {
        Serial.println("[STORAGE] PaperBank ungueltig. Lade Defaults...");
        saveErrorState(ERR_STORAGE_CORRUPTED);
        defaultPapers();
        savePapers();
    }
}