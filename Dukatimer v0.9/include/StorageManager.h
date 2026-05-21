/* =============================================================================
 * StorageManager.h - DUKATIMER BETA (v0.912)
 * * Zentrale Instanz für die LittleFS-Persistenz.
 * * ARCHITEKTUR-MERKMALE:
 * - Deferred Writes: Schont den Flash durch verzögertes Schreiben nach UI-Input.
 * - Integrity: Nutzt IDataProvider Schnittstelle zur Serialisierung/Hash-Prüfung.
 * - Safety: Write-Interlock sperrt Flash-Zugriffe während Belichtungen und Messungen,
 * um Jitter und Latenzen auf den Sensor-Bussen zu vermeiden.
 * ========================================================================== */

#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include "IDataProvider.h"

// Forward Declaration zur Vermeidung zirkulärer Header-Abhängigkeiten.
class SystemContext; 

class StorageManager
{
public:
    /**
     * Konstruktor mit Dependency Injection.
     * @param ctx Pointer auf den SystemContext (benötigt für den Write-Interlock Check).
     */
    StorageManager(SystemContext *ctx);

    /**
     * Initialisiert das LittleFS Dateisystem und lädt Daten von allen 
     * registrierten Providern.
     * @return true bei Erfolg.
     */
    bool init();

    /**
     * Zyklische Verarbeitung der Dirty-Flags (Aufruf in loop()).
     * Implementiert die Deferred-Write Logik und den Exposure-Interlock.
     */
    void process();

    /**
     * Registriert ein Modul (z.B. SystemContext, PaperManager) für die Persistenz.
     */
    void registerProvider(IDataProvider *provider);

private:
    static constexpr size_t MAX_PROVIDERS = 4;
    
    /**
     * Wartezeit in ms nach dem letzten markDirty(), bevor tatsächlich 
     * auf den Flash geschrieben wird (Write-Debouncing).
     */
    static constexpr unsigned long DEFER_DELAY_MS = 2500;

    SystemContext *_ctx;
    IDataProvider *_providers[MAX_PROVIDERS];
    
    // Zustands-Tracking für Deferred Writes
    bool          _dirtyLatched[MAX_PROVIDERS];
    unsigned long _dirtySince[MAX_PROVIDERS];
    size_t        _providerCount;
};