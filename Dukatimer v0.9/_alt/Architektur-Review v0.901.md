Dukatimer v0.901 — Absolut schonungsloses Code-Audit
Datum: 2026-03-18

Kurz: Dieses Dokument fasst das tiefe Audit des Phase-1-Fundaments zusammen. Es listet Executive Summary, kritische Bugs (Prio 0), architektonische Design-Flaws (Prio 1), Zukunfts-Risiken und konkrete Optimierungen sowie eine priorisierte ToDo-Liste.

**1. Executive Summary**

Das Fundament von v0.901 ist in seiner Schichtentrennung und Ownership deutlich verbessert: Globals sind eliminiert, `SystemContext` ist die zentrale Single-Source-of-Truth, `HardwareManager` kapselt Aktoren und I2C-Infrastruktur, `PaperManager` hält Papierdaten im PSRAM und `StorageManager` sorgt für Deferred-Writes und Hash-Checks. Das ist für Phase 2 grundsätzlich tragfähig.

Trotzdem existieren mehrere kritische Probleme, die vorweiterer Entwicklung behoben werden müssen: drei P0-Fehler (stilllaufende Datenkorruption, sicherheitskritische silent-fail Setter, Blocking-Update auf Core 1), mehrere P1-Designfehler (Dirty-Timestamp-Coupling, lost-dirty-race, unsichere Migration) und mehrere Optimierungen (RAII, float statt double, atomic statt volatile). Phase 2 (AppManager, IAppMode, ExposureEngine) ist machbar — aber nur nachdem P0/P1 behoben sind.

**2. Kritische Bugs (Prio 0)**

- BUG-1 — NVS-Backup überschreibt valide Flash-Daten (StorageManager)
  - Problem: `_loadActivePaperFromNVS()` wird immer ausgeführt, auch nach erfolgreichem LittleFS-Load. Veraltetetes NVS kann valide PaperBank aus Flash überschreiben → stiller Datenverlust.
  - Schwere: Sehr hoch (stiller Datenverlust von Nutzerkalibrierungen).
  - Kurzfix: NVS-Restore nur als Fallback, bei erfolgreichem Flash-Load NVS mit `_saveActivePaperToNVS()` aktualisieren.

- BUG-2 — Sicherheitskritische Setter in `SystemContext` sind "Fail-Open" (silent on mutex timeout)
  - Problem: Setter wie `setExposureState()` returnen `void` und ignorieren Mutex-Timeouts stillschweigend. Fehlschläge führen zu inkonsistenten Sicherheitsflags (z.B. Belichtungs-Guard nicht gesetzt) → potentiell gefährliche Flash-Schreibvorgänge während Belichtung.
  - Schwere: Sehr hoch (Sicherheits-/Integritätsproblem).
  - Kurzfix: Setter müssen `bool` zurückgeben; Caller muss Retry/Abort/Log implementieren. Timeout-Länge für kritische Setter anheben.

- BUG-3 — `updateSensors()` blockiert Core 1 (Timeout 30ms)
  - Problem: `updateSensors()` nutzt `UI_WAIT` (30 ms). Core 1 (Realtime) darf nicht auf Core-0-Mutex warten — das verliert Samples und verfälscht Dosismessung.
  - Schwere: Hoch (Realtime-Integrität).
  - Kurzfix: `updateSensors()` muss non-blocking sein (xSemaphoreTake(..., 0)).

**3. Architektonische Design-Flaws (Prio 1)**

- FLAW-1 — `StorageManager::_lastChangeTime` koppelt Settings und Papers
  - Problem: Ein gemeinsamer Timestamp führt zu sofortigen Writes für PaperBank oder zu falscher Defer-Logik.
  - Fix: Separate Timestamps `_settingsChangedAt` und `_papersChangedAt`; PaperManager/StorageManager müssen Paper-Dirty-Events timestampen.

- FLAW-2 — Dirty-Flag Race beim Write: Lost-Update
  - Problem: `process()` liest `isDirty()`, schreibt die Bank, und setzt dann `clearDirty()`. Änderungen, die während des Writes auftreten, gehen verloren.
  - Fix: `clearDirty()` VOR Write; bei Schreibfehlern erneut markieren.

- FLAW-3 — Migration via Direct-Struct-Copy ist fragil
  - Problem: Direkte Assignment-/memcpy-Strategie zerbricht, wenn Feldgrößen/Order sich ändern. Keine Runtime-Warnung außer Logs.
  - Fix: `static_assert` für Struct-Größen, Runtime-Checks für payloadLen vs expected, und ein Field-by-Field-Mapping-Branch wenn Layout abweicht.

- FLAW-4 — `_exposureLockActive` ist `volatile bool` statt `std::atomic<bool>`
  - Problem: `volatile` garantiert keine C++-Speicherkohärenz/Cross-Core-Sichtbarkeit. Use `std::atomic<bool>`.

- FLAW-5 — Init-Fehler (PSRAM / LittleFS) werden stumm toleriert
  - Problem: `papers.init()` oder `storage.init()` können fehlschlagen; `main()` ignoriert Rückgabewerte → halbtoter Zustand ohne user-visible Fehler.
  - Fix: Bei fatalen Init-Fehlern sauberes Fallback-Verhalten oder sicherer Halt mit Fehleranzeige/beep.

**4. Zukunftssicherheit & Optimierungen**

- RAII MutexGuard: Ersetze manuelle `xSemaphoreTake/xSemaphoreGive`-Paare durch eine kleinen `MutexGuard`-Klasse (Konstruktor = Take, Destruktor = Give). Verhindert Mutex-Leaks bei Early-Return/Fehlerpfaden.

- `double` → `float` prüfen: ESP32 Single-Precision FPU nutzt `float` effizienter; `PaperProfile` enthält viele LUT-Werte: Wechsel zu `float` reduziert Speicher und CPU-Last signifikant.

- Persisted Structs: `__attribute__((packed))` ist bereits teilweise genutzt; ergänzen und `static_assert(sizeof(...))` ergänzen, oder besser: explizite Serialisierung für Forward-Kompatibilität.

- NeoPixel und Hardware-Zugriffe: Entweder strikt Core-0-only halten (mit `configASSERT(xPortGetCoreID()==0)`) oder Thread-Schutz hinzufügen.

- I2C-API: Verberge `takeI2C()`/`giveI2C()` hinter `SensorManager` statt in public `HardwareManager` — reduziert Missbrauch riskanter Lock-Aktionen.

**5. Priorisierte ToDo-Liste (Kurz)**

Prio 0 — Blocking Bugs (sofort):
- P0-1: NVS-Restore nur als Fallback; bei Erfolg Flash→NVS syncen. (StorageManager)
- P0-2: Sicherheitskritische Setter `bool` return + Retry/Log (SystemContext + Callers). 
- P0-3: `updateSensors()` non-blocking (SystemContext).

Prio 1 — Vor Phase 2:
- P1-1: Separate Dirty-Timestamps + clear-before-write pattern (StorageManager).
- P1-2: `std::atomic<bool>` für `_exposureLockActive` (HardwareManager).
- P1-3: Add `static_assert` für persisted struct sizes + safe field-by-field migration branch.
- P1-4: Init failure handling in `main()` (halt or visible error).

Prio 2 — Empfehlenswerte Verbesserungen:
- RAII `MutexGuard`-Implementierung und Replace (SystemContext, PaperManager, StorageManager).
- Double→Float Evaluation & Migration Plan (PaperProfile).
- Introduce `IAppMode` interface and `AppManager` skeleton (Phase 2 prerequisite).

**6. Konkrete Quick-Fixes (Code-Hinweise)**

- `StorageManager::init()`:
  - Wenn `papersLoaded == true`: rufe `_saveActivePaperToNVS()` statt `_loadActivePaperFromNVS()` auf.

- `SystemContext`:
  - Ändere sicherheitskritische Setter-Signaturen zu `bool` und erhöhe Timeout für diese Calls.
  - `updateSensors()` mit `xSemaphoreTake(_mutex, 0)` implementieren.

- `StorageManager::process()`:
  - Stelle sicher, dass Paper-Dirty-Path ein eigenes `_papersChangedAt` hat und `clearDirty()` vor dem Flash-Write ausgeführt wird.

---

Frage: Soll ich die P0-Fixes jetzt automatisch implementieren (Patch-Patches für `StorageManager.cpp`, `SystemContext.cpp`, `main.cpp` und kleine API-Änderungen), oder möchtest Du zuerst priorisieren/Review machen? Ich kann die P0-Änderungen sofort committen.
