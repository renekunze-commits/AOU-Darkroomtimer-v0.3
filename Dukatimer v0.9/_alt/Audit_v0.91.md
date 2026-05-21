# Dukatimer v0.91 — Deep Architecture & Math Audit

**Auditor:** Senior Embedded C++ Architect  
**Target:** ESP32-S3-N16R8, FreeRTOS, Dual-Core  
**Datum:** 2026-03-19  
**Quellstand:** v0.902 Beta (Header: `SW_VERSION_HEX 0x0902`)

---

## 1. Executive Summary

Das Refactoring von v0.3 auf v0.9x hat die Architektur fundamental verbessert: Dependency Injection, RAII MutexGuard, Dual-Mutex ExposureEngine, PSRAM-basierte PaperBank, deferred Flash-Writes. Die Grundstruktur ist solide.

**Aber: Das System ist nicht lauffähig.**

Die Analyse offenbart **6 kritische Integrationslücken**, die jede einzelne für sich verhindern, dass der Timer als Belichtungstimer funktioniert. Zusätzlich enthält die SystemContext-Persistenz einen Hash-Verifikationsfehler, der dafür sorgt, dass Benutzereinstellungen **niemals** aus dem Flash wiederhergestellt werden.

Die mathematische Substanz (FPU float-only, `powf`/`exp2f`, f-Suffixe) ist sauber durchgezogen. Die FreeRTOS Mutex-Architektur im SystemContext ist korrekt. Die eigentliche Gefahr liegt nicht in den implementierten Modulen, sondern in den **fehlenden Verbindungen** zwischen ihnen.

**Freigabe-Status: NICHT FREIGABEFÄHIG. 6 Blocker.**

---

## 2. Kritische Landminen (Prio 0 — Sofort beheben)

### L1: ExposureEngine wird nirgends instanziiert

**Datei:** [main.cpp](src/main.cpp)  
**Risiko:** Kein Belichtungsbetrieb möglich. Gesamtsystem ist ein teurer Sensor-Logger.

`main.cpp` erstellt `HardwareManager`, `PaperManager`, `StorageManager`, `AppManager` — aber **kein** `ExposureEngine`-Objekt. Kein `init()` wird aufgerufen, kein Core-1-Task gestartet, kein esp_timer angelegt.

```cpp
// main.cpp — das fehlt komplett:
// ExposureEngine engine(&context, &hw);
// engine.init();
```

Alle Apps (BWDoseApp, SGDoseApp, BWFStopApp) referenzieren `liveDose`, `liveTime` und `isExposureRunning` aus dem SystemContext, aber es gibt keinen Produzenten dieser Daten. Konsequenz: Alle Dosis-Werte sind permanent `0.0f`, alle Belichtungs-Flags permanent `false`.

### L2: InputManager wird nirgends instanziiert

**Datei:** [main.cpp](src/main.cpp)  
**Risiko:** Kein Benutzer-Input wird verarbeitet. Encoder, Taster — alles tot.

Es gibt kein `InputManager`-Objekt in `main.cpp` und keinen `process()`-Aufruf im Loop. Die Hardware (4 Encoder, 8 Taster) ist physisch angeschlossen aber softwareseitig vollständig getrennt.

```cpp
// main.cpp loop() — das fehlt:
// inputManager.process();
```

### L3: Phase-2-Apps nicht im AppManager registriert

**Datei:** [AppManager.h](include/AppManager.h), [AppManager.cpp](src/AppManager.cpp)  
**Risiko:** `MODE_BW_DOSE`, `MODE_SG_DOSE`, `MODE_BW_FSTOP`, `MODE_ZONE` sind nicht erreichbar.

`AppManager.h` inkludiert nur: `LiveViewApp`, `CalibrationApp`, `DensApp`, `BurnApp`, `PreflashApp`, `TestStripApp`. Die vier fotografisch zentralen Apps fehlen komplett:

- `BWDoseApp` (MODE_BW_DOSE)
- `SGDoseApp` (MODE_SG_DOSE)
- `BWFStopApp` (MODE_BW_FSTOP)
- `ZoneModeApp` (MODE_ZONE)

In `switchMode()` landen alle vier im `default:`-Zweig und bekommen **LiveView** zugewiesen:

```cpp
// AppManager.cpp, switchMode():
default:
    next = &_appLive;  // ← BW_DOSE, SG_DOSE, BW_FSTOP, ZONE landen hier
    break;
```

### L4: `HardwareStatus.liveTime` wird nirgends geschrieben

**Datei:** [SystemContext.h](include/SystemContext.h#L56), [BWFStopApp.cpp](src/BWFStopApp.cpp#L71)  
**Risiko:** BWFStopApp zeigt immer `remainingTime == calculatedTime` an, weil `liveTime` permanent `0.0f` ist.

Das Feld ist deklariert, der Kommentar beschreibt die Absicht korrekt:

```cpp
// SystemContext.h:56
float liveTime;  // "erlaubt Core-1 die exakte Belichtungszeit an Core-0 zu melden"
```

Aber `ExposureEngine` hat keinen `updateLiveTime()`-Aufruf, und `SystemContext` hat keine entsprechende Setter-Methode. Es existiert nur `updateLiveDose()` und `updateSensors()` — kein Pendant für die Zeitachse.

Die Initialisierung in [SystemContext.cpp](src/SystemContext.cpp#L66) ist zudem ein Typ-Mismatch:

```cpp
_hw = {0.0f, 0.0f, 25.0f, 22.0f, 0.0f, false};
//                                       ^^^^^
// 6 Werte für 7 Felder: 'false' (bool) wird implizit zu float 0.0f für liveTime,
// overheatActive wird value-initialisiert zu false. Funktioniert, ist aber ein Code-Smell.
```

### L5: SystemContext::deserialize — Hash-Verifikation ist mathematisch gebrochen

**Datei:** [SystemContext.cpp](src/SystemContext.cpp)  
**Risiko:** Benutzereinstellungen (`SystemPreferences`) werden **niemals** aus dem Flash restauriert. Jeder Boot ist ein Factory-Reset der Preferences.

**Serialisierung** (korrekt):
```cpp
blob.hash = 0;                                         // Hash-Feld nullen
uint16_t h = calculateHash16(&blob, sizeof(SettingsBlob)); // Hash über Blob MIT hash=0
blob.hash = h;                                         // Hash eintragen
```

**Deserialisierung** (gebrochen):
```cpp
memcpy(&tmp, buffer, sizeof(SettingsBlob));
// tmp.hash ist jetzt h (der gespeicherte Hash-Wert)
uint16_t calc = calculateHash16(&tmp, sizeof(SettingsBlob));
// calc ist FNV-1a über den gesamten Blob INKLUSIVE tmp.hash = h (≠ 0)
if (calc != tmp.hash)   // calc ≠ h, weil die Eingangsdaten unterschiedlich sind
    return false;        // ← IMMER false
```

FNV-1a ist nicht selbst-verifizierend. `H(data+0) ≠ H(data+H(data+0))`. Ergebnis: `deserialize()` gibt immer `false` zurück. Der `hash`-Parameter von `IDataProvider` (der externe Hash vom StorageManager) wird komplett ignoriert.

**Fix:** Entweder den externen `hash`-Parameter verwenden (wie PaperManager es korrekt tut), oder vor der Hash-Berechnung `tmp.hash = 0` setzen.

### L6: `onPredictiveShutoff()` wird von Core 1 aufgerufen — NeoPixelBus Race Condition

**Datei:** [ExposureEngine.cpp](src/ExposureEngine.cpp)  
**Risiko:** NeoPixelBus-Korruption, RMT-Treiber-Crash, undefiniertes Lichtverhalten.

`onPredictiveShutoff()` ruft `_hw->allLightsOff()` auf, welches `updateNeoPixels(0,0,0)` beinhaltet. Diese Funktion wird aus **zwei verschiedenen Kontexten** aufgerufen:

1. **esp_timer Callback** (Core 0, hohe Priorität) — via `timerCallbackWrapper`
2. **Core-1 clTaskLoop** — direkter Aufruf bei `remaining <= 0.0f`:

```cpp
// ExposureEngine.cpp, clTaskLoop:
if (remaining <= 0.0f)
{
    onPredictiveShutoff();  // ← Core 1 ruft NeoPixelBus auf!
    return;
}
```

NeoPixelBus (`Neo800KbpsMethod`) nutzt den RMT-Treiber, der nicht thread-safe ist. Gleichzeitige Zugriffe von Core 0 (z.B. abort() → allLightsOff()) und Core 1 (clTaskLoop → onPredictiveShutoff()) erzeugen eine Data Race auf dem RMT-Kanal.

Zusätzlich: Die State-Caching-Member im HardwareManager (`_currentR`, `_currentG`, `_currentB`) sind nicht `std::atomic` und werden ohne Lock aus beiden Cores geschrieben.

---

## 3. Mathematik & FPU-Leaks (Single-Precision Check)

### M1: Float-Literal-Disziplin — BESTANDEN

Alle Laufzeit-relevanten Konstanten tragen korrekt das `f`-Suffix. Stichproben:

| Stelle | Wert | Status |
|--------|------|--------|
| `Config.h` TIME_MIN_S | `1.0f` | ✓ |
| `Config.h` STOP_RESOLUTION | `12.0f` | ✓ |
| `Config.h` TEMP_MAX_ALU | `60.0f` | ✓ |
| `BWFStopApp` exp2f-Aufruf | `exp2f((float)_fStopTicks / STOP_RESOLUTION)` | ✓ |
| `ZoneModeApp` exp2f-Aufruf | `exp2f((float)((int)_selectedZone - (int)anchor))` | ✓ |
| `PaperManager` powf-Aufruf | `powf(10.0f, -density)` | ✓ |
| Default-Parameter | `suggestedDose = 0.0f`, `suggestedGrade = 2.5f` | ✓ |

Keine impliziten `double`-Promotions in den Rechenpfaden gefunden. `pow()`, `exp2()`, `log()`, `sqrt()` werden nirgends im v0.9-Code verwendet — nur `powf()`, `exp2f()`.

### M2: Floating-Point Absorption im Dose-Integrator — RISIKO NIEDRIG

```cpp
// ExposureEngine.cpp, updateDoseAndPredictiveTimer:
_currentDose += (lux * dtSeconds);
```

Naive Summation. Bei einer 10-Minuten-Belichtung mit 10ms Zykluszeit: 60.000 Additionen. Worst Case bei `_currentDose = 500.0f` und `lux * dt = 0.001f`: Die letzte signifikante Stelle von 0.001 liegt bei ~2^-10, die von 500 bei ~2^9. Differenz: ~19 Bit. IEEE 754 Single hat 23 Bit Mantisse → **4 Bit Restauflösung**. Für Dunkelkammer-Belichtung (Toleranz ~2-5%) akzeptabel.

**Empfehlung:** Für Belichtungen >5 Min oder wissenschaftliche Anwendung: Kahan-Summation einführen (ein `float _doseCompensation` Member). Kosten: 3 zusätzliche float-Operationen pro Zyklus.

### M3: Predictive-Shutoff Division-durch-Null — ABGESICHERT

```cpp
if (currentDose >= (targetDose * 0.95f) && lux > 0.01f && _shutoffTimer)
{
    uint64_t remUs = static_cast<uint64_t>((remaining / lux) * 1000000.0f);
```

Die Guard-Clause `lux > 0.01f` verhindert Division durch Null. Der Cast auf `uint64_t` ist safe, weil `remaining > 0` und `lux > 0.01f` zum Zeitpunkt der Division garantiert sind. `remaining` wurde zuvor als `targetDose - currentDose` berechnet und ist positiv (sonst wäre der sofortige `onPredictiveShutoff`-Pfad genommen worden).

### M4: Overflow-Schutz in armTimeModeTimer — KORREKT

```cpp
uint64_t durationUs = static_cast<uint64_t>(durationMs) * 1000ULL;
```

`durationMs` ist `uint32_t`, Maximum `4.294.967.295`. `* 1000ULL` → Max `4.294.967.295.000` → passt in `uint64_t`. `kMinTimerUs = 500` als untere Grenze verhindert 0-µs Timer. Korrekt.

### M5: ZoneModeApp Anchor-Berechnung — KORREKT, aber SEMANTISCH FRAGWÜRDIG

```cpp
uint8_t anchor = 2; // Anchor Zone II
float calc = _baseDose * exp2f((float)((int)_selectedZone - (int)anchor));
```

Der Cast `(int)_selectedZone - (int)anchor` ist notwendig, da `uint8_t - uint8_t` unsigned ist und bei Zone 0 oder 1 umbrechen würde. Die int-Casts sind korrekt. `exp2f` liefert Single-Precision.

**Semantik:** Zone II als Anchor bedeutet Zone V (Mittelgrau) liegt bei `baseDose * 8.0f`. Ob das fotografisch beabsichtigt ist, liegt außerhalb dieses technischen Audits.

---

## 4. FreeRTOS & Concurrency Flaws

### C1: SystemContext MutexGuard — KORREKT

Kein Setter ruft intern einen anderen Setter auf. Alle Getter/Setter sind flat: ein Lock, eine Operation, ein Unlock. Der FreeRTOS-Mutex ist non-recursive, und das Pattern ist sicher:

```cpp
bool SystemContext::setBWDose(float dose) {
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return false;
    _exp.targetDoseBw = dose;   // ← kein weiterer Mutex-Aufruf
    return true;
}
```

Geprüft: Alle 15 Setter/Getter folgen diesem Pattern. Kein Deadlock-Risiko.

### C2: ExposureEngine Dual-Mutex — KORREKT, mit einer Einschränkung

Das Zwei-Mutex-Design (_stateMutex, _doseMutex) reduziert Contention korrekt. `clTaskLoop` nimmt `_stateMutex` nur kurz zum Zustandscheck und `_doseMutex` für die Dosisintegration. Die Lock-Reihenfolge ist konsistent: immer State vor Dose (in `startTime`, `startDose`).

**Einschränkung:** In `startDose()` wird `_stateMutex` gehalten während `_doseMutex` genommen wird (nested lock):
```cpp
if (!lockState(kStateLockWait)) return;
// ... _state check ...
if (!lockDose(kDoseLockWait)) {
    unlockState();  // ← korrekt aufgeräumt
    return;
}
_currentDose = 0.0f;
unlockDose();
// ... weiter unter _stateMutex ...
unlockState();
```
Die Reihenfolge (State→Dose) ist in allen Pfaden konsistent. Kein Deadlock. Aber das Nested-Lock-Pattern sollte dokumentiert werden, damit zukünftige Entwickler die Reihenfolge nicht umkehren.

### C3: Core-Separation I2C — KORREKT

I2C-0 (Wire): Core 0, geschützt durch `_i2cMutex` + `_exposureLockActive`.  
I2C-1 (Wire1): Core 1, ohne Mutex (exklusiv für `clTaskLoop`).

Die ExposureEngine greift auf Wire1 via `_tsl.getEvent()` zu. Kein Core-0-Code berührt Wire1. Keine Cross-Core-Blockade auf dem I2C-Bus.

### C4: `_lastDoseSampleMs` Race Window — AKZEPTABEL

`_lastDoseSampleMs` wird in `startDose()` (Core 0, unter _stateMutex) geschrieben und in `clTaskLoop()` (Core 1, ohne Lock) gelesen/geschrieben. Zwischen dem Setzen in `startDose()` und dem ersten Lesen in `clTaskLoop()` gibt es ein kurzes Fenster, in dem der alte Wert gelesen werden könnte. Das Ergebnis wäre ein einmaliger, leicht verfälschter `dt`-Wert im ersten Zyklus. Bei 10ms Zykluszeit und der Guard `if (dt <= 0.0f) dt = 0.01f` ist das harmlos.

### C5: `_flags.appState` als Union — DESIGN-WARNUNG

`AppSharedState` ist ein `union`. Der SystemContext hat genau eine Instanz. Jede App überschreibt ihren Zweig, was die anderen Zweige implizit als Garbage hinterlässt. Das funktioniert nur, solange:
1. Genau eine App aktiv ist (durch AppManager garantiert)
2. Display-Code den richtigen Zweig basierend auf `currentMode` liest

Es gibt keinen Discriminator/Tag im union. Eine `enum ActiveAppType` wäre robuster, ist aber nicht kritisch.

### C6: StorageManager::process() blockiert Main-Loop

`LittleFS.open()` + `file.write()` + `file.close()` in `process()` sind blockierende Flash-Operationen (~10-100ms je nach Größe). Während dieser Zeit werden keine `appManager.onUpdate()` und keine `inputManager.process()` (sofern integriert) ausgeführt.

Aktuell ist das unkritisch (2.5s Defer-Delay reduziert die Häufigkeit), aber bei Zufügung von weiteren Core-0-Aufgaben (Display-Updates, BLE) kann es zu spürbaren UI-Aussetzern kommen.

---

## 5. Ressourcen- & Integritäts-Risiken

### R1: Task Stack für ExposureCL — KNAPP

```cpp
xTaskCreatePinnedToCore(..., "ExposureCL", 6144, ...);
```

6144 Bytes für einen Task der:
- `sensors_event_t` (≥32 Bytes) auf dem Stack hat
- Wire1 I2C-Transaktionen ausführt (~128-256 Bytes Puffer)
- Floating-Point-Berechnungen mit lokalen Variablen
- FreeRTOS Overhead (~300 Bytes)
- printf/log im Fehlerfall

**Empfehlung:** 8192 Bytes. Stack-Overflow auf Core 1 ist ein Hard Fault ohne Diagnose.

### R2: `_migrateBankAndUpdate` Overload ohne Header-Deklaration

**Datei:** [PaperManager.cpp](src/PaperManager.cpp#L111)

```cpp
bool PaperManager::_migrateBankAndUpdate(const PaperBank *oldBank) { ... }
```

Diese Member-Funktion ist in der `.cpp`-Datei definiert, aber **nicht** im Header deklariert. Der Header deklariert nur:
```cpp
bool _migrateBankAndUpdate(const uint8_t *legacyBuffer, size_t length, uint16_t legacyVersion);
```

In Standard-C++ ist das ein Kompilierungsfehler. Der Code ist entweder toter Legacy-Code oder verursacht Build-Fehler. Falls der Build durchläuft, ist es ungenutzter Code, der entfernt werden sollte.

### R3: SystemPreferences nicht `__attribute__((packed))`

**Datei:** [SystemContext.h](include/SystemContext.h#L19)

`SystemPreferences` wird in `SettingsBlob` eingebettet, welches `packed` ist. Das äußere `packed` wirkt transitiv auf die Member — aber nur auf GCC/Clang. Sauberer wäre es, `SystemPreferences` selbst als `packed` zu deklarieren und mit einem `static_assert(sizeof(SystemPreferences) == erwartete_Größe)` abzusichern.

Das gleiche gilt für `ExposureParams` und `WorkflowFlags` — diese werden zwar nicht persistiert, aber als Snapshot kopiert. Padding-Inkonsistenz könnte bei zukünftiger Serialisierung (z.B. BLE-Export) problematisch werden.

### R4: Kein `static_assert` auf persistierten Struct-Größen

`PaperProfile`, `PaperBank`, `SettingsBlob` — alle haben `packed`, aber keine Size-Assertion. Bei einer unbemerkten Struct-Änderung verschiebt sich das gesamte Speicherlayout, und die Migration greift ins Leere.

```cpp
// Empfohlen:
static_assert(sizeof(PaperProfile) == 272, "PaperProfile size changed — update migration!");
static_assert(sizeof(PaperBank) == 5445, "PaperBank size changed — update migration!");
static_assert(sizeof(SettingsBlob) == sizeof(SystemPreferences) + 2, "SettingsBlob layout changed!");
```

### R5: Error Propagation — Silent Failures vorhanden

| Stelle | Fehler | Reaktion |
|--------|--------|----------|
| `ExposureEngine::startTime()` | Mutex-Timeout | Silent Return, kein Fehlercode |
| `SystemContext::updateLiveDose()` | Mutex-Timeout (0-tick) | Return false, wird mit `(void)` ignoriert |
| `PaperManager::_isWriteLocked()` | Context-Mutex-Timeout | Fail-Closed (gut), aber kein Logging |
| `DisplayManager::update()` | Alle 3 Snapshots fehlschlagen | Silent Return, Display zeigt Stale-Daten |

Die `[[nodiscard]]`-Attribute auf den Settern sind korrekt. Aber die `(void)`-Casts in ExposureEngine unterdrücken die Warnungen bewusst. Das ist akzeptabel für non-blocking Core-1-Operationen, sollte aber mit einem kurzen Kommentar begründet werden (was teilweise schon geschieht).

### R6: Memory Leak bei `serialize()` Fehler in StorageManager

```cpp
// StorageManager.cpp, process():
provider->clearDirty();
// ...
if (!provider->serialize(&buffer, &length, &hash)) {
    provider->markDirty();
    continue;   // ← buffer wurde von serialize() möglicherweise teilweise allokiert
}
```

Wenn `serialize()` intern `malloc()` aufruft und dann in einem späteren Schritt fehlschlägt (nach der Allokation), gibt es keinen Cleanup. In der aktuellen Implementierung ist `serialize()` so geschrieben, dass `buffer` nur bei Erfolg gesetzt wird, also kein akutes Leak. Aber die Schnittstelle gibt keine Garantie: `IDataProvider::serialize` hat keinen Vertrag, der besagt, dass `*buffer` bei Fehler `nullptr` bleibt.

### R7: Apps schreiben AppSharedState ohne Discriminator

Jede App überschreibt `AppSharedState s{}` (zero-initialized) und setzt nur ihren Union-Zweig. Wenn zwei `setAppState()`-Aufrufe schnell hintereinander kommen (z.B. während App-Switch), kann ein Display-Update einen inkonsistenten Zustand lesen. Ohne Discriminator-Feld (`enum ActiveApp`) ist die Zuordnung fragil.

---

## 6. Fazit & Freigabe-Status

### Blocker (müssen vor jedem Test behoben werden)

| ID | Schwere | Problem | Aufwand |
|----|---------|---------|---------|
| L1 | 🔴 KRITISCH | ExposureEngine nicht instantiiert | Klein (10 Zeilen in main.cpp) |
| L2 | 🔴 KRITISCH | InputManager nicht instantiiert | Klein (5 Zeilen in main.cpp) |
| L3 | 🔴 KRITISCH | Phase-2-Apps nicht in AppManager | Mittel (Header + switchMode erweitern) |
| L4 | 🔴 KRITISCH | liveTime ohne Setter/Producer | Klein (SystemContext + ExposureEngine erweitern) |
| L5 | 🔴 KRITISCH | SystemContext Hash-Verifikation gebrochen | Klein (1 Zeile: `tmp.hash = 0;` vor Hash-Berechnung) |
| L6 | 🔴 KRITISCH | NeoPixelBus Race von Core 1 | Mittel (Shutoff-Signaling statt direktem HW-Aufruf) |

### Empfehlungen (sollten vor Feldtest behoben werden)

| ID | Schwere | Problem | Empfehlung |
|----|---------|---------|------------|
| R1 | 🟡 HOCH | Stack 6144 für ExposureCL | Auf 8192 erhöhen |
| R2 | 🟡 MITTEL | Toter _migrateBankAndUpdate Overload | Entfernen |
| R3 | 🟡 MITTEL | SystemPreferences nicht packed | packed + static_assert |
| R4 | 🟡 MITTEL | Keine static_assert auf Struct-Größen | Hinzufügen |
| M2 | 🟢 NIEDRIG | Floating-Point Absorption | Kahan-Summation für >5 Min |
| C6 | 🟢 NIEDRIG | Flash-Write blockiert Main-Loop | Mittelfristig in separaten Task |

### Was GUT ist

- **MutexGuard-Pattern:** Sauber, flach, kein Deadlock-Risiko
- **Dual-Mutex ExposureEngine:** Intelligente Contention-Reduktion
- **I2C Bus-Trennung:** Core-0/Core-1 Isolation korrekt
- **Float-Disziplin:** Durchgehend f-Suffix, powf/exp2f
- **StorageManager Defer-Logik:** clearDirty vor Write + markDirty bei Fehler
- **PaperManager Hash-Verifikation:** Korrekt (im Gegensatz zu SystemContext)
- **IDataProvider-Abstraktion:** Saubere Trennung von Serialisierung und Storage
- **InputManager PCNT-Design:** Hardware-Dekodierung statt ISR, robustes Debouncing

### Nächster Schritt

L1–L6 beheben, dann einen End-to-End Integrationstest auf realer Hardware durchführen. Die Module sind individuell solide — das Problem ist die fehlende Verdrahtung.
