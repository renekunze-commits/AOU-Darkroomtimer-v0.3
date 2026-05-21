# Dukatimer v0.916 — Architektur-Audit

**Datum:** 2026-03-21  
**Auditor:** Claude Opus 4.6 (Senior Embedded C++ / FreeRTOS Systemarchitekt)  
**Scope:** Gesamte Kernel-Schicht (SystemContext, AppManager, InputManager, ExposureEngine, StorageManager, PaperManager, HardwareManager, SensorManager, DisplayManager, main.cpp)  
**Methode:** Statische Code-Analyse aller 24 Header + 21 Sourcen auf Thread-Sicherheit, Speicher-Integrität, Echtzeit-Determinismus und Hardware-Entkopplung.

---

## Executive Summary

**Bewertung: BEDINGT industriell belastbar.**

Das Fundament ist architektonisch solide konzipiert — die Dual-Core-Trennung, das RAII-Mutex-Pattern, die Hash-Payload-Separation und die Producer-Consumer-Entkopplung sind *korrekt designt*. Die Codebasis zeigt eine seltene Disziplin für ein Embedded-Projekt dieser Komplexitätsklasse.

**Jedoch existieren 4 echte Zeitbomben**, die unter spezifischen Lastszenarien zu Race Conditions, Datenverlust oder Priority Inversion führen können. Keine davon wird im Normalbetrieb sofort zuschlagen — aber unter Stress (volle Queue + Mutex-Contention + Flash-Write gleichzeitig) sind sie deterministisch reproduzierbar.

---

## 1. Kritische Zeitbomben

### ZEITBOMBE T1: `PaperManager::_isDirty` ist nicht atomar (Race Condition)

**Datei:** [PaperManager.h](include/PaperManager.h#L88)  
**Schweregrad:** KRITISCH  
**Technische Begründung:**

`_isDirty` ist als `bool _isDirty` deklariert — ein einfacher, nicht-atomarer Datentyp. Im Gegensatz dazu verwendet `SystemContext` korrekt `std::atomic<bool> _isDirty{false}`.

**Die Race Condition:**
- Core 0 `loop()` ruft `StorageManager::process()` auf, welches `provider->isDirty()` liest.
- Gleichzeitig kann eine App (ebenfalls Core 0, aber in einem anderen Execution-Kontext nach `vTaskDelay`) `PaperManager::updateActiveProfile()` aufrufen, welches `_isDirty = true` setzt — *innerhalb* des Mutex.
- Problem: `isDirty()` und `clearDirty()` werden in `StorageManager::process()` *ohne* den `_dbMutex` aufgerufen. Der `isDirty()`/`clearDirty()`-Pfad im StorageManager hält **niemals** den PaperManager-Mutex.

**Konkretes Szenario:**
1. StorageManager liest `isDirty() == true` → beginnt serialization.
2. StorageManager ruft `clearDirty()` auf → `_isDirty = false`.
3. *Zwischen* `clearDirty()` und dem tatsächlichen `file.write()` setzt eine App `_isDirty = true`.
4. Der Write vollendet sich.
5. StorageManager setzt `_dirtyLatched[i] = false`.
6. **Ergebnis:** Die neue Änderung ist verloren — das Dirty-Flag wurde gelöscht, aber die Daten im Flash sind veraltet.

**Obwohl dieses Szenario auch bei `std::atomic<bool>` möglich ist**, verschärft der fehlende atomare Zugriff das Problem: Ohne `std::atomic` kann der Compiler Lese-/Schreib-Operationen auf `_isDirty` *reordnen* oder *cachen*, was auf einem Dual-Core-System zu Sichtbarkeitsproblemen zwischen Tasks führt.

**Fix:**
```cpp
// PaperManager.h, Zeile 88
std::atomic<bool> _isDirty{false};
```

Zusätzlich: Der StorageManager hat ein *logisches* Dirty-Loss-Window zwischen `clearDirty()` und dem tatsächlichen Write. Dies betrifft *beide* Provider (SystemContext und PaperManager). Ein robusterer Ansatz wäre *Clear-After-Write* oder ein Sequence-Counter statt eines simplen Flags.

---

### ZEITBOMBE T2: ExposureEngine Manual Lock/Unlock ohne RAII (Deadlock-Potenzial)

**Datei:** [ExposureEngine.cpp](src/ExposureEngine.cpp)  
**Schweregrad:** HOCH  
**Technische Begründung:**

Die gesamte `ExposureEngine` verwendet manuelle `lockState()`/`unlockState()` und `lockDose()`/`unlockDose()` Paare — im direkten Widerspruch zur dokumentierten Architektur-Konvention ("FreeRTOS-Mutexe sind non-recursive. Daher wird konsequent RAII via MutexGuard verwendet.").

**Problemstellen:**

1. **`tick()` (Zeile ~123-161):** Lock wird genommen, dann folgt ein komplexer `switch`-Block mit mehreren `break`-Pfaden. Ein vergessenes `unlockState()` in einem zukünftigen Code-Pfad = Deadlock.

2. **`startTime()` / `startDose()`:** Lock wird genommen, dann wird *innerhalb* des State-Locks ein zweiter Lock (`lockDose`) genommen. Dies ist eine klassische **Nested-Lock-Sequenz**. Wenn Core 1 (`clTaskLoop`) den Dose-Lock hält und versucht den State-Lock zu nehmen (Zeile ~319), während Core 0 in `startTime` den State-Lock hält und den Dose-Lock versucht → **Deadlock.**

   Tatsächliche Lock-Ordnung:
   - `startTime()`: _stateMutex → _doseMutex
   - `clTaskLoop()`: _stateMutex → (unlock) → _doseMutex
   - `updateDoseAndPredictiveTimer()`: _doseMutex (ohne _stateMutex)
   
   Die aktuelle Reihenfolge in `clTaskLoop` ist: State-Lock → check → State-Unlock → Dose-Lock. Das ist *korrekt* und vermeidet den Deadlock — **aber nur weil die Reihenfolge zufällig stimmt.** Ohne RAII-Guards ist dies fragil.

3. **`abort()` (Zeile ~220):** Ruft `disarmPredictiveTimer()` innerhalb des State-Locks auf. `disarmPredictiveTimer()` stoppt einen `esp_timer`. Falls der Timer-Callback (`onPredictiveShutoff`) gerade ausgeführt wird und selbst versucht auf Atomics zuzugreifen — kein Mutex-Problem, aber die fehlende RAII-Absicherung macht den Code anfällig für regressions.

**Fix:** Refactor auf lokale `MutexGuard`-Instanzen (wie in SystemContext/PaperManager), um garantierte Freigabe zu erreichen.

---

### ZEITBOMBE T3: AppSharedState Union — Tag-Update nicht atomar mit Daten

**Datei:** [SystemContext.cpp](src/SystemContext.cpp#L163), [SystemContext.h](include/SystemContext.h#L50)  
**Schweregrad:** MITTEL-HOCH  
**Technische Begründung:**

`setAppState()` schreibt die komplette `AppSharedState`-Struktur (inklusive Union + Tag) unter dem SystemContext-Mutex:

```cpp
bool SystemContext::setAppState(const AppSharedState &state)
{
    MutexGuard lock(_mutex, CRITICAL_WAIT);
    if (!lock.isAcquired()) return false;
    _flags.appState = state;
    return true;
}
```

Der DisplayManager liest die Daten über `getFlags()`:
```cpp
bool SystemContext::getFlags(WorkflowFlags &out) 
{ 
    MutexGuard lock(_mutex, UI_WAIT);  // 30ms Timeout!
    if (!lock.isAcquired()) return false; 
    out = _flags; 
    return true; 
}
```

**Das Problem ist das `activeStateMode`-Tag:**

Die Apps (z.B. BWDoseApp) setzen `setAppState()` mit Daten für ihren Union-Zweig, aber **setzen NICHT das `activeStateMode`-Tag.** BWDoseApp, Zeile ~110:

```cpp
AppSharedState s{};           // activeStateMode = MODE_BW (Default!)
s.bwDose.targetDose = target; // Schreibt in den bwDose-Zweig
_ctx->setAppState(s);         // Tag ist MODE_BW, nicht MODE_BW_DOSE!
```

Der DisplayManager nutzt `flags.currentMode` (vom `WorkflowFlags`-Struct) anstatt `flags.appState.activeStateMode` zum Routing — das **funktioniert**, aber das Tag-Feld im AppSharedState ist faktisch eine unbelegte Leiche. Wenn jemals Code geschrieben wird, der dem Tag vertraut, wird er fehlerhafte Union-Daten lesen.

**Zweites Risiko:** Wenn `getFlags()` mit `UI_WAIT` (30ms) den Mutex nicht bekommt, gibt es `false` zurück und der DisplayManager *überspringt das gesamte Update.* Das ist korrekt und sicher. Aber: `updateMeasuringProgress()` nutzt Timeout 0 — wenn das fehlschlägt, wird der Fortschritt **nicht aktualisiert**, was zu einer eingefrorenen Progress-Bar führt. Das ist kein Crash, aber ein UX-Defekt unter Stress.

**Fix:** Jede App MUSS das `activeStateMode`-Tag korrekt setzen. Ein Helper wäre ideal:
```cpp
AppSharedState s{};
s.activeStateMode = MODE_BW_DOSE;  // Explizit!
```

---

### ZEITBOMBE T4: StorageManager Dirty-Flag Clear-Before-Write Window

**Datei:** [StorageManager.cpp](src/StorageManager.cpp#L84)  
**Schweregrad:** MITTEL  
**Technische Begründung:**

```cpp
provider->clearDirty();  // Zeile ~84: Flag wird SOFORT gelöscht

uint8_t *buffer = nullptr;
size_t length = 0;
uint16_t hash = 0;

if (!provider->serialize(&buffer, &length, &hash))
{
    provider->markDirty();  // Rollback bei Serialisierungsfehler ✓
    continue;
}

// ... File-Write passiert HIER ...

if (!writeOk)
{
    provider->markDirty();  // Rollback bei Schreibfehler ✓
    continue;
}
```

**Das Window:** Zwischen `clearDirty()` und dem erfolgreichen `file.write()` existiert ein Zeitfenster. Wenn in diesem Fenster — was bei deferred writes von 2.5s und einem Flash-Write von ~10-50ms zugegebenermaßen selten ist — eine neue Änderung eintritt:

1. `clearDirty()` → `_isDirty = false`
2. App setzt `_isDirty = true` (neue Änderung)
3. `serialize()` erfasst den *alten* Snapshot (der unter Mutex kopiert wird)
4. Write erfolgt mit altem Snapshot → OK
5. `_dirtyLatched` wird `false` gesetzt
6. **Die neue Änderung wird nicht am nächsten DEFER_DELAY-Zyklus erkannt**, weil `_dirtyLatched` + `isDirty()` jetzt beide `false/true` sind und der Timer erst bei *erneutem* dirty-Transition startet.

Tatsächlich wird die neue Änderung beim nächsten `process()`-Durchlauf durch `isDirty() == true` und `!_dirtyLatched` erkannt — der 2.5s-Timer beginnt von vorn. Das ist **korrekt implementiert**. Aber der Flash-Inhalt ist für bis zu 2.5s *stale*. Bei einem Stromausfall in diesem Fenster → Datenverlust der letzten Änderung.

**Bewertung:** Akzeptables Risiko für ein batteriebetriebenes Gerät. Kein Fix nötig, aber die Dokumentation sollte das Fenster benennen.

---

## 2. Architektonische Bewertung

### 2.1 Producer-Consumer-Entkopplung (InputManager → AppManager)

| Aspekt | Bewertung | Details |
|--------|-----------|---------|
| Queue-Dimensionierung (80 Slots) | ✅ Korrekt | 4 Encoder × 16 max Steps + 8 Buttons = max. 72 Events/Cycle. 80 Slots reichen. |
| PCNT-Sync bei voller Queue | ✅ Gut gelöst | `_lastCount[i]` wird nur um `successfulSteps` inkrementiert. Verlorene Events bleiben im Hardware-Zähler. |
| EV_ABORT via `xQueueSendToFront` | ✅ Korrekt | Not-Aus wird bei Event 0 priorisiert. `onButtonPressed(BTN_IDX_ENC_MODE_SW)` → `dispatchEvent(EV_ABORT)` → `toFront=true`. |
| Verlustfreiheit bei voller Queue | ⚠️ Bedingt | Wenn die Queue voll ist, schlägt auch `xQueueSendToFront` für ABORT fehl (Timeout 0). In der Praxis irrelevant, da 80 Slots nie volle Queue + gleichzeitigen Abort produzieren. |

**Gesamtbewertung:** Robust. Die Encoder-Desync-Korrektur (successfulSteps) ist eine elegante Lösung.

### 2.2 Speicherhärtung

| Aspekt | Bewertung | Details |
|--------|-----------|---------|
| `SystemPreferences` packed + static_assert | ✅ Korrekt | 24 Bytes, kein verstecktes Padding. |
| `PaperProfile` packed + static_assert | ✅ Korrekt | 157 Bytes verifiziert. |
| `PaperBank` packed + static_assert | ✅ Korrekt | 3145 Bytes verifiziert. |
| Hash-Payload-Trennung | ✅ Korrekt | `offsetof(SettingsBlob, hash)` / `offsetof(PaperBank, hash)` → Hash wird nie in die Payload serialisiert. |
| OOB-Schutz (Deserialisierung) | ✅ Korrekt | `if (length != payloadLen) return false;` verhindert Buffer-Overread. |
| `PaperManager::_isDirty` | ❌ Nicht atomar | Siehe Zeitbombe T1. |
| PaperBank PSRAM-Allokation | ✅ Korrekt | `heap_caps_malloc(sizeof(PaperBank), MALLOC_CAP_SPIRAM)` mit NULL-Check. |

### 2.3 RAII-Konsistenz

| Modul | RAII? | Details |
|-------|-------|---------|
| SystemContext | ✅ Ja | Durchgängig `MutexGuard` mit korrekten Timeouts. |
| PaperManager | ✅ Ja | Durchgängig `MutexGuard` mit `DB_WAIT` (150ms). |
| ExposureEngine | ❌ **Nein** | Manuelle `lockState()`/`unlockState()`. Siehe Zeitbombe T2. |
| HardwareManager | ⚠️ Teilweise | `takeI2C()`/`giveI2C()` ist manuell, aber bewusst designed als API. Consumer (SensorManager, DisplayManager) sind korrekt. |

### 2.4 Mutex-Timeout-Strategie

| Kontext | Timeout | Bewertung |
|---------|---------|-----------|
| UI-Getter (SystemContext) | 30ms | ✅ Angemessen. Display-Refresh verträgt Verzögerung. |
| Kritische Setter (SystemContext) | 50ms | ✅ Angemessen. Rare Konflikte. |
| Core 1 Live-Updates | 0ms | ✅ Korrekt. Non-blocking für Echtzeit. |
| PaperManager DB-Zugriff | 150ms | ✅ Konservativ, aber PSRAM-safe. |
| PaperManager Migration/Reset | 500ms | ✅ Einmalige, seltene Operationen. |
| HardwareManager I2C-0 | 20ms | ✅ Verhindert Stalls bei langsamen Peripherals. |

---

## 3. Detaillierte Prüfpunkt-Ergebnisse

### 3.1 Multi-Core & FreeRTOS

**Event-Queue (80 Slots):**  
Mathematisch korrekt dimensioniert. Worst-Case: 4 × 16 + 8 = 72. Reserve: 8 Slots. Die `kMaxEncoderEventsPerCycle`-Begrenzung auf 16 ist der Schlüssel — ohne sie könnte ein einzelner Encoder bei extrem schnellem Drehen hunderte PCNT-Counts in einem Zyklus aufbauen.

**ABORT-Priorisierung:**  
`EV_ABORT = 0` wird korrekt von `dispatchEvent()` erkannt und via `xQueueSendToFront` bevorzugt eingereiht. Der `handleInput`-Guard im AppManager lässt Event 0 durch, auch wenn `isExposureRunning == true`. **Vollständig korrekt.**

**PCNT-Synchronisation:**  
`_lastCount[i]` wird nur um die tatsächlich einreihbaren Steps adjustiert. Bei voller Queue bleiben überschüssige Counts im PCNT-Register erhalten und werden beim nächsten `process()` erneut versucht. **Verlustfrei.**

### 3.2 Speicher-Integrität & Persistenz

**ABI/Padding:**  
Alle persistenten Strukturen nutzen `__attribute__((packed))` mit `static_assert`. `SystemPreferences` (24 Bytes) enthält 4× `float` (16 Bytes) + 8× `uint8_t` (8 Bytes) = 24 Bytes — kein Padding. `PaperProfile` (157 Bytes) enthält mischbare `bool`/`float`/`int` Felder, aber durch `packed` korrekt. **Sauber.**

**Union-Sicherheit (AppSharedState):**  
Das Auto-Tagging über `activeStateMode` ist *implementiert* aber von den Apps *nicht genutzt*. Alle Apps setzen `AppSharedState s{}` mit dem Default-Tag `MODE_BW`. Der DisplayManager nutzt stattdessen `flags.currentMode` als Discriminator — was funktioniert, weil `setMode()` und `setAppState()` beide unter dem gleichen Mutex liegen und der DisplayManager `getFlags()` als atomaren Snapshot bekommt.

**Risiko:** Wenn ein Moduswechsel (Mode A → Mode B) stattfindet, existiert ein winziges Fenster zwischen `_ctx->setMode(newMode)` und dem `_activeApp->onEnter()` → `_ctx->setAppState()`, in dem der DisplayManager alte Union-Daten mit dem neuen Modus sieht. **In der Praxis irrelevant**, da `switchMode` die Belichtung sperrt und der Loop sequenziell ist.

**NVS-Flash-Integrität (PaperManager):**  
Init-Sequenz: `factoryReset()` → `_loadActivePaperFromNVS()` → `_isDirty = false`. Der StorageManager ruft *nach* `init()` seinen eigenen `init()` auf, der dann `deserialize()` aufruft und die Flash-Daten über die NVS-Defaults schreibt. **Korrekt sequenziert.**

**Aber:** `_loadActivePaperFromNVS()` ruft intern `updateActiveProfile(nvsPaper)` auf, welches `_isWriteLocked()` prüft. Beim Boot ist `isExposureRunning = false` und `isMeasuring = false` → kein Lock → funktioniert. Allerdings: `updateActiveProfile()` sperrt den `_dbMutex` erneut, obwohl `_loadActivePaperFromNVS()` selbst keinen Mutex hält. **Korrekt, kein Deadlock** — aber ein Hinweis darauf, dass interne Methoden extern-facing APIs aufrufen (gegen die Konvention, aber harmlos ohne rekursive Mutexe).

**NVS_SPACE-Konstante:** Wird in `PaperManager.cpp` verwendet, ist aber **nirgends im sichtbaren Code definiert**. Dies muss über einen externen Header oder platformio-Build-Flag kommen, oder es ist ein Compile-Error in einer nicht-sichtbaren Konfiguration.

### 3.3 Echtzeit-Hardware-Entkopplung

**Write-Interlock (StorageManager):**  
```cpp
lockOutWrites = flags.isExposureRunning || flags.isMeasuring;
```
Dies blockt Flash-Writes während beider kritischer Phasen. Der Fail-Closed-Pfad (`lockOutWrites = true` bei Mutex-Timeout) ist korrekt. **Vollständig sicher.** Flash-Writes können Core 1 nicht stören, da Core 1 niemals Flash-APIs aufruft.

**I2C-Isolation:**  
- Wire (I2C-0): Core 0, geschützt durch `_i2cMutex` in HardwareManager.
- Wire1 (I2C-1): Core 1, kein Mutex, exklusiv für `ExposureEngine::clTaskLoop()`.

Die TSL2561-Instanz in ExposureEngine wird mit `_hw->getI2C1Bus()` (Wire1) initialisiert:
```cpp
_sensorReady.store(_tsl.begin(_hw->getI2C1Bus()));
```
Die TSL2591-Instanz im SensorManager wird mit `_hw->getI2C0Bus()` (Wire) initialisiert:
```cpp
_tslOk = _tslProbe.begin(_hw->getI2C0Bus());
```
**Physisch und logisch sauber getrennt.** Keine Cross-Bus-Zugriffe gefunden.

**Audio/LED (Core 0 Isolation):**  
- **NeoPixelBus:** Nutzt RMT-Peripherie. `neoStrip->Show()` wird ausschließlich von Core 0 aufgerufen (`HardwareManager::updateNeoPixels`, `HardwareManager::allLightsOff`). Core 1 (`clTaskLoop`) berührt NeoPixel niemals direkt. Stattdessen setzt Core 1 `_shutoffFired = true` (atomar), und Core 0 `tick()` reagiert mit `_hw->allLightsOff()`. **Korrekte Core-1-to-Core-0 Bridge.**

- **Buzzer (LEDC):** `ledcWriteTone()` und `ledcWrite()` werden ausschließlich von Core 0 aufgerufen (via `HardwareManager::playBeep` und dem statischen `buzzerOffCallback` des RTOS-Timers). LEDC-Channel 0 ist fest konfiguriert (`ledcSetup` in `init()`). **Keine Kollision mit Core 1.**

- **Einziger Grenzfall:** Der RTOS Software-Timer `buzzerTimer` läuft auf dem Timer-Service-Task (normalerweise Core 0, `configTIMER_TASK_RUN_CORE`). Der Callback `buzzerOffCallback` ruft `ledcWrite(0, 0)` auf — das ist eine direkte Hardware-Register-Operation, die atomar und Core-sicher ist. **Kein Problem.**

### 3.4 Workflow-Stabilität (Event Guard)

**AppManager::handleInput():**
```cpp
if (flags.isExposureRunning || flags.isMeasuring)
{
    if (event != 0) return;  // Blockiert alles außer ABORT
}
```

**Analyse:**
1. ✅ Während `isExposureRunning`: Nur ABORT (Event 0) durchlässig.
2. ✅ Während `isMeasuring`: Nur ABORT durchlässig.
3. ⚠️ **TOCTOU-Fenster:** `getFlags()` liefert einen Snapshot. Zwischen dem Snapshot und dem tatsächlichen `_activeApp->handleInput(event)` könnte sich der Zustand geändert haben (z.B. Belichtung endet in genau diesem Tick). In der Praxis ist dies ein 1-Tick-Fenster (~1ms) und **völlig unkritisch** — der Event wird einfach im nächsten Zustand verarbeitet.

4. ⚠️ **Guard greift nicht bei `switchMode()`:** Der Mode-Switch hat einen eigenen Guard in `AppManager::switchMode()`:
   ```cpp
   if (flags.isExposureRunning || flags.isMeasuring) return false;
   ```
   Aber `switchMode()` wird nicht über die Event-Queue aufgerufen — es wird direkt von App-Code aufgerufen. Wenn ein App-Modus intern `switchMode()` triggert, passiert dies synchron im selben Task. **Korrekt abgesichert.**

5. ✅ **`onExit()` Safety:** Apps wie BWDoseApp räumen in `onExit()` Pending-Flags auf (`setBWPending(false, 0.0f)`). Das verhindert "Geister-Belichtungen".

---

## 4. Weitere Befunde (Nicht-Zeitbomben)

### F1: ExposureEngine — `lockState` und `lockDose` = const-Methoden mit Seiteneffekten

`lockState()` und `unlockState()` sind als `const`-Methoden deklariert, obwohl sie den Semaphore manipulieren:
```cpp
bool lockState(TickType_t timeout) const;
void unlockState() const;
```
Da `_stateMutex` ein `SemaphoreHandle_t` (Pointer) ist, ist der Pointer `const`, aber das Objekt auf das er zeigt nicht. Das ist **technisch legal**, aber semantisch irreführend und sollte kommentiert werden.

### F2: Globale Instanzen in `main.cpp`

```cpp
SystemContext context;     // Globale Variable!
HardwareManager hw(&context);
PaperManager papers(&context);
```
Die Architektur-Konvention fordert: "Keine globalen Variablen. Instanzen werden in main.cpp erzeugt und per Pointer in Manager/App-Klassen injiziert."

Diese Deklarationen im File-Scope von `main.cpp` sind technisch *globale Variablen* (BSS-Segment). Für ein Arduino-Framework-Projekt ist dies allerdings **die einzige Möglichkeit**, da `setup()` und `loop()` freie Funktionen sind, die keine lokalen Objekte über ihren Scope hinaus leben lassen können. **Akzeptabler Kompromiss** — die Dependency-Injection über Pointer ist korrekt umgesetzt.

### F3: `NVS_SPACE` und `kStateLockWait`/`kDoseLockWait`/`kPreWaitMs` — Undefinierte Symbole

Diese Symbole werden in `PaperManager.cpp` und `ExposureEngine.cpp` verwendet, sind aber in keinem der untersuchten 24 Header oder 21 Source-Dateien definiert. Möglichkeiten:
- Definiert in einem nicht sichtbaren `.h`-File (z.B. in lib/ oder als auto-generierter Header)
- Definiert via `-D` Build-Flags in platformio.ini (nicht gefunden)
- **Compile-Error** in der aktuellen Codebasis

Da IntelliSense keine Fehler meldet, ist die wahrscheinlichste Erklärung, dass diese Konstanten in einer nicht-indizierten oder build-generierten Datei liegen. **Für ein vollständiges Audit müsste dies geklärt werden.**

### F4: `PaperManager::_saveActivePaperToNVS()` — `const_cast` Anti-Pattern

```cpp
const_cast<Preferences&>(_nvs).begin(NVS_SPACE, false);
```
`_saveActivePaperToNVS()` ist `const`, aber `Preferences::begin()` ist nicht const-qualifiziert. Der `const_cast` ist ein Code Smell. Besser: `_nvs` als `mutable` deklarieren oder die Methode nicht-const machen.

### F5: `DisplayManager::smartLCD()` — Keine Thread-Sicherheit

`smartLCD()` schreibt in `_smartLine1`/`_smartLine2`/`_smartPending` ohne Mutex. Der Kommentar sagt "Darf nur von Core 0 gerufen werden!", aber es gibt keinen Runtime-Check dafür. Da `update()` ebenfalls auf Core 0 läuft und der Arduino-Loop single-threaded ist (kein preemptives Multitasking innerhalb Core-0-Loop), ist dies **de facto sicher** — aber eine `configASSERT(xPortGetCoreID() == 0)` wäre eine günstige Absicherung.

### F6: `SensorManager` — Kein Mutex für State-Machine-Zustand

`_measState`, `_measProgress`, `_measResultLux` werden ohne Mutex gelesen und geschrieben. Die öffentlichen Getter:
```cpp
bool isMeasurementRunning() const { return (_measState != MEAS_IDLE && _measState != MEAS_DONE); }
float getMeasurementResult() { float res = _measResultLux; _measState = MEAS_IDLE; return res; }
```
`getMeasurementResult()` ist *nicht* const und *modifiziert* `_measState`. Wenn dies von einem anderen Task aufgerufen wird → Race Condition. Allerdings: Alle Aufrufe kommen aus Core 0 Apps (sequentiell in der Loop). **De facto sicher**, aber nicht formell abgesichert.

---

## 5. Fehlender Kontext für 100% Urteil

| Datei / Information | Warum benötigt |
|---------------------|----------------|
| Definition von `kStateLockWait`, `kDoseLockWait`, `kPreWaitMs` | Klärung ob die Mutex-Timeouts der ExposureEngine 0ms oder >0ms sind. Bei Timeout 0 in `clTaskLoop()` (Core 1) → gut. Bei >0ms → Priority Inversion möglich. |
| Definition von `NVS_SPACE` | Klärung ob NVS-Namespace korrekt isoliert ist. |
| `BWFStopApp.cpp`, `ZoneModeApp.cpp`, `SGDoseApp.cpp` | Verifikation, dass alle Apps das ABORT-Event und die Pending-Flags korrekt behandeln. |
| `SetupApp.cpp` | Prüfung ob Setup-Menü korrekt auf `appState.setup.line1/2` schreibt ohne Union-Korruption. |
| FreeRTOS-Konfiguration (`sdkconfig`, `FreeRTOSConfig.h`) | Klärung von `configTIMER_TASK_RUN_CORE`, `configMAX_PRIORITIES`, ob Priority Inheritance für Mutexe aktiviert ist. |
| Board-JSON (`4d_systems_esp32s3_gen4_r8n16`) | Verifikation der PSRAM-Konfiguration (OPI vs. QSPI). |

---

## 6. Zusammenfassung der Prioritäten

| # | Finding | Schweregrad | Aufwand | Empfehlung |
|---|---------|-------------|---------|------------|
| T1 | `PaperManager::_isDirty` nicht atomar | KRITISCH | 1 Zeile | **Sofort fixen** |
| T2 | ExposureEngine ohne RAII-Locks | HOCH | ~2h Refactor | **Nächster Sprint** |
| T3 | AppSharedState Tag nicht gesetzt | MITTEL-HOCH | ~30min | Tags in allen Apps setzen |
| T4 | StorageManager Clear-Before-Write | MITTEL | Architektonisch | Dokumentieren, ggf. Sequence-Counter |
| F3 | Undefinierte Symbole klären | UNKLAR | 15min | **Sofort klären** (Compile-Test) |
| F4 | `const_cast` in NVS | NIEDRIG | 5min | `mutable` Keyword |
| F5 | smartLCD ohne Core-Check | NIEDRIG | 2 Zeilen | `configASSERT` einfügen |
| F6 | SensorManager ohne State-Mutex | NIEDRIG | Architektonisch | Dokumentieren als "Core-0-only" |
