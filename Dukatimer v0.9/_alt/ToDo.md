Dukatimer v0.91 — Architektur-Review & Phase 2 Roadmap

Status: Fundament (Phase 1) abgeschlossen und gehärtet.
Datum: Aktueller Entwicklungsstand.

A. POSITIV-MATRIX — Das gehärtete Fundament

Die massiven Schwachstellen von v0.3 und die initialen Fehler des v0.9-Entwurfs sind behoben.

Architektur: Strikte 4-Schichten-Trennung ohne Zirkelbezüge. Globals sind zu 100% eliminiert.

Speicher-Sicherheit: SettingsBlob, PaperProfile und PaperBank sind via __attribute__((packed)) gegen Padding-Fehler geschützt.

Daten-Integrität: Zero-Copy SPI Writes, FNV-1a Hashing für Settings & Papers, Best-Effort Versionsmigration implementiert. Keine Raw-Pointer-Leaks (allocateBankCopyForStorage).

Thread-Sicherheit: Non-blocking liveDose Update (Timeout 0), atomare Sichtbarkeit via std::atomic<bool> für Dirty-Flags, Lock-Before-Copy in den Gettern mit boolescher Fehlerbehandlung. Bounded Timeouts (pdMS_TO_TICKS(500)) eliminieren Deadlocks.

Ressourcen: Statische Allokation der Manager in main.cpp (kein new, kein Heap-Fragmentierungsrisiko). Asynchroner Buzzer via FreeRTOS Timer (Hardware-Timer nicht blockiert).

B. OFFENE OPTIMIERUNGEN (Phase 1.5 - Feinschliff)

Diese Punkte sind keine Bugs, sondern C++ Best-Practices, die wir vor oder während Phase 2 einziehen sollten.

ID

Bereich

Ist-Zustand

Soll-Zustand

Aufwand

O1

RAII Mutex-Guard

Manuelle xSemaphoreTake/Give-Paare in ~30 Stellen.

MutexGuard RAII-Klasse. Mutex wird im Destruktor automatisch freigegeben.

Klein

O4

Dirty-Timestamps

Ein globaler _lastChangeTime im StorageManager für Settings (setzt den Timer) und Papers. Koppelt Speicherzyklen unnötig.

_settingsChangedAt und _papersChangedAt separat tracken.

Trivial

O5

Const-Correctness

Getter wie getActiveIndex() sind nicht const-qualifiziert.

const wo möglich anhängen — dokumentiert Intent und hilft dem Compiler.

Trivial

O7

double → float

PaperProfile nutzt double für LUT. ESP32 hat keine Hardware-FPU für double (Soft-Float = langsam).

Prüfen, ob float-Präzision (7 Dezimalstellen) für die Photometrie ausreicht. Halbierung des Speicherbedarfs.

Mittel

C. ARCHITEKTUR-LÜCKEN (Der Fokus für Phase 2)

Diese Komponenten fehlen aktuell komplett, um aus dem Fundament ein funktionierendes System zu machen.

ID

Fehlendes Modul

Impact

Prio

L1

IAppMode Interface

Ohne Basis-Klasse kein modulares App-Konzept.

P0

L2

AppManager (Event-Router)

Kein InputQueue-Consumer, kein Mode-Switching, kein Belichtungs-Guard.

P0

L3

InputManager

Encoder-Polling, Button-Debounce und Interrupts fehlen.

P0

L5

ExposureEngine

Die zeitpräzise Belichtungssteuerung (esp_timer) fehlt. Echtzeit-Herzstück.

P1

L4

DisplayManager

UI (Nextion/LCD) ist nicht angebunden.

P1

L6

SensorManager

Sensoren (TSL2591/2561, BMP280, OneWire) lesen noch keine echten Daten in den Context.

P1

D. MASTER-TODO-LISTE ZUR UMSETZUNG

Vorbereitungs-Sprint (Abschluss Phase 1)

[ ] T1: MutexGuard RAII-Klasse (z.B. in Types.h oder SystemContext.h) definieren und manuelle xSemaphoreTake/Give in SystemContext.cpp und PaperManager.cpp ersetzen.

Kommentar: In `SystemContext.cpp` ist `MutexGuard` umgesetzt. In `PaperManager.cpp` werden die Semaphoren weiterhin manuell genommen und freigegeben.

[x] T2: _lastChangeTime im StorageManager in _settingsChangedAt und _papersChangedAt trennen.

Kommentar: Funktional umgesetzt über `_dirtyLatched[i]` und `_dirtySince[i]` pro Provider statt über zwei feste Zeitstempel.

[ ] T3: Lese-Methoden im PaperManager (z.B. isFixedGradeActive() const) als const deklarieren.

Phase 2, Schritt 1: Das OS-Skelett

[x] T4: IAppMode.h erstellen (Interface mit onEnter, handleInput, onUpdate, onExit).

[ ] T5: AppManager.h/.cpp erstellen.

Kommentar: `AppManager` existiert und routet die Modi korrekt. Der im Plan geforderte Belichtungs-Guard für eingehende Events ist aber nicht vollständig umgesetzt; aktuell schützt `switchMode()` vor Wechseln während Belichtung, `handleInput()` reicht Events weiterhin direkt durch.

Integration eines Pointers auf den aktiven IAppMode.

Implementierung von switchMode().

Belichtungs-Guard: Events verwerfen, wenn SystemContext -> isExposureRunning wahr ist.

[ ] T6: InputManager.h/.cpp portieren.

Kommentar: `InputManager` ist implementiert und in `main.cpp` verdrahtet. Der Planpunkt „Dispatch an die FreeRTOS-Queue“ ist jedoch nicht erfüllt; die Events gehen direkt an `AppManager::handleInput()`.

Encoder- und Button-Abfrage via Interrupts/Polling.

Überführung in neutrale InputEvent-Strukturen und Dispatch an die FreeRTOS-Queue.

Phase 2, Schritt 2: Die Echtzeit- und Hardware-Kerne

[ ] T7: ExposureEngine.h/.cpp portieren.

Kommentar: `ExposureEngine` ist implementiert und in `main.cpp` verdrahtet. Der Stand ist aber noch nicht build-stabil: Der Projekt-Build bricht aktuell in `PaperManager.cpp` ab, und für `ExposureEngine.cpp` sind zusätzliche API-/Diagnoseprobleme offen. Außerdem basiert `liveTime` derzeit auf `_phaseStartedMs` statt auf der echten Belichtungsstartzeit.

Integration der esp_timer-Logik.

Non-blocking Feedback an SystemContext::updateLiveDose().

Kopplung mit HardwareManager::setExposureLock().

[ ] T8: SensorManager.h/.cpp implementieren.

Einbindung der Adafruit-Treiber für TSL2561, TSL2591, BMP280, DS18B20.

I2C-Locking (HardwareManager::takeI2C()) bei jedem Bus-Zugriff.

[ ] T9: DisplayManager.h/.cpp portieren.

Kommentar: `DisplayManager.h/.cpp` existieren. Das Modul ist aber aktuell nicht in `main.cpp` instanziiert oder zyklisch aufgerufen und damit nicht end-to-end integriert.

Entkoppelte UI-Aktualisierung basierend auf SystemContext-Shadow-Buffern.