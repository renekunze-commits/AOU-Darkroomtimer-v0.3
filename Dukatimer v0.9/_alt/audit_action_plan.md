# Audit Action Plan Status: Dukatimer v0.902

## Hinweis

Diese Datei wurde am 2026-03-19 neu angelegt, weil im Projekt kein bestehendes `audit_action_plan.md` vorhanden war. Der Status unten basiert ausschließlich auf verifiziertem Quellcode und einem echten Build-Versuch.

## Tasks T1-T8

- [ ] T1: MutexGuard RAII-Klasse definieren und manuelle `xSemaphoreTake/Give`-Paare in `SystemContext.cpp` und `PaperManager.cpp` ersetzen.
  Kommentar: In `SystemContext.cpp` ist `MutexGuard` umgesetzt. `PaperManager.cpp` nutzt weiterhin manuelle Semaphore-Paare und erfüllt den Task daher nur teilweise.

- [x] T2: Dirty-Timestamps im `StorageManager` trennen.
  Kommentar: Nicht als zwei benannte Variablen, sondern generischer pro Provider via `_dirtyLatched[i]` und `_dirtySince[i]` umgesetzt. Die ursprüngliche Koppelung über einen einzigen Zeitstempel ist damit funktional beseitigt.

- [ ] T3: Lese-Methoden im `PaperManager` als `const` deklarieren.
  Kommentar: Methoden wie `isFixedGradeActive()`, `getActiveFixedGradeValue()` und `getActiveIndex()` sind im Header weiterhin nicht `const`.

- [x] T4: `IAppMode.h` erstellen.

- [ ] T5: `AppManager.h/.cpp` erstellen und Belichtungs-Guard für Events implementieren.
  Kommentar: `AppManager` existiert, hält alle App-Instanzen und routet Modi korrekt. Der Guard in `switchMode()` ist vorhanden. Ein vollständiger Event-Guard in `handleInput()` während aktiver Belichtung ist jedoch nicht implementiert.

- [ ] T6: `InputManager.h/.cpp` portieren und Eingaben über neutrale Events bzw. Queue dispatchen.
  Kommentar: `InputManager` ist vorhanden, initialisiert Encoder/Taster und wird in `main.cpp` aufgerufen. Die Events werden aktuell direkt an `AppManager::handleInput()` weitergereicht; eine FreeRTOS-Queue gibt es nicht.

- [ ] T7: `ExposureEngine.h/.cpp` portieren.
  Kommentar: Die Engine existiert, wird in `main.cpp` instanziiert und bindet `esp_timer`, `updateLiveDose()` sowie `setExposureLock()` ein. Der Stand ist aber noch nicht compile-clean: Der Build stoppt vorher in `PaperManager.cpp`, und die Diagnose meldet zusätzlich eine nicht passende `Adafruit_TSL2561_Unified::begin(...)`-Signatur. Außerdem verwendet `liveTime` aktuell `_phaseStartedMs` statt `_exposureStartedMs`.

- [ ] T8: `SensorManager.h/.cpp` implementieren.
  Kommentar: Es gibt im Workspace kein `SensorManager`-Modul in `Dukatimer v0.9`.

## Gesamtstatus

Vier Aufgaben sind vollständig oder funktional gleichwertig erledigt: T2 und T4 sicher, T5/T6/T7 nur als Teilumsetzung, T1/T3/T8 offen. Die Roadmap ist damit noch nicht abgeschlossen.