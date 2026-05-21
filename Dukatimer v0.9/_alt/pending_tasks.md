# Pending Tasks: Dukatimer v0.902

## Kritische Blocker

- Projekt baut aktuell nicht.
  Ursache: `src/PaperManager.cpp` definiert `bool PaperManager::_migrateBankAndUpdate(const PaperBank *oldBank)`, aber `include/PaperManager.h` deklariert nur die Signatur mit `const uint8_t *legacyBuffer, size_t length, uint16_t legacyVersion`.

- `liveTime` ist nur teilweise korrekt verdrahtet.
  `SystemContext::updateLiveTime()` existiert, aber `ExposureEngine::tick()` berechnet die Zeit mit `_phaseStartedMs` statt `_exposureStartedMs`. Das verfälscht die Telemetrie um die Pre-Wait-Phase und ist bei `skipPreWait` potenziell inkonsistent.

- `SensorManager` fehlt vollständig.
  Der Roadmap-Punkt T8 ist offen; es gibt in `Dukatimer v0.9` keine `SensorManager.h/.cpp`.

## Technische Risiken

- `ExposureEngine.cpp` ist nicht compile-clean validiert.
  Die Diagnose meldet eine nicht passende `Adafruit_TSL2561_Unified::begin(...)`-Signatur sowie einen auffälligen Sensor-ID/Overflow-Hinweis beim Konstruktor von `_tsl`.

- Event-Guard im `AppManager` ist nur teilweise vorhanden.
  `switchMode()` blockiert Mode-Wechsel während aktiver Belichtung, `handleInput()` verwirft Eingaben während Belichtung aber nicht. Damit ist der im Plan geforderte Guard nicht vollständig umgesetzt.

- `InputManager` umgeht die im Plan vorgesehene Queue.
  Die Events gehen direkt an `AppManager::handleInput()`. Das reduziert Entkopplung und erschwert spätere Priorisierung oder Mehr-Producer-Szenarien.

- `PaperManager` nutzt weiter manuelle Semaphore-Steuerung.
  `SystemContext` verwendet RAII via `MutexGuard`, `PaperManager` nicht. Das erhöht das Risiko für zukünftige Leaks bei Early-Returns.

## Optimierungspotenzial

- `HardwareStatus` in `SystemContext` sollte explizit mit sieben Werten initialisiert werden.
  Der aktuelle Initializer setzt `liveTime` implizit über `false` und ist unnötig fragil.

- `PaperManager`-Getter sollten `const` werden.
  Das ist klein, verbessert aber Schnittstellenklarheit und reduziert Seiteneffekt-Risiko.

- `DisplayManager` ist implementiert, aber nicht bootstrapped.
  Für echte UI-Validierung fehlt die Instanziierung und der zyklische Aufruf in `main.cpp`.

- `ExposureCL`-Task-Stack bleibt bei 6144 Byte.
  Das war bereits im Audit als knapp markiert und ist noch nicht auf 8192 Byte angehoben.