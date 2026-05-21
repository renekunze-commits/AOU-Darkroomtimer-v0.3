# Architecture Audit Status: Dukatimer v0.902

## Verifizierungsbasis

- Datum: 2026-03-19
- Geprüfte Artefakte: Quellcode in `src/` und `include/`, `ToDo.md`, Build via `pio run -e esp32-s3-devkitc-1`
- Letzter verifizierter Build-Status: fehlgeschlagen in `PaperManager.cpp`

## Blocker L1-L6

- [x] L1: `ExposureEngine` wird in `main.cpp` instanziiert und in `setup()` initialisiert.

- [x] L2: `InputManager` wird in `main.cpp` instanziiert und in `loop()` zyklisch aufgerufen.

- [x] L3: Die fotografischen Phase-2-Apps sind im `AppManager` registriert und für `MODE_BW_DOSE`, `MODE_SG_DOSE`, `MODE_BW_FSTOP` und `MODE_ZONE` geroutet.

- [ ] L4: `liveTime`-Telemetrie ist vollständig korrekt umgesetzt.
  Kommentar: `SystemContext::updateLiveTime()` existiert und `ExposureEngine::tick()` schreibt Telemetrie. Die Zeitbasis ist aber funktional nicht sauber: Es wird `now - _phaseStartedMs` verwendet statt der echten Belichtungsstartzeit `now - _exposureStartedMs`. Damit wird die Pre-Wait-Phase mitgezählt; bei `skipPreWait` kann der Wert sogar veraltet sein. Zusätzlich initialisiert `SystemContext` das Feld `liveTime` noch implizit über `_hw = {0.0f, 0.0f, 25.0f, 22.0f, 0.0f, false};` statt explizit mit sieben Werten.

- [x] L5: Die Hash-Verifikation in `SystemContext::deserialize()` wurde auf das reale Storage-Format umgestellt.
  Kommentar: Der Provider verarbeitet jetzt nur noch das Payload ohne angehängten Hash; der Vergleich erfolgt gegen den externen Hash aus dem `StorageManager`.

- [x] L6: Der Core-1-Pfad schaltet NeoPixel-Hardware nicht mehr direkt ab.
  Kommentar: Bei `remaining <= 0.0f` setzt Core 1 nur noch `_shutoffFired`. Die eigentliche `allLightsOff()`-Ausführung läuft über `tick()` auf Core 0 via `exchange(false)`.

## Zusätzliche Projekt-Realität

- [ ] Der Projektstand ist aktuell kompilierfähig.
  Kommentar: Der echte Build bricht an `src/PaperManager.cpp` ab, weil die Definition `bool PaperManager::_migrateBankAndUpdate(const PaperBank *oldBank)` keine passende Deklaration in `include/PaperManager.h` besitzt. Zusätzlich meldet die Compiler-/Indexer-Sicht in `src/ExposureEngine.cpp` eine nicht passende `Adafruit_TSL2561_Unified::begin(...)`-Signatur sowie weitere offene Warnungen/Fehler.

## Fazit

Die ursprünglichen Integrationsblocker L1, L2, L3, L5 und L6 sind im Code erkennbar adressiert. L4 ist nur teilweise gelöst. Das Projekt ist trotz dieser Fortschritte noch nicht release- oder feldtestfähig, weil der Build aktuell nicht sauber durchläuft.