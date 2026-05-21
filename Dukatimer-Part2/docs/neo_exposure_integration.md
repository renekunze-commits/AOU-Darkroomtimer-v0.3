# NeoPixelHead — Integration mit ExposureEngine (Timing-Messung)

Kurzüberblick
- Ziel: Beschreiben, wie das `NeoPixelHead`-Modul present()-Dauern misst, welche Statistiken verfügbar sind und wie die `ExposureEngine` die Laufzeit-Latenz zur predictiven Abschaltung verwendet.

Geänderte / relevante Dateien
- `src/teensy/NeoPixelHead.h` / `src/teensy/NeoPixelHead.cpp` — present()-Instrumentation, `NeoPixelHeadPresentStats`, `presentStats()` und `resetPresentStats()`.
- `src/teensy/ExposureEngine.h` / `src/teensy/ExposureEngine.cpp` — neue Runtime-API `observeHeadPresentDurationUs(uint32_t)` und `runtimeHeadBusLatencyMs_` (smoothing / clamp).
- `src/teensy/main.cpp` — synchonisierungs-Aufruf (`syncHeadTimingIntoExposureEngine()`), optionale serielle Berichte (`kEnableHeadTimingSerialReport`).
- `src/teensy/HeadTimingConstants.h` — Basiskonstanten `kHeadBusLatencyMs`, `kHeadFadeOutMs`.

APIs & Verhalten
- `NeoPixelHeadPresentStats` (wichtige Felder):
  - `presentCalls` : Anzahl `present()`-Aufrufe seit Reset.
  - `appliedFrames` : tatsächlich ausgegebene Frames.
  - `skippedFrames` : übersprungene Frames (Rate-Limit).
  - `lastDurationUs`, `averageDurationUs`, `maxDurationUs` : Messwerte in Mikrosekunden.

- Zugriff:
```cpp
const auto& stats = neoPixelHead.presentStats();
// z.B. stats.maxDurationUs, stats.averageDurationUs
neoPixelHead.resetPresentStats();
```

- `ExposureEngine::observeHeadPresentDurationUs(uint32_t durationUs)`
  - Erwartet die gemessene `present()`-Dauer in Mikrosekunden.
  - Wandelt intern in Millisekunden um, clamped an die Basiskonstante `kHeadBusLatencyMs` und an eine konservative Obergrenze.
  - Aktualisiert `runtimeHeadBusLatencyMs_` mit langsamer Abwärts-Glättung (niemals unter `kHeadBusLatencyMs`), damit die Engine die predictive shutoff lead adaptiv berechnet.
  - `predictiveShutoffLeadMs()` liefert `max(kHeadBusLatencyMs, runtimeHeadBusLatencyMs_) + kHeadFadeOutMs`.

Serielles Reporting (Hardware-Validierung)
- Um die Messwerte live zu sehen, setze in `src/teensy/main.cpp` die Flagge `kEnableHeadTimingSerialReport = true`, build & upload für `teensy41` und öffne den seriellen Monitor (115200).
- Ablauf zum Messen:
  1. Firmware hochladen (teensy41).
  2. Testpattern aktivieren (`NeoPixelHeadTestPattern`) oder manuelle Befehle ausgeben.
  3. Serielles Log beobachten: `present()`-Dauern (last/avg/max) werden periodisch ausgegeben.
  4. Notiere `maxDurationUs` — dieser Wert wird an die `ExposureEngine` übergeben.

Empfohlene Validierungs-Checks
- Prüfe die Stabilität von `maxDurationUs` über mehrere Pattern-Runs (volle Helligkeit vs. dunkle Szenen).
- Wenn Messwerte stark variieren, erhöhe die Messdauer (mehr Frames) oder passe die Glättungsparameter in `ExposureEngine::observeHeadPresentDurationUs()` an.

Nächste Schritte
- Option: UI-Telemetrie hinzufügen, die `runtimeHeadBusLatencyMs_` anzeigt.
- Option: Changelog-Eintrag mit den vorgenommenen API-Änderungen erstellen.

Wenn du möchtest, übernehme ich direkt diesen Changelog-Eintrag oder ergänze die UI-Telemetrie.
