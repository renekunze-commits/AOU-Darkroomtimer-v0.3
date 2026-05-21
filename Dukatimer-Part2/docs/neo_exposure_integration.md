# NeoPixelHead — Integration mit ExposureEngine (Timing-Messung)

Kurzüberblick

- Ziel: Beschreiben, wie das `NeoPixelHead`-Modul present()-Dauern misst, welche Statistiken verfügbar sind und wie die `ExposureEngine` die Laufzeit-Latenz zur predictiven Abschaltung verwendet.

Geänderte / relevante Dateien

- `src/teensy/NeoPixelHead.h` / `src/teensy/NeoPixelHead.cpp` — present()-Instrumentation, `NeoPixelHeadPresentStats`, `presentStats()` und `resetPresentStats()`.
- `src/teensy/ExposureEngine.h` / `src/teensy/ExposureEngine.cpp` — neue Runtime-API `observeHeadPresentDurationUs(uint32_t)` und `runtimeHeadBusLatencyMs_` (smoothing / clamp).
- `src/teensy/ExposureRuntimeState.h` — sichtbares UI-Feld `runtimeHeadBusLatencyMs` fuer den geglaetteten Millisekundenwert.
- `src/teensy/main.cpp` — synchonisierungs-Aufruf (`syncHeadTimingIntoExposureEngine()`), runtime-seitiger Diagnose-Schalter fuer Testpattern plus serielle Berichte.
- `src/teensy/SetupWorkflow.cpp` / `src/teensy/UiPresenter.cpp` — Setup-Aktionspunkt `HEAD DIAG`, der das feste Head-Testpattern und das Timing-Serialreporting gemeinsam toggelt, plus sichtbare `BUS:...ms`-Telemetrie im Exposure-Diagnoseblock.
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

- Sichtbare UI-Telemetrie
  - `ExposureRuntimeState::runtimeHeadBusLatencyMs` spiegelt den konservativ geglaetteten Buswert als sichtbaren UI-Vertrag.
  - Die aktuelle LVGL-Placeholder-Diagnose zeigt ihn in `UiPresenter::getExposureInfo()` als `BUS:...ms`, ohne die Mess- oder Belichtungslogik selbst in die UI zu verlagern.

Wichtige Einheitengrenze

- Diese Telemetrie bleibt bewusst in us und ms.
- Die neue EV-Darstellung fuer Messworkflows betrifft fotografische Messwerte wie Kalibrierung, Densitometrie oder Spotmessung, nicht technische Timing- oder Busmetriken.

Serielles Reporting (Hardware-Validierung)

- Um die Messwerte live zu sehen, build & upload fuer `teensy41`, oeffne den seriellen Monitor (115200) und aktiviere im Setup-Menue den Punkt `HEAD DIAG`.
- `HEAD DIAG` schaltet bewusst zwei Dinge gemeinsam:
  1. das feste NeoPixel-Head-Testpattern `SegmentCornerMarkers`
  2. den periodischen Serialreport fuer `present()`-Dauern.
- Erneutes Bestaetigen von `HEAD DIAG` schaltet Testpattern und Serialreport wieder aus.
- Ablauf zum Messen:
  1. Firmware hochladen (`teensy41`).
  2. Setup oeffnen und `HEAD DIAG` bestaetigen.
  3. Serielles Log beobachten: `present()`-Dauern (last/avg/max) werden periodisch ausgegeben.
  4. Notiere `maxDurationUs` — dieser Wert bleibt Diagnosemetrik; fuer die laufende Engine-Regelung wird weiter das juengste `lastDurationUs`-Sample beobachtet.

Empfohlene Validierungs-Checks

- Prüfe die Stabilität von `maxDurationUs` über mehrere Pattern-Runs (volle Helligkeit vs. dunkle Szenen).
- Wenn Messwerte stark variieren, erhöhe die Messdauer (mehr Frames) oder passe die Glättungsparameter in `ExposureEngine::observeHeadPresentDurationUs()` an.

Nächste Schritte

- Option: zusaetzlich auch den abgeleiteten predictive shutoff lead (`lead`) sichtbar machen.
- Option: Hardware-Logs und UI-Anzeige parallel vergleichen, um Glättungs- und Clamp-Parameter mit realen Lastfaellen abzugleichen.

Wenn du möchtest, übernehme ich direkt diesen Changelog-Eintrag oder ergänze die UI-Telemetrie.
