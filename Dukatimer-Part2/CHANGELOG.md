# CHANGELOG

## Unreleased — 2026-04-26

- NeoPixelHead: present() Timing-Instrumentation
  - `NeoPixelHeadPresentStats` hinzugefügt (`presentCalls`, `appliedFrames`, `skippedFrames`, `lastDurationUs`, `averageDurationUs`, `maxDurationUs`).
  - APIs: `presentStats()`, `resetPresentStats()`.
  - Dateien: `src/teensy/NeoPixelHead.h`, `src/teensy/NeoPixelHead.cpp`.

- ExposureEngine: Runtime-Ingestion von Head-Latenz
  - Neue API `observeHeadPresentDurationUs(uint32_t)` implementiert.
  - `runtimeHeadBusLatencyMs_` mit konservativem Clamp und langsamer Abwärts-Glättung; `predictiveShutoffLeadMs()` verwendet runtime-Latenz + `kHeadFadeOutMs`.
  - Dateien: `src/teensy/ExposureEngine.h`, `src/teensy/ExposureEngine.cpp`.

- main.cpp: Synchronisierung + optionales Reporting
  - `syncHeadTimingIntoExposureEngine()` hinzugefügt und optionales serielles Reporting (`kEnableHeadTimingSerialReport`).
  - `neoPixelHead.resetPresentStats()` beim Start.
  - Datei: `src/teensy/main.cpp`.

- Dokumentation
  - Neue Doku: `docs/neo_exposure_integration.md` — beschreibt Messung, APIs und Validierungsablauf.

- Build
  - Änderungen erfolgreich für `teensy41` kompiliert (Build-Validierung durchgeführt).

Hinweis: Hardware-Messungen (Stromaufnahme / reale present()-Dauern unter Last) sind noch ausstehend. Empfohlen: serielles Reporting aktivieren und Messläufe durchführen, um `maxDurationUs` zu validieren.

Nächste Schritte:
- Changelog-Eintrag in die Versionshistorie einpflegen und committen.
- Optional: UI-Telemetrie hinzufügen, die `runtimeHeadBusLatencyMs_` anzeigt.
