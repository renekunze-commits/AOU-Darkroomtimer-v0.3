# Head Timing und I²C-Timeout — Änderungen (2026-04-27)

Kurzfassung

- Zentralisierte Head-Timing-Konstanten wurden nach `src/teensy/HeadTimingConstants.h` verschoben.
- `ExposureEngine` und `NeoPixelHead` nutzen jetzt diese gemeinsamen Konstanten.
- Die Dokumentation zur I²C-Bus-Härtung wurde auf den realen Plattformstand korrigiert: moderne Cores bieten oft Timeout-Unterstützung, der Teensy-Core jedoch nicht.

Geänderte/wichtige Dateien

- `src/teensy/HeadTimingConstants.h` — neue Konstanten: `kHeadSoftStopMs`, `kHeadPredictiveShutoffBaseLeadMs`, `kHeadPresentDurationClampMs` (zusätzlich zu `kHeadBusLatencyMs`/`kHeadFadeOutMs`).
- `src/teensy/ExposureEngine.cpp` — verwendet jetzt die zentralen Konstanten und berechnet die predictive shutoff lead dynamisch anhand der laufzeitgemessenen Bus-Latenz.
- `src/teensy/ExposureRuntimeState.h` / `src/teensy/UiPresenter.cpp` — exportieren und zeigen die geglaettete Head-Bus-Latenz jetzt sichtbar als UI-Telemetrie (`BUS:...ms`).
- `src/teensy/NeoPixelHead.h` — nutzt `kHeadSoftStopMs` für weiches Abschalten.
- `src/teensy/SensorManager.cpp` — dokumentiert explizit, dass der Teensy-Core keinen softwareseitig nutzbaren Hardware-I²C-Timeout bereitstellt.
- `src/teensy/DukaTeenBoardPins.h` — zentrale Software-Spiegelung der aktiven Teensy-Pinbelegung aus der Schaltplanuebersicht.

Warum diese Änderungen

- Konsistenz: Eine einzige Quelle für Head-Timing verhindert Inkonsistenzen zwischen Treiber (`NeoPixelHead`) und Steuerlogik (`ExposureEngine`).
- Robustheit: Die Härtung des Busses greift primär bei modernen Cores (ESP32/ESP8266 etc.). Der Teensy-Core bietet hardwareseitig keine Non-Blocking-Rückkehr aus defekten I²C-Transaktionen; ein Watchdog-Reset ist hier das Mittel der letzten Instanz.

Einheitenhinweis

- Diese Doku beschreibt bewusst technische Telemetrie in us, ms und Buszuständen.
- Die neue EV-Darstellung fuer Messworkflows gilt nur fuer fotografisch interpretierbare Messwerte und nicht fuer Timing- oder I2C-Diagnosewerte.

Status der I²C-Bus-Härtung

- Die Härtung des Busses greift primär bei modernen Cores (ESP32/ESP8266 etc.), die eine echte Wire-Timeout-API bereitstellen.
- Der Teensy-Core (`WireIMXRT`) bietet hardwareseitig keine Non-Blocking-Rückkehr aus defekten I²C-Transaktionen.
- `Stream::setTimeout()` ist in diesem Kontext keine Lösung, weil es keine blockierenden `Wire.endTransmission()`- oder `Wire.requestFrom()`-Aufrufe absichert.
- Für den Teensy 4.1 ist deshalb ein Hardware-Watchdog-Reset die letzte wirksame Rückfallebene bei einem festhängenden I²C-Bus.
- Seit 2026-05-02 erzwingt der Teensy-Laufzeitpfad nach einem erkannten Main-Loop-Watchdog-Reset sowie bei lokalen TSL2561-Dienstfehlern (`InitializationFailed`, `ReadFailure`, `InterruptWatchdog`, `StaleSample`) explizit Time-Modus fuer neue Splitgrade-Starts und zeigt den Grund sichtbar in der Diagnose an.
- Seit 2026-05-05 ist ausserdem die Zielkombination fuer echte lokale Entklemmung festgelegt: kein Repurposing von `I2C1_*`, `TSL_INT` oder `1W_DATA`, sondern ein dedizierter, Teensy-gesteuerter High-Side-Schalter in `+3V3_HEAD`; die Recovery-Sequenz bleibt Software-Eigentum des `SensorManager`.

So testest du die Änderungen (Hardware)

1. Build & Upload fuer Teensy 4.1: `pio run -e teensy41 -t upload`.
2. Setup am Geraet oeffnen und `HEAD DIAG` aktivieren, damit Testpattern und serieller Timing-Report gemeinsam laufen.

3. Oeffne den seriellen Monitor (115200) und beobachte die Meldungen zu `present()`-Dauer, `bus` und `lead`.
4. Vergleiche parallel die sichtbare UI-Telemetrie `BUS:...ms` mit dem seriellen `bus`-Wert.
5. Fuehre mehrere Runs durch und sammle die Logs — diese werden fuer die Analyse der Smoothing-/Clamp-Parameter benoetigt.

Empfohlene nächste Schritte

- Commit + Push: dokumentierte Änderungen zusammen mit den Code-Änderungen committen und pushen.
- Hardware-Validierung: serielle Logs sammeln, Werte vergleichen, `kHeadPresentDurationClampMs` ggf. anpassen.
- Optional: zusaetzlich den abgeleiteten predictive shutoff lead auch sichtbar machen.

## Delta 2026-05-01: Spike-resistente Latenz-Ingestion

- Die ExposureEngine uebernimmt groessere Anstiege der beobachteten Head-Latenz nicht mehr sofort, sondern erst nach bestaetigten Wiederholungen innerhalb eines kleinen Fensters.
- Kleine Aufwaertsaenderungen bleiben direkt erlaubt; Abwaerts wird weiterhin langsam geglaettet.
- Der Wiring-Pfad speist jetzt bewusst nur frische `lastDurationUs`-Samples aus neu angewendeten Frames statt `maxDurationUs` in die Engine ein. `maxDurationUs` bleibt Diagnose fuer serielle Reports, aber keine Regelgroesse mehr.
- Hintergrund: Das bisherige Ingestionsverhalten konnte einen einmaligen `present()`-Spike als neue Bus-Laufzeit festschreiben und damit den predictive shutoff lead fuer nachfolgende normale Frames zu weit vorziehen.

## Delta 2026-05-01: Hardware-Stresstelemetrie aktiviert

- `kEnableHeadTimingSerialReport` ist jetzt standardmaessig aktiviert, damit die aktuelle Hardware direkt unter UI-/Menue-Last beobachtet werden kann.
- Der serielle Report zeigt jetzt zusaetzlich die laufzeitadaptierte Head-Bus-Latenz (`bus`) und den daraus abgeleiteten predictive shutoff lead (`lead`).
- Damit laesst sich bei wildem Parallelbedienen direkt sehen, ob die Head-Latenz nur kurz spike-t oder dauerhaft auf neue Plateaus steigt.

## Delta 2026-05-02: Watchdog-latchter TSL-Time-Fallback

- Der Teensy bootet nach einem durch den Main-Loop-Watchdog markierten Anwendungs-Reset nicht mehr still in potenziell unveraenderter Dose-Nutzung, sondern haelt eine latched Diagnose und erzwingt fuer neue Splitgrade-Starts Time-Modus.
- Dasselbe gilt fuer lokale TSL2561-Dienstfehler, die den Sensorpfad zwar nicht hart blockieren, aber fuer geschlossene Dosisregelung unzuverlaessig machen.
- Diese Aenderung taeuscht keinen nicht existierenden `Wire`-Timeout vor; sie dokumentiert und verdrahtet nur die tatsaechlich verfuegbare Recovery-Politik: Reset als letzte Rueckfallebene, danach sichtbare Degradierung auf Time.

## Delta 2026-05-05: Teensy-TX-Vertrag und I2C-Recovery-Richtung festgezogen

- Der Teensy-Servicepfad sendet jetzt nicht nur `availableForWrite()`-gesichert, sondern sortiert wartende Frames auch wirklich nach ihrer dokumentierten Prioritaet `TeensyCommand/VFS > Heartbeat > RemoteRender > Diagnostic`.
- Der direkte Flush aus Producer-Pfaden ist auf ein kleines Budget pro Aufruf begrenzt, damit bereits wartende UART-Frames nicht in einem fachfremden Call-Kontext komplett abgearbeitet werden.
- Fuer lokale I2C-Entklemmung ist die Richtung jetzt explizit gesetzt: aktuelle Hardware hat noch keinen schaltbaren Kopfversorgungszweig; die vorgesehene Erweiterung ist ein dedizierter High-Side-Schalter in `+3V3_HEAD`, den spaeter der `SensorManager` fuer eine begrenzte Power-Cycle/Reinit-Sequenz nutzt.

Kontakt / Rückfragen

- Bei Unklarheiten: ich passe die Doku an oder erweitere sie um Messbeispiele/Log-Auszüge.

---
Datum: 2026-04-27
