---
title: Dukatimer-Part2 Audit — ExposureEngine & Inter-MCU Link (Ausführlicher Befund)
date: 2026-04-25
author: Senior Embedded C++ Systems Architect
---

# Auditbericht: ExposureEngine, SensorManager, EspServiceLink, NeoPixelHead

Kurz: Detaillierte Befunde aus einem gnadenlosen Code-Audit bezüglich der
5 architektonischen Hard-Rules (Realtime-Integrität, präzise Dosis-Integration,
VFS-Integrität, Trennung Fachlichkeit/Ausführung, Speicher/Performance).

Zielgruppe: Entwickler Team Dukatimer-Part2

Scope & Methodik
- Geprüfte Dateien (Auszug):
  - `src/teensy/SensorManager.cpp` (+ `.h`)
  - `src/teensy/ExposureEngine.cpp` (+ `.h`)
  - `src/teensy/EspServiceLink.cpp` (+ `.h`)
  - `src/teensy/NeoPixelHead.cpp` / `NeoPixelHead.h`
  - `src/teensy/main.cpp`

Analysezeitpunkt: 2026-04-25 — Quelltext-Ansicht und Laufzeit-Architektur-Spezifikation
(`docs/software/erledigt/dukatimer-part2-laufzeitspezifikation-exposureengine-closed-loop-und-fail-safe.md`).


## Executive Summary

- Kritische Verstöße (muss vor Real-Belichtung behoben werden):
  1. I2C-Transaktionen ohne deterministischen Timeout → mögliches Blocking
     in der Hauptschleife → Brandrisiko bei 15A NeoPixel-Matrix.
  2. VFS-Schreibpfad wird während `realtimeHold` weiterhin verarbeitet (vor-
     gepufferte Frames + SD-Schreib-Operationen) → Blocking in `espServiceLink.tick()`
     während `EXPOSING`.
  3. VFS implementiert keinen atomaren Datei-Upload (direktes `O_TRUNC` auf Zielpfad).

- Hohe Priorität (muss vor Feldtests):
  - Atomare VFS-Uploads (.tmp + rename) — Vermeidung unwiederbringlicher JSON-Zerstörung.
  - Gate VFS-Operationen, wenn `realtimeHold` gesetzt ist (ACK mit Busy oder Drop).

- Mittlere Priorität:
  - Konsolidierung der Fade-/Latency-Konstanten zwischen `ExposureEngine` und `NeoPixelHead`.
  - Kleine ISR-Atomics und Beobachtungsfenster härten gegen Doppelverarbeitung.


## Detaillierte Befunde und vorgeschlagene Fixes

1) I2C / TSL2561 Blocking-Risiko (Hard-Rule 1)
------------------------------------------------

Ort:

- `src/teensy/SensorManager.cpp`
  - `writeTslRegister()`
  - `readTslChannels()`
  - `pollLocalTsl2561()` / `initializeLocalTsl2561()`

Befund:

- `TwoWire`-Aufrufe wie `endTransmission()` und `requestFrom()` sind potentiell blockierend
  (abhängig vom Core/I2C-Controller), und es wird kein konfigurierter, deterministischer
  Timeout gesetzt. Bei SDA/SCL-Hänger (z. B. Sensor Spannungsausfall) kann die
  Hauptschleife mehrere 10–100ms pro Aufruf blockieren; kumulativ führt das zu
  spürbaren Verzögerungen bis hin zum Ausbleiben von `ExposureEngine.tick()`-Aufrufen.

Risiko:

- Matrix bleibt bei vollem Output, während Sensor-Read / I2C hängt → Brand-/Überstromrisiko.

Empfohlener Fix (Konzept):

- Setze expliziten, kleinen I2C-Timeout auf `tslBus_` direkt nach `begin()`/`setClock()`:

```cpp
// SensorManager::initializeLocalTsl2561()
tslBus_->begin();
tslBus_->setClock(kTslI2cClockHz);
tslBus_->setTimeout(5); // 5 ms per I2C operation (adjustierbar)
```

- Alternativ: Verwende asynchrone I2C-API oder FSM mit `start`-`poll`-Pattern, niemals
  blocking calls in `tick()` ohne garantierten, kleinen Timeout.

Rationale: 5 ms × N Fehlversuche bleibt << ExposureEngine Watchdog (420 ms) und
ermöglicht deterministischen Übergang in FAULT.


2) VFS / EspServiceLink: Verarbeitung während `realtimeHold` (Hard-Rule 1)
-----------------------------------------------------------------------

Ort:

- `src/teensy/EspServiceLink.cpp`
  - `setRealtimeHold()`
  - `readFrames()`
  - `processVfsRequest()`
  - `processVfsChunk()`

Befund:

- `setRealtimeHold(true)` setzt RTS per `digitalWrite(...)` und `realtimeHoldActive_ = true`,
  aber empfangene (gepufferte) Bytes im internen `serialReadStorage_` werden weiterhin
  in `tick()` via `readFrames()` gelesen und decodiert. `processVfsChunk()` schreibt
  direkt auf die SD (potentiell lange Blocking-Operationen), während `EXPOSING` läuft.

Risiko:

- SD-Write oder Verarbeitung eines großen Uploads kann `espServiceLink.tick()` blockieren →
  Hauptloop Verzögerung → `exposureEngine.tick()` wird nicht regelmäßig ausgeführt →
  Fail-Safe- und Watchdog-Reaktionen verzögert.

Empfohlener Fix (Minimal):

- Frühzeitige Rejection: wenn `realtimeHoldActive_` gesetzt ist, *reject* VFS-Requests
  und VFS-Chunks mit `VfsStatusCode::Busy` statt sie zu verarbeiten.

Code-Snippet (oben in `processVfsRequest` / `processVfsChunk` zu Beginn):

```cpp
if (realtimeHoldActive_) {
    state_.vfs.lastStatus = dukatimer::protocol::VfsStatusCode::Busy;
    sendVfsError(dukatimer::protocol::VfsStatusCode::Busy, payload.transactionId, 0u, 0u, 0u, nowMs);
    return;
}
```

- Optional: In `readFrames()` nach Decodierung die VFS-MessageTypes droppen, wenn `realtimeHoldActive_`.

Rationale: RTS/CTS soll die Kommunikation „einfrieren“. Die Implementierung muss das nicht
nur auf Byte-Ebene tun, sondern auch auf Frame-/Operationsebene.


3) Atomarer VFS-Datei-Upload fehlt (Hard-Rule 3)
-------------------------------------------------

Ort:

- `src/teensy/EspServiceLink.cpp`
  - `processVfsRequest()` (öffnen mit `O_TRUNC`)
  - `processVfsChunk()` (partial writes, finalize logic)

Befund:

- Bei `FileWriteRequest` wird direkt auf das Ziel mit `O_TRUNC` geöffnet. Ein Übertragungsabbruch
  (Abort-Chunk, Write-Error, Reset) lässt eine beschädigte Ziel-Datei zurück.

Risiko:

- Unwiederbringliche Zerstörung sensibler JSON-Profile oder Kalibrierungsdateien.

Empfohlener Fix (Staging + atomic rename):

- Schreibe in eine temporäre Datei (`<path>.tmp`) und nur nach einem erfolgreichen Finalize
  `sd_->rename(tmpPath, finalPath)` (atomarer Rename auf FAT-fs, SdFat bietet `rename`).
- Entferne `.tmp` bei Abort/Fehler.

Code-Skizze:

```cpp
// processVfsRequest(): open tmp
char tmpPath[...];
snprintf(tmpPath, sizeof(tmpPath), "%s.tmp", requestedPath);
if (!activeWriteFile_.open(tmpPath, O_WRONLY | O_CREAT | O_TRUNC)) { ... }
// store tmpPath separately

// processVfsChunk(): on finalize:
activeWriteFile_.close();
if (!sd_->rename(tmpPath, activeVfsPath_)) {
    // Fehlerbehandlung: send VfsError, set status
}

// resetWriteSession(): ensure tmp removed on abort
sd_->remove(tmpPath);
```

Hinweis: Pflege `activeVfsTmpPath_` State und sichere `resetWriteSession()` gegen partiellen Zustand.


4) Predictive Shutdown vs. Fade-Out (Hard-Rule 4)
-------------------------------------------------

Ort:

- `src/teensy/ExposureEngine.h`:
  - `kPredictiveShutoffLeadMs = 8`
  - `kHeadSoftStopEquivalentLatencyMs = 32` (→ predictiveShutoffLeadMs() = 40)

- `src/teensy/NeoPixelHead.h`:
  - `kSoftStopMs = 64` (tatsächliche Soft-Stop Dauer)

Befund:

- Die Engine plant den Abschaltzeitpunkt mit 40 ms Vorlauf, der Head benötigt tatsächlich
  64 ms zum Fade-Out. Ergebnis: verbleibende Lichtemission (Fade-Out) wird **nicht** in
  die Ist-Dosis eingerechnet → systematische Überdosierung (nicht trivial zu messen,
  besonders bei hohen Lux-Werten).

Risiko:

- Messbare Dosisabweichung, inkonsistente Reproduzierbarkeit. Die Spezifikation fordert
  explizit, dass Fade-Out (~100ms) einbezogen wird.

Empfohlener Fix:

- Extrahiere gemeinsame Timing-Konstanten in `src/teensy/HeadTiming.h` und verwende diese
  in beiden Modulen. Beispiel:

```cpp
// HeadTiming.h
static constexpr uint32_t kHeadBusLatencyMs = 8;    // Bus/Latch latency
static constexpr uint32_t kHeadFadeOutMs = 100;    // konsolidierter Fade-Out (spec)
```

- In `ExposureEngine::predictiveShutoffLeadMs()` returniere `kHeadBusLatencyMs + kHeadFadeOutMs`.
- In `NeoPixelHead` setze `kSoftStopMs = kHeadFadeOutMs`.

Hinweis: Die exakte Fade-Zeit kann hardwareabhängig sein; sorge für Build-Time-Checks
und Mess-Hooks, damit Feldkalibrierung möglich ist.


5) ISR- und Concurrency-Feinheiten (kleine Race-Conditions)
---------------------------------------------------------

Ort:

- `src/teensy/SensorManager.cpp` — minimaler ISR `tslIntIsr()` und `tick()`

Befund & Fixes:

- `tslIntIsr()` setzt `tslIntPending_` und `tslIntCapturedMs_` (volatile). In `tick()`
  wird `tslIntPending_` zurückgesetzt und `tslIntCapturedMs_` gelesen. Es besteht ein kleiner
  Race, bei dem ein neuer Interrupt zwischen `tslIntPending_ = false;` und der Lese-Operation
  auftritt, was zu doppelter Verarbeitung oder zweimaligem Timestamp-Handling führen kann.

Empfehlung:

- Verwende eine kurze atomic snapshot Sequenz oder temporär `noInterrupts()`/`interrupts()` um
  `tslIntPending_` & `tslIntCapturedMs_` konsistent zu konsumieren:

```cpp
if (tslIntPending_) {
    noInterrupts();
    const uint32_t ts = tslIntCapturedMs_;
    tslIntPending_ = false;
    interrupts();
    lastTslIntMs_ = nowMs;
    pollLocalTsl2561(ts);
}
```

- Das ist eine kleine, sichere Änderung, reduziert Double-Handling und macht Timings reproduzierbarer.


6) ExposureEngine.observeSensorStatus verändert Telemetrie auch im `FAULT`
---------------------------------------------------------------------

Ort:

- `src/teensy/ExposureEngine.cpp` — `observeSensorStatus()` und `updateThermalRuntime()`

Befund:

- `observeSensorStatus()` ruft `updateThermalRuntime()` auch dann, wenn Engine bereits im
  `FAULT`-Zustand ist. Das führt dazu, dass latched Fault-Informationen (z. B. `runtimeOutputLimit`)
  nachträglich verändert werden, obwohl `FAULT` eine latched Aussage sein sollte.

Empfehlung:

- Guard in `observeSensorStatus()`:

```cpp
if (state_.phase == ExposurePhase::Fault) {
    // nur minimale Telemetrie akzeptieren oder ganz return;
    return;
}
```

oder: stelle sicher, dass `updateThermalRuntime()` keine latched/Fehler-relevanten Felder überschreibt.


## Zukunftsfähigkeit / Drei-Schichten-Modell

- Status: Die drei Modellschichten (`HeadSpectrumCommand`, `HeadCalibrationProfile`, `HeadLightCommand`)
  sind als Datentypen vorhanden, werden aktuell aber **nicht** durchgängig genutzt. `main.cpp`
  hat ein pragmatisches `resolveExposureBaseColor()` mit hardcodierten Farben; das umgeht
  die fachliche Schicht. Die notwendige Infrastruktur existiert, ist jedoch nicht verdrahtet.

Empfehlung:

- Implementiere eine schmale Mapping-Funktion `HeadSpectrumCommand -> HeadLightCommand` und ersetze
  die aktuelle `resolveExposureBaseColor()`-Bypass-Logik Schritt-für-Schritt. Begleitend:
  - Regressionstests für Kette `Spectrum -> Calibration -> Drive` (Unit + Hardware-in-the-loop),
  - Coverage-Messungen, die zeigen, dass jedes Feld des `HeadSpectrumCommand` genutzt wird.


## Querschnittsgrenze fuer EV-/F-Stop-Logik und Messwerte

- `ExposureEngine` bleibt intern lux-, dosis- und zeitbasiert.
- Fotografische EV-/F-Stop-Logik fuer die Bedienung mehrerer Belichtungsmodi gehoert in eine gemeinsame Schicht oberhalb der Engine.
- Messwerte aus echten Messworkflows duerfen zusaetzlich in EV dargestellt werden, technische Telemetrie dieser Doku bleibt jedoch bewusst in Lux, ms oder Zustandswerten.
- Diese Ableitung darf nicht pro Modus oder pro View erneut implementiert werden.


## Priorisierte ToDo-Liste (konkret)

1. (CRITICAL) `SensorManager`: setze I2C-Timeout; überprüfe alle Wire-Aufrufe, vermeide blocking.
2. (CRITICAL) `EspServiceLink`: implementiere `.tmp`-Staging + `rename()` für VFS und gate VFS-Operationen
   während `realtimeHold` (reject mit `Busy`).
3. (HIGH) Konsolidiere Head-Fade- und Latency-Konstanten in `HeadTiming.h` und passe Engine-Führung an.
4. (MEDIUM) ISR snapshot-Atomic: sichere `tslIntPending_`/`tslIntCapturedMs_` Konsum.
5. (MEDIUM) Harden `observeSensorStatus()` gegen Mutation latched FAULT-Telemetrie.
6. (LOW) Verdrahte `HeadSpectrumCommand` komplett durch den Pfad und ergänze Unit-Tests.


## Tests & Verifikation

- Simuliere I2C-Hänger: physisch oder per Test-Dummy; verifiziere, dass innerhalb
  100 ms ein Fehler in `SensorManager` markiert wird und `ExposureEngine` deterministisch
  in `FAULT` geht (kein weitere light output).
- Simuliere große VFS-Upload während `EXPOSING`: verifiziere, dass Uploads abgelehnt
  bzw. in `.tmp` geschrieben werden und kein Blocking im Main-Loop auftritt.
- Messung: End-to-End-Dosisabweichung bei maximalem Lux mit aktuellem Fade (Vorher/Nachher).


## Fazit (hartes Urteil)

- Architekturprinzipien sind grundsätzlich korrekt und dreischichtiges Modell ist vorhanden.
- Aktueller Implementierungsstand verletzt jedoch mehrere sicherheitskritische Regeln: I2C-Blocking
  und ungeprüfte VFS-/SD-Operationen während `EXPOSING` sind kritische Landminen — **vor**
  jeglichen Real-Belichtungstests zu beheben.
- Priorität: I2C-Timeout & atomarer VFS-Upload + Hold-Gating sofort implementieren, dann Fade-/Latency-
  Abstimmung und kleine ISR-Härtungen.

Bei Bedarf kann ich jetzt die vorgeschlagenen Code-Patches (kleine, gezielte apply_patch-Patches)
erstellen und Testscripte (host-side) anlegen, um die Fixes zu verifizieren.

---

Appendix: Kurze Zitat-Exzerpte (für schnellen Patch)

```cpp
// SensorManager.cpp (vereinfachtes Exzerpt)
bool SensorManager::writeTslRegister(uint8_t reg, uint8_t value) {
    tslBus_->beginTransmission(tslAddress_);
    tslBus_->write(static_cast<uint8_t>(kTslCommandBit | reg));
    tslBus_->write(value);
    return tslBus_->endTransmission() == 0; // <-- riskant ohne timeout
}

// EspServiceLink.cpp (kritische Stelle)
if (!activeWriteFile_.open(requestedPath, O_WRONLY | O_CREAT | O_TRUNC)) {
    // <-- direktes O_TRUNC auf Zielpfad
}

// ExposureEngine.h (konstante Diskrepanz)
static constexpr uint32_t kPredictiveShutoffLeadMs = 8;
static constexpr uint32_t kHeadSoftStopEquivalentLatencyMs = 32; // -> 40ms

// NeoPixelHead.h
static constexpr uint16_t kSoftStopMs = 64; // tatsächlicher Fade
```

Datei erstellt von Audit-Tool; Änderungen am Quellcode sind nicht vorgenommen — nur dokumentiert.
