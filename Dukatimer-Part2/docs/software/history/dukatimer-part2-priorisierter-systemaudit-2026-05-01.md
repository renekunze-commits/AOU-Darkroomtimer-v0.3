# Dukatimer-Part2 priorisierter Systemaudit 2026-05-01

## Scope

Audit-Fokus war der aktuelle `Dukatimer-Part2` Stand mit Teensy 4.1, ESP32-S3 Service-Gateway und Wireless-TSL2591/C6-Terminal. Historische Trees wurden nur als Referenz gelesen. Bearbeitete bzw. gehaertete Slices aus dem Audit sind in separaten Delta-Eintraegen dokumentiert; dieser Bericht fasst den Restzustand priorisiert zusammen.

## Kurzfazit

- Die zentrale Belichtungs- und SG-Mathematik ist inzwischen deutlich besser an das PaperProfile-Modell angebunden: 0,5er-SG-LUT, ISO-P-Skalierung und ein begrenzter ISO-R-Bias sind implementiert und build-validiert.
- Die wichtigsten Safety-/Robustheitskanten im Teensy-Laufzeitpfad wurden bereits gehaertet: Stale-Wait-Recovery, latenzbewusste Head-Abschaltung, stabile SG-0-Phasen und Drift-resistente manuelle Split-Edits.
- Der Protokollpfad ist strukturell sauberer als zu Beginn: SharedProtocol ist zwischen Part2 und C6 zeilenidentisch, ESP-NOW Payloadgroessen sind per `static_assert` abgesichert, VFS arbeitet transaktional.
- Offen bleiben vor allem Remote-Terminal-Semantik, Teensy-seitige UART-Blockierfreiheit, Hardwarevalidierung und ein echter UI-Layout-Slice.

## Prioritaeten

| Prio | Thema | Risiko | Empfehlung |
| --- | --- | --- | --- |
| P0 | Keine neue offene Safety-Blockade im aktuell geprueften Stand | Die bekannten P0-Landminen wurden im laufenden Audit gehaertet oder sind als Hardwarevalidierung offen. | Keine neue P0-Codeaenderung vorziehen; Hardwaretests fuer Exposure/Head/Wait-Fault als naechsten P0-Nachweis planen. |
| P1 | Remote-Kommandos erreichen das C6-Terminal nicht fachlich | Teensy sendet `RemoteMeasurementStart`, `RemoteMeasurementCancel` und `RemoteHaptic`; `WirelessRemoteGateway::applyTeensyCommand()` konsumiert sie derzeit aber ohne ESP-NOW-Weiterleitung. Remote-Display und Lux-Rueckweg funktionieren, explizite Haptik/Messkommandos sind noch Scheinfunktion. | ESP-NOW-Command-Payload oder erweiterte `RemoteDisplayPayload`-Semantik definieren und C6-seitig auswerten. |
| P1 | C6 ignoriert `RemoteDisplayPayload.sequenceNumber` | Aeltere ESP-NOW-Renderpakete koennen ein neueres Displaybild ueberschreiben. | C6-seitig letzte Display-Sequenz merken und nur neuere Sequenzen akzeptieren; Wraparound wie Gateway-Eingang behandeln. |
| P1 | Teensy-UART-Sends koennen noch blockieren | ESP32-Seite prueft `availableForWrite()` vor Frame-Sends. Teensy-Seite nutzt in Heartbeat, RemoteRender, TeensyCommand und VFS-Ack/Error noch direkte `serial_.write(...)`. Bei CTS/Backpressure kann das den Loop belasten. | Teensy `EspServiceLink` analog ESP-Seite auf `tryWriteFrame()` mit Retry/Drop-Policy umstellen; RemoteRender darf coalescen, VFS-Ack/Error brauchen klare Retry- oder Fehlersemantik. |
| P1 | ISO-R-Modell ist fachlich plausibilisiert, aber nicht nass/hardwarekalibriert | `isoP` folgt historisch belegter CHD-Skalierung. `isoR` wirkt bewusst begrenzt als Split-Bias, ist aber noch kein gemessener Papiercharakteristik-Fit. | Testprints mit mindestens zwei Papieren und G0/G2.5/G5 fahren; ISO-R-Bias gegen Dichte-/Kontrastziel pruefen und ggf. durch kalibrierte LUT bevorzugen. |
| P1 | Hardwarevalidierung fehlt fuer neue Laufzeit-Haertungen | Builds sind gruen, aber Head-Latenz, Stale-Wait-Faults, Remote-Input-Ownership und SG-0-Phasen sind noch nicht am realen Geraet provoziert worden. | Kurzen Abnahmelauf definieren: Exposure Start/Abort, Sensorverlust, Pre/PostWait-Stale, SG soft-only/hard-only, Wireless stale/online. |
| P2 | SharedProtocol wird im C6-Projekt lokal kopiert | Header sind aktuell zeilenidentisch, aber Drift bleibt organisatorisch moeglich. | Entweder gemeinsame Include-Quelle in Workspace/Build einbinden oder einen kleinen Vergleichscheck als Build-/Review-Schritt dokumentieren. |
| P2 | LVGL-UI ist noch statisches Diagnose-Layout | `LvglUi` stapelt Labels mit festen Abstaenden und verankert das Overlay unten. Bei langen Diagnose-/Gatewayzeilen koennen Inhalte verdeckt oder ausserhalb des Screens liegen. | AP-UI-Slice fuer echte Layoutstruktur: getrennte Hauptzone, Status-/Overlayzone, Scroll/Tab oder komprimierte Diagnoseansicht. |
| P2 | LVGL-Flush-API enthaelt Async-Reste ohne Async-Pfad | `handleDisplayFlushComplete()` ist deklariert, `handleDisplayFlushCompleteImpl()` existiert, aber der aktive Flush ruft `writeRect()` synchron und sofort `lv_disp_flush_ready()`. | Entweder Async-Pfad voll verdrahten oder tote Async-Reste entfernen; Timingtelemetrie fuer reale Flushdauer behalten. |

## Bereits gehaertete Punkte

### SG-Mathematik

- `SplitgradeWorkflow` nutzt das aktive `PaperExposureProfile` als Quelle fuer SG-Startwerte.
- Die 0,5er-LUT ist eine feste 11-Stufen-Struktur und wird ueber den normalisierten Gradationsindex adressiert.
- Manuelle Soft-/Hard-Edits werden als fixed-point Soft-Anteil pro Gradationsindex stabilisiert.
- `useIsoMath` nutzt `isoP` fuer die CHD-Basis `100 / isoP` und `isoR` als begrenzten Split-Bias; ungueltige Werte fallen auf P/R 100 zurueck.
- AP-07-Harness deckt 0,5er-LUT, ISO-P, ISO-R, Fallbacks, 0-Phasen und Drift-Roundtrips ab; Syntaxcheck mit ARM-Compiler und `teensy41` Build waren erfolgreich.

### Robustheit und Safety

- Exposure-Stale-Waits werden nicht still entsperrt, sondern in einen sichtbaren Fault ueberfuehrt.
- `ExposureRuntimeState` exportiert `phaseAgeMs` fuer Diagnose und Guard-Entscheidungen.
- Head-Present-Dauer fliesst in die predictive shutoff lead time ein.
- SG soft-only und hard-only laufen ohne versteckte WaitForFilter-Phase.
- PaperSlot-Load unterscheidet FileNotFound von korrupten/inkompatiblen Daten und ueberschreibt fragliche Nutzdaten nicht still.

### Protokoll und Gateway

- `DukatimerProtocol.h` ist in Part2 und Wireless TSL2591 aktuell zeilenidentisch.
- `WirelessRemotePayload` ist auf 11 Byte, `RemoteDisplayPayload` auf 78 Byte per `static_assert` fixiert.
- UART-Frames sind versioniert, typisiert und CRC16-gesichert.
- ESP32-Link sendet nicht blockierend ueber `availableForWrite()` und haelt Render als latest-state/coalescing.
- VFS-Uploadpfad ist transaktional: absoluter Pfad, keine `..`, Transaction-ID, strikte Offsets, temp-Datei, atomarer Rename.

## Naechster sinnvoller Arbeitsschnitt

1. P1 Remote-Terminal-Kommandos schliessen: Haptic und MeasurementStart/Cancel wirklich bis zum C6 transportieren und dort sichtbar/hoerbar machen.
2. Direkt im selben Slice: C6 `RemoteDisplayPayload.sequenceNumber` validieren, damit Render-Reorder nicht mehr sichtbar wird.
3. Danach Teensy-`EspServiceLink` auf nicht-blockierende TX-Policy bringen.
4. Erst nach diesen Protokollkanten den UI-Layout-Slice angehen, weil die UI sonst weiterhin Symptome eines unvollstaendigen Remote-Pfads zeigt.

## Validierungsstand

- `pio run -e teensy41`: zuletzt erfolgreich nach SG-Math-Haertung.
- AP-07-Harness: syntax-only mit PlatformIO-ARM-Compiler erfolgreich; Host-Ausfuehrung offen, weil kein `g++`/`clang++` im PATH vorhanden ist.
- ESP32-S3 und C6 Builds waren im vorherigen Protokollslice erfolgreich; nach diesem Bericht wurden keine Firmwaredateien geaendert.
- Hardware-in-the-loop ist fuer die P1-Abnahme noch offen.
