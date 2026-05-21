---
title: Dukatimer-Part2 Priorisierte TODO-Liste
date: 2026-04-29
author: GitHub Copilot
---

## Dukatimer-Part2 Priorisierte TODO-Liste

Kurz: Diese Liste konsolidiert die offenen Punkte aus den vorhandenen
Markdown-Dokumenten zu einer belastbaren Reihenfolge fuer die naechsten Schritte.
Sie ersetzt keine Einzelanalysen, sondern ueberfuehrt sie in eine umsetzbare
Priorisierung.

## Quellenbasis

- `docs/software/dukatimer-part2-migrations-backlog.md`
- `docs/software/dukatimer-part2-step-by-step-abarbeitung-2026-04-28.md`
- `docs/software/erledigt/dukatimer-part2-statusaudit-ui-grundgeruest-2026-04-24.md`
- `docs/software/erledigt/dukatimer-part2-architekturzugriffspunkte-math-sensorik-und-workflows.md`
- `docs/software/erledigt/dukatimer-part2-audit-exposureengine-findings.md`
- `docs/software/erledigt/dukatimer-part2-laufzeitspezifikation-exposureengine-closed-loop-und-fail-safe.md`
- `docs/head_timing_and_i2c_timeout.md`
- `docs/hardware/duka-teen-electrical-risk-analysis.md`
- `docs/hardware/duka-teen-power-rail-capacitor-analysis.md`

## Bereinigungen gegen aeltere Einzelbefunde

Die folgende Priorisierung beruecksichtigt, dass einige aeltere TODOs durch
neuere Dokumente bereits teilweise ueberholt sind:

- Der lokale TSL2561-Dosispfad ist nicht mehr offen, sondern implementiert.
  Offen sind Feldvalidierung, Kalibrierung und Fehlertests.
- Die Konsolidierung der Head-Timing-Konstanten ist laut
  `docs/head_timing_and_i2c_timeout.md` bereits erfolgt.
  Offen bleiben Telemetrie und Hardwarevalidierung.
- Der historische Sensorverlust ohne Fail-Safe war eine Landmine. Der aktuelle
  Restpunkt ist jetzt nicht mehr die reine Implementierung, sondern die
  End-to-End-Validierung und die globale UI-Fuehrung ausserhalb des SG-Overlays.
- Die Forderung nach einem einfachen `Wire.setTimeout()` ist fuer Teensy nicht
  mehr als allgemeine Hauptloesung zu lesen. Fuer diesen Pfad ist stattdessen
  eine belastbare Bus-Hang-Strategie inklusive Hardware-/Watchdog-Validierung
  relevant.

## P0 - vor realen Belichtungs- und Feldtests abschliessen

### 1. Sicherheits- und Realtime-Haertung der aktiven Belichtung

Warum zuerst:
Die vorhandenen Audits werten Blocking- oder unkontrollierte Laufzeitpfade in
der Belichtung weiterhin als kritisch. Das ist vor jedem realen Belichtungstest
wichtiger als neuer Funktionsausbau.

Enthaelt:

- VFS-Schreibpfad robust machen: `.tmp`-Staging, `rename()`-Commit und klares
  Reject/Bussy-Verhalten waehrend `realtimeHold`.
- Teensy-I2C-Bus-Hang-Fall nicht als normales Timeout-Problem behandeln,
  sondern als eigene Ausfallklasse mit validierter Rueckfallebene.
- End-to-End-Fail-Safe-Tests fuer Sensorverlust, Null-Lux-Plausibilitaet,
  Thermik und parallelen VFS-Traffic definieren und durchfuehren.
- UI-seitig sicherstellen, dass Fehler- und Fail-Safe-Zustaende nicht nur im
  SG-Overlay, sondern als belastbare Bedienfuehrung sichtbar bleiben.

Abnahmekern:

- Keine SD-/VFS-Operation blockiert waehrend aktiver Belichtung den Schutzpfad.
- Sensor-, Thermik- und Kommunikationsfehler fuehren nachvollziehbar in einen
  sicheren Zustand oder in den dokumentierten Fallback.
- Testfaelle und Messergebnisse sind dokumentiert, nicht nur vermutet.

### 2. Zentrale EV-/F-Stop- und Messwertbasis einziehen

Warum jetzt:
Mehrere Dokumente markieren dies als eigentlichen Schutz gegen neue
Zerklueftung. Ohne diese Basisschicht wuerden SG, BW, Burn, Teststrip,
Messsession und spaetere Modi wieder eigene Mathematik und Formatter bauen.

Enthaelt:

- `ExposureValueMath` als einzige Quelle fuer EV-, F-Stop- und
  Schrittlogik einfuehren.
- `MeasurementValueFormatter` als einzige Quelle fuer EV-/Lux-Ausgabe
  einfuehren.
- Direkte Workflow- und UI-Stringbildung schrittweise aus `LvglUi` und
  modusspezifischen Pfaden herausziehen.
- Erste feste Zugriffsschicht fuer Workflow-Services, Measurement-Domain und
  Presenter vorbereiten.

Abnahmekern:

- SG nutzt keine lokale Sondermathematik mehr.
- Weitere Modi koennen dieselbe EV-/F-Stop-Basis direkt konsumieren.
- EV plus Lux ist fuer fotografisch sinnvolle Messwerte zentral definiert.

### 3. Papierslots, Persistenz und Migrationsstrategie vollenden

Warum jetzt:
Der dokumentierte erste fachliche Slice bleibt ohne belastbaren Papier- und
Profiltraeger unvollstaendig. Kalibrierung, SG-Vorschlaege und reproduzierbare
Workflows haengen daran.

Enthaelt:

- Restintegration der Slot-Bank in die Laufzeitpfade.
- Version/CRC-, Migrations- und Fehlerstrategie dokumentiert abschliessen.
- Minimalen Testleitfaden fuer Leerdaten, defekte Daten, Migration und
  Rueckschreiben fertigstellen.
- Persistenzfehler im UI und in der Diagnose sauber sichtbar machen.

Abnahmekern:

- Gueltige Daten bleiben ueber Boot und Updates stabil.
- Fehlerhafte Daten werden kontrolliert erkannt und nicht blind uebernommen.
- MB-07 und MB-08 koennen auf einem echten persistenten Unterbau aufsetzen.

### 4. Debug-UI in globale Bedienfuehrung ueberfuehren

Warum jetzt:
Die Dokumente sind konsistent: Der heutige Stand ist fuer Blindbedienung,
Dialogfokus und modale Fehlerfuehrung noch nicht belastbar genug.

Enthaelt:

- SG-Hauptscreen zum echten Bedienkern machen statt nur zum erweiterten
  Debug-Snapshot.
- Globales Fehler-/Confirm-/Wait-Modal statt SG-spezifischem Sonderoverlay.
- Touch-/Encoder-Fokusmodell, globalen EventGuard und klare Dialogregeln
  festziehen.
- Blind-UI-Definition of Done festschreiben: feste Encoderrollen,
  bestaetigte kritische Aktionen, eindeutige Zustandscues, Undo wo noetig.

Abnahmekern:

- Modale Fehler- und Confirm-Zustaende sind eindeutig und konsistent.
- Schnelle Bedienung erzeugt keine stillen Fokus- oder Routingfehler.
- Die UI ist nicht mehr nur Beobachtungsflaeche, sondern belastbare Bedienebene.

## P1 - naechster Integrations- und Fachsprint

### 5. SG-Mathematik und papiergetriebene Vorschlagslogik fertigziehen

- SG-Targets auf die neue zentrale EV-/F-Stop-Basis umstellen.
- Papierprofile und Slots in Vorschlagsbildung, Startwerte und Rueckschreiben
  einbeziehen.
- Execution-States, WaitForFilter und Hard/Soft-Ablauf fachlich zu Ende
  modellieren.

### 6. Papierkalibrierungs-Wizard aufsetzen

- Wizard-Flow, Datenhaltung und Rueckschreibelogik implementieren.
- Messwerte konsistent als EV plus Lux darstellen.
- Kalibrierergebnis direkt in Papierslots und SG-Vorschlaege integrieren.

### 7. Remote- und Servicepfade end-to-end verdrahten

- TeensyCommand-, Remote-Render- und Messkommando-Pfade fachlich schliessen.
- Reale Remote-Messintegration in den SG-Workflow einziehen.
- ESP-Servicepfade fuer 1-Wire- und Zusatzsensorik inklusive Fehlerpfad
  produktiv machen.

### 8. Input- und Workflow-Skalierung absichern

- Quadraturvalidierten Encoderpfad mit Lost-Step-Monitoring einfuehren.
- ModeCoordinator/Workflow-Zugriffe von harten Spezialverdrahtungen loesen.
- Feste Service-Injektion und erweiterbare Workflow-Ports vorbereiten.

## P2 - fachlicher Ausbau nach stabiler Kernbasis

### 9. Messpipeline und Histogramm-Session

- Spot-, Multi-Spot- und Histogramm-Session aufbauen.
- Undo-Logik und Messhistorie in einer zentralen Measurement-Schicht fuehren.
- Formatter und Presenter statt UI-Sonderpfaden verwenden.

### 10. BW, Burn und Teststrip

- BW zuerst produktiv auf derselben EV-/F-Stop-Basis umsetzen.
- Burn und Teststrip anschliessen, ohne eigene Mathematikinseln zu erzeugen.

### 11. Densitometer, Filmtest, Preflash und Flash-Kalibrierung

- REF/BASE/MEAS-Flow und Zone-I/VIII-Helfer einziehen.
- Preflash- und Flash-Kalibrierpfade auf derselben Basis aufbauen.

### 12. LiveView, Zone und Wireless-Komfortfunktionen

- Erst nach stabiler Kernfunktion und gemeinsamer Mathematik-/Formatter-Basis.
- Nur noch UI-spezifisch auspraegen, nicht erneut fachlich zerspalten.

## Paralleler Hardware-Track fuer Robustheit und naechste Revision

Diese Punkte sind nicht identisch mit dem naechsten Firmware-Slice, bleiben aber
laut Hardware-Dokumenten hoch relevant fuer robuste Feldhardware:

1. ESP-seitigen lokalen 5-V-Puffer direkt am DevKit-Eingang vorsehen.
2. Rolle von `+3.3V` im Schaltplan eindeutig machen.
3. `S3_EN_RST` und `S3_GPIO0` servicetauglich von direkter Push-Pull-Kopplung loesen.
4. 5-V-Verteilung in lokal gepufferte, rueckspeisesichere Zweige aufteilen.
5. Externe Kopf-/NeoPixel-/SSR-Anschluesse mit ESD-/Transientenschutz absichern.
6. Worst-Case-Last, Hot-Plug und Einschaltverhalten real vermessen.

## Empfohlene direkte Arbeitsreihenfolge

1. Realtime-/Sicherheits-Haertung und dokumentierte Fail-Safe-Validierung.
2. Zentrale EV-/F-Stop- und Measurement-Basis.
3. Papierslot-/Persistenz-Abschluss.
4. Globale UI-/Dialog-/Fokusfuehrung.
5. SG-Mathematik und Papierkalibrierung.
6. Remote- und Serviceintegration.
7. Messpipeline und weitere fotografische Modi.

## Kurzfazit

Die staerkste dokumentierte Gefahr liegt aktuell nicht mehr im reinen
Grundgeruest, sondern in zwei offenen Fronten:

- Sicherheits- und Realtime-Haertung vor echten Belichtungstests.
- Verhinderung neuer fachlicher Zerklueftung durch eine zentrale EV-/F-Stop-
  und Messwertbasis.

Wenn diese beiden Fronten zuerst geschlossen werden, koennen SG, Kalibrierung,
Messsession und weitere Modi auf einer belastbaren Basis wachsen statt erneut
in dokumentierte Sonderpfade auseinanderzulaufen.

## Delta-Update 2026-04-29 (P0 Step 1 umgesetzt)

Abgeschlossener Schritt:

- P0 Punkt 2 gestartet und im ersten technischen Slice umgesetzt:
  zentrale Einfuehrung von `ExposureValueMath` und `MeasurementValueFormatter`.

Konkreter Stand:

- SG-EV-Schrittskalierung wird nicht mehr lokal ueber `powf` im Workflow gerechnet,
  sondern ueber `ExposureValueMath`.
- Die SG-Dosis/Lux-Laufzeitzeile wird nicht mehr direkt in `LvglUi` formatiert,
  sondern ueber `MeasurementValueFormatter`.
- Semantik und Zahlenformat wurden bewusst unveraendert gehalten (keine
  Verhaltensaenderung).

Validierung:

- statische Fehlerpruefung der betroffenen Dateien ohne Befund
- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-04-29 (P0 Step 2 umgesetzt)

Abgeschlossener Schritt:

- P0 Punkt 3 im Dokumentations-/Diagnose-Slice weiter umgesetzt
  (MB-06 Restintegration ohne Logikaenderung).

Konkreter Stand:

- Minimaler Persistenz-Testleitfaden erstellt fuer:
  - Leerdaten / fehlende Datei
  - defekte Daten (Blob/Header/CRC/Version)
  - sichtbaren Schreibfehlerpfad
  - Reboot-Stabilitaet gueltiger Daten
- Persistenzstatus im Runtime-Snapshot sichtbar gemacht.
- Debuganzeige erweitert um `PS e:<code> d:<detail>`.

Validierung:

- statische Fehlerpruefung der betroffenen Dateien ohne Befund
- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-04-30 (P1 Step 7 umgesetzt)

Abgeschlossener Schritt:

- P1 Punkt 7 wurde im produktiven Slice umgesetzt:
  Remote- und Servicepfade sind Ende-zu-Ende verdrahtet.

Konkreter Stand:

- TeensyCommand- und Remote-Render-Pfade laufen produktiv zwischen Teensy und ESP.
- Wireless-TSL2591-Remote ist als echter ESP-NOW-Controller/Meter integriert.
- Reale DS18B20- und AHT-Servicepfade laufen auf ESP-Seite inklusive Fehlerflags.
- Diagnosepfad wurde bis in den sichtbaren Gatewaystatus durchgezogen.
- Das historische C6-Event-ABI ist im Gateway auf 16 Byte abgeglichen.

Validierung:

- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (Splitgrade-UX und Histogramm-Dunkelbereich)

Abgeschlossener Schritt:

- Die offene Splitgrade-UX-Landmine und das Dunkelbereich-Clipping des
  Measurement-Histogramms sind im Code entschärft.

Konkreter Stand:

- `SplitgradeWorkflow` normalisiert die sichtbare Gradation jetzt strikt auf
  0,5er Schritte.
  - Primär- und Sekundärencoder im Grade-Panel laufen beide auf derselben
    0,5er Semantik.
  - Off-Grid-Werte wie `2.1` oder `2.3` werden intern nicht mehr aufgebaut.
  - Damit verschwinden Ghost-UI-Zustände, bei denen Anzeige und Lichtausgabe
    auseinanderlaufen.
- `MeasurementDomainService` behält die temporäre `log2(lux)`-Heuristik bei,
  legt den provisorischen Referenzpunkt aber in die Mitte des 11er
  Zonensystems.
  - Wireless-Messwerte unterhalb von `1 Lux` spreizen sich dadurch ueber
    mehrere dunkle Buckets statt komplett auf Zone `0` zu kollabieren.
  - Die absolute, kalibrierte Lux->Zone-Politik bleibt weiterhin ein separater
    spaeterer Fachschritt.

Validierung:

- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (Remote-Input Backpressure entschaerft)

Abgeschlossener Schritt:

- Der teilweise noch reale Remote-Input-Landminenpfad bei lokaler Ownership-Sperre
  ist weiter eingegrenzt worden.

Konkreter Stand:

- `EspServiceLink` behaelt den Teensy-seitigen Remote-Input weiter als FIFO,
  verdichtet jetzt aber gleichgerichtete Rotationsbursts desselben Remote-
  Encoders bereits im Queue-Puffer.
- Beim Consume werden diese verdichteten Bursts wieder als einzelne
  Drehschritte ausgegeben.
- Dadurch bleibt die bestehende `DeferredRemoteOwnership`-Logik unveraendert,
  waehrend die Wahrscheinlichkeit sinkt, dass eine kurze lokale Bedienphase den
  kleinen FIFO mit redundanten Einzelschritten volllaufen laesst.

Ergaenzender Abschluss dieses Pfads:

- Der fruehere kleine FIFO wurde auf beiden Linkhaelften durch feste,
  per-Source begrenzte Pending-Buckets ersetzt. Hohe Remote-Eventraten fuehren
  damit nicht mehr in einen linearen Queue-Wachstumspfad.
- Die Hauptloop-Arbitrierung besitzt jetzt eine explizite Fairnesspolitik:
  lokale Eingaben bleiben priorisiert, entfernte Eingaben werden nach Ablauf
  der lokalen Ownership-Sperre nur noch in begrenztem Budget pro Loop und
  source-fair ueber Round-Robin nachgezogen.

Validierung:

- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (P1 Punkt 7 AP-09 gehaertet)

Abgeschlossener Schritt:

- Der Remote-/ESP-Servicepfad wurde an zwei konkreten Landminen gehaertet:
  explizite Backpressure-Strategie im ESP-Link und zentrale Arbitrierung
  zwischen lokaler und entfernter Bedienung.

Konkreter Stand:

- `TeensyLinkService` auf ESP-Seite schreibt SharedProtocol-Frames nicht mehr
  blind in den UART, sondern nur noch, wenn im TX-Puffer genug Platz ist.
  Damit bleibt der Linkpfad nicht-blockierend, auch wenn Gegenstelle oder
  Downstream kurzzeitig stauen.
- Renderdaten sind jetzt fachlich explizit als latest-state-Pfad definiert:
  ein noch nicht konsumierter Remote-Render wird mit dem juengsten Frame
  ersetzt, statt den Servicepfad zu blockieren. Solche Coalescing-Faelle werden
  ueber `LINK-R` sichtbar diagnostiziert.
- Teensy-Kommandos laufen auf ESP-Seite nicht mehr als stilles Single-Slot-
  Override, sondern ueber eine kleine FIFO. Erst bei echter Queue-Ueberlast
  wird ein neues Kommando verworfen und als `LINK-C` diagnostiziert.
- Remote-Input-Events laufen ebenfalls ueber eine kleine FIFO statt ueber ein
  stilles Last-Writer-Wins-Feld. Echte Ueberlaeufe werden als `LINK-I`
  diagnostiziert.
- Die AP-06-Policy unterscheidet jetzt zusaetzlich lokale gegen entfernte
  Bedienhoheit: Nach lokaler Encoder-/Start-Aktivitaet blockt der Router fuer
  `350 ms` entfernte Steuerereignisse. Lokale Eingaben duerfen damit eine
  laufende Remote-Interaktion jederzeit uebernehmen, waehrend Remote nicht mehr
  in lokale Bedienung hineinraced.
- Diese Bedienhoheit ist in der Mode-Diagnose sichtbar als `O LOCAL/...` bzw.
  `O REMOTE/...`.

Noch bewusst nicht Teil dieses Slices:

- tieferes Prioritaetsmodell fuer mehrere unterschiedliche Remote-Quellen
- Hardware-in-the-loop-Nachweis mit provozierter UART-/ESP-NOW-Stauung

Validierung:

- Sprachserver-Fehlerpruefung fuer die AP-09-betroffenen Dateien: ohne Befund
- `pio run -e esp32s3_n16r8`: erfolgreich
- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (Stabilisierungs-Follow-up: Timing, Persistenz, Hardware-Sync)

Abgeschlossener Schritt:

- Drei direkte Folgemaßnahmen aus dem Stabilisierungsfazit wurden umgesetzt:
  laufende Head-Latenz-Telemetrie aktiviert, Persistenz-Load gegen transiente
  SD-/Blob-Fehler gehaertet und die ESP-Service-Pinbelegung zentralisiert.

Konkreter Stand:

- Der Head-Timing-Serial-Report ist jetzt standardmaessig aktiv und zeigt nicht
  nur `last/avg/max`, sondern auch die laufzeitgemessene Head-Bus-Latenz und
  den aktuellen predictive shutoff lead.
- Wichtige Einordnung: Der Bootpfad ueberschreibt `InvalidBlob` bereits seit
  AP-05 nicht mehr still mit Defaults. Die neue Härtung setzt deshalb am echten
  Root Cause an: `PaperSlotStorage::load()` versucht denselben Blob bei
  `ReadFailed` oder `InvalidBlob` jetzt bis zu drei Mal neu zu lesen/parsen,
  bevor ein Fehler sichtbar gemeldet wird.
- Damit fuehrt ein einzelner transienter SD-Lesefehler nicht mehr sofort zu
  einer sichtbaren `InvalidBlob`-/`ReadFailed`-Lage im Startup.
- Neben `src/teensy/DukaTeenBoardPins.h` existiert jetzt auch
  `src/esp32/DukaEspServiceBoardPins.h` als zentrale Software-Spiegelung der
  dokumentierten ESP-Service-Pins fuer UART, 1-Wire, AHT-I2C und die bereits im
  Schaltplan belegten ENC4-Leitungen.
- Der aktuell verifizierte Stand ist damit:
  - Teensy-seitige Produktionspins leben zentral in `DukaTeenBoardPins.h`
  - ESP-seitige Servicepins leben zentral in `DukaEspServiceBoardPins.h`
  - `SensorManager` nutzt weiterhin `Wire1` fuer den dokumentierten Kopf-Bus
    `I2C1_SCL/I2C1_SDA`
  - ESP-AHT bleibt auf GPIO11/GPIO12, 1-Wire auf GPIO8, UART-Handshake auf
    GPIO4/GPIO5 entsprechend der Hardware-Uebersicht

Noch bewusst nicht Teil dieses Slices:

- automatischer Clamp der predictive-shutoff-Logik anhand echter Stress-Logs
- produktive lokale Verdrahtung von ENC4 als ESP-Hardwareencoder

Validierung:

- Sprachserver-Fehlerpruefung fuer Timing-, Persistenz- und ESP-Board-Pin-Dateien: ohne Befund
- `pio run -e teensy41`: erfolgreich
- `pio run -e esp32s3_n16r8`: erfolgreich

## Delta-Update 2026-05-01 (Technische Schulden und Integrationshygiene)

Abgeschlossener Schritt:

- Vier konkrete Debt-/Robustheitskanten wurden direkt im produktiven Code
  bereinigt: delete-first-Rename-Fallbacks, eingefrorene Head-Latenzmittelung,
  versteckter `/4`-Encoder-Teiler und handgerollte String-Kopierpfade.

Konkreter Stand:

- `PaperSlotStorage::save()` und der Teensy-seitige VFS-Finalisierungspfad in
  `EspServiceLink` loeschen die Zieldatei bei Rename-Problemen nicht mehr
  blind weg. Stattdessen wird eine vorhandene Zieldatei zuerst nach `.bak`
  verschoben, dann die `.tmp` promoted und bei Fehlern ein Restore versucht.
- `NeoPixelHead::present()` berechnet `averageDurationUs` nicht mehr ueber
  einen lebenslang anwachsenden Integer-Mittelwert, sondern ueber einen festen
  EMA-Pfad mit internem Q8-Akkumulator. Damit bleibt die Timing-Telemetrie auch
  nach sehr langer Laufzeit reaktionsfaehig.
- Die Encoder-Rohschritte pro Rastung sind in `main.cpp` jetzt explizit pro
  Encoder konfigurierbar, statt implizit als verstecktes `/4` in
  `EncoderState::begin()` zu leben.
- Die lokalen String-Kopierhelfer fuer Snapshot-/Persistenztexte verwenden nun
  begrenztes `snprintf(...)` statt eigener Zeichen-fuer-Zeichen-Loops.

Wichtige Einordnung / weiterhin offen:

- Der ESP32-HTTP-Uploadpfad bleibt architektonisch noch synchron: die
  `waitForUploadReady()`/`waitForUploadCompletion()`-Schleifen blockieren
  waehrend eines Upload-Requests weiter den ESP-Hauptpfad. Das ist bestaetigt,
  aber kein kleiner lokaler Patch mehr, sondern ein eigener asynchroner
  Upload-/Polling-Umbau.
- Der fehlende Hardware-Watchdog gegen hart blockierende Teensy-I2C-Aufrufe ist
  ebenfalls weiter offen. Hier reicht kein kosmetischer Code-Fix; benoetigt wird
  eine echte WDOG-Strategie samt Recovery-Politik.
- Die Singleton-/Thunk-Landmine ist im aktuellen Stand nur teilweise zutreffend:
  `WirelessRemoteGateway` und `SensorManager` nutzen weiterhin statische
  Callback-Bruecken, besitzen aber bereits `nullptr`-Guards; `ServiceSensorHub`
  verwendet in diesem Repo-Stand keinen vergleichbaren statischen IRQ-/ESP-NOW-
  Thunkpfad.
- Die Dosis-Praezisionslandmine ist nicht mehr aktuell: die interne
  Dosisakkumulation laeuft inzwischen bereits ueber einen `double`-Akkumulator.

Validierung:

- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (Hardware-Integration/Timing gehaertet)

Abgeschlossener Schritt:

- Der NeoPixel-Head-Timingpfad wurde an der konkreten Spike-Kante gehaertet;
  die aktive Teensy-Pinbelegung liegt jetzt in einer dedizierten Board-Datei
  statt verstreut in `main.cpp`.

Konkreter Stand:

- `syncHeadTimingIntoExposureEngine()` speist nicht mehr `maxDurationUs`,
  sondern bewusst das juengste `lastDurationUs` in die ExposureEngine ein.
- `ExposureEngine::observeHeadPresentDurationUs()` uebernimmt groessere
  Aufwaertsspruenge der Head-Latenz nicht mehr sofort, sondern erst nach
  bestaetigten Wiederholungen innerhalb eines kleinen Fensters.
- Kleine Aufwaertsbewegungen bleiben direkt erlaubt; Abwaerts wird weiterhin
  langsam geglaettet.
- Die produktive Teensy-Pinbelegung wurde nach
  `src/teensy/DukaTeenBoardPins.h` gezogen und dort explizit als
  Software-Spiegel der aktuellen Schaltplanuebersicht markiert.
- Wichtige Einordnung: Es wurde kein aktueller Pin-Mismatch erzwungen
  umverdrahtet. Der derzeitige Code stand bereits im Einklang mit der
  dokumentierten Pin-Tabelle; gehaertet wurde hier die Wartbarkeit gegen
  spaetere PCB-Revisionen.

Noch bewusst nicht Teil dieses Slices:

- Feldmessung der neuen Timing-Filterparameter auf echter Hardware
- sichtbare UI-Telemetrie fuer `runtimeHeadBusLatencyMs_`

Validierung:

- Sprachserver-Fehlerpruefung fuer `ExposureEngine.*`, `main.cpp` und
  `DukaTeenBoardPins.h`: ohne Befund
- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (P1 Punkt 5 AP-07 gehaertet)

Abgeschlossener Schritt:

- AP-07 wurde an zwei Landminen gehaertet:
  stabile Rueckschreibung manueller Split-Edits und explizite 0-Phasen-
  Uebergaenge in der Execution-State-Maschine.

Konkreter Stand:

- `SplitgradeWorkflow` schreibt manuelle Soft-/Hard-Edits nicht mehr nur als
  wiederholte Float-Normalisierung in die laufende LUT zurueck.
- Stattdessen wird pro aktuellem LUT-Gradationsschritt ein stabiler
  Soft-Anteil als fixed-point Override gehalten; daraus werden spaetere
  Rueckkehr-Splits deterministisch rekonstruiert.
- Die sichtbare Basisgroesse `baseTarget_` bleibt dabei der eine gemeinsame
  Gesamtwert, waehrend der aktuelle Split-Anteil nicht mehr bei jedem
  Gradations-Roundtrip neu aus Float-Zwischenwerten berechnet werden muss.
- Soft-only- und Hard-only-Faelle wurden im Workflow auf explizite Helfer
  fuer Start, Soft-Abschluss und Filter-Confirm umgezogen.
- Dadurch ist fachlich festgeschrieben:
  - Soft-only endet direkt in `Completed`
  - Hard-only startet direkt in `ArmingHard`
  - `WaitForFilter` existiert nur noch, wenn wirklich eine Hard-Phase folgt
- Der AP-07-Acceptance-Harness deckt jetzt zusaetzlich ab:
  - Done-Quittierung nach Soft-only/Hard-only statt versteckter zweiter Phase
  - wiederholte Gradations-Roundtrips ohne erkennbare Drift der editierten
    Soft-/Hard-Werte

Noch bewusst nicht Teil dieses Slices:

- hostseitig direkt lauffaehige AP-07-Testumgebung ueber `pio test`
- Hardware-Validierung der neuen Drift-Grenzen auf echter Belichtungszeit

Validierung:

- Sprachserver-Fehlerpruefung fuer `SplitgradeWorkflow.*` und
  `test/ap07_splitgrade_acceptance/test_main.cpp`: ohne Befund
- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (P0 Punkt 4 AP-06 gehaertet)

Abgeschlossener Schritt:

- P0 Punkt 4 wurde an der kritischen Deadlock-Kante gehaertet:
  Ein haengendes globales Wait-Modal bleibt nicht mehr unbegrenzt stehen.

Konkreter Stand:

- `ExposureRuntimeState` exportiert jetzt das sichtbare Phasenalter `phaseAgeMs`.
- Unmoeglich lange Wait-Phasen werden im Wiring-Layer nicht still nach `Ready`
  entsperrt, sondern kontrolliert in einen sichtbaren `InternalFault`
  ueberfuehrt.
- Abgedeckte Stale-Faelle:
  - `PreWait` laeuft unplausibel lang
  - `PostWait` laeuft unplausibel lang
  - `Paused` bleibt als derzeit nicht produktiv genutzter Wait-Pfad haengen
  - Time-Exposure bleibt mit `remainingTimeSeconds <= 0` in `Exposing`
- Input-/Modal-Diagnose wurde verbreitert:
  - Snapshot exportiert jetzt Alter von Fokus, Modal und Guard
  - Presenter zeigt diese Alter direkt in der Diagnosezeile an
- Wichtige Einordnung:
  - `WorkflowConfirm` bleibt absichtlich modal, ist aber keine Deadlock-Falle,
    weil `Confirm`, `Start`, `Measure` und `Undo` weiterhin deterministische
    Exit-Pfade offenhalten

Noch bewusst nicht Teil dieses Slices:

- automatische Recovery fuer echte Fault-Modale
- policyseitige Timeout-Logik fuer Confirm-Zustaende ohne fachlichen Exitbedarf
- formale Hardware-in-the-loop-Tests fuer absichtlich provozierte Stale-Wait-Faelle

Validierung:

- `pio run -e teensy41`: erfolgreich

## Delta-Update 2026-05-01 (P2 Punkt 9 vorbereitet, Slice 2 umgesetzt)

Abgeschlossener Schritt:

- Die Measurement-Session fuehrt Session-Samples jetzt in echte Zonen- und
  Histogramm-Buckets ueber; Undo und kuenftige Workflow-Zugriffe sind direkt an
  dieser Session-Schicht vorbereitet.

Konkreter Stand:

- Histogramm-Bucket-Mapping in `MeasurementDomainService` eingefuehrt:
  - Startheuristik `round(log2(lux))`
  - Begrenzung auf Zonen `0..10`
  - Balkenskala in `20`er Schritten bis `240`
- Sessionstatus enthaelt jetzt zusaetzlich:
  - Zonensystem-Histogramm
  - letzten Zonenbucket
  - Undo-Tiefe und Undo-Verfuegbarkeit
  - getrennte Sicht auf aktive Session-Samples vs. insgesamt erfasste Samples
- `MeasurementQueryPort` liefert den kompletten Sessionstatus fuer spaetere
  Mess-Workflows.
- Neuer `MeasurementCommandPort` vorbereitet fuer:
  - `undoLastSessionSample()`
  - `resetSession()`
- `ModeWorkflowServices` injiziert Query- und Command-Port jetzt explizit in
  die Workflow-Schicht.
- Remote-Render-Payload bezieht sein `zoneHistogram` jetzt direkt aus der
  Measurement-Session statt aus einem spaeteren UI-Sonderpfad.

Noch bewusst nicht Teil dieses Slices:

- kalibrierte absolute Lux->Zone-Referenzpolitik
- Multi-Spot-Mittelung
- mode-spezifische Mess-Workflows, die Undo/Query bereits aktiv konsumieren

Validierung:

- `pio run -e teensy41`: erfolgreich
- `pio run -e esp32s3_n16r8`: erfolgreich

Naechster Fokus nach diesem Delta:

- P2 Punkt 9 vorbereiten: Measurement-Domain/Histogramm-Session,
  beginnend mit der fachlichen Einspeisung der Wireless-Luxwerte in einen
  dedizierten Measurement-Pfad statt nur Gateway-Telemetrie.

## Delta-Update 2026-05-01 (P2 Punkt 9 vorbereitet, Slice 1 umgesetzt)

Abgeschlossener Schritt:

- P2 Punkt 9 im ersten fachlichen Vorbereitungsslice umgesetzt:
  Wireless-Luxwerte laufen jetzt nicht mehr nur als Gateway-Telemetrie,
  sondern werden an Sequenzkanten in eine dedizierte Measurement-Session der
  `MeasurementDomainService` eingespeist.

Konkreter Stand:

- `MeasurementRuntimeStatus` enthaelt jetzt einen ersten Sessionzustand mit:
  - Gesamtzahl ingestierter Messpunkte
  - letztem Messpunkt
  - kleinem Recent-Ringpuffer fuer spaetere Undo-/Histogramm-Slices
- `MeasurementDomainService` zeichnet neue Wireless-Messungen ueber
  `measurementSequence` nur einmal pro Ereignis auf.
- Die sichtbare Diagnose zeigt den Sessionstand jetzt als `MS <count>/<recent>`
  mit letztem Lux-/Sequenzwert.
- Noch bewusst nicht Teil dieses Slices:
  - Histogramm-Zonenlogik
  - Undo-Verhalten
  - Multi-Spot-/Mittelungsregeln

Validierung:

- `pio run -e teensy41`: erfolgreich
