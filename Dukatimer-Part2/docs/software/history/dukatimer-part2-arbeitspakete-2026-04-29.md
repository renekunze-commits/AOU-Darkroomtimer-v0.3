---
title: Dukatimer-Part2 Arbeitspakete
date: 2026-04-29
author: GitHub Copilot
---

## Dukatimer-Part2 Arbeitspakete

Kurz: Diese Arbeitspakete leiten sich aus der priorisierten TODO-Liste und dem
Migrations-Backlog ab. Sie sind kleiner und umsetzungsnaeher als die MB-Pakete,
bleiben aber an deren Zielbild und Abnahmelogik angeschlossen.

## Paketlogik

- AP = umsetzbares Arbeitspaket fuer einen zusammenhaengenden Slice.
- Ein Arbeitspaket darf mehrere MB-Pakete beruehren, wenn es fachlich genau eine
  Schnittstelle oder ein zusammenhaengendes Risiko schliesst.
- Die Reihenfolge ist verbindlich als Default-Reihenfolge zu verstehen.
- Parallelisierung ist nur dort sinnvoll, wo dieselben Kernschnittstellen nicht
  gleichzeitig umgebaut werden.

## AP-01 Realtime-Schutzpfad und VFS-Hold-Gating

Prioritaet: P0
Bezug: MB-03, MB-04, MB-05, MB-08
Ziel: Blocking- und Seiteneffekte waehrend aktiver Belichtung aus dem VFS- und
Servicepfad fernhalten.

Umfang:

- VFS-Requests und VFS-Chunks bei `realtimeHold` deterministisch ablehnen oder
  parken.
- `.tmp`-Staging plus `rename()`-Commit fuer Datei-Uploads produktiv machen.
- Fehlercodes und Busy-Verhalten im Linkstatus sichtbar machen.
- Regressionscheck fuer `EXPOSING` bei parallelem ESP-Traffic aufnehmen.

Nicht Teil:

- allgemeine Erweiterung des Serviceprotokolls
- neue Komfortfunktionen fuer Wireless

Abhaengigkeiten:

- keine

Abnahmekern:

- Kein SD-/VFS-Schreibpfad blockiert den Schutzpfad waehrend `EXPOSING`.
- Upload-Abbruch hinterlaesst keine kaputte Zieldatei.
- Fehlerfall ist im Runtime- und UI-Status nachvollziehbar.

## AP-02 Sensor-Fail-Safe, I2C-Ausfallstrategie und Feldvalidierung

Prioritaet: P0
Bezug: MB-05, MB-08
Ziel: Den bereits angelegten Closed-Loop-Schutzpfad real belastbar machen.

Umfang:

- Bus-Hang-Strategie fuer Teensy sauber dokumentieren und im Codepfad gegen die
  reale Core-Lage absichern.
- Sensorverlust-, Null-Lux-, Thermik- und Paralleltraffic-Testfaelle definieren.
- Fail-Safe-Restzeit, Fehlerpopup und Schutzreaktion an echter Hardware messen.
- Telemetrie fuer Bus-/Head-Latenz und Schutzereignisse soweit noetig sichtbar
  machen.

Nicht Teil:

- neue Sensortypen
- Kalibrier-Wizard

Abhaengigkeiten:

- AP-01 fuer parallelen VFS-Traffic unter Last

Abnahmekern:

- Dokumentierte Testfaelle existieren und sind mit Messergebnissen hinterlegt.
- Sensorverlust fuehrt reproduzierbar in den dokumentierten Fallback oder Fault.
- Der I2C-Ausfallfall bleibt kein stiller, unklassifizierter Zustand.

## AP-03 ExposureValueMath und MeasurementValueFormatter einfuehren

Prioritaet: P0
Bezug: MB-01, MB-08, MB-10, MB-11, MB-12, MB-13, MB-15
Ziel: Eine einzige fachliche Quelle fuer EV-, F-Stop- und Messwertlogik schaffen.

Umfang:

- `ExposureValueMath` als zentrale Rechenschicht anlegen.
- `MeasurementValueFormatter` als zentrale Formatter-Schicht anlegen.
- Bestehende SG-Schrittlogik auf diese Schichten umlegen.
- Regeln fuer EV plus Lux gegen nicht-fotografische Diagnosedaten abgrenzen.

Nicht Teil:

- vollstaendige Messsession
- vollstaendige BW-/Burn-/Teststrip-Workflows

Abhaengigkeiten:

- keine

Abnahmekern:

- SG hat keine lokale Sondermathematik mehr.
- EV-/Lux-Darstellung ist fuer alle weiteren Modi zentral anschlussfaehig.
- Neue Modi muessen nicht mit eigener Rechen- oder Formatter-Logik starten.

## AP-04 Presenter- und Workflow-Zugriffsschicht vorbereiten

Prioritaet: P0
Bezug: MB-02, MB-04, MB-08, MB-10
Ziel: Verhindern, dass `main.cpp`, `LvglUi` und einzelne Workflows wieder zu
fachlichen Sonderpfaden anwachsen.

Umfang:

- minimale Presenter-Schicht zwischen `SystemSnapshot` und `LvglUi` anlegen
  oder vorbereiten.
- klare Service-Injektion fuer Workflows statt impliziter Querzugriffe definieren.
- Measurement- und Paper-Query-Ports als feste Zugriffspunkte skizzieren.
- UI-Textbildung schrittweise aus Runtime-State und Workflow-Code herausziehen.

Nicht Teil:

- komplette neue UI-Architektur
- komplette Measurement-Domain

Abhaengigkeiten:

- AP-03

Abnahmekern:

- Neue fachliche UI-Texte entstehen nicht direkt in `LvglUi` oder im Workflow.
- Die Zugriffe auf Math, Paper und Measurement folgen einem expliziten Port-Modell.

## AP-05 Papierslots und Persistenz fertig integrieren

Prioritaet: P0
Bezug: MB-06
Ziel: Den vorbereiteten Persistenzunterbau in einen abnahmefaehigen Laufzeitpfad ueberfuehren.

Umfang:

- Slot-Bank in relevante Laufzeitpfade und Standardauswahl integrieren.
- Migrations-, Version-, CRC- und Fehlerstrategie vervollstaendigen.
- Testleitfaden fuer Leerdaten, defekte Daten, Migration und Rueckschreiben
  abschliessen.
- Persistenzfehler fuer UI und Diagnose sichtbar machen.

Nicht Teil:

- inhaltliche Papierkalibrierung
- SG-Vorschlagsalgorithmen

Abhaengigkeiten:

- keine

Abnahmekern:

- Persistente Daten sind ueber Boot und Update stabil.
- Fehlerhafte Daten werden kontrolliert erkannt und behandelt.
- MB-07 und MB-08 koennen direkt auf dieser Basis arbeiten.

## AP-06 UI-Kern, Modalebene und Fokusmodell

Prioritaet: P0
Bezug: MB-02, MB-04
Ziel: Aus dem erweiterten Debugscreen eine belastbare Bedienfuehrung machen.

Umfang:

- globales Fehler-/Confirm-/Wait-Modal statt rein SG-spezifischem Overlay.
- feste Fokuslogik fuer Touch, Encoder und modale Zustaende.
- globalen EventGuard fuer kritische oder blockierte Bedienphasen schliessen.
- Blindbedienungsregeln als technische DoD in die UI-Schnitte uebersetzen.

Nicht Teil:

- Design-Feinschliff
- LiveView- oder Zusatzmodi

Abhaengigkeiten:

- AP-04

Abnahmekern:

- Kritische Dialog- und Fehlerzustaende sind global und konsistent.
- Fokuswechsel sind reproduzierbar und nicht implizit.
- Die UI ist fuer den SG-Kern bedienbar, nicht nur beobachtbar.

## AP-07 SG-Mathematik und papiergetriebene Vorschlaege

Prioritaet: P1
Bezug: MB-07, MB-08
Ziel: Den existierenden SG-Ablauf fachlich vervollstaendigen.

Umfang:

- SG-Targets auf `ExposureValueMath` umstellen.
- papier- und profilgetriebene Vorschlagslogik anschliessen.
- Startwerte, Rueckschreiben und WaitForFilter-/HardSoft-Ablauf fachlich
  schliessen.

Nicht Teil:

- Remote-Messintegration
- allgemeine Messsession

Abhaengigkeiten:

- AP-03
- AP-05
- AP-06

Abnahmekern:

- SG arbeitet fachlich auf Profil- und Papierbasis statt nur auf manuellen Targets.
- Die bestehende Execution-State-Maschine bleibt erhalten und wird nur gefuellt.

## AP-08 Papierkalibrierungs-Wizard

Prioritaet: P1
Bezug: MB-07
Ziel: Papierkalibrierung als echten Arbeitsablauf verfuegbar machen.

Umfang:

- Wizard-Flow, Datenhaltung und Rueckschreibelogik.
- Messwertdarstellung in EV plus Lux.
- Uebergabe der Kalibrierergebnisse an Slots und SG-Vorschlaege.

Nicht Teil:

- Densitometer/Filmtest
- Preflash/Flash-Kalibrierung

Abhaengigkeiten:

- AP-03
- AP-05
- AP-06

Abnahmekern:

- Ein Papier kann vom Mess-/Wizard-Flow bis zum persistenten Profil durchlaufen werden.
- Ergebnisse sind reproduzierbar gespeichert und fuer SG nutzbar.

## AP-09 Remote-Messung und ESP-Serviceintegration end-to-end

Prioritaet: P1
Bezug: MB-03, MB-09, MB-14
Ziel: Die strukturell vorhandenen Protokoll- und Servicepfade in einen echten
fachlichen Betrieb ueberfuehren.

Umfang:

- TeensyCommand- und Renderdatenpfade produktiv verdrahten.
- Remote-Messkommandos in SG und spaeter Measurement-Domain einspeisen.
- reale 1-Wire- und Zusatzsensorik samt Fehlerpfad anbinden.
- Status- und Fehlertelemetrie fuer Link- und Servicepfade vervollstaendigen.

Nicht Teil:

- Wireless-Komfortfunktionen
- Zone- oder LiveView-Modi

Abhaengigkeiten:

- AP-01
- AP-04
- AP-06

Abnahmekern:

- Ein echter Remote-Mess- oder Renderpfad laeuft Ende-zu-Ende.
- Zusatzsensorik hat keinen reinen Platzhalterstatus mehr.

## AP-10 Encoder-, Input- und Workflow-Skalierung

Prioritaet: P1
Bezug: MB-01, MB-04
Ziel: Die Basis fuer mehrere Modi und schnelle Bedienung stabil machen.

Umfang:

- quadraturvalidierten Encoderdecoder mit Lost-Step-Monitoring einfuehren.
- globale Editsemantik fuer EV-/F-Stop-Schritte festziehen.
- ModeCoordinator von harten Spezialverdrahtungen loesen.
- Workflow-Registry oder erweiterbare Workflow-Anbindung vorbereiten.

Nicht Teil:

- neue fotografische Modi selbst

Abhaengigkeiten:

- AP-03
- AP-04
- AP-06

Abnahmekern:

- Schnelle Encoderbedienung bleibt reproduzierbar.
- Neue Modi erfordern keine Umbauten am Grundgeruest aus Zwei-Workflow-Annahmen.

## AP-11 Measurement-Domain und Histogramm-Session

Prioritaet: P2
Bezug: MB-10
Ziel: Die gemeinsame Messschicht fuer spaetere fotografische Modi schaffen.

Umfang:

- Session-Modell fuer Spot, Multi-Spot, Undo und Histogramm.
- Measurement-Domain-Service fuer Start, Undo, Mittelung und Vorschlaege.
- Presenter-/Formatter-Anbindung statt UI-Sonderlogik.

Nicht Teil:

- komplette Zone- oder Densitometer-Workflows

Abhaengigkeiten:

- AP-03
- AP-04
- AP-06
- AP-09

Abnahmekern:

- Messdaten leben in einer eigenen Fachschicht statt in UI oder SensorManager.

## AP-12 Weitere fotografische Modi auf gemeinsamer Basis

Prioritaet: P2
Bezug: MB-11, MB-12, MB-13, MB-15, MB-16
Ziel: BW, Burn, Teststrip, Densitometer, Filmtest, Preflash, Zone und spaetere
Komfortfunktionen ohne neue Fachinseln aufbauen.

Umfang:

- BW zuerst auf der gemeinsamen EV-/F-Stop-Basis produktiv machen.
- Burn/Teststrip, danach Densitometer/Filmtest und Preflash/Flash-Kalibrierung.
- LiveView, Zone und Komfortfunktionen erst nach stabiler Measurement- und
  Formatter-Basis anschliessen.

Nicht Teil:

- Grundsatzentscheidungen zur Kernarchitektur

Abhaengigkeiten:

- AP-03
- AP-04
- AP-08
- AP-09
- AP-10
- AP-11

Abnahmekern:

- Jeder neue Modus nutzt dieselben Basisbausteine fuer Mathematik, Messung,
  Formatter und Workflow-Zugriffe.

## Empfohlene Reihenfolge

1. AP-01
2. AP-02
3. AP-03
4. AP-04
5. AP-05
6. AP-06
7. AP-07
8. AP-08
9. AP-09
10. AP-10
11. AP-11
12. AP-12

## Sinnvolle Parallelisierung

- AP-03 und AP-05 koennen parallel vorbereitet werden, sollten aber vor AP-07
  beide abgeschlossen sein.
- AP-06 kann in der Konzeptphase parallel zu AP-05 laufen, sollte aber nicht
  vor AP-04 technisch verdrahtet werden.
- AP-09 sollte erst starten, wenn AP-01 stabil und AP-06 fuer Fehler- und
  Statusdarstellung belastbar ist.

## Kurzfazit

Die Arbeitspakete trennen bewusst zwischen:

- Schutz- und Realtime-Paketen vor realen Belichtungstests,
- Basispaketen gegen neue Architektur-Zerklueftung,
- darauf aufbauenden fotografischen Fachpaketen.

Damit ist die Reihenfolge nicht nur priorisiert, sondern direkt als
Umsetzungsplan verwendbar.
