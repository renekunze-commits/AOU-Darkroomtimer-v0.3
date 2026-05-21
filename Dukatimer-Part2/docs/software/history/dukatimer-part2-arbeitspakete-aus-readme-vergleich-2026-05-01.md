# Arbeitspakete aus dem historischen README-Vergleich

Stand: 2026-05-01

## Zweck

Dieses Dokument zerlegt die offenen und nur teilweise erfuellten Punkte aus dem
Vergleich der historischen v0.3-README mit dem aktuellen `Dukatimer-Part2`-Stand
in konkrete Arbeitspakete.

Es ist bewusst kein Ersatz fuer den bestehenden Migrations-Backlog und nicht fuer
die allgemeine AP-Liste. Es ist ein Anschlussdokument, das die historischen
Anwenderziele in umsetzbare Slices uebersetzt.

## Ableitungsregel

- Bereits erfuellte Punkte werden nicht als neue Featurepakete erneut geoeffnet.
- Bereits uebertroffene Architekturentscheidungen werden als Guardrails behandelt,
  nicht als neue Features.
- Nur die noch offenen oder nur teilweise erreichten historischen Ziele werden in
  Arbeitspakete zerlegt.

## Nicht erneut oeffnen, sondern als Guardrails beibehalten

Diese Punkte brauchen aktuell kein neues Feature-Arbeitspaket, sondern Schutz
gegen Regression:

- Closed-Loop-Dosispfad ueber `ExposureEngine` und lokalen TSL2561 nicht wieder
  auf blosse Zeitsteuerung zurueckfallen lassen.
- Aktive SG-Mathematik mit papiergetriebener 0,5er-LUT und ISO-P/ISO-R-Modell
  als zentrale Basis behalten.
- Dual-MCU-Rollenmodell beibehalten: Teensy bleibt Autoritaet fuer Belichtung,
  ESP32-S3 bleibt Service- und Gateway-MCU.
- SharedProtocol mit Version, Sequenz und CRC nicht durch ad-hoc Wireless- oder
  UI-Sonderpfade unterlaufen.
- PaperSlot-Persistenz mit Versionierung, CRC und Recovery nicht durch direkte
  Einzeldatei- oder UI-Schreibpfade umgehen.

## Arbeitspakete

## AP-H01 Encoderpfad und Blindbedienung vervollstaendigen

Prioritaet: P0
Bezug: historischer Blindbedienungsanspruch, Vergleichspunkt Encoder/Taster,
MB-01, MB-02
Ziel: Die lokale Bedienung soll reproduzierbar, schnell und workflowtauglich
werden, statt nur positionsbasiert und diagnoseorientiert zu bleiben.

Umfang:

- quadraturvalidierten Encoderpfad mit Lost-Step-Monitoring einfuehren
- Rollen von Encoder 1 bis 3 fuer SG, BW und Folgemodi verbindlich festziehen
- Taster-, LongPress- und Repeat-Semantik lokal und remote angleichen
- Blindbedienungsregeln technisch abnahmbar machen statt nur dokumentarisch

Nicht Teil:

- visuelles UI-Design
- neue fotografische Modi

Abhaengigkeiten:

- keine

Abnahmekern:

- schnelle Encoderbedienung bleibt reproduzierbar
- Fokus- und Guard-Regeln greifen bei lokaler Bedienung konsistent
- SG und BW koennen auf denselben normierten Eingabekonventionen aufsetzen

## AP-H02 LVGL-Placeholder in produktive Bedienoberflaeche ueberfuehren

Prioritaet: P0
Bezug: Vergleichspunkt historische UI gegen heutige Placeholder-UI, MB-02,
AP-06
Ziel: Aus der aktuellen gestapelten Diagnoseansicht eine echte Arbeitsoberflaeche
fuer den Dunkelkammerbetrieb machen.

Umfang:

- `LvglUi::createPlaceholderScreen()` durch echte Screen-Struktur ersetzen
- SG- und BW-Hauptscreen mit festen Informationszonen, Overlay- und
  Dialogkanal aufbauen
- Screen-Navigation, Touch-Zielbereiche und sichtbare Betriebszustaende
  konsolidieren
- Paper-Slot-, Mess- und Linkstatus ohne Diagnosecharakter, aber weiterhin klar
  sichtbar machen

Nicht Teil:

- finaler Design-Feinschliff
- Densitometer- oder LiveView-Screens

Abhaengigkeiten:

- AP-H01 fuer stabile Eingabekonventionen

Abnahmekern:

- SG ist ueber die Hauptoberflaeche benutzbar, nicht nur beobachtbar
- BW-Shell kann auf derselben UI-Basis sichtbar gefuehrt werden
- Fehler-, Wait- und Confirm-Zustaende bleiben global konsistent

## AP-H03 Wireless-Terminal-Kommandos und Render-Freshness schliessen

Prioritaet: P0
Bezug: Vergleichspunkt offener Remote-Pfad, MB-09, Audit P1 Remote-Kommandos
Ziel: Das Wireless-Handgeraet soll fachlich wirklich Terminal sein und nicht nur
ein halb verdrahteter Anzeige- und Eventpfad.

Umfang:

- `RemoteMeasurementStart`, `RemoteMeasurementCancel` und `RemoteHaptic`
  end-to-end bis zum C6-Terminal durchleiten
- Freshness-Pruefung fuer `RemoteDisplayPayload.sequenceNumber` auf C6-Seite
  einfuehren
- Fehler- und Defaultverhalten bei ausbleibender Gegenstelle definieren
- gezielte Diagnosecodes fuer Remote-Kommandoverlust und Stale-Render sichtbar
  machen

Nicht Teil:

- neue Komfortmodi auf dem Remote-Terminal
- vollstaendige Messsession-Logik

Abhaengigkeiten:

- keine

Abnahmekern:

- Start, Cancel und Haptik kommen nachweisbar am C6 an
- veraltete Renderpakete werden ignoriert
- Gateway und Teensy melden den Pfad nicht mehr nur scheinbar als produktiv

## AP-H04 Measurement-Domain und Pre-Metering-Sitzung vervollstaendigen

Prioritaet: P0
Bezug: Vergleichspunkt Messpipeline offen, MB-10, historische Pre-Metering-Logik
Ziel: Aus der begonnenen Measurement-Domain eine gemeinsame Grundlage fuer Spot,
Undo, Histogramm und spaetere Densitometer-/Kalibrier-Workflows machen.

Umfang:

- Session-Modell fuer Einzelspot, Undo und Histogramm fachlich vervollstaendigen
- zentrale EV-plus-Lux-Aufbereitung fuer echte Messworkflows abschliessen
- Wireless-Messwerte nicht nur als Diagnosedaten, sondern als normale
  Measurement-Quelle in Workflows nutzbar machen
- Query- und Command-Ports fuer spaetere Kalibrier-, Densitometer- und BW-Pfade
  stabilisieren

Nicht Teil:

- lokale TSL2591-Hardwareintegration
- kompletter Densitometer-UI-Flow

Abhaengigkeiten:

- AP-H03 fuer vollstaendige Remote-Messkommandos

Abnahmekern:

- ein Messworkflow kann Samples sammeln, undoen und sichtbar auswerten
- EV- und Lux-Darstellung kommen aus einer gemeinsamen Schicht
- Folgepakete muessen keine neue Messinfrastruktur mehr erfinden

## AP-H05 TSL2591-Quellenentscheid und lokaler Pre-Metering-Pfad

Prioritaet: P1
Bezug: Vergleichspunkt historischer TSL2591-Doppelpfad nur teilweise erreicht
Ziel: Die historische Erwartung "kabelgebunden oder wireless" entweder wieder
herstellen oder bewusst und sauber als neue Produktentscheidung aufloesen.

Umfang:

- entscheiden, ob Part2 einen lokalen kabelgebundenen TSL2591-Pfad produktiv
  tragen soll oder wireless-only als bewusstes Zielbild gilt
- falls lokal gewollt: HAL, Sensorstatus und Measurement-Anschluss implementieren
- falls lokal nicht gewollt: Doku, UI und Vergleichsstellen konsistent auf das
  neue Zielbild bereinigen
- Quelle, Prioritaet und Fallback zwischen Wireless- und eventuellem lokalem
  Pre-Metering explizit regeln

Nicht Teil:

- Densitometer-Fachlogik
- Kopfseitiger TSL2561-Dosispfad

Abhaengigkeiten:

- AP-H04

Abnahmekern:

- es gibt keine implizite oder halbfertige TSL2591-Erwartung mehr
- Measurement-Workflows wissen eindeutig, aus welcher Pre-Metering-Quelle sie
  lesen

## AP-H06 Papierkalibrierungs-Wizard produktiv machen

Prioritaet: P1
Bezug: Vergleichspunkt offener Kalibrier-Wizard, MB-07
Ziel: Die bereits vorhandenen Paper-Slots und Profilstrukturen in einen echten
Kalibrier-Arbeitsablauf ueberfuehren.

Umfang:

- Wizard-Flow fuer Messaufnahme, Zwischenwerte und Ergebnispruefung aufbauen
- K-Faktor- und Profilableitung auf Measurement-Domain und Paper-Slot-Basis
  verdrahten
- Rueckschreiben in aktiven Slot mit sicherer Persistenz und Rollback-Verhalten
  anbinden
- UI-Fuehrung fuer neue, unkalibrierte und bereits kalibrierte Papiere trennen

Nicht Teil:

- Densitometer/Filmtest
- Preflash/Flash-Kalibrierung

Abhaengigkeiten:

- AP-H02
- AP-H04
- AP-H05

Abnahmekern:

- ein Papier kann ohne manuelle Datenstruktur-Eingriffe kalibriert werden
- das Ergebnis landet reproduzierbar im gewaehlten Slot
- SG und BW koennen die neuen Profildaten direkt konsumieren

## AP-H07 BW-Mischlicht als produktiven Modus anschliessen

Prioritaet: P1
Bezug: Vergleichspunkt BW nur teilweise erfuellt, MB-11
Ziel: Den historischen BW-Mixed-Light-Modus auf der heutigen zentralen
Mathematik-, Head- und Exposure-Basis produktiv verfuegbar machen.

Umfang:

- `ModeId::BlackWhite` von Shell auf echten Workflow erweitern
- Gruen/Blau-Mischung fuer Grade 0.0 bis 5.0 auf gemeinsamer EV-/F-Stop- und
  Paper-Basis aufbauen
- `HeadSpectrumCommand` und `ExposureEngine` ohne Modus-Sonderinsel anbinden
- Bedien- und Sichtlogik an SG-nahe Konventionen anlehnen

Nicht Teil:

- Burn und Teststrip
- Densitometer

Abhaengigkeiten:

- AP-H01
- AP-H02
- AP-H04
- AP-H06

Abnahmekern:

- BW ist nicht mehr nur Mode-Enum, sondern ein nutzbarer Belichtungsmodus
- Gradeingabe, Zielgroessen und Belichtung folgen derselben Grundlogik wie SG

## AP-H08 Burn- und Teststrip-Familie anschliessen

Prioritaet: P2
Bezug: Vergleichspunkt Burn/Teststrip offen, MB-11
Ziel: Die historischen Dunkelkammer-Helfer als Folgepaket auf derselben
Belichtungs- und Messbasis anschliessen.

Umfang:

- Teststrip-Sequenzgenerator mit EV- oder F-Stop-Schritten aufbauen
- Burn/Dodge-Korrekturen als kontrollierte Abweichung von BW/SG-Basis modellieren
- Kopfsemantik `Burn` und `TestStrip` produktiv verwenden
- UI, Bedienung und Sicherheitsregeln aus BW/SG wiederverwenden statt duplizieren

Nicht Teil:

- Densitometer/Filmtest
- Preflash-Fachlogik

Abhaengigkeiten:

- AP-H07

Abnahmekern:

- Burn und Teststrip besitzen keine eigene Mathematikinsel
- Belichtungs- und Sicherheitsregeln bleiben mit BW/SG konsistent

## AP-H09 Densitometer-, Filmtest- und Preflash-Familie aufbauen

Prioritaet: P2
Bezug: Vergleichspunkt Densitometer offen, MB-12, MB-13
Ziel: Die historisch beschriebenen Mess- und Kalibrierhilfen als dritte grosse
Fachfamilie nach SG und BW aufbauen.

Umfang:

- REF/BASE/MEAS-Flow fuer Densitometer und Filmtest einfuehren
- Zonenhelfer, LogD-Ableitung und Messdarstellung auf Measurement-Domain-Basis
  anschliessen
- Preflash- und Flash-Kalibrierung auf denselben Profil- und Slotstrukturen
  aufbauen
- Ergebnisse so modellieren, dass sie in Kalibrierung, BW und SG zurueckwirken
  koennen

Nicht Teil:

- neue Remote-Komfortfunktionen
- LiveView/Zone-Modus als gesonderte Oberflaechenfamilie

Abhaengigkeiten:

- AP-H04
- AP-H06
- AP-H07

Abnahmekern:

- Densitometer ist ein echter Workflow statt nur ein Zukunftsplatzhalter
- Preflash und Flash-Kalibrierung entstehen nicht als seitlicher Sonderpfad

## AP-H10 Audio- und lokale Feedbackstrategie entscheiden und schliessen

Prioritaet: P2
Bezug: Vergleichspunkt historischer Piezo-Buzzer offen
Ziel: Die historische Erwartung an akustisches beziehungsweise haptisches Feedback
lokal sauber zu beantworten.

Umfang:

- entscheiden, ob Part2 einen lokalen Piezo-/Buzzer-Pfad produktiv tragen soll
- falls ja: Eigentuemerschaft, Signalarten und Realtime-Regeln festlegen
- falls nein: Feedbackstrategie explizit auf visuelle und Remote-Haptik-Pfade
  begrenzen
- Start-, Fehler-, Abschluss- und Metronom-Signale nicht als spontane Nebenlogik,
  sondern als eigenes Feedbackmodell definieren

Nicht Teil:

- Musik- oder Komfortfunktionen
- Ersatz fuer die globale UI-Fuehrung

Abhaengigkeiten:

- AP-H02
- AP-H03

Abnahmekern:

- lokale und Remote-Feedbackkanäle widersprechen sich nicht
- es gibt eine klare Produktantwort auf den historischen Piezo-Anspruch

## Empfohlene Reihenfolge

1. AP-H01 Encoderpfad und Blindbedienung vervollstaendigen
2. AP-H02 LVGL-Placeholder in produktive Bedienoberflaeche ueberfuehren
3. AP-H03 Wireless-Terminal-Kommandos und Render-Freshness schliessen
4. AP-H04 Measurement-Domain und Pre-Metering-Sitzung vervollstaendigen
5. AP-H05 TSL2591-Quellenentscheid und lokaler Pre-Metering-Pfad
6. AP-H06 Papierkalibrierungs-Wizard produktiv machen
7. AP-H07 BW-Mischlicht als produktiven Modus anschliessen
8. AP-H08 Burn- und Teststrip-Familie anschliessen
9. AP-H09 Densitometer-, Filmtest- und Preflash-Familie aufbauen
10. AP-H10 Audio- und lokale Feedbackstrategie entscheiden und schliessen

## Kurzfazit

Die historische README scheitert im heutigen Stand nicht an fehlender
Grundarchitektur, sondern an den noch nicht geschlossenen Anwenderpaketen.
Genau deshalb liegen die naechsten sinnvollen Slices nicht mehr bei allgemeiner
Grundlagenarbeit, sondern bei Bedienbarkeit, Measurement-Domain,
Kalibrierworkflow, BW-Familie und dem Abschluss der Remote- und Feedbackpfade.
