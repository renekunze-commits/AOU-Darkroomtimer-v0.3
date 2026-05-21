# Offene Entscheidungen

Stand: 2026-05-06

Dieses Dokument sammelt Architektur- und Produktentscheidungen, die vor weiteren grossen Implementierungsschritten geklaert werden sollten. Entscheidungen sind hier nur dann als gesetzt markiert, wenn sie durch Projektregeln und Codearchitektur bereits eindeutig sind.

## Bereits gesetzte Leitplanken

- Teensy bleibt Autoritaet fuer Exposure, Papier, UI, lokale Eingaben und Safety.
- ESP32-S3 bleibt Service- und Gateway-MCU, nicht zweite Exposure-Instanz.
- Wireless C6 bleibt Terminal/Messgeraet, nicht Papier- oder Belichtungsautoritaet.
- `lib/SharedProtocol` ist die Part2-Vertragsflaeche fuer Teensy/ESP32-S3; C6 muss damit konsistent bleiben.
- Historische Ordner bleiben Referenz und werden nicht als aktive Firmwarelinie editiert.
- Vorhandener interner RAM und externer PSRAM sind bewusst als
   Produktressource zu nutzen: fuer Staging, Puffer, Historie und
   Schreibentlastung. Speichersparen ist kein Selbstzweck, solange Hot-Path,
   DMA- und Safety-Regeln respektiert werden.

## E1: Wie intelligent darf das C6-Terminal werden?

Status: Architekturleitplanke gesetzt, Detail offen.

Optionen:

- Dumb Terminal: C6 misst, sendet Eingaben/Lux, rendert Teensy-Payloads und zeigt Standalone nur als Fallback.
- Semi-Smart Terminal: C6 fuehrt lokale Messsessions, Haptic und UI-Freshness selbst, aber ohne Papier-/Exposure-Math.
- Smart Terminal: C6 berechnet eigene Mess-/Papierlogik.

Empfohlene Richtung: Semi-Smart nur fuer Terminal-Hygiene. Messbutton, Haptic, Render-Freshness, Offline-Screen und lokale Sensoranzeige duerfen auf C6 leben; Paper, EV-Entscheidungen und Exposure bleiben auf Teensy.

Warum: Das behebt die aktuelle Remote-Scheinloesung, ohne die Safety- und Mathematikautoritaet zu duplizieren.

Klärungsbedarf: Welche lokalen Standalone-Funktionen soll das C6 ohne Teensy behalten?

-> Semi-Smart Terminal C6 fuehrt lokale Messsessions, Haptic und UI-Freshness selbst, aber ohne Papier-/Exposure-Math.

## E2: Wie wird SharedProtocol zwischen Part2 und C6 synchron gehalten?

Status: Richtung gesetzt, konkrete Abnahmekriterien je Feature offen.

Optionen:

- Eine Quelle: C6 bindet den Part2-Header direkt ein oder nutzt ein gemeinsames Submodul.
- Kopie plus Check: Header bleibt kopiert, aber ein Script/CI vergleicht Inhalt und ABI.
- Manuell: Kopie wird bei Bedarf händisch gepflegt.

Empfohlene Richtung: Eine Quelle oder mindestens Kopie plus automatischem Check. Manuell ist bei ESP-NOW-ABI zu riskant.

Warum: ABI-Drift kann ohne Compilerfehler falsche Payloadinterpretation erzeugen.

Klärungsbedarf: Soll `Wireless TSL2591` langfristig eigenstaendiges Projekt bleiben oder in eine gemeinsame Workspace-Struktur mit Part2-SharedProtocol ruecken?

-> Eine Quelle oder mindestens Kopie plus automatischem Check.

## E3: Wie wird die C6-Firmware versioniert?

Status: Offen.

Entscheidungspunkte:

- Datei- und Namensschema fuer `FirmwareVersion.h` im C6-Projekt.
- Version im Bootlog, OLED-Serviceanzeige und optional Wireless-Payload.
- Kopplung an SharedProtocol-Version.

Empfohlene Richtung: Gleiches Muster wie Teensy/ESP32-S3: zentrale Build-Konstanten, Serial-Bootlog, optional Diagnose/Status.

Warum: Ohne Version ist jede Remote-Fehlersuche unscharf.

->Gleiches Muster wie Teensy/ESP32-S3: zentrale Build-Konstanten, Serial-Bootlog, optional Diagnose/Status.

## E4: Wie funktioniert finales MAC-Pairing?

Status: Offen.

Optionen:

- Feste Compile-Time-MACs.
- Broadcast-Discovery mit erstem validen Peer und persistiertem Binding.
- UI-gestuetztes Pairing mit Anzeige/Bestätigung.

Empfohlene Richtung: Bring-up darf Broadcast nutzen; Produktpfad sollte bewusstes Pairing mit sichtbarem Peerstatus und Resetmoeglichkeit haben.

Warum: Dunkelkammergeraete sollen nicht versehentlich ein falsches Terminal bedienen.

Klärungsbedarf: Wie viele C6-Terminals sind real vorgesehen: genau eins, mehrere, oder Servicegeraete wechselnd?

-> genau eins

## E5: Welche Messquelle ist fotografisch massgeblich?

Status: Offen.

Optionen:

- Lokaler TSL2561 als Exposure-Regelquelle, C6 TSL2591 als Komfort-/Spotmeter.
- C6 TSL2591 als primaere Messquelle fuer Workflows, lokaler TSL2561 nur Safety/Closed-Loop.
- Externe Referenzkalibrierung bestimmt beide Sensoren.

Empfohlene Richtung: Lokaler Sensor fuer geschlossene Belichtung und Safety, C6 fuer Spot/Multi-Spot. Beide muessen gegen eine definierte Referenz kalibriert werden.

Warum: Sensorposition, Spektralantwort und Messgeometrie unterscheiden sich. Eine pauschale Lux-Gleichsetzung ist fotografisch nicht sauber.

Klärungsbedarf: Welche reale Sensorbestueckung ist final geplant: TSL2561 lokal, TSL2591 wired lokal, TSL2591 nur wireless?

-> TSL2561 lokal als closed-loop sensor direkt unter Neopixel / oberhalb von Vergrößererobjektiv
   TSL2591 als Messsesor remote auf Papierebene, er misst durch das Negativ; aus Wert von TSL2561 und Wert von TSL2591 lässt sich das Delta durch Negativ und Objektivblende usw. bestimmen; erster Abgleich durch Messung ohne Negativ

## E6: Wie wird der I2C-Hang des lokalen Sensors geloest?

Status: Richtung festgelegt, Hardware-Umsetzung offen.

Optionen:

- Hardware-Bus-Recovery/Reset fuer Sensorversorgung oder I2C-Isolator.
- Alternative Wire-/I2C-Implementierung mit Timeout auf Teensy.
- Watchdog-basierter Neustart mit klarer Faultdiagnose.
- Lokale Sensorik vom Safety-Pfad entkoppeln und Dosismodus bei Unsicherheit sperren.

Festgelegte Richtung: Hardware- und Softwaremassnahme kombinieren: Bus-Recovery/Power-Cycle plus expliziter Fault/Fallback. Reiner Watchdog ist nur letzte Sicherung.

Gewaehlte Kombination:

- Hardware: dedizierter High-Side-Schalter in `+3V3_HEAD` auf der Teensy-Seite vor dem Sensor-RJ45, gesteuert von einem neuen, ausschliesslich dafuer reservierten Teensy-GPIO.
- Hardware-Regel: bestehende Leitungen `I2C1_SCL`, `I2C1_SDA`, `TSL_INT` und `1W_DATA` werden nicht fuer Recovery repurposed; undokumentierte RJ45-Reserven gelten nicht als Beleg fuer verfuegbare Steuerleitungen.
- Software: `SensorManager` bleibt Besitzer der lokalen Recovery-Logik und darf nach dokumentierten TSL-Fehlern genau eine begrenzte Power-Cycle/Reinit-Sequenz gegen diesen Schalter ausfuehren; bei weiterem Fehler bleibt die bestehende Time-Degradierung plus Watchdog als letzte Rueckfallebene aktiv.

Warum: Ein blockierter I2C-Call kann die komplette Exposure-Autoritaet stoppen. Gleichzeitig belegt die aktuelle Hardwaredoku am Sensorpfad nur `+3V3_HEAD`, `GND`, `I2C1_SCL`, `I2C1_SDA`, `TSL_INT` und `1W_DATA`; damit ist ein sauberer Power-Cycle nur ueber eine neue schaltbare Kopfversorgung belastbar, nicht ueber improvisierte Umnutzung vorhandener Signale.

Restoffen: konkrete Schaltung, GPIO-Wahl und Hardwareabnahme dieser neuen Kopfversorgungsstufe.

## E7: Welche Papierdefaults sind autoritativ?

Status: Offen.

Aktueller Code: 20 Slots, nur `Ilford MGIV RC` als kalibrierter Default, lineare SG-Demo-LUT. Foma Variant 311 ist nicht codebelegt als Default.

Entscheidungspunkte:

- Duerfen historische Werte als Startwerte ausgeliefert werden?
- Wie werden Demo-, importierte und real gemessene Werte getrennt?
- Wer ist Datenautoritaet fuer ISO-P/ISO-R und 0,5er LUT?

Empfohlene Richtung: Defaults nur als Demo markieren, bis reale Messprotokolle vorliegen. Echte Profile erhalten Quellen- und Kalibrierstatus.

Warum: Papierdaten sind Teil der fotografischen Wahrheit, nicht UI-Dekoration.

->Defaults nur als Demo markieren, bis reale Messprotokolle vorliegen. Echte Profile erhalten Quellen- und Kalibrierstatus.

## E8: Wie soll PaperSlot-Recovery bei korrupten Daten aussehen?

Status: Code-Grundsatz aktiv, HTTP-/Service-Recoveryvertrag vorhanden, erster lokaler Placeholder-Hinweis aktiv, finale EEZ-Einbindung offen.

Aktueller Code: Defaults werden nur bei `FileNotFound` automatisch geschrieben. InvalidBlob, ReadFailed, UnsupportedFormatVersion und InvalidBank bleiben sichtbar. Der ESP-Service bietet zusaetzlich `inspect`, `recovery-state`, `recover` und `restore-backup`, aber keine stille Auto-Recovery.

Offene Produktentscheidung:

- Wie wird der bisherige Placeholder-Hinweis spaeter in EEZ als produktiver Recovery-Dialog oder Aktionsblock ueberfuehrt?
- Reicht eine sichtbare Meldung mit Aktion oder braucht es einen eigenen Recovery-Dialog?
- Soll `recommendedAction = offer_restore_backup` direkt einen lokalen Shortcut bekommen?

Empfohlene Richtung: Code-Grundsatz beibehalten. Keine stillen Default-Overwrites bei moeglich wertvollen Kalibrierdaten; Recovery bleibt eine explizite Nutzerentscheidung. Den vorhandenen HTTP-/Service-Vertrag und den jetzigen Placeholder-Hinweis als Vorstufe fuer spaetere EEZ-UI-Recovery weiterverwenden.

Warum: Kalibrierte Papierdaten koennen aufwendig erstellt sein und duerfen nicht transient verloren gehen. Ein expliziter Recovery-Hinweis ist sinnvoll, aber automatische Ersetzung bleibt riskant.

->Code-Grundsatz beibehalten. Keine stillen Default-Overwrites bei moeglich wertvollen Kalibrierdaten; Recovery bleibt explizite Nutzerentscheidung.

## E9: UI-Technologie und Produktpfad

Status: Technische Richtung gesetzt, code-first Zwischenstand build-verifiziert, finaler EEZ-Produktpfad offen.

Aktueller Code: LVGL 8.x, ILI9488_t3_mm-Shim, EEZ-ready, handgeschriebene LVGL-C-Screenbaeume fuer Boot, Busy, Splitgrade, PaperWorkspace, Setup, Measurement und WirelessRemote. Ein vollstaendiges `.eez-project` existiert weiterhin nicht.

Entscheidungspunkte:

- EEZ Studio als Hauptquelle fuer Screens oder handgeschriebene LVGL-Komponenten?
- Wie werden Touch und Encoder in denselben Fokus-/Modalpfad gebracht?
- Welche Bildschirmhierarchie ist final?

Empfohlene Richtung: LVGL/EEZ beibehalten, aber Runtime-Glue streng klein halten. Encoderbedienung muss gleichwertig zu Touch bleiben.

Warum: Die Hardware ist auf TFT/Touch/Encoder ausgelegt; ein externes Nextion-/Web-UI-Modell wuerde die aktuelle Architektur zerfasern.

-> eez wird favorisiert; encoder behalten priorität; Touch nur als komfort-feature

Nachtrag 2026-05-06: Der code-first Stand ist als funktionaler Zwischenschritt akzeptiert, weil er alle Zielseiten sichtbar und buildbar macht. Er ersetzt aber nicht die finale EEZ-/Style-Entscheidung. Insbesondere duerfen Screen-spezifische Formatter-, Farb- oder Fokusinseln jetzt nicht als neue Normalform wachsen; sie muessen entweder auf Presenter/Formatter/Palette zurueckgefuehrt oder in diesem Dokument als bewusste Diagnoseausnahme begruendet werden.

## E10: Welches mathematische Modell gilt fuer BW?

Status: Fachentscheidung fuer fixed-grade/multigrade gesetzt, Target-Datenmodell offen.

Optionen:

- BW als Mischung aus Soft/Hard-Kanaelen mit Gradationsziel.
- BW als separater `kBw`-Faktor ohne Gradationsmodell.
- BW als fixed-grade Papier-/Filtermodell.

Gesetzte Richtung: BW bleibt paper-driven. Das gewaehlte Papierprofil muss
verbindlich definieren, ob es `fixed grade` oder `multigrade` ist; daraus
leitet der BW-Workflow ab, ob eine Gradationsbedienung angeboten oder sichtbar
gesperrt wird. Die mathematische Ausfuehrung bleibt auf `PaperProfile` und
zentrale EV-/Dosisdienste gestuetzt.

Warum: BW ist aktuell nur Shell. Eine vorschnelle Formel wuerde spaeter schwer zu entwirren sein.

Klärungsbedarf: Die Papier-Eigenschaft ist mit `PaperGradeMode` im aktiven
`PaperProfile` modelliert. Offen bleibt die saubere Target-Einheit fuer
BW-Zeit/Dosis, damit UI, Math und Validierung denselben Exposure-Vertrag sehen.

-> BW-Vorgabe ergibt sich aus dem ausgewaehlten Papier; fuer jedes Papier muss
   explizit definiert werden, ob es fixed grade oder multigrade ist.

Nachtrag 2026-05-06: Die Zielrichtung ist fuer den expliziten BW-Modus
konkretisiert. Fixed-grade-Papier fuehrt zu einer Weisslichtbelichtung mit
gesperrter Gradationsanzeige aus `fixedGradeValue`; Multigrade-Papier fuehrt zu
0.5er Gradationswahl, Soft-/Hard-Farbmischung und einem einzelnen Zeit-/Dosis-
Exposure-Ablauf. Der konkrete Umsetzungsplan liegt in
[dukatimer-part2-bw-modus-umsetzungsplan-2026-05-06.md](dukatimer-part2-bw-modus-umsetzungsplan-2026-05-06.md).

Nachtrag 2026-05-06 (E17 umgesetzt): Die Entflechtung von `kBw` ist in Code
geloest. `kBw` ist fortan ausschliesslich der dimensionslose, statische
Transmissionsfaktor des Weissunkts (10^-D_N, Wertebereich (0,1)) und wird in
`PaperExposureProfile.h` mit einem unveraenderbaren Vertrag kommentiert.
`SplitgradeWorkflow::initializePaperDrivenState()` laedt `kBw` nicht mehr als
Belichtungszeit; der Basiszielwert startet immer mit `kDefaultBaseTarget`.
Der AP-07-Akzeptanztest-Harness ist angepasst und kompiliert fehlerfrei;
der Teensy41-Build bestaetigt die Regressionsfreiheit (SUCCESS).

## E11: Umfang von Densitometer und Filmtest

Status: Offen.

Entscheidungspunkte:

- Nur Reflexions-/Printmessung oder auch Negativ-/Transmissionsmessung?
- Welche Messgeometrie und welcher Sensor?
- Welche Ausgabe: LogD, Zone, EV, Papierempfehlung?

Empfohlene Richtung: Nicht implementieren, bis Messgeometrie und Referenzstandard definiert sind. Vorher nur Datenmodell-/UI-Platzhalter vermeiden.

Warum: Densitometrie ist kein reines Softwarefeature; Geometrie und Kalibrierung bestimmen die Wahrheit.

-> Densitometrie am Anfang nur als Negativ-/Transmissionsmessung über TSL2591 aus Remotebedienteil einarbeiten; spätere weiterentwicklung für Auflicht mit separaten remote messteil derzeit offen

## E12: Audio, Haptic und lokale Rueckmeldung

Status: Offen.

Optionen:

- Piezo/Audio am Teensy fuer lokale Phasen und Faults.
- Haptic/Buzzer am C6 fuer Remote-Feedback.
- Beide, aber mit zentraler Ereignissemantik.

Empfohlene Richtung: Zentrale Feedback-Events definieren, Ausgabegeraete bleiben austauschbar. Remote-Haptic erst nach funktionierendem Commandpfad claimen.

Warum: Feedback ist sicherheitsnah, wenn Nutzer im Dunkeln arbeitet. Doppelte oder widerspruechliche Signale waeren schlechter als keine.

-> Zentrale Feedback-Events definieren, Ausgabegeraete bleiben austauschbar. Remote-Haptic erst nach funktionierendem Commandpfad claimen.

## E13: Produktives Dunkelkammer-UI-Grundlayout

Status: gesetzt am 2026-05-02; code-first Umsetzung 2026-05-06 geprueft.

Gesetzte Entscheidung:

- Die Produkt-UI nutzt fuer Dunkelkammerbetrieb eine moeglichst dunkle,
   rotbasierte Palette. Gruen- und Blautoene bleiben Diagnose- oder
   Entwickleransichten vorbehalten.
- Eine separate untere Status- oder Overlay-Leiste ist im Produktlayout nicht
   vorgesehen. Die historischen Werte `overlayText` und `overlayColor` werden in
   das Meldungsband der Header-Zeile integriert.
- Die unterste Zeile ist ausschliesslich fuer Modusreiter reserviert.
- Diese Modusreiter bilden Workflow-Familien ab: `PAPER`, `MEAS`, `PRINT`,
   `SETUP`. SG/BW und andere Fachschritte liegen innerhalb dieser Familien und
   werden nicht als gleichrangige Hauptreiter exponiert.
- Redundante permanente Anzeigen wie ein staendiger Linkstatus im Header
   entfallen; degradiertes Link-/Wireless-Verhalten erscheint als Meldung oder
   auf der Remote-Diagnoseseite.
- Die NeoPixel-Leistung wird fuer den Nutzer als Prozentwert sichtbar gemacht:
   im Setup als konfigurierte Grenze plus Live-Wert und im Laufzeitkontext als
   effektive Leistung, insbesondere bei thermischer Reduktion.
- Die Farbwerte bleiben normativ in der Software-Doku definiert, werden aber
   fuer die produktive UI primaer in EEZ Studio umgesetzt (Styles, Themes,
   Flow-Bindings), damit visuelle Feinabstimmung ohne C++-Layoutaenderung
   moeglich ist.
- `LvglUi` liefert die fachlichen Zustaende und Texte; eine zusaetzliche
   C++-seitige Farbzuweisung fuer dieselben produktiven Widgets ist nur als
   expliziter Fallback zulaessig und darf die EEZ-Definitionen nicht
   stillschweigend uebersteuern.

Nachtrag 2026-05-06:

- Der handgeschriebene LVGL-Zwischenstand nutzt diese Zonenaufteilung grundsaetzlich: Header 0..48, Hauptzone 48..284, ModeTabs 284..320.
- Einzelne C++-Farbsetzungen in Paper/Setup/Measurement/Remote gelten nur als Bring-up-Fallback. Gruen/Rot in der Remote-Diagnoseseite ist als Diagnoseausnahme zulaessig, darf aber nicht auf produktive Hauptscreens ausstrahlen.
- Der offene technische Rest ist kein neues Layoutziel, sondern Konsolidierung: Tab-Handle-Semantik, Magic-Farbwerte und PageMeasurement-Formatter-Nutzung muessen vor finaler EEZ-Uebernahme bereinigt werden.

Warum:

- Dunkelkammernutzung verlangt minimale Gesamthelligkeit, schnelle visuelle
   Priorisierung und moeglichst wenig konkurrierende Dauerindikatoren.
- Ein eigener unterer Meldungsbereich und eine zweite Statusleiste erzeugen bei
   480x320 zu viel Fragmentierung und verschenken Flaeche fuer die eigentliche
   Hauptinformation.

## E14: VFS/Web/PSRAM als Produktfeature oder Servicewerkzeug?

Status: Offen.

Aktueller Code: HTTP-VFS-Bridge existiert als Bring-up-/Uploadpfad. Kein belegtes UI-Asset-System, kein Zugriffsschutz, keine PSRAM-Cache-Strategie.

Optionen:

- Service-only: Upload fuer Entwickler/Techniker.
- Produkt-Konfiguration: Paperprofile/Backups ueber HTTP.
- Voller Assetpfad: UI-/Webassets, ggf. PSRAM-Caching.

Empfohlene Richtung: Kurzfristig Service-only plus Paperprofil-Import/Export. Asset-System erst nach Safety-TX-Queue und Zugriffsschutz.

Warum: Web/VFS darf die Belichtungsautoritaet nicht stoeren und darf keine unsicheren Updates einfuehren.

-> Produkt-Konfiguration: Paperprofile/Backups ueber HTTP. Belichtung muss waehrend aktiver Websession sicher blockiert werden.

## E15: Welche Validation ist fuer „fertig“ ausreichend?

Status: Offen.

Entscheidungspunkte:

- Reicht Build plus Unit-/Harness-Test fuer reine Logik?
- Welche Hardwaremessungen sind fuer Exposure, Sensorik, Remote und UI Pflicht?
- Welche Toleranzen gelten fotografisch?

Empfohlene Richtung: Drei Stufen definieren: Code-verifiziert, Build-verifiziert, hardware-verifiziert. Fotographische Claims brauchen Hardware- und Messdaten.

Warum: Das Projekt enthaelt Safety- und Photochemie-nahe Funktionen. „Kompiliert“ ist kein Abschlusskriterium.

-> Drei Stufen definieren: Code-verifiziert, Build-verifiziert, hardware-verifiziert. Fotographische Claims brauchen Hardware- und Messdaten.

Nachtrag 2026-05-06: Der aktuelle UI-Screenabschluss ist code-/build-verifiziert, aber nicht hardware-verifiziert. Aussagen wie "alle Screens existieren" sind damit erlaubt; Aussagen wie "produktive UI ist fertig" oder "fotografisch validiert" bleiben gesperrt, bis Encoder/Touch/Sichtbarkeit, reale Sensorik und Papiermessdaten abgenommen sind.

## E16: Reihenfolge der naechsten Arbeitspakete

Status: Offen, Reihenfolge nach UI-Codeabschluss aktualisiert.

Empfohlene Reihenfolge:

1. UI-Zwischenstand hygienisieren: PageMeasurement auf Presenter/Formatter fuehren, Magic-Farbwerte/Tab-Handles bereinigen und Hardware-Sicht-/Encoderabnahme fahren.
2. I2C-Hangstrategie und lokale TSL2561-Recovery ohne Neustart klaeren.
3. Remote-Commandpfad und C6-Freshness/Versionierung/Pairing schliessen.
4. MeasurementDomain kalibrieren, Proposal-Eligibility einfuehren und Spot/Multi-Spot nutzbar machen.
5. Paper-Kalibrierwizard mit realem End-to-End-Protokoll und BW-Workflow.
6. Teststrip, Burn/Dodge, Preflash, Densitometer nach Validationsbasis.

Warum: Diese Reihenfolge reduziert zuerst Safety- und Wahrheitsrisiken, bevor neue fotografische Funktionen darauf aufbauen.

## E17: Wie wird das explizite Setup-Menue geschnitten?

Status: Richtung gesetzt, Runtime- und Widgetpfad code-/build-verifiziert, Hardware-/Safety-Abnahme offen.

Aktueller Befund:

- v0.3 und v0.9 hatten bereits einen eigenen Setup-Modus bzw. eine Setup-App.
- Historisch wurden dort aber globale Systemsettings, Papier-/Preflash-Werte,
   Kalibrieraktionen und Servicefunktionen vermischt.
- Der aktuelle Part2-Code besitzt `ModeId::Setup`, `SetupWorkflow`, `SetupModeRuntimeState`, persistente `SystemSettings`, einen `PageSetup`-Widgetbaum und Runtime-Anwendung auf `SensorManager` sowie `ExposureEngine`.
- Safety-relevante Thermikgrenzen sind nicht mehr nur Compile-Time-Konstanten: Defaults bleiben Migrationsanker, die laufenden Werte kommen ueber den Systemsettings-Pfad. Offen sind expliziter Safety-Bestaetigungsdialog, Reboot-/SD-Abnahme und Hardwaretests fuer Derating/Hard-Stop.

Entscheidungspunkte:

- Bleibt Setup ein impliziter Sammelort fuer alles, was "nicht SG" ist?
- Oder wird Setup als klare globale Systemkonfiguration mit enger Scope-Regel
   aufgebaut?
- Welche historischen Einstellwerte tragen noch fotografisch oder betrieblich,
   welche gehoeren bewusst nicht in den neuen Menuepfad?

Empfohlene Richtung: Explizites Setup als eigener globaler Workflow mit
strikter Scope-Regel. Verbindlich hinein gehoeren:

- Sound-Modus
- Lautstaerke
- Vibration an/aus
- Max. NeoPixel-Helligkeit
- Thermikschwelle fuer Leistungsreduktion
- Thermikschwelle fuer Abschalten

Zusaetzlich sinnvoll:

- globaler EV-/F-Stop-Schritt
- Display-Helligkeit nur dann, wenn der reale Part2-TFT-Pfad dafuer
   hardwareseitig belegbar dimmbar ist

Bewusst nicht im Setup v1:

- Papierprofile, ISO-P/ISO-R, SG-LUT, Preflash
- Messquellen-Umschalter lokal/remote
- Dark-Calibration, USB-Bridge, Pairing, Error-Reset
- sonstige Service-/Maintenance-Aktionen

Warum: Part2 braucht eine sichtbare globale Systemkonfiguration, aber keine
Rueckkehr zum historischen Sammelmodus. Safety, Papierwahrheit und Servicepfade
bleiben besser beherrschbar, wenn das Setup-Menue nur echte Systemparameter
enthaelt.

-> Explizites Setup als eigener Workflow mit enger Scope-Regel. Pflichtwerte:
    Sound-Modus, Lautstaerke, Vibration, NeoPixel-Maxhelligkeit,
    Derating-Schwelle und Hard-Stop-Schwelle. Zusaetzlicher v1-Kandidat:
    globaler EV-/F-Stop-Schritt. Service- und Papierpfade bleiben getrennt.

Nachtrag 2026-05-06: Der Pflichtumfang ist umgesetzt, `HEAD DIAG` ist als nicht persistenter Bring-up-Hook zulaessig. Nicht umgesetzt und weiterhin offen ist der optionale globale EV-/F-Stop-Schritt.

## E18: Welcher Dateisystemvertrag gilt auf dem Teensy?

Status: Entscheidung gesetzt und im Build abgesichert.

Aktueller Befund:

- Der Teensy-Code nutzt bereits direkt `SdFs`, `FsFile` und `SdioConfig(FIFO_SDIO)`
   fuer Paper-, Systemsettings- und VFS-Pfade.
- Die wiederkehrende `SdFat.h`-Warnung zu `FS.h` entsteht nicht durch ein fehlendes
   Headerfile, sondern weil das Teensy-Framework bereits ein eigenes `FS.h` und
   einen globalen `File`-Typ mitbringt.
- Ein Wechsel auf `SD.h` wuerde den Unterbau nicht vereinfachen, sondern nur auf
   einen Wrapper ueber denselben SdFat-Kern wechseln.

Entscheidung:

- Der Teensy-Dateisystempfad bleibt verbindlich auf nativer SdFat-API.
- Erlaubte Projekttypen sind `TeensyStorageVolume` und `TeensyStorageFile` aus
   `src/teensy/TeensyStoragePolicy.h`.
- `SD.h`, `FS.h`, direkte `SdFat.h`-Includes ausserhalb des Policy-Headers und
   bare `File` sind im Teensy-Quellpfad verboten.
- Die upstream-Warnung `File not defined because __has_include(FS.h)` wird erst
   deshalb zentral unterdrueckt, nachdem ein Pre-Build-Check diesen Vertrag
   durchsetzt.

Warum: Der Code nutzt den SdFat-SDIO-Pfad nicht zufaellig, sondern wegen der
klaren Kontrolle ueber `SdFs`, `FsFile` und das native SDIO-Mounting. Damit
bleiben VFS, Paper- und Settings-Persistenz technisch aus einem Guss, statt
zwischen generischem Arduino-FS und nativen SdFat-Typen zu driften.

-> Teensy-Storage bleibt nativer SdFat-Pfad mit zentralem Policy-Header,
    Pre-Build-Verbot fuer `SD.h`/`FS.h`/direkte `SdFat.h`/bare `File` und erst
    danach gezielter Unterdrueckung der upstream-Kompatibilitaetswarnung.
