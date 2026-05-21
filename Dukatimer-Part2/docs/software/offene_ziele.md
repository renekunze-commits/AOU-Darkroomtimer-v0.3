# Offene Ziele

Stand: 2026-05-06

Dieses Dokument beschreibt den verbleibenden Produkt- und Entwicklungszielraum fuer `Dukatimer-Part2`. Es ist bewusst funktionsorientiert: nicht „welche Datei fehlt“, sondern welcher nutzbare Dunkelkammerwert noch entstehen muss.

## Zielbild

`Dukatimer-Part2` soll ein duales MCU-Dunkelkammer-System werden:

- Teensy 4.1 bleibt Autoritaet fuer UI, Eingabe, Exposure, Kopf, Papierdaten, Sicherheit und lokale Messung.
- ESP32-S3 bleibt Service-MCU fuer Encoder 4, Zusatzsensorik, HTTP/VFS und ESP-NOW-Gateway.
- ESP32-C6 Wireless TSL2591 bleibt Terminal/Messgeraet, nicht zweite Belichtungsautoritaet.
- Fotographische Mathematik wird zentral und nachvollziehbar gefuehrt; Workflows nutzen diese Dienste, statt eigene EV-/Dosisformeln zu erfinden.

## Z0: Status und Dokumentationshygiene

Ziel: Die drei Dateien `offene_punkte.md`, `offene_ziele.md` und `offene_entscheidungen.md` sind kuenftig die Einstiegsebene fuer Planung und Review.

Lieferumfang:

- Alte Audit-, Vergleichs- und Arbeitspaketdokumente bleiben historisch erhalten, werden aber nicht mehr als alleinige Wahrheit verwendet.
- Changelog und neue Arbeitspakete verweisen auf diese drei Dateien, wenn sie offene Punkte schliessen.
- Firmwareversionen und Protokollversion werden bei jedem relevanten Release aktualisiert.

Akzeptanz:

- Jeder neue „erledigt“-Claim nennt Codepfad, Firmwareversion und Validation.
- Scheinloesungen werden nicht als Featureabschluss gefuehrt.

## Z1: Remote-/Servicepfad produktiv machen

Ziel: Wireless-Terminal und ESP32-S3-Servicepfad werden vom Bring-up-Pfad zu einem robusten Bedien- und Messpfad.

Lieferumfang:

- Remote-Kommandopfad nach dem nun realen C6-Forwarding um einen belastbaren Ack-/Status-Rueckkanal fuer `commandSequence` erweitern.
- Ack-/Status-Rueckkanal fuer `commandSequence` weiter ausbauen: von den jetzt lokalen und bis zum C6 telemetrierten `RMT`-Zaehlern zu optionalem HTTP-Export oder SD-Ablage der Diagnosedaten und ggf. weiteren Command-Klassen mit eigener Policy.
- Explizite Diagnose oder bewusstes Monitoring fuer stale Render und laengeren Renderverlust entlang C6, Gateway und Teensy nachziehen.
- C6-Firmwareversion einfuehren und in Bootlog/Diagnose sichtbar machen.
- SharedProtocol-Sync zwischen Part2 und C6 automatisieren oder pruefbar machen.
- MAC-Pairing fuer genau ein C6-Terminal mit sichtbarem Peerstatus und Resetpfad umsetzen.
- Optional: Render-Ack oder minimale Display-Freshness im Wireless-Snapshot.

Akzeptanz:

- Ein Remote-Messstart vom Teensy fuehrt nachweisbar zu Terminalzustand, Rueckmeldung und sichtbarer Ack-Sequenz.
- Alte Renderpakete werden vom C6 verworfen.
- Protokolldrift wird vor Build/Release erkannt.

## Z2: Produktive LVGL/EEZ-Bedienoberflaeche

Ziel: Der Placeholder-Screen wird durch eine echte Bedienoberflaeche fuer Dunkelkammerbetrieb ersetzt.

Status 2026-05-06: Der alte Stub-Block ist code-/buildseitig geschlossen. Boot, Busy, Splitgrade, PaperWorkspace, Setup, Measurement und WirelessRemote besitzen handgeschriebene LVGL-Widget-Baeume und werden von `LvglUi::pushWidgetsFromSnapshot()` aus `SystemSnapshot` gespeist. Offen bleibt, dass dieser Stand code-first ist, noch kein finales `.eez-project` besitzt, PageMeasurement die Presenter-/Formatter-Schicht teilweise umgeht und die Hardware-/Encoder-/Touch-Abnahme fehlt.

Lieferumfang:

- [umgesetzt, Abnahme offen] Hauptscreen fuer aktive Belichtung mit grossen Zeit-/Dosis-/Statuswerten; aktueller Code-Stand ist ein priorisierter `SCREEN_ID_BUSY` mit Zeit-/Dosisfortschritt, SG-Soft/Hard-Detail und Pause-Aktionen, aber noch ohne finalen EEZ-Designer-/Encoder-Abnahmestand.
- [umgesetzt, Abnahme offen] Splitgrade-Screen fuer Gradation, Soft/Hard-Anteile, Filterwechsel und Paper-Profil.
- BW-Screen fuer gemischte Lichtbelichtung.
- [teilweise umgesetzt] Messscreen fuer Spot/Multi-Spot, Histogramm, aktive Quelle und Undo. Widgetbaum und Snapshot-Push existieren; offen sind Formatter-Konformitaet, Rollen-/Proposal-Anzeige, reale Messabnahme und vollstaendige Bedienpfade.
- [teilweise umgesetzt] Paper-Profil-Auswahl und kalibrierbare Profilwerte mit klarer Anzeige von Demo/gemessen/importiert. SELECT/CAL und Profilwerte existieren; Demo/gemessen/importiert ist noch nicht ausreichend sichtbar.
- [umgesetzt, Abnahme offen] Explizites Setup-Menue fuer globale Systemkonfiguration statt versteckter Konstanten oder Debugpfade. Pflichtumfang: Sound-Modus, Lautstaerke, Vibration an/aus, NeoPixel-Maxhelligkeit, Thermikschwelle fuer Leistungsreduktion und Thermikschwelle fuer Abschalten. Schnitt und Begrenzung sind in [explizites-setup-menue-zielvorgabe.md](explizites-setup-menue-zielvorgabe.md) festgelegt; offen sind Hardware-/Reboot-/Safety-Dialog-Abnahme.
- Fehler-/Warnmodal fuer Sensor, Link, SD, Thermik, I2C und Exposure-Faults.
- Touch/Encoder-Fokus bleibt sichtbar und mit InputRouterPolicy konsistent.

Akzeptanz:

- Bedienung ist mit Encodern allein moeglich.
- Kritische Zustaende sind auf Teensy-Display und C6-Terminal konsistent erkennbar.
- Systemweite Einstellungen sind sichtbar, versioniert persistiert und nicht mehr nur als rohe Konstanten, Shell-Defaults oder Service-Nebenpfade versteckt.
- Keine produktive Funktion versteckt sich nur in Debuglabels.
- Messwerttexte laufen ueber Presenter/Formatter und nicht direkt ueber Screen-spezifische `snprintf`-Inseln.

## Z3: Messdomain, Spot und Multi-Spot

Ziel: Lokale und wireless Messungen werden zu einem fotografisch belastbaren Messworkflow.

Status 2026-05-06: `MeasurementDomainService`, zentrale Formatter und ein sichtbarer Measurement-Screen existieren. Der aktuelle Zustand ist aber weiterhin session-relativ und quellengebunden: keine absolute Zone, keine Dichte/LogD, keine automatische Papier- oder Belichtungsempfehlung ohne Proposal-Eligibility. Die lokale PageMeasurement-Darstellung muss noch auf Presenter/Formatter umgestellt werden.

Lieferumfang:

- Lux/EV/Zonen-Referenz festlegen und `MeasurementDomainService` daran anbinden.
- Spotmessung mit aktueller Quelle, Sample-Age, Messsequenz und Undo.
- Multi-Spot-Session mit definiertem Histogramm, Highlights/Shadows und Papierbezug.
- Wireless-Measure-Button als bewusster Session-Sample-Trigger.
- Lokaler TSL2561 und Wireless TSL2591 werden nicht vermischt, ohne Quelle/Alter sichtbar zu machen.
- Sensorrollen gemaess Entscheidung festziehen: TSL2561 lokal als closed-loop Sensor am Kopfpfad, TSL2591 remote als Papier-/Transmissionsmessung; definierter Erstabgleich ohne Negativ.
- [teilweise umgesetzt] Messwertformatierung zentral erweitern: Lux, EV, Zone, Dichte/LogD falls Densitometerziel aktiv wird. Lux/EV/Range/Controls sind als Formatter vorhanden; Zone/Dichte/LogD und die direkte PageMeasurement-Nutzung sind offen.
- Die vorsichtige Proposal-Mathematik auf Basis der aktuellen Measurement-Range
  ist in [dukatimer-part2-ap11f-measurement-proposal-math-todo.md](dukatimer-part2-ap11f-measurement-proposal-math-todo.md)
  als Analyse- und Umsetzungsschnitt dokumentiert.
- Der schonungslose Audit der aktuellen Measurement-Logik und Mathematik ist in
  [dukatimer-part2-ap11g-measurement-logic-math-audit.md](dukatimer-part2-ap11g-measurement-logic-math-audit.md)
  dokumentiert; er blockiert Proposal-Code auf History-/Source-/UI-Hygiene.

Akzeptanz:

- Histogramm-Buckets haben dokumentierte fotografische Bedeutung.
- Wireless und lokale Messwerte liefern reproduzierbare Ergebnisse gegen Referenz.
- Undo und Sequenzbarrieren verhindern doppelte Wireless-Samples.
- Vorschlaege bleiben Preview/Accept-gebunden und werden nie direkt aus relativer Measurement-Range aktiv gesetzt.

## Z4: Papier- und Kalibrierworkflow

Ziel: Papierdaten werden nicht nur gespeichert, sondern im Geraet messbar gepflegt und fachlich nachvollziehbar genutzt.

Lieferumfang:

- Kalibrierwizard fuer Papierprofile: Basiszeit/Dosis, ISO-P, ISO-R, Soft/Hard-Faktoren, 0,5er Gradations-LUT.
- Defaultprofile mit Quellenstatus: Demo, historisch, gemessen, importiert.
- Ilford MGIV RC und Foma Variant 311 nur dann als echte Defaults markieren, wenn Messdaten vorliegen.
- Demo-Defaults bleiben explizit als Demo markiert, bis reale Protokolle den Status auf gemessen/importiert heben.
- Profil-Editor mit SD-Persistenz, Fehleranzeige und Import/Export ueber VFS.
- Preflash-Werte aus Datenstruktur in echten Workflow ueberfuehren.

Akzeptanz:

- Ein neues Papier kann am Geraet kalibriert, gespeichert, geladen und in SG/BW genutzt werden.
- Korrupte Persistenzdaten werden sichtbar gemeldet und nicht still ueberschrieben.

## Z5: Belichtungsworkflows komplettieren

Ziel: Historische Dukatimer-Kernfunktionen werden als echte Part2-Workflows portiert, nicht nur als Datenmodelle.

Lieferumfang:

- Splitgrade: bestehendes SG weiter haerten, Filterwechsel/Remote/UI finalisieren, ISO-R real kalibrieren.
- BlackWhite: gemischte Soft/Hard-Kanalueberblendung, Gradations-/Papierbezug, Zeit- und Dosisfuehrung; detaillierter Einfuehrungsplan in [dukatimer-part2-bw-modus-umsetzungsplan-2026-05-06.md](dukatimer-part2-bw-modus-umsetzungsplan-2026-05-06.md).
- Teststrip: Streifenmodell, Stufenlogik, Maskier-/Sequenzanzeige, Exposure-Kommandos.
- Burn/Dodge: Nachbelichtung mit definierter EV-/Zeit-/Dosis-Semantik und sicherem Abbruch.
- Preflash/Flash: Schwellwert, Faktor, Kanalwahl und Kalibrierablauf.
- Densitometer/Filmtest: nur aufnehmen, wenn Messgeometrie, Sensor und mathematischer Umfang entschieden sind.

Akzeptanz:

- Jeder Modus hat eigene UI, Input-Semantik, Exposure-Kommandos, Tests und Hardware-Abnahmepunkte.
- Kein Modus ist nur Enum, Header oder Shell.

## Z6: Exposure, Kopf und Sensorik validieren

Ziel: Die vorhandene ExposureEngine wird gegen reale Hardware abgesichert.

Lieferumfang:

- Dosislinearitaet ueber definierte Luxbereiche und Zeiten pruefen.
- Head-Latenz fuer NeoPixel-Present, SSR, Sensorposition und praediktiven Shutoff messen.
- Thermische Schwellen und DS18B20-Position validieren.
- [umgesetzt, Abnahme offen] Safety-relevante Thermikschwellen aus der bisherigen Konstantenlage in einen service-gesicherten, plausibilitaetsgeprueften Systemsettings-Pfad ueberfuehren. Der Runtime-Pfad schreibt `SystemSettings` in `SensorManager` und `ExposureEngine`; offen sind Hardware-/Reboot-/Grenzfallvalidierung und ein expliziter Safety-Bestaetigungsdialog.
- Stromaufnahme und Kanalbegrenzung des NeoPixel-Kopfs messen.
- I2C-Hangstrategie fuer lokalen Sensor final umsetzen.
- HeadCalibrationProfile wirklich in die Output-Pipeline integrieren, falls fachlich gewuenscht.

Akzeptanz:

- Belichtungsende liegt innerhalb definierter Toleranz.
- Sensor-/Busfehler fuehren zu vorhersehbaren Fallbacks oder Faults.
- Head-Ausgabe ist fuer BW/SG/Preflash kalibriert statt nur farblich plausibel.

## Z7: Storage, VFS und Servicewerkzeuge

Ziel: SD/VFS/HTTP bleiben nuetzlich als Produkt-Konfigurationspfad, ohne Safety oder Datenintegritaet zu gefaehrden.

Lieferumfang:

- Nicht-blockierenden Teensy-TX jetzt gegen reale CTS-/Backpressure-Hardware validieren; der Codevertrag lautet `Command/VFS > Heartbeat > Render > Diagnose` bei kleinem Flush-Budget pro Aufruf.
- VFS-Uploads gegen aktive Belichtung, Linkverlust, SD-Fehler und Abbruch testen.
- Import-/Exportformat fuer PaperProfile und spaetere UI-Assets definieren.
- Globale `SystemSettings` getrennt von Paperprofilen persistieren, versionieren und gegen korrupte Daten absichern.
- Paperprofile/Backups als Produkt-Konfiguration ueber HTTP robust machen.
- Belichtung waehrend aktiver Websession sicher blockieren und den Sperrzustand sichtbar diagnostizieren.
- Entscheiden, ob PSRAM/Web-Asset-Caching spaeter Teil des Produkts wird.
- Zugriffsschutz fuer HTTP-Service klaeren.

Akzeptanz:

- Upload kann Exposure nicht stoeren.
- Fehler sind per HTTP und Teensy-UI diagnostizierbar.
- Produktdaten sind versioniert und recoverbar.

## Z8: Test- und Abnahmesystem

Ziel: Regressionen werden frueh sichtbar, Hardwareclaims werden reproduzierbar.

Lieferumfang:

- Host-/PIO-Tests fuer SharedProtocol, FrameDecoder, PaperSlotCodec, MeasurementDomain, InputRouter und ExposureEdge-Cases.
- SG-Akzeptanztests um Randfaelle fuer nicht exakt 0,5er Importgrade und ISO-R-Grenzen erweitern.
- Buildmatrix fuer `teensy41`, `esp32s3_n16r8` und Wireless C6 dokumentieren.
- Hardware-Abnahmeprotokolle fuer Sensorik, Head, Remote, VFS und UI.

Akzeptanz:

- Jeder Workpackage-Abschluss nennt Build, Test und ggf. Hardwaremessung.
- Bekannte Baseline-Warnungen sind getrennt von neuen Warnungen dokumentiert.

## Zielprioritaet

1. Safety und Wahrheit: I2C-Hang, Hardwarevalidierung.
2. Remote-Vertrauen: Commands, C6-Freshness, Versionierung, Protocol-Sync.
3. Bedienbarkeit: produktive LVGL/EEZ-UI und Remote-Anzeige.
4. Fotografische Tiefe: Messdomain, Paper-Kalibrierung, BW/Teststrip/Burn/Preflash/Densitometer.
5. Wartbarkeit: Tests, VFS-Produktpfad, Dokumentationshygiene.
