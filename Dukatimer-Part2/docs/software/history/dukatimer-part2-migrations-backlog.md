# Dukatimer-Part2 Migrations-Backlog

Stand: 2026-04-30

## Ziel

Dieser Backlog uebersetzt die Analyse von v0.3, v0.9 sowie den historischen Zwischenstaenden 2.30, 2.240 und B1.0 in konkrete Arbeitspakete fuer Dukatimer-Part2.

Leitlinien:

- v0.9 ist die Struktur-Basis.
- v0.3 ist die Verhaltens-Referenz.
- 2.240 ist der frueheste historische Stand mit nahezu vollstaendigem Kernfunktionsumfang.
- Teensy 4.1 traegt die Kernlogik, ESP32-S3 bleibt Service-MCU.
- Encoder- und UI-Grundlagen werden vor den Fachmodi festgezogen.
- Wireless ist von Beginn an Architektur- und Integrationsbestandteil.
- Der erste fachliche Slice besteht aus Papierkalibrierung, Modus SG, SG-Mathematik und Belichtungslogik.

Dieser Stand wurde gegen den aktuellen Codestand (src/teensy) sowie gegen die historische Closed-Loop- und SG-Referenz aus v0.3/v0.9 abgeglichen.

## Prioritaetslogik

- Muss: erforderlich fuer eine fachlich vollwertige Part2-Version
- Soll: sinnvoll fuer Funktionsparitaet und Bedienqualitaet
- Kann: optionale Erweiterung nach stabiler Kernfunktion

## Statuslogik

- Erledigt: Abnahmekern fuer dieses Paket ist im aktuellen Stand erreicht
- Teilweise: wesentliche Teilziele umgesetzt, Abnahmekern noch unvollstaendig
- Offen: noch nicht oder nur als Platzhalter vorhanden

## Arbeitspakete mit Ist-Abgleich

| ID | Arbeitspaket | Prioritaet | Status 2026-04-29 | Historischer Bezug | Ist-Abgleich (kurz) | Naechster Schritt |
| --- | --- | --- | --- | --- | --- | --- |
| MB-01 | Encoder- und Inputarchitektur festlegen | Muss | Teilweise | v0.3/v0.9 Schnellzugriff und Eventsemantik | InputNormalizer und semantische Actions vorhanden, lokale LongPress/Repeat/Measure/Undo erzeugt; Encoder A/B weiterhin positionsbasiert (/4) statt quadraturvalidiert; zentrale EV-/F-Stop-Editsemantik fuer alle Belichtungsmodi ist noch nicht verbindlich festgezogen | quadraturvalidierten Decoder plus Lost-Step-Monitoring einziehen und gemeinsame EV-/F-Stop-Editsemantik festlegen |
| MB-02 | LVGL-/TFT-/Touch-Basis dokumentieren und initialisieren | Muss | Teilweise | v0.9 UI-Schichtung, Part2-LVGL-Zielbild | Display, Touch, LVGL, DMA-Flush und Runtime-Snapshot aktiv; UI ist weiter Debug-Placeholder, kein UiKernel/Screen-Navigation | SG-Hauptscreen plus Overlay-Kanal und Fokusmodell umsetzen |
| MB-03 | SharedProtocol und Wireless-Basisdienst | Muss | Teilweise | v0.3/v0.9 Inter-MCU und Wireless-Basis | versionierter Header, Sequenz, CRC, Heartbeat, Remote-Events, Wireless-Status und VFS-Basis vorhanden; TeensyCommand-/RemoteRender-Pfade sowie Diagnoseframes laufen produktiv, Measurement-Domain-Anschluss bleibt offen | Measurement-Domain-Integration auf den produktiven Linkpfad aufsetzen |
| MB-04 | Laufzeitbasis und sicherer Boot | Muss | Teilweise | v0.9 Runtime-Owner plus sichere Startfolge | getrennte Runtime-Dienste vorhanden, sicherer Startpfad und Off-Initialisierung vorhanden; globaler EventGuard und komplette Startfreigabe-Policy noch nicht vollstaendig | globalen Guard-Schritt vor Modusrouting und Boot-Readiness-Policy finalisieren |
| MB-05 | Teensy-HAL fuer Licht, Sensorik und Eingaben | Muss | Erledigt | v0.9 Trennung plus v0.3 Sicherheitspfad | Save/Focus/Room inkl. SaveLatch, SensorRuntimeStatus, Head-Arbiter, Exposure-Override und Runtime-Limit aktiv; lokaler TSL2561-Readout implementiert und integriert (SensorManager -> ExposureEngine) | Validierung vor Feldtest: Sensor-Verdrahtung pruefen, I2C-Budget an Hardwarelatenzen anpassen, Unit/Integration-Tests und Kalibrier-Flow ergaenzen |
| MB-06 | Zustandsmodell, Papierslots und Persistenz | Muss | Teilweise | v0.9 Storage plus 2.240 Profilkern | Zustandsmodelle und PaperExposureProfile-Struktur vorhanden; Basis fuer Papierslot-Persistenz mit Version/CRC ist angelegt, Restintegration, Migration und Testleitfaden bleiben offen | Restintegration, Migrationsregeln und dokumentierte Persistenztests abschliessen |
| MB-07 | Papierkalibrierung | Muss | Offen | 2.240 plus v0.3/v0.9 | kein Wizard- oder Arbeitspfad vorhanden; Messwerte werden noch nicht konsistent als EV plus Lux ausgegeben | Kalibrier-Wizard (Flow, Datenhaltung, UI) mit Lux-plus-EV-Darstellung starten |
| MB-08 | Modus SG mit Mathematik und ExposureEngine | Muss | Teilweise | v0.9 SG-Ablauf plus v0.3 Closed-Loop | SG-Execution-States, Command-Bridge, Start/Abort/WaitForFilter, ExposureEngine mit Closed-Loop, praediktivem Shutoff, Watchdog und Thermal-Hard-Stop vorhanden; EV-logarithmische Zielschritte sind in SG gestartet, aber SG-Mathematik, papiergetriebene Vorschlagslogik und die modusunabhaengige EV-/F-Stop-Basis fehlen | gemeinsame EV-/F-Stop-Basis plus SG-Math und paper-driven targets auf aktuelle Execution-States mappen |
| MB-09 | Wireless-Remote im ersten fachlichen Slice | Muss | Teilweise | v0.3/v0.9 Remote-Messgeraet | SG-spezifische Messkommandos und Render-Roundtrip sind produktiv verdrahtet; reale Remote-Hardwareintegration laeuft inkl. C6-Event-ABI-Abgleich (16 Byte); offen bleibt die fachliche Uebernahme der Luxdaten in die Measurement-Domain | Wireless-Luxdaten aus Gateway-Telemetrie in Measurement-Service ueberfuehren |
| MB-10 | Messpipeline und Histogramm-Session | Muss | Offen | v0.3/v0.9 Messsession | keine lokale Spot/Multi-Spot/Histogramm-Session im Part2-Workflow; zentrale Messwertaufbereitung in EV plus Lux fehlt | Mess-Session-Datenmodell, Undo-Logik und gemeinsame EV-/Lux-Formatter aufbauen |
| MB-11 | BW, Burn und Teststrip | Muss | Offen | 2.240 plus v0.3/v0.9 | BW aktuell nur Shell-Workflow, Burn/Teststrip offen; gemeinsame EV-/F-Stop-Basis fuer mehrere Belichtungsmodi ist noch nicht produktiv genutzt | BW zuerst auf gemeinsamer EV-/F-Stop-Basis produktiv machen, dann Burn/Teststrip anschliessen |
| MB-12 | Densitometer und Filmtest | Muss | Offen | 2.30 plus v0.3/v0.9 | nicht implementiert; fotografische Messwertdarstellung in EV plus Lux fehlt | REF/BASE/MEAS-Flow mit Zone-I/VIII-Helfern sowie EV-/Lux-Darstellung aufsetzen |
| MB-13 | Preflash und Flash-Kalibrierung | Muss | Offen | 2.240 plus B1.0 plus v0.9 | nur Profilstruktur vorbereitet, kein Laufzeitpfad; keine Einbindung in eine gemeinsame EV-/F-Stop-Schicht | Flash-Wizard und Preflash-Execution auf derselben EV-/F-Stop-Basis implementieren |
| MB-14 | ESP-Serviceintegration jenseits Wireless | Muss | Teilweise | Part2-Rollenmodell plus v0.9 Servicepfad | reale 1-Wire- (DS18B20) und AHT-Zusatzsensorik inkl. Fehlerflags und Diagnoseframes laufen produktiv; VFS-Bruecke ist integriert; offen sind weiterfuehrende Sensorabdeckung und Feldvalidierung | Feldvalidierung und ggf. weitere Zusatzsensorik auf demselben Diagnosepfad erweitern |
| MB-15 | LiveView, modusunabhaengige F-Stop-/EV-Bedienebene und Zone-Modus | Soll | Offen | v0.9 Zusatzmodi | LiveView und Zone sind nicht implementiert; die konkrete F-Stop-Oberflaeche fuer einzelne Modi darf erst auf der zuvor zentral definierten EV-/F-Stop-Logik aufsetzen | nach MB-10/11 sequenziell einziehen und nur noch UI-spezifisch auspraegen |
| MB-16 | Erweiterte Wireless-Komfortfunktionen | Kann | Offen | v0.3/v0.9 Komfortebene | Basis vorhanden, Komfortebene offen | erst nach stabilem MB-09/MB-14 starten |

## Erledigte Punkte seit dem vorherigen Backlog-Stand

- MB-03 ist als Protokoll-Basiskern weit fortgeschritten, aber noch nicht vollstaendig abgeschlossen.
- Innerhalb MB-08 wurden zentrale historische Ablaufpunkte umgesetzt:
  - SG-Execution-States inkl. WaitForFilter und Abort-Route
  - Command-Bridge Workflow -> ExposureEngine statt ad-hoc Trigger
  - Closed-Loop-Dosisintegration mit praediktivem Shutoff
  - Sensor-Watchdog, Null-Lux-Plausibilitaet und Thermal-Hard-Stop
- Innerhalb MB-05 wurde die Head-Kopplung erweitert:
  - aktiver Exposure-Override auf dem Head-Arbiter
  - RuntimeOutputLimit-Skalierung
  - deterministische Soft-Start/Soft-Stop-Rampen

## Repriorisierte naechste Schritte

### P0 (direkt, blockierend fuer reproduzierbaren Fachbetrieb)

1. MB-01 und MB-08: gemeinsame EV-/F-Stop-Grundlogik und Messwertkonvention (EV plus Lux) zentral definieren, damit weitere Belichtungsmodi nicht jeweils eigene Mathematik oder Formatter mitbringen.
2. MB-06: Restintegration der Papierslots plus dokumentierte Version/CRC-Teststrategie abschliessen, damit MB-07/MB-08 fachlich belastbar werden.
3. MB-02: Debug-UI in einen minimal bedienbaren SG-Hauptscreen mit Fehler-/Confirm-Overlay ueberfuehren.

### P1 (naechster Sprint)

1. MB-07: Papierkalibrierungs-Wizard mit Datenhaltung, Rueckschreibelogik und EV-/Lux-Darstellung.
2. MB-08: SG-Mathematik und Vorschlagsbildung gegen die neuen Execution-States integrieren.
3. MB-03, MB-09 und MB-14: Wireless-Remote-Messkommandos, Renderpfade und erweiterte ESP-Servicepfade durchgaengig anbinden.

### P2 (anschliessender Funktionsausbau)

1. MB-10: Messpipeline, Histogramm-Session und gemeinsame EV-/Lux-Aufbereitung.
2. MB-11: BW, Burn, Teststrip auf der zentralen EV-/F-Stop-Basis.
3. MB-12 und MB-13: Densitometer/Filmtest sowie Preflash/Flash-Kalibrierung.
4. MB-15 und MB-16: LiveView, modusspezifische F-Stop-Oberflaechen, Zone und Wireless-Komfortfunktionen.

## Lieferstufen mit aktuellem Fortschritt

### Stufe A: technische Basis

- Umfang: MB-01 bis MB-05
- Fortschritt: in Arbeit (MB-01/02/03/04/05 teilweise)

### Stufe B: erster fachlicher Slice

- Umfang: MB-06 bis MB-09
- Fortschritt: in Arbeit (MB-08 deutlich vorangeschritten, MB-06/09 teilweise, MB-07 offen)

### Stufe C: schrittweiser Funktionsausbau

- Umfang: MB-10 bis MB-14
- Fortschritt: gestartet (MB-14 teilweise, MB-10/11/12/13 offen)

### Stufe D: Ausbau

- Umfang: MB-15 bis MB-16
- Fortschritt: offen
