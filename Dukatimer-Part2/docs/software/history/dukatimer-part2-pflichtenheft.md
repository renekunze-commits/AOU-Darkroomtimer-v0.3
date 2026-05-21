# Dukatimer-Part2 Pflichtenheft

Stand: 2026-04-20

Grundlage dieses Dokuments ist die Analyse der drei Codebasen und der historischen Zwischenstaende:

- Dukatimer v0.3
- Dukatimer v0.9
- Dukatimer-Part2
- 2.30 (Densi_AVG)
- 2.240 (Probestreifen)
- B1.0 (Probestreifen + Smart Math)

Ziel ist die technische Festlegung, welche Funktionen Dukatimer-Part2 uebernehmen soll, wie diese auf der neuen Dual-MCU-Hardware sinnvoll verteilt werden und wie die Umsetzbarkeit einzuschaetzen ist.

## 1. Kurzfazit

Dukatimer-Part2 ist grundsaetzlich gut umsetzbar, aber nicht als 1:1-Port einer bestehenden Firmware. Die fachliche Logik aus Dukatimer v0.3 und vor allem die modulare Architektur aus Dukatimer v0.9 koennen uebernommen werden, die Hardware-nahe Schicht muss fuer Dukatimer-Part2 jedoch weitgehend neu erstellt werden.

Die wichtigste Architekturentscheidung lautet:

- Teensy 4.1 ist das Hauptsystem fuer Echtzeit, UI, Belichtung und sicherheitskritische Ausgaenge.
- ESP32-S3 ist das Service-System fuer Zusatzsensorik, Encoder 4, Kommunikation und optionale Komfortfunktionen.

Empfohlene Software-Basis:

- v0.9 fuer Zustandsmodell, Modusverwaltung, Persistenzmodell und grundsaetzliche Modultrennung
- v0.3 als Referenz fuer bewiesenes Zielverhalten, Messablaeufe, Timing-Details und Randfaelle

Verbindliche Startprioritaeten fuer Part2:

- Encoder- und Inputarchitektur werden vor den Fachmodi fest definiert.
- Die UI-/LVGL-/TFT-Basis wird frueh dokumentiert und nicht erst waehrend der Modusimplementierung improvisiert.
- Die Wireless-Anbindung wird von Beginn an im Architektur- und Protokollmodell beruecksichtigt.
- Die erste fachliche Implementierungsstufe fokussiert auf Papierkalibrierung, Modus SG, die erforderliche Mathematik und die dazugehoerige Belichtungslogik.

## 2. Analyse der Ausgangsbasis

## 2.1 Dukatimer v0.3

Die v0.3-Codebasis ist funktional breit, aber stark monolithisch aufgebaut. Sie basiert auf globalem Zustand, FreeRTOS-Tasks und hardware-nahen Modulen.

Erkannte Kernfunktionen:

- BW-Belichtung
- Splitgrade-Belichtung
- spektrale Spotmessung mit Histogramm
- automatische Vorschlagsberechnung auf Basis von Messpunkten
- Teststrip-Modus
- Burn-Modus
- Papier-Kalibrierung
- Densitometer-Modus
- Setup und Speicherung von Einstellungen
- Display-Ausgabe auf Nextion und 16x2-LCD
- Wireless-TSL2591-Handgeraet per ESP-NOW

Architekturmerkmale:

- Betriebsmodi ueber globale Enums und verteilte Handler
- ExposureEngine mit dosisgeregelter Closed-Loop-Belichtung
- Messlogik eng mit Hardware und Funkpfad gekoppelt
- starke Abhaengigkeit von der bisherigen ESP32-S3-Einplatinen-Hardware

Die Wireless-TSL2591-Einheit ist in v0.3 funktional kein reiner Funksensor, sondern ein abgesetztes Handmessgeraet. Sie kombiniert TSL2591, OLED, Taster, Encoder und Haptik, sendet Messwerte sowie Eingabe-Events an das Hauptgeraet und erhaelt von dort Render- und Messkommandos.

Wichtige Einschraenkungen aus der Analyse:

- Auto-SG und Teile der Messkopplung sind in v0.3 nicht in allen Faellen robust.
- Die Messlogik ist teilweise auf eine drahtlose Probe zugeschnitten.
- UI und HAL sind hart an Nextion, LCD, Pinout und bisherige Relais-/NeoPixel-Struktur gebunden.

## 2.2 Dukatimer v0.9

Die v0.9-Codebasis ist eine deutliche Reorganisation der Fachlogik. Sie fuehrt eine modulare App-/Manager-Architektur ein und ist damit die bessere Grundlage fuer Dukatimer-Part2.

Erkannte Kernfunktionen:

- BW Zeit-Modus
- BW Dosis-Modus
- SG Zeit-Modus
- SG Dosis-Modus
- fotografische F-Stop-/EV-Bedienlogik (in v0.9 explizit im BW-Kontext belegt)
- Zone-Modus
- LiveView
- Teststrip
- Burn
- Preflash
- Papier-Kalibrierung
- Densitometer
- Setup
- Paper- und Storage-Management
- Wireless-Protokoll fuer ein externes TSL2591-Handmessgeraet

Architekturmerkmale:

- SystemContext als zentrale, thread-sichere Zustandsquelle
- AppManager fuer Moduswechsel und Event-Routing
- HardwareManager fuer die Hardware-Abstraktion
- SensorManager, ExposureEngine, DisplayManager, InputManager, StorageManager und PaperManager als getrennte Dienste
- deutlich besser portierbar und testbarer als v0.3

Einschraenkungen fuer Dukatimer-Part2:

- Auch v0.9 ist weiterhin auf eine einzelne ESP32-S3-Plattform zugeschnitten.
- DisplayManager und InputManager sind auf Nextion/LCD und das alte Pinning ausgelegt.
- Wireless-Funktionen passen nur eingeschraenkt zur neuen, kabelgebundenen Part2-Hardware.

## 2.3 Dukatimer-Part2 Ist-Zustand

Der aktuelle Stand von Dukatimer-Part2 ist eine Hardware- und Projektstruktur ohne eigentliche Firmware-Implementierung.

Vorhanden:

- PlatformIO-Workspace mit getrennten Targets fuer Teensy 4.1 und ESP32-S3
- KiCad-Schaltplan fuer die Dual-MCU-Hardware
- dokumentierte Topologie der beiden Controller und der Peripherie

Fehlend:

- Hauptanwendung auf Teensy
- Service-Firmware auf ESP32-S3
- SharedProtocol zwischen beiden MCUs
- Zustandsmodell und Moduslogik
- Display-/Touch-UI
- neue HAL fuer Teensy-Ausgaenge, Sensorik und ESP-Dienste

## 3. Zielarchitektur fuer Dukatimer-Part2

## 3.1 Rollenverteilung der MCUs

### Teensy 4.1

Der Teensy ist das fachliche Hauptsystem.

Verifizierte Hardwareergaenzung 2026-05-03:

- Die aktuelle Duka-Teen-Revision meldet im isolierten Teensy-PSRAM-Probe-Sketch
	`16 MB` externes PSRAM.
- Die Hardwarevorgabe fuer Part2 lautet damit `16 MB` externes PSRAM am Teensy
	4.1; grosse, nicht zeitkritische Datenbereiche sind bevorzugt per `EXTMEM`
	auszulagern.

Pflichtaufgaben des Teensy:

- zentrale Zustandsmaschine des Dukatimers
- lokale UI auf TFT und Touch
- Verarbeitung der Encoder 1 bis 3 und der lokalen Taster ueber eine explizit definierte, Teensy-zentrierte Input-Schicht
- ExposureEngine fuer Zeit- und Dosisbelichtung
- direkte Ansteuerung der NeoPixel-Kanaele
- Raumlicht-SSR und weitere sicherheitskritische Ausgaenge
- Spotmessung ueber den kabelgebundenen Sensorkopf an I2C1
- Verwaltung von Papierprofilen und lokalen Einstellungen
- Erzeugung des Audio-PWM-Signals

### ESP32-S3

Der ESP32-S3 ist das Service- und Erweiterungssystem.

Pflichtaufgaben des ESP32-S3:

- Verarbeitung von Encoder 4
- 1-Wire-Auswertung
- U-SENS1 beziehungsweise Zusatzsensorik
- Service- und Kommunikationsfunktionen
- Vibrationsfeedback und servicebezogene Audio-Funktionen
- Weitergabe von Events und Messwerten an den Teensy
- ESP-NOW-Gateway fuer das Wireless-TSL2591-Handgeraet von Beginn an

## 3.2 Architekturprinzipien

- Sicherheitskritische Funktionen duerfen nicht vom ESP32-S3 abhaengen.
- Eine laufende Belichtung muss auch dann sicher beendet werden, wenn der ESP32-S3 ausfaellt oder neu startet.
- Die fachliche Hauptlogik lebt auf dem Teensy.
- Die Inter-MCU-Kommunikation dient zur Erweiterung, nicht zur Auslagerung sicherheitskritischer Kernlogik.
- Encoderrollen, Eventsemantik und Eingabeschichten werden vor der Modusimplementierung verbindlich festgelegt.
- Die Encoder 1 bis 3 werden auf dem Teensy ueber eine interrupt- oder hardwaregestuetzte Quadraturauswertung mit normierten Drehereignissen erfasst; die aufwaendige zeitbasierte Soft-Entprellung der alten ESP32-Staende wird dafuer nicht als Primaerstrategie uebernommen.
- Encoder-Taster und sonstige diskrete Tasten erhalten eine getrennte, definierte Debounce-Schicht.
- LVGL-, TFT-, Touch- und Navigationsgrundlagen werden in einer eigenen Basisdokumentation festgelegt, bevor fachliche Screens proliferieren.
- Die Wireless-Remote ist vom ersten Integrationsstand an im Nachrichtenmodell, Statusmodell und UI beruecksichtigt, auch wenn spaetere Komfortfunktionen schrittweise folgen.
- Die lokale Schalterlogik fuer Fokus, Save und Room wird als zentrale Licht- und Umfeldhierarchie frueh festgelegt; Save hat Prioritaet vor Fokus und die SaveLatch-Logik wird uebernommen.

## 3.3 Inter-MCU-Protokoll

Zwischen Teensy und ESP32-S3 ist ein neues, explizit versioniertes SharedProtocol erforderlich.

Das Protokoll muss mindestens folgende Nachrichtengruppen enthalten:

- Heartbeat und Boot-Status
- Input-Events vom ESP an den Teensy
- Sensordaten vom ESP an den Teensy
- Service-Kommandos vom Teensy an den ESP
- Funk-Gateway-Nachrichten fuer Wireless-Remote-Input, Renderdaten und Messkommandos
- Fehler- und Diagnosemeldungen in beide Richtungen
- Einstellungs- und Status-Snapshots zur Synchronisation

Pflichtmerkmale des Protokolls:

- Header mit Version, Typ, Laenge und Sequenznummer
- Pruefsumme oder CRC
- Timeouts und Wiederanlaufverhalten
- definierte Default-Reaktion bei Verbindungsverlust

## 3.4 Funktion Wireless TSL2591

Die Wireless-TSL2591-Funktion ist fuer Dukatimer-Part2 eine von Beginn an beruecksichtigte externe Erweiterung in Form eines abgesetzten Handmess- und Fernbediengeraets.

Typische Hardware des Geraets:

- TSL2591 als Messsensor
- OLED zur Status- und Histogrammanzeige
- zwei Taster fuer Messen und Undo oder Zurueck
- Drehencoder mit Druckfunktion fuer Navigation und Bestaetigung
- Vibrations- oder Haptik-Ausgabe
- ESP32-C6 oder vergleichbare Funk-MCU mit ESP-NOW

Fachliche Aufgaben des Wireless-TSL2591-Geraets:

- abgesetzte Spotmessung direkt am Messpunkt
- Start eines spektralen Mess-Handshake fuer Gruen und Blau auf Anforderung des Hauptsystems
- Uebertragung von Lux-Rohwerten beziehungsweise Messdaten an das Hauptsystem
- Uebertragung von Taster- und Encoder-Ereignissen
- Anzeige von Status, Texten und Zonensystem-Histogramm auf dem OLED
- Haptik-Feedback fuer Klick, Fehler oder Abschluss
- Heartbeat zur Verbindungsueberwachung

Architekturregeln fuer Dukatimer-Part2:

- Das Wireless-TSL2591-Geraet ist ein Dumb Terminal und besitzt keine eigene Papierlogik, keine eigene Belichtungsmathematik und keinen autoritativen Systemzustand.
- Der ESP32-S3 ist die einzige zustaendige Funk-Gateway-MCU fuer ein solches Geraet.
- Der Teensy bleibt Eigentuemer von Mess-Session, Histogramm, Auto-Vorschlaegen und Belichtungsentscheidung.
- Ein Ausfall oder Nichtvorhandensein des Wireless-TSL2591-Geraets darf die Kernfunktion der ersten Part2-Ausbaustufe nicht blockieren.

## 4. Umsetzbarkeit

## 4.1 Gesamtbewertung

Gesamturteil: umsetzbar mit mittlerem bis hohem Entwicklungsaufwand.

Bewertung im Detail:

- Fachliche Uebernahme der Funktionen: hoch umsetzbar
- Direkte Uebernahme vorhandener C++-Logik: mittel umsetzbar
- Direkte Uebernahme der bisherigen HAL/UI-Schichten: niedrig umsetzbar
- Risiko durch Dual-MCU-Schnittstelle: mittel bis hoch

## 4.2 Empfohlene Uebernahmestrategie

### v0.9 uebernehmen

Diese Bausteine sollen konzeptionell aus v0.9 uebernommen werden:

- SystemContext
- AppManager-Konzept
- PaperManager-Konzept
- StorageManager-Konzept
- ExposureEngine-Logik
- Trennung in Hardware-, Sensor-, Input- und UI-Dienste

### v0.3 als Referenz nutzen

Diese Aspekte sollen aus v0.3 als Referenz und gegebenenfalls Codequelle genutzt werden:

- bewiesene Bedienablaeufe einzelner Modi
- Histogramm- und Messlogik
- Randfaelle der Belichtungssteuerung
- Koppelung von Messung und automatischer Vorschlagsbildung

## 4.3 Wiederverwendungsgrad nach Teilbereich

| Teilbereich | Empfohlene Quelle | Wiederverwendung | Zielsystem | Bemerkung |
| --- | --- | --- | --- | --- |
| Modusverwaltung und Zustandsmodell | v0.9 | hoch | Teensy | gute Basis fuer Portierung |
| ExposureEngine | v0.9 plus v0.3-Verhalten | mittel bis hoch | Teensy | neue HAL erforderlich |
| Papierprofile und Mathematik | v0.9 plus v0.3 | hoch | Teensy | sehr gut portierbar |
| Spotmessung und Histogramm | v0.3 plus v0.9 | mittel bis hoch | Teensy | Funkpfad durch kabelgebundene Sensorik ersetzen |
| Display/UI | keine direkte Uebernahme | niedrig | Teensy | LVGL/TFT/Touch-Basis frueh definieren und dokumentieren |
| lokale Eingaben | teilweise aus v0.9 | niedrig bis mittel | Teensy plus ESP | Encoderarchitektur zuerst festziehen, neue Treiber notwendig |
| Wireless TSL2591 Handgeraet | frueh aus v0.3/v0.9 ableiten | mittel | ESP plus Teensy | von Beginn an im Gateway- und Statusmodell vorsehen |
| Zusatzsensorik und Services | neu mit Teilen aus v0.9 | mittel | ESP | neue Service-Schicht notwendig |
| Inter-MCU-Kommunikation | neu | keine | beide | komplett neu spezifizieren |

## 4.4 Hauptrisiken

1. Eine 1:1-Portierung der bisherigen UI-Schicht ist nicht moeglich, weil Part2 statt Nextion plus 16x2-LCD ein TFT mit Touch nutzt.
2. Die alte Wireless-Probe-Logik passt nicht direkt auf den neuen, kabelgebundenen Sensorkopf.
3. Bei falscher Aufgabenverteilung zwischen Teensy und ESP kann Timing-Jitter in der Belichtung entstehen.
4. Ohne robustes SharedProtocol drohen inkonsistente Einstellungen, verlorene Events oder blockierende Wiederanlaeufe.
5. Sicherheitsfunktionen duerfen nicht ueber die MCU-Grenze ausgelagert werden.

## 5. Produktfunktionen

Die Anforderungen sind in Muss, Soll und Kann unterteilt. Die Einstufung beruecksichtigt neben v0.3 und v0.9 ausdruecklich auch die historische Funktionslinie aus 2.30, 2.240 und B1.0.

## 5.1 Muss-Anforderungen

### PF-M01 Systemstart und sicherer Grundzustand

Nach dem Einschalten muessen alle Lichtkanaele, Relais und Belichtungsaktoren sicher ausgeschaltet sein. Eine Belichtung darf erst nach erfolgreicher Initialisierung der Kernkomponenten freigegeben werden.

### PF-M02 Lokale Bedienung am Hauptgeraet

Das System muss ueber TFT, Touch, Encoder 1 bis 3 und die lokalen Taster bedienbar sein. Kernfunktionen duerfen nicht vom ESP32-S3 abhaengen. Encoderrollen, Eventtypen und die Zuordnung von Dreh-, Klick- und Langdruck-Ereignissen muessen vor der Umsetzung der Fachmodi verbindlich definiert werden.

### PF-M02a Encoderbetrieb auf dem Teensy

Die Encoder 1 bis 3 muessen die Staerken des Teensy ausnutzen. Die A/B-Signale sind ueber eine dedizierte, latenzarme Quadraturauswertung zu verarbeiten. Eine grobe, rein zeitbasierte Soft-Entprellung wie in den alten ESP32-Staenden darf fuer die Teensy-Encoder nicht das Grundprinzip sein; stattdessen ist eine robuste Ereignisnormalisierung mit getrennter Behandlung der Encoder-Taster vorzusehen.

### PF-M02b UI-/LVGL-/TFT-Basis

Die Grundarchitektur fuer TFT, Touch, LVGL, Eingabegeroutung und Screen-Navigation muss zu Projektbeginn sauber festgelegt und dokumentiert werden. Fachscreens fuer Modi duerfen diese Basis nicht ad hoc redefinieren.

### PF-M02c Lokale Schalterlogik

Die lokale Schalterlogik muss bereits im ersten Implementierungsschritt verbindlich umgesetzt werden. Dabei gilt:

- `Fokus` schaltet den NeoPixel-Lichtkopf auf Weiss mit 100 Prozent.
- `Save` schaltet den NeoPixel-Lichtkopf ausschliesslich auf Rot.
- `Room` schaltet ueber das SSR-Relais die externe Raumbeleuchtung aus.
- `Save` hat Prioritaet vor `Fokus`.
- Die SaveLatch-Logik ist zu uebernehmen: Wird `Save` verlassen, waehrend `Fokus` physisch noch aktiv ist, bleibt das System dunkel, bis `Fokus` bewusst auf `AUS` und danach erneut auf `AN` geschaltet wurde.

### PF-M03 BW-Belichtung

Dukatimer-Part2 muss Schwarzweiss-Belichtung mindestens in Zeit- und Dosisvariante unterstuetzen. Einstellwerte muessen lokal sichtbar, aenderbar und persistent speicherbar sein.

### PF-M04 Splitgrade-Belichtung

Dukatimer-Part2 muss Splitgrade-Belichtung in Zeit- und Dosisvariante unterstuetzen. Soft- und Hard-Anteil muessen getrennt verwaltbar und gemeinsam ausloesbar sein.

### PF-M05 Spotmessung und Histogramm

Das System muss Messpunkte erfassen, in ein Zonensystem ueberfuehren und als Histogramm speichern. Undo fuer den letzten Messpunkt ist vorzusehen.

### PF-M06 Automatische Vorschlagsbildung

Aus den Messpunkten muessen Vorschlaege fuer BW und SG abgeleitet werden koennen. Vorschlaege duerfen nicht ungeprueft sofort aktiv werden, sondern muessen bestaetigt oder explizit uebernommen werden.

### PF-M07 ExposureEngine

Die Belichtungsengine muss Zeit- und Dosisbelichtung mit lokaler Sensoranbindung, definierten Pre- und Post-Wait-Phasen, sicherem Abbruch und reproduzierbarem Shutoff unterstuetzen. Die Engine muss als zustandsbasierte State-Machine mit mindestens `PRE_WAIT`, `EXPOSING`, `PAUSED` und `POST_WAIT` ausgelegt sein. Im Dosisbetrieb muss die Ist-Dosis ueber laufende `lux * dt`-Integration eines lokalen `TSL2561` gefuehrt werden. Der Shutoff muss praediktiv unter Beruecksichtigung der NeoPixel-Ausgangslatenz erfolgen.

### PF-M08 Teststrip

Das System muss Teststrip-Sequenzen mit mehreren Schritten, nachvollziehbarer Schrittverwaltung und sauber getrenntem Ablauf gegenueber der normalen Belichtung unterstuetzen. Historisch bewaehrtes Verhalten wie additive Schrittbildung und das Unterdruecken von Preflash im Teststrip-Pfad ist funktional zu erhalten.

### PF-M09 Burn-Modus

Das System muss Nachbelichtungen mit reproduzierbarer Dosis- oder Zeitberechnung und definierter Lichtkanalsteuerung unterstuetzen.

### PF-M10 Papierprofile und Kalibrierung

Papierprofile muessen angelegt, gespeichert, geladen und kalibriert werden koennen. Die Kalibrierlogik aus den Vorgaengersystemen ist funktional zu uebernehmen. Das persistente Papiermodell muss neben K-Faktoren auch historisch belegte slot-gebundene Zusatzparameter wie Preflash-Konfigurationen aufnehmen koennen. Die Arbeitskalibrierung soll end-to-end ueber Lichtkopf, Sensor und Papier reproduzierbar bleiben, auch wenn Kopfkalibrierung und Papierprofil architektonisch getrennt persistiert werden.

### PF-M11 Densitometer und Filmtest

Densitometer- und Filmtest-Workflows muessen als Kernfunktion uebernommen werden. Dazu gehoeren mindestens die Arbeitszustaende fuer Referenz, Basis und Messung sowie die aus den fruehen Versionen bekannten Hilfen fuer Zone I und Zone VIII.

### PF-M12 Preflash

Preflash muss als Kernfunktion uebernommen werden. Dazu gehoeren ein gefuehrter Kalibrierablauf, slot-gebundene Preflash-Parameter, reproduzierbare Ausloesung vor der Belichtung und die funktionale Trennung vom Teststrip-Ablauf.

### PF-M13 Setup und Persistenz

Systemeinstellungen, Papierprofile und betriebsrelevante Parameter muessen persistent, versioniert und gegen korrupte Daten abgesichert gespeichert werden.

### PF-M14 Sicherheits- und Abbruchlogik

Belichtungsrelevante Ausgaenge muessen lokal und deterministisch deaktiviert werden koennen. Start/Stop, Overheat, Sensorfehler und Kommunikationsfehler muessen definierte sichere Reaktionen ausloesen. Ein Sensor-Watchdog muss aktive Belichtungen hart abbrechen, wenn der lokale Dosis-Sensor ausfaellt oder unter aktivem Licht unplausible Werte wie dauerhaft `0 Lux` liefert. Oberhalb einer thermischen Warnschwelle ist die Kopfleistung zu drosseln; oberhalb einer kritischen Schwelle ist ein zwingender Emergency Shutoff auszufuehren.

### PF-M15 Inter-MCU-Basisdienst

Teensy und ESP32-S3 muessen ueber ein robustes SharedProtocol kommunizieren. Verbindungsverlust darf die Kernfunktion des Hauptsystems nicht blockieren.

### PF-M16 ESP-Servicefunktionen

Der ESP32-S3 muss Zusatzsensorik, Encoder 4 und weitere Service-Eingaenge erfassen und diese stoerungsarm an den Teensy melden.

### PF-M17 Wireless-Basisintegration

Das Wireless-TSL2591-Handgeraet muss von Beginn an in Protokoll, Statusmodell und UI-Basis beruecksichtigt werden. Mindestens Heartbeat, Peer-Status, Remote-Eingabeereignisse, Messkommandos und Renderdaten muessen in der fruehen Architektur vorgesehen sein, auch wenn spaetere Komfortfunktionen stufenweise folgen.

## 5.2 Soll-Anforderungen

### PF-S01 Historische Bedienparitaet

Historisch bewaehrte Schnellablaeufe fuer Messung, Undo, Vorschlagsuebernahme, Teststrip und Densitometer sollen im neuen TFT-/Touch-UI konzeptionell erhalten bleiben, auch wenn die alte 16x2- und Tastenbelegung nicht 1:1 uebernommen wird.

### PF-S02 LiveView

Ein kontinuierlicher LiveView-Modus fuer Mess- oder Fokuszwecke soll bereitgestellt werden. Im LVGL-/TFT-UI sollen dabei mindestens Ist-Dosis, Restzeit oder Rest-Dosis sowie Kopf- und Umgebungstemperatur klar sichtbar gemacht werden.

### PF-S03 F-Stop-/EV-Bedienlogik fuer belichtungsrelevante Modi

Die in v0.9 explizit als BW-F-Stop-Modus sichtbare fotografische F-Stop-/EV-Bedienlogik soll fuer Part2 nicht als isolierte Einzel-App neu entstehen.

Verbindliche Vorgabe fuer Part2:

- alle belichtungsrelevanten Modi benutzen dieselbe zentrale EV-/F-Stop-Grundlogik
- dazu gehoeren mindestens BW, SG, Burn, Teststrip und Preflash
- ein einzelner Modus darf nur noch UI-Fokus, fachliche Randbedingungen und Defaultwerte spezifizieren
- Stop-Schritte bleiben logarithmisch und fotografisch konsistent statt pro Modus linear oder ad-hoc zu variieren

### PF-S04 Zone-Modus

Der Zone-Modus aus v0.9 soll uebernommen werden, wenn die Bedienung auf TFT/Touch konsistent darstellbar ist.

### PF-S05 SG-Regelwerk-Validierung

Die in B1.0 erprobte Smart-Math-Variante fuer Splitgrade soll als Vergleichsregelwerk dokumentiert und gegen das spaetere Verhalten aus v0.3 und v0.9 validiert werden, ohne automatisch zum neuen Standard erklaert zu werden.

### PF-S06 Messwertdarstellung in EV und Lux

Messwerte aus echten Messvorgaengen sollen fuer Part2 zusaetzlich in EV angegeben werden, sofern die fotografische Einordnung sinnvoll ist.

Verbindliche Vorgabe fuer Part2:

- Densitometrie, Papierkalibrierung, Spot-/Zonenmessung und vergleichbare Messworkflows zeigen Messwerte zusaetzlich in EV an
- die Roh- oder Basisgroesse bleibt sichtbar, zum Beispiel Lux bei Lichtmessung
- reine Laufzeittelemetrie wie Head-Lux, Temperatur, Buslatenz oder Heartbeat bleibt primaer in Lux, C, ms oder Statuscodes
- die Umrechnung nach EV erfolgt ueber eine zentrale Formatter-/Math-Schicht und nicht separat pro Screen oder Workflow

## 5.3 Kann-Anforderungen

### PF-K01 Erweiterte Wireless-Komfortfunktionen

Ueber die verpflichtende Wireless-Basisintegration hinaus koennen spaeter erweiterte Remote-Anzeigen, zusaetzliche Sonderablaeufe, Diagnoseansichten und Komfortinteraktionen auf dem Handgeraet aufgebaut werden. Das Geraet soll auch dann keine eigene Belichtungslogik, keine Papierprofile und keinen eigenstaendigen autoritativen Systemzustand besitzen.

### PF-K02 Netzwerk- und Diagnosekomfort

Erweiterte Logging-, Service- oder Netzwerkfunktionen koennen spaeter auf dem ESP32-S3 aufgebaut werden.

## 6. Nichtfunktionale Anforderungen

### NF-01 Lokale Deterministik

Alle sicherheitskritischen Licht- und Belichtungsvorgaenge muessen vollstaendig auf dem Teensy ablaufen.

### NF-02 Robustheit bei ESP-Ausfall

Ein Ausfall oder Neustart des ESP32-S3 darf keine unkontrollierte Lichtaktivierung und keinen unkontrollierten Belichtungszustand verursachen.

### NF-03 Sichere Standardzustaende

Nach Boot, Reset, Kommunikationsverlust oder Fehler muessen die Ausgaenge in einen sicheren Off-Zustand gehen.

### NF-04 Versionsfaehige Persistenz

Persistente Daten muessen versioniert, pruefbar und bei Inkompatibilitaet migrierbar oder sicher zuruecksetzbar sein.

### NF-05 Entkopplung von Fachlogik und Hardware

Die Softwarearchitektur muss Fachlogik, HAL, UI und Inter-MCU-Kommunikation sauber trennen.

### NF-06 Reaktionsfaehige Bedienung

Lokale Bedienung am Hauptgeraet muss ohne wahrnehmbare Abhaengigkeit von der Service-MCU moeglich sein.

### NF-07 Diagnosefaehigkeit

Fehlerzustaende fuer Sensorik, Persistenz, Kommunikation und Belichtung muessen protokollierbar und im UI sichtbar sein.

### NF-08 Encoder-Stabilitaet

Die Encoder 1 bis 3 muessen auch bei schneller Bedienung verlustarm und reproduzierbar arbeiten. Prellen, Fehlschritte und inkonsistente Richtungswechsel muessen auf Ebene der Input-Schicht abgefangen werden, ohne die lokale Bedienung durch hohe Entprelllatenz traege zu machen.

### NF-09 Dokumentierte UI-Basis

Die LVGL-/TFT-/Touch-Basisarchitektur, die Navigationsebenen und die Kopplung zwischen Eingaben und UI muessen schriftlich dokumentiert sein, bevor mehrere Fachmodi parallel darauf aufbauen.

### NF-10 Zentrale Wiederverwendung gemeinsamer Logik

Logik, die in mehreren Modi oder Workflows verwendet wird, muss einmal zentral definiert werden.

Das gilt insbesondere fuer:

- EV-/F-Stop-Schrittlogik
- Messwertumrechnung und EV-Formatter
- Eingabe- und Encodersemantik
- belichtungsrelevante Hilfs- und Grenzfunktionen

## 7. Abgrenzung und Entscheidungen

Folgende Punkte sind fuer die Umsetzung verbindlich zu beruecksichtigen:

- Die Part2-Hardware ist kein einfaches Rehosting der alten ESP32-S3-Firmware.
- Display- und Eingabeschicht muessen neu entwickelt werden.
- Encoderarchitektur und Eventsemantik sind kein spaeteres Feintuning, sondern ein Startartefakt.
- Die erste fachliche Umsetzungsstufe beginnt mit Papierkalibrierung, Modus SG, SG-Mathematik und Belichtungslogik.
- Historisch belegte Kernfunktionen aus 2.30 und vor allem 2.240 gelten als fachlicher Mindestumfang fuer eine spaetere vollwertige Part2-Version.
- Die Wireless-TSL2591-Funktion ist von Beginn an im Architektur- und Integrationsmodell vorzusehen.
- Die Kernlogik soll nicht zwischen beiden MCUs aufgespalten werden, sondern primaer auf dem Teensy liegen.
- Belichtungsrelevante Logik, die mehrfach verwendet wird, wird nur einmal zentral definiert und von den Modi konsumiert.
- Messwerte aus echten Messvorgaengen werden in fotografisch sinnvollen Workflows zusaetzlich in EV dargestellt; Rohtelemetrie bleibt in ihren physischen Grundeinheiten.

Offene Punkte fuer das Feindesign:

- genaue Detailbelegung der Encoderrollen pro Screen innerhalb der zuvor fixierten Grundsemantik
- exakte API-Grenze zwischen zentraler EV-/F-Stop-Logik und modusspezifischen Defaults oder Panels
- konkrete Nutzung des 1-Wire-Busses im Zusammenspiel mit der kabelgebundenen Sensorik
- Rollenverteilung fuer Audio-Freigabe, Vibrationsfeedback und Servicefunktionen
- Auswahl des konkreten Inter-MCU-Transports inklusive Framing und Recovery-Strategie
- genaue Tiefe der ersten Wireless-UI- und Remote-Workflows gegenueber spaeteren Komfortausbaustufen

## 8. Empfohlene Umsetzungsreihenfolge

### Phase 1: Input-, UI- und Wireless-Grundarchitektur

- Encoderarchitektur fuer Encoder 1 bis 4 und die Eventsemantik verbindlich festlegen
- gemeinsame EV-/F-Stop-Grundlogik, Messwertkonventionen und zentrale Formatter festlegen
- LVGL-, TFT-, Touch- und Navigationsbasis dokumentieren und technisch initialisieren
- lokale Schalterlogik fuer Fokus, Save und Room inklusive SaveLatch zentral definieren
- SharedProtocol-Grundgeruest mit Heartbeat, Boot-Status, Fehlerzustand und Wireless-Gateway-Pfaden definieren
- sicheren Boot- und Off-Zustand lokal validieren

### Phase 2: erster fachlicher Slice SG plus Papierkalibrierung

- Paper-/Storage-Modell mit SG-relevanten Parametern und Kalibrierdaten portieren
- Modus SG inklusive Mathematik, Vorschlagslogik und Belichtungsengine lauffaehig machen
- Papierkalibrierung mit passendem UI-Flow integrieren
- Wireless-Basis fuer Heartbeat, Remote-Events, Renderdaten und Messkommandos parallel anbinden

### Phase 3: Mess- und Bedienausbau

- Messpipeline, Histogramm, Undo und EV-Darstellung fuer echte Messworkflows vervollstaendigen
- BW, Burn, Teststrip und Setup stueckweise ergaenzen
- ESP-Serviceintegration fuer Encoder 4, Zusatzsensorik und 1-Wire robust anbinden

### Phase 4: restliche historische Muss-Funktionen und Erweiterungen

- Densitometer und Filmtest vollstaendig integrieren
- Preflash-Kalibrierung und Vorbelichtungsablauf vervollstaendigen
- LiveView, modusspezifische F-Stop-Oberflaechen auf Basis der zentralen EV-Logik, Zone und erweiterte Wireless-Komfortfunktionen ausbauen

## 9. Abnahmekriterien

Die erste fachlich vollwertige Part2-Version gilt als erreicht, wenn mindestens folgende Punkte nachweisbar erfuellt sind:

1. Das Geraet bootet in einen sicheren Off-Zustand.
2. Die Encodergrundlage arbeitet auf dem Teensy stabil, reproduzierbar und ohne das fruehere ESP32-typische Entprellchaos.
3. Die UI ist ohne Nextion/LCD vollstaendig auf dem neuen TFT/Touch-/LVGL-System nutzbar.
4. SG-Belichtung inklusive Mathematik, Vorschlagslogik und lokaler ExposureEngine laeuft reproduzierbar auf dem Teensy.
5. Papierkalibrierung und Papierprofile werden persistent und konsistent gespeichert.
6. Die Wireless-Basisintegration liefert Heartbeat, Remote-Status und die vorgesehenen Remote-Ereignisse ohne die Kernfunktion zu blockieren.
7. Messpunkte erzeugen ein Zonensystem-Histogramm, koennen fuer Auto-Vorschlaege verwendet werden und werden in fotografisch sinnvollen Workflows zusaetzlich als EV dargestellt.
8. Teststrip, Densitometer/Filmtest und Preflash sind lokal nutzbar, greifen auf dieselbe zentrale EV-/F-Stop-Logik zurueck und verhalten sich konsistent zu den historischen Referenzen.
9. Ein ESP-Ausfall verhindert keine sichere Beendigung laufender Belichtungen.

## 10. Schlussbewertung

Die funktionale Uebernahme von Dukatimer v0.3 und v0.9 durch Dukatimer-Part2 ist realistisch und sinnvoll. Die beste technische Strategie ist nicht die Uebernahme einer kompletten Alt-Firmware, sondern die Uebernahme des v0.9-Architekturmodells, angereichert mit bewaehrter Fachlogik und Zielverhalten aus v0.3.

Der groesste Aufwand liegt nicht in den Fachfunktionen selbst, sondern in drei Punkten:

- Neubau der Hardware-Abstraktion fuer die Part2-Platine
- Neubau der UI fuer TFT plus Touch
- Definition und Absicherung der Inter-MCU-Schnittstelle

Wenn diese drei Punkte sauber geloest werden, ist Dukatimer-Part2 fachlich gut realisierbar.
