# Offene Punkte

Stand: 2026-05-06

Dieses Dokument ersetzt verstreute ToDo- und Audit-Listen als zentrale technische Resteliste fuer `Dukatimer-Part2`. Ein Punkt gilt hier nur als erledigt, wenn er im aktuellen Code physisch nachweisbar ist. Historische Zieltexte, alte `erledigt`-Dokumente und Kommentare zaehlen nicht als Abschluss, wenn der Laufzeitpfad fehlt.

## Aktueller Versionsstand

- Teensy-Firmware: `Dukatimer Part2 Teensy`, `0.2.44-dev`, Stage `visual threshold calibration method 1`, Backend `ILI9488_t3_mm shim`, Runtime `LVGL 8.x` (`src/teensy/FirmwareVersion.h`).
- ESP32-S3-Service: `Dukatimer Part2 ESP Service`, `0.1.14-dev` (`src/esp32/FirmwareVersion.h`).
- SharedProtocol: `kProtocolVersion = 1`, Frame-CRC und ABI-Static-Assertions fuer `WirelessRemotePayload == 25` und `RemoteDisplayPayload == 78`; `WirelessSnapshotPayload` traegt jetzt auch `commandAckSequence` sowie Renderstatus/stale-/timeout-Zaehler und `HeartbeatPayload` zusaetzliche Runtime-Flags fuer laufzeitnahe MCU-Zustaende (`lib/SharedProtocol/DukatimerProtocol.h`). Der Part2-Build fuehrt inzwischen `tools/check_dukatimer_protocol_sync.py` als Pre-Script aus; eine echte gemeinsame Quelle fuer C6/Part2 ist damit noch nicht ersetzt, aber Drift wird beim Build sichtbarer.
- ESP32-C6 Wireless TSL2591: keine zentrale `FirmwareVersion.h` gefunden. Klärungsbedarf: Versionsquelle und Release-Kennung fuer Terminal-Firmware einfuehren.

## Code-verifizierte Basis

Diese Punkte sind nach aktuellem Code als echte Basis vorhanden und koennen aus alten offenen Listen in Archiv-/Erledigt-Kontext verschoben werden, sofern dort nicht weitergehende Produktanforderungen gemeint waren.

- Dual-MCU-Split ist umgesetzt: Teensy baut UI, lokale Eingaben, Exposure, Paper/Storage und Safety; ESP32-S3 baut Encoder-4-/Sensor-/Wireless-/HTTP-Servicepfade per PlatformIO-Filter.
- SharedProtocol v1 ist realer Contract mit Framing, CRC, Message-Typen, Remote- und VFS-Payloads sowie statischen ABI-Grenzen.
- ESP32-S3-Link sendet Heartbeat, Service-Snapshot, Wireless-Snapshot, Input-Events, Diagnose und VFS-Frames; beide UART-Richtungen arbeiten jetzt nicht-blockierend. Der Teensy puffert ausgehende Frames lokal, prueft `availableForWrite()` vor `serial_.write()`, sortiert die Queue selbst nach `Command/VFS > Heartbeat > Render > Diagnose`, begrenzt den direkten Flush pro Aufruf und zaehlt Queue-Evictions/Drops sichtbar im Linkstatus.
- HTTP-VFS-Bridge existiert mit Health-Endpoint, Raw-Upload, Pfadfilter und serieller VFS-Uploadmaschine.
- Teensy-VFS nimmt Uploads transaktional an und setzt bei aktiver Belichtung einen Realtime-Hold, der neue VFS-Requests/Chunks mit Busy/Flow-Hold abweist. Umgekehrt sperrt der Startpfad jetzt neue Belichtungen, solange eine aktive serielle VFS-Session oder eine noch laufende HTTP-/ESP-Dateitransaktion sichtbar ist.
- Paper-Slot-Persistenz existiert fuer 20 Slots: binaeres Blobformat, Schema- und CRC-Pruefung, transaktionales Schreiben ueber `.tmp` plus Backup/Rename.
- Defaults werden nur bei fehlender Datei automatisch geschrieben. Korrupte, unlesbare oder inkompatible Paper-Blobs werden nicht still durch Defaults ueberschrieben.
- Paper-Profilmodell enthaelt `isoP`, `isoR`, `kBw`, `kSoft`, `kHard`, 11 Splitgrade-0,5er-Stuetzstellen und Preflash-Struktur; zusaetzlich muss fuer jedes Papier explizit definiert werden, ob es fixed grade oder multigrade ist.
- Splitgrade-Workflow nutzt paper-driven Targets, 0,5er Gradation, LUT-Pfade und ISO-P/ISO-R-Modell; Soft-/Hard-Phasen mit Filterwechselzustand sind im Workflow vorhanden.
- AP-07-Akzeptanztests fuer Splitgrade-Math existieren im `test/ap07_splitgrade_acceptance`-Harness.
- ExposureEngine besitzt Zeit- und Dosisbelichtung, geschlossene Lux-Sekunden-Integration, Sample-Watchdog, Thermik-Stopp, Null-Lux-Plausibilitaet, Sensor-Fallback und praediktiven Head-Shutoff.
- NeoPixel-Head-Ausgabe, Head-Arbiter, SSR-Raumlicht und Save/Focus/Room-Schalterhierarchie sind als Laufzeitpfad vorhanden.
- Lokaler TSL2561-Pfad ist implementiert: Initialisierung, Interrupt/Polling-Fallback, Lux-Berechnung, Aging, Watchdog und Statusuebergabe an Exposure/Measurement.
- Bei lokalem TSL2561-Dienstfehler oder nach erkanntem Main-Loop-Watchdog-Reset erzwingt das Teensy-Wiring fuer neue Splitgrade-Starts explizit Time-Modus statt Dose; eine bereits laufende lokale Dose-Belichtung laeuft dagegen ueber die berechnete Restzeit zu Ende und das UI markiert dieses Ergebnis sichtbar als nicht closed-loop-abgesichert und damit nicht vertrauenswuerdig.
- DS18B20/AHT20/BMP280-Servicepfad auf dem ESP32-S3 ist vorhanden; DS18B20 wird asynchron alle `5s` konvertiert und explizit auch waehrend aktiver Belichtung weitergelesen. AHT20/BMP280 werden auf dem gemeinsamen U-SENS1-I2C-Bus nur alle `30s` gelesen; waehrend aktiver Belichtung bleiben ihre Lesezyklen per Teensy-Heartbeat-Flag ausgesetzt und werden danach wieder aufgenommen.
- ESP32-S3 liest Encoder 4 jetzt lokal per PCNT (`ESP32Encoder`) ein und publiziert nur bestehende `RemoteInputSource::Encoder4`-Ereignisse fuer Kontextnavigation und Confirm an den Teensy.
- Die lokalen Teensy-Encoder 1-3 laufen jetzt ueber einen normierten `RotaryEncoderDriver` mit Detent-Modell, Phasenplausibilitaet sowie sichtbaren Reverse-/Mismatch-Zaehlern statt ueber den alten positionsbasierten Wrapper in `main.cpp`.
- MeasurementDomainService existiert als zentraler Messdatenhalter fuer lokale und Wireless-Lux-Samples, aktive Quelle, Session-Historie, Undo und Histogramm; das Session-Histogramm nutzt jetzt explizite quellengebundene Referenz-Luxwerte und relative EV-Abstaende statt eines festen 1-Lux-Ankers.
- MeasurementValueFormatter besitzt inzwischen zentrale Formatter fuer Quellen, Hauptmesswert, Meta, Referenz, Range und Controls; `UiPresenter` nutzt diese Formatter. Offen bleibt, dass der aktuelle `LvglUi::pushWidgetsFromSnapshot()`-Block fuer `SCREEN_ID_PAGE_MEASUREMENT` diese Presenter-/Formatter-Schicht noch teilweise umgeht und eigene `snprintf`-Texte schreibt.
- InputNormalizer und InputRouterPolicy existieren mit lokalen Encodern, Remote-Encoder 4, Wireless-Encoder, Modal-/Touch-/Owner-Guards und Dispatch-Budget.
- SetupWorkflow, `ModeId::Setup`, persistente `SystemSettings`, Systemsettings-Storage-Diagnose und Runtime-Anwendung auf `SensorManager` sowie `ExposureEngine` sind vorhanden. `HEAD DIAG` bleibt bewusst ein runtime-only Bring-up-Schalter und schreibt keine persistenten Produktwerte.
- LVGL nutzt auf dem Teensy einen PSRAM-Heap ueber `LV_MEM_CUSTOM`/`lvgl_psram_alloc.h`; die 2x80-Zeilen-Draw-Buffer liegen weiterhin in `DMAMEM`, nicht im PSRAM.
- Boot, PageSplitgrade, PagePaperWorkspace, PageSetup, PageMeasurement, PageWirelessRemote und ein priorisierter `SCREEN_ID_BUSY` fuer aktive Exposure-Phasen besitzen echte handgeschriebene LVGL-Widget-Baeume in `screens.c`. `LvglUi::pushWidgetsFromSnapshot()` aktualisiert alle sieben Seiten aus `SystemSnapshot`; der Build `pio run -e teensy41` war nach diesem UI-Slice erfolgreich mit RAM1 frei `21120` Bytes. Das ist code-/build-verifiziert, aber noch kein finaler EEZ-Designer- oder Hardware-Abnahmeabschluss.
- C6-Terminal sendet `WirelessRemotePayload` mit Lux, Buttonmaske, Encoder-Delta und Sequenz; ESP32-S3 verwirft alte eingehende Remote-Sequenzen.
- Teensy erzeugt `RemoteDisplayPayload` mit Sequenz, View-Type, Textzeilen und Progress; ESP32-S3 sendet diesen Payload per ESP-NOW an das C6-Terminal.
- Remote-Kommandos `RemoteMeasurementStart`, `RemoteMeasurementCancel` und `RemoteHaptic` werden jetzt vom ESP32-S3 roh per ESP-NOW an das C6-Terminal weitergeleitet; das Terminal puffert sie asynchron, setzt Messkontext/Haptik lokal um und publiziert die zuletzt wirklich ausgefuehrte `commandSequence` ueber Gateway und Wireless-Snapshot zurueck.
- C6-Renderverlust ist jetzt entlang des bestehenden Rueckkanals sichtbar: das Terminal zaehlt stale Renderpakete und Render-Timeout-Episoden lokal, zeigt sie im Footer an, der ESP-S3 hebt Counter-Anstiege als `WirelessRenderStale`/`WirelessRenderTimeout` in den Diagnosekanal und der Teensy sieht Status plus Zaehler im Gateway-Snapshot.
- Teensy-Wiring enthaelt jetzt einen `RemoteCommandTracker` als Vermittler zwischen Workflow und `EspServiceLink`; mehrere kritische Remote-Messkommandos werden in Sendereihenfolge ueber `commandAckSequence` mit Retry/Timeout ueberwacht, waehrend unkritische Kommandos per Policy fire-and-forget bleiben.
- BlackWhite-Modus ist im ModeCoordinator sichtbar, aber nur als Shell. Das ist ein sauberer Platzhalter, keine fertige BW-Funktion.

## Kritische offene Punkte

### P1: UI ist widgetseitig code-/build-verifiziert, aber noch nicht durchgaengig presenter-/formatter-konform

Nachweis: Alle sieben lokalen Screens besitzen echte handgeschriebene LVGL-Widget-Baeume und werden aus `LvglUi::pushWidgetsFromSnapshot()` aktualisiert. Der neue `SCREEN_ID_PAGE_MEASUREMENT`-Block formatiert Local/Wireless-Lux, Hauptlux, Referenz und EV-Differenz jedoch direkt per `std::snprintf`, obwohl `UiPresenter` und `MeasurementValueFormatter` dafuer zentrale Funktionen bereitstellen. Gleichzeitig bleiben in `screens.c` und `LvglUi.cpp` noch einzelne produktnahe Magic-Farbwerte und Diagnosefarben ausserhalb der normativen Palette.

Warum: Die Architekturvorgabe lautet, dass `LvglUi` keine eigene Messwertformatierung und keine eigene Referenzpolitik pflegt. Der aktuelle Stand ist kein mathematischer Drift, weil EV-Werte aus `MeasurementDomainService` kommen; er ist aber ein Darstellungs- und Ownership-Drift am UI-Rand. Dadurch drohen source-abhaengige Praezision, Invalid-Texte und spaetere Zone-/LogD-Formatierung erneut pro Screen zu wachsen.

Mindestloesung: PageMeasurement ueber `UiPresenter::getMeasurement...()` beziehungsweise `MeasurementValueFormatter` verdrahten, Invalid-/Source-/Range-Texte zentral halten, nicht normative Farben auf dokumentierte Palette oder explizite Diagnoseausnahme zurueckfuehren und danach eine echte Encoder-/Touch-/Sichtabnahme auf Hardware fahren.

### P1: End-to-End-Papierkalibrierprotokoll ist angelegt, reale Abnahme fehlt noch

Nachweis: Das zentrale Protokoll liegt jetzt in [../hardware/end-to-end-papierkalibrierprotokoll.md](../hardware/end-to-end-papierkalibrierprotokoll.md) und fuehrt Sessionkopf, Rohdatentabellen, Invalidierungskriterien, Graukeil-Referenzlauf sowie die reproduzierbare Ableitung ins aktive Paper-Profil zusammen. Die aktiven Codepfade fuer NeoPixel, SSR, TSL2561, DS18B20, Thermik und Head-Latenz bleiben damit bewusst Teil derselben End-to-End-Abnahme statt eines separaten Nebenprotokolls.

Warum: Die Struktur der Mindestloesung ist vorhanden, aber noch keine reale Hardwaremessreihe im Repository. Belastbar wird der Pfad erst, wenn mindestens ein echter Papier-/Kopf-/Sensorstand mit Rohdaten, Soll/Ist, Toleranz und Rueckschreibebegruendung komplett durch das Protokoll gelaufen ist.

Mindestloesung: Erste vollstaendige reale Session mit 21-Stufen-Durchsichtsgraukeil, Messgeraet, Umgebung, Firmwareversion, Rohdaten und belegter Ableitung nach `ISO-P`, `ISO-R`, SG-LUT und relevanten Kopf-/Sensorparametern.

### P2: Measurement-Domain ist brauchbar, aber noch kein produktiver fotografischer Messworkflow

Nachweis: `MeasurementDomainService` sammelt lokale und Wireless-Samples, fuehrt quellenbezogene Referenzen, relative EVs, Undo und Histogramm. `MeasurementValueFormatter` kann diese Werte zentral anzeigen. Proposal-Logik aus [dukatimer-part2-ap11f-measurement-proposal-math-todo.md](dukatimer-part2-ap11f-measurement-proposal-math-todo.md) ist aber noch nicht umgesetzt; absolute Zonen, Dichte/LogD und papierbezogene Vorschlaege bleiben gesperrt.

Warum: Das Histogramm ist derzeit relativ zur jeweiligen Quellenreferenz und nicht automatisch ein fotografisches Zonensystem. Gemischte lokale/Wireless-Quellen duerfen ohne dokumentierten Abgleich nicht zu einer gemeinsamen Wahrheit verdichtet werden.

Mindestloesung: Proposal-Statusmodell mit Eligibility-Flags, Source-/Korrektur-/Profilstatus und Preview/Accept-Vertrag einfuehren; erst danach SG/BW-Vorschlaege anzeigen oder uebernehmen.

### P2: Lokaler TSL2561-Bus hat nur Reset-basierte Recovery, keine lokale I2C-Entklemmung

Nachweis: `SensorManager` kommentiert weiterhin korrekt, dass der Teensy-`WireIMXRT`-Pfad keinen lokal nutzbaren `setWireTimeout()`-Mechanismus bietet. Der implementierte Laufzeitpfad in `main.cpp` reagiert darauf, indem ein erkannter Main-Loop-Watchdog-Reset sowie lokale TSL-Dienstfehler neue Splitgrade-Starts explizit auf Time-Modus zwingen und den Grund sichtbar exportieren. Fuer Recovery ohne Teensy-Neustart ist als Richtung eine schaltbare `+3V3_HEAD`-Versorgung gesetzt.

Warum: Die Closed-Loop-Logik selbst ist vorhanden und bleibt auf dem Teensy, aber ein blockierter lokaler I2C-Bus wird im aktuellen Boardstand noch nicht lokal freigeloest. Das ist ein Safety-/Wahrheitsrisiko, kein UI-Layoutproblem.

Mindestloesung: Dedizierten High-Side-Schalter fuer `+3V3_HEAD` plus begrenzte Power-Cycle/Reinit-Sequenz im `SensorManager` hardware- und softwareseitig umsetzen; bis dahin Reset/Time-Degradierung als letzte Rueckfallebene beibehalten.

### P2: C6-Terminal hat keine Firmware-Identitaet

Nachweis: In `Wireless TSL2591` wurde weiterhin keine zentrale `FirmwareVersion.h` oder vergleichbare Build-Identitaet gefunden.

Warum: Die Richtung ist gesetzt (gleiches Muster wie Teensy/ESP32-S3). Ohne Umsetzung bleibt Remote-Fehlersuche unscharf, besonders bei SharedProtocol-, Render- und Command-Ack-Fragen.

Mindestloesung: Zentrale C6-Version einfuehren, im Serial-Bootlog und optional im Wireless-Status publizieren.

### P2: Remote-UI ist lokal diagnostisch sichtbar, aber Terminal-Produktpfad bleibt offen

Nachweis: `SCREEN_ID_PAGE_WIRELESS_REMOTE` zeigt jetzt lokal LinkHealth, PeerState, Akku/Alter, LastLux/Sequenz, Diagnose, TxQueue und RMT-Zaehler. Der C6-Produktpfad braucht trotzdem weiter Versionierung, Pairing, Display-Freshness/Render-Ack-Entscheidung und reale Ende-zu-Ende-Abnahme.

Warum: Die lokale Teensy-Seite macht Drift und Queue-Zustaende sichtbar, ersetzt aber nicht den Nachweis, dass das Handterminal den richtigen Zustand rechtzeitig rendert und seine Mess-/Haptic-Kommandos nachvollziehbar quittiert.

Mindestloesung: C6-Version, Pairing-Resetpfad, Render-Freshness- oder bewusst acklose Timeout-Policy und Remote-Messstart mit sichtbarer Ack-Sequenz hardwareseitig validieren.

### P2: Setup-UI ist umgesetzt, Safety-Abnahme und Bedienbestaetigung fehlen noch

Nachweis: `SetupWorkflow`, `SystemSettings`, Persistenz, Runtime-Anwendung auf Sensor/Exposure und `PageSetup`-Widgetbaum sind vorhanden. Thermikwerte werden plausibilisiert und `Apply` schreibt persistent. Nicht belegt sind bisher eine echte SD-/Reboot-Abnahme, ein expliziter Safety-Dialog fuer Thermikwerte und Hardwaretests fuer Derating/Hard-Stop nach UI-Aenderung.

Warum: Die fruehere Luecke "Setup nur als Datenpfad" ist geschlossen. Fuer Safety-Werte reicht Code-/Build-Verifikation aber nicht als Produktabschluss.

Mindestloesung: Setup mit Encoder allein bedienen, Apply/Discard/SafetyDefaults ueber Reboot pruefen, unplausible Thermikwerte und aktive Exposure als Sperrfaelle testen, Derating/Hard-Stop mit Sensor- oder Simulationstest validieren.

### P2: BW, Burn, Teststrip, Densitometer und Preflash sind nicht produktiv

Nachweis: BW ist `ModeShellWorkflow`. `HeadSpectrumCommand` kennt Semantiken fuer BW, Burn, TestStrip, Preflash und Calibration, ist aber nicht als produktiver Workflow mit eigener UI, Input-Semantik, Exposure-Kommandos und Abnahmepunkten verdrahtet. Preflash liegt nur als Datenstruktur im Paperprofil.

Warum: Diese historischen Kernziele koennen nicht als erledigt gelten. Datenmodelle und Enum-Werte sind wertvoll, aber keine Workflows.

BW-Umsetzungsplan: [dukatimer-part2-bw-modus-umsetzungsplan-2026-05-06.md](dukatimer-part2-bw-modus-umsetzungsplan-2026-05-06.md) legt fixed-grade-Weisslicht, multigrade 0.5er Gradationsmischung, Runtime-Vertrag, Exposure-Wiring, Head-Mapping, UI, Tests und Hardwareabnahme als konkrete Arbeitspakete fest.

Mindestloesung: Jeden Modus als separaten Workflow mit Eingabe, Anzeige, Exposure-Kommandos, Mess-/Papierbezug und Akzeptanzkriterien ausbauen.

### P2: Paper-Defaults und Kalibrierstatus sind fachlich zu duenn

Nachweis: Defaultbank erzeugt 20 Slots, aber echte Papierdaten sind nicht durch reale Abnahmeprotokolle belegt. Die UI zeigt aktuell Slot/Name, CAL/RAW und FG/MG, aber noch keinen vollstaendigen Quellenstatus wie Demo, gemessen, importiert oder gesperrt.

Warum: Papierdaten sind Teil der fotografischen Wahrheit, nicht UI-Dekoration. Ohne sichtbaren Status bleibt Verwechslungsgefahr zwischen Demo und Messwertprofilen.

Mindestloesung: Papierdatenquellen, Defaultstatus und Kalibriergrad im Profilmodell und UI sichtbar trennen; echte Defaults erst nach Protokoll-Lauf als gemessen/importiert markieren.

### P2: HTTP-VFS ist Bring-up-tauglich, aber noch kein Produkt-Asset-System

Nachweis: HTTP-VFS kann raw Dateien uploaden und die Teensy-Link-Wartephasen weiter bedienen. Ein belegter Produktpfad fuer Paperprofile/Backups mit Import-/Exportformat, Zugriffsschutz und expliziter Websession-Safety ist noch offen.

Warum: Die Richtung ist gesetzt (Produkt-Konfiguration ueber HTTP), damit ist die Safety-Kopplung Pflicht: Belichtung und Web-Transfer duerfen nicht parallel aktiv sein.

Mindestloesung: Produkt-Konfigurationspfad fuer Paperprofile/Backups umsetzen, inklusive klarer Belichtungsblockade bei aktiver Websession sowie sichtbarer Diagnose fuer Sperr-/Freigabezustand.

### P3: SharedProtocol ist kopiert, Build-Sync-Guard ist vorhanden

Nachweis: `Dukatimer-Part2/lib/SharedProtocol/DukatimerProtocol.h` und `Wireless TSL2591/include/DukatimerProtocol.h` enthalten dieselbe ABI-Struktur. Der Part2-Build ruft `tools/check_dukatimer_protocol_sync.py` als Pre-Script auf; eine echte gemeinsame Quelle existiert aber weiterhin nicht.

Warum: Der automatische Check reduziert Drift-Risiko deutlich, ersetzt aber noch nicht die Architekturentscheidung fuer eine dauerhaft einzige Quelle.

Mindestloesung: Build-Guard beibehalten. Spaeter eine Quelle der Wahrheit herstellen oder den vorhandenen Script-Check in CI/Release-Pfade heben.

### P3: UI-Code-First-Zwischenstand ist noch kein finaler EEZ-Produktstand

Nachweis: Ein vollstaendiges `.eez-project` existiert nicht; die produktiven Screenbaeume sind handgeschriebene LVGL-C-Funktionen. `build_modetabs()` speichert Tab-Handles auf Labels, weshalb `LvglUi` fuer Tab-Hintergruende `lv_obj_get_parent()` nutzen muss. Einzelne Magic-Farbwerte in Paper/Setup/Measurement/Remote sind noch nicht auf Styles/Palette zurueckgefuehrt.

Warum: Der aktuelle Stand ist funktional brauchbar und build-verifiziert, aber nicht die endgueltige Designer-/Style-Quelle. Ohne Konsolidierung droht visueller Drift beim naechsten EEZ-Export.

Mindestloesung: Entweder `.eez-project` als Quelle einfuehren oder den code-first-Vertrag bewusst dokumentieren; Tab-Handle-Semantik bereinigen und Farbcodes in benannte Styles/Konstanten ueberfuehren.

### P3: Test- und Build-Abdeckung ist lueckenhaft

Nachweis: AP-07-SG-Harness existiert; generelle automatisierte Tests sind nicht etabliert. Hardwaretests sind nicht Teil eines reproduzierbaren Reports.

Warum: Das Projekt enthaelt Safety-, Timing-, Mess- und Protokollpfade. Reine Kompilierbarkeit reicht fuer Regressionen nicht.

Mindestloesung: Kleine Host-/PIO-Harnesses fuer Protocol, PaperSlotCodec, MeasurementDomain, InputRouter und ExposureEdge-Cases ergaenzen; Hardware-Abnahmen separat dokumentieren.

## Stale-Dokumente und Schein-Erledigt-Kandidaten

Folgende Aussagen aus aelteren Dokumenten muessen bei weiterer Dokumentationspflege vorsichtig behandelt werden:

- „Lokaler TSL2561 fehlt“ ist veraltet; der Codepfad existiert. Offen bleibt nicht mehr die fehlende Degradationsstrategie, sondern nur die fehlende lokale I2C-Entklemmung ohne Neustart sowie die Hardwarevalidierung.
- „Remote-Servicepfad ist implementiert“ muss weiterhin praezise gelesen werden: Input/Render und kritische Command-Retries sind real, offene Punkte liegen jetzt vorrangig in Diagnose-Tiefe, Versionierung und Protokoll-Sync.
- „Measurement-Histogramm ist provisorisch“ gilt nicht mehr fuer den frueheren 1-Lux-Anker. Offen bleibt jetzt die absolute Papier-/Sensor-Kalibrierung, nicht mehr die session-relative Referenzbildung.
- „Lokale Encoder sind noch positionsbasiert“ gilt nicht mehr fuer den alten `EncoderState`-Wrapper. Offen bleibt hoechstens die reale Hardware-Abnahme der Encoderphasen, nicht mehr das fehlende Detent-/Diagnosemodell im Laufzeitpfad.
- „Zentrale EV-/Formatter-Architektur“ ist nur als Anfang vorhanden. Sie darf nicht als vollstaendige mathematische Basis fuer alle Workflows gelten.
- „PaperSlot-Recovery schreibt Defaults“ muss praezise formuliert werden: nur `FileNotFound` wird automatisch persistiert; InvalidBlob/Read/Format/Bank-Fehler bleiben sichtbar.
- „BW-Modus existiert“ bedeutet nur Shell und Modusplatzhalter, nicht produktive BW-Belichtung.

## Klärungsbedarf

- Welche reale C6-Firmwareversion ist auf dem Terminal geflasht und wie soll sie im Projekt versioniert werden?
- Wie wird der definierte Doppel-Sensorpfad (TSL2561 lokal closed-loop, TSL2591 remote auf Papierebene) praktisch kalibriert und mit Toleranzen/Abgleich dokumentiert, insbesondere unter Nutzung des vorhandenen 21-Stufen-Durchsichtsgraukeils?
- Welche Toleranzen gelten fuer Dosislinearitaet, Splitgrade-Anteile, ISO-P/ISO-R, Kopf-Latenz und thermische Abschaltung?
- Wie wird der gesetzte Default-Status (Demo vs. gemessen/importiert) im UI, in Persistenzmetadaten und beim Import sichtbar und erzwingbar gemacht?
- Welche alten Dokumente sollen nach Annahme dieser drei Konsolidierungsdateien nur noch als Historie gelten?
