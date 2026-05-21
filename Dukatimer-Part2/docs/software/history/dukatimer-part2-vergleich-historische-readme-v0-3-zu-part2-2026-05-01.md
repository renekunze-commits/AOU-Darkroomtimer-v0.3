# Vergleich der historischen README v0.3 mit dem aktuellen Dukatimer-Part2-Stand

Stand: 2026-05-01

## Anlass und Bewertungslogik

Diese Auswertung vergleicht die fruehe historische Zielbeschreibung aus `Projects/readme.md` mit dem heutigen Ist-Stand von `Dukatimer-Part2`.

Bewertungskategorien in diesem Dokument:

- Erfuellt: Das historische Ziel ist im aktuellen Code sichtbar und fachlich produktiv angeschlossen.
- Teilweise erfuellt: Architektur, Datenmodell oder Teilpfade existieren, aber der End-to-End-Workflow ist noch nicht komplett.
- Offen: Das Ziel ist noch nicht produktiv umgesetzt oder nur als Platzhalter/Backlog vorhanden.
- Bewusst ersetzt und uebertroffen: Die historische Forderung wird nicht 1:1 reproduziert, weil die aktuelle Architektur eine staerkere Nachfolge loest.

## Kurzfazit

Der aktuelle `Dukatimer-Part2` Stand erfuellt den fotografisch-technischen Kern der historischen README bereits in wichtigen Punkten: der NeoPixel-Lichtkopf ist als logische 16x16-Lichtflaeche vorhanden, der lokale TSL2561-Dosispfad ist produktiv an die `ExposureEngine` angeschlossen, Splitgrade ist als echter Workflow mit papiergetriebener Mathematik aktiv, die Paper-Profile existieren mit 20 Slots, und der thermische DS18-Pfad ist in die Laufzeitlogik integriert.

Nicht erreicht ist der damalige Funktionsumfang dort, wo aus der historischen Sicht ganze Anwender-Workflows gemeint waren: BW-Mischlicht als produktiver Modus, Teststrip, Burn/Dodge, Densitometer, Kalibrier-Wizard und eine voll ausgebaute Messpipeline sind im aktuellen Stand noch nicht fertig. Besonders auffaellig ist, dass die heutige UI-Basis zwar architektonisch viel sauberer ist, sicht- und bedienseitig aber noch auf einem Diagnose- beziehungsweise Placeholder-Niveau liegt.

Gleichzeitig uebertrifft der heutige Code die damaligen Anforderungen in mehreren tragenden Punkten deutlich. Part2 ersetzt die alte Einplatinenlogik durch eine klar getrennte Dual-MCU-Architektur mit sicherheitskritischer Autoritaet auf dem Teensy, versioniertem SharedProtocol mit Sequenznummern und CRC, robuster Persistenz inklusive Recovery-Pfaden, einer zentralen Input-Semantik ueber lokal, Remote und Wireless sowie einer wesentlich expliziteren Diagnose- und Zustandsmodellierung.

## Vergleichsmatrix

| Historisches Ziel aus README | Aktueller Stand in Part2 | Bewertung | Primaere Evidenz |
| --- | --- | --- | --- |
| RGB-NeoPixel-Lichtquelle fuer den Vergrösserer | Logischer 16x16-NeoPixel-Kopf mit 4 Segmenten, Framebuffer, weichen Uebergaengen und Timing-Statistik vorhanden | Erfuellt | `src/teensy/NeoPixelHead.h` |
| Geschlossene Dosisregelung ueber zweiten Sensor am Kopf | `ExposureEngine` integriert reale TSL2561-Luxwerte, summiert Luxsekunden, hat Null-Lux-Watchdog, Sensor-Fallback und praediktiven Shutoff | Erfuellt | `src/teensy/ExposureEngine.h`, `src/teensy/ExposureEngine.cpp`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| Splitgrade mit Soft/Hard getrennt | Aktiver `SplitgradeWorkflow` mit Execution-States, papiergetriebenen Targets, 0,5er-LUT und ISO-P/ISO-R-Modell | Erfuellt | `src/teensy/SplitgradeWorkflow.cpp`, `CHANGELOG.md` |
| BW Mixed Light als Einzelexposition 0.0 bis 5.0 | `ModeId::BlackWhite` existiert, produktiver BW-Workflow fehlt laut Backlog noch | Teilweise erfuellt | `src/teensy/ModeRuntimeState.h`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| Pre-Metering mit TSL2591, kabelgebunden oder wireless | Wireless-TSL2591/C6-Pfad ist real integriert; ein lokaler kabelgebundener TSL2591-Basisboardpfad ist im aktuellen Stand nicht sichtbar | Teilweise erfuellt | `src/teensy/MeasurementDomainService.cpp`, `src/esp32/WirelessRemoteGateway.cpp`, `Wireless TSL2591/src/main.cpp` |
| Densitometer-Modus auf Basis des Pre-Meterings | Historisch beschrieben, im aktuellen Part2 noch nicht als produktiver Workflow vorhanden | Offen | `docs/software/dukatimer-part2-migrations-backlog.md`, `docs/software/dukatimer-part2-priorisierte-todo-liste-2026-04-29.md` |
| Teststrip-Generator | Semantik am Kopfmodell vorbereitet, kompletter Workflow noch offen | Offen | `src/teensy/HeadSpectrumCommand.h`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| Burn/Dodge mit EV-Korrekturen | Semantik vorbereitet, produktiver Laufzeitpfad noch offen | Offen | `src/teensy/HeadSpectrumCommand.h`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| 20 Papierprofile speichern | `kPaperSlotCount = 20`, aktive Slotbank, Persistenz und Recovery vorhanden | Erfuellt | `src/teensy/PaperSlotBank.h`, `src/teensy/PaperSlotStorage.h`, `src/teensy/main.cpp` |
| Kalibrier-Wizard fuer papierspezifische K-Faktoren | Profilstruktur und Persistenz sind da, aber kein Wizard- oder Arbeitsfluss | Offen | `src/teensy/PaperExposureProfile.h`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| Temperaturkompensation ueber DS18B20 | DS18-Werte werden vom ESP-Service uebernommen, im Sensorzustand gehaertet und zur thermischen Leistungsbegrenzung genutzt | Erfuellt | `src/teensy/SensorManager.cpp`, `src/teensy/ExposureEngine.cpp`, `src/teensy/EspServiceLink.cpp` |
| Drei Encoder plus Taster fuer Blindbedienung | Drei lokale Encoder plus Taster sind im Pinmodell und in der Input-Normalisierung vorhanden; quadraturvalidierter Decoder bleibt offen | Teilweise erfuellt | `src/teensy/DukaTeenBoardPins.h`, `src/teensy/InputNormalizer.cpp`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| Nextion-Touchdisplay als Haupt-UI | Nicht uebernommen; ersetzt durch TFT plus Touch plus LVGL-Basis. Diese Basis laeuft, ist aber noch Placeholder-UI | Bewusst ersetzt und teilweise erfuellt | `platformio.ini`, `src/teensy/LvglUi.cpp`, `docs/software/dukatimer-part2-migrations-backlog.md` |
| RGB-LCD als zusaetzliches Farbrueckmeldesystem | Historisches LCD ist entfallen; im aktuellen `platformio.ini` ist das LCD explizit auskommentiert, weil TFT ueber SPI die Rolle uebernimmt | Bewusst ersetzt | `platformio.ini` |
| Piezo-Buzzer fuer Ticks und Feedback | In den aktuellen produktiven Teensy-Pfaden ist kein belastbarer lokaler Piezo-/Buzzer-Pfad sichtbar | Offen | `src/teensy/DukaTeenBoardPins.h`, `src/**` Suche ohne entsprechenden lokalen Laufzeitpfad |
| Hauptprogramm auf einem ESP32-S3 mit PSRAM/OPI | Nicht uebernommen; ersetzt durch Teensy als Hauptsystem plus ESP32-S3 als Service/Gateway | Bewusst ersetzt und uebertroffen | `platformio.ini`, `docs/software/dukatimer-part2-pflichtenheft.md` |
| Separates Wireless-TSL2591-Firmwareprojekt | Weiterhin vorhanden; heute als C6-Dumb-Terminal mit Display- und Eingabepfad in die Gesamtarchitektur eingebunden | Erfuellt und uebertroffen | `Wireless TSL2591/src/main.cpp`, `src/esp32/WirelessRemoteGateway.cpp`, `CHANGELOG.md` |

## Erfuellte Punkte im Vergleich zum historischen Stand

### 1. Closed-Loop-Belichtung ist wieder ein echter Kernpfad

Die historische README definiert den zentralen Unterschied zu einfachen Dunkelkammer-Timern ueber die Dosisregelung am Lichtkopf. Genau dieser Kern ist in Part2 wieder produktiv vorhanden. `ExposureEngine` arbeitet nicht nur zeitbasiert, sondern integriert im Dose-Modus reale Luxsekunden, verwaltet Restdosis und kann den Lichtkopf praediktiv abschalten. Dazu kommen Null-Lux-Fehlerfenster, Plausibilitaetspruefungen und ein sensorischer Fallback-Pfad. Damit ist die historische Grundidee des "nicht blind Sekunden zaehlen" klar wieder hergestellt.

### 2. Splitgrade ist im aktuellen Stand fachlich weiter als ein blosser Port

Der aktive `SplitgradeWorkflow` ist nicht nur ein Platzhalter. Er besitzt einen echten fachlichen Ablauf mit `ArmingSoft`, `ExposingSoft`, `WaitForFilter`, `ArmingHard`, `ExposingHard`, `Completed`, `Aborted` und `Fault`. Dazu kommt die papiergetriebene Mathematik mit 0,5er-LUT, exakter Grade-Normalisierung sowie ISO-P- und ISO-R-Auswertung. Gegenueber der historischen README ist der SG-Kern damit nicht nur wieder vorhanden, sondern im aktuellen Code sauberer expliziert und dokumentiert.

### 3. Die historische Forderung nach 20 Papierprofilen ist konkret abgebildet

Die historische README nennt "up to 20 paper types". In Part2 ist diese Zahl nicht nur lose geplant, sondern als `kPaperSlotCount = 20` im produktiven Datenmodell fest verankert. `PaperSlotBank`, `PaperExposureProfile`, `PaperSlotStorage` und die dazugehoerigen Adapter bilden eine reale Persistenzbasis. Die Slots werden geladen, normalisiert, bei Defekten auf Defaults zurueckgefuehrt und kontrolliert zurueckgeschrieben.

### 4. Temperaturpfad und thermische Rueckwirkungen sind produktiv verdrahtet

Die historische README wollte die LED-Heatsink-Temperatur im Dose-Modus beruecksichtigen. Part2 fuehrt diesen Gedanken ueber einen robusteren Servicepfad fort: DS18-Werte kommen vom ESP-Service, werden ueber `EspServiceLink` an den Teensy geliefert, im `SensorManager` in einen expliziten Sensor- und Thermal-Zustand ueberfuehrt und von der `ExposureEngine` fuer Leistungsbegrenzung bis hin zum Hard-Stop verarbeitet.

### 5. Das Wireless-Messgeraet ist wieder real angebunden

Das historische Projekt hatte ein separates Wireless-TSL2591-Handgeraet. Dieses Motiv lebt in Part2 klar weiter. `WirelessRemoteGateway` bindet das Handgeraet produktiv per ESP-NOW an, das C6-Projekt sendet Eingaben und Luxwerte, und `MeasurementDomainService` fuehrt Wireless-Luxdaten in einen dedizierten Measurement-Pfad ueber. Damit ist der historische Gedanke eines abgesetzten Mess- und Bediengeraets nicht verloren gegangen.

## Offene Punkte im Vergleich zur historischen README

### 1. BW Mixed Light ist fachlich noch nicht fertig

Die historische README beschreibt BW Mode als vollwertigen Modus mit dynamischer Gruen/Blau-Mischung. Im aktuellen Part2-Stand existiert zwar `ModeId::BlackWhite`, laut Migrationsbacklog ist BW aber derzeit nur als Shell-Workflow vorhanden. Historisch war diese Funktion im Zielbild bereits Kernbestandteil; heute ist sie noch nicht wieder voll produktiv.

### 2. Teststrip und Burn/Dodge sind noch keine benutzbaren Workflows

Die historischen Kernfeatures "Test Strip Generator" und "Burning / Dodging" sind aktuell nicht als fertige End-to-End-Pfade vorhanden. Dass `HeadSpectrumSemantic` bereits `Burn`, `TestStrip`, `Preflash` und `Calibration` kennt, zeigt die fachliche Zielrichtung, ersetzt aber keinen lauffaehigen Anwender-Workflow. Gegen den damaligen Zieltext bleibt das ein klar offener Bereich.

### 3. Papierkalibrierung ist strukturell vorbereitet, aber praktisch noch offen

Im historischen README war der Kalibrier-Wizard ein zentrales Nutzversprechen. In Part2 existieren heute bereits Profilstruktur, Slotbank, Persistenz und papierbezogene Mathematik-Anker, aber laut Backlog fehlt weiterhin der eigentliche Wizard- oder Arbeitsfluss. Der Datenkern ist also deutlich weiter als null, der Benutzerfluss aber noch nicht auf dem historischen Zielstand.

### 4. Densitometer und komplette Messpipeline fehlen noch

Die historische README verbindet den Pre-Metering-Sensor direkt mit Densitometer- und LogD-Aufgaben. In Part2 existiert bereits eine erste Measurement-Domain mit Sessiongrundlagen, Wireless-Ingestion, Undo-Historie und Histogramm-Ankern. Trotzdem fehlen nach wie vor der komplette Densitometer-Workflow, Multi-Spot-Mittelung und eine konsistente EV-plus-Lux-Aufbereitung ueber alle Messpfade. Gegen die historische Funktionsbeschreibung ist das noch offen.

### 5. Die historische UI ist nicht eins zu eins wieder da, und die neue UI ist noch nicht ausgereift

Historisch bestand die Bedienung aus Nextion-Touch, RGB-LCD, Piezo und Blindbedienung ueber Encoder. Die heutige Plattform ersetzt das bewusst durch Teensy, TFT, Touch und LVGL. Architektonisch ist das sinnvoll, aber praktisch ist die UI noch kein ausgebautes Produkt-Interface. `LvglUi::createPlaceholderScreen()` baut weiterhin im Kern eine gestapelte Diagnoseoberflaeche aus Labels und Overlays. Damit ist der alte Hardwaremix zwar bewusst abgeloest, die neue Oberflaeche liegt aber sichtbar noch hinter dem damaligen Anspruch "practical, fast, smooth to use".

### 6. Der historische TSL2591-Doppelpfad ist heute nur teilweise wiederhergestellt

Historisch war das Pre-Metering entweder kabelgebunden ueber I2C oder wireless moeglich. Im aktuellen Part2-Stand ist der Wireless-Zweig real vorhanden. Ein produktiver lokaler kabelgebundener TSL2591-Basisboardpfad ist dagegen nicht sichtbar; lokal dominiert heute der TSL2561-Kopfpfad. Wenn das alte Ziel explizit beide Varianten meinte, ist Part2 hier bisher nur teilweise deckungsgleich.

### 7. Der Remote-Pfad ist fachlich noch nicht ganz geschlossen

Die Wireless-Basis ist produktiv, aber noch nicht auf voller historischer Reife: `WirelessRemoteGateway::applyTeensyCommand()` konsumiert Remote-Kommandos wie `RemoteMeasurementStart`, `RemoteMeasurementCancel` und `RemoteHaptic` derzeit ohne fachliche Weiterleitung. Das C6-Terminal uebernimmt empfangene `RemoteDisplayPayload` ausserdem ohne sichtbare Freshness-Pruefung auf `sequenceNumber`. Das ist weniger ein Widerspruch zum alten README als eine heutige Restluecke im Anspruch eines wirklich fertigen Handterminals.

## Punkte, in denen der aktuelle Code die damaligen Anforderungen deutlich uebertrifft

### 1. Die Systemarchitektur ist heute sicherer und technisch sauberer als das historische Einplatinenmodell

Die historische README beschreibt den damaligen Wechsel auf einen leistungsfaehigeren ESP32-S3 als Antwort auf wachsende Komplexitaet. Part2 geht einen Schritt weiter und trennt die Gesamtverantwortung bewusst: der Teensy 4.1 ist Hauptsystem fuer UI, Belichtungslogik und sicherheitskritische Ausgaenge, der ESP32-S3 ist nur Service-, Sensor- und Gateway-MCU. Diese Aufteilung ist fotografisch und embedded-technisch ein deutlicher Fortschritt, weil eine laufende Belichtung nicht von WLAN, ESP-NOW, OneWire oder sonstigen Serviceaufgaben abhaengt.

### 2. SharedProtocol und Gateway-Design liegen weit ueber dem historischen Ad-hoc-Niveau

Historisch reichte es, dass ein Wireless-Sensor Messwerte lieferte. Part2 hat dafuer inzwischen ein explizit versioniertes Nachrichtenmodell mit Header, Nachrichtentypen, Sequenznummern, CRC, Heartbeats, Diagnosen, Remote-Renderdaten, VFS-Frames und klaren Rollen zwischen Teensy, ESP-Service und Wireless-Terminal. Das ist ein qualitativer Sprung: nicht nur "Funk geht", sondern ein wartbares, pruefbares und erweiterbares Protokollgeruest.

### 3. Persistenz und Fehlertoleranz sind wesentlich robuster als die historische Zielbeschreibung

Die historische README nennt PSRAM/EEPROM fuer die PaperBank. Part2 geht bei Persistenz und Datenintegritaet viel weiter: Paper-Slots besitzen Schema-Versionen, Blob-Header, CRC, Validierungsregeln, differenzierte Fehlercodes und einen Temp-Datei-plus-Rename-Schreibpfad. Korruptionsfaelle werden nicht still geschluckt, sondern diagnostiziert und kontrolliert auf Defaults zurueckgefuehrt. Das uebertrifft die alte Forderung nicht nur funktional, sondern vor allem qualitativ.

### 4. Das Eingabesystem ist heute als gemeinsame Semantik modelliert statt als verstreute Bedienlogik

Die historische README beschreibt primär Hardwareelemente: drei Encoder, Taster, Display, Buzzer. Part2 formuliert daraus ein explizites semantisches Eingabesystem: `InputNormalizer`, `InputRouterPolicy`, modaler EventGuard, Touch-vs-Encoder-Fokus und gemeinsame Eventsemantik fuer lokale Encoder, Encoder 4 und Wireless-Eingaben. Diese Trennung ist fuer einen wachsenden Multimodus-Timer wesentlich staerker als die alte, hardwarezentrierte Sicht.

### 5. Die Messdomäne ist heute als eigenständiger Dienst angelegt

Historisch war das Messsystem eng an den damaligen Ablauf gekoppelt. In Part2 existiert mit `MeasurementDomainService` bereits ein eigener fachlicher Dienst, der lokale und Wireless-Luxwerte bewertet, Staleness behandelt, Messpunkte sequenzbasiert einspeist, Undo-Historie fuehrt und Histogramm-Anker vorbereitet. Selbst dort, wo der volle Funktionsumfang noch nicht fertig ist, ist die Architektur bereits deutlich besser fuer spaetere Densitometer-, Spot- und Histogramm-Workflows vorbereitet als der historische Stand.

### 6. Der Lichtkopf ist heute nicht nur ein Ausgang, sondern eine gemessene Runtime-Komponente

Historisch war das NeoPixel-Panel der eigentliche Belichter. Part2 instrumentiert diesen Pfad zusaetzlich: `NeoPixelHead` sammelt `present()`-Statistiken, die Laufzeit kann in die praediktive Abschaltlogik der `ExposureEngine` einfliessen, und Timing-Dokumentation plus Validierung sind angelegt. Damit wird aus dem Lichtkopf nicht nur eine Lichtquelle, sondern ein beobachtbarer Bestandteil der Regelkette.

### 7. Der aktuelle Code modelliert Diagnose und Fehler als First-Class-Thema

Im historischen README ist "core logic rock solid" eher eine qualitative Aussage. Part2 macht diesen Anspruch explizit technisch sichtbar: Sensor-Health, Sample-Validity, Thermal-State, Link-Health, DiagnosticCodes, Persistenzfehler, Snapshot-Diagnostik und sichtbare Runtime-Telemetrie ziehen sich durch die Architektur. Das ist eine deutliche Uebererfuellung gegenueber dem damaligen Zieltext, weil hier nicht nur Funktionalitaet, sondern auch Betriebsbeobachtbarkeit systematisch mitgebaut wird.

### 8. Das Wireless-Handgeraet ist heute naeher an einem echten Terminalkonzept

Historisch war das Wireless-Geraet bereits mehr als nur ein Sensor. Part2 zieht diese Richtung konsequenter: das C6-Terminal hat eigenes Display-Rendering, Eingabepaket, Lux-Rueckkanal und eine feste Rolle als Dumb Terminal. Auch wenn einzelne Komfortpfade noch offen sind, ist die Gesamtidee klarer getrennt: keine eigene Papierlogik, keine eigene Belichtungsmathematik, dafuer ein sauber definierter Fernbedien- und Messpfad.

## Gesamturteil

Verglichen mit der historischen README ist `Dukatimer-Part2` heute kein nostalgischer Rueckbau, sondern eine fachlich ernsthafte Nachfolgearchitektur. Der Kern der Dunkelkammer-Idee lebt bereits sichtbar: geregelte Belichtung, Splitgrade, Papiermodell, thermischer Pfad, Remote-Messgeraet und NeoPixel-Lichtkopf sind nicht mehr nur Absicht.

Was noch fehlt, ist nicht die Basis, sondern der breite zweite Ring darum: die produktive BW-/Burn-/Teststrip-Familie, Kalibrier- und Densitometer-Workflows sowie eine wirklich ausgereifte Alltags-UI. Gleichzeitig ist die technische Qualitaet des Unterbaus bereits in mehreren Bereichen deutlich hoeher als in der historischen Zielbeschreibung. Genau darin liegt der eigentliche Fortschritt von Part2: weniger improvisierte Gesamtfunktion, mehr tragfaehige Systemarchitektur.
