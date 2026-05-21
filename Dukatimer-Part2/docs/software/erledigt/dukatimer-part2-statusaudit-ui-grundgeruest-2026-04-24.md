# Dukatimer-Part2 Statusaudit: UI-Grundgeruest, Historischer Abgleich, Landminen

Stand: 2026-04-25

## Ziel dieses Audits

Dieses Dokument bewertet den aktuellen Part2-Implementierungsstand als Grundgeruest fuer eine spaetere vollwertige, durchgaengige und blind bedienbare UI- und Workflow-Schicht.

Es liefert:

- Soll-Ist-Abgleich gegen das Pflichtenheft
- historischen Vergleich zu v0.3 und v0.9
- priorisierte Landminen und Stolperfallen
- konkrete Steuerempfehlungen fuer die naechsten Umsetzungsschritte

## Bewertungsrahmen

### Eingesetzte Soll-Quellen

- docs/software/dukatimer-part2-pflichtenheft.md
- docs/software/dukatimer-part2-grundarchitektur-input-ui-wireless.md
- docs/software/dukatimer-part2-migrations-backlog.md

### Historische Referenzquellen

- Dukatimer v0.3/src/Mode_SG.cpp
- Dukatimer v0.9/src/AppManager.cpp
- Dukatimer v0.9/src/SGTimeApp.cpp
- Dukatimer v0.9/src/SGDoseApp.cpp

### Bewertungsstufen

- Erfuellt: im Grundgeruest klar und konsistent umgesetzt
- Teilweise: sichtbar begonnen, aber mit klaren Luecken
- Offen: fuer den aktuellen Slice noch nicht umgesetzt
- Landmine: Risiko, das spaeter teuer wird, wenn jetzt nicht korrigiert

## Executive Summary

Der aktuelle Stand ist als technisches Grundgeruest und als erster historisch plausibler SG-Laufzeitkern brauchbar, aber noch nicht als durchgaengige, blind bedienbare und fachlich reproduzierbare Part2-Basis ausreichend.

Positiv:

- SG besitzt nicht mehr nur eine Shell, sondern einen expliziten Execution-State inkl. Start-, WaitForFilter-, Abort- und Done/Fault-Uebergaengen.
- ExposureEngine ist real in den Runtime-Pfad eingebunden, inklusive Closed-Loop-Dosisintegration, praediktivem Shutoff, Sensor-Watchdog, Plausibilitaetspruefung und Thermal-Hard-Stop.
- Lokale und entfernte Eingaben laufen ueber zentrale semantische Normalisierung; LongPress/RepeatPress/Measure/Undo sind lokal wie remote im Bedienpfad angelegt.
- Head-Arbitration, Exposure-Override und deterministische Soft-Start/Soft-Stop-Rampen sind vorhanden.

Kritisch:

- Der lokale TSL2561-Dosispfad fehlt weiterhin; Dosisstart bleibt damit praktisch auf injizierte Testsamples beschraenkt.
- UI ist weiterhin ein Debug-/Statusscreen und noch kein blind bedienbares Screen-/Overlay-Grundgeruest.
- Papierslots, Persistenz und papiergetriebene SG-Mathematik fehlen noch.
- Inter-MCU-/Wireless-Render- und Messkommando-Pfade sind nur strukturell vorbereitet, nicht end-to-end in Betrieb.

Fazit fuer Steuerung:

- Der Schritt war fachlich richtig und hat zwei zentrale Landminen bereits beseitigt: Eventparitaet und SG/Exposure-Kopplung.
- Der naechste Umbauaufwand entsteht nicht mehr primaer im Inputpfad, sondern in Sensoranbindung, Persistenz, UI-Fuehrung und Remote-Command-Roundtrip.

## Historischer Abgleich (v0.3/v0.9 -> Part2)

### Was historisch fachlich bestaetigt ist

1. SG ist kein einzelner Parameterdialog, sondern ein Zustandsablauf.
2. Es gibt klar getrennte Rollen zwischen globalem Event-Guard und modusinterner Logik.
3. Bedienung ist schnellzugriffsfaehig (inkl. LongPress/Sonderevents) und nicht rein menueartig.

### Evidenz

- v0.9 globaler Guard und Modusrouting in AppManager
  - Dukatimer v0.9/src/AppManager.cpp:158
  - Dukatimer v0.9/src/AppManager.cpp:177
- v0.9 SG als explizite Ablaufmaschine (IDLE -> SOFT -> WAIT_FOR_FILTER -> HARD)
  - Dukatimer v0.9/src/SGDoseApp.cpp:73
  - Dukatimer v0.9/src/SGDoseApp.cpp:142
  - Dukatimer v0.9/src/SGDoseApp.cpp:174
- v0.9 SG-Zeit mit Edit-Fokus und Start/Abort
  - Dukatimer v0.9/src/SGTimeApp.cpp:52
  - Dukatimer v0.9/src/SGTimeApp.cpp:84
  - Dukatimer v0.9/src/SGTimeApp.cpp:90
- v0.3 SG-Input mit mehreren Achsen plus LongPress-Schnellpfaden
  - Dukatimer v0.3/src/Mode_SG.cpp:101
  - Dukatimer v0.3/src/Mode_SG.cpp:149
  - Dukatimer v0.3/src/Mode_SG.cpp:167
  - Dukatimer v0.3/src/Mode_SG.cpp:213

### Aktuelle Part2-Einordnung

Part2 uebernimmt mittlerweile mehr vom historischen Kern als am 2026-04-24 bewerteten Stand: semantisches Inputrouting, workfloweigene Execution-Commands, WaitForFilter-/Abort-Pfad sowie Exposure-Kopplung liegen im Runtime-Pfad. Noch nicht durchgaengig hergestellt sind jedoch globaler EventGuard und Boot-Readiness-Policy, papiergetriebene SG-Mathematik, realer lokaler Dosis-Sensorpfad sowie der fachlich verdrahtete Remote-Command-/Render-Roundtrip.

## Soll-Ist-Abgleich gegen Pflichtenheft (fokussiert auf Grundgeruest)

### PF-M01 Systemstart und sicherer Grundzustand

Bewertung: Teilweise

- Positiv: lokale Schalterlogik, Head-Arbitration und Exposure-Override greifen jetzt zusammen; Fault-/Done-/Nicht-Expose-Phasen schalten den Head reproduzierbar auf sicheren Zustand.
- Luecke: globale Boot-/Readiness-Policy ist noch nicht vollstaendig; der Dosispfad hat keinen realen lokalen TSL2561-Readout.
- Evidenz:
  - src/teensy/LightController.cpp:13
  - src/teensy/main.cpp:532
  - src/teensy/main.cpp:615
  - src/teensy/ExposureEngine.cpp:111

### PF-M02 Lokale Bedienung am Hauptgeraet

Bewertung: Teilweise

- Positiv: lokale Encoder/Taster werden gelesen, lokale Gesten erzeugt und zentral semantisch normalisiert; LongPress/RepeatPress/Measure/Undo sind lokal vorhanden.
- Luecke: Encoderpfad bleibt bring-up-orientiert (/4), und ein explizites Fokus-/Router-Modell zwischen Encoder, Touch und Overlay fehlt weiterhin.
- Evidenz:
  - src/teensy/main.cpp:380
  - src/teensy/main.cpp:417
  - src/teensy/main.cpp:479
  - src/teensy/InputNormalizer.cpp:15
  - src/teensy/InputNormalizer.cpp:24

### PF-M02a Encoderbetrieb auf dem Teensy

Bewertung: Teilweise mit Landmine

- Positiv: dedizierte lokale Encoderverarbeitung ist vorhanden.
- Landmine: weiterhin positionsbasierter Bring-up-Teiler (/4) statt expliziter quadraturvalidierter Ereignisschicht.
- Evidenz:
  - src/teensy/main.cpp:133
  - src/teensy/main.cpp:148
  - src/teensy/main.cpp:161

### PF-M02b UI-/LVGL-/TFT-Basis

Bewertung: Teilweise

- Positiv: LVGL, Touch-Read und Snapshot-Renderpfad sind aktiv.
- Luecke: weiterhin Placeholder-Screen statt echter UiKernel/Screen-Navigation.
- Evidenz:
  - src/teensy/LvglUi.cpp:308
  - src/teensy/LvglUi.cpp:500

### PF-M02c Lokale Schalterlogik

Bewertung: Erfuellt (im aktuellen Grundgeruest)

- Fokus/Save/Room inkl. SaveLatch und Prioritaet sind zentral umgesetzt.
- Evidenz:
  - src/teensy/LightController.h:23
  - src/teensy/LightController.h:39
  - src/teensy/LightController.h:48

### PF-M04 Splitgrade-Belichtung

Bewertung: Teilweise

- Positiv: dedizierter Splitgrade-Workflow mit expliziten Execution-States, WaitForFilter, Abortroute und Command-Bridge zur ExposureEngine.
- Luecke: SG-Mathematik, papiergetriebene Targets und Mess-/Histogramm-Session fehlen; der Dosisbetrieb ist ohne echten lokalen TSL-Pfad fachlich noch nicht abnahmefaehig.
- Evidenz:
  - src/teensy/SplitgradeWorkflow.cpp:119
  - src/teensy/SplitgradeWorkflow.cpp:163
  - src/teensy/SplitgradeWorkflow.cpp:295
  - src/teensy/main.cpp:301
  - src/teensy/ExposureEngine.cpp:111

### PF-M07 ExposureEngine

Bewertung: Teilweise

- Positiv: ExposureEngine wird aus dem Hauptworkflow getrieben; StartTime/StartDose, Sensorbeobachtung, Closed-Loop-Dosisintegration, Watchdog, Plausibilitaetspruefung und Thermal-Hard-Stop sind im Runtime-Pfad aktiv.
- Luecke: realer lokaler TSL2561-Readout, papier-/profilgetriebene Sollwertableitung und ein vollstaendiger globaler EventGuard fehlen weiterhin.
- Evidenz:
  - src/teensy/main.cpp:301
  - src/teensy/main.cpp:615
  - src/teensy/ExposureEngine.cpp:111
  - src/teensy/ExposureEngine.cpp:316
  - src/teensy/ExposureEngine.cpp:333
  - src/teensy/ExposureEngine.cpp:349

### PF-M15 Inter-MCU-Basisdienst

Bewertung: Teilweise

- Positiv: SharedProtocol, versionierter Framing-Kern, Heartbeat, InputEvent, ServiceSnapshot, WirelessSnapshot und VFS-Basis sind vorhanden.
- Luecke: Capability-Bits fuer Remote-Render genuegen noch nicht fuer Abnahme; TeensyCommand-/Diagnostic-Pfade sind auf der Teensy-Seite nicht fachlich verdrahtet, und die ESP-Seite speichert Kommandos bislang nur als lastCommand_.
- Evidenz:
  - lib/SharedProtocol/DukatimerProtocol.h:1
  - src/teensy/EspServiceLink.cpp:167
  - src/teensy/EspServiceLink.cpp:265
  - src/esp32/TeensyLinkService.cpp:441

### PF-M17 Wireless-Basisintegration

Bewertung: Teilweise

- Positiv: Wireless-Status und Remote-Input-Events laufen in den Runtime-Status und durch die gleiche semantische Action-Schicht; SG verarbeitet Measure/Undo/Confirm, wenn diese Events ankommen.
- Luecke: SG-spezifische Messkommandos, Render-Roundtrip und reale Remote-Hardwareintegration sind noch nicht durchgaengig umgesetzt.
- Evidenz:
  - src/teensy/main.cpp:280
  - src/teensy/InputNormalizer.cpp:15
  - src/teensy/InputNormalizer.cpp:19
  - src/teensy/SplitgradeWorkflow.cpp:295
  - src/teensy/LvglUi.cpp:412
  - src/teensy/LvglUi.cpp:415

### PF-S01 Historische Bedienparitaet

Bewertung: Teilweise mit Risiko

- Positiv: Schnellzugriffe (LongPress, Measure, Undo, Start/Confirm) sind semantisch angelegt, und SG arbeitet nicht mehr nur als Parameterpanel.
- Risiko: quadraturvalidierter Encoderpfad, Fokusmodell, blinde Overlay-Fuehrung und papier-/messgetriebene Rueckmeldelogik fehlen weiterhin.

### NF-08 Encoder-Stabilitaet

Bewertung: Teilweise mit Landmine

- Risiko: Bring-up-Skalierung und pollingbasierte Positionsableitung koennen bei hoher Bediengeschwindigkeit Drift/Verlust erzeugen.

### NF-09 Dokumentierte UI-Basis

Bewertung: Erfuellt auf Doku-Ebene, Teilweise in Implementierung

- Dokumentation ist vorhanden.
- Implementierung entspricht noch nicht dem dokumentierten UiKernel/Screen-Lifecycle-Zielbild.

## Priorisierte Landminen und zukuenftige Probleme

## P0 (vor naechstem grossen Modusausbau loesen)

1. Lokaler TSL2561-Dosispfad: Implementiert (2026-04-25)

- Evidenz:
  - src/teensy/SensorManager.h: Deklaration und I2C-Parameter (kTslI2cOperationBudgetMs)
  - src/teensy/SensorManager.cpp: Initialisierung, ISR-Pfad, Poll/Read, I2C-Budget und Sample-Publishing
  - src/teensy/main.cpp: `sensorManager.begin(...)` und `exposureEngine.observeSensorStatus(sensorManager.state(), ...)`
  - src/teensy/ExposureEngine.cpp: Konsumiert `SensorRuntimeStatus` und verwendet TSL-Samples fuer Closed-Loop-Dosis
- Status:
  - Implementiert: `SensorManager` initialisiert und liest den lokalen TSL2561 (Interrupt- und Poll-Pfade), publiziert Lux-Samples inklusive Health/Validity/Age; I2C-Operation-Budget und Interrupt-Watchdog sind aktiv; `ExposureEngine` beobachtet und nutzt die Werte fuer Closed-Loop.
- Naechste Schritte (Validierung):
  - In-field-Validierung: Sensor-Verdrahtung pruefen und Testmessungen durchfuehren.
  - Messungen: Sample-Freshness, `present()`-/Sample-Latenz messen; bei Bedarf `kTslI2cOperationBudgetMs` anpassen.
  - Tests & Kalibrierung: Unit/Integration-Tests ergaenzen und Kalibrier-Flow (Offset/Gain) anlegen.

1. Papierslots, Persistenz und Version/CRC fehlen

- Evidenz:
  - src/teensy/PaperExposureProfile.h:48
  - src/teensy/main.cpp:185
  - src/teensy/main.cpp:237
- Auswirkung:
  - Papierkalibrierung, SG-Vorschlagslogik und reproduzierbare Fachablaeufe haben noch keinen belastbaren Datentraeger.
  - Spaetere Migration wird teurer, wenn Slot-/Versionierungsregeln nicht jetzt festgelegt werden.
- Empfohlene Gegenmassnahme:
  - persistente Paper-Slot-Verwaltung mit Version/CRC und Lade-/Speicherpfad jetzt festziehen.

1. UI bleibt Debug-/Statusscreen statt blind bedienbarer Bedienfuehrung

- Evidenz:
  - src/teensy/LvglUi.cpp:352
  - src/teensy/LvglUi.cpp:412
  - src/teensy/LvglUi.cpp:421
- Auswirkung:
  - Kein taktiles/kognitives Muskelgedaechtnis, keine klaren Navigationsebenen, hoher Bedienfehlerdruck im Dunkelraum.
- Empfohlene Gegenmassnahme:
  - minimalen SG-Hauptscreen mit Overlay-Kanal fuer Fehler/Confirm und festen Encoderrollen pro Ebene einziehen.

## P1 (im naechsten Sprint loesen)

1. ModeCoordinator ist auf zwei Workflows hart kodiert

- Evidenz:
  - src/teensy/ModeCoordinator.h:13
  - src/teensy/ModeCoordinator.cpp:5
- Auswirkung:
  - spaeterer Ausbau (BW, Burn, Teststrip, Setup, Densitometer, Preflash) erzwingt API-Umbau.
- Empfohlene Gegenmassnahme:
  - dynamisch konfigurierbare Workflow-Registry (Array/View) statt fester Zweierkonstruktor.

1. Encoderpfad noch bring-up-orientiert statt validierter quadraturbasierter Ereignisschicht

- Evidenz:
  - src/teensy/main.cpp:133
  - src/teensy/main.cpp:148
  - src/teensy/main.cpp:161
- Auswirkung:
  - Risiko fuer fehlerhafte Schritte bei schneller Bedienung, besonders kritisch fuer blindes Arbeiten.
- Empfohlene Gegenmassnahme:
  - quadraturvalidierten Step-Decoder mit Richtungs-/Uebergangsfilter und explizitem Lost-Step-Monitoring einziehen.

1. TeensyCommand-/Remote-Render-/Messkommando-Pfad ist nur strukturell vorhanden

- Evidenz:
  - src/teensy/EspServiceLink.cpp:167
  - src/teensy/EspServiceLink.cpp:265
  - src/esp32/TeensyLinkService.cpp:441
- Auswirkung:
  - Wireless wirkt im Statusbild weiter fortgeschrittener als im Fachbetrieb.
  - Der erste echte Remote-Messslice droht sonst mit Parallelpfaden statt sauberer End-to-End-Kopplung.
- Empfohlene Gegenmassnahme:
  - TeensyCommand, Renderdaten und SG-spezifische Messkommandos auf beiden MCU-Seiten fachlich verdrahten.

1. Touch ist verbunden, InputRouter-Basis ist vorhanden, aber Workflow-Integration bleibt unvollstaendig

- Evidenz:
  - src/teensy/InputRouterPolicy.h:1
  - src/teensy/InputRouterPolicy.cpp:1
  - src/teensy/main.cpp:492
  - src/teensy/LvglUi.cpp:509
- Auswirkung:
  - Der Grundkonflikt (Encoderrotation waehrend Touch/Modal) ist baseline-seitig entschaerft,
    aber ein vollstaendiger Touch-Bedienworkflow inkl. Screen-Navigation und Dialogfokus fehlt weiterhin.
- Empfohlene Gegenmassnahme:
  - InputRouter in den naechsten Slices um explizite Touch-Ziele, Dialogfokus und globale Guard-Policy erweitern.

## P2 (frueh planen, spaeter implementieren)

1. SG-Runtime-State ist nativ in ModeRuntimeState eingebettet

- Evidenz:
  - src/teensy/ModeRuntimeState.h:41
  - src/teensy/ModeRuntimeState.h:49
- Auswirkung:
  - bei vielen Modi droht wachsende zentrale Zustandsstruktur mit enger Kopplung.
- Empfohlene Gegenmassnahme:
  - moduspezifische UI-DTOs ueber Presenter-Layer statt breiter globaler Runtime-State-Ausdehnung.

1. Blindbedienung ist noch nicht als explizites UX-Sicherheitsziel operationalisiert

- Evidenz:
  - PF-S01 vorhanden, aber keine konkrete Bediennorm im Code.
- Auswirkung:
  - hohe Gefahr von inkonsistenten Interaktionsmustern je Modus.
- Empfohlene Gegenmassnahme:
  - Blind-UI-DoD definieren (konstante Encoderrollen, bestaetigte kritische Aktionen, eindeutige Zustandscues, Undo-Verfuegbarkeit, minimaler Blickbedarf).

## Steuerempfehlung fuer die naechsten 2 Umsetzungsschritte

### Schritt A (blockierende Fachbasis)

1. lokalen TSL2561-Dosispfad real anbinden und gegen Watchdog-/Plausibilitaets-/Thermalpfade testen.
2. Paper-Slot-/Persistenzschicht mit Version/CRC einziehen.
3. UI vom Snapshotscreen auf minimalen SG-Hauptscreen plus Fehler/Confirm-Overlay umstellen.

### Schritt B (naechster Integrationsschnitt)

1. SG-Mathematik und papiergetriebene Targets auf die bestehenden Execution-States mappen.
2. TeensyCommand-, Remote-Render- und Messkommando-Pfade end-to-end verdrahten.
3. ModeCoordinator/InputRouter fuer mehr Modi, Touch-Fokus und globalen EventGuard auf eine erweiterbare Form bringen.

## Ampelbewertung

- Architektur-Basis: Gelb (deutlich stabilisiert, aber noch nicht skaliert)
- SG-Grundgeruest: Gelb (Ablaufkern real, fachliche Vollstaendigkeit offen)
- Blindbedienbarkeit: Rot (entscheidende Interaktionsbausteine fehlen)
- Risiko fuer spaeteren Umbauaufwand: Mittel bis hoch; Input-/SG-Kopplungsrisiko ist gesunken, Sensor-/Persistenz-/Remote-Risiko bleibt kritisch

## Delta-Update 2026-04-25 (korrigiert nach Re-Audit)

Dieses Delta ersetzt die uebereilten Zwischenschluesse vom 2026-04-24-Audit durch den gegen Code und Backlog geprueften Stand.

1. Fruehere Audit-Aussagen zur lokalen Event-Luecke und zur fehlenden SG/Exposure-Kopplung sind ueberholt.

- Lokale Pfade erzeugen jetzt Press, LongPress, RepeatPress sowie Measure/Undo.
- SG besitzt explizite Execution-States und treibt die ExposureEngine ueber eine Command-Bridge statt ueber ad-hoc Trigger.

1. Der Exposure-Slice ist fachlich deutlich weiter als im ersten Auditstand.

- Closed-Loop-Dosisintegration, praediktiver Shutoff, Sensor-Watchdog, Null-Lux-Plausibilitaet und Thermal-Hard-Stop sind implementiert.
- Exposure-Override, RuntimeOutputLimit-Skalierung sowie deterministische Soft-Start/Soft-Stop-Rampen liegen im Head-Pfad.

1. Der Inter-MCU-Befund wurde nach unten korrigiert.

- PF-M15 und damit auch MB-03 bleiben Teilweise: Capability-Bits und Payload-Typen sind vorhanden, aber TeensyCommand-, Render- und Messkommando-Pfade sind noch nicht end-to-end in Betrieb.

1. Weiterhin offen (Audit-Stand 2026-04-24; Statuskorrekturen siehe Delta-Update):

- lokaler TSL2561-Dosispfad (implementiert; Feldvalidierung offen)
- Papierslots und Persistenz mit Version/CRC (Basis inzwischen umgesetzt; Restintegration bleibt Fokus)
- minimaler SG-Hauptscreen mit Overlay-Fuehrung (Basis-Slice gestartet)
- fachlich verdrahteter Remote-Command-/Render-Roundtrip

## Delta-Update 2026-04-28 (Teensy-PSRAM und Speicheroptimierung)

Ab diesem Stand wird Teensy 4.1 mit bestuecktem externem PSRAM (8MB) als
Projektannahme gefuehrt.

Hinweis aus spaeterer Hardwarevalidierung: Ein isolierter Teensy-PSRAM-Probe-
Sketch hat am 2026-05-03 fuer die aktuelle Duka-Teen-Hardware `16 MB`
verifiziert. Die `8MB`-Angabe beschreibt hier bewusst nur den damaligen
Arbeitsstand dieses Delta-Updates.

Konfigurationsentscheidungen:

- [Dukatimer-Part2/platformio.ini](Dukatimer-Part2/platformio.ini):
  - `DUKATIMER_TEENSY_HAS_PSRAM=1`
  - `DUKATIMER_TEENSY_PSRAM_MB=8`
  - spaeter auf `DUKATIMER_TEENSY_PSRAM_MB=16` korrigiert; dieser Eintrag bleibt
    als historischer Slice-Stand sichtbar
  - `TEENSY_OPT_SMALLEST_CODE_LTO` zur Code-/Link-Groessenreduktion
- Hinweis: Die Teensy-Platform in PlatformIO bietet fuer T4.1 keine eigene
  `board_build.psram`-Option wie ESP32; PSRAM-Nutzung erfolgt ueber
  Speicherattribute (`EXTMEM`) im Code.

Bisher umgesetzte PSRAM-nahe Optimierungen:

- Paper-Slot-Bank wurde aus internem RAM ausgelagert:
  - `src/teensy/main.cpp`: `PaperSlotBank` liegt in `EXTMEM`.
- Blob-I/O-Puffer des Slot-Stores wurde in externen Speicher gelegt:
  - `src/teensy/PaperSlotStorage.cpp`: statischer `EXTMEM`-Puffer statt Heap-Buffer.

Statuskorrektur seit Audit-Stand:

- Papierslot-Persistenz (Version/CRC) ist als Basis umgesetzt und in den
  Teensy-Bootpfad integriert (Load mit kontrolliertem Default-Fallback/Save).

Build-Validierung nach PSRAM-/LTO-Anpassung (`pio run -e teensy41`):

- Ergebnis: SUCCESS
- teensy_size:
  - FLASH: code 204640, data 49408, headers 9116
  - RAM1: variables 197024, code 199592, padding 29784, free local variables 97888
  - RAM2: variables 146816, free malloc/new 377472
  - EXTRAM: variables 6432

Weitere umgesetzte Optimierungen (Slice 2, 2026-04-28):

- Nicht-DMA-kritische Inter-MCU-Puffer in EXTMEM verschoben:
  - `src/teensy/main.cpp`: globale `EspServiceLink`-Instanz liegt in `EXTMEM`.
- Groesse LVGL-Heap in EXTMEM verankert:
  - `include/lv_conf.h`: `LV_ATTRIBUTE_LARGE_RAM_ARRAY` auf `.externalram` gesetzt
    (betroffen: LVGL `work_mem_int`-Pool).

Re-Validierung nach Slice-2-Optimierung (`pio run -e teensy41`):

- Ergebnis: SUCCESS
- teensy_size:
  - FLASH: code 204640, data 49408, headers 9116
  - RAM1: variables 56896, code 199592, padding 29784, free local variables 238016
  - RAM2: variables 146816, free malloc/new 377472
  - EXTRAM: variables 146560
- Delta gegen vorherige Messung:
  - RAM1 variables: -140128
  - freie lokale RAM1-Reserve: +140128
  - EXTRAM variables: +140128

Open-Points-Slice 1 (2026-04-28): SG-Hauptscreen + Overlay-Basis

- Erster additiver Umsetzungsschnitt fuer MB-02 umgesetzt:
  - `src/teensy/LvglUi.h`: SG-Mainlabel- und Overlay-Felder ergaenzt.
  - `src/teensy/LvglUi.cpp`: SG-spezifischer Hauptscreen (Header/Targets/Dose)
    sowie Overlay-Kanal mit Zustandsmeldungen eingefuehrt.
- Overlay-Basislogik im SG-Hauptzustand:
  - Fehler/Fault
  - WaitForFilter-Hinweis
  - Completed-/Aborted-Status
  - Remote-Stale/Lost-Hinweis
  - Ready-/Dirty-Hinweis
- Sichtbarkeitsregel:
  - SG-Hauptzustand zeigt den neuen Mainscreen + Overlay.
  - Nicht-SG-Zustaende behalten den bisherigen Debug-/Statusblock.

Build-Validierung nach Open-Points-Slice-1 (`pio run -e teensy41`):

- Ergebnis: SUCCESS
- teensy_size:
  - FLASH: code 206408, data 64768, headers 8372
  - RAM1: variables 72768, code 200408, padding 28968, free local variables 222144
  - RAM2: variables 146816, free malloc/new 377472
  - EXTRAM: variables 146560

Open-Points-Slice 2 (2026-04-28): InputRouter-Policy-Baseline

- Additiver Basis-Schnitt fuer MB-02 InputRouter umgesetzt:
  - `src/teensy/InputRouterPolicy.h`: Fokus-/Modalzustand und Policy-API.
  - `src/teensy/InputRouterPolicy.cpp`: Baseline-Regeln fuer Touch-Fokus,
    Timeout-Rueckfall und modalen Rotationsguard.
  - `src/teensy/main.cpp`: zentrale Dispatch-Gate-Integration in
    `dispatchNormalizedInputEvent(...)` sowie Touch-/Runtime-Beobachtung im Loop.
- Baseline-Regeln im Ergebnis:
  - Touch-Aktivitaet setzt Fokus auf `Touch`.
  - Nach 900ms ohne Touch-Rueckmeldung Rueckfall auf `Encoder`.
  - In `WaitForFilter` und Fault-Zustaenden werden Rotations-Events blockiert.
- Restluecke fuer folgende Slices:
  - Touch-Zielmapping, Dialogfokus und globale EventGuard-Policy sind noch nicht vollstaendig.

Build-Validierung nach Open-Points-Slice-2 (`pio run -e teensy41`):

- Ergebnis: SUCCESS
- teensy_size:
  - FLASH: code 206856, data 64768, headers 8948
  - RAM1: variables 72800, code 200856, padding 28520, free local variables 222112
  - RAM2: variables 146816, free malloc/new 377472
  - EXTRAM: variables 146560

Weitere sinnvolle Optimierungen (naechste Slices):

- weitere grosse, nicht-DMA-kritische Caches und Tabellen priorisiert in `EXTMEM` verschieben
- zeitkritische ISR-/Hotpath-Daten weiter in internem RAM belassen
- pro Slice mit `teensy_size` gegenpruefen, ob RAM1-/RAM2-Druck sinkt

## Delta-Update 2026-04-29 (EV-logarithmische Skalierung SG-Targets)

### Historischer Abgleich

v0.3 Mode_SG.cpp verwendet durchgaengig multiplikative EV-Skalierung:

- `target *= pow(2.0, evt.value * evStep)` fuer Zeit- und Dosismodus gleich.
- Default-Schrittweite im Caller: `1/3 Stop` (STEP_THIRD).
- Gilt fuer Soft-Kanal (EVT_ENC_SOFT), Hard-Kanal (EVT_ENC_HARD) und
  Master-Brightness-Shift (EVT_ENC_GRADE, proportional auf beide Kanaele).

Part2 hatte bis zu diesem Stand lineare Additivschritte:

- `softTarget_ += direction * (dose ? 0.1f : 0.5f)` -- nicht EV-konform.
- Verhaltensbruch gegenueber v0.3: Schrittgroesse waechst nicht proportional
  mit dem Targetwert; grosse Targets werden ueberdimensional langsam skaliert.

### Umsetzung (2026-04-29)

EV-logarithmische Skalierung in `SplitgradeWorkflow` eingefuehrt:

- `src/teensy/SplitgradeWorkflow.h`:
  - `static constexpr float kEvStep = 1.0f / 3.0f` ergaenzt (1/3 Stop, Default wie v0.3).
- `src/teensy/SplitgradeWorkflow.cpp`:
  - `#include <cmath>` ergaenzt.
  - `adjustPrimaryValue()` SplitTargets-Zweig:
    `softTarget_ = clampTargetValue(softTarget_ * powf(2.0f, direction * kEvStep))`
  - `adjustSecondaryValue()` SplitTargets-Zweig:
    `hardTarget_ = clampTargetValue(hardTarget_ * powf(2.0f, direction * kEvStep))`
  - Untergrenze bleibt `0.1f` (via `clampTargetValue`).
  - Gilt einheitlich fuer Zeit- und Dosismodus (Modusdifferenzierung nicht noetig).

Hinweis: Grade-Panel-Anpassungen (adjustGrade) bleiben linear (fotografische
Gradationsskala 0-5 ist kein EV-Wert).

### Historischer Bezug

- Evidenz v0.3: `Dukatimer v0.3/src/Mode_SG.cpp:101` (EVT_ENC_SOFT),
  `Mode_SG.cpp:115` (EVT_ENC_HARD), `Mode_SG.cpp:131` (EVT_ENC_GRADE).
- Evidenz v0.3 Schrittweite: `Dukatimer v0.3/src/Logic_Timer.cpp:175-181`
  (STEP_THIRD = 1/3 als Default).

### Build-Validierung (pio run -e teensy41)

- Ergebnis: SUCCESS
- teensy_size:
  - FLASH: code 207848, data 65792, headers 8980
  - RAM1: variables 73824, code 201848, padding 27528, free local variables 221088
  - RAM2: variables 146816, free malloc/new 377472
  - EXTRAM: variables 146560

### Soll-Ist-Korrektur PF-M04 / PF-S01

- PF-M04 Splitgrade-Belichtung: Schrittlogik jetzt historisch konform.
  EV-Verhalten ist ein wesentliches Qualitaetsmerkmal fuer fotografische
  Reproduzierbarkeit; dieser Punkt war bis dato eine stille Divergenz.
- PF-S01 Historische Bedienparitaet: Lernkurvenbruch durch nicht-proportionale
  Schrittgroessen beseitigt. Restrisiken (quadraturvalidierter Encoder, Fokusmodell,
  blinde Overlay-Fuehrung) unveraendert offen.

## Delta-Update 2026-04-29 (Querschnittsvorgabe EV/F-Stop und Messwertdarstellung)

### Architekturentscheidung

Die Doku fuehrt F-Stop fuer Part2 ab diesem Stand nicht mehr als isolierten BW-Sondermodus, sondern als zentrale EV-/F-Stop-Logik fuer alle belichtungsrelevanten Modi.

Verbindliche Folgerung:

- dieselbe logarithmische Schrittlogik soll fuer BW, SG, Burn, Teststrip und Preflash gelten
- Modi duplizieren diese Mathematik nicht, sondern konsumieren eine gemeinsame Zentralschicht
- modusspezifisch bleiben nur UI-Fokus, Defaultwerte und fachliche Randbedingungen

### Messwertdarstellung

Messwerte aus echten Messvorgaengen werden ab diesem Doku-Stand zusaetzlich in EV gefordert, sofern die fotografische Interpretation sinnvoll ist.

Das betrifft insbesondere:

- Densitometrie
- Papierkalibrierung
- Spot-, Zonen- und Vergleichsmessungen

Nicht darunter fallen reine Laufzeit- und Hardwaretelemetrie wie:

- Head-Lux waehrend der Belichtung
- Temperatur
- Buslatenz
- Heartbeat- oder Link-Status

Diese Werte bleiben primaer in Lux, C, ms oder Statuscodes.

### Landminen-Korrektur

Die bisherige Einzelbehandlung von F-Stop als spaeterem BW-Feature waere eine neue Architektur-Landmine geworden. Sie haette dieselbe fotografische Bedienlogik in mehrere Modi zerschnitten und spaetere Inkonsistenzen in Schrittgroesse, Messwertdarstellung und UI-Semantik erzeugt.

Ab diesem Stand gilt deshalb zusaetzlich:

- mehrfach genutzte Logik wird einmal zentral definiert
- EV-/Lux-Umrechnung und Formatter gehoeren in eine gemeinsame Schicht
- `ExposureEngine` bleibt intern lux- und dosisbasiert; EV ist eine fachliche Bedien- und Darstellungsabstraktion darueber
