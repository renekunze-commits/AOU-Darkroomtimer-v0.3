# CHANGELOG

## Unreleased — 2026-05-08

- Opt: RAM1-Audit Schritte 1–4 (Teensy 4.1, RAM1 frei: 62 KB → 98 KB)
  - Schritt 1 — FLASHMEM auf `setup()`, `setupInputs()`, `setupStorage()`,
    `setupSystemSettingsPersistence()`, `setupPaperSlotPersistence()` und
    `executePrintExposureStart()` in `main.cpp`. Einsparung: ~9 472 B ITCM.
  - Schritt 2 — 19 diagnostische `UiPresenter`-Puffer (`subtitleBuffer_`,
    `modeInfoBuffer_`, `exposureInfoBuffer_`, `sensorInfoBuffer_`, u. a.) als
    `static EXTMEM` ins externe PSRAM verlagert (statt DTCM). Einsparung: ~3 168 B DTCM.
  - Schritt 3 — `FLASHMEM` auf `LvglUi::begin()`: kein messbarer RAM-Effekt,
    trotzdem ITCM-schonend da begin() nur einmalig aufgerufen wird.
  - Schritt 4 — Setup-Screen Lazy-Loading: `pushWidgetsFromSnapshot()` ueberspringt
    den gesamten LVGL-Widget-Update-Block fuer `SCREEN_ID_PAGE_SETUP`, wenn sich
    weder `SetupModeRuntimeState` noch `thermalDeratingActive` noch `runtimeOutputLimit`
    gegenueber dem letzten Frame veraendert haben. Cache-Member `setupStateCache_`,
    `setupThermalCache_`, `setupOutputCache_` in `LvglUi.h` hinzugefuegt (17 B DTCM).
    RAM1 nach Schritt 4: 98 656 B frei. `teensy41` Build validiert (SUCCESS).

- Docs: EEZ-Studio-Projekt um Busy-Screen ergaenzt
  - `DukatimerPart2TeensyUi.eez-project` enthielt nur 6 Screens; `SCREEN_ID_BUSY`
    (Index 7) war zwar vollstaendig in `screens.c`/`screens.h` implementiert,
    fehlte aber im EEZ-Projekt-JSON.
  - Busy-Screen-Definition hinzugefuegt (objID-Namespace `b7000000–b7000008`)
    mit pixelgenauem Layout aus `create_screen_busy()`: Header-Streifen (y=0 h=48),
    Zeit-Label (y=58 Montserrat 28), Fortschrittsbalken (y=122 400×18),
    Prozent-Label (y=146), SG-Detail-Label (y=178 HIDDEN), run_overlay und
    pause_overlay (je y=228 h=64 HIDDEN) mit PAUSE-/FORTSETZEN-/ABBRECHEN-Buttons.
  - JSON validiert; Projekt-Datei: 6 307 → 7 196 Zeilen.
  - Teensy-Firmware auf `0.2.47-dev` angehoben.

## Unreleased — 2026-05-07

- Fix: RAM1-Overflow durch LVGL-Schriftart-Bitmaps im DTCM (Teensy 4.1)
  - GCC LTO hat `static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[]`
    in LVGL-Schriftartdateien (Montserrat 12–28) aus `.rodata` in `.data` (DTCM)
    verschoben. Der Teensy-4.1-Linker mappt `.rodata` DTCM-seitig (`> DTCM AT> FLASH`),
    daher hatte `__attribute__((section(".rodata")))` keinen Effekt.
  - Loesung: `LV_ATTRIBUTE_LARGE_CONST __attribute__((section(".progmem")))` in
    `include/lv_conf.h` hinzugefuegt. Der Linker legt `.text.progmem > FLASH` als
    reines Flash-Segment an (kein AT>-Copy in RAM), sodass die Bitmaps dort verbleiben
    und ueber DCACHE gelesen werden.
  - Einsparung: ~76 KB DTCM. `variables:177856 → 101056`, `free:-14016 → +62784`.
  - `teensy41` Build validiert (SUCCESS, 62784 Bytes RAM1 frei).

- Feature: BW-Modus LVGL-UI-Integration (AP-BW-05 / SCREEN_ID_PAGE_SPLITGRADE)
  - Kein eigener BW-Screen; BW nutzt `SCREEN_ID_PAGE_SPLITGRADE` (ModeId-Routing
    war bereits vorhanden). Neuer Code kapselt BW-spezifische Textformatie‌rung.
  - `UiPresenter`: drei neue oeffentliche Methoden `getBwHeader()`, `getBwTargets()`,
    `getBwExposureMain()` hinzugefuegt. Shared private Puffer (`sgHeaderBuffer_`,
    `sgTargetsBuffer_`, `sgExposureMainBuffer_`) werden wiederverwendet – BW und SG
    sind mutually exclusive, keine Kollision.
  - `getBwHeader()`: zeigt BW-Panel, ExecState, Slot-Info oder
    "SCHWARZWEISS  KEIN PAPIERPROFIL" wenn kein Profil geladen.
  - `getBwTargets()`: im Measurement-Panel Lux/Ref, sonst Grade/Mix/ZIEL-Formatierung.
  - `getBwExposureMain()`: im Measurement-Panel Lux/Ref/dEV, sonst
    `MeasurementValueFormatter::formatSplitgradeDoseTelemetry`.
  - `LvglUi.cpp`: BW-Block im `SCREEN_ID_PAGE_SPLITGRADE`-Zweig nutzt nun
    `presenter_.getBwHeader(snapshot)` fuer `sg_exec_state_lbl` statt
    `presenter_.getSgHeader(snapshot)`.
  - `main.cpp` `buildRemoteDisplayPayload()`: eigenstaendiger BW-Zweig mit
    `getBwHeader/Targets/ExposureMain`; SG/Paper/Setup-Zweig bleibt unveraendert.
  - Nicht-Hot-Path-Presenter-Funktionen (getSubtitle, getMeasurement*, getModeInfo,
    getExposureInfo, getSensorInfo, getRemoteCommandInfo, getGatewayInfo,
    getEncoderInfo, getSwitchInfo, getTouchInfo, getDmaInfo) tragen
    `FLASHMEM` fuer LTO-bewusstes Flash-Placement.
  - Teensy-Firmware auf `0.2.46-dev` angehoben; `teensy41` Build validiert.

- Fix: `isMeasurementContextArmed` erweitert auf BW-Modus (AP-BW-03)
  - `isMeasurementContextArmed()` in `main.cpp` akzeptierte Wireless-Messwerte bisher nur
    wenn `ModeId::Splitgrade` und `SplitgradePanel::Measurement` aktiv war.
  - Erweiterung: bei `ModeId::BlackWhite` gilt `BwPanel::Measurement` als gleichwertiger
    Arming-Kontext. Ohne diese Aenderung konnten Handteil-Messwerte im BW-Modus nicht
    eingeliefert werden, obwohl Workflow und Execution-Pfad vollstaendig vorhanden sind.
  - Alle uebrigen AP-BW-01 bis AP-BW-04 Bestandteile waren bereits implementiert:
    `ModeRuntimeState` Schema 9, `BwModeRuntimeState`/`BwPanel`/`BwExecutionState`,
    `BlackWhiteWorkflow`, `executePrintExposureStart`, `broadcastExposureStateToWorkflows`,
    `processBlackWhiteExecutionCommands` im `dispatchNormalizedInputEvent`-Pfad,
    `updateRemoteExecutionFeedback` mit dynamischer BW/SG-Verzweigung.
  - Alle uebrigen E17/Loesung-B Bestandteile waren bereits implementiert:
    `PaperExposureProfile.kBw`-Kommentar, `SplitgradeWorkflow.initializePaperDrivenState`
    nutzt `kDefaultBaseTarget`, `HeadSpectrumMapper` mit Identity-Fallback und kalibriertem
    Pfad, `resolveExposureSpectrum()` in `main.cpp`.
  - Teensy-Firmware auf `0.2.45-dev` angehoben; `teensy41` Build validiert (SUCCESS).

## Unreleased — 2026-05-05

- Feature: EEZ-UI-Integration des Paper-Workspace-Screens
  - `page_paper_workspace` war ein leerer Platzhalter; der Screen wurde jetzt vollstaendig hand-crafted nach dem Split&grade-Vorbild aufgebaut.
  - Aufbau: Panel-Info-Streifen (Panel-Typ, Slot-Name, Kalibrierzustand), 7-Zeilen-Gleitfenster fuer CAL- und SELECT-Panel, Encoder-Hint-Bar am unteren Rand.
  - Das Gleitfenster zentriert sich dynamisch auf das aktuell selektierte Item (CAL) bzw. den selektierten Slot (SELECT) und hebt die aktive Zeile farbig hervor (roter Hintergrund fuer Selektion, gruener fuer Bearbeitungsmodus).
  - `WEISSPUNKT (N)` und `SCHWARZPUNKT (M)` sind als Zeilen im CAL-Gitter sichtbar; die Wertspalte zeigt `N=<Stufe>` bzw. `M=<Stufe>`.
  - Die 12 weiteren CAL-Items (GRADE MODE..DISCARD) werden mit ihren Snap&shot-Werten inline dargestellt, ohne neue EEZ-Flow-Variablen zu benoetigen.
  - Im SELECT-Panel zeigt jede Zeile Slot-Nummer, `[AKT]`-Marker fuer den aktiven Slot sowie Kurzinfo (CAL/MG + Slot-Name) in der Wertspalte.
  - Widget-Handles (`paper_panel_lbl`, `paper_cal_row_cont[7]`, `paper_cal_row_lbl[7]`, `paper_cal_row_val[7]`, `paper_hint_lbl`) sind in `DukaWidgets` eingetragen und werden in `LvglUi::pushWidgetsFromSnapshot()` per `if (currentScreenId_ == SCREEN_ID_PAGE_PAPER_WORKSPACE)` befuellt.
  - `teensy41` Build validiert (SUCCESS, keine neuen Warnungen).

- Feature: Methode 1 (Visuelle Schwellenwert-Methode) im CAL-Panel
  - Zwei neue virtuelle `PaperCalibrationItem`-Eintraege: `StepWhite` (Weisspunkt N) und `StepBlack` (Schwarzpunkt M), beide per Encoder einstellbar in [1..21] mit der Invariante N > M.
  - `recalculateFromSteps()` berechnet bei jeder Eingabeeaenderung sofort ISO-R, ISO-P und k_Bw aus dem Graukeil-Densitaets-Modell (`D(s) = 0.05 + (s-1)*0.15`) und schreibt die Werte direkt in `stagedProfile_`.
  - Stufenwerte (`stepWhite`, `stepBlack`) werden im `PaperModeRuntimeState`-Snapshot mitgefuehrt und sind ueber `UiPresenter` direkt sichtbar.
  - `paperCalibrationItemLabel()` zeigt "WEISSPUNKT (N)" / "SCHWARZPUNKT (M)"; `formatPaperCalibrationValue()` zeigt "Stufe %u" fuer den jeweiligen Stufenwert.
  - Die Eintraege werden beim Eintritt in den CAL-Editor auf sichere Defaults (N=15, M=8) zurueckgesetzt; kein Schema-Break, keine Aenderung an `PaperExposureProfile` oder Persistenz.
  - `kModeRuntimeStateSchemaVersion` auf 8 angehoben; `PaperCalibrationItem::Count` waechst auf 13.
  - Teensy-Firmware auf `0.2.44-dev` angehoben; `teensy41` Build validiert.

## 2026-05-04

- Safety-Fix: Belichtungsstart waehrend aktiver VFS-/HTTP-Dateitransaktion verriegelt
  - Der ESP32-S3 publiziert aktive Datei- oder HTTP-Transaktionen jetzt sofort ueber ein dediziertes Heartbeat-Runtime-Bit an den Teensy, statt nur die serielle VFS-Session lokal zu kennen.
  - `main.cpp` blockiert neue Splitgrade-Starts zentral vor `ExposureEngine::start*()`, solange entweder eine laufende serielle VFS-Session am Teensy oder eine noch aktive HTTP-/Download-/Upload-Transaktion auf dem ESP-Servicepfad sichtbar ist.
  - Der blockierte Start faellt sichtbar in `ExposureFaultReason::StartBlocked`, damit UI und Diagnose denselben technischen Stopgrund sehen.
  - Teensy-Firmware auf `0.2.43-dev` angehoben; `teensy41` und `esp32s3_n16r8` muessen nach dem Slice validiert werden.

- Service-Fix: Teensy-TX-Vertrag auf echte Prioritaet und bounded Flush festgezogen
  - `EspServiceLink` sortiert ausgehende Frames jetzt auch in der Queue selbst nach Prioritaet, statt Prioritaet nur fuer Eviction/Drops zu kennen. Damit gilt unter Backpressure jetzt tatsaechlich `TeensyCommand/VFS > Heartbeat > RemoteRender > Diagnostic`.
  - `flushPendingFrames()` arbeitet nur noch mit kleinem Budget pro Aufruf, damit Producer-Pfade keinen bereits aufgelaufenen UART-Stau im fremden Call-Kontext komplett abtragen.
  - Die Richtungsentscheidung fuer lokale I2C-Entklemmung ist jetzt dokumentiert: aktuelle Hardware hat keinen dokumentierten schaltbaren Kopfversorgungszweig; die vorgesehene Recovery-Kombination ist eine dedizierte, Teensy-gesteuerte High-Side-Schaltung in `+3V3_HEAD` plus `SensorManager`-eigener Power-Cycle/Reinit-Logik. Bis zu dieser Hardwareaenderung bleibt der Watchdog-Reset die letzte Rueckfallebene.
  - Teensy-Firmware auf `0.2.42-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Startup-Fix: PSRAM-Fail-Safe direkt an den Anfang des Teensy-Startpfads gezogen
  - `setup()` prueft `external_psram_size` jetzt direkt nach dem Bootlog und bricht bei fehlendem externem RAM fail-closed ab, bevor weitere Subsysteme von LVGL-/EXTMEM-Nutzung ausgehen.
  - `LvglUi::begin()` behaelt denselben Check als zweite Sicherung, damit der UI-Pfad auch bei spaeterer Startreihenfolge nicht still mit fehlendem PSRAM weiterlaeuft.
  - Die serielle Diagnose meldet den Fehler jetzt eindeutig als fruehen Startup-Abbruch statt erst als spaetes UI-Init-Problem.
  - Teensy-Firmware auf `0.2.41-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Audit-Fix: Snapshot- und EEZ-UI-Vertrag geschaerft
  - Der Runtime-Fallback fuer `global_overlayColor` nutzt jetzt dieselben warmen Zustandsfarben wie die verbindliche EEZ-Palette und entfernt die bisherigen blau/gruen driftenden Magic-Hexwerte aus dem Header-Meldungsband.
  - Die `paper_selectedSlot*`-Flow-Variablen werden aus `paperSlotSummaries[selectedSlot]` befuellt, statt beim Browsen weiter still die aktiven Slot-Daten zu spiegeln.
  - Der Snapshot-Bau kappt `paperSlotCount` und `paperActiveSlot` defensiv an der UI-Grenze; `setup_maxHeadBrightnessPercent` und die Setup-Textvariablen folgen jetzt dem EEZ-Typ-/Presenter-Vertrag.
  - Teensy-Firmware auf `0.2.40-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Runtime-Fix: Start-Taste als Busy-Pause/Resume-Parität verdrahtet
  - Der lokale Start-Taster darf im globalen Wait-Modal jetzt bewusst durch den Guard und bleibt damit die blinde Paritaetsroute zum Busy-Screen.
  - Im SG-Workflow pausiert `Start` waehrend `ExposurePhase::Exposing` und setzt waehrend `ExposurePhase::Paused` dieselbe Belichtung wieder fort.
  - Der Abbruch bleibt auf der bestehenden Undo-/Zurueck-Schiene des Encoder-Buttons; Touch-`PAUSE` und lokale Tasterbedienung teilen damit denselben Engine-Command-Rand.
  - Teensy-Firmware auf `0.2.39-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- UI-Fix: sichtbaren Pause-Button im Busy-Overlay ergänzt
  - Der Busy-Screen blendet waehrend `ExposurePhase::Exposing` jetzt ein eigenes Run-Overlay mit einem zentralen `PAUSE`-Button ein.
  - Der neue Button nutzt denselben globalen Modal-Vertrag wie Resume/Abort und meldet direkt `UiModalAction::Pause`, statt einen weiteren UI-Sonderpfad zu oeffnen.
  - In `ExposurePhase::Paused` bleibt das bestehende Overlay mit `FORTSETZEN` und `ABBRECHEN` aktiv; beide Overlay-Zustaende schliessen sich auf derselben Busy-Zeile gegenseitig aus.
  - Teensy-Firmware auf `0.2.38-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Architektur-Fix: globalen Modal-Button-Vertrag verallgemeinert
  - `LvglUi` meldet Modal-Buttons nicht mehr als Busy-spezifische Quelle, sondern als generische `UiModalAction` ueber `lvgl_ui_modal_action_callback()` und `pollModalAction()`.
  - `main.cpp` speist daraus genau einen lokalen Rohkanal `LocalModalButton`; `Pause`, `Resume` und der bestehende Undo-/Abort-Pfad werden erst im Normalizer semantisch aufgeloest.
  - Der globale Wait-Guard laesst damit nur noch explizite Laufzeitaktionen (`Undo`, `Pause`, `Resume`) passieren, statt einen Busy-Einzelpfad im Router zu kennen.
  - Der SG-Workflow uebersetzt `Pause` und `Resume` jetzt als eigene `PauseExposure`-/`ResumeExposure`-Commands an der Engine-Grenze. Kuenftige BW- oder Burn-Modal-Buttons koennen denselben Vertrag ohne neue Sonderquellen nutzen.
  - Teensy-Firmware auf `0.2.37-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Review-Fix: Busy-Touch ueber zentrale Input-Route und Measurement-Leerlauf klarer
  - Die Busy-Buttons `FORTSETZEN` und `ABBRECHEN` laufen nicht mehr als Engine-Direktpfad aus `main.cpp`, sondern als synthetische lokale InputEvents ueber Normalizer, InputRouter, ModeCoordinator und Splitgrade-Workflow.
  - Der globale Wait-Guard erlaubt dabei weiterhin nur den Sicherheitsausstieg `Undo` und explizit den lokalen Busy-Resume-Touchpfad; Resume wird im SG-Workflow als `ResumeExposure`-Command in die Engine uebersetzt.
  - Die Measurement-Statuszeile zeigt ohne sichtbare Session-History jetzt `Z--` statt eines impliziten `Z0`-Leerlaufs.
  - Teensy-Firmware auf `0.2.36-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Review-Fix: LVGL-PSRAM-Startpfad gehaertet
  - `LvglUi::begin()` prueft vor `lv_init()` jetzt explizit `external_psram_size` und lehnt den UI-Start ohne erkanntes externes PSRAM fail-closed ab.
  - `setup()` behandelt ein fehlgeschlagenes `ui.begin()` jetzt als fatalen Startup-Fehler und bleibt bewusst vor dem normalen Loop stehen, statt mit moeglichem UI-/Belichtungspfad weiterzulaufen.
  - Teensy-Firmware auf `0.2.35-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- AP-11g Measurement-UI-Invaliddarstellung begonnen
  - `MeasurementValueFormatter::formatMeasurementRange()` meldet eine fehlende technische Session-Range jetzt explizit als `NO RNG`, statt im Leerlauf nur `LO/HI/SPAN`-Platzhalter mit Default-Semantik zu zeigen.
  - Der AP-11-Hostharness deckt diesen Formatter-Fall jetzt explizit ab; auf dieser Arbeitsstation lief mangels Host-Compiler nur der Syntaxpfad ueber den Teensy-ARM-Compiler.
  - Teensy-Firmware auf `0.2.34-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- Schritt 13 (LVGL-PSRAM und Busy-Screen): LVGL nutzt jetzt einen Custom-Allocator ueber `lvgl_psram_alloc.h`, so dass Widget-Objekte, Styles und LVGL-interne Heap-Allokationen im Teensy-PSRAM statt im knappen RAM1 liegen.
  - Die LVGL-Draw-Buffer wurden von 40 auf 80 Zeilen verdoppelt; beide Puffer bleiben bewusst in `DMAMEM`/RAM2, weil sie Display-/DMA-nah sind.
  - `SCREEN_ID_BUSY` ist als belichtungsaktive Sicherheitsdarstellung fuer `PreWait`, `Exposing`, `Paused` und `PostWait` eingefuehrt. Der Screen besitzt eigene Header-Handles (`busy_hdr_*`) und ueberschreibt die shared `hdr_*`-Pointer der normalen Seiten nicht.
  - Der Busy-Screen zeigt Phasenlabel, Header-Meldungsband, THERM-Chip, Zeit- oder Dosisfortschritt, Fortschrittsbalken, Prozentwert und SG-Soft/Hard-Detail. In `Paused` erscheint ein Touch-Overlay mit `FORTSETZEN` und `ABBRECHEN`.
  - Der Busy-Fortschritt wird bei `PreWait` auf 0 gesetzt, waehrend `Exposing` aktualisiert, in `Paused` gehalten und bei `PostWait` auf 100 gesetzt; die UI verwechselt damit Pausendauer nicht mit echter Belichtungszeit.
  - Die Busy-Touch-Aktionen laufen ueber `lvgl_ui_busy_action_callback()` zu `LvglUi::pollBusyAction()` und werden in `main.cpp` bewusst erst in `ExposureEngine::resume()` oder `ExposureEngine::stopGracefully()` uebersetzt.
  - `screens.c` bleibt fuer neue UI-Funktionen konsequent `FLASHMEM`, damit der aktuelle RAM1-Spielraum nicht durch ITCM-Code aufgefressen wird.
  - Teensy-Firmware auf `0.2.33-dev` angehoben; `teensy41` Build validiert. Speicherstand nach Build: RAM1 free for local variables `23552`, RAM2 variables `166016`, EXTRAM variables `22144`.

- Schritt 12 (UI Widget-Bäume): Boot-Screen und PageSplitgrade erhalten echte LVGL-Widget-Bäume in `screens.c`; alle Widget-Handles werden über `DukaWidgets g_duka_widgets` (C-Struct, `screens.h`/`screens.c`) an `LvglUi::pushWidgetsFromSnapshot()` übergeben.
  - Header-Band (y=0 h=48): Titel links, Message-Container mitte mit dynamischer BG-Farbe, THERM-Chip rechts (hidden bis Derating).
  - ModeTab-Leiste (y=284 h=36): 4 Tabs PAPER/MEAS/PRINT/SETUP mit aktiver/inaktiver Farbgebung.
  - Boot-Screen: Titel Montserrat 28, Subtitle Montserrat 16, Status Montserrat 14 wrap.
  - PageSplitgrade: Grade/Time-Labels Montserrat 28, Soft/Hard Montserrat 20, ctrl-mode-Chip, exec-state, paper-slot, dose/lux/head, context-bar.
  - `pushWidgetsFromSnapshot()` (FLASHMEM) treibt alle Label- und Farb-Updates direkt aus `SystemSnapshot`.
  - Alle `create_screen_*`-, `tick_screen_*`- und Helper-Funktionen in `screens.c` sind mit `FLASHMEM` annotiert; `LV_FONT_MONTSERRAT_36` bleibt deaktiviert (wurde durch 28 ersetzt), um ITCM-Overflow zu vermeiden.
  - RAM1 nach Build: free for local variables 23552 (–1120 vs. Schritt 11, unkritisch).

- PaperSlot-Recovery-Telemetrie bis in den Teensy-Statuspfad begonnen
  - `TeensyLinkService` publiziert im periodischen `EspServiceSnapshot` jetzt eine kompakte Recovery-Zusammenfassung fuer `paperslots.bin` und `.bak`; `HttpVfsBridge` aktualisiert diese nicht-destruktiv nur ausserhalb aktiver Transfers und nicht waehrend Belichtung.
  - `EspServiceLink`, `EspLinkRuntimeStatus` und der Placeholder-`UiPresenter` machen daraus erstmals einen sichtbaren lokalen Hinweis: die Gateway-Diagnose traegt jetzt `REC ...`, und im PAPER-Modus meldet das Header-Band bei stabiler Entscheidung entweder ein verfuegbares Backup-Restore oder den Bedarf eines Host-Recover-Uploads.
  - Der HTTP-Produktvertrag steigt dafuer auf `contractVersion = 4`; die ESP-Service-Firmware steht jetzt auf `0.1.14-dev`, die Teensy-Firmware auf `0.2.32-dev`.

- PaperSlot-Recovery-Statusvertrag fuer explizite Restore-Angebote begonnen
  - `HttpVfsBridge` bietet jetzt `GET /api/v1/product/paperslots/recovery-state`, das aktiven Blob und Backup ueber denselben RAM-first-Readpfad inspiziert, ohne dabei Export-/Health-Snapshots umzudefinieren oder einen Schreibpfad anzustossen.
  - Die Antwort beschreibt beide Pfade mit Fetch-/VFS-/Parse-Zustand und berechnet daraus `decisionStable`, `canRestoreBackup`, `restoreWouldChangeActive`, `manualRecoverUploadSuggested` und `recommendedAction`, damit eine spaetere lokale UI oder ein Host-Tool Recovery explizit anbieten kann, statt nur rohe Diagnose einzusammeln.
  - Der Produktvertrag `/api/v1/product/paperslots` steigt dafuer auf `contractVersion = 3`; die ESP-Service-Firmware steht jetzt auf `0.1.13-dev`.

- PaperSlot-Blob-Inspektion und Recovery-Vorvalidierung begonnen
  - `HttpVfsBridge` bietet jetzt `GET /api/v1/product/paperslots/inspect` und
    `/api/v1/product/paperslots/inspect/backup`, die den lokal gestagten Blob
    gegen Header, Formatversion, Groesse, CRC und Bank-Schema pruefen und dabei
    die wichtigsten aktiven Slot-Metadaten als JSON sichtbar machen.
  - `POST /api/v1/product/paperslots/recover` und
    `POST /api/v1/product/paperslots/restore-backup` committen `paperslots.bin`
    nur noch nach erfolgreicher Blob-Pruefung; ungueltige Dateien werden mit
    HTTP `422` und einem benannten Parse-Fehler abgewiesen, ohne die aktive
    Produktdatei zu ueberschreiben.
  - Der Produktvertrag `/api/v1/product/paperslots` steigt dafuer auf
    `contractVersion = 2`; die ESP-Service-Firmware steht jetzt auf
    `0.1.12-dev`.

- Interner PaperSlot-Restore `backup -> active` ohne Host-Upload begonnen
  - `HttpVfsBridge` bietet jetzt `POST /api/v1/product/paperslots/restore-backup`, das `paperslots.bin.bak`
    zuerst lokal im ESP-RAM/PSRAM staged und danach ueber den bestehenden
    VFS-Schreibpfad wieder nach `paperslots.bin` commitet.
  - Der bestehende Export-/Recover-Vertrag bleibt damit hostfreundlich, bekommt
    aber zusaetzlich einen lokalen Recovery-Schritt fuer den Fall, dass eine
    bekannte Backup-Datei schon auf der SD liegt und kein Host-Binaer mehr
    hochgeladen werden soll.
  - Erfolgreiche Export-, Backup-Export- und Recover-Snapshots markieren jetzt
    ihren Status im Health-Diagnosemodell korrekt als `ok`, statt den
    Initial-Defaultzustand fortzuschreiben; die ESP-Service-Firmware steht
    dafuer auf `0.1.11-dev`.

- PaperSlot-Export-/Recovery-Vertrag ueber HTTP/VFS begonnen
  - `TeensyLinkService` und `EspServiceLink` bedienen das bereits reservierte
    `FileReadRequest` jetzt als strikten Slice-Read, so dass aktive
    `paperslots.bin`- und `.bak`-Daten kontrolliert zum ESP gelesen werden
    koennen, ohne einen zweiten langlaufenden VFS-Streampfad einzufuehren.
  - `HttpVfsBridge` bietet dafuer die ersten expliziten Produktdatenrouten
    `/api/v1/product/paperslots`, `/api/v1/product/paperslots/export`,
    `/api/v1/product/paperslots/export/backup` und
    `/api/v1/product/paperslots/recover`; Export und Recovery laufen jeweils
    ueber denselben RAM-first-Staginggedanken und schreiben ihre letzte
    Download-/Upload-Diagnose sichtbar in den Health-Endpoint.
  - `include/ProductDataStoragePaths.h` zieht den ersten gemeinsamen
    Produktdaten-Pfadvertrag fuer `paperslots.bin` und `.bak`; die ESP-Service-
    Firmware steht dafuer auf `0.1.10-dev`, die Teensy-Firmware auf
    `0.2.31-dev`.

- HTTP-VFS-Stagingdiagnose sichtbar gemacht
  - `HttpVfsBridge` exportiert im Upload-Response und im Health-Endpoint jetzt,
    wie viele Bytes lokal gestagt wurden und ob dafuer Heap oder PSRAM benutzt
    wurde.
  - Der Health-Endpoint behaelt ausserdem die letzte Upload-Zusammenfassung mit
    Status, Pfad, Staginggroesse und geschriebenen Bytes, damit der neue
    RAM-first-Pfad auch nach dem Request nachvollziehbar bleibt.
  - Die ESP-Service-Firmware steht dafuer auf `0.1.9-dev`.

- HTTP-/VFS-Uploadpfad auf RAM-first-Staging umgestellt
  - `src/esp32/HttpVfsBridge.*` puffert den kompletten RAW-HTTP-Body jetzt erst
    in lokalem RAM bzw. bevorzugt PSRAM und startet den eigentlichen
    Teensy-VFS-Transfer erst nach `RAW_END`.
  - Abgebrochene oder unvollstaendige HTTP-Requests erzeugen damit keinen
    angefangenen VFS-Schreibpfad mehr auf dem Teensy; der bestehende
    Temp-/Backup-Renamepfad greift erst fuer vollstaendig gestagte Uploads.
  - Die ESP-Service-Firmware steht dafuer auf `0.1.8-dev`.

- RAM-/PSRAM-Strategie fuer Part2 als Hardwareleitlinie dokumentiert
  - `docs/hardware/ram-und-psram-strategie.md` fixiert die verifizierte
    Speicherbasis von Teensy 4.1 und ESP32-S3 sowie die Projektregel, dass
    vorhandener RAM bewusst fuer Staging, Puffer, Historie und
    Schreibentlastung genutzt werden soll.
  - Die Hardwareuebersicht und die offenen Entscheidungen verankern diese
    Leitlinie jetzt explizit, damit spaetere Slices Speicher nicht blind klein
    rechnen, waehrend externer RAM ungenutzt bleibt.
  - Die Doku nennt konkrete Stellen, an denen RAM/PSRAM Schreibzugriffe auf SD
    oder Flash reduzieren kann, insbesondere bei Paper-/Settings-Commits,
    Diagnosehistorien und kuenftigen HTTP-/Backup-Pfaden.
  - Kein Firmware-Version-Bump: Es wurde der Architektur- und
    Dokumentationsvertrag geschaerft, nicht die Runtime-Logik geaendert.

- Verifizierter Teensy-PSRAM-Hardwarevertrag auf `16 MB` korrigiert
  - Ein isolierter Teensy-4.1-PSRAM-Probe-Sketch meldet reproduzierbar `16 MB` statt der bisher angenommenen `8 MB`.
  - `platformio.ini`, der lokale Probe-Helfer und die Workspace-Defines verwenden dafuer jetzt `DUKATIMER_TEENSY_PSRAM_MB=16`.
  - Die aktive Hardwareuebersicht und das Part2-Pflichtenheft halten die korrigierte Speicher-Vorgabe jetzt ausdruecklich fest.
  - Kein Firmware-Version-Bump: Es wurde der Hardware-/Buildvertrag korrigiert, aber keine Laufzeitlogik geaendert.

- PAPER-CAL ist jetzt als echter lokaler Editierpfad verdrahtet
  - `PaperWorkflow` besitzt in `CAL` jetzt einen staged Profil-Editor fuer `gradeMode`, `fixedGradeValue`, `useIsoMath`, `isoP`, `isoR`, `kBw`, `kSoft`, `kHard` und den sichtbaren Kalibrierstatus statt nur eines vorbereiteten Unterkontexts.
  - `PaperProfileCommandPort` kann Profile jetzt slotgezielt ueber `saveProfileAt()` schreiben; CAL bearbeitet damit den gewaehlten Slot, ohne ihn vorher still zum aktiven PRINT-Papier machen zu muessen.
  - `ModeRuntimeState::paper` exportiert jetzt Edit-/Dirty-/Persistenzzustand und die staged Kalibrierwerte; `UiPresenter` zeigt daraus einen echten Placeholder-CAL-Editor mit Apply/Discard statt eines WIP-Hinweises.
  - Teensy-Version auf `0.2.30-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- PAPER-Familie ist jetzt als erster echter lokaler Workspace verdrahtet
  - `ModeId::Paper`, `PaperWorkflow` und `PaperModeRuntimeState` machen die bisher nur dokumentierte `PagePaperWorkspace` jetzt im Teensy-Runtimepfad sichtbar: Enc1/Enc2 drehen durch Slots, Enc3/Enc4 wechseln zwischen `SELECT` und dem vorbereiteten `CAL`-Unterkontext.
  - `PaperProfileCommandPort` kann den aktiven Slot jetzt explizit umschalten; `PaperSlotBankCommandAdapter` persistiert diese Auswahl direkt in der bestehenden Slot-Bank statt nur das aktive Profil zu speichern.
  - Der Placeholder-/Remote-Presenter zeigt den selektierten Slot jetzt als echte PAPER-Seite an; bis produktive EEZ-ModeTabs existieren, springen lokale Encodertaster per Tap provisorisch zwischen `PAPER` (Enc1) und `PRINT` (Enc2).
  - `SplitgradeWorkflow` laedt das aktive Papierprofil beim Wiedereintritt neu, wenn der PAPER-Workspace inzwischen einen anderen Slot aktiviert hat.
  - Teensy-Version auf `0.2.29-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- PaperGradeMode laeuft jetzt auch durch die vorbereiteten Vollprofil-UI-Pfade
  - `SystemSnapshot` traegt neben dem aktiven Slot jetzt ein kompaktes aktives Papierdetail und per-Slot-Summaries, die aus dem bestehenden `PaperProfileQueryPort` gefuellt werden.
  - Der BW-Placeholder wertet `gradeMode`, `fixedGradeValue`, `useIsoMath` und den naechsten Slot bereits sichtbar aus; derselbe Presenter-Pfad wird jetzt auch fuer den Remote-Render genutzt.
  - Die EEZ-Arbeitsdoku trennt damit klarer zwischen bereits vorhandenem Papierdatenvertrag im Snapshot und weiterhin fehlendem echten PAPER-/BW-Workflow.
  - Teensy-Version auf `0.2.28-dev` angehoben; `teensy41` Build nach dem Slice validieren.

- Aktives Papierprofil fuehrt fixed-grade/multigrade jetzt ueber einen expliziten Vertragstyp
  - `PaperExposureProfile` ersetzt das lose `fixedGrade`-Bool durch `PaperGradeMode`, so dass Persistenz, Workflow und Snapshot denselben benannten Typ fuer die Papierart teilen.
  - `SystemSnapshot` exportiert den aktiven Papiermodus jetzt sichtbar als `paperActiveGradeMode`; `UiPresenter` nutzt ihn bereits in SG-Header und Gateway-Diagnose.
  - `PaperSlotPersistenceCodec` validiert den neuen Typvertrag beim Laden der Slot-Bank; der AP-07-Splitgrade-Harness wurde auf denselben Vertrag umgestellt.
  - Teensy-Version auf `0.2.27-dev` angehoben; `teensy41` Build nach dem Slice validieren.

- Sichtbare UI-Telemetrie fuer adaptive Head-Bus-Latenz begonnen
  - `ExposureRuntimeState` exportiert jetzt `runtimeHeadBusLatencyMs` als sichtbare technische Millisekunden-Telemetrie; die Schema-Version steigt dafuer auf `4`.
  - `ExposureEngine` schreibt die konservativ geglaettete Head-Bus-Latenz bewusst in den sichtbaren Runtime-State, und `UiPresenter::getExposureInfo()` zeigt sie im aktuellen LVGL-Placeholder als `BUS:...ms` an.
  - `docs/neo_exposure_integration.md`, `docs/head_timing_and_i2c_timeout.md` und `docs/software/eez-studio-step-by-step-und-seitenuebersicht.md` dokumentieren den neuen UI-Vertrag.
  - Teensy-Version auf `0.2.26-dev` angehoben; `teensy41` Build nach dem Slice validieren.

- Setup-Menue kann Head-Testpattern plus Timing-Serialreport jetzt runtime-seitig schalten
  - `SetupWorkflow` fuehrt einen neuen runtime-only Aktionspunkt `HEAD DIAG`, der das feste NeoPixel-Diagnosemuster fuer Segment-/Eckmarker zusammen mit dem seriellen Head-Timing-Report toggelt, ohne diese Diagnose als persistente `SystemSettings` zu behandeln.
  - `main.cpp` fuehrt dafuer einen expliziten Head-Timing-Diagnostics-Port ein; die bisherigen Compile-Time-Hooks fuer Testpattern und Serialreport laufen jetzt ueber denselben runtime-seitigen Schalter und resetten beim Umschalten die Present-Statistik bewusst neu.
  - `docs/neo_exposure_integration.md` beschreibt die Hardware-Validierung jetzt ueber den neuen Setup-Menuepfad statt ueber eine manuelle Flag in `main.cpp`.
  - Teensy-Version auf `0.2.25-dev` angehoben; `teensy41` Build nach dem Slice validieren.

- AP-11g P3 Validierungs- und Sync-Basis begonnen
  - `test/ap11_measurement_domain/test_main.cpp` plus `tools/run_ap11_measurement_harness.py` pruefen die sichtbaren Measurement-Invarianten fuer Local-Range, Undo, Mixed-Source-Sperre, Overflow/Histogramm und Reference-Rebuild; ohne Host-Compiler faellt der Runner bewusst auf eine Syntax-Pruefung mit der installierten Teensy-Toolchain zurueck.
  - `tools/check_dukatimer_protocol_sync.py` vergleicht `lib/SharedProtocol/DukatimerProtocol.h` automatisiert mit der C6-Kopie unter `../Wireless TSL2591/include/DukatimerProtocol.h`; Part2- und C6-Builds laufen jetzt vor dem eigentlichen Compile durch diesen Guard.
  - `docs/software/dukatimer-part2-ap11g-measurement-validation-protocol.md` definiert das Cross-Source-Hardwareprotokoll, die explizite LogD-/Densitometer-Sperre und die hardware-nahe UI-Abnahmecheckliste fuer die Measurement-Seite.
  - Kein Firmware-Version-Bump in Part2: dieser Slice haertet Test-, Doku- und Build-Vertrag, ohne die Runtime-Logik im Teensy/ESP-Service erneut zu aendern.

- AP-11g P2 C6-Sensoralter ueber Protokollpfad getrennt von Peer-Freshness
  - `WirelessRemotePayload` traegt jetzt ein explizites `activeLuxAgeMs`, das am C6 aus dem letzten echten TSL2591-Samplezeitpunkt abgeleitet und bei neuem Sample auch dann erneut gesendet wird, wenn der Luxwert numerisch gleich blieb.
  - Gateway- und Teensy-Linkpfad fuehren daraus einen laufenden Sensor-Sample-Zeitstempel ab und publizieren im `WirelessSnapshotPayload` ein getrenntes `sensorSampleAgeMs`, statt `lastSeenAgeMs` missbraeuchlich als Messalter wiederzuverwenden.
  - `MeasurementDomainService` bewertet Wireless-Messstaleness jetzt gegen das echte C6-Sensoralter; Gateway-Diagnostik zeigt Sample- und Peer-Alter getrennt als `S...` und `P...` an.
  - Firmware-Version auf `0.2.24-dev` angehoben; nach SharedProtocol-Aenderung `teensy41` und `esp32s3_n16r8` bauen.

- AP-11g P2 Rollenworkflow fuer Measurement-Captures begonnen
  - `MeasurementCommandPort` und `MeasurementDomainService` besitzen jetzt eine explizite `pendingCaptureRole`, die per Measurement-Workflow zyklisch gewaehlt und beim naechsten lokalen oder Wireless-Capture als explizite Sample-Rolle auf das Session-Sample gelegt wird.
  - `MeasurementSessionStatus` exportiert diese Pending-Rolle sichtbar, und der Session-Mode springt fuer Kalibrier-/Dark-/Paper-Rollen bewusst in `PaperCalibration`, statt alle expliziten Rollen weiter unter `RelativeSpot` zu verstecken.
  - Die Measurement-Seite zeigt die aktive Capture-Rolle jetzt als Bedienhinweis `E2/C6 ROLE ...`, so dass die neue Rollenvergabe ohne neue UI-Seite direkt im bestehenden Panel nutzbar ist.
  - Teensy-Version zunaechst auf `0.2.23-dev` angehoben; der anschliessende Protokollslice zieht den Gesamtstand jetzt auf `0.2.24-dev`.

- AP-11g P2 Dark-/Offset-Vertrag pro Quelle begonnen
  - `MeasurementLuxSample`, `MeasurementReferenceStatus`, `MeasurementSessionSample` und `MeasurementSessionStatus` tragen jetzt einen expliziten `MeasurementCorrectionStatus`, damit die Measurement-Domain Raw-Lux, Dark-Offset-pending und spaetere Korrekturzustande nicht mehr still implizit vermischt.
  - `MeasurementDomainService` leitet fuer den aktiven Part2-Stand eine bewusste Default-Policy ab: lokaler TSL2561-Kopflux ist als `RAW` sofort zulaessig, waehrend der Wireless-TSL2591-Papierpfad bis zu einem echten Dark-Offset-Workflow explizit als `DARK?` beziehungsweise proposal-blockierend markiert bleibt.
  - Die Measurement-UI zeigt den Korrekturstatus jetzt sichtbar in Meta- und Range-Zeile; `PROP` wird nur noch dann gruen, wenn sowohl Range- als auch Korrekturvoraussetzungen tragen.
  - Teensy-Version auf `0.2.22-dev` angehoben; `teensy41` Build nach dem Slice validieren.

- AP-11g P2 Rollenvertrag in der Measurement-Domain begonnen
  - `MeasurementSessionSample` traegt jetzt einen `roleMask`; `MeasurementSessionStatus` exportiert zusaetzlich `mode`, den aggregierten Session-`roleMask` und `hasExplicitRoles`, damit kuenftige Proposal-/Kalibrierpfade technische Session-Anker von spaeteren fotografischen Nutzerrollen trennen koennen.
  - `MeasurementDomainService` rekonstruiert die heute bereits belegbaren Rollen bewusst aus der sichtbaren History: erste sichtbare Probe je Quelle wird wieder `Reference`, die aktuellen EV-Extrema tragen `RangeLow`/`RangeHigh`; Undo, Overflow und Reset koennen damit keine stale Rollenanker im Vertrag hinterlassen.
  - Die Measurement-UI zeigt den Sessionmodus jetzt explizit als `MOD ...` an, und die Range-Zeile benennt die Extrema neutral als `LO/HI`, damit der aktuelle Slice keine fotografischen Shadow-/Highlight-Claims aus blossen EV-Min/Max ableitet.
  - Teensy-Version auf `0.2.21-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- AP-11g P2 Formatter- und Praezisionsbasis begonnen
  - `MeasurementValueFormatter` formatiert Measurement-Quellen, aktiven Luxwert, Referenz-/dEV-Zeile und Range-Zeile jetzt zentral statt diese Measurement-Texte weiter roh im `UiPresenter` zusammenzusetzen.
  - Die sichtbare Lux-Praezision ist jetzt quellenabhaengig: lokaler TSL2561-Kopflux bleibt ehrlich grob mit Integer-Lux, waehrend der Wireless-TSL2591-Papierpfad seine Milli-Lux-Sichtbarkeit behaelt.
  - Ungueltige Measurement-Werte werden auf der Messseite nicht mehr als `0.000 lux` oder `+0.00` kaschiert, sondern explizit als `NO SAMPLE`, `NO REF` oder `--.--` angezeigt.
  - Der P2-Schritt bleibt bewusst auf den Teensy-Formatter-Slice begrenzt; C6-Samplealter folgt weiter spaeter als eigener Cross-MCU-Schnitt.
  - Teensy-Version auf `0.2.20-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- AP-11g P1 Referenz-Rebuild und Wireless-Arming umgesetzt
  - `MeasurementDomainService` rekonstruiert Session-Referenzen jetzt aus der sichtbaren Undo-History, so dass nach Undo oder Overflow keine bereits herausgefallene Probe als stiller lokaler oder Wireless-Referenzanker weiterlebt.
  - Beim History-Overflow wird der Referenzraum direkt nach dem Herausschieben des aeltesten Samples neu aufgebaut, bevor das neue Sample seine relative EV-Lage gegen die aktuelle sichtbare Session bestimmt.
  - Wireless-Measure-Sequenzen werden jetzt nur noch im aktiven Measurement-Kontext (`SplitgradePanel::Measurement`) als Session-Capture akzeptiert; ausserhalb davon werden sie einmalig verbraucht und koennen spaeter nicht nachtraeglich in die Session einsickern.
  - Teensy-Version auf `0.2.19-dev` angehoben; `teensy41` Build nach dem Slice validiert.

- AP-11g P0 Measurement-Hygiene umgesetzt
  - `MeasurementDomainService` entfernt beim Session-Overflow jetzt den Histogrammbeitrag des herausfallenden Samples vor dem neuen Append und fuehrt einen expliziten `droppedSampleCount`, so dass Histogramm und sichtbare History nicht mehr auseinanderdriften.
  - `MeasurementSessionStatus` trennt aktuelle Sessiongroesse (`sampleCount`) jetzt sauber von Lifetime-Captures (`capturedSampleCount`) und exportiert zusaetzlich Source-Komposition (`sourceMask`, `sourceCount`, `commonSource`, `mixedSources`) sowie die strengere Proposal-Vorstufe `rangeUsableForProposal`.
  - Die Measurement-Seite zeigt Counts jetzt explizit als `CUR/CAP/DR`; die Range-Zeile zeigt Quelle und Proposal-Freigabe sichtbar an, statt Source-Mix nur implizit zu verstecken.
  - Die normale SG-Dose-Telemetrie liest `LUX` jetzt wieder ausschliesslich aus `ExposureRuntimeState.measuredLux`; ein onlineer Wireless-Spotpfad kann damit nicht mehr den Head-/Dose-Lux der Belichtung uebermalen.
  - Teensy-Version auf `0.2.18-dev` angehoben; `teensy41` Build nach dem Slice erneut validiert.

- AP-11e Measurement-Panel als eigener SG-Slice begonnen
  - `SplitgradePanel` besitzt jetzt ein explizites `Measurement`-Panel, das im bestehenden lokalen LVGL-Placeholder eine eigene Messseite mit Quellenblock, aktivem Luxwert, Referenz-/dEV-Anzeige, 11er Histogramm sowie Status-/Undo-Zeile sichtbar macht.
  - `UiPresenter` liefert dafuer separate Measurement-Strings; Remote-/Placeholder-SG-Header reagieren auf das neue Panel und zeigen nicht mehr nur generische SG-Zielwerte.
  - Auf dem Measurement-Panel sind `Start` und lokales `Measure` jetzt explizite Session-Captures fuer den lokalen TSL2561-Pfad, waehrend `Undo` die letzte Session-Messung rueckgaengig macht.
  - Der C6-Measure-Button bleibt bewusst an den vorhandenen `measurementSequence`-Pfad gebunden und erzeugt auf der neuen Messseite kein lokales Capture-Duplikat und keinen versehentlichen Belichtungsstart.
  - Teensy-Version auf `0.2.17-dev` angehoben; `teensy41` Build validiert.

- AP-11d Measurement-Session um Schatten/Lichter-Range erweitert
  - `MeasurementDomainService` leitet aus der bestehenden Session-Historie jetzt explizit den dunkelsten und hellsten session-relativen Messpunkt sowie die dazwischen aufgespannte EV-Spreizung ab.
  - Die neuen Session-Extrema bleiben bewusst im bereits eingefuehrten quellengebundenen Referenzraum und behaupten damit weiterhin keine absolute Papierzone; sie bilden nur die aktuell gemessene Spreizung fuer kuenftige Messscreen- und Vorschlagslogik ab.
  - `UiPresenter` zeigt diese Session-Range in der bestehenden Gateway-/Measurement-Diagnose jetzt als `RNG shadow..highlight` plus Spannweite, damit der neue Slice im Placeholder-UI und Remote-Renderpfad sofort sichtbar ist.
  - Teensy-Version auf `0.2.16-dev` angehoben.

- Teensy-Storage-Policy gegen `SdFat`/`FS.h`-Drift festgezogen
  - Der Teensy-Dateisystempfad ist jetzt explizit ueber `src/teensy/TeensyStoragePolicy.h` auf nativen SdFat-Zugriff mit `SdFs`, `FsFile` und `SdioConfig(FIFO_SDIO)` festgelegt.
  - Ein neuer PlatformIO-Pre-Build-Check blockiert im Teensy-Quellpfad ab jetzt `SD.h`, `FS.h`, direkte `SdFat.h`-Includes und bare `File`, damit keine gemischte File-Abstraktion still einzieht.
  - Die bisherige upstream-`SdFat.h`-Kompatibilitaetswarnung wird im Teensy-Build nun gezielt via `DISABLE_FS_H_WARNING` unterdrueckt, weil der Projektvertrag dafuer jetzt explizit und automatisch geprueft ist.

- AP-11c ESP32-S3-Sensorpolling an aktive Belichtung gekoppelt
  - Der Teensy publiziert jetzt im bestehenden Heartbeat ein kompaktes Runtime-Flag fuer aktive Belichtung, so dass der ESP32-S3 den Belichtungszustand ohne neuen Kommandokanal zeitnah sehen kann.
  - Der ESP32-S3 liest den DS18B20-OneWire-Pfad jetzt nur noch alle `5s`, behaelt diesen Pollingpfad aber ausdruecklich auch waehrend aktiver Belichtung bei.
  - AHT20 und BMP280 laufen jetzt mit `30s` Pollingintervall; waehrend aktiver Belichtung werden ihre I2C-Lesezyklen bewusst ausgesetzt und erst nach Belichtungsende wieder aufgenommen.
  - Teensy-Version auf `0.2.15-dev` und ESP-Service-Version auf `0.1.7-dev` angehoben.

- AP-11b ESP32-S3-Umweltsensorpfad um BMP280 erweitert und UI-Status vervollstaendigt
  - Der ESP32-S3-Servicepfad liest jetzt AHT20 und BMP280 gemeinsam auf dem U-SENS1-I2C-Bus und publiziert Temperatur, Luftfeuchte sowie Luftdruck als gemeinsamen Service-Snapshot an den Teensy.
  - `EspServiceSnapshotPayload`, `EspLinkRuntimeStatus` und `UiPresenter` wurden erweitert, so dass das Teensy-UI jetzt AHT20-Temperatur/Feuchte und BMP280-Temperatur/Druck sichtbar im Statuspfad anzeigt.
  - EEZ-Arbeitsdoku und Hardwareuebersicht benennen den S3-Umweltsensorpfad jetzt explizit als AHT20 plus BMP280; Teensy-Version auf `0.2.14-dev` und ESP-Service-Version auf `0.1.6-dev` angehoben.

- AP-11a Setup-Workflow und getrennte SystemSettings begonnen
  - Neuer globaler `SetupWorkflow` haengt jetzt produktiv am Modussystem; `ModeCoordinator` ist nicht mehr auf genau zwei Workflows fixiert und kann den neuen `ModeId::Setup` mitfuehren.
  - Long-Press auf lokalen Encoder 3 oeffnet das Setup aus einem idle-faehigen Modus heraus; bei sauberem, nicht-dirty Setup schliesst dieselbe Geste den Modus wieder.
  - Neue globale `SystemSettings` werden getrennt von `PaperExposureProfile` als eigener SD-Blob (`/systemsettings.bin`) versioniert und integritaetsgeprueft gespeichert.
  - Der erste Setup-Slice fuehrt die Pflichtwerte aus der Zielvorgabe ein: Sound-Modus, Lautstaerke, Vibration an/aus, NeoPixel-Maxhelligkeit sowie Thermikschwellen fuer Derating und Hard-Stop.
  - Thermische Grenzwerte werden jetzt aus `SystemSettings` in `SensorManager` und `ExposureEngine` gespiegelt; der globale Head-Cap wirkt im realen Head-Output-Pfad, und Remote-Haptic respektiert den neuen Vibrationsschalter.
  - Placeholder-UI kann den Setup-Modus jetzt sichtbar darstellen; Teensy-Version auf `0.2.13-dev` angehoben.

- AP-10d Laufender TSL2561-Fallback markiert Papierergebnis explizit als unsicher
  - Eine bereits laufende lokale Dose-Belichtung bleibt beim TSL2561-Ausfall im bestehenden Fallback-Pfad und laeuft ueber die letzte plausible Dosisrate als berechnete Restzeit zu Ende; sie wird nicht neu gestartet.
  - Overlay und Exposure-Diagnose benennen jetzt explizit, dass Closed-Loop waehrend des Fallbacks aus ist und das fertige Ergebnis nur noch auf Timer-/Restzeitschaetzung beruht und deshalb nicht vertrauenswuerdig ist.
  - Teensy-Version auf `0.2.12-dev` angehoben; `teensy41` Build validiert.

- AP-10c TSL2561-I2C-Degradierung auf expliziten Time-Fallback verdrahtet
  - Der Teensy-Wiring-Layer erzwingt fuer neue Splitgrade-Starts Time-Modus, wenn der lokale TSL2561-Dienst wegen `InitializationFailed`, `ReadFailure`, `InterruptWatchdog` oder `StaleSample` nicht mehr als verlässlich fuer lokale Dose-Regelung gilt.
  - Ein durch den bestehenden Main-Loop-Watchdog erkannter Anwendungs-Reset wird jetzt als latched Diagnose gehalten und sperrt Dose-Starts fuer die laufende Session ebenfalls auf Time, statt nach dem Neustart still wieder in Closed-Loop zu gehen.
  - Overlay- und Exposure-Diagnose zeigen den Degradationsgrund sichtbar als `WDOG` oder TSL-Diagnosekuerzel an; der eigentliche `WireIMXRT`-Timeout bleibt bewusst nicht vorgetaeuscht.
  - Teensy-Version auf `0.2.11-dev` angehoben; `teensy41` Build validiert.

- AP-10b Normierter Teensy-RotaryEncoderDriver fuer lokale Encoder integriert
  - Die lokale Teensy-Eingabeschicht nutzt fuer Encoder 1-3 nicht mehr den positionsbasierten `EncoderState`-Wrapper in `main.cpp`, sondern einen eigenen `RotaryEncoderDriver` mit explizitem Detent-Akkumulator ueber `Encoder.h`.
  - Der neue Treiber plausibilisiert die beobachtete A/B-Phase gegen das Rohzaehlwerk, zaehlt partielle Richtungswechsel vor einer vollen Rastung sowie Phasenmismatches und exportiert diese Diagnose sichtbar in die Encoder-UI-Zeile.
  - Die bestehende Eingabesemantik bleibt erhalten: lokale Encoder liefern weiterhin nur normierte Dreh- und Tastereignisse an `InputNormalizer` und `InputRouterPolicy`.
  - Teensy-Version auf `0.2.10-dev` angehoben; `teensy41` Build validiert.

- AP-10a ESP32-PCNT fuer lokalen Encoder 4 und C6-Wireless-Encoder integriert
  - Der ESP32-S3 liest Encoder 4 jetzt lokal ueber `ESP32Encoder`/PCNT statt als unverdrahteten Platzhalter und publiziert nur bestehende `RemoteInputSource::Encoder4`-Rotate-/Press-/LongPress-Ereignisse an den Teensy-Link.
  - Der Teensy konsumiert diese Events weiterhin ueber den vorhandenen semantischen Pfad (`RemoteEncoder4` als Kontextnavigation/Confirm), so dass Encoder 4 Menue-/Modusfunktionen bedient und keine neue belichtungsrelevante Direktrolle erhaelt.
  - Das C6-Wireless-Terminal ersetzt die bisherige `RotaryEncoder`-ISR durch denselben PCNT-Treiber (`ESP32Encoder`) und behaelt die bestehende `input.h`-API fuer Encoder-Delta und Retry-Restore unveraendert bei.
  - ESP-Service-Version auf `0.1.5-dev` angehoben; `esp32s3_n16r8` Build validiert. Der C6-Build ist in dieser Session an einem Plattform-/Package-Problem (`MissingPackageManifestError` im pioarduino-Umfeld) haengen geblieben; die geaenderte C6-Datei zeigt aber keinen Editor-Fehlerzustand.

- Measurement-Histogramm: Session-Referenz statt festem 1-Lux-Anker (P2) integriert
  - `MeasurementDomainService` legt pro Messquelle jetzt einen expliziten Session-Referenzpunkt aus der ersten gueltigen Messprobe an, statt relative Zonen weiterhin aus `log2(lux / 1.0)` abzuleiten.
  - Die relative EV-Umrechnung liegt jetzt zentral in `ExposureValueMath`; `MeasurementSessionSample` speichert Referenz-Lux und relative EV-Stops sichtbar mit, so dass Histogramm und Undo an derselben Semantik haengen.
  - `MeasurementRuntimeStatus` exportiert aktive Referenz und aktuellen relativen EV-Abstand; der Teensy-Gatewayblock zeigt damit nicht nur rohe Luxwerte, sondern auch den aktuellen Session-Anker sowie `dEV`/`REF` fuer die letzte Session-Messung.
  - Teensy-Dev-Version auf `0.2.9-dev` angehoben; Build fuer `teensy41` validiert.

- AP-09i Explizite C6-Renderverlust-Diagnose bis Gateway/Teensy (P2) integriert
  - Das C6 zaehlt jetzt lokal verworfene stale `RemoteDisplayPayload.sequenceNumber`-Pakete sowie Render-Timeout-Episoden und publiziert diese Diagnose ueber den bestehenden `WirelessRemotePayload`-Rueckkanal.
  - `WirelessRemotePayload`/`WirelessSnapshotPayload` tragen jetzt zusaetzlich Renderstatus, stale-Render-Zaehler und Render-Timeout-Zaehler; die ESP-NOW-ABI des Terminal-Rueckkanals wurde in Hauptprojekt und C6 gemeinsam auf `25` Byte aktualisiert.
  - `WirelessRemoteGateway` hebt Counter-Anstiege in echte ESP-Service-Diagnosecodes (`WirelessRenderStale`, `WirelessRenderTimeout`) und reicht die sichtbaren Zaehler ueber den Wireless-Snapshot bis in den Teensy-Gatewaystatus durch.
  - Das C6 zeigt lokale Renderverlust-Diagnose im Footer (`RS... RT...`) sichtbar an; der Teensy-Gatewayblock zeigt den aktuellen Renderstatus samt stale-/timeout-Countern.
  - Dev-Versionen auf Teensy `0.2.8-dev` und ESP-Service `0.1.4-dev` angehoben. `teensy41` und `esp32s3_n16r8` Build validiert; der C6-Build lief in dieser Session durch wiederholte Erstinstallation der pioarduino-Abhaengigkeiten, die geaenderten C6-Dateien zeigen aber keinen Editor-Fehlerzustand.

- AP-09h Nicht-blockierende Teensy-TX-Queue fuer den ESP-Servicepfad (P1) integriert
  - `EspServiceLink` puffert Heartbeat-, Command-, Render-, Diagnose- sowie VFS-Ack/Error-Frames jetzt lokal und schreibt nur noch dann in den UART, wenn `availableForWrite()` genug Platz meldet; direkte blockierende `serial_.write(...)`-Pfadstellen aus dem Teensy-Hauptloop sind entfernt.
  - Der Teensy-Sendepfad priorisiert `TeensyCommand` vor VFS-Antworten, Heartbeats, Render und Diagnose; pending Heartbeat/Render/Diagnose koaleszieren auf Latest-State, waehrend verdraengte oder verworfene Frames als Queue-Diagnose mitgezaehlt werden.
  - `EspLinkRuntimeStatus` und die Gateway-Diagnose zeigen jetzt `TXQ`-Belegung sowie kumulative Drop-/Eviction-/Coalescing-Zaehler des Teensy-UART-Ausgangs.
  - Teensy-Dev-Version auf `0.2.7-dev` angehoben; Build fuer `teensy41` validiert (bekannte `SdFat.h`-Warnung unveraendert).

- AP-09g RemoteCommandTracker als eigener UI-Block und Diagnose-Telemetrie bis C6 (P1) integriert
  - Der Teensy-Placeholder zeigt `RMT` jetzt als eigene Diagnosezeile mit In-Flight-Belegung, Ack, letzter Tracking-Sequenz sowie kumulativen Retry-/Saettigungs-/Timeout-Zaehlern statt nur als Gateway-Praefix.
  - Teensy sendet bei veraenderten RMT-Zaehlern echte `DiagnosticPayload`-Frames ueber den ESP-Servicepfad; der ESP-S3 leitet diese als eigene Pending-Diagnosen per ESP-NOW ans C6 weiter.
  - Das C6 fuehrt die kumulativen RMT-Zaehler lokal nach und zeigt sie ausserhalb aktiver Progress-Balken als kompakten Diagnose-Footer; eingehende Diagnosen werden zusaetzlich seriell protokolliert.
  - Teensy-Dev-Version auf `0.2.6-dev` angehoben.

- AP-09f Sichtbare RemoteCommandTracker-Diagnose in UI/Gateway (P1) integriert
  - Der Teensy exportiert jetzt `RemoteCommandTracker`-Status als Snapshotdaten fuer UI und Remote-Render: aktuelle in-flight-Belegung, kumulative Retry-Zaehler, Queue-Saettigung und Timeoutzaehler.
  - Die bestehende Gateway-Info beginnt jetzt mit einem kompakten `RMT ...`-Praefix. Dadurch sehen sowohl das lokale Teensy-Placeholder-UI als auch die gekuerzte Gateway-Zeile auf dem C6 sofort Retry-/Saettigungszustand und aktuelle Ack-Sequenz.
  - Teensy-Dev-Version auf `0.2.5-dev` angehoben.

- AP-09e Multi-In-Flight-RemoteCommandTracker und Command-Policy (P1) integriert
  - `RemoteCommandTracker` verfolgt jetzt mehrere kritische Remote-Kommandos in Sendereihenfolge statt nur eines einzelnen Slots; kumulative `commandAckSequence` raeumt bestaetigte Eintraege wrap-sicher ab.
  - Die Command-Policy liegt jetzt im Tracker selbst: `RemoteMeasurementStart` und `RemoteMeasurementCancel` bleiben kritisch mit Ack-Retry/Timeout, `RemoteHaptic` und `Ping` bleiben bewusst fire-and-forget.
  - Vor jedem neuen Tracking-Eintrag werden bereits bestaetigte Kommandos kompakt entfernt; wenn trotzdem kein Platz mehr fuer ein kritisch zu ueberwachendes Kommando bleibt, faellt der Teensy fail-closed in einen Exposure-Fault statt still Tracking zu verlieren.
  - Teensy-Dev-Version auf `0.2.4-dev` angehoben.

- AP-09d Teensy-RemoteCommandTracker fuer Ack-Retry/Timeout (P1) integriert
  - Neuer `RemoteCommandTracker` im Teensy-Wiring (`main.cpp`) vermittelt zwischen Workflow und `EspServiceLink`, ohne Link-Transport oder Workflows mit Netzwerktimeouts zu belasten.
  - Kritische Remote-Messkommandos (`RemoteMeasurementStart`/`RemoteMeasurementCancel`) werden jetzt ack-basiert ueberwacht und bei ausbleibendem Ack mit `150ms` Timeout bis zu `4` Mal erneut gesendet.
  - Bei finalem Timeout wird der Teensy in einen sichtbaren Exposure-Fault ueberfuehrt, damit Splitgrade-Ablauf und Sicherheitslogik nicht in einen stillen Deadlock laufen.
  - Teensy-Dev-Version auf `0.2.3-dev` angehoben.

- AP-09c Wireless-Render-Freshness und Offline-Fallback (P1) integriert
  - Das C6 akzeptiert `RemoteDisplayPayload` jetzt nur noch mit strikt neuer `sequenceNumber`; stale, replayte oder verspätete Renderpakete verlaengern den Slave-Timeout nicht mehr und ueberschreiben die Anzeige nicht mehr.
  - Nach einer echten Remote-Session rendert das Terminal bei ausbleibenden frischen Renderdaten einen expliziten Offline-Fallback mit lokaler Lux-/EV-Anzeige statt still den letzten Remote-Screen stehen zu lassen.
  - Das Wireless-Terminal besitzt weiterhin keine zentrale `FirmwareVersion.h`; fuer diesen Slice wurde die Luecke bewusst nur dokumentiert und nicht mit einer neuen Versionsquelle vermischt.

- AP-09b Wireless-Command-Ack-Rueckkanal (P1) integriert
  - `WirelessRemotePayload` fuehrt jetzt die terminalseitig wirklich im Loop angewendete `commandAckSequence`; die ESP-NOW-ABI wurde in Hauptprojekt und C6-Terminal gemeinsam auf `15` Byte aktualisiert.
  - Das C6 bestaetigt Kommandos erst nach lokaler Verarbeitung im Hauptloop, nicht schon im ESP-NOW-Callback. Dadurch trennt der Rueckkanal Funkannahme und echte Terminal-Ausfuehrung.
  - `WirelessRemoteGateway`, `TeensyLinkService` und `EspServiceLink` reichen die Ack-Sequenz jetzt bis in den sichtbaren Teensy-Gatewaystatus durch; `UiPresenter` zeigt sie als `ACK#...` in der Gatewaydiagnose an.
  - Dev-Versionen auf ESP-Service `0.1.3-dev` und Teensy `0.2.2-dev` angehoben. Das Wireless-Terminal besitzt weiterhin keine zentrale `FirmwareVersion.h`.

- AP-09a Wireless-Command-Forwarding (P1) integriert
  - `WirelessRemoteGateway` leitet `RemoteMeasurementStart`, `RemoteMeasurementCancel`, `RemoteHaptic` und `Ping` jetzt als rohe `TeensyCommandPayload`-Pakete per ESP-NOW an das C6-Terminal weiter, statt die Kommandos lokal zu verschlucken.
  - Das Wireless-Terminal verarbeitet eingehende Teensy-Kommandos asynchron im Loop statt im ESP-NOW-Callback; damit bleiben lokaler TSL2591-Messtask und Remote-Sendefenster unblockiert.
  - Das C6 nutzt `RemoteMeasurementStart/Cancel` fuer einen expliziten Remote-Messkontext auf dem OLED und fuehrt `RemoteHaptic` ueber einen nicht-blockierenden Haptik-Scheduler aus.
  - ESP-Service-Version auf `0.1.2-dev` angehoben. Das Wireless-Terminal besitzt weiterhin keine zentrale `FirmwareVersion.h` und bleibt diesbezueglich ein offener Punkt.

- AP-09 Teensy/ESP Service- und Remote-Pfade (P1) integriert
  - Shared Protocol um `RemoteDisplayPayload` und `MessageType::RemoteRender` erweitert, damit Teensy-Kommandos und datengetriebene Renderdaten produktiv ueber den Service-Link laufen.
  - ESP-NOW-ABI fuer `WirelessRemotePayload` und `RemoteDisplayPayload` jetzt mit exakten `static_assert`-Groessen in Hauptprojekt und Wireless-Terminal abgesichert.
  - Dev-Versionen auf Teensy `0.2.1-dev` und ESP-Service `0.1.1-dev` angehoben; das Wireless-Terminal besitzt derzeit keine eigene FirmwareVersion-Quelle.
  - Teensy-Seite verdrahtet: `EspServiceLink` sendet jetzt Remote-Render/Kommandos; `main.cpp` erzeugt aus `UiPresenter`/`SystemSnapshot` echte Remote-OLED-Inhalte, startet und beendet Remote-Messkommandos aus dem SG-Ablauf und sendet haptisches Feedback bei Zustandswechseln.
  - ESP-Seite verdrahtet: `TeensyLinkService` puffert Kommandos und Renderdaten und sendet jetzt auch `DiagnosticPayload`; `WirelessRemoteGateway` bindet die historische Wireless-TSL2591-Fernbedienung per ESP-NOW als echten Controller/Meter an; `ServiceSensorHub` publiziert DS18B20- und AHT-Telemetrie inkl. Fehlerflags statt Platzhalterwerten.
  - Sichtbare Telemetrie erweitert: Gateway-Status zeigt jetzt Wireless-Sequenz/Batterie, Service-Sensorflags, AHT-Werte sowie den letzten expliziten Diagnosecode mit Detail/Countern an.
  - Build validiert fuer `teensy41` und `esp32s3_n16r8` (BUILD_OK).

- AP-04 Presenter-Slice (P0) weitergefuehrt
  - `UiPresenter`-Methoden in `src/teensy/UiPresenter.cpp` rein stilistisch in mehrzeilige, besser lesbare Form gebracht (ohne Ablaufaenderung).
  - Presenter-Verdrahtung explizit gemacht: `LvglUi` erhaelt `UiPresenter` jetzt per Konstruktor-Injektion aus `main.cpp` statt interner Instanz.
  - Build validiert fuer `teensy41` (BUILD_OK).

- AP-04 Workflow-Service-Slice (P0) vorbereitet
  - `ModeWorkflowServices` als minimales Injektions-Skelett angelegt.
  - `MeasurementQueryPort` und `PaperProfileQueryPort` als reine Interface-Stubs vorbereitet.
  - `IModeWorkflow` und `ModeCoordinator` besitzen jetzt mit `bindServices()` einen festen Service-Rand ohne Verhaltensaenderung der bestehenden Workflows.
  - Build validiert fuer `teensy41` (BUILD_OK).

- AP-05 Paper-Slot-Persistenz (P0) integriert
  - `PaperProfileQueryPort` von Stub auf konkreten Read-Query-Umfang erweitert.
  - `PaperSlotBankQueryAdapter` hinzugefuegt und in `main.cpp` als `workflowServices.paperProfiles` verdrahtet.
  - `PaperSlotPersistenceCodec` um `PaperSlotBlobParseError` und `parseBlobWithError(...)` erweitert, bestehendes `parseBlob(...)` bleibt als Wrapper erhalten.
  - `PaperSlotStorage` mappt Parsefehler differenziert auf Storage-Fehler (`UnsupportedFormatVersion`, `InvalidBank`, generische `InvalidBlob`-Faelle mit Detailwert).
  - Start-Recovery in `setupPaperSlotPersistence()` erweitert: fehlende/ungueltige/nicht kompatible Daten werden auf Defaults zurueckgesetzt und kontrolliert rueckgeschrieben.
  - Aktive Slot-Selektion wird normalisiert (gueltiger Bereich, bevorzugt erster kalibrierter Slot).
  - `SystemSnapshot` und `UiPresenter` erweitern Diagnoseausgabe um Slot- und Persistenzstatus (Code/Detail, Slotindex, Kalibrierung, Name).
  - Testleitfaden erstellt: `docs/software/ap-05-paperslot-persistenz-testleitfaden.md`.

- AP-06 UI-Kern, Modalebene und Fokusmodell (P0) integriert
  - `InputRouterPolicy` um globales Modalmodell (`WorkflowFault`, `WorkflowConfirm`, `WorkflowWait`) und expliziten EventGuard-Zustand erweitert.
  - Fokusprioritaet klar gemacht: `Modal` > `Touch` > `Encoder`.
  - Globaler EventGuard vor Workflow-Dispatch verdrahtet; Guard-Entscheidung ist jetzt action-aware (nicht nur event-kind-basiert).
  - `SystemSnapshot` exportiert Fokus-, Modal- und Guard-Codes fuer UI/Diagnose.
  - `UiPresenter` liefert globales Modal-Overlay und zeigt Fokus/Modal/Guard sichtbar in `ModeInfo`.
  - `LvglUi` faerbt Overlay global nach Modalstatus statt rein SG-spezifischer Bedingung.
  - DoD-Dokument fuer Blindbedienung und Abnahme erstellt: `docs/software/ap-06-ui-kern-modal-fokusmodell-dod.md`.

- AP-07 SG-Mathematik und papiergetriebene Vorschlaege (P1) integriert
  - `SplitgradeWorkflow` initialisiert seine SG-Startwerte jetzt aus dem aktiven `PaperExposureProfile` ueber `workflowServices.paperProfiles` statt nur aus lokalen Konstanten.
  - Gradationswechsel berechnen Soft-/Hard-Targets papiergetrieben neu; `fixedGrade`, ISO-Fallback und LUT-basierte Split-Anteile werden im Workflow beruecksichtigt.
  - 0,5er-SG-LUT bleibt als feste 11-Stufen-Tabelle erhalten; der Workflow verwendet den normalisierten Gradationsindex jetzt direkt fuer die Papier-LUT.
  - ISO-Math nutzt nun `isoP` fuer die CHD-Zielbasis (`100 / isoP`) und `isoR` als begrenzte Kontrast-Bias im Splitverhaeltnis; ungueltige Werte fallen auf P/R 100 zurueck.
  - Manuelle Soft-/Hard-Edits werden auf eine gemeinsame Basisgroesse und die laufende LUT-Suggestion zurueckgeschrieben, damit Folgegrade nicht wieder auf rohe Defaults zurueckfallen.
  - AP-07-Acceptance-Harness um Regressionen fuer 0,5er-LUT, ISO-P-Kompensation, ISO-R-Bias und ISO-Fallbacks erweitert.
  - Der bestehende Execution-State-Ablauf bleibt erhalten, ueberspringt aber jetzt fachlich leere Split-Phasen (`soft==0` oder `hard==0`) statt immer blind `WaitForFilter` zu erzwingen.
  - Hardwarebezogenes Abnahmeprotokoll fuer die vier Pflichtfaelle erstellt: `docs/software/ap-07-sg-hardware-abnahmeprotokoll.md`.
  - Build validiert fuer `teensy41` (BUILD_OK).

- Dokumentationsentscheidungen fuer Belichtungslogik und Messwerte
  - F-Stop-/EV-Logik wird fuer Part2 als zentrale, modusunabhaengige Belichtungswert-Schicht gefuehrt und nicht mehr als isolierter BW-Sondermodus.
  - Belichtungsrelevante Modi sollen dieselbe logarithmische Schrittlogik verwenden.
  - Messwerte aus echten Messvorgaengen werden zusaetzlich in EV dargestellt, sofern die fotografische Einordnung sinnvoll ist; reine Runtime-Telemetrie bleibt in Lux, ms oder C.
  - Logik, die mehrfach verwendet wird, ist kuenftig nur einmal zentral zu definieren und von den Modi zu konsumieren.

- NeoPixelHead: present() Timing-Instrumentation
  - `NeoPixelHeadPresentStats` hinzugefügt (`presentCalls`, `appliedFrames`, `skippedFrames`, `lastDurationUs`, `averageDurationUs`, `maxDurationUs`).
  - APIs: `presentStats()`, `resetPresentStats()`.
  - Dateien: `src/teensy/NeoPixelHead.h`, `src/teensy/NeoPixelHead.cpp`.

- ExposureEngine: Runtime-Ingestion von Head-Latenz
  - Neue API `observeHeadPresentDurationUs(uint32_t)` implementiert.
  - `runtimeHeadBusLatencyMs_` mit konservativem Clamp und langsamer Abwärts-Glättung; `predictiveShutoffLeadMs()` verwendet runtime-Latenz + `kHeadFadeOutMs`.
  - Dateien: `src/teensy/ExposureEngine.h`, `src/teensy/ExposureEngine.cpp`.

- main.cpp: Synchronisierung + optionales Reporting
  - `syncHeadTimingIntoExposureEngine()` hinzugefügt und optionales serielles Reporting (`kEnableHeadTimingSerialReport`).
  - `neoPixelHead.resetPresentStats()` beim Start.
  - Datei: `src/teensy/main.cpp`.

- Dokumentation
  - Neue Doku: `docs/neo_exposure_integration.md` — beschreibt Messung, APIs und Validierungsablauf.

- Build
  - Änderungen erfolgreich für `teensy41` kompiliert (Build-Validierung durchgeführt).

Hinweis: Hardware-Messungen (Stromaufnahme / reale present()-Dauern unter Last) sind noch ausstehend. Empfohlen: serielles Reporting aktivieren und Messläufe durchführen, um `maxDurationUs` zu validieren.

Nächste Schritte:

- Changelog-Eintrag in die Versionshistorie einpflegen und committen.
- Optional: UI-Telemetrie hinzufügen, die `runtimeHeadBusLatencyMs_` anzeigt.
