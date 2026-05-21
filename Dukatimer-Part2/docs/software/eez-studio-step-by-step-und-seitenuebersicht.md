# EEZ Studio: Verbindliche Implementierungsanweisung fuer das Dukatimer-Part2-UI

Stand: 2026-05-08

---

## 0. Zweck und Bindungskraft

Dieses Dokument ist die einzige verbindliche Arbeitsanweisung fuer den Aufbau
der produktiven EEZ-Studio-UI in `Dukatimer-Part2`.

Es ersetzt keinen Editor-Workflow, sondern trifft vorab alle Entscheidungen, die
spaeter in EEZ nicht mehr guenstig revidierbar sind:

- Screenmodell und Seitenzuordnung
- Eingabemodell und Blindbedienbarkeit
- Variablennamen und Datenherkunft
- Farbkonvention und Zustandscodierung
- Architekturgrenze zwischen EEZ-Code und Firmware-Laufzeit
- Integrationsregel fuer den Einbau in `LvglUi`

Regel: Alle Abweichungen von diesem Dokument muessen in einem
Entscheidungseintrag in `offene_entscheidungen.md` begruendet werden, bevor die
Abweichung in Code umgesetzt wird.

---

## 1. Ist-Stand der UI (code-verifiziert)

| Punkt | Aktueller Befund |
| --- | --- |
| EEZ-Projektfile | **vorhanden** — `DukatimerPart2TeensyUi.eez-project` (19 209 Zeilen, valides JSON); 7 User-Pages (`userPages`), 155 globale Variablen, 19 benannte Farben, 3 UserWidgets (`CompMessageBand`, `CompHeader`, `CompModeTabs`), 19 `lvglStyles` (11 Grundstile + 8 MessageBand-Zustandsstile) — designseitig angelegt, `styles.c` runtime-seitig noch leer |
| Aktive UI | Boot, PageSplitgrade, PagePaperWorkspace, PageSetup, PageMeasurement, PageWirelessRemote und Busy-Screen besitzen echte LVGL-Widget-Baeume in `screens.c`; `LvglUi::pushWidgetsFromSnapshot()` aktualisiert diese Widget-Handles direkt aus `SystemSnapshot` |
| EEZ-Runtime-Rolle heute | `ui_init()` initialisiert EEZ Flow, `LvglUi::updateSnapshot()` setzt alle 155 Flow-Globals und schaltet Screens; die sichtbare Darstellung wird aber weiterhin durch handgeschriebene LVGL-Objekte, direkte Label-/Style-Updates und leeres `styles.c` bestimmt |
| LVGL-Version | 8.x |
| Display | ILI9488, 480 x 320, SPI, Partial-Flush mit Ping-Pong-Draw-Buffern (80 Zeilen) |
| Touch | XPT2046, kalibriert ueber `TouchCalibration` in main.cpp |
| UI-Datenpfad | `SystemSnapshot` -> `UiPresenter`/Formatter -> `LvglUi::updateSnapshot()` -> EEZ-Flow-Globals plus direkte LVGL-Handle-Updates |
| LVGL-Speicher | `LV_MEM_CUSTOM` nutzt `lvgl_psram_alloc.h` und damit den Teensy-PSRAM; Draw-Buffer bleiben in `DMAMEM` |
| Setup-Workflow | produktiv vorhanden; persistiert in `/systemsettings.bin` |
| PAPER-Workflow | produktiv vorhanden; `SELECT` lokal navigierbar, `CAL` als staged Kernfeld-Editor mit Apply/Discard |
| BW-Workflow | Shell, keine eigenen Runtime-Felder |
| Exposure | vollstaendig in `ExposureRuntimeState` |
| Messung | vollstaendig in `MeasurementRuntimeStatus` |
| Gateway/ESP | vollstaendig in `EspLinkRuntimeStatus` |
| RAM1 (0.2.47-dev) | 98 656 B frei; RAM1-Audit Schritte 1–4 abgeschlossen |

Audit-Nachtrag 2026-05-08:

- Das EEZ-Projekt bildet den gewuenschten Seitenumfang inzwischen designseitig
  ab. Es ist aber noch nicht die produktive Quelle fuer Layout, Styles,
  Touch-Flaechen oder Seitenuebergaenge im geflashten UI.
- Die handgeschriebenen `screens.c`/`screens.h` sind der aktuelle produktive
  Runtime-Stand und damit Referenz fuer den Ist-Zustand. Sie sind nicht das
  Zielbild fuer die finale UI-Erarbeitung.
- Zielrichtung ab diesem Stand: neue visuelle Details, finale Darstellung,
  Seitenuebergaenge, Style-Zustaende und Touch-Felder werden in EEZ Studio
  erarbeitet. Firmware-Code bleibt Besitzer von Snapshot, Presenter,
  Sicherheitslogik, Eingaberouting und fachlicher Modusentscheidung.
- Landminen vor weiterer Detailarbeit: `PageWirelessRemote` ist runtime-
  seitig nicht erreichbar; mehrere Touch-Elemente wirken klickbar, besitzen aber
  keine Actions; Messwertformatierung liegt noch teilweise in `LvglUi.cpp`;
  Farbwerte sind zwischen Doku, EEZ-Projekt und C/C++ dupliziert (verbindlich
  dokumentiert in Abschnitt 4, Prompt A4 bereinigt).
  ~~EEZ- und Runtime-Workflow-Familien verwenden noch unterschiedliche Nummern~~
  **Erledigt (Prompt A1).**
- PageMeasurement darf nicht dauerhaft eigene Messwerttexte per `snprintf`
  pflegen; diese Texte muessen auf `UiPresenter`/`MeasurementValueFormatter`
  zurueckgefuehrt werden.
- C++-Farbsetzungen und Magic-Hexwerte bleiben Uebergangs-/Bring-up-Fallbacks.
  Fuer den finalen Produktstand gelten die Palette und Zustandszuordnungen aus
  Abschnitt 4, umgesetzt in EEZ Studio.

---

## 2. Architekturgrenze

### 2.1 Was EEZ darf

- LVGL-Screens und Widgets anlegen
- `lv_obj_t*`-Handles aus dem generierten Code exponieren
- Lokale UI-Zustands-Flags fuer Animations- oder Sichtbarkeitslogik halten

### 2.2 Was EEZ ausdruecklich nicht darf

- GPIOs lesen oder Hardwareobjekte anfassen
- Exposure-, EV-, Dosis-, Thermik- oder Safety-Logik selbst rechnen
- Messwerte von `float` in Strings umwandeln (das bleibt `UiPresenter`)
- eine zweite parallele Wahrheit neben `SystemSnapshot` erzeugen
- Encoder- oder Touch-Routing selbst entscheiden
- Fachseiten-spezifische Input-Guards implementieren

### 2.3 Grenzpunkt

```text
SystemSnapshot  --(buildSystemSnapshot())--> main.cpp
                --(updateSnapshot())--> LvglUi
                                      |
                                      +-- UiPresenter/Formatter fuer Texte
                                      |
                                      +-- EEZ Flow Variablen und EEZ-Objekte
```

`LvglUi` bleibt Besitzer von Display-Init, Tick, Flush und Snapshot-Update.
EEZ wird Zielbesitzer fuer Layout, Styles, Komponenten, Touch-Felder,
Animationen und Seitenuebergangsdefinitionen. `LvglUi` fuellt Daten und ruft
nur die dafuer vereinbarte EEZ-Schnittstelle auf.

Uebergangsregel:

- Bis der EEZ-Exportpfad belastbar ist, bleibt `screens.c` die produktive
  Laufzeitbasis und das EEZ-Projekt muss den sichtbaren Ist-Stand korrekt
  spiegeln.
- Neue UI-Entscheidungen werden zuerst im EEZ-Projekt festgelegt und danach in
  den Runtime-Glue uebernommen; neue manuelle Layout-Arbeit in `screens.c` ist
  nur als befristeter Bring-up-Schritt zulaessig.
- Generated/handgeschriebene Grenzen muessen pro Slice klar sein: Entweder ist
  eine Seite noch handgeschriebene Runtime mit EEZ-Spiegel, oder sie ist EEZ-
  gefuehrte Runtime. Mischzustaende ohne explizites Abnahmekriterium sind nicht
  erlaubt.

---

## 3. Display-Geometrie und Zonen (480 x 320)

Das Display ist Querformat. Die folgenden Zonen sind verbindliche Vorgaben fuer
alle Seiten. Kein Widget darf dauerhaft ausserhalb seiner Zone liegen.

```text
+--------------------------------------------------+ y=0
| HEADER-ZONE (H=48px)                            |
| Links: Seitenname/Modus-Label                    |
| Mitte: Meldungsband (`overlayText`)              |
| Rechts: nur essenzielle Runtime-Hinweise         |
+--------------------------------------------------+ y=48
| HAUPT-ZONE (H=236px)                            |
| Groesste relevante Information der aktiven Seite |
| (Exposure-Zahl, SG-Targets, Messfeld, ...)       |
+--------------------------------------------------+ y=284
| MODETAB-ZONE (H=36px, gesamte Unterkante)       |
| PAPER | MEAS | PRINT | SETUP                     |
+--------------------------------------------------+ y=320
```

Regeln:

- Eine eigene OVERLAY-ZONE oder STATUS-ZONE gibt es im Produktlayout nicht
  mehr. Die historischen Variablen `overlayText` und `overlayColor` bleiben aus
  Glue-Gruenden erhalten, werden visuell aber in das Meldungsband der
  HEADER-ZONE integriert.
- Die HEADER-ZONE ist immer vorhanden und traegt sowohl Moduslabel als auch
  Meldungen, Dirty-/Fault-Hinweise und kurze Bestätigungsaufforderungen.
- Die HAUPT-ZONE entscheidet die fachliche Lesbarkeit bei Dunkelkammer-Licht.
- Die MODETAB-ZONE ist immer sichtbar und darf keine Alarm- oder Statusmeldungen
  tragen; sie dient ausschliesslich der Orientierung ueber den aktiven
  Modus/Page-Kontext.
- Die Reiter in der MODETAB-ZONE bilden Workflow-Familien ab, nicht jede
  einzelne Fachfunktion und auch nicht 1:1 die aktuellen `ModeId`-Werte.
- Permanente globale Redundanzen entfallen. Ein staendiger Link-/Wireless-Chip
  im Header ist nicht vorgesehen; degradierter Linkzustand erscheint als
  Meldung im Header oder auf der Remote-Diagnoseseite.
- Dauerhafte Reserven fuer AHT20/BMP280 in einem separaten unteren Bereich
  entfallen. Diese Werte werden
  nur noch dort gezeigt, wo sie fachlich helfen und keine wichtigere Information
  verdraengen.

---

## 4. Farbkonvention und Zustandscodierung

Die Farben sind hier normativ definiert und werden primaer in EEZ Studio
(Styles, Themes, Flow-Bindings) umgesetzt, damit das UI visuell angepasst
werden kann. `LvglUi` liefert dafuer die Zustaende und Texte; ein zusaetzlicher
Color-Hinweis (`global_overlayColor`) bleibt als optionaler Runtime-Glue erhalten.

**Farb-Ownership (Prompt A4 bereinigt):**

| Schicht | Status | Dateien |
| --- | --- | --- |
| **EEZ-Projekt** (19 Farben in `colors[]`) | **Zielvertrag** — normative Quelle fuer alle `C_BG`, `C_TEXT*`, `C_MSG*`, `C_TAB*` | `DukatimerPart2TeensyUi.eez-project` |
| **`screens.c` `#define`-Block** | Runtime-Fallback — Werte identisch mit EEZ; 3 Tab-Namen abweichend (`C_TAB_INACT_BG/FG` vs. EEZ `C_TAB_IDLE_BG/TEXT`); 9 zusaetzliche Button-/Struktur-Defines ohne EEZ-Pendant | `screens.c` Zeile 27 ff. |
| **`LvglUi.cpp` `kOverlayColor*`** | Runtime-Fallback — 9 benannte Konstanten, Werte identisch mit EEZ `C_MSG_*`; werden bis zur vollstaendigen EEZ-Flow-Migration benoetigt | `LvglUi.cpp` Zeile 24 ff. |
| **Singleton-Hex in `screens.c`** | Runtime-Fallback — bleiben inline, da 1× verwendet: `0x0E0505` (Paper-Stripbg), `0x9A8080` / `0x7A6868` (Paper-Zeilentexte), `0x0D0606` (Setup-Editbg), `0x1A0808` (Balken-Track), `0x1A0A0A` (Histogramm-Spaltenbg) | `screens.c` |

**Namens-Mapping EEZ → `screens.c`** (gleiche Werte, andere Namen):

| EEZ-Name | `screens.c`-Name | Hex-Wert |
| --- | --- | --- |
| `C_TAB_ACTIVE_TEXT` | `C_TAB_ACTIVE_FG` | `#F1E7E0` |
| `C_TAB_IDLE_BG` | `C_TAB_INACT_BG` | `#0B0404` |
| `C_TAB_IDLE_TEXT` | `C_TAB_INACT_FG` | `#8A6A64` |

### 4.1 Hintergrundfarben des Header-Meldungsbands (historisch `overlayColor`)

Diese Farben sind der EEZ-Zielvertrag (`C_MSG_*`). Bis zur vollstaendigen EEZ-Flow-Migration sind sie identisch in `LvglUi.cpp` (`kOverlayColor*`) gespiegelt.

| Zustand | EEZ-Name | Hex-Farbe | `LvglUi.cpp`-Konstante | Bedeutung |
| --- | --- | --- | --- | --- |
| Normal | `C_MSG_NORMAL` | `0x120606` | `kOverlayColorNormal` | ruhiger Grundzustand, nahezu schwarz mit warmer Rottoenung |
| WorkflowFault | `C_MSG_FAULT` | `0x3A0506` | `kOverlayColorWorkflowFault` | kritischer Fehler / Exposure gestoppt |
| WorkflowConfirm | `C_MSG_CONFIRM` | `0x2A100A` | `kOverlayColorWorkflowConfirm` | Benutzer muss aktiv bestaetigen |
| WorkflowWait | `C_MSG_WAIT` | `0x1C0B0B` | `kOverlayColorWorkflowWait` | System wartet auf externe Bedingung |
| Setup persistFailed | `C_MSG_FAULT` | `0x3A0506` | `kOverlayColorWorkflowFault` | Speichern fehlgeschlagen (identisch mit Fault) |
| Setup parametersDirty | `C_MSG_CONFIRM` | `0x2A100A` | `kOverlayColorWorkflowConfirm` | nicht gespeicherte Aenderungen |
| Sensor fallback aktiv | `C_MSG_FALLBACK` | `0x34120C` | `kOverlayColorSensorFallback` | Dose-Betrieb ohne geschlossenen Regelkreis |
| SG Completed | `C_MSG_COMPLETED` | `0x1B100C` | `kOverlayColorSgCompleted` | Belichtung erfolgreich beendet |
| SG Aborted | `C_MSG_ABORTED` | `0x241112` | `kOverlayColorSgAborted` | Belichtung abgebrochen |
| Link stale/lost | `C_MSG_LINK_LOST` | `0x2D120E` | `kOverlayColorLinkDegraded` | ESP-Verbindung unterbrochen |
| SG parametersDirty | `C_MSG_SG_DIRTY` | `0x1F0C0C` | `kOverlayColorSgDirty` | ungespeicherte SG-Parameter |

### 4.2 Textfarben (persistent, kein Override erlaubt)

| Zweck | EEZ-Name | Hex-Farbe | `screens.c`-Define |
| --- | --- | --- | --- |
| Standardtext | `C_TEXT` | `0xF1E7E0` | `C_TEXT` |
| Subtitel / sekundaer | `C_TEXT_SEC` | `0xB8948A` | `C_TEXT_SEC` |
| SG-Header / OK-Status | `C_TEXT_SG` | `0xD6B4AA` | — (nicht in screens.c) |
| Warnung / Pending | `C_WARN` | `0xE0A06E` | `C_WARN` |
| Meldungstext | `C_MSG` | `0xF5EDE8` | — (nicht in screens.c) |
| Hintergrund | `C_BG` | `0x050202` | `C_BG` |

Farbaenderungen fuer nicht aufgefuehrte Faelle muessen in `offene_entscheidungen.md`
begruendet werden.

### 4.2a Aktions-Button-Farben (Diagnose-Ausnahme)

Gruen, Rot und Amber werden ausschliesslich fuer Aktion-Buttons (Apply/Fortsetzen,
Discard/Abbrechen, SafetyDefaults/Pause) verwendet. Diese Farben sind **nicht** in
der EEZ-Palette und gelten als befristete Diagnose-Ausnahme bis zur EEZ-Style-Migration.
Auf keinen Fall in Produktfarben ausweiten.

| Semantik | BG | Border | Text | `screens.c`-Defines |
| --- | --- | --- | --- | --- |
| Apply / Fortsetzen | `0x0D2E18` | `0x1A5C30` | `0x7FD4A0` | `C_BTN_CONFIRM_*` |
| Discard / Abbrechen | `0x2E0D0D` | `0x5C1A1A` | `0xD47F7F` | `C_BTN_DISCARD_*` |
| SafetyDefaults / Pause | `0x2F2610` | `0x7A6730` | `0xE5D08A` | `C_BTN_SAFETY_*` |

### 4.2b Strukturelle Runtime-Farben (kein EEZ-Pendant)

Diese Farben werden nur in handgeschriebenen Screens verwendet und haben kein
EEZ-Projektpendant. Sie sind Runtime-Fallback bis zur vollstaendigen EEZ-Migration.

| Zweck | Hex-Wert | `screens.c`-Define |
| --- | --- | --- |
| Header-/Abschnittstrennlinie | `0x2A1010` | `C_SECTION_BORDER` |
| Innere Panel-/Block-Rahmen | `0x1C0808` | `C_PANEL_BORDER` |
| Quell-/Diag-Block-Hintergrund | `0x0D0505` | `C_BLOCK_BG` |
| Fortschrittsbalken / Histogramm-Fuell | `0xC05030` | `C_ACCENT_PROGRESS` |

Zusatzregel fuer die Produkt-UI:

- Gruen- und Blautendenzen sind in der Dunkelkammer-Produktoberflaeche zu
  vermeiden. Ausnahmen gelten nur fuer explizite Diagnose- oder Entwicklerseiten.

### 4.3 EEZ-Color-Workflow-Checkliste (Teamstandard)

Diese Checkliste ist fuer jede UI-Farbanpassung verpflichtend, damit alle
Screens konsistent bleiben und visuelles Tuning direkt in EEZ erfolgt.

1. Palette aus Abschnitt 4.1/4.2 als Quelle verwenden.
2. In EEZ unter Project -> Colors alle benoetigten Farben als benannte Eintraege
  pflegen (keine neuen Magic-Hexwerte direkt in Widgets).
3. In EEZ unter Themes -> Default wiederverwendbare Grundstile pflegen
  (mindestens Textfarben fuer Standard, Subtitel, Warnung, Meldung).
4. Komponentenfarben lokal nur dort setzen, wo ein bewusstes Override noetig ist
  (zum Beispiel CompMessageBand fuer Header-Meldungen).
5. CompMessageBand in allen Seiten gleich halten: Label-Text bleibt an
  global_overlayText gebunden, Label-Farbe bleibt Meldungstext, und der
  Container-Hintergrund wird ueber EEZ-Flow oder Style-Condition auf die
  Zustaende aus Abschnitt 4.1 gemappt.
6. Flow-Editor: Zustandsbasierte Farbumschaltung fuer das Meldungsband an einer
  zentralen Stelle pflegen (nicht pro Screen unterschiedlich).
7. Nach jeder Farb-Aenderung Sichtpruefung auf allen 6 Seiten durchfuehren
  (Boot, Paper, Splitgrade, Setup, Measurement, WirelessRemote).
8. Danach Build-Check ausfuehren (`pio run -e teensy41`) und Aenderung in
  offene_entscheidungen.md nur dann eintragen, wenn neue Farben oder neue
  Zustand-zu-Farbe-Zuordnungen eingefuehrt wurden.

Kurzregel fuer Ownership:

- Doku definiert Palette und Zustandscodierung.
- EEZ Studio setzt die produktive Darstellung um.
- LvglUi liefert fachliche Zustaende und Texte; C++-Farbsetzung ist nur
  Fallback und darf EEZ-Styling nicht still ueberschreiben.

---

## 5. Globales Eingabemodell (Blindbedienbarkeit)

Die physische Encoder-Zuordnung ist verbindlich dreigeteilt:

- Enc1 bis Enc3 sitzen lokal am Teensy und sind die primaeren Bedienelemente der
  Haupt-UI.
- Enc4 sitzt am ESP32-S3-Serviceboard und ist der entfernte Kontext-/Menueencoder
  fuer nicht-belichtungskritische Parameter und Confirm-Aktionen.
- Der C6 besitzt einen eigenen Wireless-Encoder fuer das Handbedienteil. Dieser
  Encoder bedient den Mess- und Auswahlkontext des Handteils (zum Beispiel Zone,
  Messbereich, Spot-/Mehrpunktwahl) und ist nicht mit Enc4 gleichzusetzen.

### 5.1 Encoder-Rollen je Modus

| Encoder | PAPER | Splitgrade | BW | Messung | Setup | Overlay-aktiv |
| --- | --- | --- | --- | --- | --- | --- |
| Enc1 (Teensy, lokal) | Slotwahl fein; im CAL-Edit Feldwert fein | SG-Hauptwert (Grade oder Target) | BW-Hauptwert | nicht belegt | Wert am Cursor anpassen | gesperrt |
| Enc2 (Teensy, lokal) | Slotwahl grob; im CAL-Edit Feldwert grob | Unterparameter (Soft/Hard-Wechsel) | Unterparameter | nicht belegt | kein Effekt (reserviert) | gesperrt |
| Enc3 (Teensy, lokal) | `SELECT/CAL`, im CAL Itemwahl | Panel-Navigation / Kontextaktion | Navigation | nicht belegt | Cursor durch Items | gesperrt |
| Enc3 LongPress | Einstieg in Setup | **Einstieg in Setup** | Einstieg in Setup | Einstieg in Setup | **Verlassen von Setup** | - |
| Enc4 (ESP32-S3, Service-Encoder) | Kontextnavigation, Confirm fuer Slot sowie `Apply/Discard` | Kontextnavigation, Confirm; keine primären Belichtungsparameter | Kontextnavigation, Confirm; keine primären Belichtungsparameter | Menue-/Kontextnavigation auf dem Teensy; nicht fuer Zonen- oder Messbereichswahl des Handteils | Confirm/Discard auf fokussierten Aktionseintraegen | - |
| EncC6 (ESP32-C6, Wireless-Handencoder) | keine direkte PAPER-Rolle im Teensy-UI | keine direkte SG-Hauptseitenrolle im Teensy-UI | keine direkte BW-Hauptseitenrolle im Teensy-UI | Zonenwahl, Messbereich, Spot-/Mehrpunktkontext am Handbedienteil | keine direkte Setup-Rolle im Teensy-UI | - |
| Start-Taste | kein Effekt | Exposure starten/stoppen | Exposure starten | Messung ausloesen | kein Effekt | Modal bestaetigen |

Regeln:

- Kein Encoder hat je nach Seite eine entgegengesetzte Wirkrichtung.
- Enc3 ist immer die lokale Navigationsachse des Teensy-UI; Enc1 ist immer die
  lokale Wertachse.
- Solange die produktiven EEZ-ModeTabs noch fehlen, nutzt der Placeholder einen
  bewusst provisorischen Familienwechsel: Enc1-Press springt nach `PAPER`,
  Enc2-Press zurueck in den letzten `PRINT`-Kontext.
- Enc3 LongPress ist der einzige Einstieg in Setup aus jedem Hauptmodus.
- Enc4 am ESP32-S3 ist fuer Kontextnavigation, Menuefokus und Confirm reserviert.
  Er aendert keine primaeren Belichtungsparameter.
- Der C6-Encoder ist dem Handbedienteil und dessen Messkontext zugeordnet. Er ist
  kein Ersatz fuer Enc4 und uebernimmt nicht die lokale EEZ-Seitennavigation des
  Teensy.
- `InputNormalizer` ordnet Enc3, Enc4 und C6-Encoder derzeit derselben
  Kontext-/Confirm-Semantikfamilie zu; die UI-Beschriftung und Dokumentation
  muessen die physische Quelle trotzdem korrekt trennen.
- Im Overlay-Zustand `WorkflowFault` oder `WorkflowConfirm` werden Enc1 und Enc2
  gesperrt; nur Enc3 Press / Start-Taste loest die Bestaetigung aus.
- Kein Encoder darf Touch-Eingaben uebersteuern oder umgekehrt, wenn ein
  Modal-Guard aktiv ist.

### 5.2 Touch-Regeln

- Touch ist ergaenzend, nicht ersetzend fuer Encoder.
- Jede Aktion, die per Touch ausloesbar ist, muss auch blind per Encoder
  erreichbar sein.
- In der Dunkelkammer darf kein produktiver Workflow nur per Touch bedienbar sein.
- Touch-Zonen duerfen nicht kleiner als 48x48 px sein.
- Aktive Touch-Zone wird im Meldungsband der HEADER-ZONE kurz quittiert
  (`global_touchActive`-Flag setzt `overlayText` fuer einen Tick).

### 5.3 Navigationsmodell zwischen Seiten

```text
Boot
 |
 +-- [Auto] --> PageSplitgrade (Defaultseite innerhalb PRINT)
      |
      +-- [ModeTab PAPER] --> PagePaperWorkspace
      |        |
      |        +-- [lokale Unterstruktur] --> SELECT / CAL
      |
      +-- [ModeTab MEAS] --> PageMeasurement
      |        |
      |        +-- [lokale Unterstruktur] --> ZONE / DENS
      |
      +-- [ModeTab PRINT] --> letzte Druckseite (`PageSplitgrade` oder `PageBlackWhite`)
      |        |
      |        +-- [lokale Unterstruktur] --> SG / BW
      |        +-- [Kontextaktionen] --> PREFLASH / TEST / BURN
      |
      +-- [Enc3 LongPress oder ModeTab SETUP] --> PageSetup
      |        |
      |        +-- [Enc3 LongPress] --> zurueck zu vorheriger Seite
      |
      +-- [Service-Geste oder Service-Menue] --> PageWirelessRemote
      |
      +-- [Fault-/Confirm-Zustand] --> bleibt auf aktueller Seite, Meldungsband im Header uebernimmt die Warnung
```

Regeln:

- Es gibt keinen globalen Screen-Stack. Der ModeCoordinator entscheidet den
  aktiven Modus.
- Die MODETAB-Reiter sind Familienwechsler: `PAPER`, `MEAS`, `PRINT`, `SETUP`.
  Die konkrete Fachseite innerhalb einer Familie wird lokal ueber Unterreiter,
  Panels oder Kontextaktionen gewechselt.
- Der Start nach Boot bleibt bis zur spaeteren Persistenz des zuletzt genutzten
  Druckkontexts auf `PageSplitgrade`; fachlich gehoert diese Seite aber bereits
  zur Familie `PRINT`.
- Bis echte EEZ-ModeTabs existieren, bildet der aktuelle Placeholder den
  Familienwechsel lokal ueber Encodertaster ab: Enc1-Press -> `PAPER`,
  Enc2-Press -> Rueckkehr in den letzten `PRINT`-Modus.
- `PageSetup` ist kein normaler Modus im ModeCoordinator-Sinn, sondern ein
  Overlay-Modus. Der vorherige Modus bleibt als Ruecksprungziel in
  `setupReturnMode` (bereits in `main.cpp` vorhanden).
- `PageMeasurement` und spaetere Densitometrie-Seiten sind Vorbereitungs- und
  Analysepfade. Sie loesen keine Belichtung aus, sondern liefern Vorschlaege an
  `PAPER` und `PRINT` zurueck.
- `SCREEN_ID_BUSY` ist im aktuellen Code die belichtungsaktive
  Sicherheitsdarstellung fuer `PreWait`, `Exposing`, `Paused` und `PostWait`.
  Er ist kein normaler Navigationsscreen und kein Workflowmodus, sondern eine
  priorisierte Runtime-Darstellung; Rueckkehr und Zielseite ergeben sich danach
  wieder aus `computeTargetScreen()` und dem aktuellen `SystemSnapshot`.
- `PageBootStatus` ist nur waehrend des Bootvorgangs aktiv. Nach erfolgreicher
  Initialisierung wechselt die UI automatisch in `PageSplitgrade`.
- `PageWirelessRemote` ist kein eigenstaendiger Modus, sondern eine Diagnosesicht.
  Zielbild: erreichbar per spezieller Touch-Geste oder spaeterem Menuepunkt,
  verlassbar per Enc3-Press, kein eigener Reiter in der MODETAB-ZONE. Ist-Stand:
  die Seite und ihre Runtime-Daten existieren, `computeTargetScreen()` routet
  aber noch nie dorthin.

Aktuelle Navigationslandminen:

- Die unteren ModeTabs sind heute sichtbare Orientierung, aber noch kein
  vollstaendig verdrahteter Touch-Familienwechsler. **Touch-Kontrakt
  dokumentiert (Prompt A6):** Abschnitt 12 definiert den Signalpfad,
  Implementierungsmuster und offene Hooks fuer alle vier Tabs.
- ~~Die EEZ-Conditions fuer `global_activeWorkflowFamily` verwenden aktuell eine
  andere Nummerierung als `LvglUi::computeWorkflowFamily()`.~~
  **Erledigt (Prompt A1):** `computeWorkflowFamily()` liefert jetzt `0=PAPER`,
  `1=MEAS`, `2=PRINT`, `3=SETUP`; EEZ-Conditions und ModeTabs-Vergleiche
  wurden synchronisiert. Build erfolgreich.
- Setup-Aktionsbuttons sind im handgeschriebenen Layout klickbar markiert, aber
  die produktive Wirkung muss ueber Firmware-Guards und Actions noch eindeutig
  verdrahtet werden. **Touch-Kontrakt dokumentiert (Prompt A6):** Abschnitt
  12.3 definiert den Signalpfad und Implementierungsmuster; Callbacks fehlen
  noch.

### 5.4 Fachliche Hauptfunktionen und logische Verortung

Die Produkt-UI darf den fotografischen Arbeitsablauf nicht als lose Liste
gleichrangiger Screens behandeln. Die Hauptfunktionen fallen in vier Familien,
die sich in Nutzungshaeufigkeit, Ruecksprungverhalten und Datenbesitz klar
unterscheiden.

#### 5.4.1 Empfohlene Familien fuer die MODETAB-ZONE

- `PAPER`: Papierauswahl und seltene Papierkalibrierung
- `MEAS`: Zonenvisualisierung, Messsession und spaeter Densitometrie
- `PRINT`: BW/SG-Kernfunktion plus Preflash, Teststrip, Zeit/Dosis und Burn
- `SETUP`: globale Systemkonfiguration, keine fotografischen Fachworkflows

#### 5.4.2 Verortung der einzelnen Hauptfunktionen

| Hauptfunktion | Nutzung | UI-Familie | Primaere Verortung | Wechsel- und Ruecksprungregel |
| --- | --- | --- | --- | --- |
| Papierkalibrierung | initial wichtig, spaeter sporadisch | `PAPER` | `PagePaperWorkspace` Unteransicht `CAL`; heute staged Kernfeld-Editor, spaeter Einstieg in den messgetriebenen Wizard | Start immer aus der Papierseite; Ruecksprung auf denselben Slot, nie ueber `SETUP` |
| Zonenvisualisierung | haeufig vor Druckentscheidung | `MEAS` | `PageMeasurement` Unteransicht `ZONE` | Aufruf ueber Reiter `MEAS`; Ergebnis erzeugt nur Kontext/Vorschlaege |
| Papierauswahl | vor Gradation, Teststrip und finalem Druck | `PAPER` | `PagePaperWorkspace` Unteransicht `SELECT` | Aktiver Slot wird als Zusammenfassung in `PRINT` mitgefuehrt |
| Gradationsauswahl | zentraler Druckschritt | `PRINT` | `PageSplitgrade` oder MG-Teil von `PageBlackWhite`; bei fixed-grade-Papier aus dem aktiven Profil abgeleitet und sichtbar gesperrt | Kein eigener Hauptreiter; erfolgt im aktuellen Druckkontext |
| Preflash | nur sinnvoll im Druckkontext | `PRINT` | Kontextpanel oder Aktionsblatt der aktuellen PRINT-Seite | Erbt aktives Papier und kehrt in dieselbe Druckseite zurueck |
| Teststrip | Entscheidungshelfer vor der finalen Belichtung | `PRINT` | Kontextassistent aus der aktuellen PRINT-Seite | Liefert Vorschlaege zurueck, ohne die Familie `PRINT` zu verlassen |
| BW/SG Kernfunktion | haeufigste Produktivfunktion | `PRINT` | `PageSplitgrade` / `PageBlackWhite` | Unterstruktur `SG/BW`, gemeinsamer Familienreiter `PRINT` |
| Zeit/Dosis-Umschalter | pro Druckentscheidung relevant | `PRINT` | lokaler Chip bzw. Panel innerhalb von SG/BW | Kein Familienwechsel, keine eigene Seite |
| Burn | optionaler Abschlussschritt nach Basisbelichtung | `PRINT` | spaeter `PageBurn` oder kontextuelles Tool aus der aktiven PRINT-Seite | Nur aus bestehendem Druckkontext erreichbar, nicht als globaler Hauptreiter |
| Durchlicht-Densitometrie | haeufig separater Negativ-Workflow | `MEAS` | spaetere Unteransicht `DENS` innerhalb `MEAS` | Darf Papier-/Gradationsvorschlaege liefern, aber nie ungefragt anwenden |

#### 5.4.3 Abgeleitete UI-Regeln aus dem Arbeitsablauf

- Alles, was nur im Zusammenhang mit einer konkreten Druckentscheidung sinnvoll
  ist, bleibt in `PRINT`. Das betrifft Preflash, Teststrip, Zeit/Dosis und Burn.
- Alles, was vor Belichtung analysiert oder bewertet, aber keine Belichtung
  selbst ausloest, bleibt in `MEAS`. Das betrifft Zonenvisualisierung,
  Messsession und spaetere Densitometrie.
- Alles, was einen Papier-Slot oder dessen Kalibrierstatus veraendert, bleibt in
  `PAPER`. `SETUP` bleibt davon frei.
- Das gewaehlte Papierprofil definiert fuer `PRINT`, ob der BW-Pfad als
  `fixed grade` oder `multigrade` arbeitet. `PRINT` darf daraus UI-Sperren und
  sichtbare Gradationsbedienung ableiten, aber keine zweite globale BW-Wahrheit
  erfinden.
- `PAPER`, `MEAS` und `PRINT` duerfen sich wechselseitig Vorschlaege zeigen,
  aber keine stillen Auto-Overwrites durchfuehren. Vorschlaege sind immer als
  uebernehmbare, sichtbare Empfehlung zu kennzeichnen.
- Zweiteilige Navigation ist ausdruecklich erlaubt: Familienwechsel unten,
  Fachwechsel innerhalb der Familie oben oder im Kontextblock.

---

## 6. Schritt-fuer-Schritt Implementierungsplan

### 6.0 Umsetzungsstand (2026-05-08)

| Schritt | Beschreibung | Status |
| --- | --- | --- |
| 1 | EEZ-Projekt anlegen | Abgeschlossen — 7 Pages, valides JSON |
| 2 | Globale Variablen + Farbkonstanten | Designseitig abgeschlossen — 155 Variablen + 19 benannte Farben; **Farb-Ownership bereinigt (Prompt A4)**: EEZ = Zielvertrag, `screens.c` und `LvglUi.cpp` = Runtime-Fallback; Magic-Hex in `screens.c` durch benannte Defines ersetzt |
| 2 (Styles) | `lvglStyles` — 19 Styles (11 Grundstile + 8 MessageBand-Zustandsstile) | Designseitig angelegt; `styles.c` ist runtime-seitig noch leer und erzeugt keine sichtbare Wirkung |
| 3 | `CompMessageBand` (UserWidget) | Designseitig angelegt; produktive Runtime nutzt weiter handgeschriebene Header-/Message-Handles |
| 4 | `CompHeader` + `CompModeTabs` (UserWidgets) | Designseitig angelegt; **Workflow-Family-Nummerierung normalisiert (Prompt A1 erledigt)** — `computeWorkflowFamily()` und EEZ-Conditions stimmen ueberein |
| 4a | `PagePaperWorkspace` | Designseitiger Spiegel vorhanden; produktive Darstellung kommt aus `create_screen_page_paper_workspace()` und direkten Widget-Updates |
| 5 | `PageSplitgrade` | Designseitiger Spiegel vorhanden; produktive Darstellung kommt aus `create_screen_page_splitgrade()` und direkten Widget-Updates |
| 6 | `PageSetup` | Designseitiger Spiegel vorhanden; produktive Darstellung kommt aus `create_screen_page_setup()`; **Touch-Kontrakt fuer Apply/Discard/SafetyDefaults dokumentiert (Prompt A6)** — Callbacks noch nicht implementiert, Signalpfad in Abschnitt 12.3 festgelegt |
| 7 | `Busy`-Screen | Runtime funktional handgeschrieben; EEZ-Spiegel vorhanden; Busy bleibt sicherheitskritischer Sonderfall mit eigenem Abnahmegate |
| 8 | `PageMeasurement` | Designseitiger Spiegel vorhanden; Runtime formatiert Messwerte noch teilweise in `LvglUi.cpp` und muss auf Presenter/Formatter zurueckgefuehrt werden |
| 9 | `PageWirelessRemote` | Designseitiger Spiegel und Runtime-Daten vorhanden; Seite ist aktuell nicht erreichbar, weil kein Runtime-Routing dorthin fuehrt |
| 10 | `PageBootStatus` (Boot) | Designseitiger Spiegel vorhanden; Runtime-Bootscreen existiert handgeschrieben und wird fuer `ModeId::None` genutzt |
| 11 | Glue-Code in `LvglUi` | Teiloffen — `ui_init()`, Flow-Globals und `eez_flow_set_screen()` existieren; sichtbare UI wird aber weiter direkt ueber `g_duka_widgets` aktualisiert; generierte `ui_set_*`-Setter existieren nicht |
| 12 | Validierung nach jedem Slice | Laufend — Problems-Ansicht fuer die geprueften UI-Kernfiles unauffaellig; ein verlaesslicher `pio run -e teensy41`-Abschluss ist fuer diesen Auditstand noch nachzuholen |

Statusdefinition:

- **Designseitig abgeschlossen** bedeutet: Das EEZ-Projekt enthaelt die
  benoetigten Objekte als aktuelle Arbeitsgrundlage fuer EEZ Studio.
- **Runtime abgeschlossen** bedeutet erst: Die geflashte Teensy-UI wird aus dem
  EEZ-Export oder einer klar definierten EEZ-Schnittstelle gespeist, baut ohne
  neue Warnungen und besteht die Hardware-/Bedienpruefung.
- Der aktuelle Stand ist damit kein Endzustand, sondern ein konsolidierter
  Zwischenstand: EEZ spiegelt den UI-Umfang, die produktive Runtime ist noch
  handgeschrieben.

**Naechster Schritt:** Konsolidierung A — Landminen und Inkonsistenzen
bereinigen, bevor finale Darstellung, Seitenuebergaenge und Touch-Felder in EEZ
verfeinert werden.

### 6.1 Fahrplan zur EEZ-gefuehrten Produkt-UI

Ziel: EEZ Studio wird die Arbeits- und Darstellungsquelle fuer Layout, Styles,
Komponenten, Touch-Felder und Seitenuebergaenge. Firmware-Code bleibt die Quelle
fuer Snapshot, Presenter, Eingaberouting, Safety, Exposure und fachliche
Modusentscheidung.

#### Phase A: Landminen bereinigen und Verträge festziehen

Diese Phase kommt vor jeder weiteren visuellen Detailarbeit in EEZ.

1. UI-Familienenum festlegen und normalisieren:
  `PAPER=0`, `MEAS=1`, `PRINT=2`, `SETUP=3`. Diese Werte gelten fuer
  `global_activeWorkflowFamily`, EEZ-Conditional-Styles und
  `LvglUi::computeWorkflowFamily()`. `PageWirelessRemote` erbt den zuletzt
  sinnvollen Familienkontext oder markiert `SETUP`, aber bekommt keinen eigenen
  unteren Reiter.
2. Dokumentation und `.eez-project` als Zielvertrag synchronisieren:
  Seitenumfang, Farben, UserWidgets, Style-Namen, Condition-Ausdruecke,
  Feldnamen und vorhandene Runtime-Luecken muessen dieselbe Sprache sprechen.
3. Glue-Schnittstelle festlegen: Bis echte generierte Setter existieren, ist
  `flow::setGlobalVariable(...)` die dokumentierte Schnittstelle. Beispiele mit
  nicht vorhandenen `ui_set_*`-Funktionen sind unzulaessig.
4. Farb-Ownership entkoppeln: Doku definiert Palette, EEZ setzt Styles und
  Conditions um, C/C++ darf Farben nur noch als befristeten Fallback fuer noch
  nicht migrierte Runtime-Seiten halten.
5. Messwerttexte zurueckholen: Benutzerlesbare Lux-/EV-/Alter-/Sessiontexte
  gehoeren in `UiPresenter` oder `MeasurementValueFormatter`. EEZ darf rohe
  Messwerte fuer Diagnose oder einfache Anzeige erhalten, aber keine zweite
  fachliche Formatierlogik besitzen.
6. Touch-Kontrakt klaeren: Jedes Touch-Feld bekommt einen Action-Namen, eine
  Mindestflaeche, eine Encoder-Alternative und einen Firmware-Guard. Klickbare
  Optik ohne Action ist nicht produktionsfaehig.

Abnahme Phase A:

- Doku, EEZ-Conditions und `computeWorkflowFamily()` verwenden dieselbe
  Familiennummerierung.
- `PageWirelessRemote` ist entweder bewusst als noch nicht erreichbar markiert
  oder besitzt einen implementierten Servicepfad.
- Kein Dokumentbeispiel referenziert nicht vorhandene Runtime-APIs.
- Neue visuelle Arbeit findet ab diesem Punkt zuerst in EEZ statt.

#### Phase B: EEZ-Runtime-Bruecke aktivieren

1. **✅ Prompt B0 erledigt:** EEZ-Export- und Merge-Vertrag festgelegt — Welche
  Dateien duerfen generiert werden, welche bleiben handgeschrieben, und wie werden
  lokale Anpassungen vor Ueberschreiben geschuetzt. Siehe Abschnitt 13.
2. `styles.c`, Screen-Erzeugung und Widget-Handles aus EEZ exportieren oder die
  manuelle Runtime eindeutig als temporären Spiegel kennzeichnen.
3. Seite fuer Seite migrieren: Boot/Paper/Splitgrade/Setup/Measurement/Remote,
  Busy zuletzt und nur mit Sicherheitsabnahme.
   - **⏸ Prompt B1 (Boot): BLOCKIERT** — `create_screen_boot()` ist handgeschrieben
     und registriert keine EEZ-Binding-States fuer die 3 Labels. EEZ-Exportpfad
     muss erst via B0-Vertrag aktiviert werden. Zwischenabnahme dokumentiert in
     Abschnitt 13.7. Build SUCCESS 27 s.
   - **⏸ Prompt B2 (PaperWorkspace): BLOCKIERT** — `create_screen_page_paper_workspace()`
     ist handgeschrieben, identischer Blocker wie B1. EEZ-Datenschicht vollstaendig
     (33 Flow-Globals 17-49 geschrieben). Fehlende EEZ-Vars fuer vollen CAL-Pfad:
     `PAPER_STEP_WHITE`, `PAPER_STEP_BLACK`. Abschnitt 13.8. Build SUCCESS 26 s.
   - **⏸ Prompt B4 (Setup): BLOCKIERT** — `create_screen_page_setup()` ist
     handgeschrieben, identischer Blocker wie B1/B2. 19 Setup-Flow-Globals (71-89)
     vollstaendig geschrieben; fehlt: `SETUP_HEAD_TIMING_DIAGNOSTICS_ENABLED`.
     Touch-Hooks fuer Apply/Discard/SafetyDefaults inaktiv (kein LVGL-Callback).
     Action-Vertrag dokumentiert. Abschnitt 13.9. Build SUCCESS 49 s.
   - **⏸ Prompt B5 (Measurement): BLOCKIERT** — `create_screen_page_measurement()`
     ist handgeschrieben, identischer Blocker wie B1/B2/B4. EEZ-Datenschicht
     vollstaendig: 18 Flow-Globals (105-122) alle in `updateSnapshot()` geschrieben.
     Keine fehlenden EEZ-Vars fuer Raw-Daten. Formatierter Text kommt aus
     `UiPresenter`/`MeasurementValueFormatter` (A5 abgeschlossen, kein
     `snprintf()` mehr in LvglUi.cpp fuer Measurement). Histogramm-Geometrie
     (11 Spalten × 2 Handles) nicht durch EEZ-Text-Bindung abdeckbar — erfordert
     Presenter-Aufruf im EEZ-generierten Tick. Kein Touch-Callback.
     Abschnitt 13.10. Build SUCCESS 27 s.
   - **⏸ Prompt B6 (WirelessRemote): GEPARKT** — Seite ist designseitig vollstaendig
     vorhanden (`create_screen_page_wireless_remote()`, `tick`-Stub, LvglUi.cpp-
     Direktpfad mit 9 Handles/snprintf), aber `computeTargetScreen()` gibt
     niemals `SCREEN_ID_PAGE_WIRELESS_REMOTE = 6` zurueck. Kein
     `ModeId::WirelessRemote`, kein Snapshot-Flag, keine Long-Press/Gesture-
     Infrastruktur in `InputRouterPolicy`, kein Debounce/Guard-Konzept.
     EEZ-Datenschicht vorhanden: 24 Flow-Globals (123-146). Runtime-Hook
     definiert fuer spaetere Freischaltung (Abschnitt 13.11). Kein Build
     erforderlich (nur Kommentare/Doku). Abschnitt 13.11.
   - **⏸ Prompt B7 (Busy): BLOCKIERT** — Safety-Gate-Pruefung: 2 von 7 Gates
     nicht erfuellt. Gate 1: kein Pilot (B1-B5 alle BLOCKIERT, B6 GEPARKT) —
     Prerequisite laut Prompt nicht erfuellt. Gate 2: `tick_screen_busy()` ist
     Stub, identischer EEZ-Runtime-Blocker wie B1-B5. Passierte Gates:
     Busy-Header-Isolation (eigene `busy_hdr_*` Handles, kein `build_header()`),
     Pause/Resume/Abort Event-Buffer-Vertrag (`busy_btn_cb` →
     `lvgl_ui_modal_action_callback` → `setPendingModalAction` →
     `pollModalAction()`, ExposureEngine unveraendert), `computeTargetScreen()`
     Busy-Prioritaet (erste Branch). Regression-Build SUCCESS 41.7 s
     (B6+B7 kombiniert, nur Kommentare). Abschnitt 13.12.

   **C1 (Boot, EEZ-Layout):** Boot-Seite im EEZ-Projekt verfeinert (SCROLLABLE-Flag
     entfernt, named styles `StyleLabelMain28`/`StyleLabelTextSec` verdrahtet,
     redundante `localStyles`-Definitionen bereinigt). Kein Runtime-Code geaendert.
     JSON validiert. Abschnitt 13.13.

   **C2 (Seitenuebergaenge):** Alle Uebergaenge auditiert und dokumentiert.
     Kein EEZ-Transitions-Eintrag im `.eez-project`; kein Firmware-Code geaendert.
     Firmware steuert alle Screen-Wechsel via `computeTargetScreen()` +
     `eez_flow_set_screen(... LV_SCR_LOAD_ANIM_NONE ...)`. EEZ-seitige Animationen
     sind wegen Universal-EEZ-Blocker + NONE-Override nicht implementierbar.
     Busy-Prioritaet durch erste Branch in `computeTargetScreen()` garantiert.
     Abschnitt 13.14.

   **C3 (Touch-Felder):** Pressed-Visual fuer 6 CLICKABLE-Buttons (3 Busy + 3 Setup)
     in `screens.c` implementiert; ausschliesslich Palettenfarben (`*_BD`-Variante).
     EEZ-Pressed-States und Header-Feedback blockiert (Universal-EEZ-Blocker +
     Tick-Ueberschreiben). ModeTabs bleiben rein visuell (kein CLICKABLE, 36 px
     Hoehenunterschreitung dokumentiert). Build: SUCCESS 29.07 s. Abschnitt 13.15.

   **V0 (EEZ-BOM-Fix):** UTF-8-BOM (`\xEF\xBB\xBF`) aus `.eez-project` entfernt.
     Node.js `Buffer.toString("utf8")` entfernt den BOM nicht; `JSON.parse("\uFEFF{...}")`
     warf SyntaxError in EEZ Studio 0.27.1. Nach Binary-Strip: EEZ Studio oeffnet die
     Datei korrekt (`projectType: "lvgl"` im MRU bestaetigt). Abschnitt 13.16.
4. `pushWidgetsFromSnapshot()` reduzieren: Direktes `lv_label_set_text*()` und
  direkte Style-Farben verschwinden nur dort, wo die EEZ-Seite wirklich aktiv
  uebernommen ist.
5. Actions anbinden: EEZ-Actions liefern nur Ereignisse; die Wirkung bleibt in
  `InputRouterPolicy`, `ModeCoordinator`, Setup-/Paper-Workflow oder
  Exposure-Safety-Code.

Abnahme Phase B pro Seite:

- EEZ-Projekt, generierter Runtime-Code und geflashte Darstellung zeigen dasselbe
  Layout.
- `pio run -e teensy41` laeuft ohne neue Fehler oder ungeklärte Warnungen.
- Encoder-only-Bedienung bleibt erhalten.
- Touch loest nur Aktionen aus, die auch firmwareseitig erlaubt sind.

#### Phase C: Finale Darstellung in EEZ ausarbeiten

Erst nach Phase A und einem belastbaren Brueckenpfad aus Phase B werden die
Details in EEZ verfeinert:

- finale Typografie, Abstaende, Lesbarkeit bei Dunkelkammerlicht
- konsistente Header-/MessageBand-Zustaende
- ModeTabs und lokale Unterreiter
- Seitenuebergaenge und Busy-Rueckkehr
- Touch-Felder, Fokus-/Pressed-Zustaende und Quittierung im Meldungsband
- Diagnose-/Remote-Seite ohne Vermischung mit normaler Workflow-Navigation

#### Phase D: Produktabnahme

- Build: `pio run -e teensy41` nach jedem substantiellen Slice.
- Hardware: Touch-Kalibrierung, Encoder-only-Durchgang, Exposure-Busy-Screen,
  Setup Apply/Discard, Paper SELECT/CAL, WirelessRemote-Servicepfad.
- Sichtpruefung: Boot, Paper, Splitgrade, Setup, Measurement, WirelessRemote,
  Busy bei gedimmtem Licht.
- Speicher: RAM1/PSRAM-Audit nach EEZ-Export und vor groesseren Font-/Style-
  Aenderungen.

Die folgenden urspruenglichen Schritte bleiben als Detailvertrag erhalten. Sie
werden aber ab jetzt durch den Fahrplan oben interpretiert: Ein Schritt ist erst
produktivreif, wenn Designstand, Runtime-Glue und Validierung zusammenpassen.

Kein weiterer EEZ-Detailausbau darf begonnen werden, bevor die jeweils benoetigte
Konsolidierung aus Phase A fuer den betroffenen Bereich abgeschlossen ist.

### Schritt 1: EEZ-Projekt anlegen

Voraussetzungen:

- EEZ Studio installiert (aktuelle stabile Version)
- LVGL 8.x als Target-Bibliothek gewaehlt (nicht 9.x)

Aktion:

1. Neues EEZ-Projekt unter `src/teensy/eez_ui/` anlegen.
2. Projektnamen: `DukatimerPart2TeensyUi`.
3. Display-Aufloesung: 480 x 320, Landscape.
4. Color depth: 16 Bit (RGB565, entspricht `LV_COLOR_DEPTH 16` in LVGL).
5. Noch keine Seite anlegen. Nur das Rohprojekt speichern.

Erfolgskriterium: `.eez-project`-Datei ist im Repo eingecheckt.

### Schritt 2: Globale Variablen und Farbkonstanten definieren

Aktion:

In EEZ einen globalen `ViewModel`-Namespace anlegen mit diesen Variablen:

| Variable | Typ | Herkunft |
| --- | --- | --- |
| `global_titleText` | string | `UiPresenter::getTitle()` |
| `global_subtitleText` | string | `UiPresenter::getSubtitle()` |
| `global_overlayText` | string | `UiPresenter::getOverlayText()` |
| `global_overlayColor` | color | optionaler Runtime-Hinweis aus `LvglUi::updateSnapshot()`; finale Farbzuordnung liegt im EEZ-Designer nach Abschnitt 4 |
| `global_overlayState` | int8 | `InputModalState` enum-Wert |
| `global_activeMode` | int8 | `ModeId` enum-Wert |
| `global_activeWorkflowFamily` | int8 | UI-Familie, nicht identisch mit `ModeId`: `0=PAPER`, `1=MEAS`, `2=PRINT`, `3=SETUP` |
| `global_modalState` | int8 | `InputModalState` |
| `global_linkHealth` | int8 | `EspLinkHealth` |
| `global_thermalDeratingActive` | bool | `ExposureRuntimeState::thermalDeratingActive` |
| `global_serviceSensorFlags` | uint16 | `EspLinkRuntimeStatus::serviceSensors.sensorFlags` |
| `global_ahtTemperatureCelsius` | float | `EspLinkRuntimeStatus::serviceSensors.ahtTemperatureCelsius` |
| `global_ahtHumidityPercent` | float | `EspLinkRuntimeStatus::serviceSensors.ahtHumidityPercent` |
| `global_bmpTemperatureCelsius` | float | `EspLinkRuntimeStatus::serviceSensors.bmpTemperatureCelsius` |
| `global_bmpPressureHpa` | float | `EspLinkRuntimeStatus::serviceSensors.bmpPressureHpa` |
| `global_touchActive` | bool | `TouchState::active` |
| `global_startButtonActive` | bool | `SystemSnapshot::startButtonActive` |
| `global_paperActiveSlot` | uint8 | `SystemSnapshot::paperActiveSlot` |
| `global_paperSlotCount` | uint8 | `SystemSnapshot::paperSlotCount` |
| `global_paperActiveSlotName` | string | `SystemSnapshot::paperActiveSlotName` |
| `global_paperActiveSlotCalibrated` | bool | `SystemSnapshot::paperActiveSlotCalibrated` |
| `global_paperActiveGradeMode` | int8 | `SystemSnapshot::paperActiveGradeMode` |

Die Farbkonstanten aus Abschnitt 4 als benannte EEZ-Konstanten anlegen
(nicht als Magic-Numbers in Widgets).

Hinweis:

- Die historischen Namen `global_overlayText` und `global_overlayColor` bleiben
  fuer den Runtime-Glue erhalten. Sie steuern im Produktlayout aber das
  Meldungsband im Header, nicht mehr einen separaten unteren Bereich.
- `global_linkHealth` bleibt Datenquelle fuer degradierte Meldungen,
  Boot-/Diagnoseansichten und die Remote-Seite, wird aber nicht als permanenter
  globaler Header-Chip angezeigt.
- PaperSlot-Recovery-Hinweise bleiben im ersten Schritt weiterhin Teil von
  `global_overlayText`; die sichtbare Entscheidung kommt aus
  `EspLinkRuntimeStatus::paperSlotRecovery`, wird aber nicht als eigener
  permanenter Header-Chip eingefuehrt.

Erfolgskriterium: EEZ-Build ohne Fehler; keine Fachlogik in diesen Definitionen.

### Schritt 3: Globale Meldungs-Komponente bauen (`CompMessageBand`)

Diese Komponente ersetzt den frueheren separaten unteren Overlay-Bereich und ist
die wichtigste globale Zustandsanzeige des Produktlayouts.

Layout:

- Position: zentraler Bereich der HEADER-ZONE (`x=140, y=0, h=48, w=230`)
- Hintergrundfarbe: in EEZ Studio gemaess Abschnitt 4 umgesetzt (Flow-Binding auf
  `global_overlayColor` optional moeglich)
- Text: `global_overlayText`, Schrift Montserrat 14, Farbe `0xF5EDE8`
- Innenabstand: 6px

Sichtbarkeitsregel:

- immer sichtbar, keine Visibility-Condition
- Hintergrundfarbe wechselt entsprechend Farbtabelle Abschnitt 4.1

Pflichtfaelle, die diese Komponente ausloesen muessen:

| Zustand | Farbe | Textvorgabe aus Presenter |
| --- | --- | --- |
| Normal (Idle) | `0x120606` | `UiPresenter::getOverlayText()` -> kurze Kontextmeldung |
| WorkflowFault | `0x3A0506` | Fehlertext aus Presenter |
| WorkflowConfirm | `0x2A100A` | Bestaetigungsaufforderung |
| WorkflowWait | `0x1C0B0B` | Wartemeldung |
| Setup dirty | `0x2A100A` | "Nicht gespeichert" + Item-Name |
| Setup persistFailed | `0x3A0506` | "Speichern fehlgeschlagen" |
| Sensor fallback | `0x34120C` | "SENSOR FALLBACK - kein Closed Loop" |
| SG Completed | `0x1B100C` | "Belichtung abgeschlossen" |
| SG Aborted | `0x241112` | "Belichtung abgebrochen" |
| Link stale/lost | `0x2D120E` | "Wireless-Verbindung unterbrochen" |

Hinweis zur Implementierung:

- Die Farbwerte und Zuordnungen aus Abschnitt 4 werden in EEZ Studio gepflegt,
  damit visuelles Feintuning ohne C++-Aenderung moeglich ist.
- `LvglUi::updateSnapshot()` liefert weiterhin `global_overlayText` und die
  fachlichen Zustaende. `global_overlayColor` kann als optionaler Runtime-Hinweis
  gesetzt werden, ist aber nicht die einzige Quelle fuer das Styling im Designer.
- Die konkrete Abarbeitung erfolgt nach der Checkliste in Abschnitt 4.3.

Erfolgskriterium: Komponente rendert auf allen Seiten korrekt in der
HEADER-ZONE; Farbwechsel bei den Pflichtfaellen sichtbar.

### Schritt 4: Header- und Modusreiter-Komponenten bauen (`CompHeader`, `CompModeTabs`)

Layout Header:

- Position: HEADER-ZONE (`y=0, h=48, w=480`)
- Links (w=140): `global_titleText`, Montserrat 20, `0xF1E7E0`
- Mitte (w=230): `CompMessageBand`
- Rechts (w=110): `CompEssentialStatus` fuer nur wirklich relevante Kurzindikatoren

`CompEssentialStatus` enthaelt:

| Chip | Bedingung | Farbe |
| --- | --- | --- |
| Thermal-Chip | `global_thermalDeratingActive` | Derating=`0xE0A06E`, normal versteckt |
| Head-Power-Hinweis | nur sichtbar wenn `runtimeOutputLimit < 1.0` oder `MaxHeadBrightness` aktiv bearbeitet wird | Normal=`0xB8948A`, Warnung=`0xE0A06E` |

Layout Modusreiter:

- Position: MODETAB-ZONE (`y=284, h=36, w=480`)
- Vier gleich breite Reiter: `PAPER`, `MEAS`, `PRINT`, `SETUP`
- `PageWirelessRemote` bekommt keinen eigenen unteren Reiter; waehrend die
  Seite aktiv ist, bleibt `SETUP` oder der zuletzt aktive Familienreiter
  markiert und der Seitentitel im Header zeigt `REMOTE`.
- Aktiver Reiter: Hintergrund `0x2A0C0C`, Text `0xF1E7E0`
- Inaktiver Reiter: Hintergrund `0x0B0404`, Text `0x8A6A64`

Regeln:

- Die Status-Chips sind nie anklickbar. Sie sind reine Informationsausgabe.
- Thermal-Chip ist nur sichtbar wenn `thermalDeratingActive == true`.
- Ein permanenter Link-Status-Chip ist nicht erlaubt; Link-/Wireless-Zustand
  erscheint nur im Meldungsband oder auf der Remote-Seite.
- Die vollstaendigen Umgebungswerte `AHT20 T/H` und `BMP280 T/P` liegen nicht im
  Header, sondern nur noch auf fachlich passenden Seitenbereichen.
- `CompModeTabs` dient primaer dem Familienwechsel. Touch auf Reiter ist
  erlaubt, darf aber keine zweite, vom Firmwarepfad abweichende
  Navigationslogik aufbauen.
- Zweiteilige Navigation ist Pflicht: die MODETAB-ZONE wechselt nur Familien;
  fachliche Unterfunktionen wie `SG/BW`, `SELECT/CAL` oder `ZONE/DENS` werden
  innerhalb der HAUPT-ZONE mit eigenen Unterreitern oder Kontextchips geloest.

Erfolgskriterium: Header erscheint auf allen Seiten identisch; Modusreiter sind
in der untersten Zeile stabil sichtbar und der aktive Kontext ist eindeutig markiert.

### Schritt 4a: `PagePaperWorkspace` bauen

Diese Seite ist die produktive Arbeitsseite der Familie `PAPER`.
Sie besitzt genau zwei lokale Unteransichten:

- `SELECT`: Papier-Slots browsen; erst `Confirm` macht den gewaehlten Slot zum aktiven Druckslot.
- `CAL`: den gewaehlten Slot lokal staged bearbeiten; der heutige Scope ist ein Kernfeld-Editor mit `Apply/Discard`, noch kein messgetriebener Wizard.

Layout HAUPT-ZONE:

```text
+----------------------+------------------------+ y=48
| SLOT-BLOCK           | DETAIL-/CAL-BLOCK      |
| (w=212, h=172)       | (w=236, h=172)         |
| Slotnummer + Name    | Unteransicht SELECT/CAL|
| CAL/RAW + FG/MG      | Slotdetail oder        |
| Active-/Dirty-Hinweis| CAL-Item + Wert        |
+----------------------+------------------------+ y=220
| KONTEXTBLOCK (h=64)                           |
| Enc1 fein | Enc2 grob | Enc3 Item/View |      |
| Enc4 Confirm | Undo rueck auf Stored/Active   |
+-----------------------------------------------+ y=284
```

Kein separater Status-Footer mehr:

- Aktiver Slot, Dirty-Zustand und Persist-Fehler werden im Detail-/CAL-Block und ueber das Header-Meldungsband sichtbar gemacht.
- Die unterste Zeile bleibt fuer Modusreiter reserviert.

EEZ-Variablen fuer diese Seite:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `paper_panel` | `PaperModeRuntimeState::panel` | int8 |
| `paper_selectedSlot` | `PaperModeRuntimeState::selectedSlot` | uint8 |
| `paper_selectionDirty` | `PaperModeRuntimeState::selectionDirty` | bool |
| `paper_selectedItem` | `PaperModeRuntimeState::selectedItem` | int8 |
| `paper_itemCount` | `PaperModeRuntimeState::itemCount` | uint8 |
| `paper_editingActive` | `PaperModeRuntimeState::editingActive` | bool |
| `paper_parametersDirty` | `PaperModeRuntimeState::parametersDirty` | bool |
| `paper_persistFailed` | `PaperModeRuntimeState::persistFailed` | bool |
| `paper_selectedSlotAgeMs` | `PaperModeRuntimeState::selectedSlotAgeMs` | uint32 |
| `paper_selectedItemAgeMs` | `PaperModeRuntimeState::selectedItemAgeMs` | uint32 |
| `paper_storageErrorCode` | `PaperModeRuntimeState::storageErrorCode` | uint8 |
| `paper_storageErrorDetail` | `PaperModeRuntimeState::storageErrorDetail` | uint32 |
| `paper_slotCount` | `SystemSnapshot::paperSlotCount` | uint8 |
| `paper_activeSlot` | `SystemSnapshot::paperActiveSlot` | uint8 |
| `paper_selectedSlotName` | aus `SystemSnapshot::paperSlotSummaries[selectedSlot].name` verdichtet | string |
| `paper_selectedSlotGradeMode` | aus `SystemSnapshot::paperSlotSummaries[selectedSlot].gradeMode` verdichtet | int8 |
| `paper_selectedSlotCalibrated` | aus `SystemSnapshot::paperSlotSummaries[selectedSlot].calibrated` verdichtet | bool |
| `paper_selectedSlotUseIsoMath` | aus `SystemSnapshot::paperSlotSummaries[selectedSlot].useIsoMath` verdichtet | bool |
| `paper_selectedSlotFixedGradeValue` | aus `SystemSnapshot::paperSlotSummaries[selectedSlot].fixedGradeValue` verdichtet | float |
| `paper_stagedGradeMode` | `PaperCalibrationRuntimeState::gradeMode` | int8 |
| `paper_stagedFixedGradeValue` | `PaperCalibrationRuntimeState::fixedGradeValue` | float |
| `paper_stagedUseIsoMath` | `PaperCalibrationRuntimeState::useIsoMath` | bool |
| `paper_stagedIsoP` | `PaperCalibrationRuntimeState::isoP` | float |
| `paper_stagedIsoR` | `PaperCalibrationRuntimeState::isoR` | float |
| `paper_stagedKBw` | `PaperCalibrationRuntimeState::kBw` | float |
| `paper_stagedKSoft` | `PaperCalibrationRuntimeState::kSoft` | float |
| `paper_stagedKHard` | `PaperCalibrationRuntimeState::kHard` | float |
| `paper_stagedCalibrated` | `PaperCalibrationRuntimeState::calibrated` | bool |

Regeln:

- `SELECT` und `CAL` sind lokale Unteransichten innerhalb von `PAPER`, keine globalen Reiter.
- `SELECT` darf Slotwechsel nur sichtbar vorbereiten. Erst `Confirm` schreibt ueber `selectActiveSlot()` den aktiven Druckslot um.
- `CAL` arbeitet auf dem aktuell gewaehlten Slot, nicht still auf dem gerade aktiven `PRINT`-Papier.
- `Apply` schreibt ueber `saveProfileAt(selectedSlot, stagedProfile)`; `Discard` verwirft nur den lokalen Arbeitssatz.
- Der Glue-Code darf die gerade ausgewaehlte Slot-Zeile aus `paperSlotSummaries[]` in page-lokale EEZ-Variablen verdichten; EEZ muss nicht selbst ueber das gesamte Array iterieren.

Erfolgskriterium: `SELECT` ist encoder-only browse- und bestaetigbar; `CAL` zeigt Item/Wert/Dirty/Persist-Fehler sichtbar an und `Apply/Discard` arbeiten auf dem gewaehlten Slot.

### Schritt 5: `PageSplitgrade` bauen

Diese Seite ist die primaere Arbeitsseite fuer die Dunkelkammer innerhalb der
Familie `PRINT`.

`PageSplitgrade` ist damit nicht selbst ein globaler Hauptreiter, sondern eine
Fachseite unter `PRINT`. Die Familie `PRINT` umfasst fuer Part2 mindestens
`PageSplitgrade`, spaeter `PageBlackWhite` und kontextuelle Helfer wie
Preflash, Teststrip und Burn.

Layout HAUPT-ZONE:

```text
+------------------------------------------+ y=48
| SG-PANEL LINKS (w=220, h=96)            |
|  Grade-Wert (Montserrat 36, bold)        |
|  Soft-Target (Montserrat 20)             |
|  Hard-Target (Montserrat 20)             |
|  ControlMode-Chip (Zeit/Dosis)           |
+------------------+-----------------------+ y=144
| EXECUTION-STATE  | EXPOSURE-BLOCK        |
| (w=200, h=104)   | (w=280, h=104)        |
| ExecutionState-  | RemainingTime (36px)  |
| Label            | CurrentDose/TargetDose|
| PaperSlot/Dirty  | MeasuredLux           |
| PhaseFortschritt | HeadPowerPercent      |
+------------------------------------------+ y=248
| KONTEXTBLOCK (h=36)                      |
| Encoderhinweise | AHT/BMP falls nuetzlich|
+------------------------------------------+ y=284
```

Kein separater Status-Footer mehr:

- `sg_paperActiveSlotName` und der Calibrated-Chip liegen im linken unteren
  Bereich der HAUPT-ZONE.
- Encoder-Hinweise stehen nur als kurze Kontextzeile im `KONTEXTBLOCK`.
- `AHT20 T/H` und `BMP280 T/P` erscheinen nur, wenn dafuer Platz bleibt; sie
  haben geringere Prioritaet als Papier, Dirty-Zustand und Head-Leistung.
- Die unterste Zeile bleibt ausschliesslich fuer die Modusreiter reserviert.

EEZ-Variablen fuer diese Seite:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `sg_headerText` | `UiPresenter::getSgHeader()` | string |
| `sg_targetsText` | `UiPresenter::getSgTargets()` | string |
| `sg_exposureMainText` | `UiPresenter::getSgExposureMain()` | string |
| `sg_panel` | `SplitgradeModeRuntimeState::panel` | int8 |
| `sg_executionState` | `SplitgradeModeRuntimeState::executionState` | int8 |
| `sg_controlMode` | `SplitgradeModeRuntimeState::controlMode` | int8 |
| `sg_softTarget` | `SplitgradeModeRuntimeState::softTarget` | float |
| `sg_hardTarget` | `SplitgradeModeRuntimeState::hardTarget` | float |
| `sg_grade` | `SplitgradeModeRuntimeState::grade` | float |
| `sg_parametersDirty` | `SplitgradeModeRuntimeState::parametersDirty` | bool |
| `sg_exposurePhase` | `ExposureRuntimeState::phase` | int8 |
| `sg_remainingTimeSeconds` | `ExposureRuntimeState::remainingTimeSeconds` | float |
| `sg_currentDose` | `ExposureRuntimeState::currentDose` | float |
| `sg_targetDose` | `ExposureRuntimeState::targetDose` | float |
| `sg_measuredLux` | `ExposureRuntimeState::measuredLux` | float |
| `sg_runtimeOutputLimit` | `ExposureRuntimeState::runtimeOutputLimit` | float |
| `sg_runtimeHeadBusLatencyMs` | `ExposureRuntimeState::runtimeHeadBusLatencyMs` | uint32 |
| `sg_faultReason` | `ExposureRuntimeState::faultReason` | int8 |
| `sg_faultLatched` | `ExposureRuntimeState::faultLatched` | bool |
| `sg_sensorFallbackActive` | `ExposureRuntimeState::sensorFallbackActive` | bool |
| `sg_thermalDeratingActive` | `ExposureRuntimeState::thermalDeratingActive` | bool |

Wichtig: Grosse Zahlen (Grade, Time, Dose) werden in der HAUPT-ZONE gross und
kontrastreich dargestellt. Mindestschriftgroesse fuer die Hauptzahl: Montserrat 36.

Zusatzregel fuer die Head-Leistung:

- `runtimeOutputLimit` wird nie als Rohwert `0.0 .. 1.0` angezeigt, sondern
  immer als Prozentwert `HEAD xx%`.
- Wenn die effektive Leistung unter der konfigurierten Grenze liegt und
  `thermalDeratingActive == true` ist, bekommt `HEAD xx%` Warnfarbe und einen
  kurzen Zusatz `THERM`.

Zusatzregel fuer PRINT-Kontextaktionen:

- Preflash, Teststrip und Burn werden aus `PageSplitgrade` nicht als globale
  Hauptreiter aufgerufen, sondern als kontextgebundene Aktionen aus dem aktiven
  Druckkontext.
- Messdaten aus `MEAS` duerfen hier nur als Vorschlag oder Uebernahmeaktion
  erscheinen; die Seite darf Papier, Grade oder Preflash nicht stillschweigend
  ueberschreiben.

Erfolgskriterium: Alle sechs `SplitgradeExecutionState`-Zustaende fuehren zu
sichtbar unterscheidbaren Darstellungen; das Header-Meldungsband reagiert korrekt
auf `Fault`, `Completed`, `Aborted` und die Head-Leistung ist als Prozentwert sichtbar.

### Schritt 6: `PageSetup` bauen

Setup ist eine temporaere Ueberlagerung des Hauptmodus.
Waehrend Setup aktiv ist, liegt der Hintergrundscreen weiterhin dahinter.

Layout HAUPT-ZONE:

```text
+------------------------------------------+ y=48
| SETUP-ITEM-LISTE (scrollbar, h=132)      |
|  [ ] SoundMode           [Wert]          |
|  [ ] SoundVolume         [Wert]          |
|  [ ] Vibration           [An/Aus]        |
|  [ ] MaxHeadBrightness   [Wert %]        |
|  [ ] ThermalDerating     [Wert C]        |
|  [ ] ThermalHardStop     [Wert C]        |
|  --------                                |
|  [Apply]  [Discard]  [SafetyDefaults]    |
+------------------------------------------+ y=180
| EDIT-BLOCK (h=104)                       |
| Aktueller Item-Name (Montserrat 18)      |
| Aktueller Wert (Montserrat 30, bold)     |
| HEAD CAP xx% | LIVE yy%                  |
| Erlaubter Bereich / Thermal-Hinweis      |
+------------------------------------------+ y=284
```

Kein separater Status-Footer mehr:

- Dirty- und Persist-Fehler werden im Header-Meldungsband signalisiert, nicht
  in einer unteren Statusleiste.
- `Enc1=Wert  Enc3=Item  Enc3-Long=Beenden` erscheint nur als kurze Hilfe im
  `EDIT-BLOCK`, solange sie fachlich nuetzlich ist.
- Die unterste Zeile bleibt fuer Modusreiter reserviert.

EEZ-Variablen fuer diese Seite:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `setup_selectedItem` | `SetupModeRuntimeState::selectedItem` | int8 |
| `setup_itemCount` | `SetupModeRuntimeState::itemCount` | uint8 |
| `setup_editingActive` | `SetupModeRuntimeState::editingActive` | bool |
| `setup_parametersDirty` | `SetupModeRuntimeState::parametersDirty` | bool |
| `setup_persistFailed` | `SetupModeRuntimeState::persistFailed` | bool |
| `setup_selectedItemAgeMs` | `SetupModeRuntimeState::selectedItemAgeMs` | uint32 |
| `setup_storageErrorCode` | `SetupModeRuntimeState::storageErrorCode` | uint8 |
| `setup_storageErrorDetail` | `SetupModeRuntimeState::storageErrorDetail` | uint32 |
| `setup_soundMode` | `SystemSettings::soundMode` | int8 |
| `setup_soundVolume` | `SystemSettings::soundVolume` | int8 |
| `setup_vibrationEnabled` | `SystemSettings::vibrationEnabled` | bool |
| `setup_maxHeadBrightnessPercent` | `SystemSettings::maxHeadBrightnessPercent` | float |
| `setup_thermalDeratingStartCelsius` | `SystemSettings::thermalProtection.deratingStartCelsius` | float |
| `setup_thermalHardStopCelsius` | `SystemSettings::thermalProtection.hardStopCelsius` | float |
| `setup_runtimeOutputLimit` | `ExposureRuntimeState::runtimeOutputLimit` | float |
| `setup_thermalDeratingActive` | `ExposureRuntimeState::thermalDeratingActive` | bool |
| `setup_headerText` | `UiPresenter::getSgHeader()` im Setup-Modus | string |
| `setup_valueText` | `UiPresenter::getSgExposureMain()` im Setup-Modus | string |
| `setup_overlayText` | `UiPresenter::getOverlayText()` im Setup-Modus | string |

Regel fuer die Leistungsanzeige im Setup:

- `setup_maxHeadBrightnessPercent` ist die konfigurierte Grenze (`CAP`).
- `setup_runtimeOutputLimit` ist die effektive Laufzeitleistung und wird als
  Prozentwert `LIVE yy%` dargestellt.
- Wenn `setup_selectedItem == MaxHeadBrightness` oder
  `setup_thermalDeratingActive == true`, muss der `EDIT-BLOCK` beide Werte
  gleichzeitig zeigen.

Aktionen (Touch, Enc3-Press und Enc4-Confirm):

| Aktion | Ausloeser | Wirkung |
| --- | --- | --- |
| Apply | Touch-Button, Enc3-Press oder Enc4-Confirm auf `Apply` | `SetupMenuItem::Apply` ausfuehren |
| Discard | Touch-Button, Enc3-Press oder Enc4-Confirm auf `Discard` | `SetupMenuItem::Discard` ausfuehren |
| SafetyDefaults | Touch-Button, Enc3-Press oder Enc4-Confirm auf `SafetyDefaults` | `SetupMenuItem::SafetyDefaults` ausfuehren |
| Setup verlassen | Enc3 LongPress | zurueck zu `setupReturnMode` |

Erfolgskriterium: Dirty-Chip sichtbar nach Wertaenderung; PersistFailed-Overlay
erscheint nach simuliertem SD-Fehler; `CAP/LIVE`-Leistung reagiert auf
Setup-Aenderungen oder thermische Begrenzung; Verlassen funktioniert nur wenn
kein ungespeicherter Fault vorliegt oder per Discard bestaetigt.

### Schritt 7: Busy-Screen fuer aktive Exposure-Phasen

Aktueller Code-Stand: `SCREEN_ID_BUSY` ist als eigener LVGL-Screen umgesetzt,
obwohl er fachlich kein normaler Navigationsscreen ist. Diese Trennung ist fuer
den jetzigen Sicherheits- und Bring-up-Stand bewusst: waehrend
`PreWait`, `Exposing`, `Paused` und `PostWait` hat die aktive
Belichtungsdarstellung absolute Prioritaet vor Paper-, Setup-, Measurement- und
normalen Print-Seiten.

Wichtig fuer spaetere EEZ-Arbeit:

- Der Busy-Screen darf keinen `build_header()`-Aufruf verwenden, weil sonst die
  shared `hdr_*`-Handles der normalen Seiten ueberschrieben werden. Er besitzt
  eigene `busy_hdr_*`-Handles.
- Der Screen liest nur `ExposureRuntimeState`, `SplitgradeModeRuntimeState`,
  `SystemSnapshot` und Presenter-Texte. Er rechnet keine eigene
  fotografische Belichtung.
- Touch-Aktionen im Pause-Overlay werden nur als `BusyAction` gepuffert;
  die eigentliche Safety-Entscheidung bleibt in `main.cpp` und
  `ExposureEngine`.

Busy-Screen-Layout:

```text
+------------------------------------------+ y=48
| EIGENER HEADER: Phase | Meldung | THERM   |
+------------------------------------------+ y=70
| HAUPTZAHL (Montserrat 28)                |
| Restzeit in Sekunden ODER Dosisstand     |
+------------------------------------------+ y=140
| FORTSCHRITTSBALKEN + Prozent             |
+------------------------------------------+ y=200
| SG-DETAIL: Soft/Hard-Restzeiten          |
+------------------------------------------+ y=240
| PAUSE-OVERLAY: FORTSETZEN / ABBRECHEN    |
+------------------------------------------+ y=284
```

Regeln:

- Die aktuelle handgeschriebene UI nutzt Montserrat 28 als groesste aktive
  Schrift, weil Montserrat 36 zuvor ITCM/RAM1-Druck erzeugt hat. Groessere
  Exposure-Zahlen duerfen erst nach erneutem Speicher-Buildcheck wieder
  freigegeben werden.
- Fallback-Banner und Thermal-Banner sind Rot-auf-Dunkel, immer wenn aktiv.
- `runtimeOutputLimit` ist als Prozentwert `HEAD xx%` darzustellen; rohe
  Float-Werte `0.0 .. 1.0` sind in der Produkt-UI unzulaessig.
- In dieser Darstellung sind Enc1 und Enc2 gesperrt.
- Start-Taste stoppt/pausiert die Belichtung (Firmware-Entscheidung, nicht EEZ).

EEZ-Variablen fuer diesen Zustand:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `exposure_phase` | `ExposureRuntimeState::phase` | int8 |
| `exposure_controlMode` | `ExposureRuntimeState::controlMode` | int8 |
| `exposure_targetDose` | `ExposureRuntimeState::targetDose` | float |
| `exposure_currentDose` | `ExposureRuntimeState::currentDose` | float |
| `exposure_remainingDose` | `ExposureRuntimeState::remainingDose` | float |
| `exposure_remainingTimeSeconds` | `ExposureRuntimeState::remainingTimeSeconds` | float |
| `exposure_measuredLux` | `ExposureRuntimeState::measuredLux` | float |
| `exposure_runtimeOutputLimit` | `ExposureRuntimeState::runtimeOutputLimit` | float |
| `exposure_runtimeHeadBusLatencyMs` | `ExposureRuntimeState::runtimeHeadBusLatencyMs` | uint32 |
| `exposure_thermalDeratingActive` | `ExposureRuntimeState::thermalDeratingActive` | bool |
| `exposure_sensorFallbackActive` | `ExposureRuntimeState::sensorFallbackActive` | bool |
| `exposure_sensorFallbackReason` | `ExposureRuntimeState::sensorFallbackReason` | int8 |
| `exposure_faultReason` | `ExposureRuntimeState::faultReason` | int8 |
| `exposure_faultLatched` | `ExposureRuntimeState::faultLatched` | bool |
| `exposure_phaseAgeMs` | `ExposureRuntimeState::phaseAgeMs` | uint32 |

`ExposurePhase`-Werte und empfohlene UI-Labels:

| Wert | UI-Label | Hintergrundstimmung |
| --- | --- | --- |
| `Idle` | - (kein Label) | Normal |
| `PreWait` | `BEREIT` | Normal |
| `Exposing` | `BELICHTET` | aktiv, kein Fehler |
| `Paused` | `PAUSE` | gedaempft |
| `PostWait` | `NACHWARTEZEIT` | gedaempft |
| `Done` | `FERTIG` | SG-Completed-Farbe |
| `Fault` | `FEHLER` | Fault-Farbe |

Erfolgskriterium: Wechsel in `SCREEN_ID_BUSY` und Rueckkehr zur fachlichen
Zielseite funktionieren rein aus Snapshot/Phase; Hauptzahl wechselt korrekt
zwischen Dosis und Zeit je nach `controlMode`; Pause zeigt nur dort
`FORTSETZEN`/`ABBRECHEN`, wo `ExposureEngine::resume()` und
`ExposureEngine::stopGracefully()` fachlich gueltig sind.

### Schritt 8: `PageMeasurement` bauen

Zweck: Vorbereitende Auswertung lokaler und Wireless-Lux-Messungen, Referenz und
Histogramm vor der eigentlichen Belichtung.

Layout HAUPT-ZONE:

```text
+------------------+-----------------------+ y=48
| QUELLEN-BLOCK    | HAUPTMESSWERT         |
| (w=180, h=100)   | (w=300, h=100)        |
| Lokal Lux        | Aktiv-Lux (Montserrat |
| Wireless Lux     | 48, bold)             |
| Quellen-Chip     | Alter / Sequenz       |
+------------------+-----------------------+ y=140
| REFERENZ-BLOCK (w=240, h=50)            |
| Referenz-Lux | Relative EV-Abstand      |
+------------------------------------------+ y=190
| HISTOGRAMM (w=448, h=50)               |
| 11 Zonen-Balken, proportional zu Bucket |
+------------------------------------------+ y=240
| SESSION-KONTEXT (h=44)                 |
| Count / Undo / C6-Hinweis / AHT-BMP    |
+------------------------------------------+ y=284
```

Kein separater Status-Footer mehr:

- `sampleCount`, `capturedSampleCount`, Undo-Chip und der C6-Hinweis liegen im
  `SESSION-KONTEXT` innerhalb der HAUPT-ZONE.
- `AHT20 T/H` und `BMP280 T/P` werden nur im `SESSION-KONTEXT` gezeigt, wenn
  sie die Messlesbarkeit nicht verdraengen.
- Die unterste Zeile bleibt fuer Modusreiter reserviert.

Regel:

- Enc4 bleibt auf dieser Seite der Menue-/Kontextencoder des Teensy. Die Auswahl
  von Zone, Messbereich oder Spot-/Mehrpunktkontext des Handbedienteils gehoert
  zum C6-Encoder und darf in der EEZ-Beschriftung nicht als `Enc4` erscheinen.
- `PageMeasurement` ist kein Belichtungsscreen. Die Seite sammelt, vergleicht
  und visualisiert Messwerte und gibt daraus nur sichtbare Vorschlaege an
  `PAPER` oder `PRINT` weiter.
- Die Unteransicht `ZONE` ist der Defaultfall; eine spaetere Unteransicht
  `DENS` fuer Durchlicht-Densitometrie gehoert in dieselbe Familie `MEAS`,
  nicht nach `SETUP` und nicht direkt in `PRINT`.

EEZ-Variablen fuer diese Seite:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `meas_activeSource` | `MeasurementLuxSample::source` | int8 |
| `meas_activeLuxValid` | `MeasurementLuxSample::valid` | bool |
| `meas_activeLux` | `MeasurementLuxSample::lux` | float |
| `meas_activeLuxAgeMs` | `MeasurementLuxSample::ageMs` | uint32 |
| `meas_activeLuxSequence` | `MeasurementLuxSample::sequence` | uint32 |
| `meas_referenceValid` | `MeasurementReferenceStatus::valid` | bool |
| `meas_referenceLux` | `MeasurementReferenceStatus::lux` | float |
| `meas_referenceSequence` | `MeasurementReferenceStatus::sequence` | uint32 |
| `meas_relativeEvValid` | `MeasurementRuntimeStatus::activeRelativeEvValid` | bool |
| `meas_relativeEvStops` | `MeasurementRuntimeStatus::activeRelativeEvStops` | float |
| `meas_localLux` | `MeasurementRuntimeStatus::localLux.lux` | float |
| `meas_wirelessLux` | `MeasurementRuntimeStatus::wirelessLux.lux` | float |
| `meas_sampleCount` | `MeasurementSessionStatus::sampleCount` | uint32 |
| `meas_capturedSampleCount` | `MeasurementSessionStatus::capturedSampleCount` | uint32 |
| `meas_latestZoneIndex` | `MeasurementSessionSample::zoneIndex` (latestSample) | uint8 |
| `meas_undoDepth` | `MeasurementSessionStatus::undoDepth` | uint8 |
| `meas_canUndo` | `MeasurementSessionStatus::canUndo` | bool |
| `meas_zoneHistogram` | `MeasurementSessionStatus::zoneHistogram` | int-Array[11] |

Histogramm-Rendering:

- 11 Balken, je Zone-Bucket ein Balken (Breite: (480-32)/11 = ca. 41px)
- Balkenhoehe proportional zum Bucket-Wert
- Zone mit `latestZoneIndex` gelb markiert
- Nullwert-Bucket grau, Wert > 0 in warmem Rot-/Ockerbereich

Erfolgskriterium: Histogramm zeigt korrekte Zonenbelegung nach drei
simulierten Messungen; Undo-Chip erscheint und verschwindet korrekt, und die
Seite kann sichtbare Vorschlaege an `PAPER` oder `PRINT` zurueckmelden.

### Schritt 9: `PageWirelessRemote` bauen

Diese Seite ist eine Diagnose-/Statusseite, keine Bedienungsseite.
Erreichbar per Touch-Geste (Wischgeste von rechts oder spaeter Menuepunkt).
Nicht als normaler Navigationszustand im Encoder-Modell.
Sie zeigt den Zustand von ESP32-S3-Gateway und C6-Terminal, bildet aber nicht die
eigene UI des Handbedienteils nach.

Layout HAUPT-ZONE:

```text
+------------------+-----------------------+ y=40
| LINK-BLOCK       | PEER-BLOCK            |
| (w=200, h=100)   | (w=280, h=100)        |
| LinkHealth-Chip  | PeerState-Chip        |
| RxAge / TxAge   | BatteryPercent        |
| RemoteUptime     | LastSeenAgeMs         |
|                  | LastLux               |
+------------------+-----------------------+ y=140
| DIAGNOSE-BLOCK (h=60)                   |
| DiagnosticCode + Detail + Counter       |
| TxQueue: Pending/Evicted/Dropped        |
+------------------------------------------+ y=200
| RMT-BLOCK (h=40)                        |
| InFlight / Retry / Timeout / Saturation |
+------------------------------------------+ y=240
```

EEZ-Variablen fuer diese Seite:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `remote_linkHealth` | `EspLinkRuntimeStatus::health` | int8 |
| `remote_heartbeatSeen` | `EspLinkRuntimeStatus::heartbeatSeen` | bool |
| `remote_lastRxAgeMs` | `EspLinkRuntimeStatus::lastRxAgeMs` | uint32 |
| `remote_lastTxAgeMs` | `EspLinkRuntimeStatus::lastTxAgeMs` | uint32 |
| `remote_remoteUptimeMs` | `EspLinkRuntimeStatus::remoteUptimeMs` | uint32 |
| `remote_peerState` | `WirelessGatewayStatus::peerState` | int8 |
| `remote_batteryPercent` | `WirelessGatewayStatus::batteryPercent` | uint8 |
| `remote_lastSeenAgeMs` | `WirelessGatewayStatus::lastSeenAgeMs` | uint32 |
| `remote_lastLux` | `WirelessGatewayStatus::lastLux` | float |
| `remote_measurementSequence` | `WirelessGatewayStatus::measurementSequence` | uint32 |
| `remote_commandAckSequence` | `WirelessGatewayStatus::commandAckSequence` | uint32 |
| `remote_renderStatusFlags` | `WirelessGatewayStatus::renderStatusFlags` | uint16 |
| `remote_staleRenderCount` | `WirelessGatewayStatus::staleRenderCount` | uint32 |
| `remote_renderTimeoutCount` | `WirelessGatewayStatus::renderTimeoutCount` | uint32 |
| `remote_diagnosticCode` | `EspDiagnosticStatus::code` | int8 |
| `remote_diagnosticDetail` | `EspDiagnosticStatus::detail` | uint16 |
| `remote_diagnosticCounter` | `EspDiagnosticStatus::counter` | uint32 |
| `remote_txPendingFrameCount` | `EspTxQueueStatus::pendingFrameCount` | uint8 |
| `remote_txDroppedRenderCount` | `EspTxQueueStatus::droppedRenderCount` | uint32 |
| `remote_txDroppedCommandCount` | `EspTxQueueStatus::droppedCommandCount` | uint32 |
| `remote_rmtInFlightCount` | `RemoteCommandTrackerStatus::inFlightCount` | uint8 |
| `remote_rmtRetryCount` | `RemoteCommandTrackerStatus::retryCount` | uint32 |
| `remote_rmtTimeoutCount` | `RemoteCommandTrackerStatus::timeoutCount` | uint32 |
| `remote_rmtSaturationCount` | `RemoteCommandTrackerStatus::saturationCount` | uint32 |

Erfolgskriterium: Alle Status-Chips reagieren auf simulierte Zustandsaenderungen;
Seite ist per Touch-Geste erreichbar und per Enc3-Press verlassbar.

### Schritt 10: `PageBootStatus` bauen

Aktiv nur bis `modeState.activeMode != None`. Danach automatischer Wechsel.

Layout: Ganzer Bildschirm als einfache Statusliste.

EEZ-Variablen:

| EEZ-Variable | Quelle | Typ |
| --- | --- | --- |
| `boot_titleText` | `UiPresenter::getTitle()` | string |
| `boot_subtitleText` | `UiPresenter::getSubtitle()` | string |
| `boot_activeMode` | `ModeRuntimeState::activeMode` | int8 |
| `boot_linkHealth` | `EspLinkRuntimeStatus::health` | int8 |
| `boot_paperSlotStorageErrorCode` | `SystemSnapshot::paperSlotStorageErrorCode` | uint8 |
| `boot_paperSlotStorageErrorDetail` | `SystemSnapshot::paperSlotStorageErrorDetail` | uint32 |
| `boot_remoteCapabilityBits` | `EspLinkRuntimeStatus::remoteCapabilityBits` | uint32 |
| `boot_touchActive` | `TouchState::active` | bool |

Erfolgskriterium: Seite wechselt automatisch nach PageSplitgrade, wenn
`activeMode == Splitgrade`.

### Schritt 11: Glue-Code in `LvglUi` konsolidieren

Ist-Stand:

- `LvglUi::begin()` ruft `ui_init()` auf.
- `LvglUi::updateSnapshot()` setzt alle 155 EEZ-Flow-Globals ueber
  `flow::setGlobalVariable(...)`.
- `computeTargetScreen()` und `eez_flow_set_screen()` schalten die aktiven
  Screens.
- `LvglUi::tick()` ruft ausschliesslich `lv_tick_inc()` auf. Der EEZ-Flow-Tick
  (`eez_flow_tick()` + `tick_screen()`) liegt in `LvglUi::service()`, die nach
  `lv_timer_handler()` `ui_tick()` aufruft.
- Die sichtbare Darstellung wird trotzdem weiter direkt in
  `pushWidgetsFromSnapshot()` ueber `g_duka_widgets`, `lv_label_set_text*()` und
  direkte Style-Updates geschrieben.
- Generierte `ui_set_*`-Setter sind im aktuellen Runtime-Code nicht vorhanden
  und duerfen nicht als Implementierungsbeispiel verwendet werden.
- `native_vars[]` in `ui.c` enthaelt einen einzigen `NONE`-Dummy-Eintrag;
  `actions[]` ist `{ 0 }` — beide Stubs ohne realen Inhalt; EEZ-native
  Variablen-Bindings und generierte Actions existieren nicht.

Ziel:

- EEZ Studio liefert die produktive Layout-/Style-/Action-Struktur.
- `LvglUi` bleibt der einzige Firmware-Glue aus `SystemSnapshot`, Presenter und
  Formatter in die UI.
- Direkte LVGL-Handle-Updates bleiben nur fuer noch nicht migrierte Seiten als
  klar gekennzeichneter Uebergangspfad erhalten.

Aktion:

- ~~Workflow-Familiennummerierung zuerst normalisieren~~ **Erledigt (Prompt A1):**
  `0=PAPER, 1=MEAS, 2=PRINT, 3=SETUP`; `computeWorkflowFamily()` liefert diese
  Werte korrekt; EEZ-Conditions und ModeTabs-Vergleiche sind synchronisiert.
- ~~`computeWorkflowFamily(snapshot)` auf diese UI-Familien abbilden~~ **Erledigt (Prompt A1)**.
- ~~`global_activeWorkflowFamily` in EEZ-Conditions, Doku und Runtime identisch
   verwenden~~ **Erledigt (Prompt A1)**.
- Fuer den aktuellen Glue bleibt `flow::setGlobalVariable(...)` die gueltige
   Schnittstelle. Ein Wechsel auf generierte Setter ist erst erlaubt, wenn diese
   Setter real im exportierten EEZ-Code existieren und gebaut wurden.
- Seite fuer Seite migrieren: Eine Seite gilt erst als EEZ-gefuehrt, wenn ihr
   sichtbares Layout, ihre Styles und ihre Touch-Actions aus dem EEZ-Export oder
   einem explizit dokumentierten EEZ-Handle-Vertrag stammen.
- `pushWidgetsFromSnapshot()` pro migrierter Seite zurueckbauen. Nicht migrierte
   Seiten duerfen weiter direkt aktualisiert werden, muessen aber im Status als
   handgeschriebene Runtime markiert bleiben.

Regeln:

- Keine neuen `snprintf()`-Usertexte in `LvglUi.cpp` fuer Messwerte oder andere
  fachlich formatierte Anzeigen. Benutzertexte kommen aus Presenter/Formatter.
- Keine neuen Magic-Hexwerte in `LvglUi.cpp` oder `screens.c`, ausser als
  befristeter Fallback fuer noch nicht migrierte Seiten.
- EEZ-Actions duerfen keine Safety-Entscheidung treffen. Sie liefern nur
  UI-Ereignisse, die vom bestehenden Firmwarepfad validiert werden.
- Busy bleibt der letzte Migrationskandidat, weil dort Exposure-Safety,
  Pause/Abort und Rueckkehrlogik besonders kritisch sind.

Erfolgskriterium:

- `pio run -e teensy41` ohne neue Fehler oder ungeklärte Warnungen.
- Die aktive Seite zeigt nach Flash dasselbe Layout wie der EEZ-Stand.
- `global_activeWorkflowFamily` markiert in EEZ und Runtime denselben Reiter.
- Keine migrierte Seite braucht noch direkte Placeholder-Label-Updates aus
  `pushWidgetsFromSnapshot()`.

### Schritt 12: Validierung nach jedem Slice

Nach jedem Schritt:

1. `pio run -e teensy41` muss mit Exit Code 0 enden.
2. Der Teensy-Build darf keine fruehere SdFat-Baseline-Warnung mehr tragen;
  stattdessen erzwingt der Pre-Build-Check jetzt die Storage-Policy aus
  `src/teensy/TeensyStoragePolicy.h` und blockiert `SD.h`, `FS.h`, direkte
  `SdFat.h`-Includes oder bare `File` im Teensy-Pfad vor dem eigentlichen Build.
3. Neue Warnungen muessen begruendet oder gefixt werden.
4. Manuelle Hardware-Checkliste ausfuehren.

Abnahmepunkte:

- Encoder-only-Durchgang: SG-Config ohne Touch erreichbar und bedienbar.
- PAPER `SELECT/CAL` ohne Touch bedienbar; `Apply/Discard` greifen auf den
  gewaehlten Slot statt still auf den aktiven PRINT-Slot.
- Enc3 LongPress fuehrt in Setup und zurueck.
- Header-Meldungsband stabil bei Fault-/Confirm-Simulation.
- Modusreiter in der untersten Zeile bleiben sichtbar und markieren den aktiven
  Kontext eindeutig.
- Belichtungs-Hauptzahl gross und lesbar auf Display bei gedimmtem Licht.
- Head-Leistung erscheint als Prozentwert und reagiert sichtbar auf Setup-Cap
  und Thermal-Guard.
- Setup dirty/persistFailed Farbaenderung sichtbar.
- Touch blockiert Encoder nicht unerwartet.

---

## 7. Vollstaendiger Variablenkatalog

### 7.1 Systemschnappschuss-Oberstruktur (`SystemSnapshot`)

| Feld | Typ | EEZ-Verwendung |
| --- | --- | --- |
| `encoder1Status` | `RotaryEncoderRuntimeStatus` | Diagnoseseite |
| `encoder2Status` | `RotaryEncoderRuntimeStatus` | Diagnoseseite |
| `encoder3Status` | `RotaryEncoderRuntimeStatus` | Diagnoseseite |
| `startButtonActive` | bool | `global_startButtonActive` |
| `paperSlotStorageErrorCode` | uint8 | `global`, Boot |
| `paperSlotStorageErrorDetail` | uint32 | `global`, Boot |
| `paperActiveSlot` | uint8 | `global_paperActiveSlot` |
| `paperSlotCount` | uint8 | Paper-Seite |
| `paperActiveSlotCalibrated` | bool | `global_paperActiveSlotCalibrated` |
| `paperActiveGradeMode` | `PaperGradeMode` | `global_paperActiveGradeMode` |
| `paperActiveSlotName` | string | `global_paperActiveSlotName` |
| `paperActiveProfile` | `PaperProfileUiDetail` | PAPER/BW-Seite |
| `paperSlotSummaries` | `PaperSlotUiSummary[kPaperSlotCount]` | PagePaperWorkspace-Glue |
| `inputModalStateCode` | uint8 | `global_modalState` |
| `localDoseControlForcedTime` | bool | Exposure-Banner |
| `localDoseControlWatchdogResetLatched` | bool | Exposure-Banner |
| `localDoseControlDiagnosticReasonCode` | uint8 | Exposure-Diagnose |
| `modeState` | `ModeRuntimeState` | Seitennavigation |
| `exposureState` | `ExposureRuntimeState` | SG, BW, Exposure-Overlay |
| `measurementStatus` | `MeasurementRuntimeStatus` | PageMeasurement |
| `espLinkStatus` | `EspLinkRuntimeStatus` | Header, Remote-Seite |
| `remoteCommandTrackerStatus` | `RemoteCommandTrackerStatus` | Remote-Seite |
| `sensorStatus` | `SensorRuntimeStatus` | Header (Thermal), Diagnose |
| `lightState` | `LightState` | Diagnose / Switch-Info |
| `touchState` | `TouchState` | `global_touchActive` |

### 7.2 `RotaryEncoderRuntimeStatus`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `logicalPosition` | long | aktuelle Detent-Position |
| `detentCount` | uint32 | Gesamtanzahl ausgefuehrter Detent-Schritte |
| `partialReverseCount` | uint32 | Qualitaetsindikator: unvollstaendige Rueckschritte |
| `phaseMismatchCount` | uint32 | Qualitaetsindikator: Phasenplausibilitaetsfehler |
| `buttonActive` | bool | Taster gedrueckt |
| `buttonChanged` | bool | Taster-Flankenwechsel seit letztem Tick |

### 7.3 `ModeRuntimeState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `activeMode` | `ModeId` | aktuell sichtbare Workflow-Ebene |
| `requestedMode` | `ModeId` | gewuenschter Folgemodus |
| `transitionPending` | bool | Moduswechsel noch nicht abgeschlossen |
| `activeModeAgeMs` | uint32 | seit wann aktiver Modus gilt |
| `lastInputAgeMs` | uint32 | seit wann letztes Input-Event |
| `splitgrade` | `SplitgradeModeRuntimeState` | SG-Zustand |
| `paper` | `PaperModeRuntimeState` | PAPER-Zustand |
| `setup` | `SetupModeRuntimeState` | Setup-Zustand |

### 7.4 `SplitgradeModeRuntimeState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `panel` | `SplitgradePanel` | Inactive/SplitTargets/Grade/ControlMode |
| `executionState` | `SplitgradeExecutionState` | Idle/ArmingSoft/.../Completed/Aborted/Fault |
| `controlMode` | `ExposureControlMode` | Time oder Dose |
| `softTarget` | float | Soft-Belichtungsziel |
| `hardTarget` | float | Hard-Belichtungsziel |
| `grade` | float | Gradwert 0.0 bis 5.0 in 0.5er-Schritten |
| `parametersDirty` | bool | ungesicherte Aenderungen |
| `panelAgeMs` | uint32 | Alter des letzten Panel-Wechsels |
| `executionStateAgeMs` | uint32 | Alter des letzten State-Wechsels |

### 7.4a `PaperModeRuntimeState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `panel` | `PaperWorkspacePanel` | Inactive/Select/Calibrate |
| `selectedSlot` | uint8 | aktuell lokal gewaehlter Slot |
| `selectionDirty` | bool | gewaehlter Slot weicht vom aktiven Druckslot ab |
| `selectedItem` | `PaperCalibrationItem` | gerade markiertes CAL-Item |
| `itemCount` | uint8 | Anzahl sichtbarer CAL-Items |
| `editingActive` | bool | CAL-Werteditiermodus aktiv |
| `parametersDirty` | bool | staged Profil weicht vom gespeicherten Slotprofil ab |
| `persistFailed` | bool | letzter CAL-Speicherversuch fehlgeschlagen |
| `panelAgeMs` | uint32 | Alter des letzten Panel-Wechsels |
| `selectedSlotAgeMs` | uint32 | Alter der letzten Slotwahl |
| `selectedItemAgeMs` | uint32 | Alter des letzten CAL-Item-Wechsels |
| `storageErrorCode` | uint8 | sichtbarer Storage-Fehlercode |
| `storageErrorDetail` | uint32 | erweiterter Storage-Fehlerdetail |
| `stagedProfile` | `PaperCalibrationRuntimeState` | lokal bearbeiteter Arbeitssatz fuer CAL |

### 7.4b `PaperCalibrationRuntimeState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `available` | bool | staged Profil fuer den gewaehlten Slot vorhanden |
| `calibrated` | bool | Kalibrierstatus des staged Profils |
| `gradeMode` | `PaperGradeMode` | FixedGrade oder Multigrade |
| `useIsoMath` | bool | ISO- statt LUT-/Mischpfad aktiv |
| `fixedGradeValue` | float | feste Gradation fuer Fixed-Grade-Papier |
| `isoP` | float | ISO-P Parameter |
| `isoR` | float | ISO-R Parameter |
| `kBw` | float | BW-K-Faktor |
| `kSoft` | float | Soft-K-Faktor |
| `kHard` | float | Hard-K-Faktor |

### 7.5 `SetupModeRuntimeState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `selectedItem` | `SetupMenuItem` | gerade markierter Eintrag |
| `itemCount` | uint8 | Gesamtzahl aktiver Items |
| `editingActive` | bool | Wert-Editiermodus aktiv |
| `parametersDirty` | bool | ungespeicherte Aenderungen |
| `persistFailed` | bool | letzter Speicherversuch fehlgeschlagen |
| `selectedItemAgeMs` | uint32 | Alter des letzten Item-Wechsels |
| `storageErrorCode` | uint8 | SD-Fehlercode (0=kein Fehler) |
| `storageErrorDetail` | uint32 | erweiterter Fehlerdetail |
| `stagedSettings` | `SystemSettings` | noch nicht gespeicherter Arbeitssatz |

`SetupMenuItem`-Werte und UI-Labels:

| Wert | Index | Label |
| --- | --- | --- |
| `SoundMode` | 0 | Sound-Modus |
| `SoundVolume` | 1 | Lautstaerke |
| `Vibration` | 2 | Vibration |
| `MaxHeadBrightness` | 3 | Max. Kopfhelligkeit |
| `ThermalDeratingStart` | 4 | Thermik-Drosselgrenze |
| `ThermalHardStop` | 5 | Thermik-Abschaltgrenze |
| `Apply` | 6 | Aktion: Speichern |
| `Discard` | 7 | Aktion: Verwerfen |
| `SafetyDefaults` | 8 | Aktion: Sicherheitsstandard |

### 7.6 `SystemSettings`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `soundMode` | `SoundFeedbackMode` | Off/Minimal/Full |
| `soundVolume` | `SoundVolumeLevel` | Low/Medium/High |
| `vibrationEnabled` | bool | Haptik-Feedback an/aus |
| `maxHeadBrightnessPercent` | float | globale NeoPixel-Helligkeitsgrenze 0.0-100.0 |
| `thermalProtection` | `ThermalProtectionSettings` | Thermikschwellen |

`ThermalProtectionSettings`:

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `deratingStartCelsius` | float | Temperaturschwelle fuer Leistungsreduktion |
| `hardStopCelsius` | float | Temperaturschwelle fuer sofortigen Stopp |

### 7.7 `ExposureRuntimeState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `phase` | `ExposurePhase` | Idle/PreWait/Exposing/Paused/PostWait/Done/Fault |
| `controlMode` | `ExposureControlMode` | None/Time/Dose |
| `faultReason` | `ExposureFaultReason` | Grund des Stopps |
| `sensorFallbackActive` | bool | Dose laeuft ohne geschlossenen Regelkreis |
| `sensorFallbackReason` | `ExposureFaultReason` | Grund des Sensor-Ausfalls |
| `faultLatched` | bool | Fault ist eingerastet bis manuelles Quittieren |
| `thermalDeratingActive` | bool | Thermische Leistungsreduktion aktiv |
| `phaseAgeMs` | uint32 | Alter der aktuellen Phase |
| `targetDose` | float | Soll-Dosis in Lux-Sekunden |
| `currentDose` | float | Ist-Dosis in Lux-Sekunden |
| `measuredLux` | float | aktueller Lux-Messwert |
| `remainingDose` | float | verbleibende Dosis |
| `remainingTimeSeconds` | float | verbleibende Zeit in Sekunden |
| `runtimeOutputLimit` | float | aktueller Ausgangs-Cap 0.0-1.0 |
| `runtimeHeadBusLatencyMs` | uint32 | sichtbare technische Head-Bus-Latenz in Millisekunden |

### 7.8 `MeasurementRuntimeStatus`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `activeLux` | `MeasurementLuxSample` | aktuell aktiver Messwert |
| `activeReference` | `MeasurementReferenceStatus` | Referenzmessung fuer EV-Berechnung |
| `activeRelativeEvValid` | bool | EV-Abstand gueltig |
| `activeRelativeEvStops` | float | relativer EV-Abstand in Blenden |
| `localLux` | `MeasurementLuxSample` | lokaler TSL2561-Messwert |
| `wirelessLux` | `MeasurementLuxSample` | Wireless-TSL2591-Messwert |
| `session` | `MeasurementSessionStatus` | Session-Zustand |

### 7.9 `MeasurementLuxSample`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `source` | `MeasurementLuxSource` | None/LocalTsl2561/WirelessGateway |
| `valid` | bool | Wert verwertbar |
| `lux` | float | Lux-Wert |
| `ageMs` | uint32 | Alter des letzten Samples |
| `sequence` | uint32 | monoton steigende Sample-Nummer |

### 7.10 `MeasurementSessionStatus`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `sampleCount` | uint32 | Gesamtanzahl angenommener Messungen |
| `capturedSampleCount` | uint32 | Anzahl tatsaechlich gespeicherter Messungen |
| `latestSample` | `MeasurementSessionSample` | juengster Session-Sample |
| `zoneHistogram` | array[11] uint8 | Haeufigkeit je Zonenbereich |
| `recentSamples` | array[8] | zuletzt gespeicherte Samples |
| `recentSampleCount` | uint8 | Anzahl valider Eintraege |
| `undoDepth` | uint8 | Anzahl moeglicher Undo-Schritte |
| `canUndo` | bool | Undo verfuegbar |

### 7.11 `MeasurementSessionSample`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `source` | `MeasurementLuxSource` | Messquelle |
| `lux` | float | Lux-Wert |
| `sequence` | uint32 | Sample-Nummer |
| `capturedAtMs` | uint32 | Erfassungszeitpunkt |
| `referenceLux` | float | Referenzwert zum Zeitpunkt der Messung |
| `relativeEvStops` | float | relativer EV-Abstand zur Referenz |
| `zoneIndex` | uint8 | Zonenindex 0-10 |
| `histogramWeight` | uint8 | Gewichtung im Histogramm |

### 7.12 `SensorRuntimeStatus`

`Tsl2561RuntimeStatus`:

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `health` | `SensorHealth` | Unknown/Ok/Stale/Fault |
| `sampleValidity` | `LuxSampleValidity` | Unknown/Valid/Invalid/Fault |
| `diagnosticReason` | `Tsl2561DiagnosticReason` | Fehlerursache |
| `initialized` | bool | Sensor erfolgreich initialisiert |
| `sampleFresh` | bool | letzter Wert ist aktuell |
| `sampleAgeMs` | uint32 | Alter des letzten Samples |
| `lux` | float | letzter Lux-Wert |

`Ds18b20RuntimeStatus`:

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `health` | `SensorHealth` | Unknown/Ok/Stale/Fault |
| `thermalState` | `ThermalState` | Unknown/Normal/Derating/Critical/Fault |
| `diagnosticReason` | `Ds18b20DiagnosticReason` | Fehlerursache |
| `initialized` | bool | Sensor initialisiert |
| `temperatureCelsius` | float | gemessene Temperatur |

### 7.13 `EspLinkRuntimeStatus`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `health` | `EspLinkHealth` | Unknown/Online/Stale/Lost |
| `heartbeatSeen` | bool | mindestens ein Heartbeat empfangen |
| `lastRxAgeMs` | uint32 | Alter des letzten empfangenen Frames |
| `lastTxAgeMs` | uint32 | Alter des letzten gesendeten Frames |
| `remoteUptimeMs` | uint32 | Uptime des ESP32-S3 |
| `remoteCapabilityBits` | uint32 | Capability-Flags des ESP |
| `serviceSensors` | `EspServiceSensorStatus` | DS18/AHT20/BMP280-Werte |
| `paperSlotRecovery` | `PaperSlotRecoveryStatus` | kompakte Recovery-Empfehlung fuer `paperslots.bin` und Backup |
| `wireless` | `WirelessGatewayStatus` | Wireless-Peer-Zustand |
| `lastInputEvent` | `EspRemoteInputStatus` | letztes Remote-Eingabeereignis aus Enc4 oder C6-Handbedienteil |
| `diagnostic` | `EspDiagnosticStatus` | letzte ESP-Diagnosemeldung |
| `txQueue` | `EspTxQueueStatus` | Sendepuffer-Zustand |
| `vfs` | `VfsBridgeStatus` | VFS-Transfer-Zustand |

`PaperSlotRecoveryStatus`:

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `flags` | uint16 | verdichtete Zustandsbits wie `decisionStable`, `backupBlobValid`, `restoreWouldChangeActive` |
| `recommendation` | `PaperSlotRecoveryRecommendation` | `RetryInspection/NoAction/KeepActive/OfferRestoreBackup/RecoverViaUpload` |
| `activeVfsStatus` | `VfsStatusCode` | letzter Fetch-Status fuer `paperslots.bin` |
| `activeParseError` | uint8 | Parse-Code des aktiven Blobs, falls lesbar aber inhaltlich ungueltig |
| `backupVfsStatus` | `VfsStatusCode` | letzter Fetch-Status fuer `paperslots.bin.bak` |
| `backupParseError` | uint8 | Parse-Code des Backup-Blobs, falls lesbar aber inhaltlich ungueltig |

`EspServiceSensorStatus`:

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `ds18TemperatureCelsius` | float | DS18B20-Temperatur ueber ESP-Pfad |
| `ds18SampleAgeMs` | uint32 | Alter des DS18-Samples |
| `ahtTemperatureCelsius` | float | AHT20-Umgebungstemperatur |
| `ahtHumidityPercent` | float | AHT20-Luftfeuchtigkeit |
| `bmpTemperatureCelsius` | float | BMP280-Temperatur |
| `bmpPressureHpa` | float | BMP280-Luftdruck in hPa |

### 7.14 `WirelessGatewayStatus`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `peerState` | `WirelessPeerState` | Unknown/Found/Bound/Active/Lost |
| `batteryPercent` | uint8 | Akku-Ladezustand C6-Terminal |
| `lastSeenAgeMs` | uint32 | Alter des letzten Wireless-Frames |
| `lastLux` | float | letzter C6-Lux-Messwert |
| `measurementSequence` | uint32 | Messequenz des C6 |
| `commandAckSequence` | uint32 | letzte bestaetigte Kommandosequenz |
| `renderStatusFlags` | uint16 | Render-Zustand des C6 |
| `staleRenderCount` | uint32 | Anzahl nicht rechtzeitig gerenderter Frames |
| `renderTimeoutCount` | uint32 | Anzahl Render-Timeout-Episoden |

### 7.15 `RemoteCommandTrackerStatus`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `inFlightCount` | uint8 | aktuell offene kritische Kommandos |
| `retryCount` | uint32 | kumulierte Retries gesamt |
| `timeoutCount` | uint32 | kumulierte Timeouts gesamt |
| `saturationCount` | uint32 | kumulierte Queue-Ueberlaufe |
| `lastTrackedSequence` | uint32 | letzte verfolgte Sequenz |
| `lastRetrySequence` | uint32 | letzte Sequenz mit Retry |
| `lastTimedOutSequence` | uint32 | letzte Sequenz mit Timeout |
| `lastSaturatedSequence` | uint32 | letzte Sequenz bei Saettigung |

### 7.16 `LightState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `focusSwitchRaw` | bool | Fokus-Taster Rohzustand |
| `saveSwitchRaw` | bool | Save-Taster Rohzustand |
| `roomSwitchRaw` | bool | Raumlicht-Taster Rohzustand |
| `saveLatchActive` | bool | Schutzsperr-Latch aktiv |
| `derivedFocusOutput` | bool | abgeleiteter Fokus-Ausgang |
| `derivedSaveOutput` | bool | abgeleiteter Save-Ausgang |
| `derivedRoomOutput` | bool | abgeleiteter Raumlicht-Ausgang |

### 7.17 `TouchState`

| Feld | Typ | Bedeutung |
| --- | --- | --- |
| `active` | bool | Touch-Punkt ueber Mindestdruckschwelle |
| `rawX` | int16 | Rohkoordinate X (unkalibriert) |
| `rawY` | int16 | Rohkoordinate Y (unkalibriert) |
| `rawZ` | int16 | Rohdruck Z |

---

## 8. Verbindliche Namenskonventionen fuer EEZ

- Seiten: `Page` + PascalCase, z.B. `PageSplitgrade`
- Komponenten: `Comp` + PascalCase, z.B. `CompMessageBand`
- Variablen: `<namespace>_<camelCase>`, z.B. `sg_grade`, `global_overlayText` (EEZ Studio akzeptiert keine Punkte in Variablennamen)
- Aktionen: `action_` + snake_case, z.B. `action_setup_apply`
- Keine Abkuerzungen ausser den bereits in den Firmware-Typen definierten
  (Sg, Bw, Rmt, Esp, Vfs)
- Keine numerischen Suffixe wie `label1`, `text2`

---

## 9. Abhaengigkeiten und offene Datenluecken

### 9.1 Heute schon mit Runtime-Daten vorhanden

- `PageSplitgrade` (Schritt 5): alle Runtime-Felder vorhanden; produktive
  Darstellung derzeit handgeschrieben.
- `PageSetup` (Schritt 6): alle Runtime-Felder vorhanden; produktive Darstellung
  derzeit handgeschrieben; Touch-Aktionsvertrag noch offen.
- Exposure-Overlay (Schritt 7): alle Runtime-Felder vorhanden; Busy-Screen ist
  sicherheitskritische handgeschriebene Runtime.
- `PageMeasurement` (Schritt 8): alle Runtime-Felder vorhanden; Messwerttexte
  muessen aus `LvglUi.cpp` in Presenter/Formatter zurueckgefuehrt werden.
- `PageWirelessRemote` (Schritt 9): alle Runtime-Felder und ein Screen existieren;
  Navigationspfad/Service-Geste sind noch offen.
- `PageBootStatus` (Schritt 10): alle Runtime-Felder vorhanden; produktive
  Darstellung derzeit handgeschrieben.

### 9.2 Seiten mit offenen Datenluecken

| Seite | Fehlende Voraussetzung |
| --- | --- |
| `PageBlackWhite` | Eigene `BwModeRuntimeState`-Felder fehlen noch; der Placeholder wertet das aktive Papierprofil (`gradeMode`, `fixedGradeValue`, `useIsoMath`) jetzt bereits sichtbar aus, aber der echte BW-Workflow und sein eigener Runtime-Vertrag fehlen weiter |
| `PagePaperWorkspace` / `PagePaperProfile` | Der Snapshot transportiert aktive Papierdetails plus Slot-Summaries; `ModeId::Paper` und `PaperWorkflow` machen `SELECT` produktiv navigierbar, und `CAL` bearbeitet jetzt die Kernfelder des gewaehlten Slots lokal mit Apply/Discard. Weiterhin offen sind messgetriebener Kalibrierwizard, LUT-/Preflash-Vollbearbeitung und die finale EEZ-gefuehrte Seite statt handgeschriebenem Layout |
| `PageDensitometer` | Eigener Runtime-/Servicepfad fuer Durchlicht-Densitometrie fehlt; `PageMeasurement` deckt bisher nur Lux, Referenz und Zonenhistogramm ab |
| `PageBurn` | Eigene Runtime-Felder und ein klarer Rueckkanal aus dem bestehenden Druckkontext fehlen noch; bis dahin nur als konzeptueller PRINT-Helfer behandeln |

Diese Seiten koennen als Layout mit Platzhaltervariablen angelegt werden,
duerfen aber nicht verdrahtet werden bis die fehlenden Firmware-Pfade existieren.

### 9.3 Bekannte UI-Landminen vor EEZ-Detailarbeit

Diese Punkte muessen vor der finalen EEZ-Ausarbeitung entweder behoben oder als
expliziter Uebergangsvertrag markiert sein:

- ~~EEZ-Projekt und Runtime verwenden noch nicht dieselbe
  `global_activeWorkflowFamily`-Nummerierung.~~
  **Erledigt (Prompt A1):** `computeWorkflowFamily()` und alle EEZ-Conditions
  verwenden jetzt einheitlich `0=PAPER`, `1=MEAS`, `2=PRINT`, `3=SETUP`.
- `styles.c` enthaelt runtime-seitig keine Style-Definitionen, obwohl das
  EEZ-Projekt Styles und Farben designseitig fuehrt.
- `flow::setGlobalVariable(...)` befuellt den Variablenkatalog, aber sichtbare
  Texte und Farben kommen weiterhin grossteils aus direkten LVGL-Handle-Updates.
- `PageWirelessRemote` ist als Seite vorhanden, aber nicht per Runtime-
  Navigation erreichbar.
- ModeTabs und Setup-Buttons duerfen erst dann als produktive Touch-Flaechen
  gelten, wenn Action, Firmware-Guard und Encoder-Alternative dokumentiert und
  implementiert sind.
- Messwert-Usertexte in `LvglUi.cpp` sind ein Uebergangsrest und verletzen die
  Architekturgrenze aus Abschnitt 2.2.
- Farbwerte sind noch zwischen Doku, EEZ-Projekt, `screens.c` und `LvglUi.cpp`
  dupliziert. EEZ-Styling darf erst als verbindlich gelten, wenn diese
  Duplikate fuer die betroffene Seite nicht mehr still ueberschreiben.

---

## 10. Was EEZ Studio in Part2 ausdruecklich nicht tun darf

- GPIOs oder Hardware-Treiber direkt ansprechen
- `SensorManager`, `ExposureEngine`, `EspServiceLink` oder `main.cpp` direkt
  referenzieren
- EV-, Dosis-, Thermik- oder Sicherheitsberechnungen selbst ausfuehren
- `float`-zu-String-Formatierung fuer Messwerte (Aufgabe von `UiPresenter`)
- Encoder- oder Touch-Routing (Aufgabe von `InputNormalizer` und
  `InputRouterPolicy`)
- Modal-Guards oder Eingabesperren setzen (Aufgabe von `ModeCoordinator`)
- Zweite Wahrheit neben `SystemSnapshot` aufbauen
- Seitenspezifische Sonderregeln fuer Encoder-Richtung oder Touch-Zonenlogik
  erfinden

---

## 11. Verwandte Dokumente

| Dokument | Zweck |
| --- | --- |
| [history/dukatimer-part2-lvgl-eez-dma-basis.md](history/dukatimer-part2-lvgl-eez-dma-basis.md) | LVGL-/DMA-Architektur, Basisdoku |
| [offene_ziele.md](offene_ziele.md) | Produktzielseiten fuer UI |
| [explizites-setup-menue-zielvorgabe.md](explizites-setup-menue-zielvorgabe.md) | Zielumfang Setup-Menue |
| [offene_punkte.md](offene_punkte.md) | Aktueller offener Reststand, UI-Abschnitt |
| [src/teensy/LvglUi.cpp](../../src/teensy/LvglUi.cpp) | heutiger UI-Integrationspunkt |
| [src/teensy/UiPresenter.h](../../src/teensy/UiPresenter.h) | Presenter-API |
| [src/teensy/SystemSnapshot.h](../../src/teensy/SystemSnapshot.h) | UI-Datencontainer |
| [src/teensy/ModeRuntimeState.h](../../src/teensy/ModeRuntimeState.h) | Modus- und Workflow-Zustand |
| [src/teensy/ExposureRuntimeState.h](../../src/teensy/ExposureRuntimeState.h) | Belichtungszustand |
| [src/teensy/MeasurementRuntimeStatus.h](../../src/teensy/MeasurementRuntimeStatus.h) | Messtatus |
| [src/teensy/EspLinkRuntimeStatus.h](../../src/teensy/EspLinkRuntimeStatus.h) | Gateway-/Link-Zustand |
| [src/teensy/SystemSettings.h](../../src/teensy/SystemSettings.h) | Globale Setup-Daten |

---

## 12. Touch-Kontrakt: ModeTabs und Setup-Actions

*Erstellt in Prompt A6. Dieser Abschnitt ist verbindlich, bevor Touch-Felder in
EEZ verfeinert werden.*

### 12.1 Kontrakt-Tabelle

| UI-Feld | Screen | Groesse px | Action-Name | Encoder-Alternative | Firmware-Guard | Ist-Stand |
| --- | --- | --- | --- | --- | --- | --- |
| Tab PAPER | alle Hauptseiten (y=284) | 120×36 | `TouchTabFamily(0)` | Enc1-Press (Platzhalter) | `shouldDispatch()` Open; kein Modal-Lock | Offener Hook — kein CLICKABLE auf Container, kein Callback |
| Tab MEAS | alle Hauptseiten (y=284) | 120×36 | `TouchTabFamily(1)` | (kein Enc-Shortcut belegt) | `shouldDispatch()` Open; kein Modal-Lock | Offener Hook; `ModeId::Measurement` fehlt (siehe 12.2) |
| Tab PRINT | alle Hauptseiten (y=284) | 120×36 | `TouchTabFamily(2)` | Enc2-Press (Platzhalter) | `shouldDispatch()` Open; kein Modal-Lock | Offener Hook |
| Tab SETUP | alle Hauptseiten (y=284) | 120×36 | `TouchTabFamily(3)` | Enc3 LongPress | `shouldDispatch()` Open; kein Modal-Lock | Offener Hook |
| UEBERNEHMEN (Apply) | `PageSetup` (Edit-Block) | 140×56 | `TouchSetupApply` | Enc3-Press auf `SetupMenuItem::Apply` | `shouldDispatch()` Open; blockiert bei `WorkflowWait/Confirm/Fault` | Offener Hook — CLICKABLE gesetzt, kein Callback |
| VERWERFEN (Discard) | `PageSetup` (Edit-Block) | 140×56 | `TouchSetupDiscard` | Enc3-Press auf `SetupMenuItem::Discard` | wie Apply | Offener Hook — CLICKABLE gesetzt, kein Callback |
| SICHERHEITSSTANDARDS (Safety) | `PageSetup` (Edit-Block) | 186×56 | `TouchSetupSafety` | Enc3-Press auf `SetupMenuItem::SafetyDefaults` | wie Apply | Offener Hook — CLICKABLE gesetzt, kein Callback |
| PAUSE | Busy (run_overlay) | 460×52 | `UiModalAction::Pause (3)` | Start-Taste | `WorkflowWait`: Pause-Aktion erlaubt | **Verdrahtet** — `busy_btn_cb` → `lvgl_ui_modal_action_callback` |
| FORTSETZEN | Busy (pause_overlay) | 215×52 | `UiModalAction::Resume (1)` | Start-Taste | `WorkflowWait`: Resume-Aktion erlaubt | **Verdrahtet** |
| ABBRECHEN | Busy (pause_overlay) | 215×52 | `UiModalAction::Abort (2)` | (kein direkter Encoder-Shortcut) | `WorkflowWait`: Undo-Aktion erlaubt | **Verdrahtet** |

**Hinweis Touch-Zonengroesse:** Alle Felder ausser den ModeTabs erfuellen die
48×48 px-Mindestanforderung (Abschnitt 5.2). Die ModeTabs (120×36 px)
unterschreiten die Mindesthoehe um 12 px. Das ist eine bekannte geometrische
Einschraenkung: der Tab-Bar-Bereich belegt y=284..319, also genau 36 px. Im
naechsten Layoutzyklus sind zwei Loesungsoptionen zu pruefen: (a) Tab-Bar auf
y=272 anheben (h=48), (b) unsichtbare Touch-Erweiterungszone nach oben.

### 12.2 ModeTabs: Verdrahtungsregeln

**Was ein ModeTab-Touch darf:**

- Einen Familienwechsel bei `ModeCoordinator` anfragen:
  `ModeCoordinator::requestMode(ModeId, nowMs)`.
- Das Mapping lautet:
  - Tab 0 (PAPER)  → `requestMode(ModeId::Paper, nowMs)`
  - Tab 1 (MEAS)   → *offen* — kein `ModeId::Measurement` in der Enum vorhanden;
    Routing nach `PageMeasurement` muss als eigener `ModeId` oder als
    Familien-Routing-Shortcut geklart werden, bevor dieser Tab verdrahtet wird.
  - Tab 2 (PRINT)  → `requestMode(ModeId::Splitgrade, nowMs)` oder den zuletzt
    genutzten PRINT-Modus; PRINT-Modus-Persistenz noch nicht implementiert.
  - Tab 3 (SETUP)  → `requestMode(ModeId::Setup, nowMs)` (entspricht dem
    bestehenden Enc3-LongPress-Pfad in `main.cpp`).

**Was ein ModeTab-Touch nicht darf:**

- EV-, Dosis- oder Belichtungsparameter direkt aendern.
- Fachlogik innerhalb von `screens.c` oder im LVGL-Callback aufrufen.
- Den `InputRouterPolicy`-Guard umgehen.
- Bei aktivem Modal-Zustand (`WorkflowWait`, `WorkflowConfirm`,
  `WorkflowFault`) ausgeloest werden — `shouldDispatch()` blockiert den
  Familienwechsel automatisch, da er keine der erlaubten Modal-Aktionen ist.

**Implementierungsmuster (noch nicht umgesetzt):**

Das Busy-Modal-Muster (Abschnitt 12.4) dient als Referenz. Fuer ModeTabs sind
folgende Schritte erforderlich:

1. `build_modetabs()` in `screens.c`: `LV_OBJ_FLAG_CLICKABLE` auf den
   **Container** (`tab`, nicht das Label) setzen; neuen
   `static void tab_btn_cb(lv_event_t *e)` registrieren, der
   `extern void lvgl_ui_tab_action_callback(uint8_t familyIndex)` aufruft.
2. `LvglUi.cpp`: `extern "C" void lvgl_ui_tab_action_callback(uint8_t)` analog
   zu `lvgl_ui_modal_action_callback`; `pendingTabFamily_` als
   Zwischenspeicher.
3. `LvglUi.h`: `pollTabFamily()` und `setPendingTabFamily()`.
4. `main.cpp` `loop()`: `emitLocalTabFamilySwitch(ui.pollTabFamily(), nowMs)`,
   das ein `NormalizedInputEvent` mit `NormalizedInputSource::LocalModalButton`
   und einem neuen oder bestehenden `InputSemanticActionKind::FamilySwitch`
   bildet und `dispatchNormalizedInputEvent()` aufruft.
5. `ModeCoordinator::handleInputEvent()` reagiert auf `FamilySwitch` mit
   `requestMode()`.

**ModeId-Luecke fuer MEAS:**

`ModeId` kennt aktuell: `None`, `Splitgrade`, `BlackWhite`, `Setup`, `Paper`.
Ein `ModeId::Measurement` existiert nicht. Bis das geklart ist, bleibt Tab 1
(MEAS) ein dokumentierter offener Hook ohne produktive Touch-Verdrahtung. Die
anderen drei Tabs koennen unabhaengig davon implementiert werden.

### 12.3 Setup-Actions: Verdrahtungsregeln

**Status:** Setup-Buttons haben bereits `LV_OBJ_FLAG_CLICKABLE` in
`create_screen_page_setup()` (screens.c Z. 673, 692, 711), aber keinen
`lv_obj_add_event_cb`-Callback.

**Was der Touch darf:**

- Dieselbe fachliche Wirkung wie der entsprechende Encoder-Pfad ausloesen:
  - UEBERNEHMEN → entspricht Enc3-Press auf `SetupMenuItem::Apply`
  - VERWERFEN    → entspricht Enc3-Press auf `SetupMenuItem::Discard`
  - SICHERHEITSSTANDARDS → entspricht Enc3-Press auf `SetupMenuItem::SafetyDefaults`
- Nur ausloesen, wenn der Button sichtbar ist (Sichtbarkeitsbedingungen
  bereits in `LvglUi.cpp` implementiert, Z. 1043-1054):
  - Apply und Discard: nur wenn `setup.editingActive == true`
  - SafetyDefaults: nur wenn `setup.parametersDirty == true`

**Was der Touch nicht darf:**

- `SystemSettings` direkt beschreiben.
- Den `InputRouterPolicy`-Guard umgehen.
- Bei aktivem Modal-Zustand ausloesen (wird von `shouldDispatch()` blockiert).

**Implementierungsmuster (noch nicht umgesetzt):**

1. `create_screen_page_setup()` in `screens.c`: neuen
   `static void setup_btn_cb(lv_event_t *e)` registrieren, der
   `extern void lvgl_ui_setup_action_callback(uint8_t actionId)` aufruft
   (user_data: 1=Apply, 2=Discard, 3=SafetyDefaults).
2. `LvglUi.cpp`: `extern "C" void lvgl_ui_setup_action_callback(uint8_t)` mit
   `pendingSetupAction_` (neues Enum `UiSetupAction`: `None=0, Apply=1,
   Discard=2, SafetyDefaults=3`).
3. `LvglUi.h`: `pollSetupAction()` und `setPendingSetupAction()`.
4. `main.cpp` `loop()`: `emitLocalSetupAction(ui.pollSetupAction(), nowMs)`,
   das ein `NormalizedInputEvent` mit `NormalizedInputSource::LocalModalButton`
   und dem passenden `InputSemanticActionKind` (Confirm fuer Apply, Undo fuer
   Discard) bildet und `dispatchNormalizedInputEvent()` aufruft.
5. `ModeCoordinator::handleInputEvent()` behandelt die Aktionen identisch zum
   Encoder-Pfad.

### 12.4 Busy-Modal-Buttons: Referenz-Implementierung

Die Busy-Modal-Buttons (Pause, Resume, Abort) sind die einzigen produktiv
verdrahteten Touch-Aktionen im System. Sie dienen als Implementierungsreferenz
fuer alle offenen Hooks.

**Vollstaendiger Signalpfad:**

```text
LVGL LV_EVENT_CLICKED
  -> busy_btn_cb (screens.c, static FLASHMEM)
  -> lvgl_ui_modal_action_callback(uint8_t action) [extern "C", LvglUi.cpp Z. 119]
  -> LvglUi::setPendingModalAction(decodeUiModalAction(action))
  -> [naechster main.cpp-Looptick] LvglUi::pollModalAction() [LvglUi.cpp Z. 176]
  -> emitLocalModalButtonAction(action, nowMs) [main.cpp Z. 1287]
  -> emitLocalButtonEvent(LocalModalButton, Press, encodeUiModalAction(action), nowMs)
  -> dispatchNormalizedInputEvent()
  -> InputNormalizer::normalize(): Pause->Pause, Resume->Resume, Abort->Undo
  -> InputRouterPolicy::shouldDispatch() [Guard-Check]
  -> ModeCoordinator::handleInputEvent()
```

**Guard-Verhalten im `WorkflowWait`-Modal
(`modalStateBlocksEvent`, InputRouterPolicy.cpp):**

- FORTSETZEN (Resume=1) → `InputSemanticActionKind::Resume` → erlaubt
- ABBRECHEN (Abort=2)   → `InputSemanticActionKind::Undo`   → erlaubt
- PAUSE (Pause=3)       → `InputSemanticActionKind::Pause`  → erlaubt
- Alle anderen Events aus dieser Quelle → blockiert

**Beachten:** `UiModalAction::Abort` wird im `InputNormalizer` bewusst auf
`InputSemanticActionKind::Undo` gemappt, damit globale Sicherheitsausstiege
keinen zweiten Workflowvertrag neben Undo aufmachen
(`InputNormalizer.cpp` Z. 107-111).

### 12.5 Offene Hooks und Abnahmebedingungen

| Hook | Was fehlt | Abnahmebedingung |
| --- | --- | --- |
| ModeTab PAPER Touch | CLICKABLE auf Container, Callback, `lvgl_ui_tab_action_callback` | Vollstaendiger Signalpfad bis `requestMode(Paper)` nachgewiesen |
| ModeTab MEAS Touch | Zusaetzlich: `ModeId::Measurement` oder Routing-Shortcut | Erst nach Klaerung der MEAS-ModeId-Luecke |
| ModeTab PRINT Touch | CLICKABLE auf Container, Callback | PRINT-Modus-Persistenz und Rueckkehrlogik geklart |
| ModeTab SETUP Touch | CLICKABLE auf Container, Callback | Abgrenzung zu Enc3-LongPress-Pfad dokumentiert |
| Setup Apply Touch | Callback, `UiSetupAction`-Enum, `emitLocalSetupAction` in main.cpp | Gleiche Guard-Logik wie Encoder-Pfad verifiziert |
| Setup Discard Touch | wie Apply | wie Apply |
| Setup SafetyDefaults Touch | wie Apply | wie Apply |

Kein Feld aus dieser Tabelle gilt als produktiv, bevor es den vollstaendigen
Signalpfad bis `ModeCoordinator::handleInputEvent()` oder
`ModeCoordinator::requestMode()` besitzt und der `InputRouterPolicy`-Guard
korrekt integriert ist.

---

## 13. EEZ-Export- und Merge-Vertrag

*Erstellt in Prompt B0. Dieser Abschnitt ist verbindlich, bevor irgendeine
Seite zur EEZ-gefuehrten Runtime migriert wird.*

### 13.1 Datei-Klassifikation

Alle Dateien liegen in
`src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/`.
EEZ Studio verwaltet die Dateiliste in `.eez-project-build`.

| Datei | Groesse | Typ | Inhalt |
| --- | --- | --- | --- |
| `eez-flow.cpp` | 370 KB | **Generiert** (eez-framework) | EEZ-Runtime; Autogenerated-Header datiert 2026-04-06, Commit d9ff0db4 |
| `eez-flow.h` | 160 KB | **Generiert** (eez-framework) | EEZ-Runtime-Header; `EEZ_FOR_LVGL 1` definiert hier |
| `ui.c` | 25 KB | **Generiert** (EEZ-Export) | Binaer-`assets[]`-Blob (serialisierter Flow) + `ui_init()` + `ui_tick()` |
| `ui.h` | 260 B | **Generiert** (EEZ-Export) | Deklarationen fuer `ui_init()`, `ui_tick()`, `assets[]` |
| `vars.h` | 8.8 KB | **Generiert** (EEZ-Export) | `FlowGlobalVariables`-Enum mit 155 Eintraegen (Stand: Prompt A1) |
| `images.c` | 64 B | **Generiert** (EEZ-Export) | Leeres Bild-Array |
| `images.h` | 389 B | **Generiert** (EEZ-Export) | Bild-Typen |
| `structs.h` | 199 B | **Generiert** (EEZ-Export) | Minimaler Wrapper; importiert `eez-flow.h` + `vars.h` |
| `styles.c` | 96 B | **Generiert** (EEZ-Export) | Leer — keine Runtime-Styles implementiert |
| `styles.h` | 189 B | **Generiert** (EEZ-Export) | Leerer Header |
| `actions.h` | 189 B | **Generiert** (EEZ-Export) | Leerer Header — keine EEZ-Actions definiert |
| `fonts.h` | 376 B | **Generiert** (EEZ-Export) | Font-Forward-Deklarationen |
| `screens.h` | 7.3 KB | **Gemischt** | EEZ-generiert: `ScreensEnum`, `objects_t`; **handgeschrieben:** `DukaWidgets`-Struct (~60 Widget-Handle-Pointer) |
| `screens.c` | 67 KB | **Handgeschrieben** (in EEZ-Build-Liste) | Color-Defines, `build_header()`, `build_modetabs()`, alle `create_screen_*()`, `busy_btn_cb()`, `extern lvgl_ui_modal_action_callback` |

**Befund: kein `outputFolder` konfiguriert.** Das EEZ-Projekt
(`DukatimerPart2TeensyUi.eez-project`) enthaelt keinen `outputFolder`-Eintrag
in `settings.general`. EEZ Studio schreibt beim Export in das Standardverzeichnis
`src/ui/` relativ zur `.eez-project`-Datei, also genau in den Ordner, in dem
`screens.c` und alle anderen Dateien liegen. Bevor irgendein Export ausgefuehrt
wird, muss das Output-Verzeichnis in EEZ Studio explizit bestaetigt oder
konfiguriert werden.

### 13.2 Exportvertrag

| Datei | EEZ darf ueberschreiben | Bedingung |
| --- | --- | --- |
| `eez-flow.cpp` | Ja | Nur beim Update des eez-framework; nicht bei jedem Layout-Export |
| `eez-flow.h` | Ja | Nur beim Update des eez-framework |
| `ui.c` | **Ja** | Bei jedem EEZ-Export (`assets[]`-Blob aendert sich mit dem Projekt) |
| `ui.h` | Ja | Bei jedem EEZ-Export |
| `vars.h` | **Ja** | Bei jedem EEZ-Export; nach Export `pio run -e teensy41` obligatorisch |
| `images.c` | Ja | Bei jedem EEZ-Export |
| `images.h` | Ja | Bei jedem EEZ-Export |
| `structs.h` | Ja | Bei jedem EEZ-Export |
| `styles.c` | Ja | Bei jedem EEZ-Export; bleibt leer bis Styles in EEZ gebaut sind |
| `styles.h` | Ja | Bei jedem EEZ-Export |
| `actions.h` | Ja | Bei jedem EEZ-Export |
| `fonts.h` | Ja | Bei jedem EEZ-Export |
| `screens.h` | **NEIN — nur nach manuellem Merge** | Enthalt `DukaWidgets`-Struct; EEZ wuerden den Struct loeschen |
| `screens.c` | **NEIN — nur nach manuellem Merge** | Vollstaendig handgeschrieben; EEZ wuerde alle Color-Defines, build_*()-Hilfsfunktionen und Callbacks loeschen |

**Grundregel:** Ein EEZ-Export darf nur ausgefuehrt werden, wenn `screens.c`
und `screens.h` vorher gesichert und danach manuell mit dem EEZ-Output
zusammengefuehrt wurden. Solange beide Dateien handgeschrieben sind, gilt:
**EEZ exportiert nur die uebrigen 12 Dateien**.

### 13.3 Export-Konfiguration (Ersteinrichtung)

Bevor EEZ Studio erstmals einen produktiven Export ausfuehrt:

1. EEZ Studio oeffnen: `Datei → Einstellungen → Ausgabepfad` (oder
   `settings.general.outputFolder` im Projekt-JSON).
2. Ausgabepfad explizit auf
   `src/ui` (relativ zur `.eez-project`-Datei) setzen.
3. Testen mit einem Probeexport in ein leeres Verzeichnis, um zu prueefen,
   welche Dateien EEZ tatsaechlich schreibt, bevor das Ziellverzeichnis
   beruehrt wird.
4. Git-Status vor und nach dem Export pruefen (`git diff --name-only`), um
   unerwartete Aenderungen zu erkennen.

### 13.4 Merge-Vertrag fuer screens.c und screens.h

Solange `screens.c` handgeschrieben bleibt:

- EEZ-generiertes `screens.c` darf **nicht** direkt in das Repository
  uebernommen werden.
- Der Merge-Prozess fuer eine Pilotseite ist:
  1. EEZ exportiert in ein **separates Staging-Verzeichnis** (nicht
     direkt in `src/ui/`).
  2. Diff des EEZ-generierten `screens.c` gegen das handgeschriebene.
  3. Nur den generierten Block der Pilotseite (`create_screen_boot()` o.ae.)
     in das handgeschriebene `screens.c` integrieren.
  4. Handgeschriebene Teile (Color-Defines, build_*(), Callbacks, DukaWidgets)
     erhalten bleiben.
  5. Build-Validierung: `pio run -e teensy41`.

Fuer `screens.h`:

- Der EEZ-generierte Teil (`ScreensEnum`, `objects_t`) ist stabil und aendert
  sich selten.
- Der handgeschriebene Teil (`DukaWidgets`) muss nach jedem EEZ-Export
  manuell zurueckgefuehrt werden.
- Langfristige Loesung: `DukaWidgets` in eine separate Datei
  `screens_ext.h` auslagern, die EEZ nie beruehrt. Dies ist ein
  spaeterer Schritt (nicht in B0 umzusetzen).

### 13.5 Pilotseiten-Empfehlung: `PageBootStatus` (Boot)

**Empfehlung: `PageBootStatus` als erste Pilotseite.**

Begruendung:

| Kriterium | Bewertung |
| --- | --- |
| Exposure-Kritikalitaet | Keine — Boot laeuft nur bei `ModeId::None`, kein Produktivpfad |
| Touch-Felder | Keine — rein passive Anzeige |
| Widget-Komplexitaet | Sehr gering — 3 Labels (title, subtitle, status) |
| EEZ-Variablen | 8 Boot-spezifische Variablen (Index 147-154 in `vars.h`): `BOOT_TITLE_TEXT`, `BOOT_SUBTITLE_TEXT`, `BOOT_ACTIVE_MODE`, `BOOT_LINK_HEALTH`, `BOOT_PAPER_SLOT_STORAGE_ERROR_CODE`, `BOOT_PAPER_SLOT_STORAGE_ERROR_DETAIL`, `BOOT_REMOTE_CAPABILITY_BITS`, `BOOT_TOUCH_ACTIVE` |
| Rueckfall bei Fehler | Sicher — bei Fehler zeigt Boot nur weissen Hintergrund; kein Dunkelkammer-Workflow betroffen |
| Header / ModeTabs | Keine — Boot hat weder `build_header()` noch `build_modetabs()` |
| Bestehende EEZ-Variablen-Verdrahtung | Teilweise — `LvglUi.cpp` setzt bereits direkt per `lv_label_set_text_static(w.boot_title_lbl, ...)` |

**Migrationspfad fuer Boot (B1) — geplant:**

1. EEZ Studio: `create_screen_boot()` so gestalten, dass es die 3 Labels
   an denselben Positionen und mit denselben Style-Parametern erzeugt wie
   das handgeschriebene `create_screen_boot()`.
2. EEZ exportiert in Staging-Verzeichnis; diff gegen handgeschriebenes
   `screens.c`.
3. Nur den generierten `create_screen_boot()`-Block in das handgeschriebene
   `screens.c` uebernehmen.
4. `LvglUi.cpp` weiterhin ueber `w.boot_*_lbl` aktualisieren (keine
   Flow-Global-Variable-Umstellung in diesem Schritt).
5. `pio run -e teensy41` — build muss ohne neue Warnungen durchgehen.
6. Sichtbarkeitsabnahme: Boot-Screen zeigt Titel, Untertitel und Status
   korrekt.

---

### 13.7 B1-Ergebnis: Boot-Migration BLOCKIERT (Zwischenabnahme dokumentiert)

*Erstellt in Prompt B1 (2026-05-08). `pio run -e teensy41` → SUCCESS 27 s.*

**Entscheidung: BLOCKIERT fuer vollstaendige EEZ-gefuehrte Runtime.**

#### Blocker (spezifisch)

| # | Blocker | Datei | Details |
| --- | --- | --- | --- |
| 1 | `create_screen_boot()` ist handgeschrieben | `screens.c` | Registriert keine EEZ Widget-Binding-States fuer `boot_title_lbl`, `boot_subtitle_lbl`, `boot_status_lbl`. EEZ-Framework (`eez-flow.cpp`) pflegt intern nur die 7 Screen-Root-Handles (`objects_t`), nicht die Sub-Widget-Handles in `g_duka_widgets`. |
| 2 | `tick_screen_boot()` ist ein Stub | `screens.c` | EEZ-generierter Code wuerde hier die Binding-Auswertung durchfuehren und `lv_label_set_text()` auf den gebundenen Labels aufrufen. Ohne EEZ-generierten `tick_screen_boot()` propagiert `eez_flow_tick()` keine Variablenaenderungen an die Boot-Labels. |
| 3 | EEZ-Export blockiert | `screens.c` | Exportpfad gemaess B0-Vertrag (Abschnitt 13.4): Export nur in Staging-Verzeichnis; danach Merge von nur `create_screen_boot()` in die handgeschriebene `screens.c`. Solange EEZ Studio nicht ausgefuehrt wurde, kein EEZ-generierter `create_screen_boot()` vorhanden. |

**Was die EEZ-Infrastruktur bereits liefert (ready, nicht aktiv):**

- EEZ-Projekt: Boot-Labels sind korrekt an `boot_titleText` (var 147),
  `boot_subtitleText` (var 148) und `global_overlayText` (var 2) gebunden.
- `LvglUi.cpp`: `flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_TITLE_TEXT, ...)`
  und `BOOT_SUBTITLE_TEXT` werden auf jedem Snapshot-Update geschrieben.
- `FLOW_GLOBAL_VARIABLE_GLOBAL_OVERLAY_TEXT` wird global geschrieben (nutzt die
  Boot-Statuslabel-Bindung sobald der EEZ-Pfad aktiv ist).

#### Zwischenabnahme (gueltig ab Prompt B1)

| Kriterium | Nachweis | Status |
| --- | --- | --- |
| Build | `pio run -e teensy41` → SUCCESS, 27 s, keine neuen Warnungen | **Bestanden** |
| `ModeId::None → SCREEN_ID_BOOT` | `computeTargetScreen()` in `LvglUi.cpp` Z. 71 | **Intakt** |
| `ModeId::Splitgrade → SCREEN_ID_PAGE_SPLITGRADE` | `computeTargetScreen()` in `LvglUi.cpp` Z. 81 | **Intakt** |
| Boot-Labels werden aktualisiert | Direktpfad: `pushWidgetsFromSnapshot()` via `g_duka_widgets.boot_*_lbl` | **Aktiv (Direktpfad)** |
| EEZ flow globals fuer Boot | `updateSnapshot()` schreibt `BOOT_TITLE_TEXT`, `BOOT_SUBTITLE_TEXT`, alle 8 Boot-Variablen | **Infrastruktur bereit** |
| Kein anderer Screen betroffen | Nur Kommentar-Aenderungen in `screens.c` und `LvglUi.cpp` | **Keine Seiteneffekte** |

#### Naechster Schritt fuer vollstaendige Boot-Migration

1. EEZ Studio oeffnen, Boot-Seite pruefen (Labels an `boot_titleText`,
   `boot_subtitleText`, `global_overlayText` gebunden — bereits korrekt).
2. EEZ Studio: Build/Export in ein **Staging-Verzeichnis** (nicht direkt `src/ui/`).
3. Aus dem generierten `screens.c` nur den `create_screen_boot()`-Block
   extrahieren und in die handgeschriebene `screens.c` integrieren.
4. `tick_screen_boot()` aus dem generierten Code uebernehmen — dieser enthaelt
   die Binding-Auswertung fuer die 3 Labels.
5. Den Direktpfad in `LvglUi::pushWidgetsFromSnapshot()` fuer Boot deaktivieren
   (erst nach erfolgreichem Build und Sichtabnahme).
6. `pio run -e teensy41` + Hardware-Abnahme: Boot zeigt Titel, Untertitel,
   Status korrekt.

**Naechste Kandidaten nach Boot:**

- `PageWirelessRemote` (B6): reine Diagnoseseite, keine Exposure-Logik,
  keine Touch-Actions (ausser spaeterem Ruecksprung).
- `PageMeasurement` (B5): bereits vorbereitete Presenter-Methoden (A5).

### 13.8 B2-Ergebnis: Paper-Workspace-Migration BLOCKIERT (Datenschicht vollstaendig)

*Erstellt in Prompt B2. `pio run -e teensy41` → SUCCESS 26 s.*

**Entscheidung: BLOCKIERT fuer vollstaendige EEZ-gefuehrte Runtime.**
Gleicher Grundblocker wie B1: `create_screen_page_paper_workspace()` ist handgeschrieben.

#### Blocker

| # | Blocker | Datei | Details |
| --- | --- | --- | --- |
| 1 | `create_screen_page_paper_workspace()` ist handgeschrieben | `screens.c` | Registriert keine EEZ-Widget-Binding-States fuer `paper_panel_lbl`, `paper_cal_row_cont/lbl/val[0-6]`, `paper_hint_lbl`, `paper_chip_select`, `paper_chip_cal`. |
| 2 | `tick_screen_page_paper_workspace()` ist ein Stub | `screens.c` | EEZ-generierter Code wuerde Binding-Auswertung und LVGL-Updates hier durchfuehren. Ohne EEZ-generierten Tick keine Propagation. |
| 3 | EEZ-Export blockiert | `screens.c` | Exportpfad gemaess B0-Vertrag (Abschnitt 13.4): Export in Staging; dann gezielter Merge. |
| 4 | Zwei CAL-Datenwerte fehlen in vars.h | `vars.h` / EEZ-Projekt | `PAPER_STEP_WHITE` und `PAPER_STEP_BLACK` (`paper.stepWhite`, `paper.stepBlack` aus `PaperModeRuntimeState`) existieren nicht in vars.h und koennen nicht sicher ohne EEZ-Projektaenderung hinzugefuegt werden. |

#### Widget-Inventar: Direktpfad vs. EEZ-Datenschicht

| Widget-Handle | Direktpfad (pushWidgets) | EEZ-Var vorhanden | Luecke |
| --- | --- | --- | --- |
| `paper_panel_lbl` | `lv_label_set_text()` formatierter String | PAPER_PANEL (22), PAPER_SELECTED_SLOT (23), PAPER_SLOT_COUNT (34), PAPER_ACTIVE_SLOT (35), PAPER_EDITING_ACTIVE (27), PAPER_PARAMETERS_DIRTY (28) | Kein `PAPER_PANEL_TEXT`; EEZ-Tick muesste selbst formatieren |
| `paper_cal_row_lbl[0-6]` | Static-String-Tabelle (CAL) / snprintf (SELECT) | PAPER_STAGED_* (41-49) fuer CAL; PAPER_SELECTED_SLOT_* fuer ausgewaehlten Slot | Keine Per-Zeile-Variablen; SELECT-Listenrendering benoetigt alle Slot-Summaries |
| `paper_cal_row_val[0-6]` | snprintf aus stagedProfile-Feldern / Slot-Summaries | PAPER_STAGED_* (41-49) fuer CAL | `PAPER_STEP_WHITE`, `PAPER_STEP_BLACK` fehlen in vars.h |
| `paper_cal_row_cont[0-6]` | `lv_obj_set_style_bg_color()` selected/editing | PAPER_SELECTED_ITEM (25), PAPER_EDITING_ACTIVE (27) | Nur Style-Aufrufe; kein EEZ-Var-Gap |
| `paper_hint_lbl` | Static strings (2 Faelle) | PAPER_PANEL (22), PAPER_EDITING_ACTIVE (27) | Kein `PAPER_HINT_TEXT` |
| `paper_chip_select` / `paper_chip_cal` | `lv_obj_set_style_text_color()` | PAPER_PANEL (22) | Nur Farbaenderung; kein EEZ-Var-Gap |

#### EEZ-Datenschicht (vollstaendig geschrieben in updateSnapshot())

33 Flow-Globals fuer Paper vollstaendig geschrieben:
- **Global** (17-21): `GLOBAL_PAPER_ACTIVE_SLOT`, `GLOBAL_PAPER_SLOT_COUNT`,
  `GLOBAL_PAPER_ACTIVE_SLOT_NAME`, `GLOBAL_PAPER_ACTIVE_SLOT_CALIBRATED`,
  `GLOBAL_PAPER_ACTIVE_GRADE_MODE`
- **Page** (22-49): Alle PaperModeRuntimeState-Felder, selected-Slot-Summary,
  staged-Profil-Felder (28 Variablen) — vollstaendig.
- **Nicht in vars.h** (benoetigen EEZ-Projektaenderung):
  `PAPER_STEP_WHITE`, `PAPER_STEP_BLACK`, `PAPER_PANEL_TEXT`, `PAPER_HINT_TEXT`,
  Per-Slot-Summaries fuer SELECT-Listenrendering.

#### Zwischenabnahme (gueltig ab Prompt B2)

| Kriterium | Nachweis | Status |
| --- | --- | --- |
| Build | `pio run -e teensy41` → SUCCESS, keine neuen Warnungen | **Bestanden** |
| `ModeId::Paper → SCREEN_ID_PAGE_PAPER_WORKSPACE` | `computeTargetScreen()` in `LvglUi.cpp` Z. 74 | **Intakt** |
| Paper-Widgets werden aktualisiert | Direktpfad: `pushWidgetsFromSnapshot()` via `g_duka_widgets.paper_*` | **Aktiv (Direktpfad)** |
| SELECT/CAL-Panel-Switching | `isCal = (paper.panel == PaperWorkspacePanel::Calibrate)` in `pushWidgets` | **Intakt** |
| Apply/Discard-Items sichtbar | `PaperCalibrationItem::Apply/Discard` korrekt gerendert | **Intakt** |
| Slot-Persistenz und CAL-Semantik unveraendert | Keine Aenderungen an PaperWorkflow, Storage oder CAL-Mathematik | **Keine Seiteneffekte** |
| EEZ Flow-Globals fuer Paper | 33 Variablen (17-49) in `updateSnapshot()` vollstaendig geschrieben | **Infrastruktur bereit** |

#### Naechste Schritte fuer vollstaendige Paper-Migration

1. EEZ Studio: `PAPER_STEP_WHITE` und `PAPER_STEP_BLACK` als neue globale Variablen
   im EEZ-Projekt hinzufuegen (Integer, Defaultwert 0).
2. EEZ Studio: `create_screen_page_paper_workspace()` muss aus einem EEZ-Export
   kommen (Staging-Verzeichnis gemaess B0-Vertrag, Abschnitt 13.4).
3. `tick_screen_page_paper_workspace()` aus dem generierten Code uebernehmen.
4. `LvglUi::updateSnapshot()` erweitern: `PAPER_STEP_WHITE` und `PAPER_STEP_BLACK`
   schreiben (nach Re-Export der vars.h mit neuen Indices).
5. Nach erfolgreichem Build: Direktpfad in `pushWidgetsFromSnapshot()` fuer Paper deaktivieren.

### 13.9 B4-Ergebnis: Setup-Migration BLOCKIERT (Action-Vertrag dokumentiert)

*Erstellt in Prompt B4. `pio run -e teensy41` → SUCCESS 49 s.*

**Entscheidung: BLOCKIERT fuer vollstaendige EEZ-gefuehrte Runtime.**
Gleicher Grundblocker wie B1/B2: `create_screen_page_setup()` ist handgeschrieben.

**Zusaetzlicher Befund:** Die drei Action-Buttons haben `LV_OBJ_FLAG_CLICKABLE`,
aber `actions.h` ist leer — kein LVGL-Event-Callback vorhanden. Touch auf
Apply/Discard/SafetyDefaults ist derzeit vollstaendig inaktiv.

#### Blocker

| # | Blocker | Datei | Details |
| --- | --- | --- | --- |
| 1 | `create_screen_page_setup()` ist handgeschrieben | `screens.c` | Registriert keine EEZ-Widget-Binding-States fuer 12 Handles: `setup_item_row_cont/lbl/val[0-5]`, Edit-Block (4 Labels), 3 Action-Buttons, Hint-Label. |
| 2 | `tick_screen_page_setup()` ist ein Stub | `screens.c` | EEZ-generierter Tick fehlt; `pushWidgetsFromSnapshot()` Direktpfad aktiv. |
| 3 | EEZ-Export blockiert | `screens.c` | Exportpfad gemaess B0-Vertrag (Abschnitt 13.4): erst Staging, dann Merge. |
| 4 | `SETUP_HEAD_TIMING_DIAGNOSTICS_ENABLED` fehlt in vars.h | `vars.h` / EEZ | `setup.headTimingDiagnosticsEnabled` wird in SetupMenuItem::HeadTimingDiagnostics-Renderpfad verwendet; kein EEZ-Var. Benoetigt EEZ-Projektaenderung. |
| 5 | Keine LVGL-Event-Callbacks fuer Action-Buttons | `actions.h`, `screens.c` | `setup_btn_apply`, `setup_btn_discard`, `setup_btn_safety` sind CLICKABLE aber ohne Callback. `actions.h` ist leer. Touch-Route inaktiv. |

#### Widget-Inventar: Direktpfad vs. EEZ-Datenschicht

| Widget-Handle | Direktpfad (pushWidgets) | EEZ-Var vorhanden | Luecke |
| --- | --- | --- | --- |
| `setup_item_row_cont[0-5]` | `lv_obj_set_style_bg_color()` selected/editing | SETUP_SELECTED_ITEM (71), SETUP_EDITING_ACTIVE (73) | Nur Style-Aufrufe, kein String-Var |
| `setup_item_lbl[0-5]` | `lv_label_set_text_static()` aus kSetupItemLabels[] | SETUP_SELECTED_ITEM (71) | Kein Per-Zeile-Var; EEZ-Tick muesste selbst formatieren |
| `setup_item_val[0-5]` | `snprintf()` aus `stagedSettings` + `headTimingDiagnosticsEnabled` | SETUP_SOUND_MODE (79), SETUP_SOUND_VOLUME (80), SETUP_VIBRATION_ENABLED (81), SETUP_MAX_HEAD_BRIGHTNESS_PERCENT (82), SETUP_THERMAL_DERATING_START_CELSIUS (83), SETUP_THERMAL_HARD_STOP_CELSIUS (84) | `SETUP_HEAD_TIMING_DIAGNOSTICS_ENABLED` fehlt |
| `setup_edit_name_lbl` | `lv_label_set_text_static()` aus kSetupItemLabels[] | SETUP_SELECTED_ITEM (71) | Kein `SETUP_EDIT_NAME_TEXT` |
| `setup_edit_val_lbl` | `snprintf()` aus `stagedSettings` | Teilweise via staged-Vars | Kein `SETUP_EDIT_VALUE_TEXT`; `headTimingDiagnosticsEnabled` fehlt |
| `setup_edit_cap_lbl` | `snprintf("CAP %u%%", ...)`, Sichtbarkeit via HIDDEN | SETUP_MAX_HEAD_BRIGHTNESS_PERCENT (82) | Kein `SETUP_CAP_TEXT` |
| `setup_edit_live_lbl` | `snprintf("LIVE %u%%", ...)`, Sichtbarkeit via HIDDEN | SETUP_RUNTIME_OUTPUT_LIMIT (85), SETUP_THERMAL_DERATING_ACTIVE (86) | Kein `SETUP_LIVE_TEXT` |
| `setup_edit_range_lbl` | `lv_label_set_text_static()` je selItem | SETUP_SELECTED_ITEM (71) | Kein `SETUP_RANGE_TEXT` |
| `setup_hint_lbl` | 2 static strings je editingActive | SETUP_EDITING_ACTIVE (73) | Kein `SETUP_HINT_TEXT` |
| `setup_btn_apply` | Sichtbarkeit: editingActive → unhide | SETUP_EDITING_ACTIVE (73) | Kein LVGL-Callback; Touch inaktiv |
| `setup_btn_discard` | Sichtbarkeit: editingActive → unhide | SETUP_EDITING_ACTIVE (73) | Kein LVGL-Callback; Touch inaktiv |
| `setup_btn_safety` | Sichtbarkeit: parametersDirty → unhide | SETUP_PARAMETERS_DIRTY (74) | Kein LVGL-Callback; Touch inaktiv |

#### Action-Vertrag (Setup-Buttons)

| Action | UI-Element | Encoder-Pfad | Guard | Touch-Status |
| --- | --- | --- | --- | --- |
| Apply (Uebernehmen) | `setup_btn_apply` (HIDDEN wenn !editingActive) | E1-PRESS wenn `SetupMenuItem::Apply` selektiert | SetupWorkflow prueft `parametersDirty` | **Inaktiv** — kein LVGL-Callback; offener Touch-Hook |
| Discard (Verwerfen) | `setup_btn_discard` (HIDDEN wenn !editingActive) | E1-PRESS wenn `SetupMenuItem::Discard` selektiert | SetupWorkflow setzt stagedSettings zurueck | **Inaktiv** — kein LVGL-Callback; offener Touch-Hook |
| SafetyDefaults (Werksstandard) | `setup_btn_safety` (HIDDEN wenn !parametersDirty) | E1-PRESS wenn `SetupMenuItem::SafetyDefaults` selektiert | SetupWorkflow laedt `makeDefaultSystemSettings()` | **Inaktiv** — kein LVGL-Callback; offener Touch-Hook |

**Voraussetzung fuer Touch-Aktivierung:** Jede Touch-Action muss einen LVGL-Event-
Handler in `screens.c` bekommen, der einen Ereignispuffer in `InputRouterPolicy`
fuellt. SetupWorkflow wertet diesen Buffer aus — kein direktes Ausfuehren in
EEZ/LVGL-Callback.

#### EEZ-Datenschicht (nahezu vollstaendig)

19 Flow-Globals fuer Setup geschrieben (Indices 71-89). Vollstaendig fuer
State-Navigation. Fehlend fuer vollen Renderpfad:
- `SETUP_HEAD_TIMING_DIAGNOSTICS_ENABLED` — benoetigt EEZ-Projektaenderung
- `SETUP_EDIT_NAME_TEXT`, `SETUP_EDIT_VALUE_TEXT`, `SETUP_CAP_TEXT`,
  `SETUP_LIVE_TEXT`, `SETUP_RANGE_TEXT`, `SETUP_HINT_TEXT` (formatierte Strings)
- Per-Zeile-Item-Vars fuer die 6-Zeilen-Liste

**Hinweis:** `SETUP_HEADER_TEXT` (87) wird mit `presenter_.getSgHeader(snapshot)`
befuellt und `SETUP_VALUE_TEXT` (88) mit `presenter_.getSgExposureMain(snapshot)` —
diese Zuweisungen sind Platzhalter aus SG-Pfad; fuer EEZ-gefuehrten Setup-Tick
wuerden eigene Presenter-Methoden benoetigt.

#### Zwischenabnahme (gueltig ab Prompt B4)

| Kriterium | Nachweis | Status |
| --- | --- | --- |
| Build | `pio run -e teensy41` → SUCCESS, keine neuen Warnungen | **Bestanden** |
| `ModeId::Setup → SCREEN_ID_PAGE_SETUP` | `computeTargetScreen()` in `LvglUi.cpp` | **Intakt** |
| Setup-Widgets werden aktualisiert | Direktpfad: `pushWidgetsFromSnapshot()` via `g_duka_widgets.setup_*` | **Aktiv (Direktpfad)** |
| Lazy-Loading-Guard | `setupStateCache_`, `setupThermalCache_`, `setupOutputCache_` | **Aktiv** |
| CAP/LIVE als Prozent sichtbar | `setup_edit_cap_lbl` / `setup_edit_live_lbl` via Hidden-Flag | **Intakt** |
| Apply/Discard/SafetyDefaults | Encoder-Pfad via SetupWorkflow unveraendert | **Intakt (Encoder only)** |
| Dirty/PersistFailed-State | `SETUP_PARAMETERS_DIRTY` (74), `SETUP_PERSIST_FAILED` (75) vollstaendig geschrieben | **Infrastruktur bereit** |
| Keine Seiteneffekte | Nur Kommentar-Aenderungen in `screens.c` und `LvglUi.cpp` | **Keine Seiteneffekte** |

#### Naechste Schritte fuer vollstaendige Setup-Migration

1. EEZ Studio: `SETUP_HEAD_TIMING_DIAGNOSTICS_ENABLED` als neue globale Variable hinzufuegen.
2. EEZ Studio: `create_screen_page_setup()` aus EEZ-Export generieren (Staging gemaess Abschnitt 13.4).
3. Touch-Hooks implementieren: LVGL-Event-Handler fuer `setup_btn_apply/discard/safety`
   → Ereignis in `InputRouterPolicy`-Buffer schreiben → SetupWorkflow wertet aus.
4. `SETUP_HEADER_TEXT` und `SETUP_VALUE_TEXT` durch eigene Setup-Presenter-Methoden ersetzen.
5. `pushWidgetsFromSnapshot()` Setup-Direktpfad deaktivieren nach erfolgreichem Build.

---

### 13.10 B5-Ergebnis: PageMeasurement — BLOCKIERT

**Entscheidung: BLOCKIERT** — `create_screen_page_measurement()` ist
handgeschrieben (screens.c, Zeile 774–980). `tick_screen_page_measurement()`
ist ein Stub (ruft nur `getFlowState(0,4)` und kehrt sofort zurueck).
EEZ-Runtime kann keine Flow-Variable in Widget-Updates uebersetzen.
Direktpfad in `pushWidgetsFromSnapshot()` bleibt aktiv.

#### Widget-Inventar PageMeasurement

| Handle | Typ | Quelle im Direktpfad | EEZ-Entsprechung |
| --- | --- | --- | --- |
| `meas_source_chip_lbl` | `lv_label` | `presenter_.getPageMeasSourceChip(snapshot)` | Kein `MEAS_SOURCE_CHIP_TEXT` — Presenter-Methode |
| `meas_local_lbl` | `lv_label` | `presenter_.getPageMeasLocalLux(snapshot)` | `MEAS_LOCAL_LUX` (115) — Rohwert; Text per Presenter |
| `meas_wireless_lbl` | `lv_label` | `presenter_.getPageMeasWirelessLux(snapshot)` | `MEAS_WIRELESS_LUX` (116) — Rohwert; Text per Presenter |
| `meas_lux_main_lbl` | `lv_label` | `presenter_.getPageMeasLuxMain(snapshot)` | `MEAS_ACTIVE_LUX` (107) — Rohwert; Text per Presenter |
| `meas_lux_age_lbl` | `lv_label` | `presenter_.getPageMeasLuxAge(snapshot)` | `MEAS_ACTIVE_LUX_AGE_MS` (108), `MEAS_ACTIVE_LUX_SEQUENCE` (109) — Text per Presenter |
| `meas_ref_lux_lbl` | `lv_label` | `presenter_.getPageMeasRefLux(snapshot)` | `MEAS_REFERENCE_LUX` (111) — Rohwert; Text per Presenter |
| `meas_ev_diff_lbl` | `lv_label` | `presenter_.getPageMeasEvDiff(snapshot)` | `MEAS_RELATIVE_EV_STOPS` (114), `MEAS_RELATIVE_EV_VALID` (113) — Text per Presenter |
| `meas_hist_col[11]` | `lv_obj` Container | Kein Text-Update; nur Geometrie via `lv_obj_set_*` | Keine EEZ-Bindung (geometrische Transformation) |
| `meas_hist_fill[11]` | `lv_obj` Fill | `lv_obj_set_y()` + `lv_obj_set_height()` + `lv_obj_set_style_bg_color()` — Farbe: `C_WARN` fuer aktive Zone (`latestZoneIndex`, Guard `sampleCount>0`), `C_ACCENT_PROGRESS` sonst | `MEAS_ZONE_HISTOGRAM` (122) + `MEAS_LATEST_ZONE_INDEX` (119) — Geometrie/Farbe per Direktpfad |
| `meas_session_lbl` | `lv_label` | `presenter_.getPageMeasSession(snapshot)` | `MEAS_SAMPLE_COUNT` (117), `MEAS_CAPTURED_SAMPLE_COUNT` (118), `MEAS_UNDO_DEPTH` (120), `MEAS_CAN_UNDO` (121) — Text per Presenter |

**Gesamt: 7 Label-Handles + 22 Histogramm-Handles = 29 DukaWidgets-Handles.**

#### EEZ-Datenschicht Measurement

| EEZ-Variable | Index | Typ | In `updateSnapshot()` geschrieben |
| --- | --- | --- | --- |
| `MEAS_ACTIVE_SOURCE` | 105 | int | ✅ |
| `MEAS_ACTIVE_LUX_VALID` | 106 | bool | ✅ |
| `MEAS_ACTIVE_LUX` | 107 | float | ✅ |
| `MEAS_ACTIVE_LUX_AGE_MS` | 108 | int | ✅ |
| `MEAS_ACTIVE_LUX_SEQUENCE` | 109 | int | ✅ |
| `MEAS_REFERENCE_VALID` | 110 | bool | ✅ |
| `MEAS_REFERENCE_LUX` | 111 | float | ✅ |
| `MEAS_REFERENCE_SEQUENCE` | 112 | int | ✅ |
| `MEAS_RELATIVE_EV_VALID` | 113 | bool | ✅ |
| `MEAS_RELATIVE_EV_STOPS` | 114 | float | ✅ |
| `MEAS_LOCAL_LUX` | 115 | float | ✅ |
| `MEAS_WIRELESS_LUX` | 116 | float | ✅ |
| `MEAS_SAMPLE_COUNT` | 117 | int | ✅ |
| `MEAS_CAPTURED_SAMPLE_COUNT` | 118 | int | ✅ |
| `MEAS_LATEST_ZONE_INDEX` | 119 | int | ✅ |
| `MEAS_UNDO_DEPTH` | 120 | int | ✅ |
| `MEAS_CAN_UNDO` | 121 | bool | ✅ |
| `MEAS_ZONE_HISTOGRAM` | 122 | string | ✅ |

**Bewertung: EEZ-Datenschicht vollstaendig (18/18).** Kein fehlender Raw-Wert.
Formatierter Text wird intentional nicht in EEZ-Vars abgebildet — stattdessen
`UiPresenter`/`MeasurementValueFormatter` (A5 abgeschlossen; kein `snprintf()`
mehr in LvglUi.cpp fuer den Measurement-Screen).

#### B5-Besonderheit: Messwertformatierungs-Nachweis (A5-Abschluss)

A5 wurde vor B5 vollstaendig abgeschlossen:
- Alle Measurement-Label-Texte kommen aus `UiPresenter::getPageMeas*()`.
- `MeasurementValueFormatter` formatiert Lux, Alter, EV-Abstand, Session-Text.
- `LvglUi.cpp` Measurement-Direktpfad enthaelt **kein** `snprintf()` mehr.
- Wenn ein EEZ-generierter Tick verfuegbar wird, kann er EEZ-Vars fuer
  Rohwert-Pruefungen nutzen und Presenter-Methoden fuer Text-Ausgabe aufrufen.

#### Blocker-Tabelle

| Blocker | Ursache | Aufloesung |
| --- | --- | --- |
| `create_screen_page_measurement()` handgeschrieben | Keine EEZ-Bindungszustaende registriert | EEZ-Export → Staging → Merge (Abschnitt 13.4) |
| `tick_screen_page_measurement()` ist Stub | Kein EEZ-generierter Code vorhanden | EEZ-Export erforderlich |
| Histogramm-Geometrie/Farbe (22 Handles) | Geometrie und `latestZoneIndex`-Farbe direkt implementiert; EEZ-Text-Bindung bleibt nicht anwendbar | EEZ-Custom-Action oder Presenter-Aufruf im generierten Tick fuer spaetere EEZ-Migration |
| Kein Touch-Callback | `actions.h` leer | LVGL-Event-Handler implementieren nach EEZ-Export |

#### Zwischenabnahme B5

| Kriterium | Status |
| --- | --- |
| `pio run -e teensy41` ohne neue Fehler | ✅ Build SUCCESS 27 s |
| `tick_screen_page_measurement()` mit Blocker-Kommentar annotiert | ✅ |
| LvglUi.cpp PageMeasurement-Sektion mit B5-Kommentar annotiert | ✅ |
| EEZ-Datenschicht vollstaendig dokumentiert (18/18) | ✅ |
| A5-Abschluss (Messwertformatierung im Presenter) nachgewiesen | ✅ |
| `latestZoneIndex`-Markierung im Histogramm direkt implementiert | ✅ |
| 11 Zonen erhalten, Farb-Guard `sampleCount>0` | ✅ |

#### Naechste Schritte fuer vollstaendige Measurement-Migration

1. EEZ Studio: `create_screen_page_measurement()` aus EEZ-Export generieren
   (Staging gemaess Abschnitt 13.4).
2. Histogramm-Spalten in EEZ als Custom-Widget oder per EEZ-Action-Callback
   implementieren — geometrische Manipulation erfordert Presenter-Aufruf.
3. Touch-Hooks: LVGL-Event-Handler fuer Undo/Capture → `InputRouterPolicy`-Buffer.
4. `pushWidgetsFromSnapshot()` Measurement-Direktpfad deaktivieren nach
   erfolgreichem EEZ-Build.

---

### 13.11 B6-Ergebnis: PageWirelessRemote — GEPARKT

**Entscheidung: GEPARKT** — die Seite ist designseitig vollstaendig vorhanden,
aber runtime-seitig permanent unerreichbar. Es existiert kein sicherer
Servicepfad ohne strukturellen Eingriff, der den Prompt-Rahmen sprengen wuerde.

#### Blocker-Inventar PageWirelessRemote

| Blocker | Detail |
| --- | --- |
| `computeTargetScreen()` ohne WirelessRemote-Branch | `SCREEN_ID_PAGE_WIRELESS_REMOTE = 6` hat keine Routing-Bedingung — Seite bleibt permanent unerreichbar |
| Kein `ModeId::WirelessRemote` | Enum hat 5 Werte: `None`, `Splitgrade`, `BlackWhite`, `Setup`, `Paper`. Neuer Eintrag laut Prompt explizit verboten. |
| Kein Snapshot-Flag | `SystemSnapshot` und `ModeRuntimeState` enthalten kein `showWirelessDiag`/`diagnosticPageRequested`-Flag |
| Kein Long-Press/Gesture/Service-Menu | `InputRouterPolicy` kennt nur `Encoder`/`Touch`/`Modal`-Fokus; kein Debounce/Guard-Konzept fuer Navigations-Gesten |
| `ModeCoordinator.requestMode()` | Akzeptiert nur registrierte `IModeWorkflow`-Instanzen; kein WirelessRemote-Workflow registriert |

#### Widget-Inventar PageWirelessRemote

| Handle | Typ | Quelle im Direktpfad | EEZ-Entsprechung |
| --- | --- | --- | --- |
| `remote_link_chip_lbl` | Label | `EspLinkRuntimeStatus.linkHealth` | REMOTE_LINK_HEALTH (123) |
| `remote_rx_tx_lbl` | Label | `lastRxAgeMs` / `lastTxAgeMs` | REMOTE_LAST_RX_AGE_MS (125), REMOTE_LAST_TX_AGE_MS (126) |
| `remote_uptime_lbl` | Label | `remoteUptimeMs` | REMOTE_REMOTE_UPTIME_MS (127) |
| `remote_peer_chip_lbl` | Label | `WirelessGatewayStatus.peerState` | REMOTE_PEER_STATE (128) |
| `remote_battery_lbl` | Label | `batteryPercent` | REMOTE_BATTERY_PERCENT (129) |
| `remote_peer_lux_lbl` | Label | `lastLux` / `measurementSequence` | REMOTE_LAST_LUX (131), REMOTE_MEASUREMENT_SEQUENCE (132) |
| `remote_diag_lbl` | Label | `EspDiagnosticStatus` code+detail | REMOTE_DIAGNOSTIC_CODE (137), REMOTE_DIAGNOSTIC_DETAIL (138) |
| `remote_txqueue_lbl` | Label | `EspTxQueueStatus` pending+dropped | REMOTE_TX_PENDING_FRAME_COUNT (140), REMOTE_TX_DROPPED_RENDER_COUNT (141) |
| `remote_rmt_lbl` | Label | RMT in-flight/retry/timeout | REMOTE_RMT_IN_FLIGHT_COUNT (143), REMOTE_RMT_RETRY_COUNT (144), REMOTE_RMT_TIMEOUT_COUNT (145) |

9 read-only Label-Handles; kein Touch-Callback; kein Presenter-Aequivalent
(A5-Migration ausstaendig: `snprintf()` direkt in LvglUi.cpp).

#### EEZ-Datenschicht

24 Flow-Globals: `REMOTE_LINK_HEALTH` (123) bis `REMOTE_RMT_SATURATION_COUNT`
(146); alle in `updateSnapshot()` / `pushWidgetsFromSnapshot()` geschrieben.
Variablenlaenge vollstaendig — kein fehlender EEZ-Var-Eintrag fuer Remote-Daten.

#### Definierter Runtime-Hook fuer spaetere Freischaltung

Anforderungen fuer die spaetere Implementierung (kein neuer `ModeId` noetig):

1. **Snapshot-Flag**: `bool showWirelessDiag = false` in `SystemSnapshot` als
   SETUP-Submodus-Signal — kein eigener `ModeTab`, SETUP-Tab bleibt aktiv
   (Workflow-Familie = 3, `computeWorkflowFamily()` unveraendert).
2. **computeTargetScreen()-Branch**: Nach dem `Setup`-Routing prueft die Funktion
   `snapshot.showWirelessDiag` und gibt `SCREEN_ID_PAGE_WIRELESS_REMOTE` zurueck.
3. **Aktivierungspfad**: Encoder3-LongHold (oder dedizierter Setup-Menu-Eintrag)
   setzt das Flag — erfordert Debounce/Guard-Erweiterung in `InputRouterPolicy`
   (`kLongHoldThresholdMs`-Klasse analog zu `kTouchFocusHoldMs`).
4. **Rueckkehrvertrag**: Encoder3-Back/Cancel oder Timeout (z.B. 30 s Inaktivitaet)
   setzt `showWirelessDiag = false`; naechster `computeTargetScreen()`-Aufruf
   routet automatisch zurueck zu Setup.
5. **Presenter-Migration**: `snprintf()`-Pfad in `pushWidgetsFromSnapshot()` fuer
   die 9 Remote-Handles durch `UiPresenter`-Methoden ersetzen (analog A5).

#### Zwischenabnahme B6

| Kriterium | Status |
| --- | --- |
| `tick_screen_page_wireless_remote()` mit B6-Blocker-Kommentar annotiert | ✅ |
| LvglUi.cpp PageWirelessRemote-Sektion mit B6-GEPARKT-Kommentar annotiert | ✅ |
| Phase-B-Schritte in Abschnitt 13 mit B6-Annotation ergaenzt | ✅ |
| EEZ-Datenschicht dokumentiert (24 Flow-Globals 123-146) | ✅ |
| Runtime-Hook fuer spaetere Freischaltung vollstaendig beschrieben | ✅ |
| `pio run -e teensy41` nicht erforderlich (nur Doku/Kommentare geaendert) | ✅ |

#### Naechste Schritte fuer vollstaendige WirelessRemote-Aktivierung

1. `bool showWirelessDiag` in `SystemSnapshot` hinzufuegen (Abschnitt 13.11
   Runtime-Hook Punkt 1).
2. `computeTargetScreen()` Branch fuer `SCREEN_ID_PAGE_WIRELESS_REMOTE`
   implementieren.
3. Long-Hold-Infrastruktur in `InputRouterPolicy` ergaenzen.
4. Presenter-Migration fuer die 9 Remote-Label-Handles (analog A5).
5. `pushWidgetsFromSnapshot()` Remote-Direktpfad nach EEZ-Aktivierung
   deaktivieren.

---

### 13.12 B7-Ergebnis: BusyScreen — BLOCKIERT

**Entscheidung: BLOCKIERT** — 2 von 7 Safety-Gates nicht erfuellt.
Keine EEZ-Migration ausgefuehrt. Direktpfad bleibt aktiv und safety-korrekt.

#### Safety-Gate-Tabelle BusyScreen

| Gate | Bedingung | Status | Befund |
| --- | --- | --- | --- |
| G0 | Prerequisite: min. 1 nicht-sicherheitskritische Seite erfolgreich migriert | ❌ | B1-B5: alle BLOCKIERT; B6: GEPARKT. Keine Pilotseite bisher migriert. |
| G1 | `create_screen_busy()` vollstaendig vorhanden und handgeschrieben | ✅ | 10 Handles; Klassen Boot/SG/PaperWS/Setup/Measurement nicht angetastet. |
| G2 | Busy-Header-Handles getrennt von normalen `hdr_*`-Handles | ✅ | Expliziter Kommentar im Code: „Kein build_header()-Aufruf“. Eigene `busy_hdr_*` Handles; kein Shared-Header-Override. |
| G3 | Pause/Resume/Abort puffern nur Ereignisse — Safety bleibt in Firmware | ✅ | `busy_btn_cb` → `lvgl_ui_modal_action_callback` → `setPendingModalAction` → `pollModalAction()` in main.cpp. `UiModalAction` Enum (Resume=1, Abort=2, Pause=3). ExposureEngine nicht direkt beruehrt. |
| G4 | Keine Zusammenlegung von Busy-Header und normalen Header-Handles | ✅ | Klar getrennt, unveraenderbar solange `create_screen_busy()` handgeschrieben bleibt. |
| G5 | `computeTargetScreen()` gibt Busy absolute Prioritaet | ✅ | Erste Branch in `computeTargetScreen()`: PreWait / Exposing / Paused / PostWait → `SCREEN_ID_BUSY`. Kein anderer Screen kann waehrend aktiver Exposition eingeblendet werden. |
| G6 | `tick_screen_busy()` kann EEZ-Flow-Variablen propagieren | ❌ | Stub-Implementierung (nur `getFlowState(0, 6)`). Identischer Blocker wie B1-B5: `create_screen_busy()` handgeschrieben → EEZ-Runtime hat keine Binding-Tabelle. |

#### Widget-Inventar BusyScreen

| Handle | Typ | Quelle im Direktpfad | EEZ-Entsprechung |
| --- | --- | --- | --- |
| `busy_hdr_phase_lbl` | Label | `ExposurePhase` → `"BELICHTUNG"` / `"VORBEREITUNG"` / `"PAUSE"` / `"NACHWARTEZEIT"` | EXPOSURE_PHASE (90) |
| `busy_hdr_msg_cont` | Obj (Container) | Hintergrundfarbe aus `overlayColorHex` | GLOBAL_OVERLAY_COLOR (3) |
| `busy_hdr_msg_lbl` | Label | `presenter_.getOverlayText(snapshot)` | GLOBAL_OVERLAY_TEXT (2) |
| `busy_hdr_thermal_lbl` | Label | `exp.thermalDeratingActive` (hidden/visible) | EXPOSURE_THERMAL_DERATING_ACTIVE (99) |
| `busy_time_lbl` | Label | `snprintf`: Restzeit in s oder Dosis in lx·s | EXPOSURE_REMAINING_TIME_SECONDS (95), EXPOSURE_CURRENT_DOSE (93), EXPOSURE_TARGET_DOSE (92) |
| `busy_bar` | Bar | `busyProgressPercent_` (aus phaseAgeMs + remainMs) | EXPOSURE_PHASE_AGE_MS (104), EXPOSURE_REMAINING_TIME_SECONDS (95) |
| `busy_pct_lbl` | Label | `snprintf("%d%%", pct)` | (aus busy_bar-Berechnung) |
| `busy_sg_detail_lbl` | Label | `snprintf`: SG/BW Soft/Hard-Zeiten | SG_EXECUTION_STATE (54), SG_REMAINING (61), SG_HARD_TARGET (57) |
| `busy_run_overlay` | Obj | `ExposurePhase::Exposing` → visible | EXPOSURE_PHASE (90) |
| `busy_pause_overlay` | Obj | `ExposurePhase::Paused` → visible | EXPOSURE_PHASE (90) |

10 Handles; 3 mit `snprintf`-Direktpfad (kein Presenter-Aequivalent, kein A5-Analogon).

#### EEZ-Datenschicht

Exposure-Variablen: `EXPOSURE_PHASE` (90) bis `EXPOSURE_PHASE_AGE_MS` (104) —
15 Flow-Globals, alle in `updateSnapshot()` geschrieben. Kein dediziertes
`BUSY_*`-Variablen-Subset. SG-Variablen (50–70) werden fuer SG-Detail verwendet.
`GLOBAL_OVERLAY_TEXT` (2) und `GLOBAL_OVERLAY_COLOR` (3) sind seitenuebergreifend.

#### Passierter Vertrag (fuer spaetere Migration)

Die folgenden Struktureigenschaften sind bereits korrekt und muessen bei
der EEZ-Migration erhalten bleiben:

- `create_screen_busy()` darf nie `build_header()` aufrufen. Der gemeinsame
  Header (`hdr_*`) und der Busy-Header (`busy_hdr_*`) sind strikt getrennt.
- Die drei Buttons (Pause=3, Resume=1, Abort=2) duerfen nur `setPendingModalAction`
  befuellen, nie direkt ExposureEngine-Methoden aufrufen.
- `computeTargetScreen()` Busy-Prioritaet (Pre/Exposing/Paused/PostWait)
  muss auch nach EEZ-Migration unveraendert bleiben.
- Overlay-Farbe und Overlay-Text kommen aus `presenter_.getOverlayText()` /
  `overlayColorHex` — identisch mit allen anderen Screens.

#### Definiierter Migration-Hook fuer spaetere Freischaltung

Voraussetzungen (nach Pilot-Seite):

1. Mindestens eine Seite (z.B. Boot oder PaperWorkspace) erfolgreich per EEZ-
   Runtime aktiv — beweist, dass `tick_screen_*()` Flow-Variablen propagiert.
2. `create_screen_busy()` aus EEZ-Export generieren (Staging gemaess Abschnitt
   13.4, Merge-Vertrag: `busy_hdr_*` Handles erhalten, `busy_btn_cb` in
   `screens.c` Sektion beibehalten).
3. `tick_screen_busy()` erhaelt EEZ-generierte Bindings statt des Stubs.
4. Presenter-Migration: `snprintf`-Pfad fuer `busy_time_lbl`, `busy_pct_lbl`,
   `busy_sg_detail_lbl` durch `UiPresenter`-Methoden ersetzen (analog A5).
5. Hardware-Abnahme: PreWait, Exposing (Zeitfortschritt), Paused (Overlay),
   PostWait, Done/Fault, Resume-Callback, Abort-Callback. Alle 6 Phasen
   ohne Regression bestaetigt.

#### Zwischenabnahme B7

| Kriterium | Status |
| --- | --- |
| Safety-Gate-Tabelle vollstaendig (7 Gates) | ✅ |
| `tick_screen_busy()` mit B7-Blocker-Kommentar annotiert | ✅ |
| LvglUi.cpp Busy-Sektion mit B7-BLOCKIERT-Kommentar annotiert | ✅ |
| Phase-B-Schritte mit B7-Annotation ergaenzt | ✅ |
| EEZ-Datenschicht dokumentiert (EXPOSURE 90-104, 15 Vars) | ✅ |
| Passierter Vertragsvertrag dokumentiert (Header-Isolation, Callback-Vertrag) | ✅ |
| Migration-Hook vollstaendig beschrieben (5 Schritte) | ✅ |
| `pio run -e teensy41` Regression-Check | ✅ BUILD SUCCESS 41.7 s (B6+B7 Kommentare, kein Code geaendert) |
| Manuelle Hardware-Abnahme | ⏳ ausstehend (kein Code geaendert) |

#### Hardware-Abnahmeliste (nach kuenftiger Migration)

| Phase | Erwartetes Verhalten |
| --- | --- |
| PreWait | `busy_hdr_phase_lbl` = "VORBEREITUNG", `busy_time_lbl` = "...", Balken = 0%, Overlays hidden |
| Exposing | `busy_hdr_phase_lbl` = "BELICHTUNG", Zeitfortschritt sichtbar, Balken steigt, Run-Overlay (Pause-Btn) sichtbar |
| Paused | `busy_hdr_phase_lbl` = "PAUSE", Pause-Overlay (Resume+Abort) sichtbar, Run-Overlay hidden |
| PostWait | `busy_hdr_phase_lbl` = "NACHWARTEZEIT", `busy_time_lbl` = "...", Balken = 100%, Overlays hidden |
| Done/Fault | `SCREEN_ID_BUSY` verlassen (computeTargetScreen zeigt naechsten Screen), kein Hang |
| Resume-Callback | `pollModalAction()` liefert `UiModalAction::Resume`, ExposureEngine reagiert korrekt |
| Abort-Callback | `pollModalAction()` liefert `UiModalAction::Abort`, Belichtung sicher abgebrochen |

---

### 13.13 C1-Ergebnis: Boot-Page — EEZ-Layout verfeinert

**Seitenauswahl:** Boot-Page (autonome Wahl: einfachste Seite, B1 abgeschlossen,
alle A-Schritte abgeschlossen. B3/PageSplitgrade noch offen — C1 erfordert nur
A-Schritte abgeschlossen.)

**Aenderungen im EEZ-Projekt** (`DukatimerPart2TeensyUi.eez-project`):

| Nr | Widget / Eigenschaft | Alt | Neu | Begruendung |
| --- | --- | --- | --- | --- |
| 1 | Screen `widgetFlags` | `...\|SCROLLABLE\|SCROLL_ELASTIC\|SCROLL_MOMENTUM\|SCROLL_CHAIN_HOR\|SCROLL_CHAIN_VER` | `CLICKABLE\|PRESS_LOCK\|CLICK_FOCUSABLE\|GESTURE_BUBBLE\|SNAPPABLE` | `create_screen_boot()` ruft `lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE)` — EEZ muss das abbilden |
| 2 | `boot_titleText` `style.useStyle` | `"default"` (= StyleLabelText) | `"StyleLabelMain28"` | StyleLabelMain28 deckt MONTSERRAT_28 + C_TEXT + Center vollstaendig ab |
| 3 | `boot_titleText` `localStyles.definition` | `{text_color, text_font, text_align}` | entfernt (`{}`) | Vollstaendig durch StyleLabelMain28 abgedeckt — redundante Inlining behoben |
| 4 | `boot_subtitleText` `style.useStyle` | `"default"` (= StyleLabelText) | `"StyleLabelTextSec"` | StyleLabelTextSec deckt C_TEXT_SEC + MONTSERRAT_14 ab (Farbe nicht mehr lokal dupliziert) |
| 5 | `boot_subtitleText` `localStyles.definition` | `{text_color, text_font, text_align}` | `{text_font: MONTSERRAT_16, text_align: CENTER}` | text_color entfernt (durch StyleLabelTextSec abgedeckt); font+center als notwendige Overrides erhalten |
| 6 | `global_overlayText` `style.useStyle` | `"default"` (= StyleLabelText) | `"StyleLabelTextSec"` | Gleiche Begruendung wie Zeile 4 |
| 7 | `global_overlayText` `localStyles.definition` | `{text_color, text_font, text_align}` | `{text_align: CENTER}` | text_color + text_font vollstaendig durch StyleLabelTextSec abgedeckt; nur Center-Alignment als Override noetig |

**Kein Runtime-Code geaendert.** `screens.c` und `LvglUi.cpp` unveraendert.

**Befund — Layoutuebereinstimmung EEZ ↔ screens.c:**

| Widget | EEZ Position | screens.c Position | Status |
| --- | --- | --- | --- |
| `boot_titleText` (→ `boot_title_lbl`) | (20, 60) 440×36 | (20, 60) 440×36 | ✅ deckungsgleich |
| `boot_subtitleText` (→ `boot_subtitle_lbl`) | (20, 110) 440×24 | (20, 110) 440×24 | ✅ deckungsgleich |
| `global_overlayText` (→ `boot_status_lbl`) | (20, 200) 440×48 | (20, 200) 440×48 | ✅ deckungsgleich |

**Designzone Boot:** Kein Header-Streifen, kein ModeTabs-Streifen (Fullscreen-Splash).
Titel bei y=60 (oberes Drittel), Subtitle bei y=110 (unter Titel), Status bei y=200
(unteres Drittel). Weissraum (y=0-59, y=134-199, y=248-319) ist intentional.

**Hinweis LONG_MODE:** `boot_subtitleText` hat `LV_LABEL_LONG_CLIP` in EEZ
(intentional: 24px-Einzeilenhoehe). `create_screen_boot()` setzt kein
explizites long_mode → LVGL-Default WRAP. Diese Inkonsistenz wird bei
kuenftiger EEZ-Code-Generierung automatisch aufgeloest (EEZ-Projekt ist
Quelle der Wahrheit fuer CLIP).

**Hinweis fehlende Boot-Vars in EEZ:** Boot-Vars 149-154 (`BOOT_ACTIVE_MODE`,
`BOOT_LINK_HEALTH`, `BOOT_PAPER_SLOT_STORAGE_ERROR_CODE`,
`BOOT_PAPER_SLOT_STORAGE_ERROR_DETAIL`, `BOOT_REMOTE_CAPABILITY_BITS`,
`BOOT_TOUCH_ACTIVE`) haben keine EEZ-Widgets auf der Boot-Seite. Identisch
mit `create_screen_boot()` (nur 3 Labels). Design-Intent: Boot-Seite minimal.
Keine Aenderung erforderlich.

**JSON-Validierung:** `json.load()` ohne Fehler. Alle 7 Aenderungen verifiziert.

**Zwischenabnahme C1 Boot:**

| Kriterium | Status |
| --- | --- |
| Alle A-Schritte abgeschlossen | ✅ (A1-A6 done) |
| B1-Schritt dokumentiert (BLOCKIERT) | ✅ Abschnitt 13.7 |
| EEZ-Layout ↔ screens.c deckungsgleich | ✅ alle 3 Labels identische Position/Groesse |
| Stilsystem korrekt genutzt (named styles) | ✅ 3 named styles verwendet, Redundanz entfernt |
| SCROLLABLE-Flag korrigiert | ✅ entfernt, entspricht screens.c |
| Runtime-Code unveraendert | ✅ kein screens.c / LvglUi.cpp Aenderung |
| JSON valide nach Aenderung | ✅ |
| Keine neuen Farben ausserhalb Palette | ✅ keine neuen Farben |
| Kein Build erforderlich (EEZ-only) | ✅ |

---

### 13.14 C2-Ergebnis: Seitenuebergaenge — DOKUMENTIERT, keine implementiert

**Entscheidung:** Keine EEZ-Transitions implementiert. Alle Uebergaenge bleiben
ausschliesslich Firmware-gesteuert via `computeTargetScreen()` +
`eez_flow_set_screen(..., LV_SCR_LOAD_ANIM_NONE, 0, 0)`.

#### Uebergangstabelle (Soll-Zustand)

| Von | Nach | Ausloeser | Firmware-Guard | EEZ-Transition |
| --- | --- | --- | --- | --- |
| `SCREEN_ID_BOOT` | `SCREEN_ID_PAGE_PAPER_WORKSPACE` | `requestMode(Paper)` via Encoder | — | nicht implementiert |
| `SCREEN_ID_BOOT` | `SCREEN_ID_PAGE_SPLITGRADE` | `requestMode(Splitgrade/BW)` via Encoder | — | nicht implementiert |
| `SCREEN_ID_BOOT` | `SCREEN_ID_PAGE_SETUP` | `requestMode(Setup)` via Encoder | — | nicht implementiert |
| `SCREEN_ID_PAGE_PAPER_WORKSPACE` | `SCREEN_ID_PAGE_SETUP` | `requestMode(Setup)` via Encoder | — | nicht implementiert |
| `SCREEN_ID_PAGE_PAPER_WORKSPACE` | `SCREEN_ID_PAGE_SPLITGRADE` | `requestMode(Splitgrade)` via Encoder | — | nicht implementiert |
| `SCREEN_ID_PAGE_SPLITGRADE` | `SCREEN_ID_PAGE_MEASUREMENT` | `SplitgradePanel::Measurement` via Encoder | nur bei `ExposurePhase::Idle` | nicht implementiert |
| `SCREEN_ID_PAGE_MEASUREMENT` | `SCREEN_ID_PAGE_SPLITGRADE` | `SplitgradePanel != Measurement` via Encoder | — | nicht implementiert |
| `SCREEN_ID_PAGE_SETUP` | `SCREEN_ID_PAGE_*` | `requestMode(other)` via Apply/Discard/Encoder | — | nicht implementiert |
| **Jeder Screen** | **`SCREEN_ID_BUSY`** | **`ExposurePhase` PreWait / Exposing / Paused / PostWait** | **Hoechste Prioritaet (erste Branch)** | **nicht implementiert** |
| `SCREEN_ID_BUSY` | Modus-abhaengig | `ExposurePhase` Idle / Done / Fault | — | nicht implementiert |
| (geparkt) Jeder Screen | `SCREEN_ID_PAGE_WIRELESS_REMOTE` | `showWirelessDiag`-Flag (Runtime-Hook fehlt — B6 GEPARKT) | — | — |

#### Implementiert vs. nur dokumentiert

| Uebergang | Implementiert | Begruendung |
| --- | --- | --- |
| Alle 10 Firmwere-Uebergaenge | Firmware-seitig ✅ | `computeTargetScreen()` + `eez_flow_set_screen(... NONE ...)` |
| Visuelle EEZ-Animationen | ❌ Kein Impl. | 3 Blocker (s. unten) |
| `PageWirelessRemote`-Routing | ❌ Geparkt | Runtime-Hook fehlt (B6) |

**Blocker fuer visuelle EEZ-Transitions:**

1. **Universal EEZ Blocker:** `tick_screen_*()` sind Stubs → EEZ-Flow-Aktionen
   feuern nie → `eez_flow_push_screen()` kann nie aus EEZ-Seite aufgerufen werden.
2. **Firmware-NONE-Override:** `eez_flow_set_screen(targetScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0)`
   wird bei jeder Screen-Aenderung aufgerufen und bricht jede laufende Animation sofort ab.
   Die Polling-Periode betraegt ca. 16 ms (LVGL-Frame) — kuerzer als jede sinnvolle Animationsdauer.
3. **Busy-Safety:** `SCREEN_ID_BUSY` muss sofort erscheinen, wenn `ExposurePhase`
   zu PreWait/Exposing wechselt. Jede Animationsdauer > 0 ms waere ein Safety-Verstoß.

#### Aktueller EEZ-Projekt-Stand Transitions

- `.eez-project`: **Keine** `transition`-, `animation`-, `fadeMode`-,
  `loadAnim`- oder `speed`-Felder konfiguriert.
- `eez-flow.cpp`: `eez_flow_set_screen()`, `eez_flow_push_screen()`,
  `eez_flow_pop_screen()` sind vorhanden aber nur von Firmware-Seite genutzt.
- `ui.c`/`screens.c`: Kein `lv_scr_load_anim()`-Aufruf.

#### Soll-Entwurf fuer kuenftige EEZ-Animationen (Runtime-Hook)

Eine spaetere EEZ-Animation-Aktivierung erfordert alle vier Bedingungen:

1. Universal EEZ Blocker aufgeloest (alle `create_screen_*()` aus EEZ-Export,
   `tick_screen_*()` propagieren Variablen).
2. Firmware ruft fuer normale Uebergaenge `eez_flow_set_screen(target,
   LV_SCR_LOAD_ANIM_FADE_IN, 150, 0)` statt NONE — mit Ausnahme von:
3. Busy bleibt zwingend `LV_SCR_LOAD_ANIM_NONE, 0, 0` (keine Animationsverzoegerung
   bei Exposure-Start).
4. `eez_flow_set_screen` statt `eez_flow_push_screen` — kein Screen-Stack,
   Firmware bleibt einziger Navigationsowner.

#### Zwischenabnahme C2

| Kriterium | Status |
| --- | --- |
| Uebergangstabelle vollstaendig dokumentiert | ✅ |
| Busy-Prioritaet nachgewiesen | ✅ (erste Branch `computeTargetScreen()`) |
| Kein globaler Screen-Stack eingefuehrt | ✅ (`eez_flow_set_screen` setzt Stack auf 0) |
| Keine Touch-Route ohne Encoder-Alternative | ✅ (keine Touch-Transition implementiert) |
| Keine neue Fachseite ohne Runtime-Datenvertrag | ✅ (WirelessRemote geparkt) |
| `.eez-project` gueltig / unveraendert | ✅ (kein EEZ-Edit, JSON valide) |
| Kein Build erforderlich (nur Doku) | ✅ |

---

### 13.15 C3-Ergebnis: Touch-Felder finalisiert — PRESSED-VISUAL implementiert

#### Touch-Feld-Tabelle (vollstaendig)

| Touch-Feld | Screen | Groesse px | Min 48×48? | Pressed-Visual | Action-Name | Encoder-Alternative | Firmware-Guard | Status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Tab PAPER | Alle Hauptseiten y=284 | 120×36 | ❌ Höhe 36<48 | ❌ kein CLICKABLE | `TouchTabFamily(0)` | Enc1-Press (Platzhalter) | `shouldDispatch()` Open | Offener Hook / inaktiv |
| Tab MEAS | Alle Hauptseiten y=284 | 120×36 | ❌ Höhe 36<48 | ❌ kein CLICKABLE | `TouchTabFamily(1)` | (kein Shortcut) | `shouldDispatch()` Open | Offener Hook + ModeId-Lücke |
| Tab PRINT | Alle Hauptseiten y=284 | 120×36 | ❌ Höhe 36<48 | ❌ kein CLICKABLE | `TouchTabFamily(2)` | Enc2-Press (Platzhalter) | `shouldDispatch()` Open | Offener Hook / inaktiv |
| Tab SETUP | Alle Hauptseiten y=284 | 120×36 | ❌ Höhe 36<48 | ❌ kein CLICKABLE | `TouchTabFamily(3)` | Enc3 LongPress | `shouldDispatch()` Open | Offener Hook / inaktiv |
| UEBERNEHMEN | PageSetup (Edit-Block) | 140×56 | ✅ | ✅ `C_BTN_CONFIRM_BD` | `TouchSetupApply` | Enc3-Press Apply | `shouldDispatch()` Open | CLICKABLE, kein Callback (inaktiver Hook) |
| VERWERFEN | PageSetup (Edit-Block) | 140×56 | ✅ | ✅ `C_BTN_DISCARD_BD` | `TouchSetupDiscard` | Enc3-Press Discard | `shouldDispatch()` Open | CLICKABLE, kein Callback (inaktiver Hook) |
| SICHERHEITSSTANDARDS | PageSetup (Edit-Block) | 186×56 | ✅ | ✅ `C_BTN_SAFETY_BD` | `TouchSetupSafety` | Enc3-Press SafetyDefaults | `shouldDispatch()` Open | CLICKABLE, kein Callback (inaktiver Hook) |
| PAUSE | Busy run_overlay | 460×52 | ✅ | ✅ `C_BTN_SAFETY_BD` | `UiModalAction::Pause (3)` | Start-Taste | WorkflowWait Pause erlaubt | **Verdrahtet + pressed** |
| FORTSETZEN | Busy pause_overlay | 215×52 | ✅ | ✅ `C_BTN_CONFIRM_BD` | `UiModalAction::Resume (1)` | Start-Taste | WorkflowWait Resume erlaubt | **Verdrahtet + pressed** |
| ABBRECHEN | Busy pause_overlay | 215×52 | ✅ | ✅ `C_BTN_DISCARD_BD` | `UiModalAction::Abort (2)` | (kein Shortcut) | WorkflowWait Undo erlaubt | **Verdrahtet + pressed** |

#### Mindestflaeche-Nachweis

| Kategorie | Befund |
| --- | --- |
| ModeTabs (4×) | **UNTERSCHREITUNG**: 120×36 px, Höhe 36 < 48 px. Kein `LV_OBJ_FLAG_CLICKABLE` gesetzt → rein visuell → keine aktive Touch-Zone. Verletzung im naechsten Layoutzyklus zu korrigieren: (a) Tab-Bar y=272, h=48 oder (b) unsichtbare Touch-Erweiterungszone nach oben. |
| Setup-Buttons (3×) | ✅ Alle ≥ 140×56 px. Mindestflaeche erfuellt. |
| Busy-Buttons (3×) | ✅ pause 460×52, resume/abort je 215×52. Mindestflaeche erfuellt. |

#### Implementierte Pressed-Zustaende (C3 — `screens.c`)

Alle implementierten Pressed-States verwenden ausschliesslich bestehende Palettenfarben
(`*_BD`-Variante = Randfarbe des jeweiligen Button-Typs):

| Button | Normal-BG | Pressed-BG (neu) | Palettenfarbe |
| --- | --- | --- | --- |
| PAUSE | `C_BTN_SAFETY_BG` | `C_BTN_SAFETY_BD` | ✅ Palette |
| FORTSETZEN | `C_BTN_CONFIRM_BG` | `C_BTN_CONFIRM_BD` | ✅ Palette |
| ABBRECHEN | `C_BTN_DISCARD_BG` | `C_BTN_DISCARD_BD` | ✅ Palette |
| UEBERNEHMEN | `C_BTN_CONFIRM_BG` | `C_BTN_CONFIRM_BD` | ✅ Palette |
| VERWERFEN | `C_BTN_DISCARD_BG` | `C_BTN_DISCARD_BD` | ✅ Palette |
| SICHERHEITSSTANDARDS | `C_BTN_SAFETY_BG` | `C_BTN_SAFETY_BD` | ✅ Palette |

LVGL-Selector: `LV_PART_MAIN | LV_STATE_PRESSED`. Keine neuen Farben ausserhalb der Palette.

#### Blocker: EEZ-seitige Pressed-Zustaende

**Universal EEZ Blocker** verhindert Pressed-Zustaende im `.eez-project`:

- `tick_screen_*()` sind Stubs → EEZ-Flow-Variablen propagieren nicht →
  EEZ kann keine Style-Conditions auf Pressed-State anwenden.
- `.eez-project` hat `pressed: 0` Vorkommen in EventHandlern.
- Pressed-States bleiben ausschliesslich in `screens.c` via `LV_STATE_PRESSED`.

#### Blocker: Header-Feedback (Aufgabe C3.4)

Touch-Bestaetigung im Header-Meldungsband ist nicht implementierbar:

1. **Tick-Ueberschreiben:** `LvglUi::pushWidgetsFromSnapshot()` setzt `hdr_msg_lbl`
   in jedem UI-Tick (~16 ms) neu → kurzes Feedback wuerde sofort ueberschrieben.
2. **Busy-Screen hat keinen normalen Header:** `busy_hdr_msg_lbl` ist ein eigenes
   Handle; `hdr_msg_lbl` existiert auf dem Busy-Screen nicht.
3. **Setup-Buttons ohne Callback:** Kein Callback-Pfad fuer Feedback-Aufruf.

Zukunftspfad: separater `hdr_feedback_timer_`-Mechanismus in `LvglUi`, der nach
konfigurierbarer Zeit zurueckfaellt; erst dann ist Header-Feedback fuer inaktive
Hooks sinnvoll.

#### Deaktivierte / inaktive Touch-Felder

| Feld | Warum deaktiviert | Markierung |
| --- | --- | --- |
| ModeTabs PAPER/MEAS/PRINT/SETUP | Kein `LV_OBJ_FLAG_CLICKABLE` → rein visuell | Abschnitt 12.1 + 12.2 offener Hook |
| Setup UEBERNEHMEN | CLICKABLE, kein Callback → pressed-visual vorhanden, Firmware-Aktion fehlt | Abschnitt 12.3 offener Hook |
| Setup VERWERFEN | wie UEBERNEHMEN | wie oben |
| Setup SICHERHEITSSTANDARDS | wie UEBERNEHMEN | wie oben |

Setup-Buttons nach C3: Touch erzeugt visuelles Pressed-Feedback (`LV_STATE_PRESSED`),
loest aber keine Firmware-Aktion aus. Das ist intentional fuer den Zwischenzustand
"Callback noch nicht implementiert".

#### Geaenderte Dateien (C3)

| Datei | Aenderung |
| --- | --- |
| `screens.c` | 6 × `lv_obj_set_style_bg_color(... LV_STATE_PRESSED)` hinzugefuegt |
| `.eez-project` | unveraendert (kein EEZ-Edit moeglich, Universal-EEZ-Blocker) |
| `actions.h` | unveraendert (leer) |

#### Build-Ergebnis

`pio run -e teensy41` → **SUCCESS** (29.07 s). Kein Compile-Fehler, keine neuen Warnungen.

#### Zwischenabnahme C3

| Kriterium | Status |
| --- | --- |
| Alle Touch-Felder inventarisiert | ✅ (10 Felder) |
| Mindestflaeche 48×48 nachgewiesen | ✅ (7 Felder OK; 4 ModeTabs 36 px ohne CLICKABLE — kein aktives Touch-Target) |
| Pressed-Visual nur aus Palette | ✅ (6 Buttons, nur `*_BD`-Farben) |
| Keine neue Farbe ausserhalb Palette | ✅ |
| EEZ-Pressed-Blocker dokumentiert | ✅ |
| Header-Feedback-Blocker dokumentiert | ✅ |
| Keine Touch-only-Funktion | ✅ |
| Keine Safety-Entscheidung in EEZ | ✅ |
| `.eez-project` gueltig / unveraendert | ✅ (JSON valide) |
| Build erfolgreich | ✅ (29.07 s, SUCCESS) |
| Noch deaktivierte Felder dokumentiert | ✅ (4 ModeTabs + 3 Setup-Buttons als inaktive Hooks) |

---

### 13.16 V0-Ergebnis: EEZ-Projektdatei — UTF-8-BOM entfernt (GEOEFFNET)

**Symptom:** `.eez-project` liess sich in EEZ Studio 0.27.1 nicht oeffnen.
MRU-Eintrag enthielt kein `projectType`-Feld, obwohl alle anderen Projekte
`projectType: "lvgl"` zeigten.

**Ursachenanalyse (aus EEZ-Studio-Quellcode v0.27.1):**

Ladekette in `open-projects-manager.ts`:
```
_loadProject(filePath)
  → fs.promises.readFile(filePath)        // liefert Buffer
  → fileData.toString("utf8")             // Node.js: BOM wird NICHT entfernt
  → loadProject(projectStore, projectJs)  // projectJs startet mit \uFEFF
    → loadObjectInternal()
      → JSON.parse(jsObjectOrString)      // SyntaxError: Unexpected token \uFEFF
```

`JSON.parse("\uFEFF{...}")` warf in Node.js/V8 eine `SyntaxError`. Diese Exception
wurde in `_loadProject` / `openMainProject` nicht gefangen. `setProject()` wurde nie
aufgerufen. `this.project` blieb `undefined` → MRU-Update zeigte kein `projectType`.

**Ursache:** Unsere `.eez-project`-Datei hatte einen UTF-8-BOM (`\xEF\xBB\xBF`) als
erste 3 Bytes — von EEZ-Studio-nativen Dateien fehlte dieser. EEZ Studio schreibt
Projekte ohne BOM (`fs.writeFile(filePath, json, "utf8")`). Der BOM wurde beim
manuellen Editieren durch ein Werkzeug hinzugefuegt, das UTF-8-with-BOM schreibt.

**Fix:** BOM per Python-Skript (binary read/write) entfernt:
```python
with open(path, 'rb') as f:
    data = f.read()
if data[:3] == b'\xef\xbb\xbf':
    data = data[3:]
with open(path, 'wb') as f:
    f.write(data)
```
Backup liegt unter `.eez-project.bom_backup` (nicht committen).

**Verifikation:** EEZ Studio neu gestartet mit Projektpfad als Argument.
MRU-Eintrag danach: `{ "projectType": "lvgl", "hasFlowSupport": true }` — Laden
erfolgreich bestaetigt.

**Praevention:** Zukunftig `.eez-project` nur mit Tools oeffnen/speichern, die
UTF-8-without-BOM schreiben (VS Code: `"files.encoding": "utf8"`). Nie UTF-8-BOM
oder UTF-16 verwenden.

#### Geaenderte Dateien (V0)

| Datei | Aenderung |
| --- | --- |
| `.eez-project` | UTF-8-BOM (3 Bytes `\xEF\xBB\xBF`) vom Dateianfang entfernt |

---

### 13.6 Risiken und offene Fragen

| Risiko / Frage | Schwere | Massnahme |
| --- | --- | --- |
| Kein `outputFolder` in EEZ-Projekt konfiguriert | **Hoch** | Vor dem ersten Export in EEZ Studio explizit setzen und verifizieren |
| EEZ-Export wuerde `screens.c` (67 KB handgeschrieben) ueberschreiben | **Hoch** | Exportvertrag aus 13.2 einhalten; immer in Staging-Verzeichnis exportieren |
| EEZ-Export wuerde `DukaWidgets`-Struct in `screens.h` loeschen | **Hoch** | Merge-Vertrag aus 13.4 einhalten; spaeter `screens_ext.h` auslagern |
| `eez-flow.cpp/h` (530 KB) sind an Commit d9ff0db4 gebunden | Mittel | Nicht aktualisieren ohne separaten Test-Build; Framework-Version kontrolliert halten |
| `ui.c` `assets[]`-Blob aendert sich mit jedem EEZ-Projektedit | Mittel | Nach jedem EEZ-Export sofort `pio run -e teensy41`; `assets[]`-Aenderung in git diff erkennbar |
| `vars.h` wird bei jedem Export neu generiert | Niedrig | Stabil solange keine Variablen hinzugefuegt/entfernt werden; Indexaenderungen waeren breaking changes fuer `LvglUi.cpp` |
| EEZ-generiertes `create_screen_boot()` koennte andere Widget-IDs erzeugen als handgeschriebenes | Mittel | Diff vor Integration; `objects.boot` muss stabiler Root-Handle bleiben |
| `lvglStyles` hat nur 3 Eintraege, aber 19 Farben sind EEZ-Farbpalette | Niedrig | Styles sind noch nicht runtime-wirksam; Abschnitt 4 dokumentiert den Farb-Ownership-Vertrag |
