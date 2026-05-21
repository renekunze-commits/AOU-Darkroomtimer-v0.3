# Claude-Sonnet-Prompts fuer die EEZ-UI-Konsolidierung

Stand: 2026-05-08

Diese Datei enthaelt explizite, einzeln ausfuehrbare Prompts fuer Claude Sonnet.
Jeder Prompt ist als eigener Arbeitsschritt gedacht. Nicht mehrere Prompts in
einem Lauf kombinieren, ausser der jeweilige Prompt fordert es ausdruecklich.

Grundregel fuer alle Prompts:

- Arbeitsverzeichnis ist `Dukatimer-Part2`.
- Vor Aenderungen `AGENTS.md` und
  `docs/software/eez-studio-step-by-step-und-seitenuebersicht.md` lesen.
- Historische Ordner nicht editieren.
- Teensy bleibt Besitzer von Snapshot, Presenter, Eingaberouting, Safety,
  Exposure, EV-, Dosis- und Thermiklogik.
- EEZ Studio ist Zielbesitzer fuer Layout, Styles, Komponenten, Touch-Felder und
  Seitenuebergaenge.
- Keine nicht vorhandenen APIs erfinden. Wenn ein geplanter EEZ-Setter,
  Exportpfad oder Action-Hook nicht im Code existiert, dies als Luecke
  dokumentieren statt blind darauf aufzubauen.
- Nach jeder Code-Aenderung mindestens `pio run -e teensy41` anstreben. Wenn der
  Build nicht laeuft, den Grund und den naechstkleineren Validierungsschritt
  dokumentieren.

---

## Prompt 00: Ist-Stand vor dem naechsten Slice pruefen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Pruefe vor dem naechsten EEZ-UI-Slice den tatsaechlichen Stand gegen die
Doku, ohne Dateien zu aendern.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/LvglUi.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/ui.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/styles.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Ermittle, ob sich seit dem letzten Doku-Stand die Runtime-UI, die
   EEZ-Projektstruktur oder der Glue-Code veraendert haben.
2. Liste nur echte Abweichungen, keine Spekulation.
3. Trenne Befunde in `Runtime`, `EEZ-Projekt`, `Doku`, `Validierung`.
4. Gib eine konkrete Empfehlung, welcher naechste Prompt aus dieser Datei als
   naechstes ausgefuehrt werden soll.

Nicht tun:
- Keine Dateien editieren.
- Kein Refactoring vorschlagen, das nicht direkt aus dem Befund folgt.
- Keine Build-Pass-Aussage machen, wenn kein Build gelaufen ist.

Ergebnisformat:
- Kurzfazit in 3 bis 5 Saetzen.
- Danach eine Tabelle mit `Befund`, `Datei`, `Risiko`, `Naechster Schritt`.
```

---

## Prompt A1: Workflow-Familiennummerierung normalisieren

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Normalisiere die UI-Familiennummerierung als Grundlage fuer EEZ-ModeTabs.
Der verbindliche UI-Vertrag lautet:
- 0 = PAPER
- 1 = MEAS
- 2 = PRINT
- 3 = SETUP

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ModeRuntimeState.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Finde alle Stellen, an denen `global_activeWorkflowFamily`,
   `computeWorkflowFamily()` oder ModeTab-Conditions verwendet werden.
2. Aendere die Runtime so, dass `computeWorkflowFamily()` exakt den Vertrag
   `PAPER=0`, `MEAS=1`, `PRINT=2`, `SETUP=3` liefert.
3. Aendere das EEZ-Projekt nur dort, wo vorhandene Conditions auf die alte
   Nummerierung zeigen. Erhalte Namen, Layout und andere Widget-Eigenschaften.
4. Dokumentiere in der bestehenden EEZ-Doku knapp, dass A1 erledigt ist, falls
   Code und EEZ-Projekt erfolgreich synchronisiert wurden.

Abgrenzung:
- Keine neue Navigation implementieren.
- `PageWirelessRemote` bekommt keinen eigenen Familienwert.
- `ModeId`-Enumwerte nicht umnummerieren.
- Keine Touch-Callbacks anfassen.

Validierung:
- JSON des `.eez-project` parsen.
- `pio run -e teensy41` ausfuehren, wenn moeglich.
- Problems/Compile-Fehler fuer die geaenderten Dateien pruefen.

Ergebnisformat:
- Geaenderte Dateien.
- Alte vs. neue Familiennummerierung.
- Validierungsergebnis.
- Offene Risiken.
```

---

## Prompt A2: EEZ-Projekt und Doku als Zielvertrag synchronisieren

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Synchronisiere die Doku mit dem tatsaechlichen EEZ-Projektstand, ohne
Runtime-Verhalten zu aendern.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/styles.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c

Aufgabe:
1. Ermittle aus dem `.eez-project` die aktuellen Pages, UserWidgets, Farben,
   Styles, globalen Variablen und groben Page-Zonen.
2. Vergleiche diese Fakten mit Abschnitt 1, 4, 6, 7 und 9 der Doku.
3. Aktualisiere die Doku nur faktenbasiert. Trenne klar zwischen
   `designseitig vorhanden`, `runtime wirksam`, `offen`.
4. Entferne oder korrigiere Formulierungen, die einen Runtime-Erfolg behaupten,
   obwohl nur das EEZ-Projekt angepasst wurde.

Abgrenzung:
- Keine Firmware- oder EEZ-Projektdateien aendern.
- Keine neuen Ziele erfinden.
- Keine langen Duplikate aus dem `.eez-project` in die Doku kopieren.

Validierung:
- Markdown-Problems fuer die Doku pruefen.
- Stichwortsuche nach veralteten Angaben wie falschen Zeilenzahlen,
  nicht vorhandenen APIs oder widerspruechlichen Statuswerten.

Ergebnisformat:
- Geaenderte Doku-Abschnitte.
- Drei wichtigste bereinigte Inkonsistenzen.
- Noch offene Punkte fuer A3 bis A6.
```

---

## Prompt A3: Glue-Schnittstelle verbindlich klaeren

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Klaere und dokumentiere die reale Glue-Schnittstelle zwischen
`LvglUi` und EEZ, ohne eine nicht vorhandene API zu erfinden.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/LvglUi.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/ui.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h

Aufgabe:
1. Ermittle alle real vorhandenen EEZ-/Flow-Einstiegspunkte und sichtbaren
   Runtime-Handle-Pfade.
2. Dokumentiere eindeutig: aktuell gueltig ist `flow::setGlobalVariable(...)`,
   nicht `ui_set_*`, solange generierte Setter nicht existieren.
3. Falls es tote oder irrefuehrende Doku-Beispiele gibt, korrigiere sie.
4. Optional: Ergaenze im Code nur minimale Kommentare, falls ein bestehender
   Glue-Pfad ohne Kommentar irrefuehrend ist. Keine Logik aendern.

Abgrenzung:
- Keine Migration einzelner Seiten.
- Keine neuen Setter-Funktionen schreiben.
- Keine `pushWidgetsFromSnapshot()`-Bereinigung in diesem Schritt.

Validierung:
- Problems fuer geaenderte Dateien pruefen.
- Wenn Code geaendert wurde: `pio run -e teensy41` anstreben.

Ergebnisformat:
- Realer Glue-Vertrag in 5 bis 8 Bulletpoints.
- Geaenderte Dateien.
- Build-/Problems-Ergebnis.
```

---

## Prompt A4: Farb-Ownership entkoppeln

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Bereinige den Farbvertrag, damit Doku, EEZ-Projekt und Runtime nicht drei
konkurrierende Wahrheiten behaupten.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/styles.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Inventarisiere alle Farbquellen: Doku-Palette, EEZ-Farben, `screens.c`,
   `LvglUi.cpp`, Default-Theme.
2. Markiere in der Doku eindeutig, welche Farben Zielvertrag sind und welche
   nur Runtime-Fallback fuer noch nicht migrierte Seiten sind.
3. Wenn sichere, kleine Code-Aenderungen moeglich sind, ersetze Magic-Hexwerte
   durch zentrale Konstanten oder benannte Fallbacks. Aendere keine sichtbare
   Farbe ohne ausdruecklichen Grund.
4. Keine gruene oder blaue Produktfarbe einfuehren. Bestehende Diagnose-Ausnahmen
   nur dokumentieren, nicht ausweiten.

Abgrenzung:
- Keine komplette Style-Migration.
- Keine EEZ-Exportdateien neu generieren, wenn der Exportpfad nicht geklaert ist.
- Keine visuellen Neuentwuerfe.

Validierung:
- Problems fuer geaenderte Dateien pruefen.
- Wenn Code geaendert wurde: `pio run -e teensy41` anstreben.

Ergebnisformat:
- Farbinventar mit Quelle und Status.
- Geaenderte Dateien.
- Noch verbleibende Farbduplikate mit Begruendung.
```

---

## Prompt A5: Messwerttexte aus `LvglUi` zurueckholen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Entferne fachliche Messwert-Usertextformatierung aus `LvglUi.cpp` und
fuehre sie in `UiPresenter` oder `MeasurementValueFormatter` zurueck.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/UiPresenter.h
- src/teensy/UiPresenter.cpp
- src/teensy/MeasurementValueFormatter.h
- src/teensy/MeasurementValueFormatter.cpp
- src/teensy/MeasurementRuntimeStatus.h

Aufgabe:
1. Finde alle `snprintf()`- oder direkten Stringformatierungen in
   `LvglUi.cpp`, die Messwerte, EV, Alter, Sequenzen, Histogramm- oder
   Sessiontexte fuer Benutzeranzeigen erzeugen.
2. Verschiebe diese Formatierung in bestehende oder neue Methoden von
   `UiPresenter`/`MeasurementValueFormatter`.
3. `LvglUi.cpp` darf danach fuer PageMeasurement nur noch fertige Texte aus
   Presenter/Formatter setzen oder rohe Werte an EEZ-Flow uebergeben.
4. Erhalte die sichtbaren Texte moeglichst exakt, ausser die Doku verlangt eine
   fachliche Korrektur.

Abgrenzung:
- Keine EV-, Dosis- oder Measurement-Domain-Logik aendern.
- Keine neuen mathematischen Regeln einfuehren.
- Keine UI-Layout-Aenderung.

Validierung:
- Bestehende Measurement-Tests/Harnesses ausfuehren, falls vorhanden.
- `pio run -e teensy41` anstreben.
- Problems fuer alle geaenderten Dateien pruefen.

Ergebnisformat:
- Liste der verlagerten Texte.
- Neue/angepasste Presenter- oder Formatter-Methoden.
- Validierungsergebnis.
- Offene Formatierungsreste, falls vorhanden.
```

---

## Prompt A6: Touch-Kontrakt fuer ModeTabs und Setup-Aktionen festlegen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Erstelle einen eindeutigen Touch-Kontrakt fuer ModeTabs und Setup-Actions,
bevor produktive Touch-Felder in EEZ verfeinert werden.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/LvglUi.cpp
- src/teensy/InputRouterPolicy.h
- src/teensy/InputRouterPolicy.cpp
- src/teensy/ModeCoordinator.h
- src/teensy/ModeCoordinator.cpp
- src/teensy/ModeRuntimeState.h

Aufgabe:
1. Ermittele alle aktuell klickbar wirkenden UI-Objekte und vorhandenen
   Event-Callbacks.
2. Erstelle in der Doku eine Touch-Action-Tabelle mit:
   `UI-Feld`, `Action-Name`, `Mindestflaeche`, `Encoder-Alternative`,
   `Firmware-Guard`, `Runtime-Status`.
3. Fuer ModeTabs festlegen: Touch darf nur Familienwechsel anfragen, nicht
   direkte Fachlogik oder Exposure-Aktionen ausfuehren.
4. Fuer Setup festlegen: Apply/Discard/SafetyDefaults muessen dieselbe
   Firmware-Guard-Logik nutzen wie Encoder/Confirm.
5. Nur implementieren, wenn der bestehende Firmwarepfad eindeutig ist. Sonst
   als offener Runtime-Hook dokumentieren.

Abgrenzung:
- Keine neuen Touch-Actions ohne Encoder-Alternative.
- Keine Safety-Entscheidung in EEZ oder LVGL-Callback.
- Keine `PageWirelessRemote`-Geste in diesem Schritt implementieren, ausser der
  Servicepfad ist bereits eindeutig vorhanden.

Validierung:
- Markdown-Problems fuer Doku pruefen.
- Wenn Code geaendert wurde: `pio run -e teensy41` anstreben.

Ergebnisformat:
- Touch-Kontrakt-Tabelle.
- Implementiert vs. offen.
- Naechster konkreter Implementierungsschritt.
```

---

## Prompt B0: EEZ-Exportpfad festlegen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Lege einen sicheren EEZ-Export- und Merge-Vertrag fest, bevor irgendeine
Seite zur EEZ-gefuehrten Runtime migriert wird.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/ui.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/styles.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/actions.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h

Aufgabe:
1. Ermittele, welche Dateien aktuell handgeschrieben, generiert oder gemischt
   sind.
2. Definiere einen Exportvertrag: Welche Dateien darf EEZ Studio ueberschreiben,
   welche Dateien bleiben dauerhaft handgeschrieben, und wo liegen lokale
   Glue-Anpassungen.
3. Dokumentiere den Vertrag in der EEZ-Doku oder einer neuen kurzen
   Begleitdatei, wenn das uebersichtlicher ist.
4. Schlage eine erste risikoarme Pilotseite fuer die Migration vor. Bevorzugt
   Boot oder eine nicht sicherheitskritische Diagnoseseite.

Abgrenzung:
- Keine Seite migrieren.
- Keine EEZ-Dateien neu generieren, wenn der Generator nicht tatsaechlich
   ausgefuehrt wird.
- Keine manuelle Ueberschreibung grosser generierter Dateien ohne Plan.

Validierung:
- Markdown-Problems fuer geaenderte Doku pruefen.
- Kein Firmware-Build noetig, wenn nur Doku geaendert wurde.

Ergebnisformat:
- Exportvertrag als Tabelle.
- Pilotseiten-Empfehlung mit Begruendung.
- Risiken und offene Fragen.
```

---

## Prompt B1: Pilotmigration `PageBootStatus`

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Migriere `PageBootStatus` als erste risikoarme Seite in Richtung
EEZ-gefuehrte Runtime oder dokumentiere exakt, welcher technische Blocker den
Schritt verhindert.

Voraussetzung:
- Prompt A1 und B0 sind abgeschlossen oder ihre offenen Punkte sind bekannt.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/ui.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Vergleiche den EEZ-Boot-Page-Spiegel mit `create_screen_boot()`.
2. Entscheide anhand des Exportvertrags, ob Boot jetzt aus EEZ-Code laufen kann.
3. Wenn ja: stelle Boot auf den EEZ-Pfad um, ohne andere Seiten zu veraendern.
4. Wenn nein: dokumentiere den Blocker und ergaenze eine minimal belastbare
   Zwischenabnahme fuer Boot.
5. `ModeId::None -> Boot`, danach `Splitgrade -> PageSplitgrade` muss erhalten
   bleiben.

Abgrenzung:
- Keine ModeTabs, keine Touch-Gesten, keine Busy-Logik.
- Keine kosmetische Neugestaltung.
- Keine Entfernung der handgeschriebenen Boot-Fallbacks, solange der EEZ-Pfad
  nicht gebaut und validiert ist.

Validierung:
- `pio run -e teensy41` anstreben.
- Problems fuer geaenderte Dateien pruefen.

Ergebnisformat:
- Entscheidung: migriert oder blockiert.
- Geaenderte Dateien.
- Boot-Routing-Nachweis.
- Validierungsergebnis.
```

---

## Prompt B2: `PagePaperWorkspace` migrieren oder blockiert abgrenzen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Bringe `PagePaperWorkspace` kontrolliert naeher an die EEZ-gefuehrte
Runtime, ohne Paper-Workflow, Slotpersistenz oder CAL-Semantik zu beschaedigen.

Voraussetzung:
- Prompt A1, A3, A4, A6 und B0 sind abgeschlossen oder ihre offenen Punkte sind
  dokumentiert.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ModeRuntimeState.h
- src/teensy/PaperWorkflow.cpp
- src/teensy/PaperWorkflow.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Erfasse alle Paper-Widget-Updates in `pushWidgetsFromSnapshot()`.
2. Vergleiche sie mit den vorhandenen EEZ-Paper-Variablen und dem EEZ-Layout.
3. Migriere nur einen klaren Teilbereich, z.B. Header/ModeTabs oder reine
   Statuslabels. Keine gleichzeitige Gesamtmigration, wenn der Exportpfad noch
   nicht bewiesen ist.
4. Erhalte SELECT/CAL, Apply/Discard und Slot-Auswahl exakt.
5. Dokumentiere, welche Paper-Anzeigen noch handgeschrieben sind.

Abgrenzung:
- Keine Aenderung an Papierprofilformat, Storage oder CAL-Mathematik.
- Keine stillen Auto-Overwrites von Paper- oder Print-Werten.
- Keine Touch-only-Funktion.

Validierung:
- Paper-relevante Tests/Harnesses, falls vorhanden.
- `pio run -e teensy41` anstreben.
- Manuelle Abnahmepunkte in der Doku aktualisieren, falls noetig.

Ergebnisformat:
- Migrierter Teilbereich.
- Erhaltene Paper-Vertraege.
- Validierungsergebnis.
- Restliste fuer Paper.
```

---

## Prompt B3: `PageSplitgrade` migrieren oder blockiert abgrenzen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Bringe `PageSplitgrade` kontrolliert naeher an die EEZ-gefuehrte Runtime,
ohne Exposure-, Dosis-, EV-, Splitgrade- oder Safety-Logik zu veraendern.

Voraussetzung:
- Prompt A1, A3, A4 und B0 sind abgeschlossen.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/SplitgradeWorkflow.cpp
- src/teensy/SplitgradeWorkflow.h
- src/teensy/ExposureRuntimeState.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Erfasse alle Splitgrade-Widget-Updates in `pushWidgetsFromSnapshot()`.
2. Vergleiche sie mit EEZ-Variablen und EEZ-Layout.
3. Migriere nur einen risikoarmen Teilbereich, z.B. Header/ModeTabs oder reine
   Statuschips. Belichtungsaktive Anzeigen erst nach eigener Abnahme.
4. Erhalte grosse Hauptzahlen, Parameter-Dirty, SensorFallback, FaultLatched und
   Head-Leistung als Prozent.
5. Dokumentiere explizit, welche Anzeigen weiterhin handgeschrieben bleiben.

Abgrenzung:
- Keine Aenderung an ExposureEngine oder Splitgrade-Mathematik.
- Keine Aenderung an Start/Stop/Abort-Verhalten.
- Keine Busy-Screen-Aenderung in diesem Schritt.

Validierung:
- `pio run -e teensy41` anstreben.
- Problems fuer geaenderte Dateien pruefen.
- Sichtbarer Vertrag: `PRINT`-Tab muss aktiv sein, wenn Splitgrade aktiv ist.

Ergebnisformat:
- Migrierter Teilbereich.
- Nicht beruehrte Safety-/Exposure-Pfade.
- Validierungsergebnis.
- Restliste fuer Splitgrade.
```

---

## Prompt B4: `PageSetup` migrieren oder blockiert abgrenzen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Bringe `PageSetup` kontrolliert naeher an die EEZ-gefuehrte Runtime und
klaere Apply/Discard/SafetyDefaults ohne zweite Eingabelogik.

Voraussetzung:
- Prompt A1, A3, A4, A6 und B0 sind abgeschlossen.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- docs/software/explizites-setup-menue-zielvorgabe.md
- src/teensy/LvglUi.cpp
- src/teensy/ModeRuntimeState.h
- src/teensy/SetupWorkflow.cpp
- src/teensy/SetupWorkflow.h
- src/teensy/SystemSettings.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Erfasse alle Setup-Widget-Updates und alle klickbar wirkenden Setup-Objekte.
2. Migriere nur reine Darstellung oder klar verdrahtete Actions.
3. Fuer jede Action nachweisen: Touch, Enc3/Enc4-Alternative und Firmware-Guard
   fuehren zu derselben Wirkung.
4. CAP/LIVE-Leistungsanzeige muss als Prozent sichtbar bleiben.
5. Dokumentiere offene Setup-Touch-Hooks, falls nicht implementierbar.

Abgrenzung:
- Keine Aenderung an SystemSettings-Persistenz ohne separaten Auftrag.
- Keine Setup-Aktion direkt in EEZ ausfuehren.
- Keine neuen Setup-Menuepunkte erfinden.

Validierung:
- `pio run -e teensy41` anstreben.
- Problems fuer geaenderte Dateien pruefen.
- Setup-Abnahme: Dirty, PersistFailed, Apply, Discard, SafetyDefaults.

Ergebnisformat:
- Migrierter Teilbereich.
- Action-Vertrag je Setup-Button.
- Validierungsergebnis.
- Restliste fuer Setup.
```

---

## Prompt B5: `PageMeasurement` migrieren oder blockiert abgrenzen

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Bringe `PageMeasurement` kontrolliert naeher an die EEZ-gefuehrte Runtime,
nachdem Messwerttexte aus `LvglUi.cpp` entfernt oder als Rest dokumentiert sind.

Voraussetzung:
- Prompt A1, A3, A4, A5 und B0 sind abgeschlossen.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/UiPresenter.h
- src/teensy/MeasurementValueFormatter.h
- src/teensy/MeasurementRuntimeStatus.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Erfasse alle Measurement-Widget-Updates und Histogramm-Anzeigen.
2. Migriere nur Darstellung, nicht Measurement-Domain-Logik.
3. Rohwerte duerfen an EEZ-Flow gehen, benutzerlesbare Texte kommen aus
   Presenter/Formatter.
4. Histogramm muss 11 Zonen behalten und `latestZoneIndex` sichtbar markieren.
5. C6-Encoder und Enc4 duerfen in Beschriftung und Semantik nicht vermischt
   werden.

Abgrenzung:
- Keine Densitometrie implementieren.
- Keine Proposal- oder EV-Mathematik aendern.
- Keine Touch-only-Messfunktion.

Validierung:
- Measurement-Tests/Harnesses ausfuehren, falls vorhanden.
- `pio run -e teensy41` anstreben.
- Problems fuer geaenderte Dateien pruefen.

Ergebnisformat:
- Migrierter Teilbereich.
- Messwertformatierungs-Nachweis.
- Validierungsergebnis.
- Restliste fuer Measurement.
```

---

## Prompt B6: `PageWirelessRemote` erreichbar machen oder bewusst parken

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Klaere `PageWirelessRemote`: entweder einen sicheren Servicepfad zur Seite
implementieren oder die Seite eindeutig als designseitig vorhanden, runtime-seitig
geparkt dokumentieren.

Voraussetzung:
- Prompt A1, A3, A6 und B0 sind abgeschlossen oder ihre offenen Punkte sind
  bekannt.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ModeRuntimeState.h
- src/teensy/InputRouterPolicy.h
- src/teensy/InputRouterPolicy.cpp
- src/teensy/ModeCoordinator.h
- src/teensy/ModeCoordinator.cpp
- src/teensy/EspLinkRuntimeStatus.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h

Aufgabe:
1. Pruefe, ob es bereits einen Service-Menue-, Gesture- oder Diagnose-Mode-Pfad
   gibt, der fuer `PageWirelessRemote` genutzt werden kann.
2. Wenn ein sicherer Pfad existiert: implementiere minimale Navigation hin und
   zurueck, ohne eigenen `ModeId::WirelessRemote` einzufuehren.
3. Wenn kein sicherer Pfad existiert: dokumentiere die Seite als bewusst
   geparkt und definiere den spaeteren Runtime-Hook.
4. Der untere Reiter bleibt `SETUP` oder der vorherige Familienkontext; Remote
   bekommt keinen eigenen ModeTab.

Abgrenzung:
- Keine eigene UI des Handbedienteils auf dem Teensy nachbauen.
- Keine Exposure- oder Paper-Aktionen auf der Remote-Diagnoseseite.
- Keine Touch-Geste ohne Debounce/Guard-Konzept.

Validierung:
- `pio run -e teensy41` anstreben, wenn Code geaendert wurde.
- Problems fuer geaenderte Dateien pruefen.

Ergebnisformat:
- Entscheidung: erreichbar implementiert oder geparkt.
- Navigations-/Rueckkehrvertrag.
- Validierungsergebnis.
- Offene Servicepfad-Fragen.
```

---

## Prompt B7: Busy-Screen nur mit Safety-Gate migrieren

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Pruefe eine moegliche EEZ-Migration des Busy-Screens. Fuehre sie nur aus,
wenn alle Safety-Gates erfuellt sind; sonst dokumentiere den Blocker.

Voraussetzung:
- Prompt A1, A3, A4, A6 und B0 sind abgeschlossen.
- Mindestens eine nicht sicherheitskritische Seite wurde erfolgreich migriert
  und gebaut.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ExposureRuntimeState.h
- src/teensy/ExposureEngine.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Erfasse alle Busy-spezifischen Handles, Pause/Resume/Abort-Callbacks und
   Exposure-Phase-Abhaengigkeiten.
2. Pruefe, ob der EEZ-Pfad eigene Busy-Header-Handles garantiert und nicht die
   normalen Header-Handles ueberschreibt.
3. Migriere nur, wenn Pause/Resume/Abort weiterhin nur Ereignisse puffern und
   Safety-Entscheidungen im Firmwarepfad bleiben.
4. Wenn ein Gate nicht erfuellt ist: keine Migration, sondern Blocker sauber
   dokumentieren.

Abgrenzung:
- Keine Aenderung an ExposureEngine.
- Keine Aenderung an Start/Stop/Pause-Safety.
- Keine Zusammenlegung von Busy-Header und normalen Header-Handles.

Validierung:
- `pio run -e teensy41` zwingend anstreben.
- Manuelle Hardware-Abnahme benoetigt: PreWait, Exposing, Paused, PostWait,
   Done/Fault, Resume/Abort.

Ergebnisformat:
- Safety-Gate-Tabelle.
- Entscheidung: migriert oder blockiert.
- Validierungsergebnis.
- Hardware-Abnahmeliste.
```

---

## Prompt C1: Finale EEZ-Darstellung pro Seite ausarbeiten

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Arbeite die finale Darstellung einer einzelnen EEZ-Seite aus. Bearbeite
nur die vom Benutzer genannte Seite.

Benutzer nennt vor dem Start genau eine Seite aus:
- PageBootStatus
- PagePaperWorkspace
- PageSplitgrade
- PageSetup
- PageMeasurement
- PageWirelessRemote
- Busy

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project
- die Runtime-Dateien der gewaehlten Seite

Aufgabe:
1. Pruefe fuer die gewaehlte Seite, ob Phase A und der relevante B-Schritt
   abgeschlossen sind.
2. Verfeinere nur EEZ-Layout, Styles, Textgroessen, Abstaende, Sichtbarkeiten
   und lokale UI-Zustaende der gewaehlten Seite.
3. Halte die Display-Zonen ein: Header 48 px, Hauptzone 236 px, ModeTabs 36 px.
4. Keine neue Farbe ausserhalb der Palette, keine neue Fachlogik, keine
   Touch-only-Funktion.
5. Wenn Runtime-Code angepasst werden muss, nur den minimalen Glue fuer die
   gewaehlte Seite aendern.

Abgrenzung:
- Keine zweite Seite bearbeiten.
- Keine neuen Firmware-Funktionen.
- Keine Aenderung an EV-, Dosis-, Exposure- oder Measurement-Domain-Logik.

Validierung:
- `.eez-project` als JSON validieren.
- `pio run -e teensy41` anstreben, wenn Runtime-Code oder exportierte Dateien
  geaendert wurden.
- Doku-Status der Seite aktualisieren.

Ergebnisformat:
- Gewaehlte Seite.
- Visuelle Aenderungen.
- Runtime-/Glue-Aenderungen, falls vorhanden.
- Validierungsergebnis.
- Offene Feinschliffe.
```

---

## Prompt C2: Seitenuebergaenge in EEZ ausarbeiten

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Arbeite EEZ-Seitenuebergaenge und Rueckkehrregeln aus, ohne einen globalen
Screen-Stack oder eine zweite Navigation neben dem Firmwarepfad einzufuehren.

Voraussetzung:
- Prompt A1 und A6 sind abgeschlossen.
- Der EEZ-Exportpfad aus B0 ist geklaert.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/LvglUi.cpp
- src/teensy/ModeCoordinator.h
- src/teensy/ModeCoordinator.cpp
- src/teensy/InputRouterPolicy.h
- src/teensy/InputRouterPolicy.cpp
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project

Aufgabe:
1. Dokumentiere den Soll-Uebergang fuer jede Familie: PAPER, MEAS, PRINT, SETUP.
2. Lege fest, welche EEZ-Transition nur visuell ist und welche Firmware-Aktion
   benoetigt.
3. Implementiere nur visuelle EEZ-Transitions, wenn sie keine neue fachliche
   Navigation erzeugen.
4. Firmware-seitige Modewechsel bleiben in ModeCoordinator/InputRouterPolicy.
5. Busy bleibt priorisiert und darf nicht durch normale Transitions verdeckt
   werden.

Abgrenzung:
- Kein globaler Screen-Stack.
- Keine Touch-Route ohne Encoder-Alternative.
- Keine neue Fachseite ohne Runtime-Datenvertrag.

Validierung:
- `.eez-project` als JSON validieren.
- `pio run -e teensy41` anstreben, wenn Runtime-Code geaendert wurde.

Ergebnisformat:
- Uebergangstabelle.
- Implementiert vs. nur dokumentiert.
- Validierungsergebnis.
- Offene Risiken.
```

---

## Prompt C3: Touch-Felder in EEZ finalisieren

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Finalisiere Touch-Felder in EEZ gemaess Touch-Kontrakt, ohne Encoder-
Blindbedienbarkeit zu verlieren.

Voraussetzung:
- Prompt A6 ist abgeschlossen.
- Der relevante Seiten-B-Schritt ist abgeschlossen oder die Seite ist bewusst
  als handgeschriebene Runtime markiert.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/DukatimerPart2TeensyUi.eez-project
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/actions.c
- src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c
- src/teensy/InputRouterPolicy.h
- src/teensy/InputRouterPolicy.cpp

Aufgabe:
1. Fuer jedes Touch-Feld Mindestflaeche 48x48 px nachweisen oder korrigieren.
2. Pressed/Fokus-Zustaende in EEZ gestalten, ohne neue Farben ausserhalb der
   Palette einzufuehren.
3. Jede Touch-Action muss einen Action-Namen, Encoder-Alternative und
   Firmware-Guard besitzen.
4. Touch bestaetigt kurz im Header-Meldungsband, ohne Modal-Guards zu umgehen.
5. Nicht produktionsreife Touch-Felder visuell deaktivieren oder dokumentiert
   als inaktiv markieren.

Abgrenzung:
- Keine Touch-only-Funktion.
- Keine Safety-Entscheidung in EEZ.
- Keine Gestenerkennung ohne eigenen Runtime-Guard.

Validierung:
- `.eez-project` als JSON validieren.
- `pio run -e teensy41` anstreben, wenn Runtime-Code geaendert wurde.
- Doku-Touch-Tabelle aktualisieren.

Ergebnisformat:
- Touch-Feld-Tabelle.
- Geaenderte EEZ-/Runtime-Dateien.
- Validierungsergebnis.
- Noch deaktivierte Felder.
```

---

## Prompt V1: Abschlussvalidierung fuer einen EEZ-UI-Slice

```text
Du bist Claude Sonnet und arbeitest im Repo `Dukatimer-Part2`.

Ziel: Fuehre eine Abschlussvalidierung fuer den zuletzt umgesetzten EEZ-UI-Slice
durch und erstelle ein ehrliches Fazit.

Lies zuerst:
- AGENTS.md
- docs/software/eez-studio-step-by-step-und-seitenuebersicht.md
- alle im letzten Slice geaenderten Dateien

Aufgabe:
1. Pruefe, ob die Aenderung dem passenden Prompt-Ziel entsprach.
2. Pruefe Problems/Compile-Fehler fuer geaenderte Dateien.
3. Fuehre `pio run -e teensy41` aus, wenn Firmware-, EEZ-Runtime- oder
   generierte C/C++-Dateien geaendert wurden.
4. Pruefe `.eez-project` auf valides JSON, wenn es geaendert wurde.
5. Aktualisiere die Doku nur, wenn ein Status wirklich nachweisbar erreicht
   wurde.
6. Liste offene Hardware-Abnahmen, falls das Verhalten nicht am Geraet geprueft
   wurde.

Nicht tun:
- Keine neuen Features im Validierungsschritt.
- Keine fremden/unrelated Aenderungen revertieren.
- Keine Build-Pass-Aussage ohne Exit Code 0.

Ergebnisformat:
- Validierung bestanden/nicht bestanden.
- Gelaufene Kommandos oder Problems-Pruefungen.
- Geaenderte Dateien.
- Offene Risiken.
- Naechster empfohlener Prompt.
```