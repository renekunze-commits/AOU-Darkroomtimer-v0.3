# BW-Modus: Umsetzungsplan

Stand: 2026-05-06

Ziel dieses Plans ist die Einfuehrung eines expliziten BW-Modus in
`Dukatimer-Part2`. Der Modus bleibt paper-driven: Das aktive Papierprofil
entscheidet verbindlich, ob BW als fixed-grade-Weisslichtbelichtung oder als
multigrade Mischlichtbelichtung arbeitet.

## Zielbild

- `ModeId::BlackWhite` wird von `ModeShellWorkflow` auf einen produktiven
  Workflow umgestellt.
- Fixed-grade-Papier fuehrt zu einer Einzelexposition mit weissem Licht. Die
  Gradation wird aus `PaperExposureProfile::fixedGradeValue` angezeigt und ist
  im BW-Modus sichtbar gesperrt.
- Multigrade-Papier fuehrt zu einer Einzelexposition mit simultaner Soft-/Hard-
  Kanalmischung. Die Gradation ist in 0.5er Schritten von `0.0` bis `5.0`
  einstellbar.
- Zeit- und Dosisbelichtung laufen wie SG ueber `ExposureEngine`; die lokale
  TSL2561-Dosisintegration bleibt der autoritative Closed-Loop-Pfad.
- EV-/F-Stop-Schritte, Paperprofilzugriff, Measurement-Vorschlaege und
  UI-Formatierung werden nicht als BW-Sonderinsel neu gebaut.

## Nicht-Ziele fuer den ersten BW-Slice

- Keine Burn-/Dodge- oder Teststrip-Implementierung.
- Keine automatische BW-Proposal-Uebernahme aus Measurement-Sessions.
- Keine Densitometer- oder Negativmesslogik.
- Keine direkte ESP32-S3- oder C6-Exposure-Ownership.
- Keine produktive Nutzung ungeklaerter K-Faktor-Einheiten ohne expliziten
  Target-Vertrag.

## Ist-Zustand

Positive Grundlage:

- `ModeId::BlackWhite` existiert in `ModeRuntimeState` und ist im
  `ModeCoordinator` als eigener Modus sichtbar.
- `PaperExposureProfile` enthaelt `PaperGradeMode::FixedGrade`,
  `PaperGradeMode::Multigrade`, `fixedGradeValue`, `kBw`, `kSoft`, `kHard`
  sowie 11 `gradeKSoft[]`/`gradeKHard[]`-Stuetzstellen fuer 0.5er Gradationen.
- `PaperWorkflow` kann `gradeMode`, `fixedGradeValue`, ISO-Werte und K-Faktoren
  lokal editieren und persistieren.
- `SplitgradeWorkflow` hat bereits paper-driven Zielwerte, 0.5er Gradation,
  ISO-P/ISO-R-Skalierung, LUT-basierte Soft-/Hard-Fractions und einen
  schmalen Execution-Command-Rand zur `ExposureEngine`.
- `ExposureEngine` besitzt Zeit- und Dosisstarts, Closed-Loop-Integration,
  Sensor-Fallback, Thermikbegrenzung und Fault-/Done-Zustaende.
- `HeadSpectrumCommand` kennt `BwGradeMix`, `SplitgradeSoft` und
  `SplitgradeHard` als fachliche Lichtsemantiken.

Offene Luecken:

- `main.cpp` instanziiert BW noch als `ModeShellWorkflow`.
- `ModeRuntimeState` hat keine `BwModeRuntimeState`-Felder.
- Es gibt keinen `BlackWhiteWorkflow`, keine BW-Execution-Commands und keine
  Rueckspiegelung des Exposure-Zustands in einen BW-Fachzustand.
- Der Start-/Pause-/Abort-Wiringpfad existiert nur fuer SG in
  `processSplitgradeExecutionCommands()`.
- Der Head-Pfad waehlt aktive Belichtungsfarben noch roh in `main.cpp`: SG hard
  wird Blau, SG soft wird Gruen, alle anderen Exposures werden Gruen+Blau. Das
  erfuellt fixed-grade-Weisslicht nicht.
- `HeadSpectrumCommand` ist laut Header noch nicht als produktiver
  Mapping-Pfad bis `HeadLightCommand` verdrahtet.
- Die UI routet BW aktuell auf `PageSplitgrade` und zeigt nur
  Placeholder-/Papertexte; eine echte `PageBlackWhite` mit BW-Runtime-Vertrag
  fehlt.
- Die Bedeutung von `kBw` ist widerspruechlich: Defaults und SG nutzen `kBw`
  als Zielwert fuer Sekunden/Luxsekunden, waehrend die visuelle
  Schwellenwertmethode in `PaperWorkflow::recalculateFromSteps()` `kBw` als
  dimensionslosen `10^(-D_Keil(N))`-Faktor schreibt. Das muss vor produktiver
  BW-Dosisfuehrung geklaert werden.

## Reihenfolge

Die Arbeitspakete sind bewusst so geordnet, dass zuerst der Vertrag und die
Mathematik festliegen, bevor UI und Head-Ausgabe produktiv geschaltet werden.

```text
AP-BW-00 Entscheidung einfrieren
  -> AP-BW-01 Runtime-Vertrag
  -> AP-BW-02 Paper-/Gradationsmath und Target-Einheit
  -> AP-BW-03 BlackWhiteWorkflow
  -> AP-BW-04 Exposure-Wiring
  -> AP-BW-05 Head-Spectrum-Mapping
  -> AP-BW-06 UI/Presenter
  -> AP-BW-07 Measurement-Integration
  -> AP-BW-08 Tests
  -> AP-BW-09 Hardware-Abnahme
  -> AP-BW-10 Dokumentation/Version
```

## AP-BW-00: Fachentscheidung und Abnahmekern einfrieren

Prioritaet: P1

Ziel: E10 wird von einer Richtung zu einem verbindlichen BW-Modell verdichtet.

Umsetzung:

- `offene_entscheidungen.md` festhalten: Aktives Papierprofil entscheidet
  zwischen fixed-grade und multigrade.
- Fixed-grade bedeutet im BW-Modus: weisses Licht, keine editierbare Gradation,
  `fixedGradeValue` nur Anzeige/Metadatum.
- Multigrade bedeutet im BW-Modus: Gradation `0.0..5.0` in 0.5er Schritten,
  simultaner Soft-/Hard-Mix, eine Zeit- oder Dosisexposition.
- Startbedingungen definieren: kein aktives Papierprofil blockiert produktiven
  BW-Start; unkalibriertes Profil ist mindestens sichtbar zu warnen und fuer
  Dose nur nach expliziter Entscheidung freizugeben.
- Klare erste Abnahme formulieren: manuelle BW-Belichtung mit aktiven
  Papierprofilen, ohne automatische Measurement-Proposals.

Akzeptanz:

- Das Zielverhalten ist in einem Dokument eindeutig und frei von alten
  Alternativmodellen.
- Alle folgenden APs koennen gegen dieselbe fixed/multigrade-Regel testen.

## AP-BW-01: BW-Runtime-Vertrag einfuehren

Prioritaet: P1

Ziel: BW bekommt eigene rohe Runtime-Felder, statt SG-Felder oder
Placeholdertexte mitzubenutzen.

Umsetzung:

- In `ModeRuntimeState.h` ergaenzen:
  - `BwPanel`: zum Beispiel `Inactive`, `Target`, `Grade`, `ControlMode`,
    `Measurement`.
  - `BwExecutionState`: zum Beispiel `Inactive`, `IdleConfig`, `Arming`,
    `Exposing`, `Completed`, `Aborted`, `Fault`.
  - `BwPaperMode`: optional als abgeleiteter UI-Status `NoProfile`,
    `FixedGrade`, `Multigrade`.
  - `BwModeRuntimeState` mit `panel`, `executionState`, `controlMode`,
    `targetValue`, `grade`, `gradeEditable`, `whiteLight`, `softMix`,
    `hardMix`, `parametersDirty`, `paperProfileValid`, Altersfeldern.
- `kModeRuntimeStateSchemaVersion` erhoehen.
- `ModeCoordinator::updateDerivedState()` muss `bw` wie `splitgrade`, `paper`
  und `setup` pro Tick auf Inactive resetten.
- `SystemSnapshot` kann das bestehende `modeState` weiter transportieren; nur
  UI-/Presenter-Code darf spaeter daraus sichtbare Texte bauen.

Akzeptanz:

- Nach Moduswechsel zu BW steht `modeState.activeMode == BlackWhite` und
  `modeState.bw.panel != Inactive`.
- Beim Verlassen von BW werden alle BW-Felder auf Inactive/Nullzustand
  zurueckgesetzt.
- SG-UI und SG-Tests lesen weiterhin nur `modeState.splitgrade`.

## AP-BW-02: Paper-/Gradationsmath aus SG herausloesen

Prioritaet: P1

Ziel: BW und SG nutzen dieselbe Paper- und 0.5er-Gradationslogik, ohne Code zu
kopieren.

Umsetzung:

- Aus `SplitgradeWorkflow.cpp` einen kleinen gemeinsamen Helper ableiten, zum
  Beispiel `PaperGradeMixMath` oder `PaperExposureTargetMath`:
  - Clamp/Quantisierung `grade -> index 0..10 -> grade 0.0..5.0`.
  - `PaperGradeMode::FixedGrade` sperrt Gradationsedits.
  - LUT-Fraction aus `gradeKSoft[]`/`gradeKHard[]`.
  - ISO-P-Skalierung und ISO-R-Bias in exakt derselben Semantik wie SG.
  - Fallback auf lineare 0.0..5.0-Mischung bei fehlenden LUT-Daten.
- SG wird auf diesen Helper umgestellt, ohne sein Verhalten zu aendern.
- BW nutzt denselben Helper fuer die multigrade Mischung, aber nicht fuer
  fixed-grade Weisslicht.
- Target-Einheit klaeren, bevor BW als produktiv gilt:
  - Entweder `kBw` bleibt offiziell der manuelle Basis-Targetwert in Sekunden
    oder Luxsekunden und die Schwellenwertmethode darf ihn nicht mehr als
    dimensionslosen Faktor ueberschreiben.
  - Oder `PaperExposureProfile` bekommt in einer Schema-V2-Migration getrennte
    Felder, zum Beispiel `bwBaseTarget` und `bwThresholdFactor`.
  - Bis diese Entscheidung umgesetzt ist, darf BW-Dosis nicht still aus einem
    unklaren `kBw` starten.

Empfohlene technische Richtung:

- Kurzfristig einen benannten Resolver einfuehren, zum Beispiel
  `resolvePaperBaseTarget(profile, controlMode)`, und alle Starts darueber
  fuehren.
- Mittelfristig das Profilmodell entflechten, damit visuelle
  Schwellenwertdaten und Exposure-Zielwerte nicht dasselbe Feld teilen.

Akzeptanz:

- SG-Akzeptanztests fuer Grade 0, 5, fixed-grade, ISO und Drift laufen
  unveraendert weiter.
- Ein neuer BW-Harness kann fuer MG `G0.0`, `G2.5`, `G5.0` dieselben
  Soft-/Hard-Fractions erwarten wie SG.
- Fixed-grade BW liefert keine Soft-/Hard-Fraction als Belichtungsfarbe,
  sondern `whiteLight == true`.
- Kein neuer BW-Code berechnet EV-Schritte oder `log2` selbst; EV-Schritte
  gehen weiter ueber `ExposureValueMath`.

## AP-BW-03: `BlackWhiteWorkflow` implementieren

Prioritaet: P1

Ziel: `ModeShellWorkflow` fuer BW wird durch einen echten Workflow ersetzt.

Umsetzung:

- Neue Dateien `BlackWhiteWorkflow.h/.cpp` nach dem Muster von
  `SplitgradeWorkflow` anlegen.
- In `main.cpp` `ModeShellWorkflow bwModeWorkflow(...)` durch
  `BlackWhiteWorkflow bwModeWorkflow;` ersetzen.
- `modeWorkflows[]` unveraendert ueber `IModeWorkflow` verdrahten.
- `onEnter()` laedt das aktive Paperprofil ueber `ModeWorkflowServices`.
- Bei fixed-grade:
  - `grade = normalize(fixedGradeValue)`.
  - `gradeEditable = false`.
  - `whiteLight = true`, `softMix = 0`, `hardMix = 0`.
  - Target aus dem geklaerten Target-Resolver.
- Bei multigrade:
  - Startgradation `2.5` oder zuletzt gespeicherter BW-Wert, falls spaeter
    persistiert.
  - `gradeEditable = true`.
  - `softMix`/`hardMix` aus AP-BW-02.
  - Eine gemeinsame Zielgroesse fuer Zeit oder Dosis.
- Input-Rollen:
  - Enc1/Primary: aktueller Hauptwert. Auf `Target` EV-/F-Stop-Step via
    `ExposureValueMath`, auf `Grade` 0.5er Gradationsschritt.
  - Enc2/Secondary: bei `Target` optional grober Target-Step oder ControlMode;
    bei `Grade` ebenfalls 0.5er Gradation, solange editierbar.
  - Enc3/Context: Panelwechsel.
  - Start: Belichtung starten, Pause/Resume bei laufender Engine, Done/Fault
    quittieren.
  - Undo: Abort oder Rueckkehr aus Completed/Aborted.
- `observeExposureState()` bildet Engine-Phasen auf `BwExecutionState` ab.
- `consumeExecutionCommand()` stellt BW-Kommandos fuer den Wiring-Layer bereit.

Akzeptanz:

- BW ist nicht mehr Shell: Eingaben veraendern BW-Runtime-Felder.
- Fixed-grade-Papier ignoriert Gradationsedits sichtbar und technisch.
- Multigrade-Papier quantisiert jede Gradation exakt auf 0.5er Schritte.
- Fehlendes Profil oder ungueltiger Targetwert erzeugt keinen Exposure-Start.

## AP-BW-04: Gemeinsamen Exposure-Wiringpfad schaffen

Prioritaet: P1

Ziel: BW startet Zeit/Dosis ueber dieselben Safety- und Fallback-Grenzen wie
SG, ohne `processSplitgradeExecutionCommands()` zu kopieren.

Umsetzung:

- In `main.cpp` die gemeinsamen Startschritte aus SG in einen Helper ziehen,
  zum Beispiel `executePrintExposureStart(controlMode, targetValue, phaseRole,
  nowMs)`:
  - VFS-/Dateitransaktionssperre pruefen.
  - Fault/Done vor Start sauber clearen/acknowledgen.
  - `localDoseControlForcedTime` anwenden.
  - `ExposureEngine::startDoseExposure()` oder `startTimeExposure()` aufrufen.
  - RemoteMeasurementCommand nur senden, wenn der Modus/Phase es fachlich
    braucht. SG nutzt Soft/Hard; BW fixed-grade und MG brauchen eine eigene
    Rolle statt `hardPhase`-Bool.
- `processBlackWhiteExecutionCommands(nowMs)` analog zu SG einbauen, aber den
  gemeinsamen Helper nutzen.
- `setup()` und `loop()` muessen BW-Exposure-State beobachten und BW-Kommandos
  verarbeiten.
- `InputRouterPolicy::resolveModalState()` muss BW-Fault/Wait/Completed mit
  dem eigenen BW-State korrekt behandeln; globale aktive Exposure-Phasen bleiben
  ohnehin schon ueber `ExposureRuntimeState` gesperrt.

Akzeptanz:

- BW Time-Start und BW Dose-Start laufen durch `ExposureEngine`.
- Lokaler TSL-/Watchdog-Fallback erzwingt auch fuer BW neue Starts auf Time.
- VFS-aktive Transaktion blockiert BW-Starts genauso wie SG-Starts.
- Pause, Resume, Abort, Done-Acknowledge und ClearFault funktionieren in BW
  ueber dieselben globalen Modalaktionen.

## AP-BW-05: Head-Spectrum-Mapping produktiv verdrahten

Prioritaet: P1

Ziel: BW schaltet nicht mehr rohe RGB-Fallbackfarben, sondern fachliche
Spektrumskommandos.

Umsetzung:

- Einen Mapping-Rand einfuehren, zum Beispiel
  `HeadSpectrumCommand -> HeadLightCommand`.
- SG-Soft und SG-Hard zuerst ueber diesen Rand abbilden, damit SG-Verhalten vor
  BW-Verdrahtung regressionsfrei bleibt.
- Fixed-grade BW:
  - Als explizite BW-Weisslichtsemantik modellieren. Empfohlen ist ein eigener
    fachlicher Fall, zum Beispiel `BwWhite` oder `CustomLogicalMix` mit
    `channels.focusWhite = 1.0`, aber nicht `LocalFocus`, weil Fokus ein lokaler
    Bedienzustand ist und keine Belichtungssemantik.
  - Mapping auf physisches Weiss, aktuell `RGB(255,255,255)`, spaeter ueber
    `HeadCalibrationProfile::focusWhite`.
- Multigrade BW:
  - `HeadSpectrumSemantic::BwGradeMix` mit `channels.soft` und `channels.hard`
    aus `BwModeRuntimeState`.
  - Mapping auf die aktuellen Soft-/Hard-Kanaele, heute Gruen/Blau, spaeter ueber
    logisch kalibrierte Kanaele.
- `resolveExposureBaseColor()` durch `resolveExposureSpectrum()` oder einen
  aequivalenten Spectrum-Pfad ersetzen.
- Thermik- und globale Head-Brightness-Limits muessen nach dem Mapping weiter
  greifen.

Akzeptanz:

- SG soft/hard gibt nach der Umstellung dieselben physischen Farben aus wie
  vorher.
- Fixed-grade BW gibt Weisslicht aus, nicht Gruen+Blau.
- MG-BW Grade 0.0 ist weichkanal-dominant, Grade 5.0 hardkanal-dominant,
  Grade 2.5 balanciert beziehungsweise profilgemaess.
- Belichtungsaktive Non-Exposing-Phasen bleiben Head-Off.

## AP-BW-06: `PageBlackWhite` und Presenter-Anbindung

Prioritaet: P1

Ziel: BW bekommt eine eigene produktive PRINT-Seite statt SG-Placeholder.

Umsetzung:

- In `screens.h/.c` eine `SCREEN_ID_PAGE_BLACK_WHITE` beziehungsweise
  `PageBlackWhite` ergaenzen.
- Shared Header und ModeTabs wiederverwenden.
- Widgets fuer BW:
  - Paper-Chip: Slot, Name, CAL/RAW, FG/MG.
  - Modus-Chip: `WHITE` fuer fixed-grade, `MG MIX` fuer multigrade.
  - Gradationswert: `G x.x`, fixed sichtbar gesperrt.
  - Target: Sekunden oder Luxsekunden je `ExposureControlMode`.
  - Mix-Anzeige: Weiss oder Soft/Hard-Prozent beziehungsweise Balken.
  - Exposure-Telemetrie: Restzeit, Dosis, Lux, Head-Prozent, Fault/Fallback.
  - Kontextzeile mit Encoderrollen.
- `LvglUi::computeTargetScreen()` routet BW auf die neue Seite; Measurement
  bleibt nur ueber einen expliziten BW-Panelzustand erreichbar.
- `UiPresenter` erhaelt BW-spezifische Getter oder einen generischen
  Print-Presenter, statt `getSg...()` fuer BW weiterzuverwenden.
- Keine neuen direkt formatierten Mess- oder EV-Texte in `LvglUi`; Formatierung
  bleibt in Presenter/Formatter.

Akzeptanz:

- BW zeigt keine `PLACEHOLDER`-Texte mehr.
- Fixed-grade sperrt die Gradation sichtbar und zeigt Weisslicht an.
- Multigrade zeigt die aktuelle 0.5er Gradation und den Soft-/Hard-Mix an.
- PRINT-Tab bleibt fuer SG und BW aktiv, MEAS nur fuer echte Messpanels.

## AP-BW-07: Measurement- und Proposal-Integration nachlagern

Prioritaet: P2

Ziel: BW kann spaeter Messvorschlaege nutzen, aber der erste produktive BW-Modus
bleibt manuell und sicher.

Umsetzung:

- AP-11f/AP-11g zuerst um Proposal-Eligibility erweitern:
  - eindeutige Quelle,
  - Rollenmodell fuer Samples,
  - Profilstatus,
  - Target-Einheit,
  - Preview/Accept-Vertrag.
- Danach BW-Proposals als Vorschlag in `BlackWhiteWorkflow` uebernehmen, nie
  automatisch aktiv setzen.
- Mixed-source-Sessions bleiben gesperrt, solange kein dokumentierter
  Quellenabgleich existiert.

Akzeptanz:

- BW arbeitet ohne Measurement-Proposal voll manuell.
- Ein spaeterer Proposal kann abgelehnt, previewed und explizit akzeptiert
  werden.
- Keine Measurement-Formel behauptet absolute Papierzone ohne Kalibriervertrag.

## AP-BW-08: Automatisierte Tests und Harnesses

Prioritaet: P1 fuer Workflow/Math, P2 fuer UI-nahe Tests

Ziel: Der BW-Kern bekommt mindestens dieselbe Regressionstiefe wie AP-07 SG.

Umsetzung:

- `test/ap_bw_blackwhite_acceptance/test_main.cpp` anlegen.
- Testfaelle:
  - fixed-grade Profil laedt `fixedGradeValue`, sperrt Grade-Edits und setzt
    `whiteLight`.
  - multigrade Profil quantisiert Grade 0.0, 2.5, 5.0 korrekt.
  - LUT- und ISO-P/ISO-R-Pfade liefern dieselben Mix-Fractions wie SG.
  - Target-Resolver lehnt ungeklaerte/ungueltige Targetwerte ab.
  - Start erzeugt genau ein BW-Startkommando, keine Soft-/Hard-Phasenkette.
  - Exposure Done fuehrt zu `Completed`, Confirm zu `IdleConfig`.
  - Pause/Resume/Abort/Fault-Clear werden als Commands ausgegeben.
  - Fehlendes Paperprofil blockiert Start.
- Head-Mapping-Tests:
  - SG soft/hard unveraendert.
  - fixed BW weiss.
  - MG-BW Mix mit erwarteten Kanalanteilen.
- Bestehendes `test/ap07_splitgrade_acceptance` nach jeder Math-Extraktion
  unveraendert laufen lassen.

Akzeptanz:

- Host-/PIO-Harness deckt BW-Paper-Modi und Execution-State-Uebergaenge ab.
- SG-Harness bleibt gruen.
- Teensy-Build `pio run -e teensy41` ist nach jedem BW-Slice erfolgreich.

## AP-BW-09: Hardware- und Dunkelkammerabnahme

Prioritaet: P1 vor Produktivfreigabe

Ziel: Code- und Math-Erfolg wird am realen Head, Sensorpfad und Papier belegt.

Abnahmepunkte:

- Fixed-grade Papier:
  - aktiven Slot auf `FixedGrade` setzen,
  - BW starten,
  - Head gibt weisses Licht aus,
  - Gradation ist nicht editierbar,
  - Time-Start endet reproduzierbar.
- Multigrade Papier:
  - Grade 0.0, 2.5, 5.0 einstellen,
  - sichtbare und messbare Soft-/Hard-Mix-Aenderung dokumentieren,
  - eine einzige Belichtung ohne Filterwechsel startet.
- Dose/Closed-Loop:
  - lokaler TSL2561 integriert Luxsekunden,
  - Sensor-Fallback markiert Ergebnis sichtbar als nicht closed-loop-abgesichert,
  - neuer Dose-Start wird nach lokalem TSL-/Watchdog-Fault auf Time gezwungen.
- Safety:
  - VFS-aktive Transaktion blockiert Start,
  - Pause/Resume/Abort via Busy-Overlay,
  - Thermik-Derating reduziert Output,
  - Hard-Stop faultet und kann quittiert werden.
- UI/Bedienung:
  - alle BW-Kernpfade sind mit Encodern allein erreichbar,
  - Touch erzeugt keine parallelen Encoder-Doppeldispatches,
  - Remote-Render zeigt kritische Busy/Fault-Informationen konsistent.

Akzeptanz:

- Abnahmeprotokoll mit Firmwareversion, aktivem Paperprofil, Head-Konfiguration,
  Messwerten und beobachtetem Verhalten liegt im Repository.
- Ohne diese Abnahme bleibt BW als code-verifiziert, aber nicht produktiv
  freigegeben markiert.

## AP-BW-10: Dokumentation, Version und Changelog

Prioritaet: P2, aber vor Abschluss jedes groesseren Slices aktualisieren

Umsetzung:

- `offene_punkte.md` von Shell-Luecke auf konkrete Restpunkte umstellen, sobald
  APs abgeschlossen sind.
- `offene_ziele.md` Z5 nach BW-Slices mit `[teilweise umgesetzt]`,
  `[umgesetzt, Abnahme offen]` oder `[umgesetzt]` markieren.
- `offene_entscheidungen.md` E10 nach AP-BW-00 auf gesetzte Entscheidung
  aktualisieren und verbleibende Datenmodellfrage separat fuehren.
- `eez-studio-step-by-step-und-seitenuebersicht.md` um `PageBlackWhite`-
  Runtime-Felder und Widgetvertrag erweitern.
- `CHANGELOG.md` und `FirmwareVersion.h` je nach Projektkonvention pro
  abgeschlossenem Firmware-Slice fortschreiben.

Akzeptanz:

- Alte Aussagen wie "BW ist nur Shell" bleiben nur dort stehen, wo sie fuer den
  jeweiligen Code-Stand noch wahr sind.
- Die Doku unterscheidet sichtbar zwischen code-verifiziert, build-verifiziert
  und hardwarevalidiert.

## Priorisierte Kurzliste

1. AP-BW-00 und AP-BW-02 Target-Einheit klaeren. Ohne das wird BW nur eine neue
   Stelle fuer K-Faktor-Drift.
2. AP-BW-01 Runtime-Vertrag einfuehren. Ohne eigene BW-Felder bleibt die UI an
   SG-Placeholder gebunden.
3. AP-BW-03 und AP-BW-04 Workflow plus Exposure-Wiring umsetzen. Danach ist BW
   erstmals bedien- und startfaehig.
4. AP-BW-05 Head-Mapping umstellen. Erst hier entsteht das geforderte
   fixed-grade-Weisslicht und die MG-Farbmischung.
5. AP-BW-06 UI produktiv machen. Danach ist BW fuer den Bediener sichtbar und
   pruefbar.
6. AP-BW-08 und AP-BW-09 absichern. Ohne Tests und reale Head-/Papierabnahme
   bleibt der Modus nicht produktiv freigegeben.

## Haupt-Risiken

- `kBw` darf nicht gleichzeitig dimensionsloser Schwellenfaktor und
  Exposure-Target sein. Das ist der groesste mathematische Blocker.
- Fixed-grade-Weisslicht darf nicht versehentlich mit lokalem Focus-Licht
  gleichgesetzt werden; die fachliche Semantik muss exposure-spezifisch bleiben.
- BW darf die SG-Felder nicht weiterverwenden, sonst entstehen UI- und
  Modalzustandsdrift.
- Dosis-BW darf nicht mit Wireless-TSL2591-Lux geschlossen geregelt werden;
  Closed-Loop bleibt lokal ueber TSL2561.
- Measurement-Proposals duerfen nicht vor Eligibility und Preview/Accept in BW
  hineinwachsen.

## Definition of Done fuer den expliziten BW-Modus

- `ModeId::BlackWhite` nutzt `BlackWhiteWorkflow`, nicht `ModeShellWorkflow`.
- Fixed-grade-Papier startet BW mit weissem Licht, gesperrter Gradation und
  einem gueltigen Zeit-/Dosisziel.
- Multigrade-Papier erlaubt Gradation `0.0..5.0` in 0.5er Schritten und startet
  eine einzelne Soft-/Hard-Mix-Belichtung.
- Time und Dose laufen durch `ExposureEngine`; Fallbacks, Thermik, VFS-Blockade,
  Pause/Resume/Abort/Fault bleiben wirksam.
- UI zeigt eine echte BW-Seite ohne Placeholdertexte.
- Tests und Hardwareabnahme belegen fixed-grade, MG-Mix, Time, Dose und Safety.