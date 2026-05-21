# Dukatimer-Part2 Architekturzugriffspunkte fuer Math, Sensorik und Workflows

## 1. Ziel

Dieses Dokument fixiert frueh die Stellen, an denen Part2 kuenftig auf fotografische Mathematik, F-Stop-/EV-Logik, Messung, Sensorik, Papierdaten, Belichtung und Darstellung zugreifen darf.

Das Ziel ist nicht nur eine saubere Modulstruktur, sondern die gezielte Vermeidung der beiden historischen Fehlmuster:

- `v2.240` als fachlich starke, aber global und monolithisch gekoppelte Ein-Datei-Implementierung
- `v0.9` als bereits gesplittete, aber fachlich weiter zerklueftete App-Struktur mit mehrfach verteilten Zustands-, Mess- und Mathematikzugaengen

Part2 soll von Beginn an breit genug vorbereitet werden, damit spaetere Modi und Workflows nicht wieder eigene Nebenpfade fuer EV, Sensorik, Formatter, Vorschlagslogik oder Persistenz anlegen.

## 2. Historische Lehren

### 2.1 Ursprung `sketch_jan18v_v2_240_Probestreifen_copy`

Der historische Ursprung `historischer Ursprung/sketch_jan18v_v2_240_Probestreifen_copy.ino` zeigt die fachliche Zielbreite sehr klar, aber auch die technische Ueberkopplung.

Charakteristische Koppelstellen des Ursprungs:

- globale Sensor-, Dosis-, Temperatur-, Modus-, Tracking-, Mess- und Probestreifen-Zustaende liegen in derselben Datei und im selben Namensraum
- Probestreifen-Zustand, EV-Raster, Zwischensummen und Exposure-Blockierung liegen als globale Variablen direkt neben Persistenz- und Kalibrierdaten
- Mathematik mutiert direkt die globalen Zeiten und Tracking-Variablen statt ueber klar definierte Services oder Ports zu laufen
- Sensorauslesung, Gain-Umschaltung, Filterung und Messwertaufbereitung liegen in derselben Funktion wie die fachliche Nutzung
- LCD-, LED- und Beep-Rueckmeldungen sitzen direkt in Kalibrier-, Wizard-, Densitometer- und Setupablaeufen
- Persistenz, UI, Mode-Handling, Sensorik und mathematische Ableitung teilen sich globale Schreibrechte

Die Datei ist damit ein starker fachlicher Referenztraeger, aber kein direkt uebernehmbares Architekturvorbild.

### 2.2 `v0.9`

`v0.9` verbessert die Lage gegenueber dem Ursprung deutlich, fuehrt aber ein zweites Fehlmuster ein: fachliche Zugaenge werden auf mehrere Apps verteilt, ohne zentralen Zugriffsschnitt fuer gemeinsame Belichtungslogik.

Typische Beispiele:

- `Dukatimer v0.9/src/BWFStopApp.cpp` besitzt eigene F-Stop-Zeitlogik mit `exp2f(...)`
- `Dukatimer v0.9/src/BurnApp.cpp` besitzt eigene Burn-EV-Mathematik mit `exp2f(_burnEv) - 1.0f`
- `Dukatimer v0.9/src/ZoneModeApp.cpp` besitzt wieder eigene zonenbezogene EV-Ableitung
- `Dukatimer v0.9/src/DensApp.cpp`, `CalibrationApp.cpp`, `SetupApp.cpp` und `TestStripApp.cpp` greifen jeweils direkt auf Messpfade, Lux-Werte und Ergebnisaufbereitung zu
- `Dukatimer v0.9/src/DisplayManager.cpp` formatiert mode-spezifische Rohdaten direkt fuer die UI
- `Dukatimer v0.9/src/SensorManager.cpp` traegt neben Sensorik auch Messablauf- und Filterzustand

Die Zerklueftung ist also nicht mehr monolithisch in einer Datei, sondern verteilt auf mehrere Apps und Manager.

## 3. Aktueller Part2-Stand

### 3.1 Bereits gute Architekturbausteine

Part2 ist heute deutlich besser vorbereitet als `v2.240` und auch strukturierter als `v0.9`.

Bereits tragfaehige Trennungen:

- `src/teensy/SensorManager.h` und `SensorRuntimeStatus.h` halten Sensorinitialisierung, Gueltigkeit, Frische und Thermik getrennt von UI und ExposureEngine
- `src/teensy/ExposureEngine.h` und `ExposureRuntimeState.h` halten den physischen Belichtungszustand lokal und lux-/dosis-/zeitbasiert
- `src/teensy/main.cpp` baut einen reinen `SystemSnapshot` fuer die UI und trennt damit Snapshot-Erzeugung von Darstellung
- `src/teensy/InputNormalizer.*` und `InputRouterPolicy.*` entkoppeln Eingabequellen von Workflow-Semantik
- `src/teensy/ModeCoordinator.*` zentralisiert Moduswechsel und Event-Dispatch
- `src/teensy/LvglUi.cpp` arbeitet bereits ueber `SystemSnapshot` und greift nicht direkt auf Rohhardware zu

Diese Grundlagen muessen erhalten bleiben.

### 3.2 Noch offene Architekturspalten

Der wesentliche Engpass liegt nicht mehr in der physischen Engine, sondern in der fehlenden gemeinsamen Workflow-Zugriffsschicht.

Aktuelle Risiken:

- `IModeWorkflow` kennt nur `onEnter`, `onExit`, `onTick`, `onInputEvent` und `populateRuntimeState`, aber keinen injizierten Zugriff auf gemeinsame Fachdienste
- `main.cpp` enthaelt bereits mode-spezifische SG-Orchestrierung fuer Beobachtung und Startkommandos
- `SplitgradeWorkflow.cpp` enthaelt EV-Mathematik bereits wieder direkt im Workflow
- `LvglUi.cpp` nutzt fuer Textableitung jetzt `UiPresenter`, aber weitere Fachports (Measurement/Paper) sind noch nicht als feste Query-Schnittstellen verdrahtet
- `ModeRuntimeState.h` besitzt derzeit eine echte SG-Unterstruktur, waehrend weitere Modi nur als Shell existieren
- fuer Messung, Histogramm, Undo, Vorschlagslogik und fotografische Messwertreferenzen existiert noch kein eigener fachlicher Dienst

Wenn diese Luecken vor BW, Burn, Teststrip, Preflash, Densitometer und Zone nicht geschlossen werden, entsteht dieselbe Zerklueftung erneut, nur diesmal auf mehreren Part2-Dateien statt in einem Sketch.

## 4. Verbindlicher Sollschnitt

Die folgende Aufteilung ist fuer Part2 verbindlich vorzubereiten, auch wenn einzelne Dienste zunaechst nur als Skelett existieren.

| Fachthema | Einziger fachlicher Besitzer | Direkte Konsumenten | Darf explizit nicht direkt zugreifen | Begründung |
| --- | --- | --- | --- | --- |
| EV-/F-Stop-Mathematik | `ExposureValueMath` | Workflows, Messdienst, Presenter/Formatter | `LvglUi`, `SensorManager`, `ExposureEngine`, Remote-Gateway | dieselbe EV-Logik muss fuer BW, SG, Burn, Teststrip und Preflash identisch sein |
| Messwert- und Messablauflogik | `MeasurementDomainService` | Mess-, Kalibrier-, Densitometer-, Zone-, Teststrip- und LiveView-Workflows | `LvglUi`, `SensorManager`, `EspServiceLink` | Messung ist mehr als Sensorlesen: sie umfasst Referenzwahl, Mittelung, Undo, Histogramm und Vorschlagslogik |
| Lokale Sensorik und Thermik | `SensorManager` | `ExposureEngine`, `MeasurementDomainService`, Snapshot-Aufbau | Workflows, UI, Persistenz | Sensorik soll Status und Samples liefern, aber keine Fachentscheidungen ueber EV, Histogramm oder UI treffen |
| Physische Belichtung | `ExposureEngine` | Workflow-Kommandos, Snapshot-Aufbau | `LvglUi`, `SensorManager`, Paper-Repository | Engine bleibt lux-/dosis-/zeitbasiert und nicht fotografisch interpretiert |
| Papier-, K- und Kalibrierdaten | `PaperProfileService` oder `PaperProfileRepository` | Workflows, Messdienst, Kalibrier-Workflow, Presenter | `LvglUi`, `ExposureEngine` | Papierdaten sind fachlicher Input fuer Mathematik und Vorschlaege, nicht fuer Hardware oder rohe UI |
| Lichtkopf-/Spektrumplanung | `HeadRecipeService` oberhalb von `HeadLightArbiter` | SG, BW, Burn, Preflash, Kalibrierung | `LvglUi`, `SensorManager`, `EspServiceLink` | Lichtauswahl und Spektralrezepte duerfen nicht pro Modus neu wachsen |
| EV/Lux-/Zeit-/Dosis-Darstellung | `MeasurementValueFormatter` plus mode-spezifische Presenter | `LvglUi`, Remote-Render-Pfade | Workflows, `ExposureEngine`, `SensorManager` | UI darf keine fotografische Mathematik oder Referenzpolitik selbst rechnen |
| Snapshot-Zusammenbau | `buildSystemSnapshot()` in `main.cpp` | UI und Debug-Pfade | Workflows | Snapshot bleibt reiner Datencontainer, kein Fachdienst |
| Remote-Transport | `EspServiceLink` | Remote-Integration, Snapshot, Servicepfade | Workflows, Formatter | Remote-Link transportiert Zustaende und Events, besitzt aber keine autoritative Fachlogik |

## 5. Feste kuenftige Zugriffspunkte

### 5.1 Workflow-Schicht

Kuenftige Modi duerfen nicht mehr ueber zufaellige Includes oder globale Eintraege auf Dienste zugreifen. Stattdessen benoetigt die Workflow-Schicht eine feste Service-Injektion.

Empfohlene Basisschnittstelle:

```cpp
struct ModeWorkflowServices {
  ExposureValueMath* exposureMath;
  MeasurementDomainService* measurements;
  ExposureCommandPort* exposureCommands;
  ExposureQueryPort* exposureQuery;
  SensorQueryPort* sensorQuery;
  PaperProfileQueryPort* paperProfiles;
  HeadRecipeService* headRecipes;
  PersistenceCommandPort* persistence;
  RemoteCommandPort* remote;
};
```

Wichtige Regel:

- Workflows erhalten Dienste einmalig injiziert
- Workflows holen sich keine Singletons selbst
- Workflows bauen keine UI-Strings
- Workflows kennen keine Hardwareobjekte wie `SdFs`, `TouchSampler`, `NeoPixelHead` oder `Wire`

Stand 2026-04-30:

- `ModeWorkflowServices` ist als minimales AP-04-Skelett angelegt.
- `IModeWorkflow::bindServices()` und `ModeCoordinator::bindServices()` bilden jetzt den festen Injektionsrand.
- `MeasurementQueryPort` bleibt vorerst ein minimales Vertragsstub-Skelett.
- `PaperProfileQueryPort` besitzt seit AP-05 eine konkrete Read-Query-API; `PaperSlotBankQueryAdapter` bindet Slot-Bank und Persistenzstatus in `main.cpp` an `workflowServices.paperProfiles`.
- Persistenzvalidierung fuer Paper-Slots liefert differenzierte Parse- und Storage-Fehler (u. a. Formatversion, CRC/Blob, Invalid-Bank) fuer kontrollierte Recovery.
- AP-06 erweitert `InputRouterPolicy` zum globalen Modal-/Fokus-/EventGuard-Rand vor dem Workflow-Dispatch (Fault, Confirm, Wait, Touch-Prioritaet).
- `SystemSnapshot` transportiert seit AP-06 Fokus-, Modal- und Guard-Codes, und `UiPresenter`/`LvglUi` nutzen diese als globale Modalebene statt rein SG-spezifischer Overlay-Bedingungen.
- AP-07 nutzt `workflowServices.paperProfiles` nun im `SplitgradeWorkflow` produktiv: aktive Paper-Profile liefern SG-Startwerte und Gradationsvorschlaege, waehrend lokale Target-Edits auf eine gemeinsame Basis und die laufende LUT-Suggestion zurueckgeschrieben werden.

### 5.2 Mathematik

`ExposureValueMath` ist die einzige Stelle fuer:

- EV-Schrittweite aus StepMode
- Multiplikator `2^EV`
- Anwenden von EV auf Zeit oder Dosis
- Burn-Zusatzzeit aus Basiszeit und Burn-EV
- symmetrische Teststrip-Serien um die Basis
- Hilfsfunktionen fuer relative EV-Ableitungen

Wichtige Regel:

- keine `pow`, `powf`, `exp2f`, `log2` fuer fotografische Bedienlogik in Workflows, Apps oder UI ausserhalb dieser Schicht

### 5.3 Messung

`MeasurementDomainService` ist die einzige Stelle fuer:

- Starten, Fortschreiben und Abbrechen fachlicher Messworkflows
- Mittelung und Undo
- Zoneneinordnung und Histogramm
- Bezug auf White-Referenz, Paper-Referenz, Spot-Referenz oder Basis-Lux
- Ableitung von Vorschlaegen fuer BW, SG, Kalibrierung und Densitometrie

Wichtige Regel:

- `SensorManager` liefert Samples und Status
- `MeasurementDomainService` macht daraus Messbedeutung
- `ExposureEngine` nimmt keine Messhistorie oder Vorschlagslogik auf

### 5.4 Sensorik

`SensorManager` bleibt der einzige Besitzer des lokalen TSL2561- und des thermischen Runtime-Status.

Kuenftige direkte Zugriffe auf Sensorik sind nur noch an zwei Stellen erlaubt:

- `ExposureEngine` fuer laufende physische Belichtungsfuehrung
- `MeasurementDomainService` fuer Messworkflows

Workflows und UI lesen Sensorik nur indirekt ueber Query-Ports oder Snapshot/Presenter-Daten.

### 5.5 Presenter und Formatter

Zwischen `SystemSnapshot` und `LvglUi` braucht Part2 eine explizite Presenter-Schicht.

Empfohlene Aufteilung:

- `ModeRuntimeState` bleibt roh, zustands- und workflow-orientiert
- `SystemSnapshot` bleibt roh und hardwarefrei
- `ModePresenter` oder `SystemPresenter` baut daraus UI-taugliche View-Modelle
- `MeasurementValueFormatter` entscheidet zentral ueber EV-plus-Lux oder Lux-only

Wichtige Regel:

- `LvglUi` darf keine eigene Referenzpolitik fuer EV treffen
- `LvglUi` darf keine eigene Schrittweitenlogik oder Serienberechnung enthalten
- Remote-Render-Pfade muessen denselben Presenter-/Formatter-Pfad nutzen wie das lokale UI

## 6. Welche kuenftigen Modi welche Dienste brauchen

Die breite Vorbereitung der Basis bedeutet nicht, alle Modi sofort zu implementieren. Sie bedeutet, ihre Zugriffspunkte jetzt festzulegen.

### 6.1 BW

BW braucht spaeter mindestens:

- `ExposureValueMath`
- `ExposureCommandPort`
- `PaperProfileQueryPort`
- `MeasurementDomainService`
- `MeasurementValueFormatter`

### 6.2 SG

SG braucht spaeter mindestens:

- `ExposureValueMath`
- `ExposureCommandPort`
- `PaperProfileQueryPort`
- `HeadRecipeService`
- `MeasurementDomainService`
- `MeasurementValueFormatter`

### 6.3 Burn

Burn braucht spaeter mindestens:

- `ExposureValueMath`
- `ExposureCommandPort`
- `HeadRecipeService`
- `MeasurementValueFormatter`

### 6.4 Teststrip

Teststrip braucht spaeter mindestens:

- `ExposureValueMath`
- `ExposureCommandPort`
- `MeasurementDomainService`
- `MeasurementValueFormatter`

### 6.5 Preflash

Preflash braucht spaeter mindestens:

- `ExposureValueMath` fuer Schwell- und Rasterlogik, sofern fotografische EV-Bezuege genutzt werden
- `PaperProfileQueryPort`
- `ExposureCommandPort`
- `HeadRecipeService`
- `PersistenceCommandPort`

### 6.6 Kalibrierung, Densitometer, Zone, LiveView

Diese Workflows brauchen spaeter mindestens:

- `MeasurementDomainService`
- `SensorQueryPort`
- `PaperProfileQueryPort`
- `MeasurementValueFormatter`

## 7. Regeln gegen erneute Zerklueftung

Die folgenden Regeln sind fuer Part2 nicht optional.

- Keine Workflow-Klasse rechnet eigene EV-Schritte, Burn-Formeln oder Teststrip-Serien ausserhalb von `ExposureValueMath`.
- Keine Workflow-Klasse formatiert eigene EV-/Lux-Texte fuer lokale oder Remote-UI.
- `LvglUi` und spaetere Remote-Renderer konsumieren Presenter-/Formatter-Ergebnisse, aber keine Fachmathematik.
- `SensorManager` speichert keine Messhistorie, kein Histogramm und keine UI-spezifischen Flags.
- `ExposureEngine` bleibt physikalisch und kennt weder StepMode noch Papierprofil noch F-Stop-Referenz.
- `EspServiceLink` transportiert nur Ereignisse, Status und Servicezugriffe, aber keine autoritative Belichtungs- oder Messlogik.
- `main.cpp` bleibt Integrations- und Wiring-Ort; fachliche Entscheidungen duerfen dort nicht pro Modus anwachsen.
- `ModeRuntimeState` darf nicht zu einer Ansammlung formatierter UI-Texte verkommen; formatiert wird erst im Presenter.

## 8. Konkrete Vorbereitungsarbeiten mit hoher Prioritaet

Die Basis ist erst dann breit genug vorbereitet, wenn mindestens diese Schritte erfolgt sind:

1. `IModeWorkflow` oder seine Konstruktorpfade erhalten eine feste Service-Injektion statt impliziter Querzugriffe.
2. `ExposureValueMath` wird als zentrale Schicht eingefuehrt und der aktuelle SG-Pfad darauf umgestellt.
3. `MeasurementValueFormatter` wird eingefuehrt und die heute direkte SG-Stringbildung aus `LvglUi.cpp` dorthin verlagert.
4. `MeasurementDomainService` wird als Skelett mit klaren Ports fuer Messstart, Mittelung, Undo, Histogramm und Vorschlaege angelegt, auch wenn zunaechst nur Teilfunktionen genutzt werden.
5. Zwischen `SystemSnapshot` und `LvglUi` wird eine Presenter-Schicht vorbereitet, damit weitere Modi nicht wieder eigene UI-Mathematik erzeugen.
6. `main.cpp` bleibt Wiring-Ort, aber die heute SG-spezifischen Glue-Stellen werden schrittweise hinter generische Workflow-Ports verschoben.

## 9. Schlussfolgerung

Part2 ist heute bereits deutlich besser vorbereitet als der historische Ursprung und strukturell sauberer als `v0.9`.

Die eigentliche Gefahr einer neuen Zerklueftung liegt nicht primaer in `ExposureEngine` oder `SensorManager`, sondern in der noch fehlenden gemeinsamen Zugriffsebene fuer Workflows, Messung und Darstellung.

Wenn Part2 jetzt nur weitere Modi auf die bestehende SG-Basis aufsetzt, ohne diese Zugriffsschicht einzuziehen, wiederholen sich die historischen Inkonsistenzen in neuer Form.

Wenn dagegen die hier beschriebenen Zugriffspunkte zuerst festgezogen werden, kann Part2 die historische Funktionsbreite aus `2.240` aufnehmen, ohne erneut in globale Monolithik oder app-spezifische Inseln wie in `v0.9` zu kippen.
