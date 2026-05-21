# AP-11f Measurement-Range Proposal-Mathematik und Umsetzungs-TODO

Stand: 2026-05-02

## Zweck

Dieses Dokument beschreibt den naechsten fachlichen Slice nach AP-11d/AP-11e:
vorsichtige Vorschlags-/Proposal-Logik auf Basis der aktuellen Measurement-
Session-Range.

Das Ergebnis ist bewusst noch keine direkte Implementierung. Der aktuelle Code
enthaelt genug Grundlage fuer eine belastbare Preview- und Eligibility-Schicht,
aber noch nicht genug belegte Kalibrier- und Einheiteninformation fuer eine
automatische, sofort uebernehmbare Belichtungsvorgabe.

Verbindliche Leitplanke:

- Measurement-Range darf zuerst nur eine Vorschlagsgrundlage sein.
- Ein Vorschlag wird niemals ungeprueft aktiv.
- Die UI zeigt Status, Warnungen und Rechenbasis sichtbar an.
- Apply/Accept ist ein separater bestaetigter Schritt.
- Keine BW- oder SG-Formel darf absolute Papier-Zonen behaupten, solange der
  dafuer noetige Kalibrieranker nicht im aktiven Profil und in der Session
  sichtbar vorhanden ist.

## Kurzfazit

Der sichere naechste Slice ist nicht `Messung -> Zielwerte direkt setzen`,
sondern:

1. `MeasurementDomainService` erhaelt ein Proposal-Statusmodell.
2. Dieses Modell bewertet, ob die aktuelle Session mathematisch ueberhaupt
   proposal-faehig ist.
3. Der erste Vorschlag bleibt eine Preview mit Warnflags.
4. SG kann zuerst einen vorsichtigen Gradations-/Kontrastvorschlag erhalten.
5. Absolute Zeit- oder Dosisvorschlaege bleiben gesperrt, bis K-Faktor-Einheit,
   Messquelle, Kanalbezug und Profilstatus sauber belegt sind.

Der wichtigste Befund ist die Quellenfrage: Die aktuelle Range vergleicht
`relativeEvStops` aus der Session-Historie. Diese Werte sind je Messquelle auf
den ersten gueltigen Sample dieser Quelle bezogen. Eine gemischte Session aus
lokalem TSL2561 und Wireless-TSL2591 darf deshalb nicht ohne expliziten
Quellenabgleich zu einer gemeinsamen fotografischen Proposal-Range verdichtet
werden.

## Gepruefter aktueller Codepfad

### Measurement-Domain

Aktueller Besitzer der Messsession ist `MeasurementDomainService`.

Relevante Dateien:

- `src/teensy/MeasurementQueryPort.h`
- `src/teensy/MeasurementRuntimeStatus.h`
- `src/teensy/MeasurementCommandPort.h`
- `src/teensy/MeasurementDomainService.h`
- `src/teensy/MeasurementDomainService.cpp`

Aktueller Rechenweg:

```text
Sensor/Gateway sample
  -> MeasurementLuxSample
  -> establishSessionReference(sample.source)
  -> buildSessionSample(sample, reference)
  -> relativeEvStops = log2(sample.lux / reference.lux)
  -> zoneIndexFromRelativeEv(relativeEvStops)
  -> sessionUndoHistory_
  -> syncSessionStatusFromHistory()
  -> shadowSample, highlightSample, relativeEvSpanStops
```

Wichtige Eigenschaften:

- Jede Quelle hat ihren eigenen Session-Anker.
- `MeasurementSessionStatus::rangeValid` bedeutet nur, dass aus der History ein
  Minimum und Maximum gebildet werden konnte.
- `shadowSample` ist aktuell der Sample mit dem kleinsten session-relativen EV.
- `highlightSample` ist aktuell der Sample mit dem groessten session-relativen
  EV.
- `relativeEvSpanStops = highlight.relativeEvStops - shadow.relativeEvStops`.
- Diese Namen sind im aktuellen Code keine absolute Papierzonenzuordnung.
- Undo fuehrt `syncSessionStatusFromHistory()` erneut aus und kann die Range
  dadurch korrekt veraendern.
- Wireless-Captures sind sequence-getrieben; lokale Captures laufen ueber
  `captureLocalSample()`.

### EV-Mathematik

Aktueller Besitzer der zentralen EV-Grundformeln ist `ExposureValueMath`.

Relevante Formeln:

```text
relativeEv = log2(lux / referenceLux)
multiplier = 2 ^ evDeltaStops
adjusted = baseValue * multiplier
```

Diese Formeln sind fuer relative Messwerte und interaktive EV-Schritte geeignet.
Sie enthalten aber noch keine kalibrierte Abbildung von Lux auf Papierzone,
Papierdichte, Gradation oder Belichtungsziel.

### Splitgrade-Workflow und Papierprofil

Aktueller Besitzer der aktiven SG-Targets ist `SplitgradeWorkflow`.

Relevante Dateien:

- `src/teensy/PaperExposureProfile.h`
- `src/teensy/SplitgradeWorkflow.h`
- `src/teensy/SplitgradeWorkflow.cpp`
- `test/ap07_splitgrade_acceptance/test_main.cpp`

Aktueller SG-Rechenweg:

```text
PaperExposureProfile
  -> kBw als baseTarget_
  -> grade_ in 0.5er Raster
  -> resolveSplitFraction(grade)
  -> softTarget_ = scaledBaseTarget * softFraction
  -> hardTarget_ = scaledBaseTarget * hardFraction
  -> StartSoft/StartHard command
  -> main.cpp startet Time- oder Dose-Exposure mit targetValue
```

Wichtige Eigenschaften:

- `gradeKSoft[]` und `gradeKHard[]` werden als 11er-LUT fuer 0.5er Gradationen
  genutzt.
- `useIsoMath` nutzt `isoP` als CHD-Speed-Skalierung `100 / isoP`.
- `isoR` wirkt aktuell als begrenzter Split-Bias, nicht als nass validierter
  Papiercharakteristik-Fit.
- Manuelle Soft/Hard-Edits werden als fixed-point Soft-Anteil gespeichert, um
  Roundtrip-Drift zu vermeiden.
- Das AP-07-Harness prueft 0-Phasen, 0.5er LUT, ISO-P, ISO-R, Fallbacks und
  Drift-Roundtrips.

Mathematische Warnung:

- Historische K-Faktoren sind `lux * time` beziehungsweise praktisch
  `luxSeconds`-artige Groessen.
- Der aktuelle `SplitgradeWorkflow` verwendet `kBw` aber als `baseTarget_`, das
  je nach `ExposureControlMode` entweder Sekunden oder Luxsekunden bedeutet.
- Solange diese Einheit nicht explizit getrennt ist, darf Proposal-Logik keine
  automatische Umrechnung `K / lux` in den laufenden SG-Targetpfad schreiben.

### ExposureEngine-Einheiten

Aktueller Besitzer der physischen Belichtung ist `ExposureEngine`.

Einheiten:

- `startTimeExposure(seconds)` erwartet Sekunden.
- `startDoseExposure(targetDoseLuxSeconds)` erwartet Luxsekunden.
- `currentDose` ist integrierte Luxsekunden aus lokalem TSL2561.
- `measuredLux` stammt aus dem lokalen Closed-Loop-Sensorpfad.

Daraus folgt:

- Ein Time-Proposal und ein Dose-Proposal sind unterschiedliche Dinge.
- Ein papierseitiger TSL2591-Spotwert darf nicht ungeprueft als lokaler
  TSL2561-Dosewert verwendet werden.
- Der erste Proposal-Slice muss die Zielgroesse explizit typisieren.

## Dokumentierte Zielsetzung

### Pflichtenheft

Relevante Muss-Anforderungen:

- PF-M05: Messpunkte erfassen, Zonensystem, Histogramm, Undo.
- PF-M06: Aus Messpunkten muessen Vorschlaege fuer BW und SG abgeleitet werden
  koennen; Vorschlaege duerfen nicht ungeprueft aktiv werden.
- PF-M10: Papierprofile, Kalibrierung und K-Faktoren muessen reproduzierbar
  gepflegt werden.

Wichtig: PF-M06 fordert Proposal-Funktionalitaet, aber nicht sofortiges Apply.
Das bestaetigt den Preview-/Accept-Ansatz.

### Architekturzugriffspunkte

Die Architektur legt fest:

- `MeasurementDomainService` ist einziger Besitzer von Messung, Histogramm,
  Undo, Referenzen und Vorschlagslogik.
- `ExposureValueMath` ist einziger Besitzer gemeinsamer EV-/F-Stop-Formeln.
- `LvglUi` und Remote-Render duerfen keine eigene Referenzpolitik und keine
  eigene Proposal-Mathematik rechnen.
- `ExposureEngine` bleibt physisch und kennt keine Papier- oder UI-Semantik.

Daraus folgt:

- Proposal-Berechnung gehoert in die Measurement-Domain oder einen direkt daran
  gebundenen fachlichen Dienst.
- Splitgrade darf Vorschlaege anwenden, aber nicht selbst Messsession-Mathematik
  erfinden.
- Presenter duerfen Proposal-Ergebnis und Warnflags formatieren, aber nicht
  entscheiden.

### Offene Ziele und Entscheidungen

Aktuelle Ziel- und Entscheidungsdokumente setzen weitere Grenzen:

- Z3 fordert eine fotografisch belastbare Messdomain mit Spot/Multi-Spot,
  Histogramm, Quellenalter, Undo, Highlights/Shadows und Papierbezug.
- Z4 fordert reale Kalibrierung mit Rohdaten und Rueckschreibebegruendung.
- Z5 fordert vollstaendige Workflows, aber BW ist noch Shell.
- E5 legt die Richtung nahe: TSL2561 lokal fuer Closed-Loop/Safety,
  TSL2591 remote fuer Papier-/Transmissionsmessung.
- E10 laesst das BW-Modell noch offen.
- E7 verlangt sichtbaren Demo/gemessen/importiert-Status fuer Papierdefaults.

Konsequenz:

- SG-Preview kann zuerst kommen.
- BW-Proposal bleibt als mathematischer Blocker dokumentiert, bis der BW-Workflow
  und sein Modell entschieden sind.
- Ein Proposal muss Profilstatus und Messquelle sichtbar in die Eligibility
  einbeziehen.

## Historische Vorarbeiten und ihre Bewertung

### 2.240 / historischer Ursprung

Der historische Ursprung enthaelt eine klare K-Faktor-Logik:

```text
K = luxAvg * calibrationTime
time = K / luxAvg
```

Belegte Stellen:

- Kalibrierwizard schreibt `Khard = avg * time_hard`.
- Kalibrierwizard schreibt `Ksoft = avg * time_soft`.
- Kalibrierwizard schreibt `Kbw = avg * time_bw`.
- `applyMeasuredLuxToTimes(luxAvg, mm)` setzt fuer BW, G0 und G5 direkt
  `time_bw`, `time_soft` oder `time_hard` mit `K / lux`.

Diese Logik ist fachlich wichtig, aber fuer Part2 nicht direkt uebernehmbar:

- Sie setzt voraus, dass Messlux, K-Faktor und Zielzeit aus derselben
  Geometrie/Sensor-/Papierbeziehung stammen.
- Sie unterscheidet nicht sauber Preview und Apply.
- Sie kennt keine duale lokale/remote Sensorrolle.
- Sie kennt keine getrennte Time-/Dose-Zieltypisierung wie Part2.

### Probestreifen-Mathematik

Der historische Ursprung enthaelt eine stabile EV-Serie fuer Teststrips:

```text
r = 2 ^ stepEv
a = Tbase * (r - 1) / (r ^ (kstar + 1) - 1)
Ai = a * r ^ i
```

Diese Mathematik ist fuer Proposal indirekt relevant:

- Sie bestaetigt, dass Serien und Bedienraster EV-basiert sein sollen.
- Sie gehoert spaeter in `ExposureValueMath`.
- Sie ist kein direkter Auto-Proposal-Algorithmus fuer SG/BW.

### v0.3 Auto-SG

v0.3 enthaelt einen sichtbaren, aber fachlich riskanten Auto-SG-Ansatz:

```text
zoneLow  = erster belegter Histogramm-Bucket
zoneHigh = letzter belegter Histogramm-Bucket
evRange = zoneHigh - zoneLow
measuredRange = evRange / 3.32
grade = 5.0 * (1.5 - measuredRange) / 0.9
anchorShiftEV = 8.0 - zoneLow
suggestedTime = baseTime * 2 ^ anchorShiftEV
```

Bewertung:

- Positiv: Es gibt eine historische Absicht, Range in Gradation und Zeitvorschlag
  zu ueberfuehren.
- Positiv: Der Vorschlag arbeitet mit Histogramm-Extrema und EV-Faktoren.
- Negativ: `zoneLow`/`zoneHigh` sind integerisierte Buckets, nicht echte Samples.
- Negativ: `measuredRange / 3.32` ist ein Dichte-/LogD-artiger Schritt, aber im
  aktuellen Part2 nicht als Kalibriervertrag belegt.
- Negativ: `timer_base_seconds` ist historisch ein toter oder zumindest unsauber
  gepflegter Anker.
- Negativ: Highlights/Shadows wurden in v0.3 UI-seitig getrennt, mathematisch
  aber nicht sauber als getrennte Sessionrollen gespeichert.

Fazit: Die Formel ist eine Referenz fuer Zielrichtung und Warnungen, aber kein
unveraendert portierbarer Algorithmus.

### v0.3 Auto-BW

v0.3 enthaelt spaeter eine Auto-BW-Idee:

```text
measuredEv = log2(lux)
evShiftToTarget = targetZone - measuredEv
suggestedDose = baseDose * 2 ^ evShiftToTarget
```

Bewertung:

- Positiv: Der Vorschlag wird gepuffert und nicht direkt auf `dose_bw`
  geschrieben.
- Negativ: `log2(lux)` benutzt einen versteckten 1-Lux-Anker.
- Negativ: Der Code kommentiert selbst die fehlende globale Referenzgroesse fuer
  absoluten Lux-zu-Zone-Bezug.
- Negativ: BW-Modell ist in Part2 noch offen und nur Shell.

Fazit: Dieser Pfad darf in Part2 nicht als fertige BW-Formel dienen.

### v0.9

v0.9 verbessert Architektur und Pending-Flags, liefert aber keine belastbare
fertige Proposal-Mathematik fuer Part2:

- `SensorManager` kapselt Messungen besser.
- `SystemContext` kennt `bwAutoPending` und `sgAutoPending`.
- `CalibrationApp` misst G0/G5 und delegiert an `PaperManager`.
- `ExposureEngine` kann Pending-States ausfuehren.
- Apps enthalten weiter eigene F-Stop-/Mode-Mathematik.

Fazit: v0.9 bestaetigt den getrennten Pending-/Apply-Pfad, ersetzt aber nicht
die zentrale Part2-Measurement-Domain.

## Mathematisches Arbeitsmodell fuer AP-11f

### Groessen und Einheiten

| Groesse | Einheit | Besitzer | Bemerkung |
| --- | --- | --- | --- |
| `lux` | Lux | `MeasurementLuxSample` | Rohnahe Messgroesse je Quelle. |
| `referenceLux` | Lux | `MeasurementReferenceStatus` | Erster gueltiger Sample pro Quelle in aktueller Session. |
| `relativeEvStops` | EV/Stufen | `ExposureValueMath` / Measurement-Domain | `log2(lux / referenceLux)`. |
| `relativeEvSpanStops` | EV/Stufen | Measurement-Domain | `max(relativeEvStops) - min(relativeEvStops)`. |
| `zoneIndex` | 0..10 diskret | Measurement-Domain | Aktuell Anzeige-/Histogramm-Bucket, keine absolute Papierzone. |
| `kBw`, `kSoft`, `kHard` | derzeit uneindeutig | `PaperExposureProfile` | Historisch K-Faktor, aktuell im SG-Workflow auch Targetbasis. |
| `grade` | 0.0..5.0 in 0.5er Schritten | `SplitgradeWorkflow` | Aktuelle SG-Gradation. |
| `softTarget`, `hardTarget` | Sekunden oder Luxsekunden | `SplitgradeWorkflow` / `ExposureEngine` | Bedeutung haengt an `ExposureControlMode`. |
| `currentDose` | Luxsekunden | `ExposureEngine` | Nur lokaler TSL2561-Closed-Loop-Pfad. |

### Sichere Invarianten

Ein erster Proposal-Slice muss diese Invarianten erfuellen:

- Keine Division durch Lux <= 0.
- Keine Formel nutzt `log2(lux)` ohne explizite Referenz.
- Mixed-source Sessions sind nicht proposal-faehig, solange kein Quellenabgleich
  dokumentiert und im Code sichtbar ist.
- `relativeEvSpanStops` ist immer >= 0, wenn `rangeValid` wahr ist.
- Undo muss Proposal-Status sofort invalidieren oder neu berechnen.
- Ein Proposal bleibt Preview, bis ein Workflow es explizit uebernimmt.
- Proposal-Accept darf keine PaperProfile-Kalibrierwerte veraendern.
- Proposal-Accept darf keine laufende oder wartende Belichtung ueberschreiben.
- Vorschlaege nennen Zieltyp und Einheit: Time seconds oder Dose luxseconds.
- Grade-Quantisierung erfolgt einmalig auf 0.5er Schritte und darf keine
  Roundtrip-Drift erzeugen.

### Was mit der aktuellen Session-Range sofort belastbar ist

Belastbar ist:

```text
rangeStops = highlight.relativeEvStops - shadow.relativeEvStops
```

Das ist eine relative Kontrastspanne innerhalb einer Session und innerhalb
einer kompatiblen Messquelle.

Belastbar ist auch:

```text
relativeExposureMultiplier = 2 ^ deltaStops
```

Aber nur, wenn `deltaStops` auf einen expliziten vorhandenen Basiswert angewandt
wird, dessen Einheit bekannt ist.

### Was noch nicht belastbar ist

Nicht belastbar ist aktuell:

```text
absoluteZone = f(lux)
absoluteTime = K / lux
absoluteDose = K / lux
grade = f(rangeStops) als freigegebene Papierwahrheit
```

Gruende:

- Aktuelle Zone-Buckets sind session-relativ.
- PaperProfile hat noch keinen Quellen-/Statusvertrag fuer Demo/gemessen/importiert.
- `kBw` wird im aktuellen SG-Pfad als Targetbasis genutzt, aber historisch als
  K-Faktor verstanden.
- BW ist nur Shell und E10 ist offen.
- TSL2561 und TSL2591 haben unterschiedliche Messgeometrien und Rollen.
- Es fehlt ein expliziter Messrollen- oder Zielzonenanker pro Sample.

## Vorschlag fuer den ersten sicheren Algorithmus-Slice

### Phase A: Eligibility und Preview, keine Zielmutation

Der erste Code-Slice sollte nur einen Proposal-Status berechnen:

```text
if session.sampleCount < 2:
  state = InsufficientSamples
elif !session.rangeValid:
  state = NoRange
elif session contains mixed sources:
  state = MixedSourceBlocked
elif active profile missing or not calibrated:
  state = ProfileUntrusted
else:
  state = RangePreview
  contrastRangeStops = session.relativeEvSpanStops
  source = common source
  shadow = session.shadowSample
  highlight = session.highlightSample
```

Dieser Slice darf anzeigen:

- Range in Stops.
- verwendete Quelle.
- Shadow-/Highlight-Sample aus Code-Sicht.
- Sample-Anzahl.
- Warnungen fuer Profilstatus, Quelle, gemischte Quellen, fehlende Rollen,
  fehlende absolute Kalibrierung.

Dieser Slice darf noch nicht:

- SG-Targets setzen.
- BW-Targets setzen.
- PaperProfile-Werte rueckschreiben.
- eine Belichtung starten.

### Phase B: SG-Kontrast-/Gradationspreview

Erst wenn Phase A stabil ist, kann eine SG-Gradationspreview folgen.

Minimaler sicherer Ansatz:

```text
rangeStops = session.relativeEvSpanStops
rangeLogD = rangeStops / log2(10)
```

`rangeLogD` ist nur eine Darstellung der gemessenen Dichte-/Transmissionsspanne,
nicht automatisch Papierdichte.

Historischer Kandidat aus v0.3:

```text
gradeRaw = 5.0 * (1.5 - rangeLogD) / 0.9
grade = clamp(quantizeToHalfStep(gradeRaw), 0.0, 5.0)
```

Dieser Kandidat darf nur als experimentelle Preview mit Warnflag eingefuehrt
werden, weil die Konstanten `1.5` und `0.9` fuer Part2 noch nicht nass
validiert sind.

Bessere Zielrichtung fuer spaeter:

- Zielbereich und Mapping aus PaperProfile/Kalibrierlauf ableiten.
- ISO-R oder SG-LUT nicht blind als Range-Mapping missbrauchen.
- Wenn Profile echte gemessene LUT-Daten tragen, daraus eine monotone
  Gradationswahl ableiten.

### Phase C: SG-Exposure-Preview mit explizitem Anchor

Ein Zeit- oder Dosisvorschlag braucht zusaetzlich eine Platzierungsregel.

Notwendige Inputs:

- eindeutige Messquelle
- eindeutiger Zielmodus: Time oder Dose
- eindeutiger Basiswert mit Einheit
- aktives Profil mit vertrauenswuerdigem Status
- Sample-Rolle oder Anchor-Policy, zum Beispiel `highlight -> target print zone`
- dokumentierter Kanalbezug fuer SG: Soft, Hard, fixed grade oder aktuelle SG-Mischung

Ohne diese Inputs bleibt der Vorschlag gesperrt.

Moegliches spaeteres Time-Modell, wenn K-Faktor-Einheit geklaert ist:

```text
timeSeconds = kFactorLuxSeconds / measuredLux
```

Moegliches spaeteres Dose-Modell, wenn lokaler Closed-Loop-Dosisanker geklaert
ist:

```text
targetDoseLuxSeconds = baseDoseLuxSeconds * 2 ^ deltaStops
```

Beide Modelle duerfen nicht vermischt werden.

### Phase D: Accept/Apply als Workflow-Schritt

Ein Proposal wird erst aktiv, wenn ein Workflow es uebernimmt.

Sicherer Vertrag:

```text
MeasurementDomainService berechnet ProposalStatus
SplitgradeWorkflow zeigt Proposal-Panel/Chip
Confirm auf Proposal fordert Accept an
Workflow prueft Idle/kein Fault/kein Wait
Workflow uebernimmt vorgeschlagene Runtime-Targets
parametersDirty_ wird gesetzt
PaperProfile wird nicht automatisch veraendert
Start bleibt separater Exposure-Start
```

Damit bleibt PF-M06 erfuellt: Vorschlaege koennen abgeleitet werden, werden aber
nicht ungeprueft aktiv.

## Datenmodell-TODO

### Neue Proposal-Typen

Empfohlene neue Header-Datei:

```text
src/teensy/MeasurementProposalStatus.h
```

Vorgeschlagene Typen:

```cpp
enum class MeasurementProposalState : uint8_t {
    None,
    InsufficientSamples,
    NoRange,
    MixedSourceBlocked,
    ProfileMissing,
    ProfileUntrusted,
    RangePreview,
    SplitgradeGradePreview,
    SplitgradeExposurePreview,
    BlockedByOpenBwModel,
};

enum class MeasurementProposalScope : uint8_t {
    None,
    Splitgrade,
    BlackWhite,
};

enum class MeasurementProposalWarning : uint16_t {
    None = 0,
    MixedSources = 1 << 0,
    UncalibratedProfile = 1 << 1,
    DemoProfile = 1 << 2,
    MissingSampleRole = 1 << 3,
    MissingExposureAnchor = 1 << 4,
    ExperimentalGradeMapping = 1 << 5,
    UnitAmbiguity = 1 << 6,
    BwModelOpen = 1 << 7,
};
```

Vorgeschlagene Statusfelder:

```cpp
struct MeasurementProposalStatus {
    uint16_t schemaVersion;
    MeasurementProposalState state;
    MeasurementProposalScope scope;
    uint16_t warningFlags;
    bool valid;
    bool acceptAllowed;
    MeasurementLuxSource source;
    uint32_t sampleCount;
    MeasurementSessionSample shadowSample;
    MeasurementSessionSample highlightSample;
    float rangeStops;
    float rangeLogD;
    bool proposedGradeValid;
    float proposedGrade;
    bool proposedTargetsValid;
    ExposureControlMode targetMode;
    float proposedSoftTarget;
    float proposedHardTarget;
    float proposedBaseTarget;
};
```

Wichtig:

- `valid` bedeutet nur, dass der Status konsistent berechnet wurde.
- `acceptAllowed` bedeutet, dass der Workflow ihn wirklich uebernehmen darf.
- `proposedTargetsValid` darf in AP-11f wahrscheinlich noch false bleiben.

### Erweiterung der Measurement-Session

Fuer Proposal reichen die aktuellen Session-Felder langfristig nicht aus.

Notwendige Erweiterungen:

- `sourceMask` oder `commonSource` in `MeasurementSessionStatus`.
- `mixedSource` Flag.
- optional `role` pro Sample: `Unspecified`, `Shadow`, `Highlight`, `Midtone`,
  `Reference`, `PaperWhite`, `PaperBlack`.
- optional `channel` pro Sample: `White/BW`, `Soft/G0`, `Hard/G5`, `CurrentSG`,
  falls spaeter kanalbezogene K-Faktoren angewandt werden.
- optional `targetZone` oder `placementIntent`, wenn der Nutzer eine konkrete
  Zone anwaehlt.

Ohne Rolle und Kanal darf die Domain nur Range- und Gradationspreview berechnen,
keine absolute Exposure-Preview.

## Implementierungs-TODO

### AP-11f.1 Dokumentation und Entscheidungsschnitt

- [x] Aktuellen Codepfad fuer Measurement-Range, EV-Math, SG-Math und
  ExposureEngine-Einheiten auditieren.
- [x] Historische K-Faktor-, Auto-SG-, Auto-BW- und Probestreifen-Mathematik
  bewerten.
- [ ] Entscheiden, ob `kBw/kSoft/kHard` im aktiven Part2-Modell
  `Luxsekunden-K-Faktoren`, `Sekunden-Defaults` oder zwei getrennte Felder
  sein sollen.
- [ ] Entscheiden, ob AP-11f zuerst nur SG behandelt und BW explizit blockiert
  bleibt. Empfehlung: ja.
- [ ] Entscheiden, ob gemischte lokale/wireless Session-Ranges fuer Proposal
  hart blockiert werden. Empfehlung: ja, bis Quellenabgleich implementiert ist.

### AP-11f.2 Proposal-Statusmodell ohne Apply

- [ ] `MeasurementProposalStatus.h` anlegen.
- [ ] `MeasurementRuntimeStatus` um `proposal` erweitern und Schema-Version
  anheben.
- [ ] `MeasurementQueryPort` um `proposalStatus()` erweitern.
- [ ] `MeasurementDomainService` um `syncProposalStatusFromSession()` erweitern.
- [ ] Proposal-Neuberechnung bei Sample-Append, Undo und Reset ausloesen.
- [ ] Mixed-source-Erkennung in der Session-History implementieren.
- [ ] Profileinbindung nur lesen, nicht schreiben. Falls der Measurement-Domain
  noch kein PaperProfile-Query-Port zur Verfuegung steht, zuerst den Port sauber
  injizieren statt globale Zugriffe zu bauen.
- [ ] UI/Presenter nur mit Status/Warnungen versorgen, keine Mathematik in
  `LvglUi` einbauen.

Akzeptanz:

- Eine Session mit weniger als zwei Samples zeigt `InsufficientSamples`.
- Eine Session mit zwei lokalen Samples zeigt `RangePreview`.
- Eine gemischte lokale/wireless Session zeigt `MixedSourceBlocked`.
- Undo aktualisiert oder invalidiert Proposal sofort.
- Reset setzt Proposal auf `None`.

### AP-11f.3 Mathematische Hilfsfunktionen zentralisieren

- [ ] `ExposureValueMath` um `rangeStopsToLogDensity(float stops)` erweitern,
  falls LogD-Darstellung gebraucht wird.
- [ ] `ExposureValueMath` um `quantizeToHalfStopGrade(float grade)` oder eine
  allgemeine Quantisierungshilfe erweitern, falls nicht im Workflow bleiben soll.
- [ ] Keine neue `pow`, `powf`, `exp2f` oder `log2` in Workflows oder UI
  einfuehren.
- [ ] Nichtfinite Werte, Lux <= 0 und negative Range immer explizit abfangen.

Akzeptanz:

- `log2(10)`-Umrechnung ist zentral und kommentiert.
- Grade-Clamp 0.0..5.0 ist stabil.
- 0.5er Quantisierung ist idempotent.
- Keine neue EV-Mathematik entsteht in `LvglUi` oder `UiPresenter`.

### AP-11f.4 Experimentelle SG-Gradationspreview

- [ ] Historical-v0.3-Mapping nur hinter Warnflag `ExperimentalGradeMapping`
  verwenden oder zunaechst nur dokumentiert lassen.
- [ ] `rangeLogD = rangeStops / log2(10)` als reine Anzeige-/Zwischengroesse
  ausgeben.
- [ ] Falls ein `proposedGrade` berechnet wird, immer mit Herkunft/Warnflag
  markieren.
- [ ] `proposedGrade` auf 0.5er Raster quantisieren.
- [ ] Kein `softTarget_`/`hardTarget_` veraendern.

Akzeptanz:

- Gleiche Range erzeugt deterministisch gleichen Grade-Preview.
- Grade-Preview ist monoton: groessere gemessene Range darf nicht haertere
  Gradation erzwingen, wenn das gewaehlte Mapping weicheres Papier verlangt.
- Extremwerte werden auf 0.0 bzw. 5.0 geklemmt.
- UI zeigt klar `PREVIEW`, nicht `APPLIED`.

### AP-11f.5 Proposal-UI im Measurement-Panel

- [ ] Measurement-Seite um Proposal-Zeile erweitern.
- [ ] Warnflags kurz sichtbar machen: `MIXSRC`, `DEMO`, `NOANCH`, `EXPER`.
- [ ] Undo-Chip und Proposal-Chip getrennt halten.
- [ ] Footer-Hinweis: `CONFIRM VORSCHAU`, aber nur wenn `acceptAllowed` true ist.
- [ ] C6-Remote-Render erst nach lokaler Semantik nachziehen, damit das Terminal
  keine eigene Proposal-Logik bekommt.

Akzeptanz:

- Nutzer sieht, warum kein Vorschlag moeglich ist.
- Nutzer sieht, wenn ein Vorschlag nur experimentell ist.
- Die Messseite bleibt ohne Touch bedienbar.

### AP-11f.6 Accept-Pfad fuer SG-Preview

- [ ] Neuen Command-Port oder Workflow-API fuer `acceptMeasurementProposal()`
  entwerfen.
- [ ] Accept nur im `SplitgradePanel::Measurement`, nur bei Idle/Aborted und
  ohne aktive Exposure erlauben.
- [ ] Accept eines reinen Grade-Preview darf nur `grade_` und daraus abgeleitete
  Runtime-Targets neu setzen, nicht automatisch PaperProfile schreiben.
- [ ] Accept eines spaeteren Target-Preview muss `parametersDirty_` setzen.
- [ ] Start bleibt separater Tastendruck.

Akzeptanz:

- Proposal kann bestaetigt oder ignoriert werden.
- Nach Accept zeigt SG die neuen Werte, startet aber nicht automatisch.
- Bei Fault/Wait/Exposure wird Accept blockiert.
- PaperProfile bleibt unveraendert, solange kein eigener Persistenzschritt
  bestaetigt wird.

### AP-11f.7 Tests

- [ ] Host-/PIO-Test fuer `ExposureValueMath` ergaenzen: relative EV,
  Multiplier, LogD-Konvertierung, Grade-Quantisierung.
- [ ] Measurement-Domain-Test ergaenzen: Append, Undo, Reset, Range,
  Mixed-source-Block, nichtfinite Werte.
- [ ] Wireless-Duplikat-Test: `measurementSequence` darf nach Undo keine alten
  Samples erneut einziehen.
- [ ] Proposal-Test mit synthetischen Samples: insufficient, single-source,
  mixed-source, uncalibrated profile, experimental grade preview.
- [ ] SG-Akzeptanztest erweitern: Accept aendert keine Profile, keine
  Auto-Exposure, keine Grade-Roundtrip-Drift.

Akzeptanz:

- Tests laufen als enger Harness ohne Hardware.
- `teensy41` Build bleibt gruen.
- Hardware-Abnahme bleibt separat, weil echte Papier-/Sensorvalidierung nicht
  durch Hosttests ersetzt werden kann.

### AP-11f.8 Hardware-/Papierabnahme vor Target-Apply

- [ ] Eine reale Kalibriersession nach
  `docs/hardware/end-to-end-papierkalibrierprotokoll.md` durchfuehren.
- [ ] Mindestens einen Profilstatus `gemessen` oder gleichwertig im Code/Datenmodell
  sichtbar machen.
- [ ] TSL2561/TSL2591-Quellenabgleich ohne Negativ dokumentieren.
- [ ] 21-Stufen-Graukeil-Lauf fuer aktive SG-Familie dokumentieren.
- [ ] Erst danach absolute Zeit-/Dosisvorschlaege freigeben.

Akzeptanz:

- Jede verwendete Profilgroesse zeigt auf Rohdaten/Lauf-ID.
- Proposal kann seine Rechenbasis anzeigen.
- Demo-Profile koennen keine stillen autoritativen Proposals erzeugen.

## Mathematische Blocker vor vollautomatischem Proposal

Diese Punkte blockieren bewusst jeden direkten `Measure -> Apply -> Exposure`-
Automatismus:

1. K-Faktor-Einheit ist im aktiven Part2-Code noch nicht explizit getrennt von
   Runtime-Targets.
2. BW ist nur Shell und das BW-Modell ist offen.
3. TSL2561 und TSL2591 haben unterschiedliche Messgeometrien und Rollen.
4. Mixed-source Range ist aktuell mathematisch nicht vergleichbar.
5. Samples tragen noch keine fachliche Rolle wie Shadow/Highlight/Midtone.
6. Samples tragen noch keinen Kanalbezug wie G0/G5/White/CurrentSG.
7. ISO-R ist implementiert, aber noch nicht nass/hardwarekalibriert.
8. Defaultprofile sind teils Demo-artig und nicht durch reale Rohdaten belegt.

## Empfohlene Reihenfolge

1. Proposal-Statusmodell und Eligibility ohne Apply.
2. Mixed-source-Block und Profile-Warnungen.
3. Range-/LogD-Anzeige plus experimenteller SG-Grade-Preview.
4. Lokale UI-Preview im Measurement-Panel.
5. SG-Accept nur fuer Grade-Preview, ohne Profilpersistenz und ohne Exposure-Start.
6. Tests fuer Measurement-Domain und Proposal-Status.
7. Reale Papier-/Sensorabnahme.
8. Erst danach Time-/Dose-Target-Proposals und BW-Proposals.

## Audit-Fazit

Der aktuelle Code hat die richtige Architektur vorbereitet: Messung liegt in der
Measurement-Domain, EV-Grundlogik ist zentral, SG-Targets sind paper-profile-
getrieben und die neue Messseite liefert eine sichtbare Session-Range.

Die historische Mathematik liefert zwei wichtige Bausteine:

- `time = K / lux` als papierbezogener K-Faktor-Ansatz.
- Range-basierte Gradationsidee aus Auto-SG.

Beide duerfen aber nicht unveraendert portiert werden. Fuer Part2 muss zuerst
ein typisiertes Proposal-Modell entstehen, das Quelle, Einheit, Profilstatus,
Range, Warnungen und Accept-Zustand sichtbar macht. Alles andere wuerde die
bereits bereinigte Architektur wieder in genau die historische Zerklueftung
zurueckschieben.