# AP-11g Measurement-Logik und Mathematik: schonungsloser Audit

Stand: 2026-05-02

## Zweck

Dieses Dokument auditiert die aktuelle Measurement-Logik und Measurement-
Mathematik von `Dukatimer-Part2` nach denselben Leitplanken wie AP-11f:
Einheiten sichtbar machen, Rechenwege offenlegen, Invarianten formulieren,
Code gegen Dokumentation und Historie halten und Fehler, Ungenauigkeiten sowie
Landminen ohne Schoenfaerbung benennen.

Dieser Audit ist absichtlich read-only. Er implementiert keine Korrektur, weil
mehrere Befunde Schnittstellen- und Produktentscheidungen beruehren. Die
fachlich korrekte naechste Bewegung ist ein kleiner, testbarer Fix-/Status-
Slice, nicht ein breiter Umbau waehrend des Audits.

## Kurzfazit

Die aktuelle Measurement-Domain ist ein brauchbarer Anfang fuer relative
Session-Telemetrie. Sie ist noch kein belastbarer fotografischer
Messworkflow.

Hart gesagt:

- Der aktuelle Code kann Luxsamples sammeln, quellenbezogene relative EV-Werte
  bilden, ein 11-Bucket-Histogramm pflegen und Undo anbieten.
- Er kann noch nicht sicher sagen, ob eine Session fotografisch homogen ist.
- Er kann lokale und Wireless-Messwerte noch nicht mathematisch sauber
  vergleichen.
- Er kann keine absoluten Zonen, Dichten, Papierbereiche oder
  Belichtungsvorschlaege behaupten.
- Er zeigt an mehreren Stellen Nullwerte an, wo eigentlich "ungueltig" oder
  "nicht vorhanden" stehen muesste.
- Er besitzt echte Zustandslandminen bei History-Ueberlauf, Source-Mix,
  Wireless-Autoappend und UI-Telemetrie.

Der wichtigste positive Befund bleibt: Der fruehere versteckte `log2(lux)`-
beziehungsweise 1-Lux-Anker wurde in Part2 durch `log2(lux/referenceLux)`
ersetzt. Das ist die richtige Richtung. Aber der daraus entstehende
EV-Raum ist nur session- und quellenrelativ. Sobald diese Werte wie echte
Papierzonen, echte Schatten/Lichter oder cross-sensorische Wahrheit behandelt
werden, kippt das System fachlich.

## Gepruefter Pfad

### Aktive Part2-Dateien

- `src/teensy/SensorManager.h`
- `src/teensy/SensorManager.cpp`
- `src/teensy/SensorRuntimeStatus.h`
- `src/teensy/MeasurementQueryPort.h`
- `src/teensy/MeasurementRuntimeStatus.h`
- `src/teensy/MeasurementCommandPort.h`
- `src/teensy/MeasurementDomainService.h`
- `src/teensy/MeasurementDomainService.cpp`
- `src/teensy/ExposureValueMath.h`
- `src/teensy/ExposureValueMath.cpp`
- `src/teensy/SplitgradeWorkflow.cpp`
- `src/teensy/UiPresenter.cpp`
- `src/teensy/LvglUi.cpp`
- `src/teensy/main.cpp`
- `src/teensy/EspLinkRuntimeStatus.h`
- `src/teensy/EspServiceLink.cpp`
- `src/esp32/WirelessRemoteGateway.cpp`
- `src/esp32/TeensyLinkService.cpp`
- `lib/SharedProtocol/DukatimerProtocol.h`
- `../Wireless TSL2591/src/main.cpp`

### Dokumente und historische Referenzen

- `docs/software/history/dukatimer-part2-pflichtenheft.md`
- `docs/software/erledigt/dukatimer-part2-architekturzugriffspunkte-math-sensorik-und-workflows.md`
- `docs/software/offene_punkte.md`
- `docs/software/offene_entscheidungen.md`
- `docs/software/dukatimer-part2-ap11f-measurement-proposal-math-todo.md`
- `historischer Ursprung/sketch_jan18v_v2_240_Probestreifen_copy.ino`
- `../Dukatimer v0.3/src/Logic_Measurement.cpp`
- `../Dukatimer v0.3/src/Logic_Math.cpp`
- `../Dukatimer v0.3/tools/todo.txt`

## Anforderungen aus der Dokumentation

### PF-M05 Spotmessung und Histogramm

Das Pflichtenheft verlangt, dass Messpunkte erfasst, in ein Zonensystem
ueberfuehrt und als Histogramm gespeichert werden. Undo fuer den letzten
Messpunkt ist vorzusehen.

Aktueller Erfuellungsgrad:

- Erfassen: teilweise erfuellt.
- Zonensystem: nur session-relativ erfuellt, nicht absolut fotografisch.
- Histogramm: technisch vorhanden, aber bei History-Ueberlauf inkonsistent.
- Undo: fuer sichtbare History vorhanden, aber nicht vollstaendig gegen
  Quellenreferenzen und History-Ueberlauf gehaertet.

### PF-M06 Automatische Vorschlagsbildung

Aus Messpunkten sollen BW- und SG-Vorschlaege ableitbar sein. Vorschlaege
duerfen nicht ungeprueft aktiv werden.

Aktueller Erfuellungsgrad:

- Keine produktive Vorschlagsbildung in der Measurement-Domain.
- Keine Proposal-Eligibility.
- Keine Warnflags fuer Source-Mix, Kalibrierstatus oder Messrollen.
- Kein Accept-/Apply-Vertrag.
- AP-11f dokumentiert den sicheren Vorschlagsplan, aber der Code enthaelt ihn
  noch nicht.

### PF-S06 Messwertdarstellung in EV und Lux

Messwerte aus echten Messvorgaengen sollen Lux und, sofern sinnvoll, EV zeigen.
Die Umrechnung soll ueber zentrale Formatter-/Math-Schichten laufen.

Aktueller Erfuellungsgrad:

- `ExposureValueMath` zentralisiert die EV-Grundformel.
- `MeasurementValueFormatter` formatiert nur SG-Dosis-Telemetrie.
- Measurement-Page-Texte werden weiterhin direkt in `UiPresenter` formatiert.
- Ungueltige Werte werden an mehreren Stellen als `0.000 lux` oder `+0.00 EV`
  angezeigt. Das ist fachlich gefaehrlich.

### Architekturregel: MeasurementDomainService ist Besitzer

Die Architektur-Doku legt fest: `MeasurementDomainService` besitzt Referenzwahl,
Mittelung, Undo, Histogramm und Vorschlagslogik. `SensorManager` liefert nur
Samples und Status. `ExposureEngine` nimmt keine Messhistorie auf.

Aktueller Erfuellungsgrad:

- Besitzergrenze ist im Grundsatz gut getroffen.
- `SensorManager` entscheidet keine EV- oder Histogrammsemantik.
- `ExposureEngine` bleibt frei von Messhistorie.
- Die Measurement-Domain hat aber noch kein echtes Messworkflow-Modell:
  keine Session-Arming-Policy, keine Messrollen, keine Source-Komposition,
  keine Mittelung, keine Proposal-Ausgabe.

## Aktueller Datenfluss

### Lokaler TSL2561-Pfad

```text
TSL2561 raw channels
  -> SensorManager::pollLocalTsl2561()
  -> SensorManager::calculateLuxFromChannels()
  -> Tsl2561RuntimeStatus.lux
  -> MeasurementDomainService::observeLocalSensor()
  -> MeasurementLuxSample{source=LocalTsl2561, lux, ageMs, valid}
```

Eigenschaften:

- TSL2561 laeuft mit 101 ms Integration, 1x Gain und Fixed-Point-Luxformel.
- `calculateLuxFromChannels()` gibt einen ganzzahligen Luxwert als `float`
  zurueck.
- Sub-Lux-Information wird im lokalen Pfad nicht erhalten. Fuer die aktuelle
  Aufgabe des Sensors direkt unter den NeoPixeln ist das kein Defekt: Der
  TSL2561 ist hier Kopf-/Closed-Loop-Sensor zur Leistungsueberwachung, nicht
  Papier-Spotmeter.
- Ein rechnerisches `0.0 lux` kann im Sensorstatus gueltig sein, wird aber in
  der Measurement-Domain nicht als Session-Sample akzeptiert.
- `sampleFresh` wird durch die Measurement-Domain nicht direkt ausgewertet;
  relevant sind `initialized`, `health == Ok` und `sampleValidity == Valid`.

### Wireless-TSL2591-Pfad

```text
C6 TSL2591
  -> WirelessRemotePayload.activeLux
  -> ESP32-S3 WirelessRemoteGateway
  -> TeensyLinkService::setWirelessPeerState()
  -> WirelessSnapshotPayload.lastLuxMilliLux
  -> EspServiceLink::applyWirelessSnapshot()
  -> WirelessGatewayStatus.lastLux
  -> MeasurementDomainService::observeWirelessGateway()
  -> MeasurementLuxSample{source=WirelessGateway, lux, ageMs, sequence, valid}
```

Eigenschaften:

- C6 misst mit TSL2591, 600 ms Integration und Gain-Automatik.
- C6 pollt den Sensor alle 800 ms.
- Der Measure-Button erzwingt keine frische Sensorintegration; er benutzt den
  zuletzt publizierten Luxwert.
- Der ESP32-S3 quantisiert Wireless-Lux auf Milli-Lux.
- Diese Milli-Lux-Aufloesung ist fuer den Papier-/Low-Light-Pfad wesentlich und
  darf auf dem Weg C6 -> ESP32-S3 -> Teensy -> UI/Measurement nicht auf
  Integer-Lux zurueckfallen.
- `lastSeenAgeMs` beschreibt die Funk-/Peer-Frische, nicht sicher das Alter
  der optischen Sensorintegration.
- `measurementSequence` steigt auf ESP32-S3-Seite bei Measure-Button-Flanke,
  sofern ein gueltiger Wireless-Luxwert vorhanden ist.

### Session-Pfad

```text
MeasurementLuxSample
  -> appendSessionSample(sample, nowMs)
  -> establishSessionReference(sample.source)
  -> relativeEvStops = log2(sample.lux / reference.lux)
  -> zoneIndex = round(relativeEvStops) + 5, clamp 0..10
  -> histogramWeight = 20 oder Rest bis 240
  -> sessionUndoHistory_
  -> syncSessionStatusFromHistory()
```

Eigenschaften:

- Es gibt einen Session-Referenzlux pro Quelle.
- Die erste gueltige Messung einer Quelle wird Referenz dieser Quelle.
- Relative EVs unterschiedlicher Quellen teilen keinen gemeinsamen absoluten
  Nullpunkt.
- Das Histogramm ist 11 Buckets breit, Bucketmitte 5 bedeutet
  Quellenreferenz, nicht Zone V im fotografischen Sinn.

### UI-Pfad

```text
MeasurementRuntimeStatus
  -> SystemSnapshot.measurementStatus
  -> UiPresenter::getMeasurement...()
  -> LvglUi measurement page
```

Eigenschaften:

- Presenter formatiert Measurement-Lux und EV direkt per `snprintf`.
- Ungueltige lokale und Wireless-Quellen werden als `0.000 lux` dargestellt.
- Ungueltige Referenz oder Range wird als Nullwert dargestellt.
- Die Anzeige nutzt Begriffe wie `SH`, `HI`, `SPAN`, ohne den relativen
  Quellenkontext sichtbar zu machen.

## Groessen und Einheiten

| Groesse | Code-Ort | Einheit | Ist-Semantik | Risiko |
| --- | --- | --- | --- | --- |
| `Tsl2561RuntimeStatus::lux` | `SensorRuntimeStatus.h` | lux, lokal, Integer-Lux als float | Kopf-/Closed-Loop-naher Sensorwert direkt unter den NeoPixeln | Integer-Lux fuer Leistungsueberwachung ausreichend; Risiko nur bei Missbrauch als Papier-/Low-Light-Spotmeter |
| `WirelessRemotePayload::activeLux` | SharedProtocol/C6 | lux, float | letzter C6-TSL2591-Livewert | nicht zwingend frisch beim Measure-Button |
| `WirelessSnapshotPayload::lastLuxMilliLux` | SharedProtocol | milli-lux, uint32 | quantisierte Funkuebergabe fuer Papier-/Low-Light-Messung | Milli-Lux-Aufloesung muss erhalten bleiben; Rundung, Clamp und ABI-Drift kritisch |
| `MeasurementLuxSample::lux` | `MeasurementQueryPort.h` | lux | normalisierter Samplewert je Quelle | Quellen nicht kalibriert vergleichbar |
| `MeasurementLuxSample::ageMs` | MeasurementDomain | ms | lokaler Sample- oder Peer-Age | kein echter Integrationszeitpunkt |
| `MeasurementSessionSample::referenceLux` | MeasurementDomain | lux | erster gueltiger Samplewert pro Quelle | kann stale werden, wenn Quelle komplett undone wurde |
| `relativeEvStops` | MeasurementDomain/ExposureValueMath | Stops | `log2(lux/referenceLux)` | nur relativ, nicht Zone/Dichte/Papier |
| `zoneIndex` | MeasurementDomain | Index 0..10 | gerundeter relativer EV-Bucket um Mitte 5 | kein absolutes Zonensystem |
| `zoneHistogram[]` | MeasurementStatus | Balkenwert 0..240 | Anzeigegewicht, +20 pro Sample | bei History-Ueberlauf inkonsistent |
| `sampleCount` | MeasurementStatus | Zaehler | teils aktuelle Samples, teils Lifetime | bricht bei History-Ueberlauf |
| `capturedSampleCount` | MeasurementStatus | Zaehler | Lifetime-Captures ohne Undo-Reduktion | Name/Anzeige kann verwirren |
| `shadowSample` | MeasurementStatus | Sample | Minimum im relativen EV-Raum | kein garantierter Print-Schatten |
| `highlightSample` | MeasurementStatus | Sample | Maximum im relativen EV-Raum | kein garantierter Print-Highlight |

## Soll-Invarianten

Diese Invarianten sollte eine belastbare Measurement-Domain erfuellen:

1. Ein Histogramm darf nur Samples zaehlen, die in der aktuellen Session-History
   oder in einer klar dokumentierten Lifetime-History enthalten sind.
2. `sampleCount` muss eindeutig entweder aktuelle Sessiongroesse oder
   Lifetime-Zaehler sein, nicht beides.
3. Eine Range aus gemischten Quellen darf nicht ohne gemeinsamen Kalibrieranker
   als fotografische Range gelten.
4. `rangeValid` darf fuer Vorschlagslogik nicht mit "genug Messdaten" verwechselt
   werden.
5. Ein Undo muss alle abgeleiteten Statusfelder konsistent machen:
   Histogramm, Range, Referenz, UndoDepth, Counts, aktive Referenz.
6. Eine Wireless-Measure-Flanke darf nicht still eine Session veraendern, wenn
   der Workflow nicht im passenden Messkontext ist.
7. Angezeigte Nullwerte duerfen nicht als Ersatz fuer "ungueltig" oder "nicht
   vorhanden" dienen.
8. Lokale Kopfmessung und Wireless-Papiermessung muessen als unterschiedliche
   Messrollen sichtbar bleiben.
9. EV-Berechnung bleibt in `ExposureValueMath`; Formatierung und
   Validitaetsdarstellung gehoeren in einen zentralen Formatter/Presenter-
   Vertrag.
10. Proposal- oder Papierlogik darf nur auf Samples arbeiten, deren Quelle,
    Rolle, Alter und Kalibrierstatus bekannt sind.

## Befunde: echte Fehler und harte Landminen

### Befund 1: Histogramm wird bei History-Ueberlauf nicht bereinigt

Schweregrad: hoch.

`MeasurementDomainService::appendSessionSample()` verschiebt bei voller
`sessionUndoHistory_` die History um ein Element nach links. Dabei wird der
Histogrammbeitrag des herausfallenden Samples nicht abgezogen.

Folge:

- `zoneHistogram[]` enthaelt dann Samples, die nicht mehr in
  `sessionUndoHistory_` vorhanden sind.
- `syncSessionStatusFromHistory()` berechnet Range und Recent-Samples aus einer
  anderen Datenbasis als das Histogramm.
- Undo kann den verlorenen Altbeitrag nicht mehr entfernen.
- Bei langen Messsessions driftet das Histogramm in Richtung Vergangenheit,
  waehrend Range/Undo nur die letzten 128 Samples sehen.

Das ist kein kosmetischer Fehler. Das Histogramm ist eine fachliche Grundlage
fuer Zonen- und spaetere Vorschlagslogik. Sobald es mehr als Anzeige ist, ist
dieser Pfad falsch.

Minimaler Fix:

- Vor dem Shift den herausfallenden Sample merken.
- Dessen `histogramWeight` per `applyHistogramDelta(..., -weight)` entfernen.
- Tests fuer Bucket-Saturation, Overflow und Undo danach ergaenzen.

### Befund 2: `sampleCount` ist bei Overflow semantisch kaputt

Schweregrad: hoch.

`state_.session.sampleCount` wird bei jedem Append erhoeht und bei Undo
reduziert. Wenn die History voll ist, wird aber ein altes Sample aus der
History entfernt, ohne `sampleCount` zu reduzieren.

Folge:

- Vor Overflow wirkt `sampleCount` wie aktuelle Sessiongroesse.
- Nach Overflow wirkt `sampleCount` wie Lifetime minus Undos.
- `undoDepth` bleibt auf maximal 128, Range nutzt maximal 128, Histogramm nutzt
  aktuell sogar mehr als 128 wegen Befund 1.
- Die UI-Zeile `MS sampleCount/capturedSampleCount` wird uneindeutig.

Minimaler Fix:

- Begriffe trennen: `currentSampleCount`, `lifetimeCapturedSampleCount`,
  optional `droppedSampleCount`.
- Oder `sampleCount` strikt auf `sessionUndoHistoryCount_` setzen und
  `capturedSampleCount` als Lifetime behalten.

### Befund 3: Gemischte Quellen werden in einer Range zusammengeworfen

Schweregrad: hoch.

Lokale TSL2561-Samples und Wireless-TSL2591-Samples erhalten je eigene
Referenzen. Trotzdem sucht `syncSessionStatusFromHistory()` globales Minimum
und Maximum ueber alle Samples.

Folge:

- `relativeEvStops = 0.0` eines lokalen Kopf-Sensors und `relativeEvStops = 0.0`
  eines Wireless-Papier-Sensors bedeuten nicht dieselbe fotografische Helligkeit.
- `relativeEvSpanStops` einer gemischten Session kann mathematisch leer oder
  irrefuehrend sein.
- Die UI zeigt keinen Source-Mix-Warnstatus.
- AP-11f-Proposal-Logik darf auf dieser Range nicht arbeiten, solange keine
  Composition-/Source-Policy existiert.

Minimaler Fix:

- Session-Komposition in `MeasurementSessionStatus` aufnehmen:
  `sourceMask`, `sourceCount`, `commonSource`, `mixedSources`.
- `rangeValid` oder neue `proposalRangeValid` bei gemischten Quellen sperren,
  bis ein belegter Cross-Calibration-Anchor existiert.

### Befund 4: Wireless-Messbutton kann Session ausserhalb des Messpanels veraendern

Schweregrad: hoch.

Wireless-Samples werden nicht durch `SplitgradeWorkflow` gecaptured, sondern in
`MeasurementDomainService::observeWirelessGateway()` automatisch appendet,
sobald `measurementSequence` neuer ist.

Der Workflow verhindert zwar auf dem Measurement-Panel lokale Duplikate fuer
`WirelessMeasureButton`, aber die Domain selbst weiss nichts vom aktiven Panel,
Modus oder Messkontext.

Folge:

- Ein C6-Measure-Button kann die Session veraendern, auch wenn der Nutzer nicht
  bewusst in einem Messworkflow ist.
- Setup, Target-Panel oder spaetere Modi koennen eine Session nebenbei
  kontaminieren, sofern ein neuer Wireless-Measure-Sequenzwert ankommt.
- Das widerspricht dem Produktanspruch eines bestaetigten, klaren Messablaufs.

Minimaler Fix:

- Wireless-Autoappend an eine explizite Measurement-Armed-Policy binden.
- Alternativ `MeasurementCommandPort::captureWirelessSample(sequence)` einfuehren
  und den Workflow zum einzigen Capture-Ausloeser machen.
- Mindestens einen Status `captureRejectedReason` oder Diagnosezaehler fuehren.

### Befund 5: SG-Telemetrie kann Wireless-Lux statt Head-Lux anzeigen

Schweregrad: hoch.

`MeasurementDomainService::chooseActiveSample()` priorisiert Wireless vor lokal.
`UiPresenter::getSgExposureMain()` nutzt ausserhalb der Measurement-Page fuer
die SG-Dosis-Telemetrie zuerst `measurementStatus.activeLux`, sonst
`exposureState.measuredLux`.

Folge:

- Wenn das Wireless-Terminal online ist, kann die normale SG-Anzeige `LUX` vom
  C6-Papier-/Spotpfad zeigen, obwohl die ExposureEngine im Dose-Modus lokal mit
  dem TSL2561 am Kopfpfad integriert.
- Die Anzeige kann damit den Eindruck erzeugen, die laufende Dosisregelung
  arbeite mit dem Wireless-Luxwert.
- Die Engine bleibt zwar technisch korrekt getrennt, aber die UI-Semantik ist
  brandgefaehrlich.

Minimaler Fix:

- Exposure-/Dose-Telemetrie immer aus `ExposureRuntimeState` beziehungsweise
  lokalem Sensorpfad formatieren.
- Measurement-Active-Lux nur auf Measurement-/Spotseiten verwenden.
- Presenter-API in `MeasurementValueFormatter` oder klar getrennte Formatter
  verschieben.

### Befund 6: Referenzen koennen fuer eine Quelle stale bleiben

Schweregrad: mittel bis hoch.

Session-Referenzen werden nur komplett geloescht, wenn die gesamte History leer
ist. Wird eine Quelle vollstaendig undone, waehrend Samples einer anderen Quelle
in der Session bleiben, bleibt die Referenz der entfernten Quelle erhalten.

Beispiel:

```text
local1 -> local reference gesetzt
wire1  -> wireless reference gesetzt
wire2
undo wire2
undo wire1
Session enthaelt nur noch local1, wireless reference bleibt intern erhalten
naechster wireless sample nutzt alte wire1-Referenz
```

Folge:

- Eine spaetere neue Wireless-Teilserie kann an einer nicht mehr sichtbaren
  Alt-Referenz haengen.
- `activeReference` kann eine Quelle referenzieren, deren alte Samples nicht
  mehr in der sichtbaren Session vorkommen.

Minimaler Fix:

- Nach jedem Undo und nach Overflow Referenzen aus aktueller History neu
  rekonstruieren.
- Oder pro Quelle zaehlen und Referenz loeschen, wenn die letzte Probe dieser
  Quelle entfernt wurde.

### Befund 7: `rangeValid` ist schon bei einem einzelnen Sample true

Schweregrad: mittel.

`syncSessionStatusFromHistory()` setzt `rangeValid = true`, sobald mindestens
ein History-Sample vorhanden ist. Bei einem Sample ist die Span automatisch
`0.0`.

Folge:

- Fuer Anzeige einer mathematisch definierten Min/Max-Struktur ist das
  vertretbar.
- Fuer Messworkflow und Proposal ist es gefaehrlich, weil ein einzelner Punkt
  keine Kontrastspanne beschreibt.

Minimaler Fix:

- `rangeValid` enger definieren oder zusaetzliche Felder einfuehren:
  `hasExtrema`, `hasContrastRange`, `proposalEligible`.
- Proposal-Logik muss mindestens zwei verwertbare Samples derselben Quelle
  verlangen.

### Befund 8: Praezisionsbewertung muss nach Sensorrolle getrennt werden

Schweregrad: korrigierter Audit-Befund.

Der urspruengliche Audit war hier zu pauschal. Der lokale TSL2561 und der
Wireless-TSL2591 haben unterschiedliche Aufgaben und duerfen nicht mit derselben
Praezisionsforderung bewertet werden.

`SensorManager::calculateLuxFromChannels()` rundet die Fixed-Point-Berechnung
auf einen ganzzahligen Luxwert und gibt diesen als `float` zurueck.

Bewertung fuer den TSL2561 am Kopf:

- Der TSL2561 sitzt direkt unter den NeoPixeln und dient aktuell der
  Closed-Loop-Regelung beziehungsweise Leistungsueberwachung des Kopfpfads.
- Fuer diese Aufgabe sind Integer-Lux-Werte ausreichend. Sub-Lux-Praezision
  waere hier keine fachlich notwendige Verbesserung.
- Der Integer-Lux-Pfad ist deshalb fuer die Kopfregelung kein Fehler.

Bewertung fuer den TSL2591 im C6-Terminal:

- Der TSL2591 sitzt am Papier-/Spotmesspfad und ist gerade fuer Low-Light- und
  Transmissionsmessungen relevant.
- Hier ist die Milli-Lux-Aufloesung des SharedProtocol wesentlich.
- C6-Float-Lux und `lastLuxMilliLux` muessen end-to-end erhalten bleiben; eine
  Rueckstufung auf Integer-Lux waere fuer diesen Pfad fachlich falsch.

Tatsaechliches Risiko:

- Der lokale Integer-Lux-Wert darf nicht als Papier-/Low-Light- oder
  Densitometerwert missverstanden werden.
- Formatter und UI muessen die Quellenpraezision kennen: TSL2561-Kopflux darf
  grob angezeigt werden, TSL2591-Papierlux muss Milli-Lux erhalten.
- `MeasurementDomainService` darf spaetere Proposal- oder Dichtepfade nicht auf
  Basis eines scheinbar praezisen lokalen TSL2561-Werts freigeben.

Minimaler Fix:

- Audit-TODO korrigieren: keine Sub-Lux-Forderung fuer den TSL2561-Kopfpfad.
- Quellenrolle und Quellenpraezision im Measurement-/Formatter-Vertrag sichtbar
  machen.
- Sicherstellen, dass der TSL2591-Papierpfad seine Milli-Lux-Aufloesung bis zur
  Anzeige und spaeteren Proposal-/Dichtebewertung behaelt.

### Befund 9: Kein Dark-/Offset-Abzug, keine Mittelung, keine Ausreisserlogik

Schweregrad: mittel bis hoch.

Die historische v0.3-Messlogik subtrahierte `probeDarkLux` vor der EV-
Berechnung. Sie war insgesamt nicht sauber genug, aber dieser Punkt war
fachlich richtig: Gerade bei hohen Dichten oder kleinen Luxwerten frisst der
Dunkel-/Offsetanteil die Logarithmik.

Part2 macht aktuell:

```text
relativeEv = log2(rawLux / referenceLux)
```

ohne:

- Dark-/Offsetkorrektur
- Mittelwertbildung
- Messzeitfenster
- Varianz-/Stabilitaetspruefung
- Ausreisserverwerfung
- Quellenabhhaengige Mindesthelligkeit

Folge:

- Ein einzelner verrauschter erster Sample setzt die Referenz.
- Bei kleinen Luxwerten kann ein Offsetfehler mehrere Stops erzeugen.
- Multi-Spot bedeutet aktuell nur mehrere Klicks, nicht statistisch belastbare
  Messung.

Minimaler Fix:

- Measurement-Sample-Rohdaten um Qualitaets- und Korrekturfelder erweitern.
- Erst fuer eine konkrete Quelle Dark-/Reference-Protokoll definieren.
- Vor Densitometer-Claims zwingend Offset-/Dark-Workflow einfuehren.

### Befund 10: Histogramm-Buckets zerstoeren kontinuierliche Information

Schweregrad: mittel.

`zoneIndexFromRelativeEv()` nutzt `std::lround(relativeEvStops)` und clamped auf
0..10.

Folge:

- `+0.49` und `-0.49` landen beide im Referenzbucket.
- `+5.4`, `+8.0` und `+20.0` landen alle im oberen Randbucket.
- Out-of-range-Werte sind nicht mehr als solche sichtbar.
- Fuer Anzeige okay, fuer Mathematik unzureichend.

Minimaler Fix:

- Kontinuierliche `relativeEvStops` als Primaerwert behalten.
- Histogramm nur als Anzeige oder als separate, klar quantisierte Statistik
  behandeln.
- Underflow-/Overflow-Zaehler ergaenzen.

### Befund 11: `shadowSample` und `highlightSample` sind keine fotografischen Rollen

Schweregrad: mittel.

Aktuell gilt:

```text
shadowSample = niedrigstes relativeEvStops
highlightSample = hoechstes relativeEvStops
```

Das sind numerische Extrema. Es sind keine vom Nutzer markierten Schatten- oder
Lichterpunkte und keine sicher abgeleiteten Printzonen.

Bei Negativ-/Transmissionsmessung ist die fotografische Benennung besonders
heikel: Je nach Messaufbau bedeutet wenig Lux eher dichter Negativbereich und
damit typischerweise Print-Highlight, nicht automatisch Schatten.

Minimaler Fix:

- Interne Namen mittelfristig auf `lowEvSample` / `highEvSample` oder
  `minSample` / `maxSample` neutralisieren.
- Falls UI `SH/HI` zeigen soll, muss eine explizite Rollen-/Messkonvention
  existieren.

### Befund 12: `capturedAtMs` ist nicht der echte Sensorzeitpunkt

Schweregrad: mittel.

`MeasurementSessionSample::capturedAtMs` bekommt `nowMs` aus der Teensy-Loop.
Fuer lokale Samples ist der optische Samplezeitpunkt aber `nowMs - sampleAgeMs`.
Fuer Wireless-Samples ist `lastSeenAgeMs` nur Funk-/Peer-Age und nicht sicher
die TSL2591-Integrationszeit.

Folge:

- Eine spaetere Qualitaetsbewertung kann nicht unterscheiden, ob ein Sample im
  Moment des Tastendrucks frisch oder bereits alt war.
- C6-Measure wirkt wie ein aktueller Messpunkt, nutzt aber den letzten
  gepollten TSL2591-Wert.

Minimaler Fix:

- `MeasurementSessionSample` um `sampleAgeAtCaptureMs` und, wenn moeglich,
  `sourceSampleTimestampMs` erweitern.
- C6-Protokoll spaeter um Sensor-Sample-Age oder Sensor-Sequence ergaenzen.

### Befund 13: `0.000 lux` kaschiert ungueltige Werte

Schweregrad: mittel, UI-sicherheitsnah.

`UiPresenter` zeigt in mehreren Measurement-Funktionen bei ungueltigen Samples
oder fehlender Referenz `0.000 lux` und `+0.00` an.

Beispiele:

- `getMeasurementSources()` zeigt invalid local/wire als `0.000 lux`.
- `getMeasurementReference()` zeigt fehlende Referenz als `REF 0.000 lux`.
- `getMeasurementRange()` zeigt fehlende Range als `SH +0.00 HI +0.00 SPAN 0.00`.
- `getMeasurementStatusLeft()` zeigt bei leerer Session `Z0` aus default
  `latestSample.zoneIndex`.

Folge:

- Nutzer kann "kein Wert" mit echtem Null-Lux oder echter Nullspanne
  verwechseln.
- Fuer Dunkelkammerbedienung ist das nicht nur unschoen, sondern falsch.

Minimaler Fix:

- Formatter mit explizitem Invalid-Text einfuehren: `--`, `NO REF`, `NO RNG`,
  `NO SAMPLE`.
- Null nur anzeigen, wenn Null wirklich ein gueltiger physikalischer Messwert
  ist und die Funktion ihn bewusst anzeigen will.

### Befund 14: Quelle und Rolle sind nicht Teil des Sessionvertrags

Schweregrad: mittel bis hoch.

`MeasurementSessionSample` kennt Quelle, Lux, Sequenz, Referenz, EV, Zone. Es
kennt nicht:

- Messrolle: local closed-loop, paper spot, transmission, calibration, focus
- Kanalbezug: white, soft, hard, current SG mix
- Nutzerrolle: shadow, highlight, midtone, reference, base, dark
- Kalibrieranker: none, first sample, paper white, no-negative reference,
  dark offset, external reference
- Qualitaet: stale, saturated, low-signal, averaged, raw single sample

Folge:

- Der Code kann aktuell nur eine mathematische relative Liste fuehren, aber
  keine fotografische Session beschreiben.
- Proposal, Densitometer, Filmtest und Papierkalibrierung wuerden ohne diese
  Felder erneut implizite Annahmen einbauen.

Minimaler Fix:

- Einen `MeasurementSessionMode` und `MeasurementSampleRole` einfuehren, aber
  erst entlang eines konkreten Workflows.
- Fuer AP-11f-Proposal mindestens Source-Komposition und Warnflags einfuehren.

### Befund 15: Keine automatisierte MeasurementDomain-Testabdeckung

Schweregrad: mittel.

Aktuell existiert ein AP-07-Harness fuer Splitgrade-Math. Fuer
`MeasurementDomainService` ist kein vergleichbarer Testpfad sichtbar.

Folge:

- Die kritischen Invarianten von Histogramm, Undo, Source-Mix, Overflow und
  Range werden nur durch Builds, nicht durch Tests geschuetzt.
- Gerade die oben genannten Overflow- und Referenzprobleme sind typische
  Regressionen, die ein kleiner Host-/PIO-Test sofort fangen wuerde.

Minimaler Fix:

- `test/ap11_measurement_domain` oder aehnlichen Harness anlegen.
- Tests fuer Append, Undo, Overflow, Saturated-Bucket, Source-Mix,
  Reference-Rebuild, Range-Eligibility.

## Mathematische Bewertung

### Was korrekt ist

Die Formel

```text
relativeEvStops = log2(lux / referenceLux)
```

ist fuer relative Lichtverhaeltnisse korrekt, sofern beide Werte positive,
endliche Luxwerte derselben kompatiblen Messquelle und Messgeometrie sind.

Die Ableitung

```text
factor = 2 ^ evDeltaStops
```

ist fuer relative Belichtungs- oder Dosisfaktoren korrekt, sofern `evDeltaStops`
wirklich eine fotografische EV-Differenz zwischen kompatiblen Groessen ist.

Die Entscheidung, keine versteckte absolute `1 lux -> Zone`-Politik mehr zu
verwenden, ist richtig.

### Was mathematisch nicht belegt ist

Nicht belegt ist aktuell:

```text
lux -> absolute Zone
relativeEvStops -> Papierdichte
relativeEvSpanStops -> SG-Gradation
TSL2561 lux == TSL2591 lux fuer dieselbe fotografische Bedeutung
Wireless lastSeenAgeMs == Sensorintegrationsalter
first sample == stabile Referenz
single sample span 0 == gueltige Range fuer Vorschlaege
```

### Wo Rundung und Quantisierung kritisch sind

- Lokaler TSL2561: Integer-Lux als Float ist fuer Kopf-/Closed-Loop-
  Leistungsueberwachung ausreichend, aber nicht als Papier-/Low-Light-
  Spotmesswert zu verwenden.
- Wireless-TSL2591-Uebergabe: Milli-Lux als uint32 ist fuer Papier-/Low-Light-
  Messung essenziell und muss erhalten bleiben.
- Histogramm: Rundung auf ganze EV-Buckets.
- UI: Praezision muss quellenabhaengig sein; Kopf-TSL2561 nicht mit falscher
  Milli-Lux-Praezision, Papier-TSL2591 nicht auf Integer-Lux degradieren.
- `MeasurementRuntimeStatus::operator==`: exakter Floatvergleich; eher
  Render-/Status-Churn-Risiko als fotografischer Fehler.

### Wo Einheiten vermischt werden koennen

- SG-Dose-Telemetrie kann Wireless-Lux anzeigen, obwohl Exposure-Dose lokal
  integriert.
- `capturedAtMs` klingt wie Messzeit, ist aber Loop-Capturezeit.
- `lastSeenAgeMs` klingt nach Samplealter, ist aber Gateway-/Peer-Age.
- `zoneIndex` klingt nach Zone, ist aber relativer Bucketindex.
- `shadowSample`/`highlightSample` klingen nach Printrollen, sind aber EV-Extrema.

## Historischer Vergleich

### Historischer Ursprung v2.240

Der Ursprung hatte eine einfache, fachlich klare K-Faktor-Idee:

```text
K = avgLux * time
time = K / lux
```

Das ist nicht automatisch vollstaendig, aber die Einheiten sind sauberer als
viele spaetere Auto-Heuristiken: `K` ist lux-sekundenartig, `lux` ist gemessen,
`time` ist Sekunden. Der Kalibrier-Wizard mittelte Luxwerte und leitete
`Khard`, `Ksoft`, `Kbw` aus realem Messlauf und Zeit ab.

Part2 hat die Datenstruktur fuer K-/Paperprofile, aber MeasurementDomain nutzt
diese Kalibrieranker noch nicht.

### v0.3 Measurement

v0.3 hatte mehrere relevante Motive:

- `probeDarkLux` wurde vor EV-Berechnung abgezogen.
- G0/G5-Messung war als spektrale Sequenz angelegt.
- `spectral_ratio = corrG0 / corrG5` wurde gebildet.
- Messpunkte landeten in einem 11er Histogramm mit 20er Gewichtung.

Aber v0.3 war kein sauberer Port-Kandidat:

- `log2(corrG0)` hatte wieder einen versteckten absoluten Bezug.
- Schatten/Lichter waren UI-seitig getrennt, mathematisch aber nicht sauber als
  getrennte Sessionrollen gespeichert.
- Auto-SG hing an schwachen beziehungsweise toten Ankern.
- Der alte Audit forderte bereits ein zentrales `MeteringSessionResult`.

Part2 ist architektonisch sauberer, hat aber zentrale Fachfelder aus v0.3 noch
nicht wieder als robuste Datenvertraege aufgebaut: Dark-Offset, Rollen,
spektrale Kanaltrennung, Vorschlagsresultat.

### v0.9

v0.9 war strukturell besser getrennt, aber nicht direkt auf Part2 portierbar.
Fuer diesen Audit ist vor allem die Richtung relevant: Messung, PaperManager,
SystemContext und ExposureEngine sollten getrennte Dienste bleiben. Part2 folgt
dieser Richtung, aber die Measurement-Domain ist noch zu duenn.

## Priorisierte Korrektur-TODOs

### P0: Vor jeder Proposal-Implementierung blockieren

1. Histogramm-Overflow korrigieren.
   Beim Verwerfen des aeltesten History-Samples dessen Histogrammbeitrag
   entfernen.

2. `sampleCount`-Semantik reparieren.
   Aktuelle Sessiongroesse und Lifetime-Captures trennen.

3. Source-Komposition exportieren.
   Mindestens `sourceMask`, `sourceCount`, `mixedSources`, `commonSource`.

4. Gemischte Quellen fuer Proposal sperren.
   Solange kein Cross-Source-Kalibrieranker existiert, darf keine gemeinsame
   fotografische Range entstehen.

5. SG-Dose-Telemetrie vom Measurement-Active-Lux entkoppeln.
   Exposure-Anzeige darf Wireless-Paper-Lux nicht als Head-/Dose-Lux zeigen.

### P1: Measurement-Vertrag haerten

1. Referenzen nach Undo/Overflow rekonstruieren oder pro Quelle sauber
   invalidieren.

2. `rangeValid` in technische Extrema und fachliche Nutzbarkeit trennen.

3. Wireless-Capture an Measurement-Arming binden.
   Keine stillen Session-Aenderungen aus fremdem UI-Kontext.

4. `MeasurementSessionSample` um Age-/Qualitaetsfelder erweitern:
   `sampleAgeAtCaptureMs`, `sourceFreshness`, optional `sourceSampleSequence`.

5. UI-Invaliddarstellung korrigieren.
    `--`, `NO REF`, `NO RNG`, `NO SAMPLE` statt Nullwerte.

### P2: Mathematische Basis ausbauen

1. Measurement-Formatter ausbauen.
    Lux/EV/Range/Source/Validity zentral formatieren, nicht pro Presenter-
    Funktion neu.

2. Quellenpraezision ehrlich machen.
  TSL2561-Kopflux darf Integer-Lux bleiben; TSL2591-Papierlux muss
  Milli-Lux-Aufloesung behalten. Formatter und Measurement-Vertrag muessen
  diese Rollen unterscheiden.

3. Dark-/Offset-Modell pro Quelle definieren.
    Nicht pauschal implementieren; erst Messprotokoll und Quelle festlegen.

4. Rollenmodell einfuehren.
    Reference, shadow, highlight, midtone, dark, no-negative reference,
    calibration sample.

5. C6-Samplealter ins Protokoll bringen.
    `activeLuxAgeMs` oder eigene Sensor-Sample-Sequence; `lastSeenAgeMs` reicht
    nicht fuer fotografische Qualitaet.

### P3: Test- und Abnahmebasis

Stand 2026-05-02 nach Umsetzung des ersten P3-Slices:

- P3.1 ist als hostnaher Harness in `test/ap11_measurement_domain/test_main.cpp`
  plus Runner `tools/run_ap11_measurement_harness.py` angelegt.
- P3.4 ist als harter SharedProtocol-Sync-Guard in
  `tools/check_dukatimer_protocol_sync.py` umgesetzt und vor die Part2- sowie
  Wireless-TSL2591-Builds gehängt.
- P3.2, P3.3 und P3.5 sind in
  `docs/software/dukatimer-part2-ap11g-measurement-validation-protocol.md`
  konkretisiert.

1. MeasurementDomain-Harness bauen.
    Append, Undo, Overflow, Mixed-Source, Range, Saturation, Reference-Rebuild.

2. Cross-Source-Hardwareprotokoll definieren.
    Kein Negativ, gleiche optische Situation, TSL2561 Kopfpfad gegen TSL2591
    Papierpfad, Delta sichtbar dokumentieren.

3. Densitometer erst nach Referenz-/Dark-/Geometrieentscheidung.
    Vorher keine LogD-Claims.

4. C6/SharedProtocol-Sync pruefen.
    Lux-Payload-ABI und Header-Gleichheit automatisiert absichern.

5. Measurement-UI hardware-nah pruefen.
    Messwert invalid, stale, mixed source, undo, overflow, C6 offline,
    C6 button ohne valid lux.

## Empfohlener naechster enger Slice

Der naechste Code-Slice sollte nicht Proposal sein, sondern ein kleiner
Measurement-Domain-Hygiene-Slice:

1. `MeasurementSessionStatus` um Source-Komposition und Dropped-Count erweitern.
2. History-Overflow-Histogramm korrigieren.
3. `sampleCount` klarziehen.
4. `rangeUsableForProposal` noch nicht als Proposal, sondern als reine
   Eligibility-Vorstufe einfuehren.
5. Einen kleinen Harness fuer die vier Kernfaelle bauen:
   single source, mixed source, overflow, undo.

Erst danach sollte AP-11f-Proposal-Code entstehen. Sonst baut die Proposal-
Mathematik auf einem Statusmodell, das bei langen Sessions und gemischten
Quellen nachweislich luegt.

## Validierung dieses Audits

Durchgefuehrt:

- Aktueller Teensy-Codepfad gelesen: Sensor, Measurement, Workflow, Presenter,
  LVGL, Snapshot.
- ESP32-S3- und C6-Wireless-Luxpfad gelesen.
- SharedProtocol-Payloads geprueft.
- Pflichtenheft, Architekturregeln, offene Punkte und offene Entscheidungen
  gegengehalten.
- Historischer Ursprung, v0.3 Measurement/Math und v0.3 Audit-TODO gegen den
  aktuellen Part2-Stand verglichen.

Nicht durchgefuehrt:

- Kein Build, weil dieser Schritt Doku-only ist.
- Keine Hardwaremessung.
- Kein numerischer Testlauf, weil noch kein MeasurementDomain-Harness existiert.

Audit-Fazit:

Die aktuelle Measurement-Logik ist gut genug, um auf dem Display relative
Messpunkte sichtbar zu machen. Sie ist nicht gut genug, um ohne weitere
Hygiene Vorschlaege, Zonen, Dichten oder cross-sensorische Entscheidungen zu
tragen. Die groessten sofortigen Risiken sind nicht die EV-Formel selbst,
sondern die Statussemantik drumherum: History-Ueberlauf, Source-Mix,
Wireless-Autoappend, falsche UI-Nullwerte und der kontextlose Active-Lux-Begriff.
