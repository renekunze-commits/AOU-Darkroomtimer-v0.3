# Audit: Splitgrade-Mathematik, NeoPixel-Spektrum und Schwarzschildverhalten

Datum: 2026-05-06  
Scope: Dukatimer-Part2, Teensy-Laufzeitpfad fuer Splitgrade und BW-Multigrade

## Kurzfazit

Die aktuelle Splitgrade-Integration ist architektonisch sauber getrennt: Papierprofil und Gradationsmathematik liegen oberhalb der Hardware, die `ExposureEngine` fuehrt nur Zeit oder lx*s aus, und `NeoPixelHead` rendert nur den bereits entschiedenen RGB-Wert. Mathematisch ist der Pfad fuer einen ersten produktiven SG-Slice brauchbar, aber noch kein voll spektral kalibriertes fotografisches Modell.

Die unterschiedliche spektrale Empfindlichkeit von Multigrade-Papier wird derzeit nur indirekt und unvollstaendig beruecksichtigt:

- SG-Sequenzbetrieb: Soft und Hard laufen getrennt als reines Gruen bzw. reines Blau. Die Profil-LUT bestimmt die Aufteilung des Zielwerts. Das ist konzeptionell richtig, solange die LUT end-to-end fuer genau diesen Kopf, dieses Papier, diesen Sensorpfad und diese Zielwert-Einheit kalibriert wurde.
- BW-Multigrade-Simultanbetrieb: `softMix` und `hardMix` werden direkt in rohe NeoPixel-Gruen-/Blauwerte umgesetzt. Das erhaelt das Verhaeltnis, aber nicht zwingend die fotografisch wirksame Dosis oder Helligkeit.
- Kopfkalibrierung, Gamma-/Drive-Kurven und logisch kalibrierte Soft-/Hard-Kanaele existieren als Datenmodell, sind im produktiven Head-Pfad aber noch nicht verdrahtet.

Schwarzschild- bzw. Reziprozitaetsverhalten des Papiers wird nicht explizit modelliert. Fuer normale LED-Papierbelichtungen im typischen Arbeitsbereich ist das wahrscheinlich kleiner als die aktuellen spektralen Kalibrierluecken. Bei sehr kurzen, sehr langen oder stark gedimmten Belichtungen kann es sichtbar werden und sollte erst nach realen Papiermessungen kanalweise als Korrektur eingefuehrt werden.

## Gepruefter Pfad

### Profil und Gradationsmathematik

- `src/teensy/PaperExposureProfile.h`
  - `gradeMode`, `useIsoMath`, `isoP`, `isoR`, `kBw`, `kSoft`, `kHard`
  - `gradeKSoft[11]`, `gradeKHard[11]` fuer 0.0 bis 5.0 in 0.5er-Schritten
  - explizite Trennung vom Kopf, von Gamma und NeoPixel-Ausgabedetails
- `src/teensy/SplitgradeMath.h`
  - `gradeIndexFromFloat()`: Gradation 0.0..5.0 -> Index 0..10
  - `profileLutSoftFraction()`: LUT/ISO-Modell -> Soft-Anteil 0..1
  - `isoTargetScaleForProfile()`: bei ISO-Math `100 / isoP`
  - `isoHardFractionForGrade()`: linearer Hard-Anteil mit begrenztem ISO-R-Bias
- `src/teensy/SplitgradeWorkflow.cpp`
  - `baseTarget_` startet bewusst mit Default `10.0`, nicht mehr mit `kBw`
  - `refreshPaperDrivenTargets()` bildet `softTarget_` und `hardTarget_`
  - Startkommandos sind `StartSoft` und `StartHard` mit jeweils einem Zielwert

### Laufzeit und Head-Ausgabe

- `src/teensy/main.cpp`
  - `processSplitgradeExecutionCommands()` startet je Phase `startTimeExposure()` oder `startDoseExposure()`
  - `resolveExposureBaseColor()` setzt SG-Soft auf `RGB(0,255,0)` und SG-Hard auf `RGB(0,0,255)`
  - BW-FixedGrade setzt derzeit `RGB(0,255,255)`; BW-Multigrade setzt `RGB(0, 255*softMix, 255*hardMix)`
- `src/teensy/ExposureEngine.cpp`
  - Zeitmodus: lineare Zeit in Sekunden
  - Dosismodus: lineare Integration `currentDose += measuredLux * dtSeconds`
  - keine Papier-, Spektral- oder Schwarzschildlogik
- `src/teensy/NeoPixelHead.cpp`
  - `toNeoPixelColor()` reicht rohe 8-Bit-RGB-Werte an `RgbColor(red, green, blue)` weiter
  - keine Gamma-Korrektur, kein per-channel Drive-Mapping, keine spektrale Papierkorrektur
  - Softstart, Softstop und Crossfade sind technische Head-Uebergaenge
- `src/teensy/HeadSpectrumCommand.h` und `src/teensy/HeadCalibrationProfile.h`
  - beschreiben bereits den richtigen semantischen Zielzustand
  - laut Kommentar und Code derzeit noch nicht in LightController, ExposureEngine oder NeoPixelHead verdrahtet

## Groessen und Einheiten

| Groesse | Ort | Einheit / Bedeutung | Bewertung |
| --- | --- | --- | --- |
| `grade` | Workflow | 0.0..5.0, 0.5er Raster | stabil quantisiert |
| `softFraction`, `hardFraction` | `SplitgradeMath` / Workflow | dimensionslose Anteile, Summe ca. 1 | gut als Misch-/Splitvertrag |
| `softTarget_`, `hardTarget_` | `SplitgradeWorkflow` | je nach `controlMode`: Sekunden oder lx*s | technisch korrekt, fotografisch nur mit passender Kalibrierung belastbar |
| `targetDose` | `ExposureEngine` | lx*s aus lokalem Sensorpfad | linear integriert, keine spektrale Papiergewichtung |
| `LightRgb` | Head-Pfad | rohe 8-Bit-RGB-Werte | physische Ausgabe, keine fotografische Einheit |
| `kBw` | `PaperExposureProfile` | laut aktuellem Kommentar dimensionsloser Transmissions-/Empfindlichkeitsfaktor | wird zurecht nicht als Laufzeit-Target benutzt |
| `kSoft`, `kHard` | Profil | derzeit fachlich mehrdeutig: Basisfaktor, Normierungsanker oder historische Dosis | muss fuer automatische Korrekturen getrennt werden |
| `gradeKSoft[]`, `gradeKHard[]` | Profil | LUT-Faktoren je Gradation | kann spektrale Papierantwort abbilden, wenn end-to-end gemessen |

## Invarianten

Diese Invarianten sind im aktuellen Code weitgehend erfuellt:

- Gradation bleibt im Bereich 0.0..5.0 und im 0.5er Raster.
- Soft- und Hard-Anteile sind nichtnegativ und werden auf 0..1 geklemmt.
- SG-Sequenzbetrieb schaltet keine Mischfarbe, sondern reine Soft-/Hard-Phasen.
- Die `ExposureEngine` kennt keine UI- oder Papiersemantik und bleibt bei Zeit, Lux, lx*s und Temperatur.
- Technische Head-Latenz bleibt in ms/us und wird nicht mit EV-/Papiermathematik vermischt.

Diese Invarianten sind noch nicht fachlich abgesichert:

- Ein gleicher lx*s-Zielwert ist fuer Gruen und Blau fotografisch gleichwertig.
- Ein linearer NeoPixel-8-Bit-Wert ist proportional zur papierwirksamen Exposition.
- Eine simultane Gruen-/Blau-Belichtung entspricht einer Sequenz aus Soft-/Hard-Dosisanteilen.
- Eine bei einer Belichtungszeit gemessene Papier-LUT bleibt bei deutlich anderen Zeiten unveraendert.

## Befunde

### B1 - Spektrale Papierempfindlichkeit ist nur indirekt modelliert

`profileLutSoftFraction()` kann reale Papierempfindlichkeit abbilden, wenn `gradeKSoft[]` und `gradeKHard[]` aus einer realen Papier-/Kopf-/Sensor-Kalibrierung stammen. Der aktuelle Laufzeitpfad verwendet daraus aber nur normalisierte Anteile. Absolute Kanalempfindlichkeiten, LED-Spektralleistung und Sensor-Spektralantwort werden nicht separat in den Head- oder Dose-Pfad eingerechnet.

Risiko: Mittel bis hoch. Fuer manuell eingestellte SG-Zeiten ist das akzeptabel. Fuer automatische Dosisvorschlaege, BW-Multigrade und reproduzierbare Papierprofile reicht es noch nicht.

### B2 - ISO-Math ist ein plausibler Bias, kein papiergemessener Kurvenfit

`useIsoMath` skaliert das Basistarget mit `100 / isoP` und verschiebt die Hard-Fraktion ueber einen begrenzten `isoR`-Bias. Das ist besser als ein rein linearer Fallback, ersetzt aber keine gemessenen H&D-Kurven fuer Soft und Hard.

Risiko: Mittel. Als Startwert gut, als behauptete Splitgrade-Kalibrierung zu grob.

### B3 - SG-Sequenzfarbe ist robust, BW-Multigrade-Farbe ist nur ratio-richtig

Im SG-Sequenzbetrieb ist `RGB(0,255,0)` fuer Soft und `RGB(0,0,255)` fuer Hard sauber und regressionsarm. Die eigentliche Mischung entsteht durch getrennte Zielwerte.

Im BW-Multigrade-Simultanbetrieb wird dagegen direkt gerechnet:

```text
green = 255 * softMix
blue  = 255 * hardMix
```

Beispiel bei 60/40: Part2 gibt etwa `G=153, B=102` aus. Die historische v0.3-BW-Mischung hat den staerkeren Kanal auf 255 normiert und den anderen proportional gesetzt, also etwa `G=255, B=170`. Beide erhalten das Spektralverhaeltnis, aber Part2 ist deutlich dunkler. Im Zeitmodus aendert das die Belichtungswirkung direkt. Im Dosismodus wird die Gesamtmenge nur insoweit kompensiert, wie der lokale Luxsensor die papierwirksame Spektralmischung korrekt repraesentiert.

Risiko: Hoch fuer BW-Multigrade als finaler Printmodus; niedrig fuer SG-Sequenzbetrieb.

### B4 - HeadCalibrationProfile und HeadSpectrumCommand sind noch nicht produktiv

Die richtigen Datenstrukturen existieren bereits: logische Kanaele `soft`, `hard`, `focusWhite`, globale RGB-Skalen und Drive-Kurven. Der produktive Pfad nutzt sie aber nicht. Dadurch ist der aktuelle NeoPixel-Ausgang ein Roh-RGB-Pfad.

Risiko: Hoch fuer Reproduzierbarkeit. Ohne diesen Mapping-Rand kann man spektrale Head-Kalibrierung nur in Papierprofilen verstecken oder manuell ueber Zielwerte kompensieren.

### B5 - Sensor-lx*s ist nicht automatisch papier-lx*s

Die `ExposureEngine` integriert `measuredLux * dt`. Diese Groesse kommt aus dem lokalen Sensorpfad. Multigrade-Papier hat aber unterschiedliche gruen-/blauempfindliche Schichten. Ein breitbandiger oder anders spektral gewichteter Sensor kann Gruen und Blau anders sehen als das Papier.

Konsequenz: Dose-Mode ist fuer Wiederholbarkeit des Geraets gut, aber ohne kanalweise Sensor-zu-Papier-Kalibrierung kein absoluter Garant fuer gleiche Papierwirkung ueber Soft und Hard.

Risiko: Mittel bis hoch, besonders bei automatischer Uebertragung von Messwerten auf SG-Ziele.

### B6 - Schwarzschild/Reziprozitaet ist nicht explizit implementiert

Der Code nimmt Linearitaet an:

```text
H = Integral Lux(t) dt
Papierwirkung = Funktion(H, Gradation)
```

Es gibt kein Feld fuer einen Schwarzschild-Exponenten, keine Zeitbereichs-LUT, keine kanalabhaengige Reziprozitaetskorrektur und keine Warnung fuer Extrapolation ausserhalb des kalibrierten Zeitbereichs.

Eine reale Stouffer-/H&D-Kalibrierung absorbiert Reziprozitaetseffekte nur lokal fuer die verwendete Belichtungszeit, Intensitaet, Farbe und Entwicklung. Sie beweist nicht automatisch, dass die gleiche LUT bei z.B. 0.5 s, 10 s und 120 s identisch bleibt.

Risiko: Niedrig bis mittel im normalen Arbeitsbereich, hoch bei Extremzeiten oder Praezisions-/Densitometrieanspruch.

### B7 - Head-Fades sind technische Dosisanteile, aber keine Papierkorrektur

`NeoPixelHead` blendet Einschalten, Ausschalten und Farbwechsel. Der Dosispfad kompensiert Abschaltvorlauf ueber Bus-/Fade-Latenz, aber das ist technische Latenzkompensation, keine fotografische Schwarzschild- oder Spektralkorrektur.

Praxis: Bei 5..30 s ist der Einfluss klein. Bei sehr kurzen Teststreifen oder Minizusatzbelichtungen kann der lineare Fade-Anteil aber in die Groessenordnung sichtbarer Zehntelstufen kommen, wenn er nicht durch Dosisintegration oder Kalibrierung erfasst wird.

## Praxisrelevanz im Kontext dieses Geraets

### Was heute wahrscheinlich gut genug ist

- Manueller SG-Sequenzdruck mit separaten Soft-/Hard-Zeiten.
- Wiederholbare Belichtungen, solange Kopf, Papier, Chemie, Sensorposition und Modus gleich bleiben.
- Profil-LUT als Bedien- und Startpunkt, wenn der Anwender weiterhin Teststreifen macht.

### Was heute nicht belastbar behauptet werden sollte

- Absolute Gradationsrichtigkeit allein aus `isoP`/`isoR`.
- Gleichwertigkeit von Sensor-lx*s fuer Gruen und Blau ohne kanalweise Kalibrierung.
- BW-Multigrade-Simultanmix als physikalisch identisch zum SG-Sequenzmix.
- Reziprozitaetsstabilitaet ueber sehr kurze und sehr lange Zeiten.

### Grobe Einflussabschaetzung

| Effekt | Normalbereich ca. 5..30 s | Kurz/Extrembereich | Prioritaet |
| --- | --- | --- | --- |
| Rohes G/B-NeoPixel-Mapping ohne Kopfkalibrierung | sichtbar moeglich, besonders BW-Mix | stark moeglich | hoch |
| Sensor-/Papier-Spektralmismatch im Dose-Mode | relevant, wenn automatische Dosen genutzt werden | relevant bis stark | hoch |
| Fehlende H&D-/LUT-Kalibrierung | je nach Papier deutlich sichtbar | deutlich sichtbar | hoch |
| Head-Fade/Timing | meist klein | sichtbar bei kurzen Zeiten | mittel |
| Schwarzschild/Reziprozitaet | wahrscheinlich kleiner als obige Effekte | sichtbar bei sehr lang/kurz | mittel bis niedrig, spaeter messen |

Ohne reale Messreihen ist keine serioese numerische Korrektur moeglich. Fachlich ist aber klar: zuerst Spektral-/Kopf-/Sensor-Kalibrierung stabilisieren, danach Reziprozitaet messen.

## Priorisierte notwendige Schritte

### P0 - Begriffe und Einheiten festziehen

Ziel: verhindern, dass K-Faktoren, Sekunden und lx*s erneut vermischt werden.

Konkrete Schritte:

1. Entscheiden und dokumentieren, ob `kSoft`/`kHard` im aktiven Modell Normierungsanker, lux*s-Kalibrierwerte oder nur historische Felder sind.
2. Fuer automatische Belichtungen getrennte Felder vorsehen, falls noetig:
   - Papierempfindlichkeit / Profilfaktor
   - Zielwert in Sekunden
   - Zielwert in Sensor-lx*s
   - kanalweise Soft/Hard-Korrektur
3. In UI/Doku keine automatische Kalibrierpraezision behaupten, solange keine reale end-to-end Session referenziert ist.

Aufwand: klein.  
Nutzen: hoch, weil Folgefehler vermieden werden.

### P1 - Spektrum-Mapping-Rand produktiv einfuehren

Ziel: fotografische Logik nicht laenger direkt in rohe RGB-Werte giessen.

Konkrete Schritte:

1. `resolveExposureBaseColor()` durch einen semantischen Pfad ersetzen oder ergaenzen:
   - SG Soft -> `HeadSpectrumSemantic::SplitgradeSoft`
   - SG Hard -> `HeadSpectrumSemantic::SplitgradeHard`
   - BW MG -> `HeadSpectrumSemantic::BwGradeMix` mit `channels.soft/hard`
   - FixedGrade BW -> eigene Weisslicht-/BW-Semantik, nicht `LocalFocus`
2. Mapping `HeadSpectrumCommand -> HeadLightCommand` als eigene kleine Funktion/Klasse einziehen.
3. Initiale Identitaetsabbildung so setzen, dass SG keine Regression bekommt:
   - Soft bleibt `RGB(0,255,0)`
   - Hard bleibt `RGB(0,0,255)`
4. Danach `HeadCalibrationProfile` schrittweise nutzen: globale Skalen, Kanal-Mixe, Drive-Kurven.

Aufwand: mittel.  
Nutzen: sehr hoch. Das ist der richtige Hebel fuer Spektral- und Gamma-Korrektur.

### P1 - Reale Papier-/Kopf-/Sensor-Kalibrierung als Abnahmepunkt

Ziel: LUT-Werte muessen aus echten Prints stammen, nicht aus Defaults.

Konkrete Schritte:

1. Das vorhandene End-to-End-Papierkalibrierprotokoll fuer mindestens ein Papier voll ausfuellen.
2. Pro Kanal separat messen:
   - reines Soft/Gruen
   - reines Hard/Blau
   - optional BW-Mix/Weisslicht
3. Pro Messlauf speichern:
   - Kopfhelligkeit / globales Limit
   - Sensorquelle und Sensorposition
   - Belichtungszeit und Zielmodus
   - Entwickler/Temperatur/Prozessnotiz
4. `gradeKSoft[]` und `gradeKHard[]` nur als freigegeben markieren, wenn Rohdaten referenziert sind.

Aufwand: mittel bis hoch im Labor, klein in Firmware.  
Nutzen: sehr hoch, weil erst dadurch spektrale Papierempfindlichkeit wirklich erfasst wird.

### P2 - BW-Multigrade-Mix korrigieren oder begrenzen

Ziel: simultaner BW-Mix darf nicht still dunkler und nur scheinbar korrekt sein.

Optionen:

1. Minimal: BW-Multigrade als experimentell/unkalibriert kennzeichnen und SG-Sequenz als belastbaren Printpfad bevorzugen.
2. Klein: BW-Mix wie v0.3 max-normalisieren, also staerkeren Kanal auf 255 setzen und den anderen proportional. Das erhaelt das Verhaeltnis und vermeidet unnoetigen Helligkeitsverlust, ist aber noch keine Papierkalibrierung.
3. Besser: BW-Mix ueber `HeadSpectrumCommand` und `HeadCalibrationProfile` abbilden, danach mit realem Papier pruefen.

Empfehlung: erst P1-Spektrum-Mapping einfuehren, dann max-normalisierte Identitaetsabbildung als Default fuer BW-MG pruefen.

### P2 - Dose-Mode kanalweise kalibrieren

Ziel: lokales Sensor-lx*s muss fuer Soft und Hard in papierwirksame Einheiten uebersetzt werden koennen.

Konkrete Schritte:

1. Pro Kanal Sensor-lx*s gegen Papierergebnis messen.
2. Falls Abweichung stabil ist, kanalweise Korrekturfaktoren einfuehren.
3. Wenn Abweichung nicht stabil ist, Dose-Mode fuer automatische SG-Proposals begrenzen und Time-Mode plus Teststreifen bevorzugen.

Aufwand: mittel.  
Nutzen: hoch fuer Closed-Loop-Praezision.

### P3 - Schwarzschild erst messen, dann modellieren

Ziel: keine theoretische Korrektur ohne Papierdaten einbauen.

Konkrete Schritte:

1. Fuer ein Referenzpapier je Kanal eine kleine Zeitreihe messen, z.B. gleiche Zieldichte ueber unterschiedliche Intensitaet/Zeit:
   - 1 s, 3 s, 10 s, 30 s, 90 s
2. Pruefen, ob die benoetigte Dosis systematisch mit Zeit/Farbe driftet.
3. Wenn Drift im Praxisbereich kleiner als Prozessrauschen bleibt: nur Warn-/Metadaten speichern.
4. Wenn Drift sichtbar ist: pro Papier und Kanal eine einfache Korrektur einfuehren:
   - Gueltigkeitsbereich `tMin/tMax`
   - optional Exponent oder kleine LUT `time -> exposureCorrectionStops`
   - UI-Warnung bei Extrapolation

Aufwand: Messaufwand hoch, Firmwareaufwand klein bis mittel.  
Nutzen: erst nach P1/P2 relevant.

### P3 - Tests und Akzeptanzharness erweitern

Ziel: mathematische und spektrale Vertrage regressionsfest machen.

Konkrete Checks:

- SG Grade 0.0 -> Soft-only, physisch weiterhin Gruen-only.
- SG Grade 5.0 -> Hard-only, physisch weiterhin Blau-only.
- SG Grade 2.5 -> korrekte LUT-Anteile und keine Target-Drift.
- BW-MG 60/40 -> erwartetes Mapping nach gewaehlter Normalisierung.
- Head-Kalibrierprofil Identity -> exakt alter RGB-Ausgang fuer SG.
- Head-Kalibrierprofil mit Skalen -> monotone, geklemmte 8-Bit-Ausgabe.

## Loesungsmoeglichkeiten

### Loesung A - Minimal stabilisieren

Keine grosse Firmwarelogik. Nur Begriffe, Doku und UI-Claims schaerfen; SG-Sequenz als belastbaren Pfad behandeln; BW-Mix als noch nicht freigegeben markieren.

Vorteil: sehr geringer Aufwand, geringes Risiko.  
Nachteil: keine echte Verbesserung der spektralen Praezision.

Geeignet, wenn jetzt zuerst UI/Workflow fertig werden soll.

### Loesung B - Kleiner produktiver Spektralrand

`HeadSpectrumCommand -> HeadLightCommand` einfuehren, Identity-Mapping fuer SG, max-normalisierte oder kalibrierbare BW-MG-Abbildung. `HeadCalibrationProfile` bleibt zunaechst optional mit Identity-Defaults.

Vorteil: richtige Architektur, kleine Schritte, regressionsarm.  
Nachteil: braucht danach echte Kalibrierdaten.

Empfehlung: bester naechster Firmware-Schritt.

### Loesung C - Vollstaendiges fotografisches Modell

End-to-end H&D-Kalibrierung, kanalweise Sensor-/Papierfaktoren, Head-Gamma, Spektralmapping und optional Schwarzschild-LUT pro Papier/Kanal.

Vorteil: fachlich sauber fuer hohe Praezision.  
Nachteil: Messaufwand und Datenmodell wachsen deutlich.

Geeignet erst nach realer Papiermessung und stabilem Head-Mapping.

## Empfohlene Reihenfolge

1. P0 Einheitenentscheid fuer `kSoft/kHard/gradeK*` festschreiben.
2. P1 Spektrum-Mapping-Rand einfuehren, SG-Ausgabe per Identity-Tests unveraendert halten.
3. P1 reales Papierkalibrierprotokoll fuer ein Referenzpapier ausfuellen.
4. P2 BW-Multigrade-Mapping entscheiden: max-normalisiert als Zwischenloesung oder voll ueber HeadCalibrationProfile.
5. P2 Dose-Mode kanalweise gegen Papier pruefen.
6. P3 Schwarzschild nur dann implementieren, wenn Messdaten im relevanten Zeitbereich einen sichtbaren Drift zeigen.

## Audit-Fazit

Der aktuelle Stand ist kein Fehlentwurf. Die wichtigste Trennung ist bereits richtig: Papiermath oben, ExposureEngine einheitenrein, NeoPixelHead hardwareseitig. Die Luecke liegt an der Uebergangsstelle zwischen fachlichem Spektrum und rohem RGB sowie an fehlenden realen Papier-/Sensor-Kalibrierdaten.

Mit wenig Aufwand sollte als naechstes nicht eine komplexe Schwarzschildformel entstehen, sondern ein sauberer Spektrum-Mapping-Rand plus ein realer Kalibrierlauf. Schwarzschildkorrektur bleibt fachlich sinnvoll, aber erst nach der spektralen Kalibrierung und nur dann, wenn Messreihen im Praxisbereich einen relevanten Effekt zeigen.