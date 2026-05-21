# Dukatimer historische Umsetzung der fotografischen Lichtsteuerung

## Ziel

Dieses Dokument beschreibt die historisch belegte Umsetzung der fotografischen Lichtsteuerung im Dukatimer.

Der Fokus liegt nicht auf UI oder allgemeiner Moduslogik, sondern auf genau der Frage, wie die Altstaende die Luecke zwischen:

- linearen 8-Bit-RGB-Ausgangswerten an den LEDs
- unterschiedlicher spektraler Wirkung von Gruen, Blau und Weiss
- papierabhaengiger fotografischer Reaktion
- reproduzierbarer Belichtungszeit oder Ziel-Dosis

praktisch behandelt haben.

## Kurzfassung

Historisch wurde das Problem nicht in einer dedizierten LED-Treiber-Korrekturschicht geloest.

Stattdessen laesst sich ueber die Versionen hinweg ein vierstufiges Muster erkennen:

1. Eine rohe elektrische Ausgabeschicht schreibt RGB-Werte direkt an den Lichtkopf.
2. Eine fachliche Mischlogik legt fest, welches Spektrum fuer BW, SG, Burn oder Teststrip ueberhaupt gewuenscht ist.
3. Eine papierbezogene Kalibrierung bildet gemessene Lux-Werte auf K-Faktoren, Splitgrade-Anteile und spaeter profilgebundene Misch- oder Zeitwerte ab.
4. Die Exposure-Logik setzt daraus Zeit- oder Dosisablaeufe um, teils offen ueber Zeit, teils spaeter geschlossen ueber gemessene Lux-Integration.

Die historische Antwort auf das Problem war also primaer:

- Spektrum fachlich vorgeben
- Papier kalibrieren
- Belichtungszeit oder Dosis aus Messung und Profil ableiten

und gerade nicht:

- eine saubere, physikalisch kalibrierte Transferfunktion pro LED-Kanal im Treiber selbst.

## Begriffsrahmen

Damit die historischen Staende konsistent lesbar bleiben, werden in diesem Dokument vier Ebenen unterschieden.

### 1. Elektrischer Treiber

Diese Ebene kennt nur PWM- oder RGB-Werte und schreibt sie auf den Lichtkopf.

Beispielhafte Frage:

- Welcher 8-Bit-Wert wird an Gruen oder Blau ausgegeben?

### 2. Spektrale Sollbildung

Diese Ebene entscheidet fachlich, welche Farbe oder Kanalverteilung fuer einen Modus genutzt werden soll.

Beispielhafte Frage:

- Soll BW als fester Gruen-Blau-Mix, als reine Weissflaeche oder als getrennte Soft-/Hard-Phase laufen?

### 3. Papier- und Kalibrierlogik

Diese Ebene enthaelt K-Faktoren, Kalibriermessungen, D-logH- oder Smart-Math-Regeln und die Ableitung von Zeiten oder Mischgewichten aus Messwerten.

Beispielhafte Frage:

- Wie wird aus gemessenem Lux und kalibriertem Papierprofil eine Zielzeit?

### 4. Exposure-Regelung

Diese Ebene fuehrt die Belichtung wirklich aus.

Beispielhafte Frage:

- Wird einfach eine Zeit abgelaufen oder wird laufend Lux integriert, bis eine Ziel-Dosis erreicht ist?

## Historische Entwicklungsrichtung

Die vorhandene Beleglage im Workspace zeigt eine klare Entwicklungslinie:

- `2.30` fuehrt bereits Papier-K-Faktoren und Messlogik ein.
- `2.240` bringt einen praktisch vollwertigen Dunkelkammer-Workflow mit Kalibrierung, Teststrip, Preflash und einer festen Gradations-zu-RGB-Mischlogik.
- `B1.0` wird historisch als Recheniteration fuer Splitgrade beschrieben, nicht als Neubau der Lichttreiberschicht.
- `v0.3` vertieft die Fachlogik stark und fuehrt eine explizite papierprofilbasierte Gruen-Blau-PWM-Normalisierung ein.
- `v0.9` trennt die Architektur sauberer und verschiebt die fotografische Korrektur noch staerker in Papierprofil, D-logH-Kurve und Dosisregelung.

Die zusammenfassende Historienanalyse liegt bereits in `docs/software/erledigt/dukatimer-historische-funktionsanalyse.md` vor. Dieses Dokument zoomt tiefer in die konkrete Licht- und Kalibrierlogik hinein.

## Version 2.30

### Gesichert belegte Elemente

Die konsolidierte Historienanalyse weist fuer `2.30` bereits folgende Bausteine aus:

- Papierprofile mit `Ksoft`, `Khard` und `Kbw`
- aktiven Messpfad mit TSL2591
- Densitometer-/Filmtest-Workflow
- vorhandenes `useDoseMode` im Settings-Modell

### Technische Einordnung

Fuer die hier relevante Fragestellung bedeutet das:

- Das System denkt bereits in papierbezogenen K-Faktoren, nicht nur in nackten Zeiten.
- Die fotografische Korrektur liegt damit schon frueh in einer Profil- und Messschicht.
- Es gibt in der belegten Dokumentation keinen Hinweis, dass `2.30` bereits eine eigene, kalibrierte LED-Transferfunktion pro Kanal besass.

### Bedeutung

`2.30` markiert den Ursprung der Idee, dass Dunkelkammer-Belichtung nicht allein ueber eine statische Zeitvorgabe laufen soll, sondern ueber ein Zusammenspiel aus:

- Messung
- Papierbezug
- K-Faktoren
- Bedienworkflow

## Version 2.240

`2.240` ist der erste im Workspace direkt einsehbare Altstand, der die Lichtfrage schon relativ vollstaendig praktisch beantwortet.

### Roher LED-Ausgabepfad

Die eigentliche Ausgabe ist sehr einfach:

- `scalePwm(int val)` skaliert nur global mit `set_max`.
- `PaintLED(int r, int g, int b)` schreibt die skalierten Werte direkt auf alle Pixel.

Damit gilt fuer `2.240`:

- keine Gamma-Korrektur
- keine kanalindividuelle Kennlinie
- keine per-Kanal-LUT fuer LED-Nichtlinearitaet
- keine Trennung zwischen Soll-Spektrum und kalibriertem Kopfmodell

Die Treiberschicht ist also elektrisch simpel und fachlich weitgehend dumm.

### Feste spektrale Sollbildung

Trotz des simplen Treibers gibt es in `2.240` bereits eine explizite fachliche Mischlogik fuer den BW-Pfad:

- `multival[11][3]` bildet die 11 Splitgrade-Stufen auf feste RGB-Vorgaben ab.

Das ist historisch wichtig.

Denn damit existiert in `2.240` bereits eine klare Ebene:

- `Grade -> gewuenschter RGB-Mix`

Allerdings ist diese Ebene noch statisch und handkodiert.

Sie ist nicht:

- kopfkalibriert
- papierkalibriert im LED-Sinne
- mathematisch als eigene Transferfunktion abstrahiert

sondern eine feste fachliche Vorgabe.

### Papierbezogene Zeitkorrektur

Parallel dazu arbeitet `2.240` mit einem papierbezogenen Modell:

- `PaperProfile` enthaelt `Ksoft`, `Khard`, `Kbw`.
- Der Kalibrierwizard misst getrennt fuer `G5`, `G0` und `G2.5`.
- Die gemessenen Werte werden direkt in diese K-Faktoren geschrieben.

Danach wird die Belichtungszeit aus Messwert und Profil abgeleitet:

- `time = K / lux`

Das ist die historische Kernantwort auf die fotografische Reproduzierbarkeit.

Die praktische Aussage lautet:

- Nicht der LED-Treiber wird kalibriert.
- Stattdessen wird das Gesamtsystem aus Lichtkopf, Sensor und Papierreaktion ueber K-Faktoren in die Zeitdomäne abgebildet.

### Gradeabhaengige Zeitverschiebung

Zusätzlich existiert `paperSpeed[11]`.

Diese Tabelle verschiebt beim Aendern der BW-Gradation die Zeit mit.

Damit steckt in `2.240` bereits ein zweiter Korrekturmechanismus:

- feste Gradations-zu-RGB-Vorgabe ueber `multival`
- feste papierbezogene Geschwindigkeitskorrektur ueber `paperSpeed`

### Logarithmische Messauswertung

Im Densitometerpfad wird bereits mit `log10(...)` gearbeitet.

Das ist ein deutlicher Hinweis, dass die fotografische Domäne historisch nie als linear verstanden wurde.

Die Logik bildet also Licht und Dichte bewusst logarithmisch aus, auch wenn der LED-Ausgang selbst nur mit linearen 8-Bit-Werten arbeitet.

Fuer Part2 folgt daraus eine klare Darstellungsregel:

- echte Messworkflows wie Densitometrie, Kalibrierung oder Vergleichsmessung sollen Werte zusaetzlich in EV ausgeben
- der Rohwert, zum Beispiel Lux, bleibt parallel sichtbar
- reine Laufzeittelemetrie ohne fotografische Interpretation bleibt in der physikalischen Grundeinheit

Diese logarithmische Aufbereitung soll nicht pro Workflow neu entstehen, sondern ueber eine gemeinsame EV-/Formatter-Schicht bereitgestellt werden.

### Teststrip und Preflash

`2.240` erweitert die fachliche Lichtlogik weiter:

- Teststrip arbeitet additiv und nutzt kanalabhaengige Farben.
- Preflash besitzt eigene slot-gebundene Parameter.
- Preflash ist vom Teststrip funktional getrennt.

Auch hier zeigt sich dasselbe Muster:

- Fachlogik und Papierworkflow werden ausgebaut.
- Der LED-Treiber bleibt simpel.

### Technische Einordnung von 2.240

`2.240` loest das Problem also ueber drei Bausteine:

1. feste fachliche RGB-Mischvorgaben
2. papierbezogene K-Faktoren aus Messung
3. logarithmisches bzw. fotografisches Denken in den Mess- und Dichtepfaden

Was `2.240` noch nicht sauber loest:

- per-Kopf-Kalibrierung des realen LED-Ausgangs
- Trennung von Soll-Spektrum und elektrischer Ansteuerung
- explizite kalibrierte Kopf-Transferfunktion

## Version B1.0

### Beleglage

Im aktuellen Workspace liegt fuer `B1.0` keine direkt lesbare Firmwaredatei vor, wohl aber die konsolidierte Historienanalyse.

Diese beschreibt `B1.0` als:

- funktional nah an `2.240`
- mit erweiterter `Smart Math` fuer Splitgrade
- ohne Hinweis auf eine neuartige LED-Treiber-Korrekturschicht

### Einordnung

Damit ist `B1.0` fuer diese Fragestellung vor allem wichtig als:

- Recheniteration in der Splitgrade- und Belichtungsmathematik
- nicht als historischer Wendepunkt fuer den elektrischen Lichttreiber

Die historische Hauptlinie bleibt also auch dort:

- fachliche und mathematische Verbesserung oberhalb des Treibers
- kein Beleg fuer eine dedizierte LED-Gamma- oder Kanalkennlinienkorrektur im Treiber selbst

## Version v0.3

`v0.3` ist im Workspace der tiefste belegbare Stand fuer die Licht- und Kalibrierlogik.

### Roher Treiberpfad

Die physische NeoPixel-Ausgabe bleibt auch in `v0.3` direkt:

- `HW_SetEnlargerNeoPixel(r, g, b)` schreibt die Werte nach `expR`, `expG`, `expB`.
- `renderNeoPixels()` uebergibt diese direkt an `pixels.ClearTo(RgbColor(...))`.
- Safe und Focus werden ebenfalls direkt als rohe RGB-Werte gerendert.

Auch hier gibt es im Projektcode keinen Beleg fuer:

- Gamma-LUT
- Brightness-Wrapper mit per-Kanal-Korrektur
- White-Balance- oder Color-Correction-Schicht

### Papierprofilbasierte PWM-Mischung

Die grosse Weiterentwicklung in `v0.3` liegt eine Ebene hoeher.

`updateGradeMath()` berechnet fuer den BW-Modus die effektiven Gruen- und Blau-PWM-Werte aus dem Papierprofil.

Dabei werden zwei historische Strategien unterstuetzt:

- `useIsoMath`: lineare Baseline aus ISO-/Datenblattannahmen
- LUT-Modus: direkte Nutzung von `gradeK_Soft[]` und `gradeK_Hard[]`

Danach wird normalisiert:

- der staerkere Kanal wird auf `255` gesetzt
- der schwaechere proportional dazu skaliert

Diese Normalisierung ist fachlich bedeutsam, weil sie erstmals explizit versucht:

- das gewuenschte spektrale Verhaeltnis zu erhalten
- gleichzeitig maximale Lichtleistung zu nutzen

Damit ist `v0.3` historisch die erste klar belegte Version, die aus papierprofilbasierten Faktoren direkt PWM-Verhaeltnisse fuer Gruen und Blau erzeugt.

### Splitgrade-Zeitberechnung

`calculateSplitTimes(...)` arbeitet parallel dazu ebenfalls papierprofilbasiert:

- im ISO-Pfad aus einem Datenblattmodell
- im LUT-Pfad aus `gradeK_Soft[]` und `gradeK_Hard[]`

Das Ergebnis sind `timeGreen` und `timeBlue`.

Historisch koexistieren in `v0.3` damit zwei Antworten auf dasselbe Problem:

- spektrale Mischung direkt im PWM-Raum
- oder spektrale Aufteilung im Zeitraum

### Kalibrierung ueber getrennte Spektralmessungen

Die Kalibrierlogik misst getrennt `G0` und `G5` und erzeugt daraus:

- `Ksoft`
- `Khard`
- spaeter auch `gradeK_Soft[]` und `gradeK_Hard[]`

Die direkte Ableitung aus Messwert und Kalibrierzeit ist zentral.

Historisch ist damit die eigentliche Kalibrierung nicht:

- `PWM -> Licht`

sondern:

- `gemessenes Licht -> papierbezogene Dosis- bzw. Zeitkonstante`

### Mess- und Korrekturlogik

`processSpotMeasurement(...)` zeigt den fotografischen Charakter des Systems besonders klar:

- Dunkelstrom wird abgezogen.
- Lux wird fuer Zonenbildung logarithmisch ueber `log2(...)` interpretiert.
- Das Verhaeltnis von `corrG0 / corrG5` wird als `spectral_ratio` gespeichert.

Damit bildet `v0.3` nicht nur absolute Helligkeit, sondern auch spektrale Relationen historisch bereits als Messgroesse ab.

### Technische Einordnung von v0.3

`v0.3` beantwortet das Problem deutlich weiter entwickelt als `2.240`:

- Der Treiber bleibt roh.
- Die fachliche Mischung wird papierprofilbasiert.
- Die Kalibrierung nutzt getrennte Spektralmessungen.
- Die Messlogik arbeitet fotografisch logarithmisch.

Damit ist historisch auch begruendet, warum Part2 fotografische Messdarstellung und F-Stop-/EV-Bedienlogik nicht als einzelne Sonderfaelle pro Modus behandeln sollte.

Was aber weiterhin fehlt:

- eine eigenstaendige, kalibrierte Kopf-Transferfunktion
- eine saubere Abstraktion `Soll-Spektrum -> kalibrierte Kanalansteuerung`

## Version v0.9

`v0.9` reorganisiert die Architektur und verschiebt die fotografische Korrektur noch klarer in Profil- und Exposure-Schichten.

### Roher Treiberpfad bleibt erhalten

`HardwareManager::updateNeoPixels(r, g, b)` speichert rohe RGB-Werte.

`renderLightOutputs()` schreibt diese direkt als `RgbColor(_currentR, _currentG, _currentB)` an den NeoPixelBus.

Auch in `v0.9` gibt es im Projektcode keinen Beleg fuer eine separate Gamma- oder White-Balance-Treiberschicht.

### Papiermodell wird expliziter

`PaperProfile` enthaelt nun klar strukturiert:

- `isoP`, `isoR`
- `Ksoft`, `Khard`, `Kbw`
- `gradeK_Soft[11]`, `gradeK_Hard[11]`
- Kalibrierstatus

`PaperManager::calculateAndSaveCalibration(...)` bildet aus gemessenem `luxG0` und `luxG5` erneut `Ksoft` und `Khard` ab.

### D-logH-Kurve

Eine wichtige historische Weiterentwicklung ist `_applyDLogH(x)`.

Die Funktion verwendet eine kubische Smoothstep-Kurve als einfache D-logH-Approximation.

Damit wird die historische fotografische Kompensation expliziter mathematisch modelliert als in `2.240` und in Teilen auch sauberer als die frueheren festen Tabellen.

### Splitgrade- und Zeit-/Dosispfade

`computeSplitFactors(...)`, `calculateSplitGradeTimes(...)` und `calculateSplitGradeDose(...)` zeigen deutlich:

- Die Hauptkompensation passiert im Papiermodell.
- Die Belichtung nutzt diese Faktoren fuer Zeit oder Dosis.

### ExposureEngine

Im Ausfuehrungspfad gibt es in `v0.9` zwei wesentliche Strategien:

- Soft und Hard koennen als getrennte Vollpegel-Phasen `255,0` und `0,255` laufen.
- Im Dosisbetrieb wird gemessenes Lux ueber die Zeit integriert, bis die Ziel-Dosis erreicht ist.

Damit ist `v0.9` historisch am weitesten entfernt von einer simplen offenen Zeitsteuerung.

Die Kompensation ist hier am staerksten:

- in Papierprofilen
- in Faktorberechnung
- in Dosisregelung

und am wenigsten im LED-Treiber.

## Was historisch nie sauber geloest wurde

Ueber alle belegten Staende hinweg fehlt eine explizite, eigenstaendige Schicht fuer die Kopfkalibrierung.

Nicht sauber modelliert sind historisch insbesondere:

- per-Kanal-Transferkurven des realen Lichtkopfs
- LED-Nichtlinearitaet als eigene kalibrierte LUT
- getrennte Modelle fuer Soll-Spektrum und elektrische Ansteuerung
- kalibrierte Kopfprofile je Hardwarevariante
- eine klare Pipeline `fachlicher Farbauftrag -> kalibrierter Lichtkopf -> fotografische Dosis`

Das historische System kompensiert die Praxis trotzdem teilweise erfolgreich, aber ueber Umwege:

- feste RGB-Mischungen
- papierbezogene K-Faktoren
- Grade-Tabellen
- Zeitkorrektur
- Dosisintegration

## Historische Gesamtarchitektur in einer Zeile

Die Altstaende modellieren nicht den Lichtkopf selbst als kalibriertes System, sondern kalibrieren vor allem das Ergebnis aus Lichtkopf, Papier und Messung.

## Konsequenz fuer Part2

Fuer Part2 folgt daraus eine klare Lehre.

Die historische Umsetzung sollte nicht blind als eine einzige Logikschicht portiert werden.

Sinnvoll ist stattdessen eine explizite Aufspaltung in drei getrennte Modelle:

1. `HeadSpectrumCommand`
   Diese Schicht beschreibt das fachlich gewuenschte Spektrum oder die gewuenschte Kanalverteilung, zum Beispiel BW Grade 2.5, Soft, Hard, Burn, Teststrip oder Safelight.

2. `HeadCalibrationProfile`
   Diese Schicht bildet das Soll-Spektrum auf reale, hardwarekalibrierte Kanalwerte oder LUTs fuer den konkreten Lichtkopf ab.

3. `PaperExposureProfile`
   Diese Schicht enthaelt die papierbezogenen K-Faktoren, Splitgrade-Regeln, D-logH-Modelle und spaeter weitere fotografische Kalibrierdaten.

Erst diese Trennung beseitigt die historische Vermischung von:

- spektraler Sollbildung
- elektrischer Ansteuerung
- papierbezogener Kompensation

## Zusammenfassung

Die historische Umsetzung des Problems laesst sich belastbar so beschreiben:

- `2.30` fuehrt Messung und K-Faktoren ein.
- `2.240` arbeitet bereits mit fester Gradations-zu-RGB-Mischung und papierbezogener Zeitkompensation.
- `B1.0` verfeinert die Splitgrade-Mathematik.
- `v0.3` erzeugt papierprofilbasiert direkte Gruen-Blau-PWM-Verhaeltnisse und getrennte Zeitpfade.
- `v0.9` verschiebt die Kompensation noch klarer in Profil-, D-logH- und Dosisregelschichten.

Historisch ist die Kompensation also vorhanden, aber sie sitzt ueberwiegend oberhalb des LED-Treibers.

Die Altstaende loesen das fotografische Problem deshalb funktional, aber nicht mit einer modernen, sauber getrennten Kopf-Kalibrierarchitektur.
