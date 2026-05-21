# End-to-End-Papierkalibrierprotokoll

Stand: 2026-05-02

## Zweck

Dieses Protokoll ist die zentrale Abnahmequelle fuer die reale End-to-End-
Kalibrierung von Lichtkopf, Sensorik, Dosisintegration und Papierprofil im
aktiven `Dukatimer-Part2`-System.

Es ersetzt keine bestehenden Bring-up- oder Einzeldiagnosen, sondern zieht die
relevanten Hardwareclaims an einem fotografisch sinnvollen Ort zusammen:

- reale Lichtabgabe des Kopfes
- lokaler TSL2561-Closed-Loop-Pfad
- optionaler TSL2591-Remote-Messpfad auf Papierebene
- Dosisintegration und Shutoff-Verhalten
- Ableitung in das aktive `PaperExposureProfile`

Die Grundregel lautet:

- keine isolierte "Kopf-ist-schnell-genug"-Abnahme ohne Papierbezug
- keine papierbezogene Kalibrierbehauptung ohne belegte Rohdaten
- jede Rueckschreibung in `ISO-P`, `ISO-R`, `kBw`, `kSoft`, `kHard` oder SG-LUT
  muss auf dokumentierte Messreihen referenzieren

## Geltungsbereich

Das Protokoll deckt den aktiven Laufzeitpfad in diesen Bereichen ab:

- `src/teensy/NeoPixelHead.*`
- `src/teensy/HeadLightArbiter.*`
- `src/teensy/ExposureEngine.*`
- `src/teensy/SensorManager.*`
- `src/teensy/MeasurementDomainService.*`
- `src/teensy/main.cpp`
- `PaperExposureProfile`-Rueckschreibung ueber aktive Slot-/Profilpfade

Relevante Begleitdokumente:

- [head_timing_and_i2c_timeout.md](../head_timing_and_i2c_timeout.md)
- [neo_exposure_integration.md](../neo_exposure_integration.md)
- [offene_entscheidungen.md](../software/offene_entscheidungen.md)

## Verbindliche Referenzartefakte

Fuer jede echte Kalibriersession muessen diese Artefakte benannt und im Protokoll
mitgefuehrt werden:

- real verwendetes Papier inklusive Emulsions-/Chargenbezug, falls bekannt
- aktiver Paper-Slot und aktueller Profilstatus (`Demo`, `gemessen`, `importiert`,
  `gesperrt` oder projektintern gleichwertig)
- verwendeter Lichtkopf / Hardwarestand / Verdrahtungsstand
- eingesetztes Messgeraet oder Messgeraet-Set
- vorhandener 21-Stufen-Durchsichtsgraukeil als festes Referenzartefakt
- eingesetztes Objektiv, Blende, vergroesserungsrelevanter Aufbau

Hinweis zum Sensorpfad:

- Der lokale TSL2561 bleibt die Closed-Loop- und Safety-Quelle direkt am
  Licht-/Projektionspfad.
- Der TSL2591-Remote-Pfad ist die papierseitige Messquelle.
- Vor dem ersten Lichtlauf ist ein gemeinsamer Dunkelabgleich beider Sensoren
  verpflichtend, damit Nullpunktfehler, Streulicht und Drift nicht in den
  spaeteren Papier- oder Graukeilabgleich eingehen.
- Ein erster Sensorabgleich ist bewusst ohne Negativ zu fahren, damit das reine
  Delta aus Messgeometrie, Optik und spaeter negativbedingter Abschwaechung
  nicht vermischt wird.

## Sessionkopf

Vor jeder Messreihe ist mindestens dieser Kopfblock zu fuellen:

| Feld | Wert |
| --- | --- |
| Session-ID | |
| Datum / Uhrzeit | |
| Bedienende Person | |
| Zielpapier / Charge | |
| Aktiver Slot / Profilname | |
| Profilstatus vor Session | |
| Teensy-Firmwareversion | |
| ESP32-S3-Firmwareversion | |
| C6-Firmwareversion / Terminalstand | |
| Protokoll-/Dokumentstand | |
| Lichtkopf / Hardwarevariante | |
| Objektiv / Blende / Vergroesserungsaufbau | |
| Negativtraeger / Negativtyp | |
| Messgeraet(e) | |
| Umgebung / Temperatur | |
| 21-Stufen-Graukeil-ID | |
| Rohdatenablage | |
| Bemerkungen | |

## Messreihen-Invalidierung

Ein Lauf darf nicht zur Profilableitung verwendet werden, wenn einer dieser
Faelle auftritt:

- `sensorFallbackActive` waehrend der eigentlichen Kalibrierbelichtung
- lokaler TSL2561-Fault oder latched Watchdog-Reset
- `thermalDeratingActive` oder `ThermalHardStop`
- erkennbar stale/lost Remote-Messpfad, wenn der papierseitige TSL2591 Teil des
  Laufs ist
- fehlender, nicht referenzierter oder instabiler Dunkelabgleich beider
  Sensoren vor dem ersten Lichtlauf
- geaenderte Blende, Geometrie, Filterstellung oder Papiercharge ohne neuen
  Sessionkopf
- fehlende Rohdatenzuordnung zu einer abgeleiteten Profilgroesse

Invalidierte Laeufe werden nicht geloescht, sondern mit Grund dokumentiert.

## Ablauf

### 1. Vorab-Sicherheits- und Bring-up-Pruefung

Vor dem ersten papierrelevanten Lauf dokumentieren:

- Build-/Firmwarestand
- sichtbaren Sensorstatus
- Head-Timing-Serienreport, falls fuer den Hardwarestand aktiviert
- Zustand von Raumlicht, SSR und Head-Ausgabe
- Hinweis, ob die Session mit realem Papier oder nur Dummyflaeche begonnen wurde

### 2. Sensorabgleich ohne Negativ

Ziel: Den Basisabgleich zwischen lokalem TSL2561 und papierseitigem TSL2591 ohne
negativbedingte Dichte oder Filterwirkung dokumentieren.

Der Sensorabgleich ohne Negativ besteht immer aus zwei Teilen:

- zuerst Dunkelabgleich beider Sensoren als Nullpunkt- und Streulichtpruefung
- danach Licht-Basisabgleich ohne Negativ bei unveraenderter Geometrie

#### 2.1 Dunkelabgleich beider Sensoren

Ziel: Sicherstellen, dass beide Sensorpfade unter realem Dunkelzustand auf ihrer
jeweiligen Dunkelbasis liegen und kein verdecktes Restlicht, Offset oder Drift
spaetere Papiermessungen verfaelscht.

Aufbauvorgabe:

- kein Negativ im Strahlengang
- Kopf-Ausgabe sicher aus
- Raumlicht aus, Save/Focus/Room-Ausgaenge dokumentiert aus
- Objektivpfad lichtdicht abgedeckt oder an der Bildebene sicher verschlossen
- TSL2591 an der realen Papierposition, ebenfalls gegen Restlicht abgeschirmt
- keine Geometrieaenderung zwischen Dunkelabgleich und anschliessendem
  Licht-Basisabgleich

Durchfuehrung:

- mindestens drei Wiederholungen mit identischem Aufbau fahren
- je Wiederholung beide Sensorwerte erst nach kurzer Beruhigungszeit ablesen
- Rohwerte als Dunkelbasis protokollieren, nicht nur "ok" abhaken
- falls ein externer Referenzmesser vorhanden ist, dessen Dunkelwert ebenfalls
  mitschreiben

Wichtig:

- Der Dunkelabgleich verlangt keine identischen Lux-Werte beider Sensoren.
  TSL2561 und TSL2591 haben unterschiedliche Messgeometrie und spektrale
  Antwort. Verbindlich ist nicht Rohwertgleichheit, sondern dass beide Sensoren
  jeweils stabil auf ihrer eigenen Dunkelbasis bleiben.
- Wenn einer der beiden Sensoren im Dunkelzustand nicht stabil ist, darf der
  anschliessende Lichtabgleich nicht als belastbare Referenz fuer Papier- oder
  Profilwerte verwendet werden.

Pro Lauf dokumentieren:

| Lauf-ID | Aufbau dunkel bestaetigt | TSL2561 Dunkelwert | TSL2591 Dunkelwert | Externes Messgeraet dunkel | Drift / Auffaelligkeit | Freigabe |
| --- | --- | --- | --- | --- | --- | --- |
| DA-01 | | | | | | |
| DA-02 | | | | | | |
| DA-03 | | | | | | |

Freigaberegel fuer den naechsten Schritt:

- Der Licht-Basisabgleich darf nur auf einem dokumentierten `DA-xx` aufbauen.
- Im Feld `Drift / Auffaelligkeit` sind besonders Restlicht, wandernde Werte,
  Sensor-Faults, stale Remote-Daten oder mechanische Undichtigkeiten zu nennen.
- Falls noetig, wird der gemessene Dunkelwert spaeter als dokumentierter Offset
  zum jeweiligen Sensorlauf referenziert; stilles "Wegdenken" ist unzulaessig.

#### 2.2 Licht-Basisabgleich ohne Negativ

Pro Lauf dokumentieren:

| Lauf-ID | DA-Referenz | Blende | Lichtauftrag / Gradation | TSL2561 lokal | TSL2591 Papier | Externes Messgeraet | Delta / Bemerkung |
| --- | --- | --- | --- | --- | --- | --- | --- |
| SN-01 | | | | | | | |
| SN-02 | | | | | | | |
| SN-03 | | | | | | | |

Regel fuer die Auswertung:

- Jede `SN-xx`-Zeile muss auf eine dokumentierte `DA-xx`-Zeile verweisen.
- Wenn fuer einen Sensor ein nicht vernachlaessigbarer Dunkeloffset beobachtet
  wurde, ist das Delta nur auf Basis der dokumentierten Dunkelreferenz zu
  interpretieren.
- Ein neuer Dunkelabgleich ist Pflicht, sobald Geometrie, Abdeckung, Aufbau,
  Raumlichtsituation oder Sensorposition veraendert wurden.

### 3. Head- und Laufzeitvalidierung innerhalb der Papiersession

Diese Werte werden nicht separat als "zweites Protokoll" gefuehrt, sondern in
derselben Session dokumentiert:

| Claim | Messquelle | Soll | Ist | Toleranz | Lauf-ID / Rohdaten | Bemerkung |
| --- | --- | --- | --- | --- | --- | --- |
| Head-Latenz / predictive shutoff | Serienreport / Log | | | | | |
| Dosislinearitaet | Papierreihe / Messgeraet | | | | | |
| Kanalstrom / Leistungsgrenze | Messgeraet / Aufbau | | | | | |
| Thermische Grenze / Derating | Log / Temperatur | | | | | |
| Sensorverdrahtung / Signalstabilitaet | Sensorstatus / Log | | | | | |

### 4. Referenzlauf mit 21-Stufen-Durchsichtsgraukeil

Der Graukeil ist das feste Referenzartefakt fuer Vergleichs- und Ableitungslaeufe.
Er wird mindestens fuer den Basispfad und fuer die papierbezogene Profilableitung
mitgefuehrt.

Empfohlene minimale Staffelung:

- Referenzlauf ohne Negativ zur Sensor-/Geometriebasis
- Graukeil-Lauf fuer den papierbezogenen Verlauf
- falls relevant getrennte Laeufe fuer `G0`, `G2.5` und `G5` oder die aktive
  SG-Familie

Rohdatentabelle:

| Lauf-ID | Referenzartefakt | Gradation / Modus | Zielwert (Zeit oder Dosis) | TSL2561 | TSL2591 | Externes Messgeraet / Dichte | Bildbeobachtung | Rohdatenpfad |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| GW-01 | 21-Stufen-Graukeil | | | | | | | |
| GW-02 | 21-Stufen-Graukeil | | | | | | | |
| GW-03 | 21-Stufen-Graukeil | | | | | | | |

### 5. Ableitung ins aktive Paper-Profil

Jede Rueckschreibung muss auf dokumentierte Lauf-IDs verweisen.

| Profilgroesse | Abgeleiteter Wert | Herkunfts-Lauf-IDs | Rechenweg / Quelle | Freigabe |
| --- | --- | --- | --- | --- |
| `kBw` | | | | |
| `kSoft` | | | | |
| `kHard` | | | | |
| `ISO-P` | | | | |
| `ISO-R` | | | | |
| `SG LUT 0.0` | | | | |
| `SG LUT 0.5` | | | | |
| `SG LUT 1.0` | | | | |
| `SG LUT 1.5` | | | | |
| `SG LUT 2.0` | | | | |
| `SG LUT 2.5` | | | | |
| `SG LUT 3.0` | | | | |
| `SG LUT 3.5` | | | | |
| `SG LUT 4.0` | | | | |
| `SG LUT 4.5` | | | | |
| `SG LUT 5.0` | | | | |
| relevante Kopf-/Sensorparameter | | | | |

## Abschlussblock

Am Ende jeder Session verbindlich festhalten:

- ist das Papierprofil nach dieser Session weiter `Demo`, bereits `gemessen`
  oder nur vorlaeufig plausibilisiert?
- welche Claims sind wirklich abgenommen?
- welche Claims bleiben offen?
- welche Laeufe wurden invalidiert und warum?
- wo liegen Rohdaten, Fotos, Logauszuege und ggf. Dichtemessungen?

Kurzsignatur:

| Feld | Wert |
| --- | --- |
| Session abgeschlossen am | |
| Profil rueckgeschrieben | Ja / Nein |
| Rueckschreibedatum | |
| Freigegeben durch | |
| Offene Restpunkte | |

## Fertig, wenn

Dieser Punkt gilt erst dann als fachlich belastbar bearbeitet, wenn:

- das Protokoll fuer mindestens einen realen Papier-/Kopf-/Sensorstand komplett
  ausgefuellt ist
- die Rohdaten erreichbar sind
- abgeleitete Profilwerte auf konkrete Lauf-IDs zeigen
- kein Claim auf getrennten, nicht referenzierten Einzelnotizen beruht

---

## Anhang A — Mathematik der Profilparameter

Dieser Anhang dokumentiert die genaue mathematische Herleitung der Profilgroessen
`ISO-R`, `ISO-P`, `kBw`, `kSoft` und `kHard`, die im `PaperExposureProfile`
persistiert werden. Er gilt gleichermassen fuer die vollstaendige H&D-Methode
(Labor) und fuer Methode 1 (visuelle Schwellenwert-Methode im CAL-Panel).

### A.1 Graukeil-Grundmodell

Verwendet wird ein 21-Stufen-Transmissionsgraukeil (Typ Stouffer T2115 oder
aequivalent). Die nominale Dichte jeder Stufe s = 1..21 berechnet sich als:

```
D(s) = 0.05 + (s - 1) * 0.15
```

Stufenbereich: D(1) = 0.05 (hoechste Transmission) bis D(21) = 3.05
Stufenschritt: 0.15 log-Einheiten = 0.5 Blendenstufen

Die Beleuchtungsstaerke hinter Stufe s bei nominaler Kopf-Ausgabe I0:

```
I(s) = I0 * 10^(-D(s))
```

Referenzbeleuchtung: I0 = 150.0 lx (hardwarestandabhaengiger Richtwert,
muss pro Setup gemessen werden).

Die relative logarithmische Exposition bezogen auf die dunkelste Stufe (s=21):

```
logH(s) = log10( I(s) / I(21) )
```

Diese Normierung stellt sicher, dass logH(21) = 0 und logH(1) = log10(I(1)/I(21))
ist, und dass die gesamte Keil-Dosisreihe relativ zueinander dargestellt wird,
unabhaengig von I0.

### A.2 ISO-R — Nutzbarer Kontrastumfang

ISO-R beschreibt den nutzbaren Belichtungsbereich des Papiers in Einheiten von
1/100 einer log-Dosiseinheit (d.h. in Einheiten von 10 ms logarithmischer Exposition).

Nutzgrenzen nach ISO-Papiernormierung:
- Untere Grenze: D_min + 0.04 (erster visuell signifikanter Ton ueber Weiss)
- Obere Grenze:  D_max - 0.10 (kurz vor Absaettigung)

```
ISO-R = round( (logH(D_max - 0.10) - logH(D_min + 0.04)) * 100 )
```

Typischer Wertebereich: ISO-R 90..160 fuer Normalpapier.

### A.3 ISO-P — Charakteristische Referenzdosis

ISO-P ist die absolute Belichtungsdosis [lx*s] an der mittleren Papierdichte
des nutzbaren Bereichs:

```
D_mid  = (D_min + 0.04 + D_max - 0.10) / 2
H_mid  = I(21) * 10^(logH(D_mid))        [lx*s]
ISO-P  = H_mid
```

Die ExposureEngine verwendet ISO-P als Ankerpunkt: Wenn die am TSL2591 gemessene
Dosis gerade ISO-P erreicht, hat das Papier die mittlere Referenzbelichtung erhalten.

### A.4 kBw — Bandwidth-Faktor

kBw normiert die Schwellendosis (bei D_min + 0.04) auf die interne Referenzdosis:

```
H_ref = I0 * T_base = 150.0 lx * 10.0 s = 1500.0 lx*s
kBw   = H(D_min + 0.04) / H_ref
```

Typischer Wertebereich: kBw 0.001..0.020.

kBw ist ein dimensionsloser Empfindlichkeitsskalierungsfaktor: ein groesseres kBw
bedeutet, dass das Papier schon bei relativ wenig Licht den Schleier-Grenzton
erreicht (hohes Empfindlichkeitspapier). Ein kleines kBw entspricht einem
weniger empfindlichen Papier.

### A.5 kSoft und kHard — Splitgrade-Verteilungsfaktoren

Beim Splitgrade-Modus wird die Gesamtdosis auf zwei Filterstellungen verteilt:

```
f_Hard(SG) = (SG / 5.0) ^ 1.4
f_Soft(SG) = 1 - f_Hard(SG)
```

Der Exponent 1.4 sorgt fuer eine nichtlineare, psychophysisch plausible Verteilung:
- SG = 0.0: vollstaendig Soft (weiches Licht, minimaler Kontrast)
- SG = 5.0: vollstaendig Hard (hartes Licht, maximaler Kontrast)
- SG = 2.5: ca. 60% Soft / 40% Hard (Mittelgradation)

Als Referenzgradation fuer die Profilablage wird SG = 2.5 verwendet:

```
kSoft = f_Soft(2.5) ≈ 0.6
kHard = f_Hard(2.5) ≈ 0.4
```

kSoft und kHard werden von Methode 1 **nicht** veraendert und muessen separat
gesetzt werden, sofern die Standardwerte nicht passen.

---

## Anhang B — Methode 1: Visuelle Schwellenwert-Methode

### B.1 Zweck und Einsatzbereich

Methode 1 ist die Schnellkalibrierung ohne Densitometer. Sie eignet sich als:

- Erstkalibrierung bei Inbetriebnahme
- Schnellpruefung nach Papierwechsel oder Optikjustage
- Feldkalibrierung ohne Laborausruestung

Sie ist **nicht** geeignet als abschliessender Beleg fuer eine Profilfreigabe, wenn
densitometrisch belegbare Werte verlangt werden. In diesem Fall ist Anhang A
(vollstaendige H&D-Methode) durchzufuehren.

### B.2 Benoetigte Materialien

- Belichteter und entwickelter Probestreifen mit dem 21-Stufen-Graukeil
  (s. Abschnitt 3 des Hauptprotokolls)
- Kein Densitometer erforderlich

### B.3 Vorgehensweise am Probestreifen

1. Den entwickelten Probestreifen unter normalem Weisslicht (Aufsichtlicht)
   beurteilen — nicht unter Dunkelkammerlicht.

2. **Weissunkt N bestimmen:**
   Von Stufe 1 (hellste Stufe) nach unten (dunkler werdend) gehen.
   N ist die letzte Stufennummer, bei der noch ein eindeutig sichtbarer,
   vom reinen Papierweiss verschiedener Grauton erkennbar ist.
   Typischer Richtwert: N = 14..16.

3. **Schwarzpunkt M bestimmen:**
   Von Stufe 21 (dunkelste Stufe) nach oben (heller werdend) gehen.
   M ist die erste Stufennummer, bei der der volle Schwarzton gerade noch
   vollstaendig geschweirzt ist (kein sichtbarer Detailverlust).
   Typischer Richtwert: M = 7..10.

4. **Wichtig: N muss groesser als M sein.** Wenn N <= M ist, wurde mindestens
   einer der beiden Punkte falsch bestimmt, oder der Probestreifen ist zu kurz
   belichtet / ueberbelichtet.

5. Die Werte N und M im Dukatimer CAL-Panel unter STEP WHITE (N) und
   STEP BLACK (M) eintragen. Die Firmware berechnet ISO-R, ISO-P und kBw sofort.

### B.4 Mathematische Herleitung der Firmware-Formeln

Die folgenden Formeln sind exakt so in `PaperWorkflow::recalculateFromSteps()`
implementiert. Graukeil-Densitaetsmodell: D(s) = 0.05 + (s-1) * 0.15.

**ISO-R:**
```
ISO-R = (N - M) * 15
```
Jede Keilstufe umfasst 0.15 log-Einheiten = 15 ISO-R-Einheiten.
(N - M) Stufen entsprechen damit dem nutzbaren Kontrastumfang nach ISO-Normierung.

**kBw:**
```
D_N  = 0.05 + (N - 1) * 0.15
kBw  = 10^(-D_N)
```
An der Weissgrenze gilt: Die Beleuchtungsstaerke hinter Stufe N ist I_N = I0 * 10^(-D_N).
kBw normiert diese auf I0: kBw = I_N / I0 = 10^(-D_N).

**ISO-P:**
```
step_mid = (N + M) / 2.0
D_mid    = 0.05 + (step_mid - 1) * 0.15
ISO-P    = H_ref * 10^(-D_mid)
```
mit H_ref = 1500.0 lx*s (interne Referenzdosis).

Der Dosismittelpunkt liegt bei der Stufe (N + M) / 2. Die Beleuchtungsstaerke
hinter dieser Stufe ist I_mid = I0 * 10^(-D_mid). Die Dosis bei Referenzzeit T_base:
H_mid = I_mid * T_base = I0 * T_base * 10^(-D_mid) = H_ref * 10^(-D_mid).

### B.5 Beispielrechnung N=15, M=8

```
D(15) = 0.05 + 14 * 0.15 = 2.15
D(8)  = 0.05 +  7 * 0.15 = 1.10
D(11.5) = 0.05 + 10.5 * 0.15 = 1.625

ISO-R = (15 - 8) * 15 = 105
kBw   = 10^(-2.15) ≈ 0.00708
ISO-P = 1500 * 10^(-1.625) ≈ 35.5 lx*s
```

### B.6 Kalibrierungsprotokoll fuer Methode 1

| Feld | Wert |
| --- | --- |
| Session-ID | |
| Datum | |
| Papier / Charge | |
| Probestreifen-ID | |
| Belichtungszeit des Probestreifens [s] | |
| Kopf-Ausgabe [%] | |
| Graukeil-ID | |
| Visuell bestimmter Weissunkt N | |
| Visuell bestimmter Schwarzpunkt M | |
| Berechnetes ISO-R | |
| Berechnetes kBw | |
| Berechnetes ISO-P [lx*s] | |
| kSoft (manuell gesetzt oder Standard) | |
| kHard (manuell gesetzt oder Standard) | |
| Freigabe / Bemerkungen | |

### B.7 Abgrenzung zu Methode 2 (densitometrische H&D-Kurvenanpassung)

| Merkmal | Methode 1 | Methode 2 (H&D) |
| --- | --- | --- |
| Densitometer erforderlich | nein | ja |
| Kurvenanpassung erforderlich | nein | ja |
| Genauigkeit ISO-R | +-15 (1 Stufe) | +-3..5 |
| Genauigkeit ISO-P | +-20..30% | +-5..10% |
| Geeignet fuer Freigabe | Erstkalibrierung / Feld | verbindliche Profilfreigabe |
| kSoft / kHard abgeleitet | nein (manuell) | ja (aus LUT) |
| Implementiert in Firmware | ja (CAL-Panel) | nein (Offline-Skript) |

