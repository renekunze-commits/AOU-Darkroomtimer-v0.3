# Dukatimer historische Closed-Loop-Regelung

## Ziel

Dieses Dokument beschreibt die historisch belegte Closed-Loop-Regelung im Dukatimer.

Der Schwerpunkt liegt auf der konkreten Laufzeitregelung waehrend einer Belichtung:

- welche Groesse historisch geregelt wurde
- welche Hardware dafuer eingesetzt wurde
- wie die Software diese Rueckfuehrung umgesetzt hat
- welche Moeglichkeiten dadurch entstanden
- welche Grenzen und Landminen in den Altstaenden klar erkennbar bleiben

Dieses Dokument ergaenzt die allgemeinere Analyse in `docs/software/erledigt/dukatimer-historische-umsetzung-fotografische-lichtsteuerung.md` und fokussiert ausschliesslich den Closed-Loop-Aspekt.

## Kurzfassung

Historisch bedeutete `Closed Loop` im Dukatimer nicht:

- kontinuierliche Helligkeitsregelung des Lichtkopfs
- PID-Regelung der LED-Kanaele
- laufende Nachfuehrung von Gruen-, Blau- oder Weissanteilen
- papierdichtegefuehrte Rueckkopplung

Historisch bedeutete `Closed Loop` vielmehr:

- ein Ziel in der Dosisdomaene vorzugeben
- den real gemessenen Lichtfluss waehrend der Belichtung zu integrieren
- den Lichtkopf praezise abzuschalten, sobald die Ziel-Dosis erreicht ist

Die eigentliche Rueckfuehrung sitzt also auf dem Belichtungsende, nicht auf der Lichtintensitaet.

Die sauber belegte Laufzeit-Closed-Loop-Regelung erscheint erst in `v0.3` und wird in `v0.9` architektonisch gereinigt. Die frueheren Staende `2.30`, `2.240` und `B1.0` enthalten wichtige Vorstufen wie Messung, K-Faktoren und Dosisdenken, aber keine gleichwertig belegte laufende Dosisintegration mit prädiktivem Shutoff waehrend der Belichtung.

## Was historisch geregelt wurde

Um die Altstaende sauber einzuordnen, muessen Fuehrungs-, Mess- und Stellgroesse getrennt werden.

### Fuehrungsgroesse

Die Fuehrungsgroesse ist historisch eine Ziel-Dosis in Lux-Sekunden oder ein daraus abgeleiteter Zielwert fuer BW oder SG.

Typisch ist das Modell:

- Papierprofil oder Messlogik liefern einen Zielwert
- dieser Zielwert wird als `targetDose` an die ExposureEngine gegeben

### Messgroesse

Die Messgroesse ist historisch nicht die Papierdichte und auch nicht die echte aktinische Papierwirkung, sondern ein am Sensor gemessener Lux-Wert am Lichtkopf oder im Belichtungspfad.

Die Regelung arbeitet also auf einem Proxy:

- gemessenes Licht am Sensor
- nicht direkt fotografisches Ergebnis auf dem Papier

Fuer Part2 folgt daraus eine saubere Trennung:

- die Laufzeitregelung bleibt intern lux- und dosisbasiert
- fotografische Bedien- und Messoberflaechen duerfen daraus zusaetzlich EV-Werte ableiten
- diese EV-Ableitung gehoert in eine gemeinsame Schicht oberhalb der Engine und nicht in einzelne Modi oder in die Engine selbst

### Stellgroesse

Die Stellgroesse ist historisch nicht der LED-Pegel.

Die LED-Kanaele werden waehrend einer laufenden Belichtung im Dosisbetrieb mit festen Kanalwerten betrieben, zum Beispiel:

- `255,0` fuer Soft
- `0,255` fuer Hard
- ein fixer Mix fuer BW

Die eigentliche Stellgroesse ist vielmehr:

- Belichtung laeuft weiter
- oder Belichtung wird abgeschaltet

Die Rueckfuehrung entscheidet also ueber den Abschaltzeitpunkt.

### Stellglied

Das Stellglied ist historisch die Belichtungsengine plus Abschaltpfad:

- NeoPixel oder Lichtkopf werden eingeschaltet
- ein Task integriert die laufende Dosis
- ein praeziser Timer oder Direkt-Shutoff beendet die Belichtung

### Was nicht im Regelkreis lag

Ausserhalb des historischen Regelkreises lagen insbesondere:

- per-Kanal-LED-Nichtlinearitaet
- Kopfkalibrierung als eigene Transferfunktion
- spektrale Kalibrierung des realen Lichtkopfs
- papierdichtebasierte Rueckmeldung
- adaptive Helligkeitsmodulation waehrend der Belichtung

## Historische Entwicklungslinie

### 2.30 als Vorstufe

Fuer `2.30` ist historisch belegt:

- `useDoseMode` existiert bereits im Settings-Modell
- Papierprofile enthalten `Ksoft`, `Khard` und `Kbw`
- ein TSL2591-Messpfad ist bereits Bestandteil des Systems

Das ist eine klare fachliche Vorstufe fuer spaetere Closed-Loop-Regelung.

Aber:

- es ist keine gleichwertig belegte Laufzeit-Engine fuer fortlaufende Dosisintegration sichtbar
- die vorhandene Beleglage zeigt Messung, Kalibrierung und Dosisdenken, aber keine ausformulierte Runtime-Regelschleife wie spaeter in `v0.3`

Einordnung:

`2.30` ist der Ursprung des Dosisdenkens, nicht der erste sauber belegte Laufzeitregler.

### 2.240 als messgestuetzte Vorwaertsregelung

`2.240` arbeitet historisch bereits mit:

- Papier-K-Faktoren
- getrennten Kalibrierwerten fuer `G0`, `G5` und `G2.5`
- der Zeitableitung `time = K / lux`
- fester Gradations-zu-RGB-Mischung via `multival[11][3]`

Damit wird die Belichtung bereits stark messgestuetzt vorbereitet.

Aber die eigentliche Laufzeitregelung bleibt historisch noch vorwaertsorientiert:

- Lux wird gemessen, bevor die Zeit festgelegt wird
- danach wird die Belichtung als Zeitvorgabe ausgefuehrt
- es gibt keinen gleichwertig belegten Dosis-Integrator, der waehrend der Belichtung laufend `lux * dt` aufsummiert und den Shutoff daraus bestimmt

Einordnung:

`2.240` ist kein roher Timer mehr, aber noch keine klar ausformulierte historische Runtime-Closed-Loop-Engine.

### B1.0 als Recheniteration

`B1.0` fuehrt historisch vor allem eine weiterentwickelte `Smart Math` fuer Splitgrade ein.

Wichtig fuer die Closed-Loop-Frage ist:

- es gibt keinen belastbaren Beleg fuer eine neu eingefuehrte Laufzeit-Rueckfuehrung des Lichtkopfs
- die Entwicklung betrifft primaer Mathematik und Vorschlagsbildung oberhalb der Belichtungsengine

Einordnung:

`B1.0` verfeinert die Eingangsseite der Belichtung, nicht den historischen Runtime-Regler selbst.

### v0.3 als erste klar belegte Laufzeit-Closed-Loop-Regelung

`v0.3` ist der erste im Workspace klar belegte Stand mit echter Laufzeit-Rueckfuehrung im Dosisbetrieb.

Dafuer sprechen mehrere konkrete Eigenschaften gleichzeitig:

- dedizierter Dosismodus `EXPMODE_DOSE`
- lokale Sensorpflicht vor Start
- eigener Closed-Loop-Task mit fester Periode
- laufende Dosisintegration aus Sensormessung und Delta-Zeit
- praediktiver Shutoff kurz vor Erreichen des Sollwerts
- harter Abschaltpfad ueber `esp_timer` und `HW_EmergencyShutoff()`

### v0.9 als architektonisch gereinigte Laufzeit-Closed-Loop-Regelung

`v0.9` behaelt das historische Regelprinzip bei, trennt aber die Verantwortlichkeiten sauberer:

- `ExposureEngine` regelt die Dosisbelichtung
- `SensorManager` bedient Spot-, Umwelt- und Wireless-Messungen
- `HardwareManager` kapselt Aktoren und Busaufteilung
- `SystemContext` entkoppelt Telemetrie und UI von der Engine

Der historische Sprung von `v0.3` zu `v0.9` ist also keine neue Regelidee, sondern eine deutliche Architekturverbesserung.

## v0.3 im Detail

### Hardwareaufbau in v0.3

Die Runtime-Closed-Loop-Regelung in `v0.3` basiert auf einer klar getrennten Sensorik.

Zur Laufzeitregelung gehoert:

- ein lokaler `TSL2561` als Head- oder Live-Sensor
- Initialisierung auf dem schnellen I2C-Bus `I2C_FAST`
- feste Sensorparameter mit `TSL2561_INTEGRATIONTIME_101MS` und `TSL2561_GAIN_1X`

Wichtig ist die globale Zuordnung in `main.cpp`:

- `tslHead` ist die reale TSL2561-Instanz
- `tslLive` ist nur ein Alias auf `tslHead`

Das bedeutet:

- historisch gibt es in `v0.3` keinen getrennten zweiten Live-Sensor fuer die Regelung
- `Head` und `Live` bezeichnen denselben Dosis-Sensorpfad

Parallel dazu existieren andere Sensorpfade fuer andere Zwecke:

- `TSL2591` als Basis- oder Spot-Messsensor
- Wireless-Probe ueber `remoteLux`
- spektrale Flash-Messungen fuer `G0` und `G5`

Diese Pfade sind wichtig fuer Kalibrierung und Messung, gehoeren aber nicht in denselben Closed-Loop-Regelkreis wie die Dosisbelichtung.

### Softwarepfad in v0.3

Der Regelpfad in `v0.3` ist klar strukturiert.

#### 1. Startbedingung

`ExposureEngine_StartDose(...)` blockiert den Start, wenn kein Live-Sensor verfuegbar ist.

Das ist historisch wichtig, weil damit die Dosisbelichtung nicht ohne Rueckfuehrung anlaufen soll.

#### 2. Sollwertuebergabe

Beim Start der Dosisbelichtung werden gesetzt:

- `expTargetDose` als interner Sollwert der Engine
- `target_dose` als globale Telemetrie- oder Kompatibilitaetsgroesse
- `current_dose = 0.0`

#### 3. Vorlaufphase

Vor der eigentlichen Belichtung laeuft eine definierte Sicherheitsphase:

- `PRE_WAIT_MS = 200`
- Blackout und Safelight-Off
- Relais- und Lichtberuhigung vor Mess- und Belichtungsbeginn

#### 4. Belichtung starten

Nach dem Pre-Wait schaltet `expBeginExposure()` den Lichtkopf mit festen Kanalwerten ein.

Im Dosismodus wird dabei noch kein Abschalttimer gesetzt. Stattdessen uebernimmt der Closed-Loop-Task die Restzeitbestimmung.

#### 5. Laufende Integration

Der Task `vExposureCLTask()` laeuft mit nominell 10 ms Periode.

Die Kernoperation lautet historisch sinngemaess:

- Sensor lesen
- `dt` bestimmen
- `current_dose += lux * dt`

Zusaetzlich wird `baseFlux = event.light` aktualisiert. Diese Groesse dient spaeter fuer die Restzeitabschaetzung.

#### 6. Praediktiver Shutoff

Sobald die Restdosis klein genug ist, bestimmt die Engine eine Restzeit:

- `remaining = targetDose - currentDose`
- `remaining_time ~= remaining / baseFlux`

Danach wird ein `esp_timer` auf diese Restzeit gesetzt.

`v0.3` beruecksichtigt dabei bereits die Ausgangslatenz des NeoPixel-Pfads:

- `NEOPIXEL_LATCH_DELAY_US = (NEOPIXEL_COUNT * 30) + 50`

Das ist historisch ein bemerkenswert ausgereifter Punkt der Laufzeitregelung: Der Regler steuert nicht nur auf Soll-Dosis, sondern kompensiert auch die bekannte Abschaltverzoegerung der WS2812-Kette.

#### 7. Harte Abschaltung

Wenn die Ziel-Dosis erreicht ist oder der praediktive Timer ausloest, wird ueber `HW_EmergencyShutoff()` sofort abgeschaltet.

Danach laeuft eine Nachlaufphase:

- `POST_WAIT_MS = 400`

Diese Phase soll Nachleuchten und Restemission abklingen lassen.

### Historische Funktionsreichweite von v0.3

Der `v0.3`-Regler kann historisch bereits mehr als nur eine einzelne BW-Dosisbelichtung.

Sauber belegt sind insbesondere:

- BW-Dosismodus ueber `startTimer()` und `ExposureEngine_StartDose(...)`
- SG-Dosismodus in zwei nacheinander ausgeloesten Phasen `Soft` und `Hard`
- Pause und Resume im Dosisbetrieb
- definierter Graceful Stop mit Post-Wait
- Telemetrie ueber `current_dose`

Wichtig ist dabei:

- die Regelung wird fuer BW und SG wiederverwendet
- SG bleibt trotzdem historisch keine spektral geregelte Belichtung
- stattdessen laufen zwei fest definierte Kanalphasen nacheinander unter derselben Dosislogik

### Historische Grenzen von v0.3

Trotz der fortgeschrittenen Dosisregelung bleiben klare Grenzen.

#### Keine Leistungsregelung des Lichtkopfs

`v0.3` regelt nicht die LED-Leistung selbst.

Der Lichtkopf laeuft waehrend einer aktiven Belichtungsphase mit festen Kanalwerten. Die Rueckfuehrung entscheidet nur, wann abgeschaltet wird.

#### Keine spektrale Rueckfuehrung

Es wird keine Abweichung zwischen Soll-Spektrum und realem Spektrum geregelt.

Die Regelung kennt nur den vom Sensor gesehenen Luxwert, nicht den Fehler pro spektralem Kanal.

#### Sensorintegration ist langsamer als die Taskperiode

Der Closed-Loop-Task laeuft zwar alle 10 ms, der TSL2561 ist in `v0.3` aber auf `101 ms` Integrationszeit konfiguriert.

Damit ist die echte Beobachtungsdynamik des Regelkreises deutlich langsamer als die Taskfrequenz vermuten laesst.

#### Wireless gehoert nicht in den Dosisregelkreis

`useWirelessProbe` wirkt in `HW_Sensors.cpp` auf Spot- und Basis-Messung.

Die Runtime-Closed-Loop-Engine selbst liest jedoch `tslHead` lokal. Historisch ist die drahtlose Sonde in `v0.3` daher kein Ersatz fuer den Laufzeit-Dosisregler.

#### Laufender Sensorausfall wird nicht aktiv abgefangen

Beim Start wird das Vorhandensein des Sensors geprueft.

Aber waehrend einer laufenden Dosisbelichtung gilt:

- wenn `tslHead.getEvent(...)` fehlschlaegt, wird der Zyklus einfach uebersprungen
- es gibt in diesem Pfad keinen erkennbaren Laufzeitfehler, kein Timeout und keinen sicheren Fallback-Timer

Das ist eine klare historische Landmine:

- faellt der Sensorrueckkanal waehrend der Belichtung aus
- dann friert die Dosisintegration effektiv ein
- und der Shutoff kann ausbleiben, bis ein externer Abbruch erfolgt

#### Regelung auf Lux, nicht auf Papierwirkung

Auch `v0.3` regelt nur einen Lux-basierten Proxy.

Ob das Papier auf diese Lichtmenge fotografisch genau so reagiert, wird ausserhalb des Regelkreises durch Papierprofil, Kalibrierung und Mathematik angenaehert.

## v0.9 im Detail

### Hardwareaufbau in v0.9

`v0.9` trennt den historischen Mess- und Regelpfad hardwareseitig sauberer als `v0.3`.

Die Busaufteilung ist explizit:

- `Wire` auf I2C-0 fuer langsame oder nichtkritische Peripherie
- `Wire1` auf I2C-1 fuer den Echtzeit-Dosispfad

Am I2C-0-Pfad haengen:

- `TSL2591` fuer Spot- oder Basismessung
- `BMP280` fuer Umweltdaten
- `DS18B20` fuer Temperatur
- optional die drahtlose Lux-Einspeisung ueber den `SensorManager`

Am I2C-1-Pfad haengt:

- `TSL2561` der `ExposureEngine` fuer den Dosisregler

Diese Trennung ist historisch wichtig, weil der Dosisregler dadurch nicht denselben Mutex-geschuetzten I2C-Pfad mit UI- und Umweltsensorik teilen muss.

### Softwarepfad in v0.9

Der `v0.9`-Pfad ist funktional aehnlich zu `v0.3`, aber sauberer aufgeteilt.

#### 1. Sensorinitialisierung

Die `ExposureEngine` initialisiert ihren eigenen `TSL2561` auf `Wire1`.

Dabei werden gesetzt:

- `enableAutoRange(true)`
- `TSL2561_INTEGRATIONTIME_13MS`

Gegenueber `v0.3` ist das eine deutlich schnellere Konfiguration fuer die Dosisregelung.

#### 2. Getrennte Verantwortlichkeiten

`SensorManager` und `ExposureEngine` messen historisch nicht dasselbe fuer denselben Zweck.

`SensorManager` uebernimmt:

- Spot- und Basismessung
- Auto-Gain fuer TSL2591
- Praezisionsmessungen mit Pre- und Post-Wait
- Wireless-Lux als Messdatenquelle
- Temperaturpfade

`ExposureEngine` uebernimmt:

- laufende Dosisintegration im Belichtungsbetrieb
- prädiktiven Shutoff
- State-Machine fuer Pre-Wait, Exposing, Pause und Post-Wait

#### 3. Closed-Loop-Task

`clTaskLoop()` laeuft alle 10 ms auf Core 1.

Nur wenn gleichzeitig gilt:

- Engine initialisiert
- Dosis-Sensor bereit
- Zustand `EXP_EXPOSING`
- Modus `EXPMODE_DOSE`

wird der Sensorwert gelesen und an `updateDoseAndPredictiveTimer(...)` uebergeben.

#### 4. Dosisintegration

`updateDoseAndPredictiveTimer(...)` fuehrt die historische Kernlogik aus:

- negatives Lux wird auf `0` begrenzt
- `_currentDose += lux * dtSeconds`
- Live-Telemetrie wird ueber `_ctx->updateLiveDose(cur)` in den Kontext geschrieben

Damit wird `v0.9` gegenueber `v0.3` fuer UI und App-Schichten besser beobachtbar.

#### 5. Praediktiver Shutoff

Sobald etwa `95 Prozent` der Ziel-Dosis erreicht sind, wird die Restzeit wieder aus dem aktuellen Luxwert berechnet.

Danach setzt `v0.9` einen `esp_timer` mit:

- Restzeit aus `remaining / lux`
- Mindestwert
- pauschaler Latch-Kompensation von `8000 us`

#### 6. Zustandsmaschine

Die State-Machine bleibt historisch klar:

- `EXP_PRE_WAIT`
- `EXP_EXPOSING`
- `EXP_PAUSED`
- `EXP_POST_WAIT`
- `EXP_DONE`

Auch Pause, Resume, Abort und Graceful Stop sind in die Regelarchitektur integriert.

### Historische Funktionsreichweite von v0.9

In `v0.9` ist die Closed-Loop-Engine kein isolierter Spezialpfad mehr, sondern Teil eines groesseren App-Systems.

Belegt sind insbesondere:

- BW-Dosis ueber `BWDoseApp`
- SG-Dosis ueber die Splitdose-Sequenz in `ExposureEngine::tick()`
- Burn ueber `SGPending` und damit denselben Belichtungskern
- Preflash ueber `BWPending` und denselben Dosispfad
- Live-Telemetrie fuer Apps ueber `liveDose`

Damit ist `v0.9` historisch der breiteste belegte Einsatz der Closed-Loop-Logik als wiederverwendbarer Belichtungsdienst.

### Historische Grenzen von v0.9

#### Weiterhin keine Leistungsregelung

Auch `v0.9` regelt nur den Abschaltzeitpunkt.

Gruen und Blau bleiben waehrend einer aktiven Teilphase fest vorgegeben. Es gibt keine Nachfuehrung des LED-Pegels an den Regelfehler.

#### Wireless ist nicht Teil des Runtime-Regelkreises

Die drahtlose Sonde wird in `WirelessManager` nur in den `SensorManager` injiziert.

Das betrifft den Mess- und Umgebungsbereich, nicht die ExposureEngine.

Zusaetzlich ist dort eine weitere historische Grenze sichtbar:

- `PRB_EVT_LUX_DATA` injiziert nur `lux_raw_g0`

Das ist fuer Spot- oder Hilfsmessung ausreichend, aber keine vollwertige Grundlage fuer einen spektral oder laufzeitkritisch geregelten Belichtungspfad.

#### Head-Lux-Telemetrie ist unvollstaendig verdrahtet

`SystemContext` besitzt zwar `headLux`, aber im belegten `v0.9`-Code wird dieser Wert im Sensorpfad nur weitergereicht, nicht aus der ExposureEngine laufend aktualisiert.

Damit entsteht eine historische Sichtbarkeitsluecke:

- `liveDose` wird sauber gepflegt
- `headLux` kann dagegen stale oder `0` bleiben

Apps, die `headLux` als Live-Groesse erwarten, koennen dadurch unvollstaendige oder irrefuehrende Telemetrie sehen.

#### Laufender Sensorausfall bleibt auch in v0.9 eine Landmine

Wie schon in `v0.3` gilt:

- wenn `_tsl.getEvent(&event)` fehlschlaegt, wird der Zyklus nur uebersprungen
- es ist kein klarer Runtime-Fehlerpfad sichtbar, der die Belichtung wegen Sensorverlust sicher beendet

Damit bleibt die gleiche Grundlandmine bestehen:

- bei Sensorausfall waehrend einer laufenden Dosisbelichtung kann die Dosisintegration stehen bleiben
- der praediktive Shutoff wird dann nicht mehr sauber nachgefuehrt

#### Regelgroesse bleibt nur ein Proxy

Auch `v0.9` regelt keine kalibrierte Papierwirkung, sondern Lux-Zeit am Sensor.

Papierprofil, D-logH-Kurve, Splitgrade-Mathematik und Kalibrierung bleiben weiterhin ausserhalb des engen Closed-Loop-Regelkreises.

## Historische Moeglichkeiten

Aus der belegten Closed-Loop-Implementierung ergeben sich historisch klare Staerken.

### Reproduzierbarere Dosis trotz Flussschwankung

Wenn der Lichtfluss waehrend der Belichtung leicht schwankt, ist eine laufende `lux * dt`-Integration robuster als eine rein offene Zeitsteuerung.

### Praeziseres Belichtungsende

Durch den `esp_timer` plus Restzeitabschaetzung kann der Belichtungsabschluss deutlich praeziser erfolgen als durch grobe Polling-Logik allein.

### Wiederverwendung fuer mehrere Modi

Die historische Closed-Loop-Logik ist kein reiner BW-Spezialfall.

Sie kann wiederverwendet werden fuer:

- BW-Dosis
- SG-Dosis
- spaeter Burn
- spaeter Preflash

Dieselbe Architekturregel gilt fuer die zugehoerige fotografische EV-/F-Stop-Bedienlogik: sie ist keine Einzelfunktion eines Modus, sondern eine wiederverwendbare Querschnittslogik fuer mehrere Belichtungsmodi.

### Sichere Vor- und Nachlaufphasen

Pre-Wait und Post-Wait sind Teil der historischen Regelarchitektur und kein blosses UI-Detail.

Damit werden reale Hardwareeffekte mitgedacht:

- Relais-Settle
- Blackout vor dem Start
- Nachleuchten oder Restemission nach dem Stop

### Telemetrie und Beobachtbarkeit

Spaetestens in `v0.9` ist die Dosisregelung auch fuer UI und uebergeordnete Apps sichtbar:

- laufende Dosis
- verbleibende Dosis
- Running- oder Pending-Zustaende

## Historische Grenzen

Die folgenden Grenzen sind ueber die belegten Laufzeit-Staende hinweg konsistent.

### Kein echter Helligkeitsregler

Historisch wird nicht versucht, den Lichtkopf auf einen Soll-Luxwert einzuregeln.

Der Lichtkopf wird eingeschaltet und laeuft mit festem Ausgang. Die Regelung entscheidet nur, wann Schluss ist.

### Kein spektraler Regler

Es gibt keine Rueckkopplung pro Spektralkanal und keine laufende Korrektur von Gruen oder Blau.

### Kein papierdichtegefuehrter Regler

Die Regelung sieht keine echte fotografische Zielgroesse auf dem Papier.

Sie regelt auf Sensor-Lux-Zeit. Die Abbildung auf das Papierergebnis erfolgt ueber vorgelagerte Kalibrierung und Profile.

### Keine integrierte Kopfkalibrierung im Regelkreis

Die historische Closed-Loop-Engine kennt keine eigene Schicht:

- `Soll-Spektrum -> kalibrierter Lichtkopf -> Sensorantwort`

Damit bleiben Kopfvarianten, LED-Aging und spektrale Verschiebungen nur indirekt ueber Papier- und Zeitkalibrierung kompensiert.

### Abhaengigkeit von lokalem Dosis-Sensor

Die historische Laufzeitregelung ist lokal sensorgebunden.

Funk und Spotmessung sind Hilfspfade, aber keine vollwertigen Ersatz-Regelpfade fuer die Belichtung selbst.

### Fehlender Fail-Safe bei Sensorverlust im Lauf

Die groesste historische Landmine ist der fehlende harte Fail-Safe, wenn der Sensor waehrend einer laufenden Dosisbelichtung keine gueltigen Events mehr liefert.

### Sensorlatenz und Quantisierung

Die Regelung bleibt immer begrenzt durch:

- Integrationszeit des Sensors
- Polling- oder Task-Periode
- Rechen- und Buslatenz
- Ausgangslatenz des Lichtkopfs

`v0.9` verbessert diesen Punkt gegenueber `v0.3`, beseitigt ihn aber nicht grundsaetzlich.

## Vergleich der historischen Staende

| Stand | Laufzeit-Closed-Loop | Messgroesse | Sensorpfad | Stellgroesse | Historischer Befund |
| --- | --- | --- | --- | --- | --- |
| `2.30` | nicht klar belegt | Lux und K-Faktoren | TSL2591-Messpfad | Zeit- oder Dosisvorbereitung | konzeptionelle Vorstufe |
| `2.240` | nicht gleichwertig belegt | Lux fuer Zeitableitung | Messpfad fuer Kalibrierung | vorab berechnete Zeit | messgestuetzte Vorwaertsregelung |
| `B1.0` | nicht gleichwertig belegt | wie `2.240` plus Smart Math | kein neuer Regler belegt | vor allem Rechenverfeinerung | Mathematik-Iteration |
| `v0.3` | klar belegt | laufendes Head-Lux | lokaler TSL2561 auf schnellem Bus | praeziser Shutoff | erste echte Runtime-Dosisregelung |
| `v0.9` | klar belegt | laufendes Head-Lux plus Live-Dosis | TSL2561 auf I2C1, Messsensoren separat | praeziser Shutoff mit Telemetrie | gleiche Regelidee, deutlich sauberer organisiert |

## Historische Gesamtbewertung

Die historische Closed-Loop-Regelung im Dukatimer ist fachlich ernstzunehmen, aber sie muss korrekt benannt werden.

Sie ist historisch:

- eine Dosisregelung
- eine Abschaltregelung
- eine luxbasierte Rueckfuehrung
- keine Helligkeits-, Spektral- oder Papierdichte-Regelung

Die grossen Staerken der Altstaende liegen in:

- der praktischen Nutzbarkeit
- der Wiederverwendung fuer mehrere Dunkelkammermodi
- dem fruehen Verstaendnis fuer reale Hardwarelatenzen

Die grossen Grenzen liegen in:

- fehlender Kopfkalibrierung im Regelkreis
- fehlender spektraler Rueckfuehrung
- lokalem Sensorzwang
- fehlendem robusten Runtime-Fail-Safe bei Sensorausfall

## Konsequenz fuer Part2

Fuer Part2 folgt daraus eine klare Lehre.

Historisch sollte uebernommen werden:

- Dosis als fuehrende Regelgroesse
- laufende Integration real gemessener Lichtleistung
- praeziser Shutoff mit definierter Latenzkompensation
- Pre-Wait und Post-Wait als Teil der Engine

Historisch sollte nicht blind uebernommen werden:

- die Vermischung von Belichtungsregelung und Kopfkalibrierung
- die Annahme, dass Lux am Sensor schon die fotografische Wahrheit ist
- das Fehlen eines harten Fehlermodus bei Sensorverlust im Lauf
- die fehlende Trennung zwischen Laufzeitregler, Spektrumsoll und Kopfmodell

Die sauberste Schlussfolgerung ist deshalb:

- Part2 braucht weiterhin eine dosisgeregelte Closed-Loop-ExposureEngine
- diese Engine darf aber nur eine Schicht in einer groesseren Pipeline sein
- Kopfkalibrierung, Soll-Spektrum und papierbezogene Zielbildung muessen getrennt modelliert werden
