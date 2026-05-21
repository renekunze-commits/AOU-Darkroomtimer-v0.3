# Dukatimer-Part2 Laufzeitspezifikation fuer ExposureEngine, Closed Loop und Fail-Safe

Stand: 2026-04-24

Status:

- Schritt 2 der Part2-Lichtarchitektur
- technische Laufzeitspezifikation vor der eigentlichen Implementierung
- bewusst getrennt vom Zielmodell fuer Spektrum, Kopfkalibrierung und Papierprofil

## Versionsstand und Zweck

### Dokumentversion

- Revision A
- erstellt am 2026-04-24
- Grundlage: Zielmodell, Pflichtenheft und historische Closed-Loop-Analyse

### Zweck

Dieses Dokument beschreibt die operative Laufzeitarchitektur fuer die Belichtungsengine von Dukatimer-Part2.

Es beantwortet insbesondere diese Fragen:

- welcher Dienst besitzt welchen Sensorpfad
- wie die Dosisintegration zur Laufzeit ablaufen soll
- wie der praediktive Shutoff mit NeoPixel-Latenz funktioniert
- wie thermische Drosselung und thermischer Hard-Stop eingebunden werden
- wie Sensorfehler und unplausible Messwerte fail-safe behandelt werden
- welche Telemetrie lokal sichtbar sein muss

Dieses Dokument ist absichtlich kein Ersatz fuer das Drei-Schichten-Zielmodell.

Die Trennung lautet daher explizit:

- `HeadSpectrumCommand`, `HeadCalibrationProfile` und `PaperExposureProfile` beschreiben das fachliche und persistente Zielmodell
- dieses Dokument beschreibt die Laufzeitdienste, die dieses Zielmodell ausfuehren und absichern

## Architekturkontext

### Einordnung in die Gesamtpipeline

Die Part2-Lichtarchitektur besteht aus zwei Ebenen der Verantwortung.

#### Persistente und fachliche Ebene

- `HeadSpectrumCommand`
- `HeadCalibrationProfile`
- `PaperExposureProfile`

#### Laufzeit- und Sicherheitsdienste

- `ExposureEngine`
- `SensorManager`
- `HeadDriveMapper` oder eine gleichwertige Mapping-Schicht
- lokale Schutz- und Abschaltpfade
- UI-Telemetrie auf dem Teensy

Die Regel lautet:

- das Zielmodell entscheidet, welches Licht fachlich gemeint ist
- die Laufzeitdienste entscheiden, wie dieses Licht sicher und reproduzierbar ausgefuehrt wird

### Abgrenzung zur Historie

Historisch war Closed Loop vor allem eine Dosisregelung mit sensorgefuehrtem Abschaltzeitpunkt.

Part2 uebernimmt dieses Prinzip, erweitert es aber um zwei Punkte, die historisch nicht sauber geloest waren:

- Sensor-Watchdog waehrend laufender Dosisbelichtung
- thermische Drosselung und thermischer Hard-Stop als explizite Laufzeitfunktion

## Laufzeitdienste und Besitzverhaeltnisse

### ExposureEngine

Die `ExposureEngine` ist in Part2 die einzige Autoritaet fuer den aktiven Belichtungsablauf.

Sie besitzt zur Laufzeit insbesondere:

- den Belichtungszustand
- das aktuelle Sollziel als Zeit oder Dosis
- die aktuelle Ist-Dosis
- die Steuerung des aktiven Lichtauftrags waehrend `EXPOSING`
- den praediktiven Shutoff
- die Entscheidung ueber Pause, Resume, Stop und Emergency Shutoff

Die `ExposureEngine` ist ausdruecklich nicht verantwortlich fuer:

- Papierprofilpersistenz
- LVGL-Screen-Rendering
- direkte Definition der fachlichen Sollsemantik
- globale Storage- oder Kommunikationspolitik

### SensorManager

Der `SensorManager` ist die einzige Autoritaet fuer Sensorzugriffe und Sensorgueltigkeit.

Er besitzt mindestens diese Pfade:

- lokalen `TSL2561` fuer den Dosis-Regelkreis
- lokalen `DS18B20` fuer die Kuehlkoerpertemperatur
- optionale weitere Umgebungs- und Hilfssensorik
- optionale Wireless- oder Remote-Messpfade ausserhalb des harten Dosis-Regelkreises

Die Trennung bleibt bindend:

- der lokale `TSL2561` gehoert zum harten Closed-Loop-Belichtungspfad
- externe oder drahtlose Sensorik ist kein Ersatz fuer diesen Pfad

### Mapping-Schicht zwischen Fachmodell und Kopf

Zwischen `ExposureEngine` und `NeoPixelHead` liegt eine eigene Mapping-Schicht.

Diese Schicht nimmt entgegen:

- `HeadSpectrumCommand`
- `HeadCalibrationProfile`
- Laufzeitlimits wie thermische Drosselung

und erzeugt daraus:

- den konkreten low-level Ausgabebefehl fuer den Lichtkopf

Damit bleibt gewahrt:

- die `ExposureEngine` regelt die Dosis
- die Mapping-Schicht regelt nicht die Dosis, sondern nur die hardwaregerechte Umsetzung des Lichtauftrags

## Sensorik und harte Regeln

### TSL2561 als lokaler Dosis-Sensor

Der `TSL2561` ist der ausschliessliche Sensor fuer die Echtzeit-Dosisintegration.

Er muss lokal am Hauptsystem angeschlossen sein und darf nicht von externen Verbindungen abhaengen.

Seine Pflichtaufgaben sind:

- kontinuierliche Lieferung plausibler Lux-Werte waehrend aktiver Belichtung
- Bereitstellung einer Gueltigkeitsaussage pro Sample
- Meldung von Stale-, Timeout- oder Fehlerzustaenden an die `ExposureEngine`

### DS18B20 als lokaler Thermik-Sensor

Der `DS18B20` ueberwacht den LED-Kuehlkoerper.

Seine Pflichtaufgaben sind:

- Bereitstellung einer aktuellen Kuehlkoerpertemperatur
- Meldung plausibler oder unplausibler Werte
- Unterstuetzung von zwei Schwellen:
  - Drosselschwelle oberhalb `50 C`
  - Abschaltschwelle oberhalb `60 C`

### Optionale Sensorik ausserhalb des harten Belichtungskreises

Weitere Sensorik kann vorhanden sein, zum Beispiel:

- Wireless-Messgeraet
- Umgebungshelligkeit
- Raum- oder Zusatztemperatur

Diese Pfade koennen fuer UI, Diagnose oder Arbeitsablaeufe nuetzlich sein, sind aber nicht Teil des zwingenden lokalen Dosisregelkreises.

## Zustandsmaschine der ExposureEngine

### Verbindliche Hauptzustaende

Die ExposureEngine muss mindestens die folgenden Hauptzustaende besitzen:

- `IDLE`
- `PRE_WAIT`
- `EXPOSING`
- `PAUSED`
- `POST_WAIT`
- `DONE`
- `FAULT`

Der Zustand `FAULT` ist fuer Part2 bewusst zusaetzlich vorgesehen.

Er loest die historische Schwachstelle auf, bei Fehlern nur still in einen unklaren Zustand zu fallen.

### Bedeutung der Zustaende

#### IDLE

- keine aktive Belichtung
- kein aktiver Dosis-Shutoff-Timer
- Lichtkopf in sicherem Ruhezustand

#### PRE_WAIT

- Safelight aus
- Relais oder Ausgaenge in stabilem Vorzustand
- noch keine aktive Dosisintegration
- Einstieg in die Belichtung erst nach abgelaufener Ruhephase

#### EXPOSING

- aktiver Lichtauftrag am Kopf
- laufende Dosisintegration im Dosisbetrieb
- aktive Temperatur- und Sensorueberwachung
- aktiver oder vorbereiteter praediktiver Shutoff

#### PAUSED

- Lichtkopf aus
- Sollzustand eingefroren
- Dosisintegration gestoppt
- Wiederanlauf nur ueber explizites Resume

#### POST_WAIT

- Lichtkopf aus
- definierte Nachlaufphase zum Abklingen moeglicher Restemissionen
- kein neuer Lichtauftrag in derselben Belichtung ohne erneuten Start

#### DONE

- Belichtung fachlich erfolgreich abgeschlossen
- Telemetrie und Abschlussgrund noch sichtbar
- Rueckkehr nach `IDLE` erst ueber explizite Quittierung oder naechsten Arbeitsablauf

#### FAULT

- Belichtung wurde fail-safe beendet
- Lichtkopf ist lokal abgeschaltet
- Fehlergrund ist gelatcht und im UI sichtbar
- Rueckkehr in `IDLE` nur ueber explizite Fehlerquittierung

### Hauptuebergaenge

```text
IDLE
  -> PRE_WAIT
  -> FAULT     bei bereits erkanntem Startfehler

PRE_WAIT
  -> EXPOSING  nach definierter Ruhezeit
  -> IDLE      bei Benutzerabbruch vor Lichtbeginn
  -> FAULT     bei Sensor- oder Schutzfehler

EXPOSING
  -> POST_WAIT bei regulaerem Belichtungsende
  -> PAUSED    bei Pause-Anforderung
  -> FAULT     bei Sensorfehler, unplausiblem Licht, Thermik-Hardstop

PAUSED
  -> EXPOSING  bei Resume
  -> IDLE      bei Abbruch ohne Fortsetzung
  -> FAULT     bei Schutzfehler waehrend des Haltezustands

POST_WAIT
  -> DONE      bei regulaerem Ende
  -> FAULT     falls in dieser Phase noch ein sicherheitskritischer Fehler erkannt wird

DONE
  -> IDLE      nach Quittierung

FAULT
  -> IDLE      nach Quittierung und sicherer Freigabe
```

## Closed-Loop-Dosisregelung

### Fuehrungs- und Messgroesse

Die Fuehrungsgroesse im Dosisbetrieb ist:

- `targetDose`

Die Messgroesse im Regelkreis ist:

- lokal gemessenes `lux`

Die Ist-Groesse lautet:

- `currentDose`

mit der Laufzeitbildung:

```text
currentDose += measuredLux * dtSeconds
```

### Grundalgorithmus fuer den Dosisbetrieb

Der Dosisbetrieb folgt diesem Ablauf:

1. `targetDose` aus Papiermodell oder Arbeitsablauf uebernehmen.
2. `PRE_WAIT` abarbeiten.
3. Lichtauftrag aktivieren.
4. lokale Lux-Samples laufend lesen.
5. Ist-Dosis ueber `lux * dt` integrieren.
6. Rest-Dosis und Restzeit laufend abschaetzen.
7. praediktiven Shutoff rechtzeitig ausloesen.
8. nach Licht-Aus `POST_WAIT` abarbeiten.
9. Erfolg in `DONE` oder Fehler in `FAULT` melden.

Wichtige Trennung fuer Part2:

- `ExposureEngine` bleibt intern bei Lux, Dosis und Zeit als physikalischen Laufzeitgroessen
- fotografische EV-Werte fuer Densitometrie, Kalibrierung oder andere echte Messworkflows werden nur zusaetzlich in einer gemeinsamen Darstellungs- oder Math-Schicht abgeleitet
- reine Laufzeittelemetrie wie Head-Lux, Restzeit oder Buslatenz wird nicht kuenstlich in EV uebersetzt

### Nominale Task- und Sample-Logik

Die Belichtungslaufzeit soll ueber einen festen, kurzen Laufzeittick beobachtet werden.

Das Zielbild fuer Part2 ist:

- kurzer Engine- oder Closed-Loop-Tick fuer den Regelpfad
- Sensorintegration und echte Sample-Frische werden getrennt betrachtet
- `dtSeconds` wird aus echter Zeitdifferenz zwischen gueltigen Samples gebildet

Die Engine darf also nicht annehmen, dass jeder Tickschritt automatisch einem neuen Sensormesswert entspricht.

### Sample-Gueltigkeit

Ein Lux-Sample ist nur dann fuer die Dosisintegration gueltig, wenn gleichzeitig gilt:

- Sensorantwort formal erfolgreich
- Sample zeitlich frisch genug
- Sample physikalisch plausibel
- Sample nicht aus einem deaktivierten oder uninitialisierten Sensorzustand

Ungueltige Samples duerfen nicht in die Ist-Dosis eingehen.

## Praediktiver Shutoff

### Ziel des Shutoff-Pfads

Der praediktive Shutoff kompensiert die nichtverschwindende Reaktionszeit des Kopfpfads.

Dazu gehoeren insbesondere:

- NeoPixel-Bus- oder Latch-Latenz
- interne Verarbeitungszeit bis zum wirksamen Licht-Aus

### Verbindliche Systemgroesse

Fuer Part2 ist eine Ausgangslatenz in der Groessenordnung von rund `8 ms` als explizite Systemgroesse anzunehmen und in der Laufzeitlogik sichtbar zu halten.

Diese Groesse darf nicht stillschweigend in einen Treiber verschwinden, weil sie fuer die reproduzierbare Ziel-Dosis relevant ist.

### Algorithmischer Ablauf

Sobald die Rest-Dosis klein genug ist, bestimmt die Engine:

- `remainingDose = targetDose - currentDose`
- `remainingTime = remainingDose / measuredLux`

Danach wird der Abschaltbefehl vorgezogen um:

- die bekannte Kopf- oder Buslatenz

Die praktische Regel lautet:

- hohe Rest-Dosis: weiter integrieren
- kleine Rest-Dosis: praediktive Restzeit berechnen
- minimale Rest-Dosis oder Ueberschreitung: sofort hart abschalten

### Randfaelle

Der prädiktive Shutoff muss mindestens diese Randfaelle behandeln:

- `measuredLux <= 0`
- Rest-Dosis bereits `<= 0`
- Latenz groesser als berechnete Restzeit
- Timer bereits aktiv oder schon ausgeloest

## Thermische Kompensation

### Grundprinzip

Thermische Kompensation wird nicht ueber das Papiermodell geloest, sondern als Laufzeitfunktion des Kopfpfads.

Das Grundprinzip lautet:

- oberhalb `50 C` wird die maximal erlaubte Kopfleistung reduziert
- die reale Lichtleistung sinkt dadurch
- die Dosisregelung erkennt dies am kleineren Luxwert
- die Belichtungsdauer verlaengert sich automatisch, bis die Ziel-Dosis erreicht ist

### Drosselpfad

Die Drosselung soll als begrenzender Laufzeitfaktor in die Mapping-Schicht eingehen, zum Beispiel als:

- `runtimeOutputLimit`

Dieser Faktor wird auf die durch `HeadCalibrationProfile` berechnete Ausgabe angewendet.

Wichtig ist:

- der fachliche Sollwert aendert sich nicht
- nur die maximal erlaubte reale Ansteuerung wird reduziert

### Kennlinie der Drosselung

Die exakte Derating-Kennlinie wird in dieser Revision noch nicht festgeschrieben.

Verbindlich ist aber:

- unterhalb oder gleich `50 C` keine thermische Drosselung
- oberhalb `50 C` monoton fallende erlaubte Kopfleistung
- bei oder oberhalb `60 C` kein weiterer Betrieb, sondern Emergency Shutoff

Damit bleibt Raum fuer spaetere Mess- oder Haltbarkeitsdaten, ohne den Schutzpfad offen zu lassen.

## Fail-Safe und Watchdogs

### Sensor-Watchdog fuer den TSL2561

Sobald waehrend `EXPOSING` ein nicht-offener Lichtauftrag aktiv ist, muss die Engine den Dosis-Sensor aktiv ueberwachen.

Ein fail-safe relevanter Fehler liegt mindestens vor, wenn:

- fuer eine definierte Zeit keine frischen gueltigen Samples eintreffen
- der Sensor einen formalen Fehler meldet
- aktiv Licht angefordert ist, aber der Sensor dauerhaft `0 Lux` oder einen unplausibel niedrigen Wert liefert

### Plausibilitaetsregel fuer Null-Lux unter aktivem Licht

Die Plausibilitaetspruefung darf keine Sofortfehler bei Einschalttransienten produzieren.

Deshalb gilt:

- direkt nach dem Einschalten gibt es ein kurzes Beobachtungsfenster
- erst wenn ueber dieses Fenster hinaus unter aktivem Licht nur `0 Lux` oder physikalisch unplausible Werte anliegen
- wird ein Sensorfehler gelatcht

Die konkrete Fensterlaenge bleibt implementierungsseitig parametrierbar, muss aber groesser sein als Anlauf- und Busjitter.

### Thermal Hard Stop

Sobald der `DS18B20` eine kritische Temperatur bei oder oberhalb `60 C` meldet, gilt zwingend:

- sofortiges Licht-Aus
- laufende Belichtung verlassen
- Fehlergrund speichern
- in `FAULT` wechseln

### Prioritaetsregel bei Fail-Safe-Ereignissen

Die Sicherheitsprioritaet lautet:

1. thermischer Hard-Stop
2. lokaler Sensorverlust oder unplausibles Licht unter aktivem Kopf
3. normaler Benutzerabbruch
4. regulaerer Belichtungsabschluss

Damit wird sichergestellt, dass Schutzreaktionen nicht von Komfortpfaden ueberlagert werden.

### Emergency-Shutoff-Verhalten

Ein Emergency Shutoff muss lokal und deterministisch wirken.

Er umfasst mindestens:

- aktive Kopf-Ausgabe sofort beenden
- weitere Belichtungsbefehle blockieren
- laufende Timer oder prädiktive Shutoff-Arms disarmen
- Fehlergrund latched speichern
- UI-Telemetrie fuer die Fehlerursache aktualisieren

## Telemetrie und UI-Anforderungen

### Pflichttelemetrie waehrend aktiver Belichtung

Die UI auf dem Teensy muss waehrend der Belichtung mindestens sichtbar machen:

- aktiven State-Machine-Zustand
- Ziel-Dosis oder Zielzeit
- aktuelle Ist-Dosis
- verbleibende Rest-Dosis oder Restzeit
- aktuellen gemessenen Luxwert
- aktuelle Kuehlkoerpertemperatur
- thermische Drosselung aktiv oder inaktiv

### Pflichttelemetrie bei Fehlern

Im Fehlerfall muss lokal sichtbar sein:

- dass ein Fail-Safe-Ereignis vorliegt
- welcher Fehlergrund aktiv ist
- ob der Fehler sensorisch oder thermisch verursacht wurde
- ob eine Quittierung erforderlich ist

### Rolle von LVGL und EEZ-Studio

`LVGL` und `EEZ Studio` dienen der klaren Bedienfuehrung und Beobachtung, nicht der eigentlichen Sicherheitsentscheidung.

Damit gilt:

- Schutzreaktionen muessen ohne UI funktionieren
- UI-Ausfall darf die lokale Abschaltfaehigkeit nicht verhindern

## Nicht-Ziele dieser Revision

Diese Spezifikation legt noch nicht fest:

- die exakte Klassen- oder Dateistruktur der spaeteren C++-Implementierung
- die finale Thread- oder Core-Aufteilung
- die konkrete mathematische Form der thermischen Derating-Kurve
- die finale Parametrierung aller Timeouts und Beobachtungsfenster
- spaetere Segment- oder Uniformitaetskorrektur des Kopfes

Diese Punkte bleiben offen, ohne die Schutz- und Ablaufarchitektur in Frage zu stellen.

## Minimale Umsetzungsschritte nach dieser Spezifikation

### Schritt 2a

- Laufzeitstatusmodell fuer ExposureEngine mit `IDLE`, `PRE_WAIT`, `EXPOSING`, `PAUSED`, `POST_WAIT`, `DONE`, `FAULT` anlegen

### Schritt 2b

- Sensorzustandsmodell fuer `TSL2561` und `DS18B20` anlegen
- Gueltigkeit, Frische und Fehlergruende formalisieren

### Schritt 2c

- Dosisintegration und prädiktiven Shutoff in die ExposureEngine portieren
- NeoPixel-Latenz als expliziten Korrekturwert fuehren

### Schritt 2d

- thermischen Drosselpfad als Laufzeitlimit in die Mapping-Schicht einziehen
- Hard-Stop bei `>= 60 C` verdrahten

### Schritt 2e

- Sensor-Watchdog und Emergency Shutoff lokal und deterministisch verdrahten
- Fehlertelemetrie fuer UI und Logging freigeben

## Zusammenfassung

Die operative Zielarchitektur fuer Part2 lautet:

- fachliche Sollbildung, Kopfkalibrierung und Papiermodell bleiben als drei getrennte Schichten bestehen
- die `ExposureEngine` fuehrt darauf aufbauend eine lokale, dosisgeregelte Closed-Loop-Belichtung aus
- der `TSL2561` liefert die harte Rueckfuehrung fuer die Ist-Dosis
- der `DS18B20` begrenzt den Kopf thermisch und erzwingt bei kritischer Temperatur den Hard-Stop
- Sensorverlust oder unplausibles Licht unter aktivem Kopf fuehren fail-safe in einen lokalen `FAULT`-Zustand

Damit entsteht fuer Part2 ein Belichtungssystem, das die historische Dosisregelung beibehalt, die historischen Landminen aber nicht erneut uebernimmt.
