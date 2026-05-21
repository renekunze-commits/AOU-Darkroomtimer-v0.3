# Dukatimer historische Funktionsanalyse

## Ziel

Dieses Dokument vergleicht drei fruehe Arduino-Zwischenstaende mit dem spaeteren Stand in Dukatimer v0.3, der bereinigten Architektur in Dukatimer v0.9 und dem aktuellen Zielbild von Dukatimer-Part2.

Analysierte Zwischenstaende:

- `2.30 (Densi_AVG)`
- `2.240 (Probestreifen)`
- `B1.0 (Probestreifen + Smart Math)`

## Kurzfazit

Die fruehen Zwischenstaende zeigen keine fachlich falsche Richtung, sondern die Entstehung der spaeteren Kernfunktionen. Die eigentliche Entwicklung verlaeuft in drei Schritten:

1. `2.30` fuehrt den Densitometer-/Filmtest-Strang als eigenstaendige Arbeitsweise ein.
2. `2.240` erweitert das Geraet zum vollwertigen Dunkelkammer-Werkzeug mit Probestreifen, Preflash und slot-gebundenen Papierprofilen.
3. `B1.0` ersetzt vor allem Rechenlogik fuer Splitgrade, ist aber funktional eher eine Verfeinerung von `2.240` als eine neue Plattform.

Der heutige Stand bestaetigt das:

- Dukatimer `v0.3` enthaelt die groesste fachliche Tiefe und ist das wichtigste Verhaltens-Referenzsystem.
- Dukatimer `v0.9` ist die bessere Software-Basis fuer eine Neuimplementierung.
- Dukatimer-Part2 besitzt derzeit noch keine Firmware-Umsetzung; die Luecke ist also Implementierung, nicht Konzept.

## Entwicklungsstufen

### 2.30

Schwerpunkt dieser Version ist der Densitometer-/Filmtest-Assistent.

Enthaltene Kernelemente:

- Grundmodus `SG`, `BW`, `DENS`
- Densitometer-Zustaende `REF`, `BASE`, `MEAS`
- Zone-I- und Zone-VIII-Helfer
- Toggle-Einstieg in den Densitometer per `Shift + D`
- Setup-Einstieg `FILM TEST`
- Papierprofile mit Grund-K-Faktoren `Ksoft`, `Khard`, `Kbw`
- `useDoseMode` bereits im Settings-Modell vorhanden
- TSL2591 als aktive Messbasis

Einordnung:

`2.30` ist der erste klar erkennbare Stand, in dem der Dukatimer mehr als ein Belichtungstimer ist. Der Schwerpunkt liegt noch nicht auf komplexen Assistenzfunktionen fuer Belichtungsaufbau, sondern auf Mess- und Einmess-Workflows.

### 2.240

Diese Version ist der eigentliche funktionale Ausbau zum spaeteren System.

Neu oder deutlich erweitert gegenueber `2.30`:

- additiver Probestreifen-Modus via `Shift + C`
- eigener `TSState` mit Setup-, Lauf- und Abschlusszustand
- Preflash fuer `BW` und `SG`
- Preflash-Wizard mit Zeitgrenzen unterhalb des normalen Timer-Minimums
- slot-gebundene Preflash-Parameter im `PaperProfile`
- Flash-Kalibrierung und Slot-Storage
- robusteres Mess-UI mit Mittelung mehrerer Spotmessungen
- Densitometer bleibt erhalten
- Kalibrierungs-Wizard bleibt erhalten

Einordnung:

`2.240` ist der erste Stand, in dem fast alle spaeter als produktrelevant wahrgenommenen Dunkelkammer-Funktionen gemeinsam auftreten: Papierprofil, Kalibrierung, Messung, Probestreifen und Preflash.

### B1.0

`B1.0` ist funktional nah an `2.240`, setzt aber eine veraenderte Splitgrade-Rechenlogik oben drauf.

Gegenueber `2.240` neu oder betont:

- integrierte `Smart Math`-Formel fuer Splitgrade
- Probestreifen bleibt explizit erhalten
- Preflash-Struktur bleibt erhalten
- Kalibrierung und Densitometer bleiben erhalten
- TSL2561 fuer Papier-/Head-Messung ist vorbereitet, aber noch nicht aktiviert

Einordnung:

`B1.0` ist eher eine Rechen-Iteration als ein funktionaler Neubeginn. Die wichtigen Bedien- und Workflow-Funktionen sind bereits in `2.240` vorhanden; `B1.0` justiert vor allem die SG-Zeitbildung.

## Abgleich mit heutigem Stand

| Funktionsbereich | 2.30 | 2.240 | B1.0 | v0.3 | v0.9 | Part2 aktuell |
| --- | --- | --- | --- | --- | --- | --- |
| BW/SG-Grundtimer | vorhanden | vorhanden | vorhanden | deutlich erweitert | architektonisch sauberer | nicht implementiert |
| Dose/Time-Umschaltung | im Settings-Modell sichtbar | vorhanden | vorhanden | vorhanden | vorhanden | nicht implementiert |
| Densitometer/Filmtest | eingefuehrt | vorhanden | vorhanden | funktional ausgebaut | prinzipiell uebernehmbar | nicht implementiert |
| Kalibrierungs-Wizard | vorhanden | vorhanden | vorhanden | vorhanden | vorhanden | nicht implementiert |
| Probestreifen | noch nicht vorhanden | eingefuehrt | erhalten | vorhanden | als Zielumfang relevant | nicht implementiert |
| Preflash | noch nicht vorhanden | eingefuehrt | erhalten | vorhanden | als Zielumfang relevant | nicht implementiert |
| Slot-gebundene Papierprofile | einfach | erweitert um Flash-Parameter | erhalten | deutlich erweitert | gut portierbar | nicht implementiert |
| Multi-Spot/Mittelung | einfach | robuster | robuster | stark ausgebaut bis Histogramm/Session | strukturell gut aufnehmbar | nicht implementiert |
| Smart Splitgrade-Rechnung | noch nicht | noch nicht | eingefuehrt | spaeter durch komplexere Logik ueberholt | als Regelwerk sauber kapselbar | nicht implementiert |
| Zusatzsensorik Head/Papier | kaum | vorbereitet | vorbereitet | spaeter deutlich breiter | abstrahierbar | offen |
| UI-Modell | 16x2-LCD + Tasten | 16x2-LCD + Tasten | 16x2-LCD + Tasten | spaeter hardwaregekoppelt erweitert | besser entkoppelt | muss neu entworfen werden |

## Was im heutigen Stand erhalten geblieben ist

Die fruehen Zwischenstaende wurden fachlich nicht verworfen. Die meisten Funktionen sind in den spaeteren Versionen erhalten geblieben oder wurden ausgebaut.

Erkennbar weitergetragen:

- Grundidee von `SG`, `BW` und einem separaten Mess-/Assistentenmodus
- Papierslot-Modell mit persistenten Parametern
- Trennung zwischen Messung, Kalibrierung und eigentlicher Belichtung
- Dose-/Zeit-Logik als umschaltbare Betriebsart
- Dunkelkammer-spezifische Hilfen wie Probestreifen und Preflash

## Was spaeter erweitert oder ersetzt wurde

Im Vergleich zu den Arduino-Zwischenstaenden verschiebt sich der Schwerpunkt spaeter deutlich.

In `v0.3` kommt hinzu:

- deutlich mehr Sensor- und Messlogik
- Mess-Sessions mit Verlauf und Histogramm
- weitergehende SG-Unterstuetzung inklusive Vorschlagslogik
- staerkere Hardwarekopplung, Tasks und Nebenlaeufigkeit
- spaetere Einbindung externer oder drahtloser Messkomponenten

In `v0.9` wird vor allem die Struktur verbessert:

- `SystemContext` als gemeinsame Zustandsbasis
- Manager-/App-Architektur statt monolithischer Ablaufsteuerung
- bessere Trennung von Hardware, UI, Logik und Persistenz
- geeignetere Basis fuer Part2 als die direkte Fortsetzung der fruehen Sketche

## Gegen den aktuellen Part2-Stand abgeglichen

Fachlich fehlt in Part2 aktuell nichts an Zielbild, aber praktisch noch fast alles an Umsetzung.

Der Abstand zwischen den fruehen Zwischenstaenden und Part2 ist deshalb kein Funktionsverlust, sondern eine noch offene Neuimplementierung. Besonders relevant fuer Part2 sind:

- deterministische Belichtungslogik auf dem Teensy
- neues UI fuer Kalibrierung, Densitometer, Probestreifen und Preflash
- persistentes Papiermodell inklusive Flash-Parametern
- saubere Sensorabstraktion fuer lokale und drahtlose Messkoepfe
- Inter-MCU-Protokoll zwischen Teensy und ESP32-S3

## Bewertung pro Altfunktion fuer Part2

### Muss uebernommen werden

- BW- und SG-Belichtung
- Dose-/Zeit-Modell
- fotografische EV-/F-Stop-Bedienlogik als querschnittliche Belichtungswert-Schicht
- Kalibrierungs-Wizard
- Densitometer-/Filmtest-Workflow
- Probestreifen
- Preflash inklusive slot-gebundener Parameter
- persistente Papierprofile

### Sollte als Verhalten erhalten bleiben, aber nicht 1:1 im alten UI

- Tastenkuerzel wie `Shift + C` oder `Shift + D`
- 16x2-spezifische Overlay-Logik
- alte Setup-Menuestruktur

Diese Dinge sind fachlich wichtig, aber die konkrete Bedienform muss fuer TFT/Touch und die neue Part2-Architektur neu gedacht werden.

Fuer Part2 folgt daraus zusaetzlich:

- echte Messworkflows sollen Messwerte nicht nur roh, sondern zusaetzlich in EV ausgeben, wenn die fotografische Einordnung sinnvoll ist
- Rohtelemetrie ohne fotografische Aussage bleibt in Lux oder der physischen Grundeinheit
- mehrfach benoetigte Logik wie EV-Schrittbildung, Messwertumrechnung und Formatter wird nicht pro Modus neu angelegt

### Sollte nicht blind kopiert werden

- die `Smart Math`-Formel aus `B1.0` als alleinige SG-Wahrheit
- AVR-/EEPROM-nahe Persistenzdetails
- UI-Limitierungen aus der 14-/16-Zeichen-LCD-Welt

Hier ist der bessere Weg: Verhalten fachlich pruefen, dann in der `v0.9`-Architektur und mit `v0.3` als Referenz neu umsetzen.

## Gesamtbewertung

Die drei Zwischenstaende zeigen klar, welche Funktionen historisch gewachsen und damit fuer das Produkt wesentlich sind. Besonders ab `2.240` ist der spaetere Zielumfang bereits gut erkennbar.

Fuer Part2 bedeutet das:

- `2.30` ist wichtig, um den Ursprung des Densitometer-/Filmtest-Workflows zu verstehen.
- `2.240` ist historisch der wichtigste Funktionsmeilenstein.
- `B1.0` ist vor allem fuer die Entwicklung der SG-Rechenlogik relevant.
- fuer die Neuimplementierung bleibt `v0.9` die beste Struktur-Basis.
- fuer Detailverhalten und Vollstaendigkeit bleibt `v0.3` die wichtigste Referenz.

Damit ist der aktuelle Stand konsistent: Die fruehen Funktionen sind nicht verloren gegangen, sondern spaeter erweitert worden. Offen ist heute hauptsaechlich die technische Rueckfuehrung dieser Funktionen in die neue Part2-Hardwarearchitektur.
