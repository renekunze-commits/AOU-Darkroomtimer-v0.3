# Duka-Teen Electrical Risk Analysis

Grundlage dieser Bewertung:

- `Duka-Teen.kicad_sch` fuer die Top-Level-Kopplung
- `teensy.kicad_sch` fuer die Versorgungs- und Ausgangsstufe um den Teensy 4.1
- `ESP.kicad_sch` fuer die ESP32-S3-DevKitC-Seite, Audio/Vibra und die Boot-Leitungen

Fokus dieser Analyse:

- Versorgung ueber `+5V_EXT`
- `+3.3V` als ESP-Sheet-Hilfsschiene
- externe Kopfversorgung `+3V3_HEAD`
- Pegelgrenzen an `74HCT125`, UART- und Sensorleitungen
- Reset-/Boot-Kopplung ueber `S3_EN_RST` und `S3_GPIO0`

Wichtige Abgrenzung zur optionalen Wireless-TSL2591-Sonde:

- Die Funksonde ist kein Teil der intern verdrahteten Basisplatine und fuehrt im aktuellen Schaltplan keine zusaetzlichen Versorgungs- oder Signalleitungen in das Duka-Teen-System ein.
- Elektrisch relevant fuer die Basisplatine ist daher primaer nur die Funk-Gateway-Rolle des ESP32-S3, nicht eine direkte Einspeisung oder Sensorverdrahtung der Sonde.

Ergaenzende Detailauswertung der Puffer- und Stuetzkondensatoren:

- siehe `docs/hardware/duka-teen-power-rail-capacitor-analysis.md`

## Kurzbewertung

| Bereich | Bewertung | Kernaussage |
| --- | --- | --- |
| Versorgung | Mittel bis hoch | Kein direkter 3,3-V-Reglerkampf der Dev-Kits, aber gemeinsame `+5V_EXT` und eine leicht missverstaendliche `+3.3V`-Hilfsschiene im ESP-Sheet. |
| Pegel | Mittel bis hoch | Die 5-V-Ausgangsstufen sind sinnvoll, verlagern aber Rueckspeise- und Hot-Plug-Risiken an die externen Anschluesse. |
| Reset/Boot | Hoch | Der Teensy treibt `CHIP_PU` und `GPIO0` des ESP direkt und kann damit den Startzustand des ESP dominieren. |

## 1. Versorgung

### 1.1 Gemeinsame 5-V-Schiene `+5V_EXT`

Befund:

- Im Top-Level kommt `+5V_EXT` aus dem Teensy-Sheet und geht in das ESP-Sheet.
- `U1` wird an `VIN` aus `+5V_EXT` gespeist.
- `U2` wird an seinem `5V`-Pin ebenfalls aus `+5V_EXT` gespeist.
- Im Teensy-Sheet ist mit `F3` nur ein klar erkennbares serielles Schutzelement im 5-V-Pfad sichtbar.
- Im ESP-Sheet taucht `+5V_EXT` zusaetzlich im Service-/USB-Bereich auf.

Risiko:

- Alle grossen Verbraucher haengen an einer gemeinsamen 5-V-Schiene. Ein Einbruch durch Display, NeoPixel, Audio, Vibra oder externe Peripherie kann beide Controller gleichzeitig stoeren oder resetten.
- Ohne saubere Entkopplung ist auch Rueckspeisung ueber Service- oder Zusatzanschluesse mitzudenken.
- Ein einzelnes Schutzelement im gemeinsamen Eingangspfad reduziert Kurzschlussenergie, trennt aber die Lastzweige nicht gegeneinander.

Bewertung: mittel bis hoch.

Empfehlung:

1. Die 5-V-Verteilung in Lastzweige aufteilen: Logik, externe Ausgaenge, Kopf/Peripherie.
2. Nahe an beiden Dev-Kits lokale Bulk-Kapazitaet vorsehen.
3. Fuer Service-/Hilfsanschluesse klar festlegen, ob dort 5 V eingespeist werden duerfen; falls nein, Rueckspeiseschutz vorsehen.
4. Wenn externe LED- oder Aktorlasten nennenswert Strom ziehen, diese nicht ungefiltert aus derselben Logikschiene speisen.

### 1.2 `+3.3V` als Hilfsschiene im ESP-Sheet

Befund:

- Im Top-Level ist `+3.3V` als Ausgang des Teensy-Sheets und als Eingang des ESP-Sheets gezeichnet.
- Im Teensy-Sheet haengen mehrere `3V3`-Pins des Teensy an dieser Schiene.
- Im ESP-Sheet existiert derselbe Netznamen ebenfalls, `U2 Pin 2 (3V3)` des `ESP32-S3-DevKitC` ist dort aber explizit mit `no_connect` markiert.
- Das ESP-DevKit selbst wird im aktuellen Plan ueber `+5V_EXT` an Pin 21 versorgt, nicht ueber Pin 2 `3V3`.

Risiko:

- Ein direkter Parallelbetrieb zweier 3,3-V-Regler ist in der vorliegenden Revision gerade nicht nachweisbar; diese fruehere Annahme war falsch.
- Das reale Risiko liegt stattdessen in einer schwer lesbaren Versorgungsabhaengigkeit: ESP-seitige Hilfsnetze koennen an `+3.3V` haengen, waehrend das ESP-DevKit selbst separat aus `+5V_EXT` gespeist wird.
- Dadurch kann der Eindruck entstehen, die komplette ESP-Seite haenge an einer gemeinsamen 3,3-V-Versorgung, obwohl tatsaechlich nur Teilnetze davon betroffen sind.

Bewertung: mittel.

Empfehlung:

1. Dokumentation und Netzbenennung so schaerfen, dass klar ist: `U2 3V3` ist unbeschaltet, `+3.3V` im ESP-Sheet ist nur eine Hilfsschiene.
2. Fuer jede ESP-seitige Baugruppe mit `+3.3V` explizit festhalten, aus welcher Quelle diese Schiene stammt.
3. Falls die Hilfsschiene dauerhaft vom Teensy kommen soll, Strombudget und Schutz fuer diesen Pfad separat betrachten.
4. Falls spaeter doch `U2 3V3` genutzt werden soll, das als eigenstaendige Designaenderung behandeln und nicht stillschweigend aus dem existierenden Sheet ableiten.

### 1.3 Kopfversorgung `+3V3_HEAD` ueber RJ45

Befund:

- Die Sensor-RJ45 fuehrt `+3V3_HEAD`, `GND`, `I2C1_SCL`, `I2C1_SDA`, `TSL_INT` und `1W_DATA` zum Kopf.
- Dieselbe Leitung transportiert also Versorgung und empfindliche Logiksignale ueber einen externen Kabelweg.

Risiko:

- Ein Kurzschluss oder ESD-Ereignis am Kopf kann die interne 3,3-V-Versorgung unmittelbar belasten.
- Spannungsabfall, Ground-Bounce und Einkopplung auf I2C/1-Wire werden mit Kabellaenge und Steckzyklen wahrscheinlicher.
- Bei Hot-Plug koennen Signale anliegen, bevor Versorgung und Masse stabil sind.

Bewertung: mittel.

Empfehlung:

1. Kopfversorgung strombegrenzt oder separat absichern.
2. ESD-/Transientenschutz direkt an der RJ45-Seite vorsehen.
3. Fuer Kopf und Kabel eine definierte Hot-Plug-Reihenfolge annehmen: zuerst Masse, dann Versorgung, dann Daten.
4. Busgeschwindigkeit und Pull-ups fuer die reale Kabellaenge validieren.

## 2. Pegel und Signalgrenzen

### 2.1 `74HCT125` als 5-V-Ausgangsstufe

Befund:

- `U3` und `U4` sind `74HCT125`-Puffer im Ausgangspfad zu `SSR_ROOM_5V` sowie `NEO_1..NEO4`.
- Das ist prinzipiell sinnvoll: Ein HCT-Eingang erkennt 3,3-V-HIGH sauber, waehrend die Ausgaenge auf 5-V-Niveau arbeiten koennen.

Risiko:

- Die Logikpegel sind zwar fuer NeoPixel- und 5-V-Aktoren passend, aber die elektrische Trennung endet damit am Stecker.
- Jede externe Ueberspannung, ESD oder Rueckspeisung auf diesen Leitungen trifft zuerst die Pufferstufe und von dort potenziell die 5-V-Schiene.
- Wenn externe Lasten eine eigene Versorgung haben oder spaeter abgeschaltet werden als das Board, sind Rueckstroeme ueber Datenleitungen moeglich.

Bewertung: mittel.

Empfehlung:

1. Datenleitungen zu externen 5-V-Lasten mit kleinen Serienwiderstaenden am Treiber ausfuehren.
2. ESD-Schutz direkt an den externen Ausgangssteckern vorsehen.
3. Externe Lasten so auslegen, dass niemals eine Fremdspannung in `NEO_*` oder `SSR_ROOM_5V` zurueckgedrueckt wird.

### 2.2 Interne Board-zu-Board-Leitungen zwischen Teensy und ESP

Befund:

- `S3_TX`, `S3_RX`, `S3_RTS`, `S3_CTS`, `S3_EN_RST` und `S3_GPIO0` liegen als direkte 3,3-V-Signale zwischen beiden Boards.
- Auch ohne direkt verbundene DevKit-`3V3`-Pins bleiben das ungeschuetzte Board-zu-Board-Leitungen zwischen zwei separat aus `+5V_EXT` versorgten Modulen.

Risiko:

- Sobald eine Seite frueher hochfaehrt, spaeter abschaltet oder brownoutet, koennen die Leitungen ueber interne Schutzstrukturen eine Teilversorgung der anderen Seite verursachen.
- Das gilt besonders dann, wenn die gemeinsame 3,3-V-Schiene spaeter getrennt wird, aber die Steuersignale unveraendert direkt bleiben.

Bewertung: mittel.

Empfehlung:

1. Auf kritischen Board-zu-Board-Leitungen kleine Serienwiderstaende vorsehen.
2. Die gueltige Power-Sequenz explizit festlegen und testen.
3. Beim spaeteren Redesign die Signalpfade so auslegen, dass eine unversorgte Seite nicht ueber I/O mitversorgt werden kann.

### 2.3 Externe Sensorbusse

Befund:

- `I2C1_*` und `1W_DATA` verlassen die Platine Richtung Sensorkopf.
- `1W_DATA` wird nur vom ESP32-S3 aktiv getrieben, aber physisch ueber die Teensy-Seite und die RJ45 gefuehrt.

Risiko:

- Die Busse sehen reale Kabelkapazitaet, ESD und potenzielle Masseverschiebungen.
- I2C ist gegen lange Kabel deutlich empfindlicher als 1-Wire; beide koennen bei unguenstiger Pull-up-Wahl oder Steckereignissen stoeren.

Bewertung: mittel.

Empfehlung:

1. Pull-up-Werte und Busfrequenzen mit realem Kopfkabel pruefen.
2. Wenn die Kabellaenge waechst, I2C-Frequenz senken oder Buspuffer pruefen.
3. Steckverbinderseite mit ESD-Schutz und sauberer Massefuehrung absichern.

## 2.4 Optionale Wireless-TSL2591-Einbindung

Befund:

- Die Wireless-TSL2591-Sonde ist funktional ein externes Handmessgeraet mit eigener MCU, eigenem Sensor, lokaler Anzeige und lokaler Eingabelogik.
- Die Kopplung an Duka-Teen erfolgt nicht ueber RJ45, nicht ueber `+5V_EXT` und nicht ueber die interne `+3.3V`-Schiene, sondern ausschliesslich per ESP-NOW zum ESP32-S3.
- Damit ist die Sonde aus Sicht der Basisplatine kein weiterer externer Kabelknoten, sondern eine optionale Funk-Peripherie.

Risiko:

- Es entstehen keine zusaetzlichen Rueckspeise- oder Hot-Plug-Risiken auf den internen Versorgungsrails der Basisplatine.
- Zusaetzliche Risiken liegen stattdessen in Verbindungsqualitaet, Heartbeat-Timeouts, Peer-Binding und im sicheren Verhalten bei ausbleibenden Mess- oder Renderpaketen.
- Wenn das System spaeter Lade- oder Dockingkontakte fuer die Sonde bekommt, waere das eine neue elektrische Risikoklasse und muesste getrennt bewertet werden.

Bewertung: niedrig fuer die aktuelle interne Board-Elektrik, mittel fuer Systemverhalten bei Funkverlust.

Empfehlung:

1. Die Funksonde in der Hardware-Doku explizit als externe, elektrisch entkoppelte Option kennzeichnen.
2. Im Systemdesign Heartbeat-Timeout und sichere Fallback-Reaktion definieren.
3. Spaetere Docking-, Lade- oder Kabeloptionen als eigenstaendige Hardware-Aenderung behandeln.

## 3. Reset- und Boot-Leitungen des ESP32-S3

### 3.1 Direkter Eingriff des Teensy in `CHIP_PU`

Befund:

- `S3_EN_RST` kommt vom Teensy und geht auf `CHIP_PU` des ESP32-S3-DevKitC.
- Damit entscheidet der Teensy direkt, ob der ESP freigegeben oder im Reset gehalten wird.

Risiko:

- Ein Fehlzustand, Glitch oder Boot-Pin-Flattern am Teensy kann den ESP beim Einschalten blockieren.
- Bei Versorgungseinbruechen kann der Teensy den ESP wiederholt ungewollt resetten.
- Das erzeugt ein asymmetrisches Abhaengigkeitsverhaeltnis: Faellt der Hauptcontroller aus, kann er den Nebencontroller elektrisch mit in den Ausfall ziehen.

Bewertung: hoch.

Empfehlung:

1. `CHIP_PU` lokal am ESP mit eindeutigem Default-Zustand absichern.
2. Den Teensy nicht als blanke Push-Pull-Dominanz auf `CHIP_PU` lassen, sondern ueber Open-Drain oder Transistorstufe einkoppeln.
3. Falls Reset nur fuer Programmierfaelle noetig ist, einen trennbaren Servicepfad vorsehen.

### 3.2 Direkter Eingriff des Teensy in `GPIO0`

Befund:

- `S3_GPIO0` kommt direkt vom Teensy auf den Boot-Strapping-Pin `GPIO0` des ESP32-S3.

Risiko:

- Liegt `GPIO0` waehrend Reset oder Brownout zur falschen Zeit auf Low, landet der ESP im Bootloader statt in der Anwendung.
- Damit koennen sporadische Startprobleme entstehen, die wie Softwarefehler aussehen, aber elektrisch verursacht sind.

Bewertung: hoch.

Empfehlung:

1. `GPIO0` am ESP mit starkem, lokalem Default-Pegel versehen.
2. Den Teensy nur aktiv eingreifen lassen, wenn wirklich ein definierter Programmierablauf gebraucht wird.
3. Fuer Service einen Jumper, 0-Ohm-Link oder Schalter vorsehen, mit dem `GPIO0` vom Teensy entkoppelt werden kann.

### 3.3 Gemeinsame Wirkung von Versorgung und Boot-Straps

Befund:

- `+5V_EXT`, die ESP-seitige `+3.3V`-Hilfsschiene und die beiden Strap-/Reset-Leitungen greifen funktional ineinander, obwohl `U2 3V3` selbst nicht angebunden ist.

Risiko:

- Ein Spannungsdip auf `+5V_EXT` oder ein unklarer Zustand der Hilfsschiene kann exakt in dem Moment auftreten, in dem `GPIO0` oder `CHIP_PU` nicht sicher definiert sind.
- Dann entsteht kein harter Ausfall, sondern ein intermittierender Fehlstart. Das ist im Feld besonders schwer zu diagnostizieren.

Bewertung: hoch.

Empfehlung:

1. Boot-Straps mit klaren Default-Widerstaenden direkt am ESP definieren.
2. Teensy-seitige Ansteuerung erst nach stabiler Versorgung freigeben.
3. Die Einschaltsequenz mit realen Lasten messen: Display-Backlight, NeoPixel, Audio und Vibra gleichzeitig.

## 4. Priorisierte Massnahmen

1. Die Rolle von `+3.3V` im ESP-Sheet eindeutig machen und von der eigentlichen DevKit-Versorgung begrifflich wie elektrisch sauber trennen.
2. `S3_EN_RST` und `S3_GPIO0` aus der direkten Push-Pull-Kopplung herausnehmen und servicetauglich machen.
3. Die 5-V-Verteilung in Zweige mit lokaler Pufferung und Rueckspeiseschutz aufteilen.
4. Externe Anschluesse fuer Kopf, NeoPixel und SSR mit ESD-/Transientenschutz absichern.
5. Das Gesamtsystem mit Worst-Case-Last und Hot-Plug real vermessen, statt nur logisch zu plausibilisieren.

## Schlussfolgerung

Der Schaltplan ist funktional schluessig, aber elektrisch an drei Stellen verwundbar: gemeinsame `+5V_EXT`-Lastverteilung, direkte Teensy-Kontrolle ueber `CHIP_PU` und `GPIO0`, sowie unentkoppelte externe Leitungswege fuer Kopf und 5-V-Ausgaenge. Die fruehere Annahme einer direkt gekoppelten 3,3-V-Versorgung beider Dev-Kits trifft fuer die aktuelle Revision nicht zu, weil `U2 Pin 2 (3V3)` unbeschaltet ist. Wenn diese realen Schwachstellen sauber abgesichert werden, steigt die Robustheit deutlich staerker als durch weitere Firmwarearbeit allein.
