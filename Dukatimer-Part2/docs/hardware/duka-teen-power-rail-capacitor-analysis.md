# Duka-Teen Power Rail Capacitor Analysis

Grundlage dieser Auswertung:

- `Duka-Teen.kicad_pcb` fuer die real verdrahteten Netze und die tatsaechliche Bauteilposition auf der Basisplatine
- `teensy.kicad_sch` fuer die funktionale Einordnung der Teensy-Seite
- `ESP.kicad_sch` fuer die funktionale Einordnung der ESP-, Audio- und Vibra-Seite

Ziel dieser Analyse:

- die real bestueckten Puffer- und Stuetzkondensatoren auf `+5V_EXT`, `+3.3V` und `/teensy/+3V3_HEAD` identifizieren
- deren Positionierung auf der Basisplatine bewerten
- echte Versorgungskondensatoren von Signal- und Funktionskondensatoren trennen
- beurteilen, ob Werte und Verteilung fuer die aktuelle Architektur sinnvoll sind

## Wichtige Abgrenzungen

Diese Punkte sind fuer die Interpretation entscheidend:

1. `U2 Pin 21` des `ESP32-S3-DevKitC` haengt an `+5V_EXT`, `U2 Pin 2 (3V3)` ist auf der Basisplatine nicht an ein Netz angeschlossen.
2. `C_Power_9` und `C_Power_10` liegen zwar auf `+3.3V`, puffern damit aber nicht direkt die interne 3,3-V-Versorgung des ESP-DevKitC.
3. `C_Power_11` bis `C_Power_14` sind keine Versorgungskondensatoren, sondern optionale `NC`-Kondensatoren von GPIO-Leitungen nach GND.
4. `C_T4_17` bis `C_T4_19` sind ebenfalls keine Rail-Kondensatoren, sondern GND-Kondensatoren an `ENC4_A`, `ENC4_B` und `ENC4_SW`.
5. `AUDIO:C5` ist kein Versorgungs-Puffer, sondern liegt zwischen `AUDIO_SDN` und GND.
6. Die optionale Wireless-TSL2591-Sonde ist eine externe Funkbaugruppe mit eigener Versorgung und daher nicht Teil dieser Rail-Kondensatoranalyse der Basisplatine.

## Einbindung Wireless TSL2591

Die Wireless-TSL2591-Sonde wird im aktuellen Part2-Konzept nicht aus `+5V_EXT`, nicht aus `+3.3V` und nicht aus `/teensy/+3V3_HEAD` der Basisplatine gespeist. Sie ist als separates externes Geraet mit eigener MCU, eigenem TSL2591, OLED und lokaler Stromversorgung zu verstehen.

Fuer diese Analyse bedeutet das:

- Die Sonde erzeugt in der aktuellen Architektur keine zusaetzliche Last auf den internen Rails der Basisplatine.
- Es sind deshalb auch keine zusaetzlichen lokalen Puffer- oder Bulk-Kondensatoren auf der Duka-Teen-Platine fuer die Sonde erforderlich.
- Erst wenn spaeter eine Docking-, Lade- oder kabelgebundene Versorgungsoption vorgesehen wird, muesste die Rail- und Kondensatorauslegung dafuer separat neu bewertet werden.

## Tatsaechlich relevante Versorgungskondensatoren

| Netz | Referenz | Wert | Funktionale Einordnung |
| --- | --- | --- | --- |
| `+5V_EXT` | `C_Power_5` | `10uF` | lokaler Bulk-Kondensator auf der Teensy-Seite |
| `+5V_EXT` | `C_Power_15` | `100nF` | verteilter 5-V-Stuetzkondensator auf der Teensy-Seite |
| `+5V_EXT` | `C_Power_16` | `100nF` | verteilter 5-V-Stuetzkondensator auf der Teensy-Seite |
| `+5V_EXT` | `C_T4_2` | `100nF` | weiterer 5-V-Stuetzkondensator auf der Teensy-Seite |
| `+5V_EXT` | `AUDIO_C3` | `100nF` | lokaler HF-Bypass der Audio-Stufe |
| `+5V_EXT` | `AUDIO_C4` | `1000uF` | grosser Energiespeicher der Audio-Stufe |
| `+3.3V` | `C_Power_4` | `100nF` | verteilter 3,3-V-Stuetzkondensator auf der Teensy-Seite |
| `+3.3V` | `C_Power_7` | `100nF` | lokaler 3,3-V-Stuetzkondensator nahe der Teensy-Versorgung |
| `+3.3V` | `C_Power_9` | `100nF` | Stuetzkondensator auf der ESP-Sheet-Hilfsschiene |
| `+3.3V` | `C_Power_10` | `100nF` | Stuetzkondensator auf der ESP-Sheet-Hilfsschiene |
| `/teensy/+3V3_HEAD` | `C_Power_6` | `100nF` | HF-Bypass fuer die externe Kopfversorgung |
| `/teensy/+3V3_HEAD` | `C_Power_8` | `10uF` | lokaler Bulk-Kondensator fuer die externe Kopfversorgung |

## 1. Analyse der Schiene `+5V_EXT`

### 1.1 Tatsaechtliche Bestueckung

Auf `+5V_EXT` liegen auf der Basisplatine folgende Kondensatoren:

- `C_Power_5 = 10uF`
- `C_Power_15 = 100nF`
- `C_Power_16 = 100nF`
- `C_T4_2 = 100nF`
- `AUDIO_C3 = 100nF`
- `AUDIO_C4 = 1000uF`

Wichtig ist die funktionale Trennung:

- `C_Power_5`, `C_Power_15`, `C_Power_16` und `C_T4_2` stuetzen die allgemeine 5-V-Verteilung auf der Basisplatine.
- `AUDIO_C3` und `AUDIO_C4` liegen zwar ebenfalls an `+5V_EXT`, sind aber klar der Audio-Stufe zuzuordnen und daher kein allgemeiner 5-V-Puffer fuer die komplette ESP-Seite.

### 1.2 Positionierung

`C_Power_5` sitzt auf der Teensy-Seite in der Naehe des `VIN`-Anschlusses von `U1`. Bezogen auf das Footprint-Layout liegt der Kondensator nur rund 14 mm vom `U1`-`VIN`-Pad entfernt. Das ist fuer einen Basisplatten-Bulk-Kondensator sinnvoll.

`C_Power_15` und `C_Power_16` sind weiter ueber die 5-V-Verteilung verteilt. Das ist nicht als hochlokale Chip-Entkopplung zu lesen, sondern als verteilte Stuetze fuer 5-V-Peripherie und externe Lastpfade. In dieser Rolle ist die Positionierung in Ordnung.

Auf der ESP-Seite fehlt dagegen ein kleiner lokaler 5-V-Puffer direkt am Einspeisepunkt `U2 Pin 21`. Die naechsten 5-V-Kondensatoren dort sind die Audio-Kondensatoren. `AUDIO_C3` liegt grob 34 mm, `AUDIO_C4` grob 55 mm vom 5-V-Pad des DevKitC entfernt. Das ist fuer einen Audio-Zweig noch plausibel, aber nicht ideal als lokaler Eingangspuffer fuer das ESP-DevKit.

### 1.3 Dimensionierung

Die Kombination aus `10uF` plus mehreren `100nF` auf `+5V_EXT` ist grundsaetzlich sinnvoll:

- `10uF` ist passend als niederfrequenter bzw. mittelfrequenter Eingangspuffer auf der Basisplatine.
- `100nF` ist passend fuer schnelle Stromspitzen und lokale Stoerunterdrueckung.

`AUDIO_C4 = 1000uF` ist gross dimensioniert, aber fuer eine gepulste Audio-Last nicht unplausibel. Dieser Kondensator verbessert vor allem die lokale Stabilitaet der Audio-Stufe und ist nicht als generischer MCU-Puffer zu interpretieren.

### 1.4 Bewertung

Fuer die Basisplatine insgesamt ist `+5V_EXT` sinnvoll gepuffert, besonders auf der Teensy-Seite und im Audio-Zweig. Was fehlt, ist kein weiterer grosser Elko, sondern ein kleines lokales Pufferpaar direkt am 5-V-Eingang des ESP-DevKitC, zum Beispiel `100nF + 1uF..10uF` unmittelbar an `U2 Pin 21`.

## 2. Analyse der Schiene `+3.3V`

### 2.1 Tatsaechtliche Bestueckung

Auf der allgemeinen `+3.3V`-Schiene der Basisplatine liegen:

- `C_Power_4 = 100nF`
- `C_Power_7 = 100nF`
- `C_Power_9 = 100nF`
- `C_Power_10 = 100nF`

Es gibt auf dieser Schiene in der aktuellen Revision keinen zusaetzlichen Bulk-Kondensator auf der Basisplatine.

### 2.2 Positionierung

`C_Power_7` sitzt nahe an den `3V3`-Pads des Teensy-Footprints. Die Distanz zum naechsten `U1`-`3V3`-Pad liegt grob bei 5 mm. Das ist eine gute Position fuer einen lokalen Verteilstuetzer.

`C_Power_4` ist deutlich weiter von den `U1`-`3V3`-Pads entfernt, grob in der Groessenordnung von 45 mm. Dieser Kondensator ist damit kein lokaler Modul-Bypass mehr, sondern ein verteilter Rail-Kondensator in einem anderen Bereich der Platine. In dieser Rolle ist er noch sinnvoll, aber seine Wirkung am Teensy selbst ist naturgemaess geringer.

`C_Power_9` und `C_Power_10` liegen im ESP-Bereich der Basisplatine, aber nicht an der realen Modulspeisung des ESP32-S3-DevKitC. Sie stuetzen nur die Hilfsschiene `+3.3V` in diesem Bereich. Als Puffer direkt fuer das ESP-Modul sind sie deshalb ungeeignet.

### 2.3 Dimensionierung

Viermal `100nF` ist fuer eine verteilte Hilfs- oder Logikschiene unkritisch und plausibel. Die Auslegung waere aber zu mager, wenn dieselbe Schiene eine groessere dynamische Last direkt tragen muesste.

In der aktuellen Architektur ist das noch akzeptabel, weil:

- der Teensy als Dev-Board eigene lokale Entkopplung mitbringt
- das ESP-DevKitC intern aus `+5V_EXT` versorgt wird und seine kritische Chip-Entkopplung auf dem Modul selbst sitzt

### 2.4 Bewertung

Die `+3.3V`-Schiene der Basisplatine ist als Hilfsschiene sinnvoll, aber eher sparsam gepuffert. Das ist fuer den aktuellen Aufbau vertretbar. Falls spaeter mehr ESP-nahe Hilfslasten oder externe 3,3-V-Verbraucher auf diese Schiene gelegt werden, sollte ein zusaetzlicher Bulk-Kondensator im Bereich `1uF..4.7uF` vorgesehen werden.

## 3. Analyse der Schiene `/teensy/+3V3_HEAD`

### 3.1 Tatsaechtliche Bestueckung

Auf der extern gefuehrten Kopfversorgung `/teensy/+3V3_HEAD` liegen:

- `C_Power_6 = 100nF`
- `C_Power_8 = 10uF`

Diese Schiene fuehrt ueber den Sicherungs-/Verteilpfad weiter Richtung Sensor-RJ45 und externer Kopfversorgung.

### 3.2 Positionierung

Die beiden Kondensatoren sitzen direkt im Kopfversorgungszweig und nicht irgendwo allgemein auf der Platine. Das ist genau die richtige Stelle, weil diese Schiene aus Sicht der Basisplatine eine externe, potenziell steckbare Last versorgt.

Die Kombination aus kleinem HF-Kondensator und lokalem Bulk-Kondensator sitzt damit deutlich sinnvoller als eine rein verteilte Loesung irgendwo nahe am Hauptcontroller.

### 3.3 Dimensionierung

`100nF + 10uF` ist fuer einen extern gefuehrten 3,3-V-Zweig eine passende Kombination:

- `100nF` fuer schnelle Stoeranteile und Leitungsinduktivitaet
- `10uF` fuer Lastspruenge, Kabel- und Hot-Plug-Effekte in moderatem Rahmen

### 3.4 Bewertung

Von den betrachteten Schienen ist `/teensy/+3V3_HEAD` auf der Basisplatine am schluessigsten gepuffert. Positionierung und Dimensionierung passen gut zum Zweck der Leitung.

## 4. Kondensatoren, die nicht als Rail-Puffer gezaehlt werden duerfen

Die folgenden Bauteile sehen im Namen nach Versorgungskondensator aus oder wurden in der Suche mitgefunden, gehoeren aber nicht in die Rail-Bewertung:

| Referenz | Wert | Tatsaechtliches Netz |
| --- | --- | --- |
| `C_Power_11` | `NC` | `/ESP/S3_GPIO10` gegen GND |
| `C_Power_12` | `NC` | `/ESP/S3_GPIO09` gegen GND |
| `C_Power_13` | `NC` | `/ESP/S3_GPIO14` gegen GND |
| `C_Power_14` | `NC` | `/ESP/S3_GPIO13` gegen GND |
| `C_T4_17` | `100nF` | `/ESP/ENC4_A` gegen GND |
| `C_T4_18` | `100nF` | `/ESP/ENC4_B` gegen GND |
| `C_T4_19` | `100nF` | `/ESP/ENC4_SW` gegen GND |
| `AUDIO:C5` | `10uF` | `/ESP/AUDIO_SDN` gegen GND |

Diese Bauteile duengen die Aussage zur Versorgung nur dann aus, wenn man sie irrtuemlich mitzaehlt. Fuer die eigentliche Pufferbewertung muessen sie ausgeschlossen werden.

## 5. Gesamturteil

### 5.1 Was gut ist

- `+5V_EXT` hat auf der Basisplatine eine sinnvolle Grundpufferung auf der Teensy-Seite.
- Die Audio-Stufe ist lokal stark gepuffert.
- `/teensy/+3V3_HEAD` ist mit `100nF + 10uF` passend und zweckgerecht ausgelegt.
- Die Basisplatine versucht nicht, die echte Chip-Entkopplung der Dev-Boards zu ersetzen, sondern ergaenzt die Verteilung.

### 5.2 Was nur bedingt gut ist

- Die allgemeine `+3.3V`-Schiene ist nur mit `100nF`-Kondensatoren versehen.
- `C_Power_9` und `C_Power_10` liegen zwar im ESP-Bereich, puffern aber nicht die reale Versorgung des ESP-DevKitC.
- `C_Power_4` ist fuer eine lokale Teensy-Entkopplung schon recht weit vom Modul entfernt und wirkt eher als verteilter Rail-Kondensator.

### 5.3 Was ich in einer naechsten Revision aendern wuerde

1. Direkt am `U2`-5-V-Eingang ein kleines lokales Pufferpaar `100nF + 1uF..10uF` vorsehen.
2. Falls die Hilfsschiene `+3.3V` kuenftig mehr Last tragen soll, dort zusaetzlich `1uF..4.7uF` vorsehen.
3. Die Bezeichnung der scheinbaren `C_Power_*`-Kondensatoren an GPIOs klarer machen oder in der Doku explizit als Signalfilter markieren.

## Schlussfolgerung

Die Kondensatorbestueckung ist in der aktuellen Architektur ueberwiegend sinnvoll. Sie passt zu einer Basisplatine, die zwei bereits fertig entkoppelte Dev-Boards verbindet und zusaetzliche externe Lasten sowie eine Audio-Stufe versorgt. Die staerksten Bereiche sind die Kopfversorgung und die Audio-Pufferung. Die schwaechste Stelle ist nicht die absolute Kapazitaetsmenge, sondern das Fehlen eines kleinen lokalen 5-V-Puffers direkt am ESP-DevKit-Eingang. Genau dort wuerde eine kleine Ergaenzung die Robustheit am deutlichsten verbessern.
