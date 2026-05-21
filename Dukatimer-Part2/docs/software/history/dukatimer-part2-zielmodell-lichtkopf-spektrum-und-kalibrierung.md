# Dukatimer-Part2 Zielmodell fuer Lichtkopf, Spektrum und Kalibrierung

Stand: 2026-04-24

## Ziel

Dieses Dokument definiert das Zielmodell fuer den naechsten fachlichen Ausbau der Part2-Lichtarchitektur.

Es leitet aus zwei bereits gesicherten Fakten eine konkrete Modelltrennung ab:

- Die historische Umsetzung kompensierte die fotografische Realitaet hauptsaechlich oberhalb des LED-Treibers.
- Der aktuelle Part2-Code besitzt bereits eine saubere Trennung zwischen lokaler Lichtentscheidung und physischem NeoPixel-Transport, aber noch keine getrennten Modelle fuer Spektrum, Kopfkalibrierung und Papierprofil.

Das Ziel ist eine belastbare Architektur fuer drei getrennte Fragen:

1. Welches Licht soll fachlich erzeugt werden?
2. Wie wird dieses Soll-Licht auf den realen Lichtkopf abgebildet?
3. Wie werden daraus papierbezogene Belichtungszeit oder Ziel-Dosis?

## Ausgangslage im aktuellen Part2-Stand

Die aktuelle Architektur ist fuer lokale Schalter und Head-Basic-Output bereits sauberer als die Altstaende.

Vorhanden sind insbesondere:

- `LightController` als fachliche Autoritaet fuer `Save`, `Fokus`, `Room` und `SaveLatch`
- `HeadLightArbiter` als saubere Quell-Priorisierung zwischen lokalem Licht und spaeterer Exposure-Quelle
- `HeadLightCommand` als normierter, aber noch sehr einfacher Low-Level-Ausgabebefehl
- `NeoPixelHead` als physische 16x16- bzw. Segment-Transport-Schicht

Die aktuelle Grenze ist jedoch klar:

- `HeadLightCommand` kennt nur `Off` und `SolidColor`
- die physische Lichtausgabe wird direkt in elektrischen RGB-Werten beschrieben
- es gibt noch kein Modell fuer fotografisch logische Kanaele wie `Soft`, `Hard`, `FocusWhite` oder `SafelightRed`
- es gibt noch keine eigene Kopfkalibrierung
- es gibt noch kein getrenntes Papier- bzw. Exposure-Profil im Part2-Code

## Historische Lehre, die das Zielmodell bestimmt

Die historische Analyse zeigt konsistent:

- `2.240` besass bereits feste fachliche Gradations-zu-RGB-Mischungen, aber keine echte Treiber-Kalibrierung.
- `v0.3` erzeugte papierprofilbasiert direkte Gruen-Blau-PWM-Verhaeltnisse.
- `v0.9` verschob die Hauptkompensation in `PaperManager`, D-logH-Kurve und Dosisregelung.
- keine belegte Altversion modellierte den Lichtkopf selbst als eigene kalibrierte Transferfunktion.

Daraus folgt fuer Part2:

- Das historische Verhalten soll funktional erhalten bleiben.
- Die historische Vermischung soll aber nicht erneut entstehen.

Die fehlende Trennung wird in Part2 deshalb explizit nachgezogen.

## Zentrale Modelltrennung

Fuer Part2 werden drei eigenstaendige Modelle benoetigt:

1. `HeadSpectrumCommand`
2. `HeadCalibrationProfile`
3. `PaperExposureProfile`

Diese drei Modelle haben unterschiedliche Verantwortung und duerfen nicht ineinander aufgehen.

### Leitentscheidung fuer die Drei-Schichten-Pipeline

Die Drei-Schichten-Pipeline ist fuer Part2 die verbindliche Hauptarchitektur.

Sie bedeutet:

- `HeadSpectrumCommand` beschreibt ausschliesslich das fachlich gewuenschte Licht.
- `HeadCalibrationProfile` bildet dieses Soll-Licht auf korrigierte RGB-Ausgabewerte fuer den realen 16x16-NeoPixel-Kopf ab.
- `PaperExposureProfile` verwaltet ausschliesslich die papierbezogene fotografische Seite.

Wichtig ist die Abgrenzung:

- die Three-Layer-Pipeline ist das fachliche und persistente Kernmodell
- `ExposureEngine`, `SensorManager`, Temperaturueberwachung und UI sind Laufzeitdienste um diese Pipeline herum
- diese Laufzeitdienste duerfen die Trennung der drei Modelle nicht wieder aufheben

## 1. HeadSpectrumCommand

### Zweck von HeadSpectrumCommand

`HeadSpectrumCommand` beschreibt das fachlich gewuenschte Licht des Kopfes, noch ohne Bindung an konkrete elektrische 8-Bit-RGB-Ausgangswerte.

Diese Schicht beantwortet die Frage:

- Was soll der Lichtkopf fotografisch bzw. funktional tun?

und nicht:

- Welche NeoPixel-Rohwerte werden dafuer direkt geschrieben?

### Warum HeadSpectrumCommand ein eigenes Modell braucht

Die aktuelle Low-Level-Struktur `HeadLightCommand` ist dafuer zu flach.

Sie kann heute nur:

- aus
- eine feste Vollflaechenfarbe

ausdruecken.

Fuer die fachliche Domäne reicht das nicht, weil historische und zukuenftige Modi logisch in anderen Begriffen arbeiten:

- Safelight rot
- Focus weiss
- BW auf Basis einer Gradation
- Splitgrade Soft-Phase
- Splitgrade Hard-Phase
- Burn mit Grade-Bezug
- Teststrip-Schritt
- Preflash weiss oder gruen
- Kalibrier- oder Diagnosesollbild

### Zielinhalt von HeadSpectrumCommand

`HeadSpectrumCommand` soll deshalb nicht den elektrischen RGB-Raum repraesentieren, sondern einen logischen Lichtauftrag.

Die minimale sichere Form dafuer ist ein normalisierter logischer Kanalmix.

### Empfohlene Struktur fuer HeadSpectrumCommand

```cpp
struct LogicalHeadChannels {
    float safelightRed = 0.0f;
    float focusWhite = 0.0f;
    float soft = 0.0f;
    float hard = 0.0f;
};

enum class HeadSpectrumSemantic : uint8_t {
    Off,
    LocalSafelight,
    LocalFocus,
    BwGradeMix,
    SplitgradeSoft,
    SplitgradeHard,
    Burn,
    TestStrip,
    Preflash,
    Calibration,
    DiagnosticPattern,
    CustomLogicalMix,
};

struct HeadSpectrumCommand {
    HeadSpectrumSemantic semantic = HeadSpectrumSemantic::Off;
    LogicalHeadChannels channels = {};
    float masterIntensity = 1.0f;
    float grade = 2.5f;
};
```

### Semantik der logischen Kanaele

Die logischen Kanaele beschreiben keine physischen LED-Dies, sondern fachliche Emissionsrollen:

- `safelightRed`: lokales Schutzlicht
- `focusWhite`: lokales Einrichtlicht
- `soft`: fotografischer Soft-Anteil
- `hard`: fotografischer Hard-Anteil

Diese Unterscheidung ist absichtlich fachlich und nicht hardwarebezogen.

Denn historisch war bereits sichtbar, dass:

- `FocusWhite` nicht einfach dasselbe ist wie `Soft + Hard`
- `SafelightRed` eine eigene funktionale Rolle besitzt
- `Soft` und `Hard` historisch fotografische Begriffe sind, auch wenn sie elektrisch oft ueber Gruen und Blau abgebildet wurden

### Was HeadSpectrumCommand bewusst nicht enthalten soll

Nicht in dieses Modell gehoeren:

- NeoPixel-Pins
- Segment-Topologie
- 8-Bit-RGB-Rohwerte
- Paper-K-Faktoren
- Dosis- oder Zeitsollwerte
- Persistenzdetails des Papiermodells

### Beziehung von HeadSpectrumCommand zum aktuellen Code

Im aktuellen Stand ist `HeadLightCommand` der direkte Vorlaeufer, aber noch eine Ebene zu tief.

Zielbild:

- `LightController` und spaetere Exposure-Logik erzeugen kuenftig `HeadSpectrumCommand`
- eine gesonderte Kalibrierschicht wandelt diesen in `HeadLightCommand` oder ein spaeteres `HeadDriveCommand` um

## 2. HeadCalibrationProfile

### Zweck von HeadCalibrationProfile

`HeadCalibrationProfile` beschreibt die kalibrierte Abbildung von logischen Lichtkanaelen auf den realen Lichtkopf.

Diese Schicht beantwortet die Frage:

- Wie muss der konkrete Kopf elektrisch angesteuert werden, damit das fachlich gewuenschte Licht entsteht?

### Warum HeadCalibrationProfile ein eigenes Modell braucht

Genau diese Schicht fehlte historisch als saubere Abstraktion.

Die Altstaende arbeiteten stattdessen mit Mischtabellen, K-Faktoren und Zeitkompensation. Das konnte funktional ausreichend sein, vermischte aber:

- Soll-Spektrum
- physische Kopfcharakteristik
- Papierreaktion

Part2 soll diese Vermischung aufbrechen.

### Verantwortung von HeadCalibrationProfile

`HeadCalibrationProfile` gehoert zur Hardwareseite des Lichtkopfs.

Es ist verantwortlich fuer:

- Kopf- oder Hardwarevariante
- Zuordnung logischer Kanaele auf physische RGB-Ausgaenge
- einfache Gamma-Korrektur fuer den Ausgangspfad
- globalen spektralen Abgleich fuer Rot, Gruen und Blau
- Kanalgewichtung
- nichtlineare Transferfunktion je logischem Kanal
- globale Schutz- oder Stromgrenzen
- spaeter optional Segment- oder Uniformitaetskorrekturen

Mit `einfacher Gamma-Korrektur` ist hier bewusst kein komplexes Farbmanagement gemeint.

Der erste sinnvolle Zielumfang ist:

- eine gemeinsame, einfache Transferkurve fuer den 8-Bit-Ausgangspfad
- ein globaler RGB-Skalierungsfaktor, um Kopfstreuung, LED-Alterung oder Chargenunterschiede auszugleichen

Das ist genug, um die reale Hardwarevarianz abzufangen, ohne Papiermodell und Kopfmodell wieder zu vermischen.

### Zielinhalt von HeadCalibrationProfile

Dieses Modell muss zwei Dinge leisten:

1. Mischmatrix: logischer Kanal zu physischem RGB-Anteil
2. Transferkurve: normalisierter Sollwert zu elektrischem Ansteuerwert

### Empfohlene Struktur fuer HeadCalibrationProfile

```cpp
struct PhysicalRgbMix {
    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;
};

struct ChannelDriveCurve {
    uint8_t lut[256] = {};
};

struct GlobalRgbCalibration {
    float redScale = 1.0f;
    float greenScale = 1.0f;
    float blueScale = 1.0f;
};

struct LogicalChannelCalibration {
    PhysicalRgbMix rgbMix = {};
    ChannelDriveCurve driveCurve = {};
    float maxNormalizedOutput = 1.0f;
};

struct HeadCalibrationProfile {
    uint16_t version = 1;
    bool calibrated = false;
    float globalOutputLimit = 1.0f;
    ChannelDriveCurve sharedGammaCurve = {};
    GlobalRgbCalibration globalRgb = {};

    LogicalChannelCalibration safelightRed = {};
    LogicalChannelCalibration focusWhite = {};
    LogicalChannelCalibration soft = {};
    LogicalChannelCalibration hard = {};
};
```

### Warum eine LUT in HeadCalibrationProfile sinnvoll ist

Der aktuelle Ausgangspfad arbeitet physisch mit 8-Bit-RGB-Werten.

Eine 256er LUT pro logischem Kanal ist deshalb ein pragmatischer Startpunkt, weil sie:

- direkt zum heutigen Treiber passt
- nichtlineare Kennlinien abbilden kann
- spaeter auch aus Messung oder Kalibrierwizard erzeugbar ist
- die historische Luecke erstmals sauber schliesst

Falls der Speicher spaeter zu knapp ist, kann dieselbe Semantik spaeter intern als Stuetzwertkurve gespeichert werden.

Die hier gezeigte Kombination aus `sharedGammaCurve` und `globalRgb` bildet genau den vom Zielbild geforderten ersten Kalibrierschritt ab:

- Gamma fuer den allgemeinen Helligkeitsverlauf
- globaler Spektralabgleich fuer Rot, Gruen und Blau

Spaetere segmentweise oder kanalweise Detailkorrekturen bleiben moeglich, sind aber nicht Voraussetzung fuer den ersten funktionsfaehigen Kopf.

### Was HeadCalibrationProfile bewusst nicht enthalten soll

Nicht in dieses Modell gehoeren:

- Papier-K-Faktoren
- Splitgrade-Regeln des Papiers
- Teststrip-Schrittlogik
- Zielzeit oder Ziel-Dosis
- Zustand lokaler Schalter

### Beziehung von HeadCalibrationProfile zum aktuellen Code

Der aktuelle `NeoPixelHead` ist nicht der Ort fuer die Kalibrierentscheidung.

`NeoPixelHead` soll weiterhin nur:

- Matrixzustand halten
- Pixel schreiben
- physisch praesentieren

Die Kalibrierlogik sitzt davor.

Sinnvolles Zielbild:

- `HeadCalibrationProfile` wird von einer spaeteren `HeadCalibrationMapper`- oder `HeadDriveMapper`-Schicht genutzt
- diese erzeugt aus `HeadSpectrumCommand` den low-level `HeadLightCommand`

## 3. PaperExposureProfile

### Zweck von PaperExposureProfile

`PaperExposureProfile` repraesentiert das papierbezogene, fotografische Modell.

Diese Schicht beantwortet die Frage:

- Welche Zeit, Dosis oder Splitgrade-Aufteilung ist fuer ein bestimmtes Papier und eine bestimmte Belichtungssituation noetig?

### Historische Grundlage von PaperExposureProfile

Dieses Modell ist der am staerksten historisch abgesicherte Teil.

Historisch belegt sind insbesondere:

- `Ksoft`, `Khard`, `Kbw`
- `isoP`, `isoR`
- `gradeK_Soft[]`, `gradeK_Hard[]`
- D-logH- oder Smart-Math-Logik
- slot-gebundene Preflash-Parameter
- Zeit- und Dosispfade

### Verantwortung von PaperExposureProfile

`PaperExposureProfile` ist verantwortlich fuer:

- persistente Papierdaten
- Kalibrierstatus
- Gradationsmodell
- Zeit- oder Dosisparameter
- Preflash-Parameter
- spaeter weitere papierbezogene Regeln

Es ist ausdruecklich nicht verantwortlich fuer:

- Kopfverdrahtung
- LED-Kennlinien
- NeoPixel-Ausgabe

### Empfohlene Struktur fuer PaperExposureProfile

```cpp
struct PreflashSettings {
    bool calibrated = false;
    bool enabled = false;
    float thresholdSeconds = 0.0f;
    float factor = 1.0f;
    uint8_t level = 0;
    uint8_t colorMode = 0;
};

struct PaperExposureProfile {
    char name[24] = {};
    bool calibrated = false;
    bool fixedGrade = false;
    bool useIsoMath = false;

    float fixedGradeValue = 2.5f;
    float isoP = 100.0f;
    float isoR = 100.0f;

    float kBw = 0.0f;
    float kSoft = 0.0f;
    float kHard = 0.0f;

    float gradeKSoft[11] = {};
    float gradeKHard[11] = {};

    PreflashSettings preflash = {};
};
```

### Beziehung von PaperExposureProfile zum Spektrummodell

Das Papierprofil erzeugt selbst keine elektrischen RGB-Werte.

Es liefert stattdessen fachliche Parameter fuer:

- Splitgrade-Anteile
- BW- oder SG-Zeit
- Ziel-Dosis
- Preflash-Freigaben

Die Ausgabe in Richtung Kopf bleibt logisch.

Das Zielbild lautet also:

- `PaperExposureProfile` berechnet `soft` und `hard`
- `HeadSpectrumCommand` beschreibt daraus den Lichtauftrag
- `HeadCalibrationProfile` bildet diesen auf den realen Kopf ab

## Die Zielpipeline fuer Part2

Die drei Modelle werden in Part2 in einer festen Richtung miteinander verbunden.

### Zielrichtung

```text
Mode / UI / Local Switches / ExposureEngine
    -> HeadSpectrumCommand
    -> HeadCalibrationProfile
    -> HeadLightCommand
    -> NeoPixelHead
    -> physischer Lichtkopf

PaperExposureProfile
    -> ExposureEngine / BW / SG / Burn / Teststrip
    -> HeadSpectrumCommand
```

### Bedeutung

Damit ist klar getrennt:

- Fachseite entscheidet das gewuenschte Licht
- Kalibrierseite entscheidet die hardwaregerechte Ausgabe
- Papierseite entscheidet fotografische Zeiten und Dosen

## Laufzeitdienste um die Drei-Schichten-Pipeline

Die drei Modelle allein reichen fuer Part2 nicht aus. Um sie herum braucht es klar getrennte Laufzeitdienste.

Diese Laufzeitdienste sind keine vierte Persistenzschicht, sondern operative Komponenten, die die Pipeline verwenden.

Die wichtigsten sind:

- `ExposureEngine` fuer den Belichtungsablauf und die Dosisregelung
- `SensorManager` fuer TSL2561, DS18B20 und weitere Messpfade
- `Display/UI` fuer Telemetrie und Bedienung
- `Input/App` fuer Moduslogik, Pending-Aktionen und lokale Lichtanforderungen

### Rolle der ExposureEngine im Zielbild

Die ExposureEngine wird in Part2 nicht mehr als reines Zeitschaltwerk gedacht.

Sie ist der Laufzeitdienst, der:

- aus `PaperExposureProfile` eine Ziel-Dosis oder Zielzeit uebernimmt
- daraus einen fachlichen Lichtauftrag als `HeadSpectrumCommand` anfordert
- ueber `HeadCalibrationProfile` den realen Kopf ansteuert
- waehrend der Belichtung die Rueckfuehrung ueber Sensorik auswertet

### Geschlossener Regelpfad im Zielbild

Der geschlossene Regelpfad fuer Part2 lautet:

```text
PaperExposureProfile
    -> Ziel-Dosis / Zielzeit / Splitgrade-Anteil

ExposureEngine
    -> HeadSpectrumCommand
    -> HeadCalibrationProfile
    -> HeadLightCommand
    -> NeoPixelHead
    -> Lichtkopf

TSL2561
    -> gemessener Lichtfluss
    -> Ist-Dosis = Summe aus lux * dt
    -> ExposureEngine
```

Die wesentliche Zielaussage bleibt dabei konsistent zur historischen Lehre:

- geregelt wird die Dosis
- nicht direkt der Farbvektor und nicht direkt die Papierdichte

Fuer die Benutzer- und Messoberflaeche gilt zusaetzlich:

- echte Messworkflows wie Kalibrierung, Densitometrie oder Spotmessung zeigen Werte zusaetzlich in EV an
- der Rohwert, zum Beispiel Lux, bleibt parallel sichtbar
- diese Ableitung wird einmal zentral definiert und nicht pro Workflow separat implementiert

### Praediktiver Shutoff im Zielbild

Der prädiktive Abschaltalgorithmus bleibt als Kernprinzip erhalten und wird fuer Part2 verbindlich.

Die ExposureEngine soll daher:

- waehrend `EXPOSING` laufend die Ist-Dosis integrieren
- aus Rest-Dosis und aktuellem Lichtfluss die Restzeit abschaetzen
- den Abschaltbefehl mit definierter NeoPixel-Latenzkompensation vorziehen

Die bereits historische Groessenordnung von rund `8 ms` Bus- oder Latch-Latenz bleibt dabei eine explizite Systemgroesse und darf nicht stillschweigend im Treiber verschwinden.

### Thermische Kompensation im Zielbild

Die thermische Stabilisierung des Lichtkopfs wird in Part2 als Laufzeitfunktion ergaenzt, ohne die Drei-Schichten-Trennung aufzugeben.

Der saubere Ansatz ist:

- `DS18B20` misst die Temperatur des LED-Kuehlkoerpers
- oberhalb einer ersten Schwelle, zum Beispiel `50 C`, wird die reale Lichtleistung begrenzt
- diese Leistungsbegrenzung wirkt als Laufzeitlimit auf die Kopfansteuerung
- die Dosisregelung kompensiert den sinkenden Lichtfluss automatisch durch laengere Belichtungszeit

Damit bleibt die Architektur sauber:

- `HeadSpectrumCommand` aendert den fachlichen Sollwert nicht
- `HeadCalibrationProfile` bleibt das kalibrierte Kopfmodell
- die thermische Drosselung ist ein runtime-seitiger Begrenzungsfaktor
- die Closed-Loop-Regelung neutralisiert die dadurch entstehende Flussschwankung ueber die Ist-Dosis

### Warum thermische Kompensation nicht ins Papiermodell gehoert

Die Temperaturdrift ist eine Eigenschaft des Lichtkopfs, nicht des Papiers.

Sie darf deshalb nicht in `PaperExposureProfile` versteckt werden.

Andernfalls wuerde dieselbe historische Vermischung wieder entstehen, die Part2 gerade aufloesen soll.

## Sicherheits- und Fail-Safe-Zielbild

Die Robustheit der Part2-Belichtung muss klar ueber die historischen Altstaende hinausgehen.

### Sensor-Watchdog fuer die Dosisregelung

Sobald der Lichtkopf aktiv belichtet, muss der Rueckkanal des lokalen Dosis-Sensors aktiv ueberwacht werden.

Das Zielbild fuer Part2 lautet:

- wenn der `TSL2561` ausfaellt, keine frischen Samples mehr liefert oder unplausible Werte meldet
- und gleichzeitig Licht am Kopf angefordert ist
- dann wird die Belichtung hart beendet

Unplausibel ist dabei insbesondere ein Zustand wie:

- LEDs aktiv
- erwarteter Lichtmodus ungleich `Off`
- gemessener Lichtfluss dauerhaft `0 Lux` oder physikalisch unmoeglich niedrig

### Thermische Sicherheitsstufen

Part2 soll zwei klar getrennte Temperaturschwellen besitzen:

1. oberhalb der Drosselschwelle, zum Beispiel `50 C`
   Die Kopfleistung wird begrenzt, die Dosisregelung kompensiert ueber Zeit.

2. oberhalb der Abschaltschwelle, zum Beispiel `60 C`
   Die Belichtung wird zwingend per Emergency Shutoff beendet.

Die Grundregel lautet:

- fehlbelichtetes Papier ist hinnehmbar
- thermisch geschaedigte Hardware nicht

### Emergency-Shutoff-Zielbild

Der Emergency-Shutoff muss lokal, deterministisch und ohne UI-Abhaengigkeit moeglich sein.

Er darf insbesondere nicht warten auf:

- LVGL-Renderzyklen
- Touch-Events
- Funk- oder ESP-Kommunikation
- langlaufende Applogik

## Ziel-Workflow fuer Belichtung und Bedienung

### End-to-End-Kalibrierung als Arbeitsprinzip

Die Kalibrierung soll in Part2 praxisorientiert bleiben.

Das Ziel ist kein rein elektrisches Laborprofil, sondern eine reproduzierbare Gesamtkette aus:

- Lichtkopf
- Sensor
- Papier

Damit bleibt die historische Staerke erhalten:

- Alterung des Papiers
- Chargenunterschiede
- Kopfstreuung

werden im realen Arbeitsablauf aufgefangen, statt nur theoretisch angenommen zu werden.

### Zustandsmaschine fuer die Belichtung

Die robuste State-Machine bleibt Teil des Zielbilds.

Mindestens verbindlich sind:

- `PRE_WAIT` fuer Safelight-Off, Relais-Settle und Beruhigung vor Lichtbeginn
- `EXPOSING` fuer aktive Zeit- oder Dosisregelung
- `PAUSED` fuer unterbrechbares und fortsetzbares Arbeiten
- `POST_WAIT` fuer Restemission und definiertes Abklingen nach Licht-Aus

Diese Zustaende gehoeren in die ExposureEngine und nicht in verstreute UI- oder Moduslogik.

### Modernes UI als Beobachtungs- und Fuehrungsschicht

Das UI wird fuer Part2 nicht als dekorative Oberflaeche verstanden, sondern als klare Beobachtungs- und Arbeitsfuehrung.

Das Zielbild mit `LVGL` und `EEZ Studio` ist deshalb:

- aktuelle Ist-Dosis sichtbar machen
- verbleibende Restzeit oder Rest-Dosis sichtbar machen
- Kopf- und Umgebungstemperatur sichtbar machen
- laufenden State-Machine-Zustand sichtbar machen
- Fehler- und Fail-Safe-Ursachen klar melden

Das UI darf dabei selbst keine sicherheitskritische Instanz sein. Es visualisiert und bedient, aber die Schutzreaktionen bleiben lokal in den Laufzeitdiensten.

## Abgrenzung zum aktuellen HeadLightCommand

Der aktuelle `HeadLightCommand` bleibt im Zielbild zunaechst erhalten, aber mit bewusst engerer Rolle.

Er soll nur noch low-level repraesentieren:

- `Off`
- konkrete Vollflaechenfarbe
- spaeter eventuell ein kalibriertes `HeadFrame`

Er soll nicht zum Sammelbehaelter fuer alle fachlichen Begriffe werden.

Sonst entstuende dieselbe Vermischung erneut, die historisch bereits unklar war.

## Minimaler Migrationspfad

Damit der Umbau kontrolliert bleibt, ist folgende Reihenfolge sinnvoll.

### Stufe 2a: Neue Typen einfuehren, ohne Verhalten zu aendern

Neue Header-Typen anlegen:

- `HeadSpectrumCommand`
- `HeadCalibrationProfile`
- `PaperExposureProfile` oder ein kompatibler Vorlaeufer

Noch keine Exposure-Fachlogik anschliessen.

### Stufe 2b: Lokale Schalter auf Spektrumsebene abbilden

`Save` und `Fokus` werden nicht mehr direkt als rohe RGB-Farben gedacht, sondern als:

- `safelightRed = 1.0`
- `focusWhite = 1.0`

Die sichtbare Ausgabe bleibt identisch, aber das Modell wird fachlich sauber.

### Stufe 2c: Default-Kopfkalibrierung einbauen

Ein erstes, bewusst einfaches Default-Profil definiert:

- `safelightRed -> RGB rot`
- `focusWhite -> RGB weiss`
- `soft -> RGB gruen`
- `hard -> RGB blau`

mit identitaetsnahen LUTs, einfacher Gamma-Kurve und globalen RGB-Skalierungsfaktoren.

Damit bleibt das Verhalten zunaechst historisch kompatibel.

### Stufe 2d: PaperExposureProfile aus v0.9 portieren

Das historische Papiermodell wird portiert, ohne Kopfkalibrierung und Papierprofil erneut zu vermengen.

### Stufe 2e: ExposureEngine an HeadSpectrumCommand anbinden

Die Exposure-Seite schreibt nicht mehr auf rohe RGB-Werte, sondern liefert nur noch fachliche Lichtauftraege.

### Stufe 2f: Closed-Loop-Schutzpfade nachziehen

Der historische Dosisregler wird fuer Part2 um die fehlenden Sicherheitsreaktionen erweitert:

- Sensor-Watchdog im laufenden Dosisbetrieb
- harter Emergency Shutoff bei Sensorverlust oder thermischer Ueberlast
- explizite Telemetrie fuer Fail-Safe-Zustaende

### Stufe 2g: Thermische Drosselung und UI-Telemetrie anbinden

Nach der funktionalen Closed-Loop-Portierung folgen:

- DS18B20-basierte Drosselung oberhalb der Warnschwelle
- automatische Laufzeitkompensation ueber die Dosisregelung
- LVGL- und EEZ-Telemetrie fuer Dosis, Restzeit und Temperatur

## Was dieses Zielmodell bewusst noch nicht festlegt

Um keine ungesicherten Annahmen zu treffen, legt dieses Dokument noch nicht fest:

- konkrete Kalibrierprozedur fuer die Kopf-LUTs
- ob LUTs dauerhaft in Flash oder in einem gesonderten Kalibrierslot liegen
- ob spaeter pro Segment eine Uniformitaetskorrektur noetig ist
- ob `FocusWhite` spaeter als eigenes Messmodell oder nur als Komfortlicht behandelt wird
- welche Sensorik fuer die Kopfkalibrierung final genutzt wird

Diese Punkte bleiben offen, ohne das Modell selbst zu gefaehrden.

## Designregeln, die aus Schritt 2 verbindlich folgen

1. Papiermodell und Kopfkalibrierung bleiben getrennte Persistenzbereiche.
2. `NeoPixelHead` bleibt reiner Transport- und Matrixcode.
3. Fachlogik erzeugt keine rohen RGB-Werte mehr direkt.
4. Ein spaeterer Kalibrier-Mapping-Schritt ist die einzige Autoritaet fuer die Abbildung von logischen Kanaelen auf physische Ausgabewerte.
5. Historische Regeln wie `Soft`, `Hard`, `Safelight` und `Focus` bleiben als fachliche Begriffe sichtbar und werden nicht im Treiber versteckt.
6. Thermische Begrenzung ist eine Laufzeit-Schutzfunktion des Kopfpfads und gehoert weder ins Papierprofil noch in fachliche Spektrumsollwerte.
7. Sensor-Watchdog und Emergency Shutoff muessen lokal in der Laufzeitarchitektur leben und duerfen nicht vom UI abhaengen.
8. Fotografische EV-/F-Stop-Logik und Messwertformatter werden einmal zentral definiert und von den Modi nur konsumiert.

## Zusammenfassung

Schritt 2 besteht fuer Part2 nicht darin, sofort Gamma oder Kalibrier-LUTs in den bestehenden NeoPixel-Treiber zu schieben.

Der richtige naechste Schritt ist die Modelltrennung:

- `HeadSpectrumCommand` fuer fachliches Soll-Licht
- `HeadCalibrationProfile` fuer die reale Kopfabbildung
- `PaperExposureProfile` fuer papierbezogene Zeit- und Dosislogik

Erst diese Trennung macht es moeglich,

- das historische Verhalten sauber zu erhalten,
- die fotografische Realitaet explizit zu modellieren,
- Closed Loop, Thermik und Fail-Safe als klare Laufzeitdienste darum herum zu bauen,
- und die bisherige Vermischung von Soll-Spektrum, Kopfcharakteristik und Papierprofil in Part2 kontrolliert aufzuloesen.
