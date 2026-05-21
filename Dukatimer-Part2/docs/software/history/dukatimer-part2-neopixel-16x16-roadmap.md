# Dukatimer-Part2 NeoPixel 16x16 Roadmap

Stand: 2026-04-22

## Ziel

Diese Notiz leitet den naechsten Implementierungsschritt fuer den NeoPixel-Lichtkopf aus drei Quellen ab:

- Part2-Dokumentation als verbindliche Architektur- und Hardwarebasis
- v0.3 als Verhaltensreferenz fuer Lichtprioritaet und Belichtungsrandbedingungen
- v0.9 als Strukturreferenz fuer zentrale Hardwareautoritaet und saubere Zustandsgrenzen

Der Fokus liegt bewusst noch nicht auf Fachmodi oder UI-Effekten, sondern auf einer belastbaren 16x16-Lichtkopfarchitektur fuer den Teensy.

## Ausgangslage

### Verbindlich aus Part2

- Der Teensy 4.1 ist fuer die direkte Ansteuerung der NeoPixel-Kanaele zustaendig.
- Die lokale Schalterlogik ist bereits fachlich festgelegt:
  - `Fokus` -> Weiss 100 Prozent
  - `Save` -> nur Rot
  - `Room` -> SSR fuer Raumlicht
  - `Save` hat Prioritaet vor `Fokus`
  - `SaveLatch` bleibt verbindlich erhalten
- Die aktuelle Hardwaredokumentation zeigt vier gepufferte Datenleitungen `NEO_1..NEO_4` ueber einen `74HCT125` zum Lichtkopf.
- Aktuelle Hardwarebasis fuer den Bring-up-Stand: eine `16 x 16`-Matrix auf `NEO_1`.
- Geplanter modularer Ausbau: vier einzelne `8 x 8`-Module ueber `NEO_1..NEO_4`.

### Gesichert aus dem aktuellen Firmwarestand

- `LightController` kapselt bereits die richtige Schalterhierarchie und den `SaveLatch`.
- Die physische Lichtausgabe ist im Moment nur Bring-up-Code mit vier einzelnen Testpixeln.
- Eine echte 16x16-Abstraktion existiert noch nicht.

### Historisch relevant

- v0.3 hatte bereits eine zentrale Render-Pipeline mit Prioritaet `Belichtung > Save > Fokus > Aus`.
- v0.3 beruecksichtigte bei der Belichtungsengine explizit die Update-Laufzeit eines 256-Pixel-NeoPixel-Pfads.
- v0.9 fuehrte die Lichtlogik in eine zentrale HAL mit State-Caching und einer klaren `updateNeoPixels(r, g, b)`-Autoritaet ueber.
- Historisch war die Nutzlast logisch bereits ein 256-Pixel-Lichtkopf, auch wenn die neue Part2-Hardware elektrisch anders vorbereitet ist.

## Zentrale Schlussfolgerung

Der naechste Schritt ist nicht, die 16x16-Matrix direkt in `LightController` hineinzubauen.

Stattdessen braucht Part2 jetzt eine getrennte Lichtkopf-Schicht zwischen:

- fachlicher Lichtentscheidung
- logischem 16x16-Bild
- physischem NeoPixel-Transport

Nur so bleibt die Firmware offen fuer die noch nicht abschliessend dokumentierte Kopfverdrahtung.

## Offene Hardwareentscheidung, die der Code nicht vorwegnehmen darf

Die aktuelle Dokumentation und der aktuelle Hardwarestand ergeben fuer Part2 zwei relevante Betriebsformen, die dieselbe Softwarebasis tragen soll:

1. aktueller Bring-up: eine einzelne `16 x 16`-Matrix auf genau einem Datenkanal
2. Zielausbau: vier `8 x 8`-Segmente auf `NEO_1..NEO_4`
3. je Segment moegliche Unterschiede in Laufrichtung oder Rotation
4. optional bestueckbare oder anders orientierte Tiles statt fest angenommener Quadrantenlogik

Deshalb muss die Software zuerst ein logisches 16x16-Modell bekommen und die physische Zuordnung konfigurierbar halten.

## Zielarchitektur

### 1. `LightController` bleibt die fachliche Autoritaet

`LightController` behaelt:

- Rohschalterzustand
- `SaveLatch`
- Prioritaetslogik
- abgeleitete Lichtmodi

`LightController` soll kuenftig aber nicht mehr direkt `NeoPixelBus`-Objekte kennen.

Stattdessen gibt er einen normierten Lichtbefehl an die naechste Schicht weiter, zum Beispiel:

- `Off`
- `SafelightRed`
- `FocusWhite`
- `ExposureRgb`
- spaeter `Error`, `Blackout`, `CalibrationPattern`

Wichtig fuer die Schichtgrenze:

- fotografische EV-/F-Stop-Logik gehoert nicht in `LightController`, `NeoPixelHead` oder die Topologieebene
- mehrfach genutzte Belichtungswert-Logik wird oberhalb des Kopfpfads einmal zentral definiert und von den Modi nur benutzt

### 2. Neue Schicht `NeoPixelHead`

Diese neue Schicht bekommt die alleinige Verantwortung fuer den 16x16-Lichtkopf.

Sie kapselt:

- logische Aufloesung `16 x 16`
- physische Topologie
- Pixel-Mapping `x/y -> Strip-Index`
- ein oder mehrere Datenkanaele
- Dirty-Tracking und `present()`
- optionale Helligkeitsbegrenzung und spaetere Gamma-Korrektur

Minimales API-Ziel fuer den ersten Schritt:

- `begin()`
- `clear()`
- `fill(RgbColor color)`
- `setPixel(uint16_t x, uint16_t y, RgbColor color)`
- `present()`
- `width()` / `height()`

### 3. Neue Schicht `NeoPixelTopology`

Die physische Verdrahtung darf nicht als verstreute Speziallogik im Treiber landen.

Stattdessen bekommt sie eine explizite Konfiguration, zum Beispiel:

- Breite und Hoehe
- Anzahl der Datenkanaele
- Pixel pro Kanal
- Layout-Typ `row-major`, `row-major alternating`, `column-major`, Tile-Mosaic
- Ursprung und Rotation je Segment

Wichtig: Die aktuell eingebaute `NeoPixelBus`-Version bringt bereits Topology-Helfer wie `NeoTopology`, `NeoTiles` und `NeoMosaic` mit. Diese sollen zuerst genutzt werden, bevor eigene feste Mapping-Tabellen entstehen.

### 4. Neue Schicht `HeadFrame` oder `LightScene`

Zwischen Lichtentscheidung und Treiber sollte ein kleiner, expliziter Datentyp liegen.

Er beschreibt nicht Pins, sondern das gewuenschte logische Lichtbild, zum Beispiel:

- Vollflaeche Rot
- Vollflaeche Weiss
- Vollflaeche RGB fuer Belichtung
- spaeter Testmuster oder Kalibrierbilder

Damit bleibt der Uebergang von einfachem Vollflaechenlicht zu spaeteren Mustern oder Zonen offen.

## Historische Regeln, die erhalten bleiben muessen

### Zentrale Prioritaetspipeline

Das Altverhalten war richtig und soll fuer Part2 erhalten bleiben:

1. aktive Belichtung oder ein hoeherer Sicherheitszustand ueberschreibt alles
2. `Save` hat Vorrang vor `Fokus`
3. `Fokus` ist nur gueltig, wenn weder `Save` noch ein Lockout aktiv ist
4. sonst Licht aus

Diese Regel muss an genau einer Stelle im Code leben.

### Kein ungefiltertes Direkt-Schreiben aus Fachmodi

Wie in v0.9 soll es genau eine Autoritaet fuer die physische Lichtausgabe geben.

Das bedeutet:

- Fachlogik schreibt nicht direkt auf `NeoPixelBus`
- UI schreibt nicht direkt auf `NeoPixelBus`
- Belichtungsengine schreibt nicht direkt auf `NeoPixelBus`

Alle drei muessen ueber dieselbe Licht- oder Head-Schnittstelle gehen.

### Timing-Komponente nicht aus v0.3 hart uebernehmen

Der historische Wert fuer die 256-Pixel-Latch-Zeit ist wichtig als Warnsignal, aber nicht als Part2-Konstante.

Grund:

- die Part2-Hardware kann ein anderes Lane-Modell haben
- die Teensy-Ansteuerung kann sich zeitlich von der alten ESP32-Ansteuerung unterscheiden
- bei vier Kanaelen kann die Laufzeit anders ausfallen als bei einer 256er-Kette

Fuer Part2 gilt deshalb:

- Update-Laufzeit messen, nicht erraten
- erst danach eine Kompensation in die spaetere ExposureEngine einspeisen

## Empfohlene Umsetzungsreihenfolge

### Stufe 1: Architektur sauberziehen

Ziel:
Die heutige Testpixel-Loesung durch eine echte Lichtkopf-Abstraktion ersetzen, ohne schon Spektral- oder Matrixlogik vorwegzunehmen.

Konkrete Arbeit:

1. `NeoPixelBus` aus `LightController` herausziehen.
2. Neue Klasse `NeoPixelHead` anlegen.
3. `LightController` nur noch einen normierten Lichtbefehl oder eine `LightScene` erzeugen lassen.
4. `main.cpp` so verdrahten, dass `LightController` und `NeoPixelHead` getrennte Rollen haben.

Abnahme:

- `Fokus`, `Save`, `Room` verhalten sich unveraendert
- `SaveLatch` bleibt unveraendert korrekt
- keine direkte NeoPixel-Ansteuerung mehr aus `LightController`

### Stufe 2: Ein logisches 16x16-Modell schaffen

Ziel:
Der Lichtkopf wird als Matrix statt als roher Strip beschrieben.

Konkrete Arbeit:

1. `NeoPixelHeadConfig` mit `width = 16`, `height = 16` einfuehren.
2. Buffer fuer 256 logische Pixel anlegen.
3. Erste einheitliche Vollflaechenoperationen implementieren: `clear`, `fill`, optional `fillRect`.
4. Mapping ueber eine eigene `NeoPixelTopology` oder die vorhandenen NeoPixelBus-Topologies kapseln.

Abnahme:

- Vollflaeche Rot, Weiss und Aus funktionieren ueber das neue Matrixmodell
- keine Fachlogik kennt physische Strip-Indizes

### Stufe 3: Topologie-Bestimmung als eigener Bring-up-Slice

Ziel:
Die reale Verdrahtung des 16x16-Kopfes sicher ermitteln, ohne die restliche Architektur umzubauen.

Konkrete Arbeit:

1. Testmuster fuer Ecken, Kanten und Laufrichtung implementieren.
2. Ermitteln, ob der Kopf im aktuellen Stand als eine `16 x 16`-Matrix auf `NEO_1` laeuft und wie der spaetere `4 x 8 x 8`-Ausbau segmentiert werden soll.
3. Orientierung pro Segment dokumentieren.
4. Erst danach die finale Topologiekonfiguration festschreiben.

Abnahme:

- jede Ecke des 16x16-Felds ist eindeutig adressierbar
- Segmentgrenzen und Laufrichtung sind dokumentiert
- die finale Zuordnung ist als Konfiguration abbildbar, nicht als Sonderfall im Fachcode

### Stufe 4: Historisches Verhalten auf die neue Matrix heben

Ziel:
Das alte Lichtverhalten auf dem neuen 16x16-Head wiederherstellen.

Konkrete Arbeit:

1. `SafelightRed` als gleichmaessige rote Vollflaeche implementieren.
2. `FocusWhite` als gleichmaessige weisse Vollflaeche implementieren.
3. `ExposureRgb` als gleichmaessige spektrale Vollflaeche implementieren.
4. Dirty-Flag einfuehren, damit `present()` nur bei Aenderungen laeuft.

Abnahme:

- Part2 erreicht wieder das historische Kernverhalten `Rot / Weiss / Aus / RGB`
- keine unnoetigen `Show()`-Aufrufe im Hauptloop

### Stufe 5: Laufzeit, Strom und Schutzgrenzen messen

Ziel:
Der neue 16x16-Pfad wird elektrisch und zeitlich vermessen, bevor spaetere Belichtungslogik darauf aufbaut.

Konkrete Arbeit:

1. `present()`-Dauer in Mikrosekunden messen.
2. Laufzeit fuer mindestens diese Faelle erfassen:
   - Aus -> Weiss
   - Weiss -> Rot
   - Rot -> RGB
   - unveraenderter Frame
3. Strom- und Spannungseinbruch unter realer Last pruefen.
4. Bis zur Messfreigabe konservative Helligkeitslimits setzen.

Abnahme:

- gemessene Timingwerte liegen vor
- Software kennt sichere Standard-Helligkeiten
- Part2 verletzt die dokumentierten Versorgungsrisiken nicht blind

### Stufe 6: Erst dann an ExposureEngine anbinden

Ziel:
Die spaetere Belichtungsengine nutzt denselben Head-Pfad, aber auf gemessener Basis.

Konkrete Arbeit:

1. eine zentrale API fuer `ExposureRgb` definieren
2. Update-Laufzeit als Messwert in die spaetere Shutoff-Kompensation einbeziehen
3. sicherstellen, dass Not-Aus weiterhin den Lichtkopf zentral und deterministisch abschaltet

Abnahme:

- Belichtungslogik benutzt keine Sonderpfade am Treiber vorbei
- Timing-Kompensation basiert auf Messung statt auf geerbten Konstanten

## Was ich explizit nicht empfehlen wuerde

Folgende Abkuerzungen wuerden jetzt spaeter teuer werden:

1. die 16x16-Zuordnung direkt in `LightController` fest verdrahten
2. sofort vier Datenkanaele fest annehmen, obwohl die Kopf-Topologie noch nicht dokumentiert ist
3. die historische `7.7 ms`-Annahme ungeprueft in Part2 uebernehmen
4. Fachlogik oder UI direkt auf `NeoPixelBus` schreiben lassen
5. schon jetzt Spezialeffekte oder komplexe Pixelanimationen bauen

## Konkrete naechste Tickets

1. `NeoPixelHead` als neue Klasse einfuehren und `LightController` davon entkoppeln.
2. Ein logisches 16x16-Buffer-Modell mit konfigurierbarer Topologie anlegen.
3. Ein Hardware-Bring-up-Muster fuer Ecken, Reihen, Spalten und Segmentgrenzen bauen.
4. Die reale Kopfverdrahtung dokumentieren und daraus die finale Topologie festziehen.
5. Danach erst `Safelight`, `Focus` und `ExposureRgb` auf die neue Matrix routen.

## Entscheidender Nutzen dieser Reihenfolge

Mit diesem Ablauf wird Part2 nicht auf eine zufaellige Verdrahtungsannahme festgenagelt. Gleichzeitig bleibt das historisch bewaehrte Verhalten erhalten:

- zentrale Lichtautoritaet
- korrekte Prioritaet und `SaveLatch`
- Vorbereitung fuer spaetere Belichtungsmodi
- klare Trennung zwischen Lichtfachlogik, Matrixmodell und physischem Treiber
