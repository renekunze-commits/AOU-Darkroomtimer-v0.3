# Dukatimer-Part2 LVGL-, EEZ- und DMA-Basis

## Ziel

Diese Basis legt den ersten belastbaren UI-Pfad fuer den Teensy 4.1 fest:

- LVGL ist die Laufzeitbibliothek fuer die lokale Hauptoberflaeche.
- EEZ Studio ist der vorgesehene GUI-Designer und Codegenerator.
- Das SPI-TFT wird ueber `ILI9488_t3_mm` als Projekttypname angebunden.
- Der Displaypfad nutzt jetzt LVGL-Partial-Flushes mit zwei kleineren Draw-Buffern und einem asynchronen SPI-DMA-Transfer pro Dirty-Rect.

## Warum dieser Pfad

Die aktuelle Displaybibliothek fuer den Teensy 4.1 bietet uns zwei relevante Pfade:

- Vollbild-Framebuffer mit `updateScreenAsync(false)` fuer asynchronen DMA-Transfer
- `writeRect(...)` fuer fensterweise SPI-Updates

Fuer ein SPI-Panel ist der zweite Pfad fuer LVGL der wichtigere, weil kleine Dirty-Areas damit klein bleiben und nicht jedes Cursorblinken ein Vollbildtransfer wird.

Die neue Erweiterung liegt genau zwischen beiden Welten:

- LVGL rendert nur Teilrechtecke.
- Vor dem Versand werden diese RGB565-Pixel in einen DMA-faehigen RGB888-Transferpuffer gewandelt.
- Der Teensy-SPI-DMA sendet diesen Puffer asynchron ueber `SPI.transfer(..., EventResponderRef)`.

## Wichtige technische Grenze

Das ILI9488 haengt hier nicht an einem parallelen LCD-Controller, sondern an einem seriellen SPI-Pfad.
Dadurch bleibt trotz DMA der Bus die physische Obergrenze.

Folge:

- DMA entlastet die CPU nur dann sinnvoll, wenn nicht staendig ganze Frames geschoben werden.
- Ein 480x320-Vollbildtransfer ueber SPI bleibt auch mit DMA buslimitiert.
- Fuer typische UI-Aenderungen ist es wichtiger, nur geaenderte Rechtecke zu senden.

Deshalb ist die aktuelle Entscheidung bewusst:

- `direct_mode` ist ausgeschaltet.
- LVGL nutzt zwei kleinere Ping-Pong-Drawwbuffer mit aktuell 40 Zeilen Hoehe.
- `flush_cb` sendet nur das von LVGL invalidierte Rechteck.
- Der eigentliche SPI-Transfer laeuft asynchron und signalisiert `lv_disp_flush_ready()` erst nach DMA-Abschluss.

Das reduziert den SPI-Traffic massiv und ist fuer ein SPI-TFT die robustere Standardarchitektur.

## Aktuelle DMA-Grenze

Die aktuelle `ILI9488_t3`-Bibliothek bietet von Haus aus DMA nur fuer den Vollbild-Framebufferpfad. Deshalb wurde lokal ein eigener Async-Rect-Pfad ergaenzt.

Das bedeutet aktuell:

- Partial-Flushes laufen asynchron.
- CPU und SPI koennen sich fuer Dirty-Rects jetzt tatsaechlich ueberlappen.
- Die Pixeldaten muessen vor dem Transfer von RGB565 nach RGB888 expandiert werden, weil das Panel im 18-Bit-Modus initialisiert ist.

Der verbleibende Preis ist damit nicht mehr der Vollbildtransfer, sondern die Vorab-Konvertierung in den DMA-Puffer.

## Aktuelle Architektur

### 1. `TftDisplayHal`

Kapselt:

- Backlight
- Displayinitialisierung
- optionalen Framebuffer-Modus
- Vollbild-DMA-Pfad fuer spaetere Spezialfaelle
- fensterweisen Async-Rect-Pfad fuer LVGL-Partial-Flushes

### 2. `LvglUi`

Kapselt:

- `lv_init()`
- Registrierung von Display- und Touch-Treiber
- zwei kleinere LVGL-Draw-Buffers im Ping-Pong-Betrieb
- Partial-Flushes ueber invalidierte Rechtecke
- `lv_disp_flush_ready()` erst nach EventResponder-Callback des DMA-Transfers
- eine erste Platzhalteroberflaeche

### 3. `TouchSampler`

Bleibt die Rohdatenschicht fuer den XPT2046. Die LVGL-Input-Bridge mappt diese Rohwerte vorerst linear auf die Displayflaeche.

## EEZ-Studio-Nahtpunkt

Die jetzt aktive LVGL-Platzhalteroberflaeche ist nur die technische Bring-up-Schicht. Sie ist absichtlich so aufgebaut, dass sie spaeter durch EEZ-generierte Screens ersetzt werden kann.

Der vorgesehene Ablauf ist:

1. In EEZ Studio ein `LVGL project` auf Basis von LVGL 8.x anlegen.
2. Die generierten Screens/Actions als eigene Quellgruppe unter der Teensy-UI-Struktur ablegen.
3. Die Platzhalter-Screenerzeugung in `LvglUi::createPlaceholderScreen()` durch den Einstiegspunkt des generierten EEZ-Codes ersetzen.
4. Applikationsdaten nicht in EEZ-Widgets streuen, sondern weiterhin als normierte Snapshots und spaeter als Presenter/ViewModel in die UI-Schicht geben.

## Aktuelle Restgrenze

Der neue Pfad entfernt den groessten Engpass, aber nicht jede Kostenstelle:

- jeder Flush braucht weiterhin eine RGB565-zu-RGB888-Konvertierung in einen DMA-Puffer
- pro SPI-Transfer ist weiterhin nur ein Rechteck gleichzeitig aktiv
- fuer noch mehr Durchsatz waere ein Treiberpfad attraktiv, der Pixel direkt in einem panelgerechten DMA-Format rendert

Fuer den aktuellen Projektstand ist das trotzdem der richtige Kompromiss, weil der Bus geschont wird und EEZ/LVGL bereits auf einer belastbaren Async-Basis laufen koennen.

## Naechste sinnvolle Schritte

1. Touch kalibrieren und die Konstanten der LVGL-Input-Bridge ausmessen.
2. Encoder von positionsbasiert auf normierte Events umstellen.
3. Erste EEZ-Studio-Projektstruktur anlegen und den Platzhalter-Screen ersetzen.
4. ViewModel-/Presenter-Schnitt definieren, damit Fachlogik und generierte UI sauber getrennt bleiben.

Querschnittsvorgabe fuer die spaetere UI-Schicht:

- mehrfach genutzte EV-/F-Stop-Logik und Messwertformatter werden nicht in EEZ-Screens oder Widgets dupliziert
- echte Messworkflows konsumieren eine gemeinsame EV-plus-Lux-Darstellung aus Presenter- oder Formatter-Schichten
- reine Runtime-Telemetrie bleibt in ihren physischen Grundeinheiten und wird nicht auf UI-Ebene ad hoc in EV uebersetzt
