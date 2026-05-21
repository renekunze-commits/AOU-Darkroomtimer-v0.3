# Dukatimer-Part2 Papierslot-Persistenz Testleitfaden

Stand: 2026-04-29

## Ziel

Dieser Leitfaden deckt die minimalen Pflichtfaelle fuer MB-06 ab:

- Leerdaten / fehlende Datei
- defekte Persistenzdaten (CRC/Header/Size)
- Migrationsschutz bei Versionsabweichung
- kontrolliertes Rueckschreiben und Fehlerdiagnose

Die Tests sind bewusst reproduzierbar und ohne Logikaenderung im Laufzeitpfad.

## Sichtbare Diagnose im UI

Im Debugscreen wird die Persistenzdiagnose in der DMA-Zeile angezeigt:

- `PS e:<code> d:<detail>`

Codewerte (PaperSlotStorageError):

- `0` None
- `1` StorageUnavailable
- `2` MountFailed
- `3` FileNotFound
- `4` OpenFailed
- `5` ReadFailed
- `6` WriteFailed
- `7` InvalidBlob
- `8` UnsupportedFormatVersion
- `9` InvalidBank
- `10` BufferAllocationFailed
- `11` RenameFailed

Hinweis: Im Bootpfad wird nur bei `FileNotFound` kontrolliert ein
Default-Blob gespeichert. Bei `ReadFailed`, `InvalidBlob`,
`UnsupportedFormatVersion` oder `InvalidBank` bleiben die Defaults nur im RAM
aktiv; der Persistenzfehler bleibt sichtbar und die bestehende Datei wird nicht
still mit Defaults ueberschrieben.

## Voraussetzungen

- Build ist erfolgreich (`pio run -e teensy41`).
- SD-Karte ist im Teensy vorhanden.
- Persistenzpfad: `/paperslots.bin` (temp: `/paperslots.bin.tmp`).
- Fuer Dateimanipulation kann die SD ausserhalb des Geraets gelesen/geschrieben
  werden.

## Testfall 1: Leerdaten / fehlende Datei

1. SD-Karte einlegen und sicherstellen, dass `/paperslots.bin` nicht existiert.
2. Geraet starten.
3. Debugscreen oeffnen und `PS e:<code>` pruefen.
4. SD-Karte wieder auslesen und pruefen, dass `/paperslots.bin` erzeugt wurde.

Erwartung:

- System startet stabil.
- Kein Absturz, keine Blockade des Bootpfads.
- Datei `/paperslots.bin` wird neu erzeugt.
- Nach erfolgreichem Save ist `PS e:0`.

## Testfall 2: Defekter Blob (ungueltige Groesse oder Zufallsdaten)

1. Eine absichtlich ungueltige Datei als `/paperslots.bin` schreiben
   (z. B. sehr kurze Datei oder Zufallsbytes).
2. Geraet starten.
3. Debugscreen pruefen und danach SD-Datei erneut kontrollieren.

Erwartung:

- Defekte Daten werden nicht blind uebernommen.
- Defaultbank wird geladen und, falls moeglich, neu gespeichert.
- Bei erfolgreichem Rueckschreiben endet der Zustand wieder in `PS e:0`.

## Testfall 3: Migrationsschutz (Header/Version)

1. Einen formal gueltigen Blob nehmen und Header absichtlich veraendern
   (Magic oder Formatversion).
2. Datei als `/paperslots.bin` ablegen.
3. Geraet starten und Diagnose sowie Dateizustand kontrollieren.

Erwartung:

- Blob wird als ungueltig verworfen.
- Kein unsicheres Teiluebernehmen alter/inkompatibler Daten.
- Rueckfall auf Defaultbank mit kontrolliertem Rueckschreiben.

## Testfall 4: Schreibfehler sichtbar machen

1. Einen Rueckschreib-Fall erzwingen (z. B. `/paperslots.bin` entfernen).
2. Gleichzeitig einen Schreibfehler provozieren
   (z. B. schreibgeschuetzte SD, fehlgeschlagener Mount, volles Medium).
3. Geraet starten und Debugscreen pruefen.

Erwartung:

- Boot bleibt stabil, auch wenn Save fehlschlaegt.
- `PS e:<code>` ist ungleich `0` und zeigt den Fehlertyp.
- `PS d:<detail>` liefert Zusatzinfo (z. B. Bytezahl bei Read/Write-Fehlern).

## Testfall 5: Stabilitaet gueltiger Daten ueber Reboot

1. Mit gueltigem `/paperslots.bin` booten.
2. Ohne absichtliche Dateiaenderung neu starten.
3. Wiederholt pruefen, dass kein unerwarteter Rueckschreibzyklus erzwungen wird.

Erwartung:

- Gueltige Daten bleiben stabil ueber Reboots.
- Kein unnoetiges Ueberschreiben im Normalfall.
- `PS e:0` bleibt stabil.

## Ergebnisprotokoll (Kurzschema)

Pro Testfall dokumentieren:

- Datum/Uhrzeit
- Testfall-ID
- Beobachteter `PS e`/`PS d`-Wert
- SD-Dateistatus vor/nach Boot
- Ergebnis `PASS` oder `FAIL`
- Kurzer Freitext zu Auffaelligkeiten
