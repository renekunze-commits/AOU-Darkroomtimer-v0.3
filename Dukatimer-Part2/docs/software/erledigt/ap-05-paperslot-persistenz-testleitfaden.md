# AP-05 Paper Slot Persistenz Testleitfaden

## Ziel

Der Leitfaden prueft die AP-05-Integration fuer:

- Slot-Bank-Laden und Recovery beim Start
- format/version/crc/error-Strategie
- Sichtbarkeit von Persistenzfehlern in Runtime-Diagnosewerten

## Voraussetzungen

- Build fuer `teensy41` ist aktuell und flashbar.
- SD-Karte ist eingelegt und beschreibbar.
- Persistenzdatei: `/paper_slots.bin`
- Temp-Datei bei Save: `/paper_slots.tmp`

## Fehlercode-Referenz

`PaperSlotStorageError` (numerische Codierung):

- 0: `None`
- 1: `StorageUnavailable`
- 2: `MountFailed`
- 3: `FileNotFound`
- 4: `OpenFailed`
- 5: `ReadFailed`
- 6: `WriteFailed`
- 7: `InvalidBlob`
- 8: `UnsupportedFormatVersion`
- 9: `InvalidBank`
- 10: `BufferAllocationFailed`
- 11: `RenameFailed`

Der Detailwert liegt in `paperSlotStorageErrorDetail`.
Bei `UnsupportedFormatVersion` enthaelt `detail` die gelesene Formatversion.
Bei generischen Parse-Fehlern enthaelt `detail` den `PaperSlotBlobParseError`-Wert.

Boot-Recovery-Politik:

- Nur `FileNotFound` wird beim Boot automatisch mit einem Default-Blob initialisiert.
- `ReadFailed`, `InvalidBlob`, `UnsupportedFormatVersion` und `InvalidBank` bleiben absichtlich als sichtbarer Fehlerpfad stehen.
- Dadurch werden potenziell echte, aber transient unlesbare oder korrupte Nutzdaten nicht still mit Defaults ueberschrieben.

## Sichtbarkeit im UI

Diagnose wird in den Presenter-Strings ausgegeben:

- Gateway-Zeile:
  - `PS <label>(<code>/<detail>)`
  - `SLOT <active>/<count> <CAL|RAW> <name>`
- DMA-Zeile:
  - `PS <label> e:<code> d:<detail>`
  - `Slot <active>/<count> <CAL|RAW> <name>`

## Testszenarien

### 1) Leerdaten / Datei fehlt

Schritte:

1. `/paper_slots.bin` loeschen.
2. System neu starten.
3. Beobachten, ob Default-Bank geladen und gespeichert wird.

Erwartung:

- Start laeuft weiter (kein Blocker).
- `activeSlot` ist gueltig.
- Slot-Count ist gueltig.
- Nach Recovery endet Status bei `None` (0).

### 2) Defekte Daten / CRC-Fehler

Schritte:

1. Gueltigen Blob erzeugen.
2. Einzelnes Byte im Payload kippen (CRC danach ungueltig).
3. System neu starten.

Erwartung:

- Blob wird verworfen.
- Fehlerpfad meldet `InvalidBlob` (7), Detail zeigt Parse-Fehler.
- Recovery auf Defaults und Rueckschreiben erfolgen.
- Danach Status `None` (0).

### 3) Unsupported Format Version

Schritte:

1. Gueltigen Blob erzeugen.
2. Header-Feld `formatVersion` auf nicht unterstuetzten Wert setzen.
3. System neu starten.

Erwartung:

- Laden scheitert mit `UnsupportedFormatVersion` (8).
- `detail` entspricht der gelesenen Version.
- Recovery-Pfad initialisiert Defaults und schreibt kompatibles Format zurueck.
- Danach Status `None` (0).

### 4) Inhaltlich ungueltige Bank

Schritte:

1. Blob mit formal gueltigem Header/Groesse/CRC erzeugen.
2. Payload so manipulieren, dass `validateBank` fehlschlaegt.
3. System neu starten.

Erwartung:

- Laden scheitert mit `InvalidBank` (9) oder `InvalidBlob` (7) je nach Parse-Stufe.
- Recovery auf Defaults greift.
- Danach Status `None` (0).

### 5) Trailing Bytes / Groessenabweichung

Schritte:

1. Datei mit korrektem Blob plus zusaetzlichem Byte erzeugen.
2. System neu starten.

Erwartung:

- Datei wird als `ReadFailed` (5) verworfen.
- Recovery auf Defaults und Rueckschreiben.
- Danach Status `None` (0).

### 6) SD-Write blockiert

Schritte:

1. SD in read-only/fehlerhaften Zustand versetzen.
2. Recovery-Fall triggern (z. B. Datei loeschen).
3. System neu starten.

Erwartung:

- Save meldet `WriteFailed` (6) oder `RenameFailed` (11).
- Runtime bleibt stabil, Diagnose bleibt sichtbar.

## Regressionscheck

- Aktiver Slot bleibt in `[0, slotCount-1]`.
- Bei unkalibriertem aktivem Slot wird auf ersten kalibrierten Slot normalisiert (falls vorhanden).
- Keine Aenderung am Ablauf der Mode-Workflows ausser zusaetzlicher Query-Service-Verfuegbarkeit.
- Presenter liefert weiterhin Strings ohne Buffer-Ueberlauf.
