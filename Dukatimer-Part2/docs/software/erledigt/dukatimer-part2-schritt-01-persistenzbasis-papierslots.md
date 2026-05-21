# Dukatimer-Part2 Schritt 1 Abschluss: Persistenzbasis fuer Papierslots

Stand: 2026-04-28

Quelle: ausgelagert aus
`docs/software/dukatimer-part2-step-by-step-abarbeitung-2026-04-28.md`

## Schritt 1 (jetzt): Persistenzbasis fuer Papierslots additiv einziehen

Status: ERLEDIGT (2026-04-28)

Umfang:

- Neue additive Datenbankstruktur fuer Papierslots definieren.
- Versions- und Integritaetsfelder (Version, CRC) fest verankern.
- Serde-Helfer und Validierung als eigener Baustein implementieren.
- Noch keine Umverdrahtung bestehender Laufzeitlogik.

Abnahmekriterien:

- Neue Dateien kompilieren im Teensy-Build.
- Keine bestehende Datei verliert Verhalten.
- Kein bestehender Ablaufpfad wird semantisch geaendert.

Umsetzungsergebnis:

- Neue Datenbankstruktur: src/teensy/PaperSlotBank.h
- Neues Persistenz-Codec-Modul (Version/CRC + Validate/Serialize/Parse):
  - src/teensy/PaperSlotPersistenceCodec.h
  - src/teensy/PaperSlotPersistenceCodec.cpp
- Build-Validierung: pio run -e teensy41 erfolgreich.
