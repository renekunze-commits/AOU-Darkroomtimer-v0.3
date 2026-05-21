# AP-07 SG-Hardware-Abnahmeprotokoll

Stand: 2026-04-30

## Ziel

Dieses Protokoll deckt die vier minimalen Pflichtfaelle fuer die reale
AP-07-Abnahme am Geraet ab:

- Grade `0.0` als Soft-only-Fall
- Grade `5.0` als Hard-only-Fall
- `fixedGrade`-Profil als fachlich gesperrte Gradation
- manueller Soft-/Hard-Edit mit anschliessendem Gradationswechsel

Die Faelle pruefen gezielt, dass AP-07 den bestehenden SG-Ablauf fachlich
fuellt, ohne die bestehende Execution-State-Maschine zu ersetzen.

## Geltungsbereich

Die Abnahme beobachtet den Laufzeitpfad in:

- `src/teensy/SplitgradeWorkflow.*`
- `src/teensy/main.cpp` (Execution-Command-Wiring zur ExposureEngine)
- `src/teensy/UiPresenter.*`
- `src/teensy/LvglUi.*`
- `src/teensy/InputRouterPolicy.*`

## Bedien- und UI-Beobachtungspunkte

Lokale Bedienrollen:

- `Enc1`: primaerer Wert
- `Enc2`: sekundaerer Wert
- `Enc3 drehen`: Kontextwechsel zwischen `TARGETS`, `GRADE`, `MODE`
- `Enc3 druecken`: `Confirm`
- `Start-Taster`: `Start`
- `Enc2 lang`: `Undo`

Im Debug-/SG-Screen sind fuer die Abnahme relevant:

- Headerzeile: `SG <panel>  EXEC <state>  CTRL <mode>`
- Targetzeile: `TARGET S <soft>  H <hard>  G <grade>  <CLEAN|DIRTY>`
- Overlayzeile:
  - Idle: `BEREIT: START ZUM BELICHTEN` oder `PARAMETER GEAENDERT`
  - Confirm: `CONFIRM: FILTER WECHSELN, DANN CONFIRM ODER START`
  - Wait: `WAIT <phase> - NUR UNDO ZUM ABBRUCH AKTIV`
  - Done: `BELICHTUNG FERTIG`
- Diagnose in `ModeInfo`:
  - `EXEC <state>`
  - `F <focus>`
  - `M <modal>`
  - `G <guard>`

Modal-/Guard-Erwartung aus AP-06:

- aktive Belichtung: `M WAIT`, `G WAIT`
- Filterwechsel-Confirm: `M CONF`, `G CONF`
- normaler Idle-Zustand: `M NONE`, `G OPEN`

Farbbeobachtung des Overlays:

- `WAIT`: blau
- `CONFIRM`: amber/braun
- `BELICHTUNG FERTIG`: gruen

## Voraussetzungen

- Firmware mit AP-07-Stand ist auf dem Teensy geflasht.
- Der AP-07-Build fuer `teensy41` ist erfolgreich.
- Der Test wird in optisch sicherer Umgebung gefahren:
  - kein Papier im vergroesserungsrelevanten Pfad
  - Lichtkopf zeigt in sichere Richtung oder auf Dummyflaeche
  - Bedienende kennen den `Undo`-Abbruchpfad
- Fuer die Faelle 1, 2 und 4 ist ein nicht-`fixedGrade`-Profil aktiv.
- Fuer Fall 3 ist ein separates aktives `fixedGrade`-Profil vorbereitet.

Hinweis zur Profilvorbereitung:

- Wenn es im aktuellen UI noch keinen produktiven Slot-/Profilumschalter gibt,
  werden die Faelle ueber vorbereitete SD-Daten oder getrennte Test-Boots mit
  definiertem aktivem Profil gefahren.
- Fuer Fall 1, 2 und 4 reicht ein kalibriertes, nicht-fixes SG-Profil. Das
  mit AP-05 angelegte Standardprofil ist dafuer geeignet.
- Fuer Fall 3 wird ein Profil mit mindestens diesen Eigenschaften benoetigt:
  - `fixedGrade=true`
  - bekannter `fixedGradeValue` (empfohlen `3.0`)
  - bekannter Basiswert `kBw`

## Allgemeines Messschema

Vor jedem Testfall im Idle einmal dokumentieren:

- aktives Profil / Slotname
- `CTRL`-Modus
- Startwert `S0`
- Startwert `H0`
- Basissumme `B = S0 + H0`
- Startgradation `G0`

Toleranz fuer UI-Anzeige:

- Wegen einstelliger Nachkommastelle im Screen gilt fuer sichtbare Werte eine
  Toleranz von `+-0.2`.

## Testfall AP07-HW-01: Grade 0 als Soft-only

### AP07-HW-01 Schritte

1. Mit nicht-fixem SG-Profil booten und SG-Screen oeffnen.
2. Sicherstellen, dass `CTRL TIME` aktiv ist.
3. Mit `Enc3` ins Panel `GRADE` wechseln.
4. Mit `Enc1` und falls noetig `Enc2` die Gradation auf `0.0` stellen.
5. Targetzeile und `ModeInfo` dokumentieren.
6. Mit dem `Start`-Taster die Belichtung starten.
7. Header, Overlay und `ModeInfo` waehrend der Laufzeit beobachten.
8. Ende der Belichtung abwarten und Overlayzustand dokumentieren.

### AP07-HW-01 Erwartung

- Vor dem Start gilt:
  - `TARGET S ~= B`
  - `TARGET H ~= 0.0`
  - `G 0.0`
  - Status `DIRTY`
- Zulaessige SG-Zustandsfolge:
  - `IDLE -> ARM-S -> RUN-S -> DONE`
- Nicht zulaessig in diesem Fall:
  - `WAIT-F`
  - `ARM-H`
  - `RUN-H`
  - Confirm-Overlay
- Waehrend aktiver Belichtung gilt:
  - Overlay zeigt `WAIT <phase> - NUR UNDO ZUM ABBRUCH AKTIV`
  - `ModeInfo` zeigt `M WAIT` und `G WAIT`
- Nach Abschluss gilt:
  - Overlay zeigt `BELICHTUNG FERTIG`
  - gruenes Abschluss-Overlay ist sichtbar

## Testfall AP07-HW-02: Grade 5 als Hard-only

### AP07-HW-02 Schritte

1. Mit demselben nicht-fixen SG-Profil erneut in den SG-Screen gehen.
2. Mit `Enc3` ins Panel `GRADE` wechseln.
3. Mit `Enc1` und falls noetig `Enc2` die Gradation auf `5.0` stellen.
4. Targetzeile und `ModeInfo` dokumentieren.
5. Mit dem `Start`-Taster die Belichtung starten.
6. Header, Overlay und `ModeInfo` waehrend der Laufzeit beobachten.
7. Ende der Belichtung abwarten und Overlayzustand dokumentieren.

### AP07-HW-02 Erwartung

- Vor dem Start gilt:
  - `TARGET S ~= 0.0`
  - `TARGET H ~= B`
  - `G 5.0`
  - Status `DIRTY`
- Zulaessige SG-Zustandsfolge:
  - `IDLE -> ARM-H -> RUN-H -> DONE`
- Nicht zulaessig in diesem Fall:
  - `ARM-S`
  - `RUN-S`
  - `WAIT-F`
  - Confirm-Overlay
- Waehrend aktiver Belichtung gilt:
  - Overlay zeigt `WAIT <phase> - NUR UNDO ZUM ABBRUCH AKTIV`
  - `ModeInfo` zeigt `M WAIT` und `G WAIT`
- Nach Abschluss gilt:
  - Overlay zeigt `BELICHTUNG FERTIG`

## Testfall AP07-HW-03: fixedGrade-Profil

### AP07-HW-03 Schritte

1. Geraet mit vorbereitetem `fixedGrade`-Profil booten.
2. SG-Screen oeffnen und Startwerte dokumentieren.
3. Mit `Enc3` ins Panel `GRADE` wechseln.
4. Mehrfach mit `Enc1` und `Enc2` versuchen, die Gradation zu veraendern.
5. Header, Targetzeile und `ModeInfo` vor und nach den Drehbewegungen dokumentieren.
6. Mit dem `Start`-Taster die Belichtung starten.
7. Header, Overlay und `ModeInfo` waehrend der Laufzeit beobachten.

### AP07-HW-03 Erwartung

- Bereits im Idle gilt:
  - `G` entspricht exakt dem Profilwert `fixedGradeValue`
  - `TARGET S ~= B`
  - `TARGET H ~= 0.0`
  - Status bleibt `CLEAN`
- Die Bedienung im Panel `GRADE` veraendert weder:
  - die sichtbare Gradation
  - den Soft-Wert
  - den Hard-Wert
  - den `DIRTY`/`CLEAN`-Status
- Zulaessige SG-Zustandsfolge beim Start:
  - `IDLE -> ARM-S -> RUN-S -> DONE`
- Nicht zulaessig in diesem Fall:
  - `WAIT-F`
  - `ARM-H`
  - `RUN-H`
  - Confirm-Overlay

## Testfall AP07-HW-04: manueller Soft-Edit plus anschliessender Gradationswechsel

### AP07-HW-04 Schritte

1. Mit nicht-fixem SG-Profil booten und SG-Screen oeffnen.
2. Im Panel `TARGETS` die Startwerte `S0`, `H0`, `B0 = S0 + H0` und `G0` dokumentieren.
3. Mit `Enc1` genau einen Schritt nach rechts drehen.
4. Die neuen Werte `S1`, `H1`, `B1 = S1 + H1` dokumentieren.
5. Mit `Enc3` ins Panel `GRADE` wechseln.
6. Die Gradation auf `0.0` stellen und Werte dokumentieren.
7. Die Gradation anschliessend wieder auf den urspruenglichen Wert `G0`
   zurueckstellen und Werte erneut dokumentieren.

### AP07-HW-04 Erwartung

- Direkt nach dem einen Soft-Edit gilt:
  - `S1 > S0`
  - `H1 ~= H0`
  - `B1 > B0`
  - Status `DIRTY`
- Fachlicher Richtwert fuer den einen EV-Schritt:
  - `S1 ~= S0 * 2^(1/3)`
  - als UI-Naeherung also etwa `S1 ~= S0 * 1.26`
- Nach Gradationswechsel auf `0.0` gilt:
  - `TARGET S ~= B1`
  - `TARGET H ~= 0.0`
- Nach Rueckkehr auf den urspruenglichen Gradationswert `G0` gilt:
  - `TARGET S ~= S1`
  - `TARGET H ~= H1`
- Fail-Kriterium fuer AP-07:
  - Bei Rueckkehr auf `G0` fallen die Werte wieder auf rohe Profildefaults
    oder den urspruenglichen Zustand `S0/H0` zurueck.

## Ergebnisprotokoll (Kurzschema)

Pro Testfall dokumentieren:

- Datum/Uhrzeit
- Testfall-ID
- aktiver Profilname / Slot
- Startwerte `S0`, `H0`, `B`, `G0`
- beobachtete SG-Zustandsfolge
- beobachtete Overlaytexte
- beobachtete `M`-/`G`-Diagnosewerte
- Ergebnis `PASS` oder `FAIL`
- Freitext zu Auffaelligkeiten

## Kurzbewertung fuer AP-07

AP-07 ist auf echter Hardware nur dann bestanden, wenn alle vier Faelle gelten:

- Grade `0.0` erzeugt einen reinen Soft-Pfad ohne `WaitForFilter`.
- Grade `5.0` erzeugt einen reinen Hard-Pfad ohne vorgeschalteten Soft-Teil.
- `fixedGrade` sperrt Gradationsbearbeitung technisch und sichtbar.
- manueller Soft-/Hard-Edit bleibt ueber den anschliessenden Gradationswechsel
  fachlich erhalten und springt nicht auf rohe Paper-Defaults zurueck.
