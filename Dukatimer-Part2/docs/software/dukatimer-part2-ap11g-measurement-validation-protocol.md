# AP-11g P3 Measurement-Validierung und Abnahmebasis

Stand: 2026-05-02

## Zweck

Dieses Dokument setzt P3 aus dem AP-11g-Audit in konkrete, wiederholbare
Validierung um. Es trennt bewusst zwischen drei Ebenen:

- automatisierte Status- und Vertragspruefung im Code
- hardware-nahem Messprotokoll fuer Cross-Source-Beobachtung
- klarer Nicht-Freigabe fuer alles, was heute noch keine belastbare
  Densitometer- oder LogD-Aussage tragen kann

Die P3-Artefakte dazu sind:

- `test/ap11_measurement_domain/test_main.cpp`
- `tools/run_ap11_measurement_harness.py`
- `tools/check_dukatimer_protocol_sync.py`

## P3.1 MeasurementDomain-Harness

Der Harness deckt die bereits belegten Invarianten des aktuellen
Measurement-Vertrags ab:

- lokaler Single-Source-Range-Aufbau und Undo
- Mixed-Source-Sperre fuer `rangeUsableForProposal`
- Wireless-Reference-Rebuild nach Undo der letzten Wireless-Teilserie
- Overflow mit sauberem Histogramm-Abzug und Referenz-Rebuild
- Pending-Capture-Role aus P2.4 als Teil des sichtbaren Session-Vertrags

Ausfuehrung:

- Bevorzugt per Host-Compiler ueber `tools/run_ap11_measurement_harness.py`
- Falls auf der Arbeitsstation kein `g++`, `clang++` oder `cl.exe` verfuegbar
  ist, faellt das Script bewusst auf eine Syntax-Pruefung mit der installierten
  Teensy-ARM-Toolchain zurueck, statt den Harness still auszulassen.

Wichtig:

- Der Harness ist absichtlich kein HAL-Test.
- `test/support/Arduino.h` stellt nur `millis()` bereit.
- Jede weitere Arduino-Abhaengigkeit waere ein Warnsignal, dass die
  Measurement-Domain wieder zu tief in Laufzeit-/Boardlogik hineinrutscht.

## P3.2 Cross-Source-Hardwareprotokoll

Dieses Protokoll soll Sichtbarkeit ueber Local-TSL2561 gegen Wireless-TSL2591
schaffen, ohne schon eine unbewiesene Cross-Calibration zu behaupten.

Vorbedingungen:

- Kein Negativ im Strahlengang
- Konstante Kopfleistung und stabile Aufwaermphase
- Lokaler TSL2561 bleibt im Kopfpfad unter den NeoPixeln
- Wireless-TSL2591 misst an der Papier-/Spot-Geometrie
- Dark-/Offset-Workflow ist noch nicht aktiv; Rohwerte bleiben als Rohwerte
  dokumentiert

Messablauf je Lauf:

1. Kopf auf definierte Intensitaet bringen und 10 s stabilisieren.
2. Lokalen TSL2561-Wert im Dukatimer-Part2-Messscreen ablesen.
3. Wireless-TSL2591-Wert am C6 ablesen und per Measure in die Session bringen.
4. Beide Quellen inklusive Sample-Alter, Session-Quelle und Korrekturstatus
   fotografieren oder manuell protokollieren.
5. Drei Wiederholungen pro Intensitaetsstufe fahren.

Pflichtfelder pro Lauf:

| Feld | Bedeutung |
| --- | --- |
| Lauf-ID | fortlaufende Nummer |
| Intensitaetsstufe | Kopfleistung / Szenenbeschreibung |
| `localLux` | lokaler TSL2561-Rohwert |
| `wirelessLux` | Wireless-TSL2591-Rohwert |
| `wirelessSampleAgeMs` | echtes C6-Sensoralter am Capture-Zeitpunkt |
| `peerLastSeenAgeMs` | Funk-/Peer-Frische getrennt vom Sensoralter |
| `crossSourceDeltaStops` | rein beobachtend: `log2(wirelessLux / localLux)` |
| `correction` | sichtbarer Korrekturstatus pro Quelle |
| Bemerkung | Drift, Flackern, auffaellige Geometrie |

Interpretationsregel:

- `crossSourceDeltaStops` ist hier nur Diagnose, keine Kalibration.
- Ein stabiler, sichtbarer Delta-Verlauf ist nuetzlich.
- Ein numerischer Delta-Wert darf noch nicht in Proposal, Papierprofil oder
  Densitometerlogik rueckgeschrieben werden.

## P3.3 LogD- und Densitometer-Sperre

Bis zur Entscheidung ueber Referenz, Dark-/Offset-Behandlung und Messgeometrie
gelten folgende harte Nicht-Freigaben:

- keine LogD-Anzeige aus aktuellem Measurement-Lux
- keine Behauptung, dass `relativeEvStops` schon Papierdichte sei
- keine Cross-Source-Normalisierung zwischen TSL2561 und TSL2591
- keine Densitometer-, Filmtest- oder Papierkalibrier-Claims auf Basis einer
  unmarkierten Mixed-Source-Session

Erst nach einem belegten Entscheid fuer diese drei Punkte darf die Domain ueber
eine reine Relative-Spot-Session hinausgehen:

- Referenzanker
- Dark-/Offset-Modell pro Quelle
- Messgeometrie und optischer Aufbau

## P3.4 SharedProtocol-Sync-Guard

`tools/check_dukatimer_protocol_sync.py` vergleicht die SharedProtocol-Quelle in
`Dukatimer-Part2/lib/SharedProtocol/DukatimerProtocol.h` gegen die historische
C6-Kopie in `../Wireless TSL2591/include/DukatimerProtocol.h`.

Regel:

- Die Dateien muessen bytegleich bis auf Zeilenenden sein.
- Jeder Dukatimer-Part2-Build und jeder Wireless-TSL2591-Build laeuft vor dem
  eigentlichen Compile durch diesen Guard.
- Bei Abweichung ist der Build absichtlich rot, bevor unterschiedliche
  ESP-NOW-ABIs oder Snapshot-Vertraege entstehen koennen.

## P3.5 Hardware-nahe UI-Checkliste

Diese Checkliste ist fuer echte Geraetepruefung gedacht und bewusst von den
automatisierten Harness-Faellen getrennt.

Pflichtfaelle:

1. `NO SAMPLE`: lokale und Wireless-Quelle ungueltig, keine Fake-Nullwerte.
2. Stale Wireless: `sensorSampleAgeMs` ueber Frischeschwelle, Active-Lux darf
   nicht still als gueltiger Messwert weiterlaufen.
3. Mixed Source: lokale Probe plus Wireless-Probe in einer Session, `PROP`
   bleibt gesperrt.
4. Undo: letzte Wireless-Probe rueckgaengig, neue Wireless-Teilserie baut ihre
   Referenz neu auf.
5. Overflow: mehr als 128 Samples, Histogramm und sichtbare History bleiben
   konsistent.
6. C6 offline: Peer offline oder Link lost, Wireless darf nicht als aktiver
   Messwert gelten.
7. C6-Button ohne valid lux: kein stilles Session-Sample nur durch Taste ohne
   gueltigen Sensorwert.
8. Pending Role: `E2/C6 ROLE ...` ist sichtbar, Capture landet mit derselben
   expliziten Rolle in der Session.

Abnahmeregel:

- Ein UI-Fall ist erst bestanden, wenn Anzeige, Sessionstatus und reale
  Eingabefolge zusammenpassen.
- Ein einzelner korrekter Text im Presenter reicht nicht, wenn Session-History,
  Active-Lux oder Proposal-Freigabe dazu nicht konsistent sind.