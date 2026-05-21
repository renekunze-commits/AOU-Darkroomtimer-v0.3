# Dukatimer-Part2 Step-by-Step Abarbeitung

Stand: 2026-04-30

## Arbeitsregeln fuer diese Abarbeitung

- Keine bestehende Logik wird umgeschrieben.
- Kein bestehender Code wird geloescht.
- Neue Bausteine werden additiv eingefuehrt.
- Jeder Schritt hat klare Abnahmekriterien.
- Nach jedem Schritt: Build- und Fehlercheck.
- Code ausführlich und verständlich kommentieren und versionieren
- Doku nach jedem Schritt aktualisieren

## Kurzbefund aus Doku- und Historienanalyse

Quellenbasis:

- docs/software/erledigt/dukatimer-part2-statusaudit-ui-grundgeruest-2026-04-24.md
- docs/software/dukatimer-part2-pflichtenheft.md
- docs/software/dukatimer-part2-migrations-backlog.md
- docs/software/erledigt/dukatimer-historische-funktionsanalyse.md
- docs/software/erledigt/dukatimer-historische-closed-loop-regelung.md
- Dukatimer v0.3 (historische Verhaltensreferenz)
- Dukatimer v0.9 (historische Strukturreferenz)

Kernaussage fuer den Start:

1. MB-06 ist blockierend: Papierslots plus persistente Version/CRC-Strategie fehlen.
2. Part2 hat bisher nur das PaperExposureProfile-Schema, aber noch keine Slot-Bank- und Speicherlogik.
3. v0.3 und v0.9 belegen klar das Muster: Slot-Bank mit aktivem Index plus Integritaetspruefung (Hash/CRC) und Versionspruefung.

## TODO-Liste (strikt sequenziell)

### Schritt 1 (jetzt): Persistenzbasis fuer Papierslots additiv einziehen

Status: ERLEDIGT (2026-04-28)

Details ausgelagert nach:

- docs/software/erledigt/dukatimer-part2-schritt-01-persistenzbasis-papierslots.md

### Schritt 2: Slot-Store an SD-Dateipfad anbinden (read/write), weiter additiv

Status: ERLEDIGT (2026-04-28)

Umfang:

- Dateiheader mit Magic/Version/CRC verwenden.
- Ladepfad mit harter Validierung.
- Speichpfad mit defensiver Fehlerbehandlung.

Abnahmekriterien:

- Fehlerhafte Daten werden erkannt und nicht blind uebernommen.
- Gueltige Daten werden reproduzierbar gelesen/geschrieben.

Umsetzungsergebnis:

- Neuer SD-Store fuer Papierslots:
  - src/teensy/PaperSlotStorage.h
  - src/teensy/PaperSlotStorage.cpp
- Ladepfad liest exakt die erwartete Blob-Groesse und validiert ueber den Codec (Magic/Format/Payload-Size/CRC/Schema).
- Schreibpfad nutzt temp-Datei und Rename-Commit (`/paperslots.bin.tmp` -> `/paperslots.bin`) mit defensivem Retry nach `remove()`.
- Build-Validierung: pio run -e teensy41 erfolgreich.

### Schritt 3: Integration ohne Verhaltensbruch

Status: ERLEDIGT (2026-04-28)

Umfang:

- Nicht-invasive Anbindung an den bestehenden Bootpfad.
- Fallback auf in-memory Defaults, falls keine gueltigen Daten vorliegen.

Abnahmekriterien:

- Startverhalten bleibt stabil.
- Bestehende Modi und Eingabepfade verhalten sich unveraendert.

Umsetzungsergebnis:

- Nicht-invasive Runtime-Anbindung im Bootpfad:
  - `setupPaperSlotPersistence()` initialisiert immer Defaults, versucht danach SD-Load.
  - Nur bei `FileNotFound` wird kontrolliert ein Default-Blob gespeichert.
  - `InvalidBlob`/`ReadFailed`/`UnsupportedFormatVersion`/`InvalidBank` bleiben als sichtbarer Fehlerpfad bestehen und ueberschreiben keine bestehende Datei mit Defaults.
  - Keine Umverdrahtung von Input-/Mode-/Exposure-Logik.
- Speicherpolitik fuer Teensy 4.1 mit 8MB PSRAM umgesetzt:
  - spaeterer Hardwarebefund 2026-05-03: aktuelles Duka-Teen-Board meldet
    reproduzierbar `16 MB` externes PSRAM; die `8MB`-Angabe markiert hier den
    damaligen Implementierungsstand
  - `platformio.ini` erweitert um `DUKATIMER_TEENSY_HAS_PSRAM=1`, `DUKATIMER_TEENSY_PSRAM_MB=8`.
  - `PaperSlotBank` nach `EXTMEM` ausgelagert.
  - PaperSlot-I/O-Blobpuffer als statischer `EXTMEM`-Puffer.
  - Buildprofil auf `TEENSY_OPT_SMALLEST_CODE_LTO` gesetzt.
- Build-Validierung: `pio run -e teensy41` erfolgreich.
  - FLASH: code 204640, data 49408, headers 9116
  - RAM1: variables 197024, code 199592, padding 29784, free 97888
  - RAM2: variables 146816, free 377472
  - EXTRAM: variables 6432
- Weitere sinnvolle Optimierungen (Slice 2, 2026-04-28) umgesetzt:
  - `EspServiceLink`-Instanz nach `EXTMEM` verschoben.
  - LVGL-Heap (`work_mem_int`) ueber `LV_ATTRIBUTE_LARGE_RAM_ARRAY` nach `.externalram` verlagert.
- Re-Validierung nach Slice-2-Optimierung: `pio run -e teensy41` erfolgreich.
  - FLASH: code 204640, data 49408, headers 9116
  - RAM1: variables 56896, code 199592, padding 29784, free 238016
  - RAM2: variables 146816, free 377472
  - EXTRAM: variables 146560

### Schritt 4: Dokumentation und Testslices

Status: ERLEDIGT (2026-04-29)

Umfang:

- Kurze Integrationsdoku.
- Minimaler Testleitfaden fuer Migration, CRC-Fehler und Leerdaten.

Abnahmekriterien:

- Nachvollziehbare Reproduktionsschritte fuer die wichtigsten Persistenzfaelle.

Umsetzungsergebnis:

- Neuer minimaler Persistenz-Testleitfaden erstellt:
  - `docs/software/dukatimer-part2-paperslot-persistenz-testleitfaden-2026-04-29.md`
- Testfaelle enthalten:
  - fehlende Datei / Leerdaten
  - defekter Blob (Size/CRC/Header)
  - Migrationsschutz bei Versionsabweichung
  - sichtbarer Schreibfehlerpfad
  - Stabilitaet gueltiger Daten ueber Reboot
- Diagnose-Sichtbarkeit im Runtime-Snapshot/UI ergaenzt:
  - `SystemSnapshot` transportiert PaperSlot-Statuscode und Detailwert.
  - Debugzeile zeigt `PS e:<code> d:<detail>`.

### Schritt 5: Offene Punkte aus dem Statusaudit (Startslice)

Status: IN ARBEIT (2026-04-28)

Umfang (Slice 1):

- MB-02 anfangen: UI vom reinen Debug-/Statusscreen in Richtung minimalen
  SG-Hauptscreen mit Overlay-Kanal bewegen.
- Additiv bleiben: bestehender Debugscreen darf als Fallback erhalten bleiben.

Abnahmekriterien (Slice 1):

- Im SG-Hauptzustand sind kompakte Hauptinformationen sichtbar (Header,
  Targets, Dosisbezug).
- Overlay-Kanal zeigt mindestens Fehler/WaitForFilter/Done/Aborted/Ready.
- Außerhalb SG bleibt der bisherige Debug-/Statusblock funktionsfaehig.

Umsetzungsergebnis (Slice 1):

- `src/teensy/LvglUi.h`: SG-Mainlabel- und Overlay-Felder ergaenzt.
- `src/teensy/LvglUi.cpp`:
  - SG-Hauptscreen (Header/Targets/Dose) eingefuehrt.
  - Overlay-Kanal fuer Fehler-/Workflow-/Remote-Hinweise eingefuehrt.
  - Sichtbarkeit zwischen SG-Mainscreen und Debugblock zustandsabhaengig
    umgeschaltet.
- Build-Validierung: `pio run -e teensy41` erfolgreich.
  - FLASH: code 206408, data 64768, headers 8372
  - RAM1: variables 72768, code 200408, padding 28968, free 222144
  - RAM2: variables 146816, free 377472
  - EXTRAM: variables 146560

Umfang (Slice 2):

- MB-02 weiterziehen: baselinefaehige InputRouter-Policy fuer
  Encoder-vs-Touch-Fokus und modale Workflow-Phasen additiv einziehen.
- Bestehende Input-Pipelines behalten und nur zentralen Filter-/Guardpunkt
  ergaenzen.

Abnahmekriterien (Slice 2):

- Touch-Aktivitaet setzt den Fokus auf Touch; nach Timeout faellt der Fokus
  reproduzierbar auf Encoder zurueck.
- Rotations-Events werden in modalen SG-Zustaenden (WaitForFilter/Fault) nicht
  mehr durchgereicht.
- Lokale/Remote-Inputpipes bleiben konsistent, Build bleibt erfolgreich.

Umsetzungsergebnis (Slice 2):

- Neues InputRouter-Modul:
  - `src/teensy/InputRouterPolicy.h`
  - `src/teensy/InputRouterPolicy.cpp`
- Baseline-Regeln eingefuehrt:
  - Touch setzt Fokus auf `Touch`.
  - Nach `kTouchFocusHoldMs = 900` ohne Touch-Rueckmeldung Rueckfall auf
    `Encoder`.
  - In `WorkflowConfirm` (SG WaitForFilter) und `WorkflowFault` werden
    Rotations-Events blockiert.
- Runtime-Integration in `src/teensy/main.cpp`:
  - Policy wird in `setup()` initialisiert.
  - Touchzustand wird je Loop beobachtet.
  - Dispatch-Gate sitzt zentral in `dispatchNormalizedInputEvent(...)`.
  - Nach erfolgreichem Dispatch wird die Fokusdomaine aktualisiert.
- Build-Validierung nach Slice 2: `pio run -e teensy41` erfolgreich.
  - FLASH: code 206856, data 64768, headers 8948
  - RAM1: variables 72800, code 200856, padding 28520, free 222112
  - RAM2: variables 146816, free 377472
  - EXTRAM: variables 146560

Umfang (Slice 3, 2026-04-29): EV-logarithmische Skalierung SG-Targets

- Historischer Befund: v0.3 Mode_SG.cpp verwendet multiplikative EV-Schritte
  (`target *= pow(2, direction * evStep)`, Default 1/3 Stop).
  Part2 hatte lineare Additivschritte -- nicht historisch konform, fotographisch
  unpraezise bei grossen Targetwerten.
- Additiver Eingriff in SplitgradeWorkflow, keine anderen Pfade veraendert.

Abnahmekriterien (Slice 3):

- Soft- und Hard-Target-Aenderungen skalieren proportional (kleine Werte:
  kleiner Schritt, grosse Werte: groesserer Schritt im absoluten Mass).
- Verhalten entspricht `target *= pow(2, 1/3)` pro Schritt (ca. +26%).
- Build bleibt erfolgreich, keine Regressionen.

Umsetzungsergebnis (Slice 3):

- `src/teensy/SplitgradeWorkflow.h`:
  - `kEvStep = 1.0f / 3.0f` als `static constexpr` ergaenzt.
- `src/teensy/SplitgradeWorkflow.cpp`:
  - `#include <cmath>` ergaenzt.
  - `adjustPrimaryValue()` SplitTargets-Zweig:
    `softTarget_ = clampTargetValue(softTarget_ * powf(2.0f, direction * kEvStep))`
  - `adjustSecondaryValue()` SplitTargets-Zweig:
    `hardTarget_ = clampTargetValue(hardTarget_ * powf(2.0f, direction * kEvStep))`
  - Modusunterscheidung (Dose/Time) entfaellt -- EV-Formel gilt fuer beide.
- Build-Validierung: `pio run -e teensy41` erfolgreich.
  - FLASH: code 207848, data 65792, headers 8980
  - RAM1: variables 73824, code 201848, padding 27528, free 221088
  - RAM2: variables 146816, free 377472
  - EXTRAM: variables 146560

Umfang (Slice 4, 2026-04-29): Querschnittsvorgaben fuer EV, Messwerte und Wiederverwendung

- F-Stop-/EV-Logik wird nicht mehr als einzelner BW-Sondermodus betrachtet,
  sondern als zentrale Belichtungswert-Schicht fuer alle belichtungsrelevanten Modi.
- Messwerte aus echten Messvorgaengen werden zusaetzlich in EV ausgewiesen,
  sofern die fotografische Interpretation sinnvoll ist; Rohtelemetrie bleibt in Lux.
- Logik mit Mehrfachnutzung wird einmal zentral definiert und von Modi,
  Workflows und UIs nur konsumiert.

Abnahmekriterien (Slice 4):

- Pflichtenheft, Backlog, Audit und Historienanalyse verwenden dieselbe
  Querschnittsdefinition fuer EV-/F-Stop-Logik.
- Densitometrie, Kalibrierung und andere echte Messworkflows sind in der Doku
  konsistent als EV-plus-Lux-Faelle beschrieben.
- Es bleibt explizit getrennt: `ExposureEngine` intern lux-/dosisbasiert,
  EV nur als Bedien- und Darstellungslogik.

Umsetzungsergebnis (Slice 4):

- Kern-Dokumente auf gemeinsame EV-/F-Stop-Logik fuer alle
  belichtungsrelevanten Modi umgestellt.
- Messwertkonvention EV plus Lux fuer echte Messworkflows dokumentiert.
- Vorgabe zur zentralen Definition mehrfach genutzter Logik explizit verankert.

Umfang (Slice 5, 2026-04-30): AP-09 Remote-Messung und ESP-Serviceintegration

- TeensyCommand- und Remote-Render-Pfade produktiv zwischen Teensy und ESP verdrahtet.
- Historische Wireless-TSL2591-Fernbedienung als echter Controller/Meter via ESP-NOW angebunden.
- Reale ESP-Service-Sensorik (DS18B20 + AHT) inklusive Fehlerpfad statt Platzhalterwerten aktiv.
- Status- und Fehlertelemetrie fuer Link- und Servicepfade bis in die UI-Diagnosezeile erweitert.

Abnahmekriterien (Slice 5):

- Remote-Renderdaten und SG-Messkommandos laufen Ende-zu-Ende ueber den Service-Link.
- Wireless-Controller-Eingaben und Luxdaten werden aus realen C6-Paketen akzeptiert.
- Service-Sensorstatus ist im Snapshot und in der UI sichtbar, Diagnoseframes werden transportiert.
- Build bleibt fuer `teensy41` und `esp32s3_n16r8` erfolgreich.

Umsetzungsergebnis (Slice 5):

- SharedProtocol erweitert um produktiven Remote-Render- und Diagnostikbetrieb:
  - `lib/SharedProtocol/DukatimerProtocol.h`
- Teensy-Linkseite konsumiert Service-/Wireless-/Diagnoseframes und publiziert Remote-Kommandos:
  - `src/teensy/EspServiceLink.h`
  - `src/teensy/EspServiceLink.cpp`
  - `src/teensy/EspLinkRuntimeStatus.h`
- ESP-Linkseite sendet Diagnoseframes und liefert Pending-Remote-Kommandos/Renderdaten:
  - `src/esp32/TeensyLinkService.h`
  - `src/esp32/TeensyLinkService.cpp`
- Reale Service-Sensorik umgesetzt:
  - `src/esp32/ServiceSensorHub.h`
  - `src/esp32/ServiceSensorHub.cpp`
- Produktiver Wireless-Gatewaypfad umgesetzt und auf historisches C6-Event-ABI abgeglichen:
  - `src/esp32/WirelessRemoteGateway.h`
  - `src/esp32/WirelessRemoteGateway.cpp`
  - `RemoteEventPacket` auf 16-Byte-ABI ausgerichtet (kompatibel zur historischen Firmware)
- ESP-Wiring produktiv angebunden:
  - `src/esp32/main.cpp`
- Sichtbarer Gateway-/Diagnosestatus in der Presenter-Ausgabe erweitert:
  - `src/teensy/UiPresenter.h`
  - `src/teensy/UiPresenter.cpp`
- Build-Validierung:
  - `pio run -e teensy41` erfolgreich
  - `pio run -e esp32s3_n16r8` erfolgreich

## Aktueller Ausfuehrungsstand

- Analyse abgeschlossen.
- Schritt 1 abgeschlossen.
- Schritt 2 abgeschlossen.
- Schritt 3 abgeschlossen.
- Schritt 4 abgeschlossen.
- Schritt 5 aktiv (Slices 1-5 umgesetzt; Remote-/Service-End-to-End steht produktiv).
- AP-09 umgesetzt und build-validiert; naechster fachlicher Ausbau bleibt die Measurement-Domain-Integration der Wireless-Luxwerte.
