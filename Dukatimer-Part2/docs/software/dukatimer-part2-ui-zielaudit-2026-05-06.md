# Dukatimer-Part2 UI-Zielaudit 2026-05-06

## Zweck

Dieser Audit prueft den aktuellen `Dukatimer-Part2`-UI-Stand gegen die
dokumentierten Vorgaben, Ziele und historischen Lehren. Der Fokus liegt auf:

- Closed-Loop-Ownership und Safety
- EV-/Dosis-/Measurement-Mathematik
- UI-/EEZ-/LVGL-Architektur
- Setup-, Paper-, Measurement- und WirelessRemote-Seiten
- Drift gegen Pflichtenheft, offene Ziele, offene Entscheidungen und Historie

Der Audit ist read-only fuer Firmware-Logik. Geaendert wurden nur
Dokumentation und Statuslisten.

## Gepruefter Stand

Code-/Buildstand:

- Teensy-Firmware: `0.2.44-dev`, Stage `visual threshold calibration method 1`
- UI-Runtime: LVGL 8.x, ILI9488_t3_mm shim, code-first handgeschriebene
  EEZ/LVGL-C-Screens
- Letzter UI-Build vor diesem Doku-Audit: `pio run -e teensy41` SUCCESS,
  RAM1 frei `21120` Bytes

Wesentliche Codepfade:

- `src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.c`
- `src/teensy/ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h`
- `src/teensy/LvglUi.cpp`
- `src/teensy/UiPresenter.cpp`
- `src/teensy/MeasurementValueFormatter.cpp`
- `src/teensy/MeasurementDomainService.cpp`
- `src/teensy/ExposureValueMath.cpp`
- `src/teensy/ExposureEngine.cpp`
- `src/teensy/SetupWorkflow.cpp`
- `src/teensy/PaperWorkflow.cpp`
- `src/teensy/InputRouterPolicy.cpp`
- `src/teensy/ModeCoordinator.cpp`

Dokumentbasis:

- `docs/software/offene_punkte.md`
- `docs/software/offene_ziele.md`
- `docs/software/offene_entscheidungen.md`
- `docs/software/eez-studio-step-by-step-und-seitenuebersicht.md`
- `docs/software/explizites-setup-menue-zielvorgabe.md`
- `docs/software/dukatimer-part2-ap11g-measurement-logic-math-audit.md`
- `docs/software/dukatimer-part2-ap11f-measurement-proposal-math-todo.md`
- `docs/software/erledigt/dukatimer-part2-architekturzugriffspunkte-math-sensorik-und-workflows.md`
- `docs/software/erledigt/dukatimer-historische-closed-loop-regelung.md`
- `docs/software/history/dukatimer-part2-pflichtenheft.md`
- `docs/software/history/dukatimer-part2-grundarchitektur-input-ui-wireless.md`
- `docs/hardware/end-to-end-papierkalibrierprotokoll.md`
- `docs/head_timing_and_i2c_timeout.md`
- `docs/neo_exposure_integration.md`
- `docs/hardware/ram-und-psram-strategie.md`

Historische Linie:

- `Dukatimer v0.3` als funktional breite, aber monolithische Referenz
- `Dukatimer v0.9` als modularere, aber noch mathematik- und
  formatter-zerklueftete App-Referenz
- `Dukatimer-Part2/historischer Ursprung` als fachliche Quelle fuer
  K-Faktor-, Probestreifen- und Kalibrierlogik, nicht als aktiver
  Implementierungspfad

## Kurzfazit

Der aktuelle UI-Stand ist die konsequente code-first Umsetzung der definierten
Seitenstruktur, aber noch nicht der finale Produktabschluss.

Positiv:

- Alle sieben lokalen LVGL-Seiten sind jetzt physisch vorhanden und aus dem
  Snapshot aktualisiert: Boot, Busy, Splitgrade, PaperWorkspace, Setup,
  Measurement, WirelessRemote.
- Closed Loop ist nicht ins UI oder in den ESP/C6-Pfad gewandert. Die
  Autoritaet bleibt bei `ExposureEngine` auf dem Teensy.
- EV-/F-Stop-Grundformeln sind zentral in `ExposureValueMath`.
- Measurement-Sessions, Referenzen, Undo und Histogramm gehoeren weiterhin
  `MeasurementDomainService`.
- Setup ist jetzt ein echter globaler Workflow mit persistentem
  `SystemSettings`-Pfad und Runtime-Anwendung auf Sensorik und Engine.

Negativ:

- PageMeasurement verletzt die eigene Darstellungsvorgabe teilweise, weil sie
  Messwerte direkt in `LvglUi::pushWidgetsFromSnapshot()` formatiert, statt die
  vorhandenen `UiPresenter`-/`MeasurementValueFormatter`-Pfade zu nutzen.
- Der UI-Stand ist noch handgeschriebenes LVGL-C, kein finaler EEZ-Designer-
  Source-of-Truth.
- Einzelne Farben und Tab-Handle-Semantiken sind noch Bring-up-/Fallback-Logik
  und koennen bei weiterer UI-Arbeit driften.
- Die mathematische Measurement-Basis ist nur session- und quellenrelativ; sie
  darf noch keine absolute Zone, Dichte, Papierwahrheit oder automatische
  Belichtungsvorschlaege behaupten.
- Reale Hardware- und Papierkalibrierabnahme fehlen weiterhin.

## Priorisierte positive Befunde

| Prioritaet | Befund | Nachweis | Bewertung |
| --- | --- | --- | --- |
| P0 erledigt | Closed-Loop-Autoritaet bleibt auf Teensy | `ExposureEngine` integriert Luxsekunden, entscheidet Fallback, Fault, Thermal Hard-Stop und praediktiven Shutoff; ESP/C6 transportieren nur Ereignisse und Messwerte | Vorgabe eingehalten. Keine zweite Belichtungsautoritaet sichtbar. |
| P0 erledigt | Busy-Screen bleibt Safety-Sperre | `LvglUi::computeTargetScreen()` erzwingt `SCREEN_ID_BUSY` in aktiven Exposure-Phasen; Busy nutzt eigene `busy_hdr_*`-Handles | Kritische Exposure-Zustaende koennen nicht von normalen Seiten ueberdeckt werden. |
| P0 erledigt | EV-/F-Stop-Grundlogik ist zentral | `ExposureValueMath::relativeEvFromLux()`, `evDeltaToMultiplier()`, `applyEvDeltaStops()`; keine UI-eigenen `log2`-/`powf(2, EV)`-Inseln gefunden | Historischer v0.9-Drift mit App-eigenen EV-Formeln wird vermieden. |
| P1 erledigt | Alle lokalen Screen-Widgetbaeume existieren | `create_screen_page_setup()`, `create_screen_page_measurement()`, `create_screen_page_wireless_remote()` und bestehende Boot/SG/Paper/Busy-Funktionen in `screens.c` | Alter Stub-Status ist erledigt; UI ist sichtbar navigierbar. |
| P1 erledigt | `LvglUi::pushWidgetsFromSnapshot()` deckt alle sieben Seiten ab | Blocks fuer Busy, Header/Tabs, Boot, Splitgrade, PaperWorkspace, Setup, Measurement, WirelessRemote | Daten kommen weiterhin aus `SystemSnapshot`, nicht aus Hardwarezugriffen im UI. |
| P1 erledigt | Setup-Scope folgt Zielvorgabe | `SetupWorkflow` enthaelt Sound, Lautstaerke, Vibration, Head-Cap, Head-Diag, Thermikwerte, Apply/Discard/SafetyDefaults; Papier/Service bleiben getrennt | Historisches Sammelmenue wird nicht wiederholt. |
| P1 erledigt | Thermikwerte sind runtime-verdrahtet | `main.cpp::applySystemSettingsToRuntimeIfNeeded()` setzt `sensorManager.setThermalThresholds()` und `exposureEngine.setThermalProtectionConfig()` | Fruehere Vermutung "nur kosmetische Setup-Werte" ist falsch; offen bleibt Abnahme, nicht Verdrahtung. |
| P2 erledigt | Measurement-Domain hat zentrale relative EV-Basis | `MeasurementDomainService` nutzt `ExposureValueMath::relativeEvFromLux()` mit quellengebundener Referenz | Der alte 1-Lux-Anker ist nicht wieder eingefuehrt. |
| P2 erledigt | Measurement-Formatter existieren | `MeasurementValueFormatter` formatiert Source, Main, Meta, Reference, Range und Controls; `UiPresenter` nutzt sie | Gute Schicht vorhanden, aber PageMeasurement nutzt sie lokal noch nicht konsequent. |
| P2 erledigt | Remote-Diagnose ist lokal sichtbar | PageWirelessRemote zeigt LinkHealth, PeerState, Akku, LastLux, Diagnose, TxQueue und RMT | Bring-up/Service-Sicht verbessert; C6-Produktabschluss bleibt offen. |

## Priorisierte negative Befunde und Drift

### P1: PageMeasurement umgeht Presenter/Formatter

Gepruefter Pfad:

- `LvglUi::pushWidgetsFromSnapshot()` fuer `SCREEN_ID_PAGE_MEASUREMENT`
- `UiPresenter::getMeasurementSources/Main/Meta/Reference/Range/...`
- `MeasurementValueFormatter::*`

Soll:

- `LvglUi` konsumiert Presenter-/Formatter-Ergebnisse.
- `LvglUi` rechnet und formatiert keine Messwerttexte mit eigener
  Referenzpolitik.
- Quellenpraezision und Invalid-Texte bleiben zentral.

Ist:

- Local/Wireless/Main/Reference/EV werden in `LvglUi.cpp` direkt mit
  `std::snprintf` formatiert.
- Dadurch wird lokale TSL2561-Praezision nicht als Integer-Lux dargestellt,
  Wireless-TSL2591 verliert die milli-lux-taugliche Praezision der zentralen
  Formatter, und Invalid-Texte unterscheiden sich von `NO SAMPLE`/`NO REF`.

Bewertung:

- Keine fotografische EV-Formel wurde dupliziert, weil `activeRelativeEvStops`
  bereits aus der Domain kommt.
- Trotzdem ist es ein UI-Ownership-Drift und muss vor produktiver Measurement-
  Nutzung korrigiert werden.

Mindestkorrektur:

- PageMeasurement auf `UiPresenter::getMeasurement...()` oder direkt auf
  `MeasurementValueFormatter` umstellen.
- Session-Range, Source, Correction und Proposal-Status sichtbar machen, ohne
  neue lokale Formatter zu erfinden.

### P1: End-to-End-Hardware-/Papierabnahme fehlt

Soll:

- Closed-Loop-Claims, Head-Latenz, Sensorpfade, Thermik und Papierprofilwerte
  werden mit realen Rohdaten und 21-Stufen-Graukeil belegt.

Ist:

- Das Protokoll existiert und ist gut geschnitten.
- Noch keine reale Session mit Firmwareversion, Rohdaten, Soll/Ist, Toleranz
  und Ableitung nach `ISO-P`, `ISO-R`, `kBw`, `kSoft`, `kHard` und SG-LUT ist
  im Repository sichtbar.

Bewertung:

- Architektur stimmt, aber fotografische Wahrheit ist noch nicht abgenommen.
- UI darf Paperprofile und Kalibrierwerte deshalb nicht als nass validierte
  Produktwahrheit ausgeben, solange der Profilstatus nicht belegt ist.

### P2: Measurement ist noch kein fotografisch vollstaendiger Workflow

Soll:

- Spot/Multi-Spot, Histogramm, Undo, Quellenalter, Messrollen,
  papierbezogene Proposal-Preview und Accept-Schritt.

Ist:

- Session-Historie, Undo und Histogramm existieren.
- Die Range ist relativ zur Quellenreferenz.
- Proposal-Eligibility und Preview/Accept sind nicht implementiert.
- Absolute Zone, Dichte/LogD und papierbezogene Vorschlaege sind nicht
  belastbar.

Bewertung:

- Der Weg ist korrekt, aber nicht vollstaendig.
- Besonders wichtig: `zoneHistogram` ist derzeit ein relativer EV-Bucket-Raum,
  keine absolute Adams-Zone oder Papierdichte.

### P2: UI ist code-first, nicht finaler EEZ-Designer-Stand

Soll:

- EEZ/LVGL-Produktlayout mit normativer Zone, Palette, Flow-/Style-Quelle und
  kleinem Runtime-Glue.

Ist:

- Kein `.eez-project` vorhanden.
- Screenbaeume sind handgeschriebenes LVGL-C.
- `build_modetabs()` speichert Tab-Handles als Labels; `LvglUi` braucht
  `lv_obj_get_parent()` fuer Container-Hintergruende.
- Einzelne Farbwerte stehen noch als Magic-Hexwerte in `screens.c`/`LvglUi.cpp`.

Bewertung:

- Als funktionaler Zwischenstand konsequent und nuetzlich.
- Als finaler Produkt-UI-Contract noch nicht abgeschlossen.

### P2: Remote-Produktpfad bleibt offen

Soll:

- C6 bleibt Terminal/Messgeraet ohne Exposure-/Papierautoritaet, zeigt aber
  frische Renderzustaende, hat Version, Pairing und Rueckmeldungen.

Ist:

- Lokale Teensy-Diagnoseseite existiert.
- RMT-Zaehler, TxQueue, LinkHealth und PeerState sind sichtbar.
- C6-Firmwareversion, Pairing-Resetpfad und finale Render-Freshness-Policy sind
  offen.

Bewertung:

- Kein Autoritaetsdrift, aber Produktvertrauen ist noch nicht voll geschlossen.

### P2: Setup ist umgesetzt, aber Safety-Abnahme fehlt

Soll:

- Systemsettings sind persistent, plausibilitaetsgeprueft, mit Encodern
  bedienbar und safety-relevante Aenderungen sind bestaetigt und getestet.

Ist:

- Pflichtwerte sind sichtbar und persistent angebunden.
- Thermikwerte haben Plausibilitaetsgrenzen und Mindestabstand.
- `Apply` ist der Commit-Schritt.
- Ein dedizierter Safety-Dialog fuer Thermikwerte und echte Reboot-/SD-/Thermik-
  Hardwareabnahme fehlen.

Bewertung:

- Der alte Setup-Architekturpunkt ist erledigt.
- Produktabnahme bleibt offen.

### P2: Paperprofilstatus und reale Defaults sind unzureichend sichtbar

Soll:

- Demo, gemessen, importiert und gesperrt sind unterscheidbar.
- Echte Defaults werden nur nach belegten Messdaten als echt markiert.

Ist:

- PaperWorkspace zeigt SELECT/CAL, CAL/RAW und FG/MG.
- Ein vollstaendiger Quellen-/Kalibrierstatus ist nicht sichtbar.
- Reale Protokolldaten fehlen.

Bewertung:

- UI-Grundgeruest ist gut, aber die fotografische Wahrheitsmarkierung fehlt.

### P2: Weitere historische Modi sind noch Shells oder Datenmodelle

Soll:

- BW, Burn/Dodge, Teststrip, Preflash, Densitometer/Filmtest werden jeweils
  echte Workflows mit UI, Input-Semantik, Exposure-Kommandos, Tests und
  Hardwareabnahme.

Ist:

- BW ist Shell.
- Preflash liegt im Profilmodell.
- Semantiken existieren in Kopf-/Command-Daten, aber nicht als produktive Modi.

Bewertung:

- Kein Drift, solange diese Punkte klar offen bleiben.
- Gefahr entsteht erst, wenn Enum-/Headerwerte als Featureabschluss gelesen
  werden.

### P3: Dokumentationsdrift war vorhanden und wurde in diesem Slice reduziert

Vor diesem Audit enthielten Einstiegspapiere noch Aussagen wie:

- Paper/Setup/Measurement/WirelessRemote seien Stub-Screens.
- Setup besitze keinen produktiven Widgetbaum.
- Thermikwerte seien noch nur Konstanten.

Diese Aussagen sind fuer den aktuellen Code nicht mehr richtig. Sie wurden in
diesem Slice in `offene_punkte.md`, `offene_ziele.md`,
`offene_entscheidungen.md` und der EEZ-Anweisung nachgezogen.

## Math- und Closed-Loop-Audit

### Gepruefter Pfad

EV/F-Stop:

- `ExposureValueMath::relativeEvFromLux(float lux, float referenceLux)`
- `ExposureValueMath::evDeltaToMultiplier(float evDeltaStops)`
- `ExposureValueMath::applyEvDeltaStops(...)`

Dosis:

- `ExposureEngine::startDoseExposure(float targetDoseLuxSeconds)`
- `ExposureEngine::updateDoseClosedLoop(...)`
- `ExposureEngine::observeSensorStatus(...)`

Measurement:

- `MeasurementDomainService::buildSessionSample(...)`
- `MeasurementDomainService::updateActiveRelativeEv(...)`
- `MeasurementValueFormatter::*`

Paper-Kalibrierung:

- `PaperWorkflow::recalculateFromSteps()`

### Groessen und Einheiten

| Groesse | Besitzer | Einheit/Semantik | Bewertung |
| --- | --- | --- | --- |
| `currentDose` / `targetDose` | `ExposureEngine` | Luxsekunden | Physische Runtime-Dosis, korrekt nicht im UI berechnet. |
| `measuredLux` | `ExposureEngine` / Sensorstatus | Lux vom lokalen TSL2561 | Closed-Loop-nahe Messgroesse, nicht Papier-Spotwahrheit. |
| `relativeEvStops` | `MeasurementDomainService` | Stops relativ zu Quellenreferenz | Mathematisch korrekt fuer relative Session; nicht absolute Zone. |
| `zoneHistogram[]` | `MeasurementDomainService` | 11 relative EV-Buckets | Anzeige-/Session-Raum, noch kein fotografisch absoluter Zonenraum. |
| `runtimeHeadBusLatencyMs` | `ExposureEngine` | Millisekunden | Technische Latenz, sauber von EV getrennt. |
| `isoP`, `isoR`, `kBw` aus Methode 1 | `PaperWorkflow` | papierbezogene Profilableitung | Nicht EV/F-Stop; braucht reale Kalibrierabnahme, aber keine UI-Math-Duplikation. |

### Invarianten

- EV-Umrechnung arbeitet nur auf positiven, endlichen Luxwerten und
  Referenzluxwerten.
- Dosisintegration bleibt in Luxsekunden und wird nicht in EV umgedeutet.
- Head-/Bus-Latenz bleibt in ms/us und sickert nicht in fotografische EV-Werte.
- Remote und ESP transportieren Messwerte/Eingaben, besitzen aber keine
  Belichtungsentscheidung.
- UI zeigt Werte, mutiert aber keine Exposure-/Measurement-/Paper-Wahrheit.

### Befunde

- Keine eigene EV- oder Dosisformel in `LvglUi` gefunden.
- `powf(2.0f, EV)` und `log2(lux/referenceLux)` liegen in
  `ExposureValueMath`; das ist korrekt.
- `PaperWorkflow::recalculateFromSteps()` nutzt `powf(10.0f, -D)` fuer eine
  visuelle Schwellenwert-/Dichteableitung. Das ist keine F-Stop-Formel und
  gehoert fachlich in den Paper-Kalibrierkontext, muss aber hardware- und
  protokollseitig validiert werden, bevor daraus echte Profilwahrheit wird.
- PageMeasurement ist die einzige relevante UI-Driftstelle: nicht Mathematik,
  aber Format-/Semantikownership.

## Zielabgleich

| Ziel | Status 2026-05-06 | Bewertung |
| --- | --- | --- |
| Z0 Dokumentationshygiene | verbessert | Einstiegspapiere wurden aktualisiert; alte Auditdokumente bleiben Historie. |
| Z1 Remote-/Servicepfad | teilweise | Transport, Diagnose, RMT sichtbar; C6-Version, Pairing, Freshness und Abnahme offen. |
| Z2 Produktive LVGL/EEZ-UI | teilweise/fortgeschritten | Alle lokalen Screenbaeume existieren; finaler EEZ-/Formatter-/Hardwarestand offen. |
| Z3 Messdomain | teilweise | Domain und UI sichtbar; Proposal, absolute Zone, Dichte/LogD, Quellenkalibrierung offen. |
| Z4 Papier/Kalibrierung | teilweise | Persistenz, CAL-Editor und Methode-1-Ableitung vorhanden; echte Protokollmessung und Statusmarkierung offen. |
| Z5 Belichtungsworkflows | Splitgrade aktiv, Rest offen | SG bleibt Hauptpfad; BW/Burn/Teststrip/Preflash/Densitometer nicht produktiv. |
| Z6 Exposure/Kopf/Sensorik | architektonisch gut, Abnahme offen | Engine, Head, Sensorpfad vorhanden; I2C-Recovery und reale Messreihen fehlen. |
| Z7 Storage/VFS | teilweise | Safety-Interlock fuer Transfers vorhanden; Produkt-Import/Export/Access offen. |
| Z8 Test/Abnahme | lueckenhaft | AP-07-Harness und Builds vorhanden; breite Host-/Hardwaretests fehlen. |

## Ist der UI-Stand die konsequente Umsetzung der Vorgaben?

Ja, als code-first Zwischenstand.

Die aktuelle UI setzt die definierte Seitenstruktur, Zonenlogik und
Teensy-Autoritaet konsequent um. Sie portiert nicht die historischen
Fehlmuster zurueck: keine direkte Hardware im UI, keine zweite
Exposure-Autoritaet, keine per-Screen EV-Formeln, kein historisches
Setup-Sammelmenue.

Nein, als finaler Produktabschluss.

Der Stand ist noch nicht konsequent genug an der eigenen Presenter-/Formatter-
Regel, nicht final EEZ-style-gebunden, nicht hardware-abgenommen und nicht
fotografisch validiert. Besonders Measurement muss jetzt hygienisiert werden,
sonst entsteht genau der Darstellungsdrift, den die Architektur vermeiden soll.

## Aktualisierte Dokumente in diesem Audit-Slice

- `docs/software/offene_punkte.md`
- `docs/software/offene_ziele.md`
- `docs/software/offene_entscheidungen.md`
- `docs/software/eez-studio-step-by-step-und-seitenuebersicht.md`

## Naechste priorisierte Schritte

1. PageMeasurement auf `UiPresenter`/`MeasurementValueFormatter` umstellen und
   Source/Invalid/Range/Proposal-Texte zentral halten.
2. Magic-Farbwerte und Tab-Handle-Semantik bereinigen oder als code-first
   Vertrag dokumentieren.
3. UI-Hardwareabnahme: Encoder-only, Touch-Komfort, Busy-Modal, Setup Apply /
   Discard / SafetyDefaults, Measurement-Sourcewechsel.
4. Erste echte End-to-End-Papierkalibriersession mit Rohdaten und
   Firmwareversion dokumentieren.
5. TSL2561-I2C-Recovery ohne Neustart hardware-/softwareseitig schliessen.
6. C6-Firmwareidentitaet, Pairing/Freshness und Remote-Messstart-Ack validieren.
