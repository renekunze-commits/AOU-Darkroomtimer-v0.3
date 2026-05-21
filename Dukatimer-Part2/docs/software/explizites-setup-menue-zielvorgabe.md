# Explizites Setup-Menue fuer Part2

Stand: 2026-05-02

## Ziel

Dieses Dokument definiert die Zielvorgabe fuer ein explizites Setup-Menue in
`Dukatimer-Part2`.

Es beantwortet drei Fragen zugleich:

- was die Historie zu `MODE_SETUP` fachlich wirklich hergibt
- was davon fuer Part2 weiterhin sinnvoll ist
- wie ein neues Setup-Menue geschnitten werden muss, damit nicht erneut Papier-,
  Mess-, Service- und Safety-Logik in einem unklaren Sammelmodus vermischt werden

Die Leitidee fuer Part2 lautet:

- ein explizites Setup-Menue ist weiterhin sinnvoll und historisch belegt
- aber Part2 darf die alte Vermischung nicht wiederholen
- globale Systemkonfiguration, Papier-/Kalibrierwerte und Serviceaktionen werden
  sauber getrennt

## Historischer Befund

## v0.3

`Dukatimer v0.3/src/Mode_Setup.cpp` zeigt einen echten Setup-Modus mit eigener
State-Machine. Historisch vorhanden waren dort unter anderem:

- `SOUND MODE`
- `PWM LCD BRIGHT`
- `PWM FOCUS LAMP`
- `PWM MAX EXPOSURE`
- `BW ZIEL-ZONE`
- `WIRELESS PROBE`
- `FLASH GLOBAL`, `FLASH LEVEL`, `FLASH COLOR`
- Aktionspunkte wie `TEACH PAPER`, `TEST STRIP`, `SYSTEM ERRORS`,
  `C6 DARK CALIB`, `USB BRIDGE`

Fachlicher Wert der Historie:

- ein eigener Setup-Einstieg ist kein Fremdkoerper, sondern gehoert zur
  gewachsenen Produktidee
- globale Konfiguration wurde schon frueh als eigenes Bedienziel erkannt

Technische Schwaeche der Historie:

- globale Einstellungen, Papier-/Preflash-Werte, Kalibrier- und Serviceroutinen
  sowie Diagnoseaktionen lagen im selben Menue
- daraus entstand kein klarer Schnitt zwischen Produkt-Setup und
  Maintenance-/Wizard-Funktionen

## v0.9

`Dukatimer v0.9/src/SetupApp.cpp` fuehrt diese Linie fort, diesmal als eigene
App. Historisch konfigurierbar waren dort:

- `stdTime`
- `splitModeAuto`
- `soundMode`
- `stepMode`
- `bwTargetZone`
- `useWirelessProbe`
- `pwmLcd`, `pwmSafe`, `pwmFocus`, `pwmMax`
- `probeDarkLux` als Teil einer Dark-Calibration-Aktion
- `RESET ERRORS`, `SAVE & EXIT`

Lehre aus v0.9:

- die Richtung "Setup als eigener Modus" bleibt richtig
- aber auch hier sind globale Systemsettings, Mess-/Kalibrierpfade und
  Serviceaktionen noch nicht sauber getrennt

## Bereits angelegte Part2-Richtung

Die aktive Part2-Dokumentation hat den Platz fuer Setup bereits vorgesehen:

- `docs/software/history/dukatimer-part2-grundarchitektur-input-ui-wireless.md`
  nennt frueh einen `Setup-Grundscreen`
- `docs/software/history/dukatimer-part2-pflichtenheft.md` fuehrt `Setup und
  Persistenz` als Muss-Anforderung
- `docs/software/erledigt/dukatimer-part2-statusaudit-ui-grundgeruest-2026-04-24.md`
  benennt Setup ausdruecklich als spaeteren Ausbaudruck auf den heute noch
  kleinen `ModeCoordinator`

Gleichzeitig ist der aktuelle Ist-Stand klar:

- `ModeRuntimeState::ModeId` kennt bisher nur `Splitgrade` und `BlackWhite`
- `ModeCoordinator` ist auf zwei Workflows fest gebaut
- globale Systemsettings fuer Feedback, Head-Caps oder Thermik existieren noch
  nicht als eigener persistenter Produktpfad
- thermische Schwellen sitzen heute als harte Konstanten in der
  `ExposureEngine`

## Konsequenz fuer Part2

Part2 braucht kein historisches 1:1-Porting des alten Setup-Menues.

Part2 braucht stattdessen:

- ein explizites Setup-Menue als eigenen globalen Workflow
- einen eigenen persistenten `SystemSettings`-Pfad unter Teensy-Autoritaet
- eine klare Trennung zwischen
  - globalen Systemeinstellungen
  - papier- oder workflowbezogenen Fachwerten
  - Maintenance-/Serviceaktionen

## Verbindlicher Produktschnitt

Ins explizite Setup-Menue gehoert nur, was wirklich globale
Systemkonfiguration ist.

### Gehoert hinein

- globale Rueckmeldung fuer Bedienung und Faults
- globale Head-/Safety-Grenzen
- wenige, wirklich modusunabhaengige Bedienparameter

### Gehoert nicht hinein

- `PaperExposureProfile`-Inhalte wie `ISO-P`, `ISO-R`, `kBw`, `kSoft`, `kHard`
- Preflash-Parameter, die papierabhaengig sind
- Kalibrierwizard, Teststrip, Densitometer, Burn oder andere Fachworkflows
- Dark-Calibration, USB-/VFS-Bridge, Pairing-Reset oder aehnliche
  Service-/Maintenance-Aktionen im Haupt-Setup
- sensorrollenbezogene Umschalter wie historisches `WIRELESS PROBE`, weil die
  Rollen in Part2 fachlich bereits gesetzt sind: TSL2561 lokal fuer
  Closed-Loop/Safety, TSL2591 remote fuer Papier-/Transmissionsmessung

Solche Punkte duerfen spaeter als eigene Workflows oder in einem getrennten
Service-Menue erscheinen, aber nicht im normalen Produkt-Setup v1.

Ihre logische Verortung im Produkt-UI ist damit nicht offen: Papierwahl und
Papierkalibrierung gehoeren in die Familie `PAPER`, Zonenvisualisierung und
spaetere Durchlicht-Densitometrie in `MEAS`, Preflash, Teststrip und Burn in
den Druckkontext `PRINT` der EEZ-Anleitung.

Explizite Ausnahme fuer den aktuellen Head-Bring-up:

- Ein runtime-only Aktionspunkt fuer `HEAD DIAG` darf das feste NeoPixel-
  Testpattern plus serielles Timing-Reporting schalten, solange noch kein
  eigenes Service-Menue existiert.
- Dieser Hook ist bewusst nicht persistent, veraendert keine `SystemSettings`
  und dient nur der Hardware-Validierung des aktiven Lichtkopfs.

## Pflichtumfang des Setup-Menues

Die folgenden Punkte sind fuer die erste echte Part2-Setup-Implementierung
verbindlich.

| Gruppe | Eintrag | Verbindliche Semantik |
| --- | --- | --- |
| Feedback | Sound-Modus | Steuert die Ereignispolitik fuer akustisches Feedback. Historisches `OFF/QUIET/NORMAL` wird nicht 1:1 portiert; fuer Part2 soll `Sound-Modus` die Frage beantworten, bei welchen Eventklassen ueberhaupt Ton erzeugt wird. Empfohlen: `Aus`, `Faults`, `Normal`. |
| Feedback | Lautstaerke | Separater, persistenter Pegel fuer den akustischen Sink. Damit wird die historische Vermischung von Politik und Amplitude aufgeloest. Fuer v1 reichen grobe Stufen statt eines feinen Prozentreglers. |
| Feedback | Vibration an/aus | Globaler Schalter fuer den Haptic-Sink des C6-Terminals. Die Ereignissemantik bleibt zentral auf dem Teensy definiert; das Setup schaltet nur die Nutzung des Ausgabegeraets. |
| Kopf | Max. NeoPixel-Helligkeit | Globaler Leistungsdeckel fuer den Kopfpfad. Das ist kein Papierwert, sondern ein systemweiter Cap, der bei Head-Ausgabe wirksam wird. |
| Thermik/Safety | Schwellwert fuer Leistungsreduktion | Temperatur ab der der Kopf nicht mehr mit voller Leistung fahren darf. Die heutige harte `ExposureEngine`-Schwelle ist nur Migrationsanker, nicht mehr der einzige Ort der Konfiguration. |
| Thermik/Safety | Schwellwert fuer Abschalten | Temperatur fuer den zwingenden Hard-Stop. Dieser Wert bleibt safety-kritisch und darf nur mit Plausibilitaetspruefung und sichtbarer Bestätigung veraendert werden. |

## Sinnvoller weiterer Umfang

Die folgenden Einstellungen sind fuer Part2 sinnvoll, ohne das Setup-Menue zu
ueberladen.

| Eintrag | Empfehlung | Begruendung |
| --- | --- | --- |
| Globaler EV-/F-Stop-Schritt | Ja | Historisch in v0.9 als `stepMode` vorhanden und fuer Part2 modeuebergreifend sinnvoll. SG, BW, Burn, Teststrip und spaeter Preflash sollen dieselbe zentrale Schrittlogik nutzen. Der Schritt gehoert deshalb eher ins globale Setup als in einzelne Workflows. |
| Display-Helligkeit | Nur nach Hardwarecheck | Historisch gab es `PWM LCD BRIGHT`, die aktuelle Part2-TFT-Schicht belegt aber noch keinen produktiven PWM-Dimmpfad. Nur aufnehmen, wenn die reale Backlight-Hardware dafuer sauber belegt ist. |

## Bewusst nicht als Setup-Einstellung aufnehmen

| Eintrag | Warum nicht im Setup v1 |
| --- | --- |
| Papierprofile, ISO-P/ISO-R, SG-LUT, Preflash | Das sind Fach- und Papierdaten, keine globale Systemkonfiguration. |
| Messquellen-Umschalter lokal/remote | Widerspricht der bereits gesetzten Rollenaufteilung der Sensoren. |
| Dark-Calibration | Ist eine Mess-/Serviceaktion, keine statische Grundeinstellung. |
| USB-Bridge, Pairing, Error-Reset | Gehoert in ein Service-/Maintenance-Menue, nicht in das normale Produkt-Setup. |
| Thermische Mindestleistung oder Head-Latenzkonstanten | Zu tief im Sicherheits- und Timingmodell; fuer v1 nur dann freigeben, wenn ein echter Servicemodus und Hardwarevalidierung vorliegen. |

## Zielarchitektur fuer die Implementierung

## 1. Eigener Modus statt versteckter Dialoge

Das Setup-Menue soll als eigener Workflow sichtbar werden, nicht als lose
Sammlung von Debugdialogen.

Zielbild:

- neuer `ModeId::Setup`
- eigener `SetupWorkflow`
- eigener `SetupModeRuntimeState`
- sichtbarer Einstieg in der produktiven Navigation

Wichtige Folge:

- der heutige `ModeCoordinator` mit fester Zweierliste ist dafuer zu klein
- die Setup-Einfuehrung ist ein guter Anlass, den Coordinator auf eine
  erweiterbare Workflow-Registry oder eine vergleichbar saubere Form zu heben

## 2. Eigene Persistenz fuer Systemsettings

Globale Setup-Werte duerfen nicht in Paper-Slots oder ad-hoc-Konstanten
versteckt werden.

Empfohlener Schnitt:

- `SystemSettings` als eigener persistenter Datensatz
- getrennt von `PaperExposureProfile` und Slot-Bank
- schema-versioniert und gegen korrupte Daten abgesichert
- Teensy bleibt alleinige Autoritaet fuer Laden, Validieren und Speichern

Fachliche Untergruppen in `SystemSettings`:

- `FeedbackSettings`
- `HeadSafetySettings`
- `UiInteractionSettings`

## 3. Heisse vs. latched Einstellungen

Nicht jede Einstellung muss sofort ohne Schutz wirksam werden.

Empfohlene Regel:

- Sound-Modus, Lautstaerke und Vibration duerfen direkt uebernommen werden
- Max.-NeoPixel-Helligkeit darf nur im Idle geaendert werden und muss sofort
  sichtbar rueckmeldbar sein
- Thermikschwellen duerfen nur im Idle, mit Sicherheitsdialog und mit
  Plausibilitaetspruefung uebernommen werden

## 4. Validierungsregeln fuer Safety-Werte

Thermik- und Leistungsgrenzen brauchen feste Guardrails:

- Derating-Schwelle muss kleiner als Hard-Stop sein.
- Zwischen beiden Werten muss ein Mindestabstand bestehen.
- Unplausible Werte duerfen nicht gespeichert werden.
- Ein `Defaults wiederherstellen`-Pfad fuer diese Gruppe muss vorhanden sein.

Die heute im Code sichtbaren Werte der `ExposureEngine` (`50 C` fuer Derating,
`60 C` fuer Hard-Stop) sind sinnvolle Migrationsanker fuer Defaultwerte, aber
keine Ausrede, die Konfiguration weiter nur als Compile-Time-Konstante zu
fuehren.

## Bedien- und UI-Vorgaben

Das Setup-Menue muss mit derselben Bedienphilosophie wie der Rest des Produktes
funktionieren:

- vollstaendig mit Encodern bedienbar
- Touch darf Komfort sein, nicht einzige Zugriffsschiene
- klare Trennung zwischen Navigation, Editieren, Anwenden, Verwerfen
- keine stillen Hintergrundaenderungen beim Verlassen des Screens

Empfohlene Struktur fuer v1:

- `Feedback`
- `Kopf`
- `Safety`
- `Bedienung`

Empfohlene Pflichtaktionen im Menue:

- `Anwenden`
- `Verwerfen`
- `Defaults fuer Safety wiederherstellen`
- `Feedback testen`

`Feedback testen` ist bewusst eine Aktion und keine weitere Einstellung.

## Akzeptanz fuer die erste Implementierung

Das explizite Setup-Menue gilt fuer Part2 erst dann als sinnvoll umgesetzt,
wenn mindestens diese Punkte nachweisbar sind:

- `Setup` ist ein echter sichtbarer Modus oder gleichwertiger globaler Workflow
  im produktiven UI
- Sound-Modus, Lautstaerke, Vibration, NeoPixel-Maxhelligkeit,
  Derating-Schwelle und Hard-Stop-Schwelle sind sichtbar editierbar
- die Werte sind persistent und ueber Reboot reproduzierbar
- Papierdaten und Kalibrieraktionen liegen nicht in demselben Menuepfad
- unplausible Thermikkombinationen werden abgefangen
- Aenderungen sind mit Encodern allein bedienbar
- aktive Belichtung blockiert Safety-relevante Aenderungen sauber

## Kurzfazit

Die Historie belegt klar, dass ein eigener Setup-Modus fachlich gewollt war.
Part2 sollte diesen Modus jetzt nicht schlicht portieren, sondern architektonisch
sauber neu schneiden:

- globales Setup ja
- aber keine Rueckkehr zum historischen Sammelmodus fuer Papier, Wizard,
  Service und Systemparameter

Fuer v1 reicht ein bewusst kleiner, aber echter Schnitt:

- Feedback
- Head-Cap
- thermische Safety-Grenzen
- globaler EV-Schritt als einziger klar modeuebergreifender Zusatzwert
