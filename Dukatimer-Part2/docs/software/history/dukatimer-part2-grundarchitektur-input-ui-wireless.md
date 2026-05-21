# Dukatimer-Part2 Grundarchitektur fuer Input, UI und Wireless

Stand: 2026-04-20

## Ziel

Dieses Dokument legt die Startentscheidungen fest, die vor oder parallel zur ersten Fachlogik stabil sein muessen. Es adressiert die drei Problemfelder, die in v0.3 und v0.9 zu Unordnung gefuehrt haben oder fuer Part2 nicht erst spaet geklaert werden duerfen:

- Encoder- und Inputverhalten
- UI-/LVGL-/TFT-Basis
- Wireless-Integration

Die erste fachliche Implementierungsstufe baut darauf auf und umfasst bewusst nur:

- Papierkalibrierung
- Modus SG
- zugehoerige Mathematik
- zugehoerige Belichtungslogik

Alle weiteren Modi werden spaeter auf diese Basis aufgesetzt.

## 1. Verbindliche Startentscheidungen

1. Encoderrollen und Eventtypen werden vor der Screen-Implementierung festgelegt.
2. Die Encoder 1 bis 3 sind ein lokales Echtzeit-Eingabesystem des Teensy und werden nicht nach dem alten ESP32-Muster mit grober Software-Entprellung behandelt.
3. LVGL, TFT, Touch und Navigation werden als eigene Basisarchitektur definiert und dokumentiert.
4. Das Wireless-TSL2591-Handgeraet ist von Beginn an Teil von Nachrichtenmodell, Statusmodell und UI-Modell.
5. Der erste fachliche Slice ist nicht "alles ein bisschen", sondern gezielt Papierkalibrierung plus SG.

## 2. Hardwarebezogene Zuordnung

Abgeleitet aus der vorhandenen Part2-Hardwaredoku gilt:

- Encoder 1 bis 3 liegen auf dem Teensy 4.1.
- Encoder 4 liegt auf dem ESP32-S3.
- Touch, TFT und die lokale Hauptbedienung liegen beim Teensy.
- Das Wireless-TSL2591-Handgeraet koppelt ausschliesslich per ESP-NOW an den ESP32-S3.
- Die Kernlogik bleibt trotz Wireless und ESP-Servicepfaden auf dem Teensy.

Relevant fuer die Systemarchitektur ist damit folgende Aufteilung:

- Teensy: lokale Primärbedienung, LVGL-UI, Screenzustand, Fachlogik, Belichtungslogik
- ESP32-S3: Zusatzbedienung ueber Encoder 4, 1-Wire, weitere Services und Funk-Gateway

## 2.1 Lokale Schalterlogik im ersten Schritt

Noch vor den ersten Fachmodi wird die lokale Schalterlogik verbindlich festgelegt. Diese drei Schalter sind Bestandteil der Startbasis und nicht spaeteres UI-Detail:

- `Schalter Fokus`: setzt den NeoPixel-Lichtkopf auf Weiss mit 100 Prozent
- `Schalter Save`: setzt den NeoPixel-Lichtkopf ausschliesslich auf Rot
- `Schalter Room`: schaltet ueber das SSR-Relais die externe Raumbeleuchtung aus

Diese Schalter werden als eigene Licht- und Umfeldsteuerung behandelt. Fachscreens duerfen ihre Rohzustande nicht direkt interpretieren, sondern nur die normierten und abgeleiteten Zustandsflags verwenden.

## 2.2 Prioritaet und Hierarchie der Schalter

Die Schalterhierarchie wird aus den bewaehrten Regeln der Altversionen uebernommen:

1. `Room` bildet den Darkness-Pfad fuer die externe Raumbeleuchtung.
2. `Save` hat Prioritaet vor `Fokus`.
3. `Fokus` ist nur gueltig, wenn weder `Save` noch ein hoeherer Blackout- oder Sicherheitszustand aktiv ist.

Abgeleitete Ausgangslogik fuer den ersten Schritt:

- `Room aktiv`: SSR-Raumlicht wird ausgeschaltet
- `Save aktiv`: NeoPixel nur Rot
- `Fokus aktiv`: NeoPixel Weiss 100 Prozent
- kein aktiver Schalter: NeoPixel aus, Raumlicht nach Room-Status

Wird spaeter ein uebergeordneter Belichtungs-, Fehler- oder Blackout-Zustand aktiv, darf dieser die Schalterausgaenge weiterhin hart uebersteuern.

## 2.3 SaveLatch-Logik

Die SaveLatch-Logik wird aus dem Altverhalten explizit uebernommen.

Regelwerk:

- Ist `Save` aktiv, hat `Fokus` keine Wirkung auf die physische Lichtausgabe.
- Wird `Save` ausgeschaltet, waehrend `Fokus` physisch noch auf `AN` steht, bleibt das System dunkel.
- `Fokus` darf in diesem Fall nicht automatisch wieder anspringen.
- Der Latch wird erst geloescht, wenn `Fokus` bewusst einmal auf `AUS` geschaltet wurde.
- Erst ein anschliessendes erneutes `AN` von `Fokus` darf wieder Weiss 100 Prozent einschalten.

Damit wird verhindert, dass beim Verlassen des roten Safe-Zustands unbeabsichtigt weisses Licht aufblitzt.

## 2.4 Zustandsmodell der Schalterbasis

Die Startarchitektur braucht dafuer mindestens diese logischen Zustandsgroessen:

- `focusSwitchRaw`
- `saveSwitchRaw`
- `roomSwitchRaw`
- `saveLatchActive`
- `derivedFocusOutput`
- `derivedSaveOutput`
- `derivedRoomOutput`

Architekturregel:

- Latch- und Prioritaetslogik liegen zentral in der Hardware- oder Light-Control-Schicht.
- UI und Fachmodi lesen nur den abgeleiteten Zustand.
- Direkte Pin- oder Schalterabfragen in Moduscode sind unzulaessig.

## 3. Encoderarchitektur

## 3.1 Zielbild

Die Encoder duerfen nicht mehr pro Modus oder pro Datei anders interpretiert werden. Es gibt eine einzige Input-Schicht, die aus Rohsignalen normierte Ereignisse erzeugt. Fachlogik und UI arbeiten nur noch mit diesen normierten Ereignissen.

Normierte Ereignistypen:

- `RotateLeft`
- `RotateRight`
- `Press`
- `LongPress`
- `RepeatPress` nur falls spaeter wirklich benoetigt

## 3.2 Rollenmodell

Die exakte Screenbelegung kann spaeter verfeinert werden, die Grundsemantik wird aber frueh festgezogen:

- Encoder 1: primaere Wertachse des aktuellen Screens
- Encoder 2: sekundaere Wertachse oder Unterparameter
- Encoder 3: Navigation, Fokuswechsel oder Kontextaktion
- Encoder 4: service- oder remote-bezogene Zusatzbedienung ueber den ESP32-S3

Wichtig: Kein Screen liest GPIOs direkt. Alle Screens arbeiten ausschliesslich mit normierten Input-Events.

## 3.3 Teensy-spezifische Entscheidung

Fuer Encoder 1 bis 3 soll explizit der Vorteil des Teensy genutzt werden:

- latenzarme, lokale Verarbeitung direkt auf dem Hauptcontroller
- interrupt- oder hardwaregestuetzte Quadraturauswertung
- saubere Ereignisnormalisierung pro Rastung statt spaeter Modus-spezifischer Korrekturlogik
- getrennte Behandlung von A/B-Signalen und Encoder-Taster

Architekturregel:

- Die A/B-Leitungen der Teensy-Encoder werden nicht ueber eine grobe Zeitentprellung "beruhigt" und dann erst ausgewertet.
- Stattdessen werden gueltige Quadraturfolgen akzeptiert, ungueltige Uebergaenge verworfen und daraus stabile Drehereignisse erzeugt.
- Nur die Encoder-Taster und sonstige Taster erhalten eine klar definierte Debounce-Stufe.

Damit wird die alte ESP32-Entprelllogik nicht 1:1 portiert, sondern bewusst durch eine Teensy-first-Eingabeschicht ersetzt.

## 3.4 ESP-seitiger Encoder 4

Encoder 4 bleibt im Besitz des ESP32-S3. Dort darf weiterhin lokal gefiltert oder entprellt werden, aber an den Teensy werden nur semantische Ereignisse weitergegeben.

Beispiel:

- zulassig ueber die MCU-Grenze: `RotateLeft`, `RotateRight`, `Press`, `LongPress`
- unzulaessig ueber die MCU-Grenze: rohe Flanken, Pinzustandswechsel oder screen-spezifische Sonderinterpretationen

## 4. UI-, TFT- und LVGL-Basis

## 4.1 Grundsatz

Die lokale UI laeuft auf dem Teensy. LVGL ist der UI-Kern fuer das TFT-/Touch-System. Die UI-Basis muss stabil stehen, bevor mehrere Fachmodi darauf aufsetzen.

## 4.2 Schichtenmodell

Die UI wird in klar getrennte Schichten aufgeteilt:

- `DisplayHAL`: Display-Initialisierung, Flush, Backlight, ggf. Buffer-Strategie
- `TouchHAL`: Touch-Initialisierung, Touch-IRQ, Rohkoordinaten, Kalibrierung
- `InputRouter`: Uebersetzung normierter lokaler und entfernter Eingaben in UI-Aktionen
- `UiKernel`: LVGL-Tick, Screenwechsel, Fokusmodell, globale Overlays und Popups
- `Screen/Presenter`: fachliche Screens wie SG oder Papierkalibrierung

## 4.3 Frueh zu dokumentierende Artefakte

Vor der breiten Modusimplementierung muessen mindestens diese Punkte schriftlich festliegen:

- Pin- und Besitzmodell fuer Display, Touch, Encoder und Taster
- Ereignismodell fuer lokale und Wireless-Eingaben
- Navigationsmodell zwischen Touch, Encodern und optionalen Remote-Eingaben
- Lebenszyklus eines Screens
- Modell fuer Statusleiste, Fehlermeldungen, modale Dialoge und Remote-Verbindungsstatus

## 4.4 Startscreens des ersten Slice

Fuer den ersten fachlichen Slice reichen wenige, klare Screens:

- Boot und Systemstatus
- SG-Hauptscreen
- Papierkalibrierungs-Wizard
- Setup-Grundscreen
- Wireless-/Remote-Status

## 5. Wireless-Basisintegration

## 5.1 Grundsatz

Wireless ist nicht mehr eine spaete Zusatzfunktion. Schon die fruehe Architektur muss annehmen, dass ein externes Handgeraet vorhanden sein kann und dass dessen Status im System sichtbar ist.

## 5.2 Mindestumfang der fruehen Integration

Von Beginn an vorgesehen werden muessen:

- Peer-Bindung oder definierter Remote-Partnerzustand
- Heartbeat und Timeout-Verhalten
- Remote-Eingabeereignisse
- Messkommandos an das Handgeraet
- Lux- oder Messwert-Rueckkanal
- Render- oder Statusdaten vom Hauptsystem an das Handgeraet
- sichtbarer Remote-Verbindungsstatus im Haupt-UI

## 5.3 Architekturregel

Das Handgeraet bleibt ein Dumb Terminal. Autoritativer Zustand, Papierlogik, SG-Mathematik und Belichtungsentscheidung bleiben immer beim Teensy.

## 6. Erster fachlicher Slice

## 6.1 Enthalten

Der erste fachliche Slice umfasst:

- lokale Schalterlogik fuer Fokus, Save und Room inklusive SaveLatch
- Papierprofilmodell mit SG-relevanten Parametern
- Papierkalibrierung
- Modus SG
- notwendige SG-Mathematik
- SG-Belichtungslogik inklusive ExposureEngine
- fruehe Wireless-Pfade fuer den SG-Workflow

## 6.2 Nicht im ersten Slice

Diese Teile folgen bewusst spaeter:

- BW
- Burn
- Teststrip
- Densitometer und Filmtest
- Preflash
- LiveView
- Zone-Modus
- eine eigenstaendige BW-F-Stop-Oberflaeche

## 7. Konsequenz fuer die Umsetzung

Wenn spaeter weitere Modi hinzukommen, duplizieren sie weder Encoderlogik noch UI-Grundlagen noch Wireless-Protokolllogik. Sie benutzen nur die bereits definierte Basis.

Das gilt ausdruecklich auch fuer fotografische EV-/F-Stop-Logik, Messwertkonvertierung und Formatter. Was in mehreren belichtungsrelevanten Modi gebraucht wird, wird einmal zentral definiert und nicht pro Modus neu gebaut.

Messwerte aus echten Messvorgaengen werden in dieser gemeinsamen Basis zusaetzlich als EV aufbereitet, sofern die fotografische Einordnung sinnvoll ist. Rohtelemetrie ohne fotografische Deutung bleibt in Lux oder der jeweils physischen Grundeinheit.

Genau das ist der Schutz gegen das Durcheinander, das in v0.3 und v0.9 bei der Encodersteuerung entstanden ist.
