# RAM- und PSRAM-Strategie fuer Dukatimer-Part2

Stand: 2026-05-04

## Ziel

Diese Notiz zieht die verifizierte Hardwarelage und die daraus folgende
Projektregel zusammen: vorhandenes RAM ist keine dekorative Reserve, sondern
eine bewusst nutzbare Betriebsressource. Wenn interner RAM, `EXTMEM` am Teensy
oder PSRAM am ESP32-S3 deterministisch helfen, Schreibzugriffe auf SD oder
Flash zu reduzieren oder den Hot Path zu entlasten, sollen sie auch genau dazu
eingesetzt werden.

Die Gegenregel gilt ebenfalls: Speichersparen ist in Part2 kein Selbstzweck,
wenn dadurch waehrend derselben Laufzeit haeufiger auf SD, Flash oder andere
nichtfluechtige Medien geschrieben werden oder wenn nicht zeitkritische Daten
unnuetig in den knappen internen RAM gepresst werden.

## Verifizierte Hardwarebasis

| MCU | Interner RAM | Externer RAM | Verifizierter Projektstand |
| --- | --- | --- | --- |
| Teensy 4.1 | `512 KB` MCU-RAM plus projektspezifische RAM1/RAM2/DMAMEM-Aufteilung | `16 MB` PSRAM | aktive Duka-Teen-Revision meldet reproduzierbar `16 MB`; Nutzung erfolgt explizit ueber `EXTMEM` bzw. `.externalram` |
| ESP32-S3 | Build meldet `320 KB RAM` | PSRAM-faehige N16R8-Konfiguration aktiv | `qio_opi`, `BOARD_HAS_PSRAM`, `CONFIG_SPIRAM` und `CONFIG_SPIRAM_SUPPORT` sind aktiv; der Produktcode nutzt PSRAM derzeit aber noch nicht explizit |

## Verbindliche Projektregel

- Vorhandener RAM und PSRAM sind in Part2 bewusst zu nutzen, wenn sie internen
  Hot-Path-RAM entlasten, Heap-Druck senken, Schreibzugriffe buendeln oder
  nichtfluechtige Medien schonen.
- Ungenutzter PSRAM ist kein Qualitaetsmerkmal. Wenn grosse kalte Daten,
  Diagnosehistorien, Staging-Bloecke oder Imports weiter knapp im internen RAM
  leben oder auf haeufige Medienwrites ausweichen, obwohl externer Speicher
  verfuegbar ist, ist das ein Architekturhinweis und kein Spar-Erfolg.
- Zeitkritische, DMA-nahe und ISR-nahe Daten bleiben trotz vorhandenen PSRAMs
  im jeweils geeigneten internen Speicher.
- Schreibzugriffe auf SD, Flash, LittleFS oder spaetere NVS-Pfade sollen nicht
  an jedem kleinen UI-Schritt erfolgen. Erst RAM, dann validieren, dann
  bewusst und gebuendelt committen.

## Aktueller Ist-Zustand

### Teensy 4.1

Bereits produktiv in externen oder dafuer vorgesehenen Speicher verschoben:

- `PaperSlotBank` liegt in `EXTMEM`.
- die globale `EspServiceLink`-Instanz liegt in `EXTMEM`.
- der Blob-I/O-Puffer fuer `PaperSlotStorage` liegt in `EXTMEM`.
- der Blob-I/O-Puffer fuer `SystemSettingsStorage` liegt in `EXTMEM`.
- LVGL grosse Arrays laufen ueber `LV_ATTRIBUTE_LARGE_RAM_ARRAY` in
  `.externalram`.
- Der LVGL-Heap laeuft ueber `LV_MEM_CUSTOM` und `lvgl_psram_alloc.h` auf
  `extmem_malloc()`. Damit liegen Widget-Objekte, Styles und LVGL-interne
  Heap-Allokationen im PSRAM statt im frueheren statischen RAM1-Pool.
- DMA-relevante LVGL-Draw-Buffers bleiben bewusst in `DMAMEM`; aktueller Stand
  sind zwei Puffer mit je 80 Zeilen bei 480 px Breite.

Konsequenz:

- Der Teensy nutzt das vorhandene externe RAM bereits sinnvoll als Druckventil
  fuer RAM1/RAM2.
- Der Build setzt fuer die aktive Duka-Teen-Revision bestuecktes PSRAM voraus.
  Diese Annahme ist durch den Probe-Sketch belegt und jetzt auch im
  Runtime-Startpfad fail-closed abgesichert: `setup()` prueft
  `external_psram_size` direkt nach dem Bootlog und beendet den Start bei
  fehlendem PSRAM bewusst vor weiterer Subsystem-Initialisierung; `LvglUi::begin()`
  lehnt denselben Fall zusaetzlich weiterhin am UI-Rand ab.
- Die Richtung ist richtig, aber noch nicht voll ausgeschoepft: staged
  Arbeitskopien, Diagnosehistorien und weitere nicht zeitkritische Caches
  duerfen bei Bedarf bewusst wachsen, statt aus Gewohnheit knapp gehalten zu
  werden.

### ESP32-S3

Aktueller Produktstand:

- die Board- und Framework-Konfiguration ist PSRAM-faehig.
- `HttpVfsBridge`, `TeensyLinkService`, `WirelessRemoteGateway` und
  `ServiceSensorHub` arbeiten heute mit kleinen internen Puffern und besitzen
  keine explizite PSRAM-Allokation.
- der ESP streamt Uploads blockweise und haelt aktuell weder grosse Asset-
  Caches noch Import-/Export-Staging oder Diagnose-Ringbuffer im PSRAM.

Konsequenz:

- Auf dem ESP32-S3 ist PSRAM im aktiven Produktcode derzeit eher verfuegbar als
  architektonisch genutzt.
- Genau dort liegt ein bewusst offener Ausbauraum: Web-/VFS-/Importpfade,
  History und Diagnose muessen spaeter nicht kuenstlich klein bleiben.

## Wo RAM und PSRAM Schreibzugriffe verringern koennen

### 1. Papierprofile und globale Einstellungen

Der aktuelle Teensy-Persistenzpfad ist robust, aber write-amplifizierend:

- `PaperSlotStorage` schreibt immer die ganze `PaperSlotBank` als Blob ueber
  `.tmp` plus Rename/Backup.
- `SystemSettingsStorage` folgt demselben Muster fuer globale Einstellungen.

Das ist fachlich richtig fuer Integritaet, erzeugt aber pro Save bewusst mehr
als nur einen einzelnen Dateischreibvorgang. Externes RAM kann hier die
Lebensdauer der Speichermedien schuetzen, wenn Aenderungen zuerst als
Arbeitskopie im RAM gesammelt und erst dann als ein Commit geschrieben werden.

Verbindliche Richtung:

- keine Persistenz pro Encoder-Detent oder pro Einzelfeld-Aenderung
- staged Editoren und Dirty-Shadow-Copies im RAM beibehalten oder ausbauen
- Save erst bei explizitem `Apply`, bewusstem Menueabschluss oder klarer
  Debounce-/Idle-Regel
- mehrere kleine Aenderungen zu einem Blob-Commit buendeln

### 2. Messhistorie, Histogramme und Diagnose

Measurement-, Remote- und Linkdiagnosen sind typische Kandidaten fuer RAM statt
fuer Dauerpersistenz:

- Session-Historien
- Histogramme
- Remote-Command- und Renderdiagnosen
- VFS-Transferdiagnosen
- spaetere Support- oder Hardwareabnahmelogs

Wenn diese Daten spaeter wichtig bleiben, soll die erste Antwort nicht lauten
"frueh wegoptimieren" oder "dauernd auf SD schreiben", sondern:

- in RAM/PSRAM als Ringbuffer halten
- bei Bedarf explizit exportieren
- nur auf Nutzer- oder Servicekommando persistieren

Das senkt Schreibhaeufigkeit, erhaelt Diagnosebreite und verhindert, dass
begrenzte Medienlebensdauer mit Live-Telemetrie verheizt wird.

### 3. HTTP-, VFS-, Import- und Backup-Pfade

Fuer den ESP32-S3 ist dies der naechste logische PSRAM-Nutzen:

- Importdateien, JSON-/CSV-/Backup-Payloads zunaechst im PSRAM stagen
- im RAM validieren, parsen und konsolidieren
- erst danach einen gezielten, transaktionalen Write oder Forward ausloesen

Wichtig ist der Unterschied:

- PSRAM verlaengert die Lebensdauer nicht automatisch
- PSRAM verlaengert sie dann, wenn dadurch aus vielen kleinen Zwischenwrites
  ein einzelner validierter Commit wird

### 4. Generierte Artefakte und Asset-Pfade

Falls spaeter Web-/UI-Assets, exportierte Reports, Diagnosepakete oder
Service-Snapshots hinzukommen, sollen diese zuerst im RAM/PSRAM aufgebaut
werden. Das Medium ist das Ziel fuer den finalen Commit, nicht fuer jeden
Zwischenschritt.

## Platzierungsregeln pro MCU

### Teensy: intern halten

- DMA-nahe Display- und I/O-Bloecke
- ISR-nahe oder harte Timingpfade
- direkte Exposure-, Head- und Safety-Hotpaths, falls Messung keinen anderen
  Ort rechtfertigt

### Teensy: bevorzugt nach `EXTMEM` oder `.externalram`

- grosse Banks, Tabellen und Lookup-Daten
- Blob- und Serialisierungsbuffer
- staged Editorzustand grosser Profile
- UI-/Presenter-/History-Daten mit geringer Timing-Sensitivitaet
- Export-/Diagnosepuffer

### ESP32-S3: intern halten

- UART-Ring- und Handshake-nahe Daten
- ESP-NOW Paket- und Callback-nahe Daten
- kleine Sensor- und Gateway-Zustaende
- alles ISR-, Driver- oder Latenz-nahe

### ESP32-S3: bevorzugte kuenftige PSRAM-Kandidaten

- HTTP-Import-/Export-Staging
- Paperprofil- und Backup-Payloads
- groessere Diagnose- und History-Ringbuffer
- Web-/Asset-Caches
- groessere, aber nicht zeitkritische Brueckenqueues

## Anti-Pattern, die vermieden werden sollen

- RAM knapp rechnen, waehrend PSRAM frei bleibt und dieselben Daten dann haeufig
  auf SD oder Flash landen
- pro Encoderbewegung oder pro UI-Zwischenschritt persistieren
- Diagnosehistorie kuenstlich klein halten, nur um Speicher sparsam aussehen zu
  lassen
- kalte Daten aus Gewohnheit in internen RAM ziehen, obwohl sie den Hot Path
  nicht stuetzen
- Hot-Path-, DMA- oder ISR-Daten ohne Messbeleg in PSRAM verschieben
- PSRAM als Vorwand fuer grenzenlose Dauerlogs auf nichtfluechtige Medien
  missverstehen

## Konkrete Prioritaeten fuer naechste Slices

1. Paper- und Systemsettings-Pfade als explizite RAM-first-Commitpfade
   beibehalten und bei Bedarf weiter entkoppeln: mehrere UI-Aenderungen sollen
   einen Persistenzvorgang ergeben, nicht viele.
2. Teensy-Startpfad um eine fruehe, sichtbare PSRAM-Pruefung ergaenzen, damit
  die harte Build-Annahme `DUKATIMER_TEENSY_HAS_PSRAM=1` bei falscher Hardware
  fail-closed statt spaet ueber LVGL-Heap-NULLs sichtbar wird.
3. Auf dem ESP32-S3 einen kleinen PSRAM-Diagnosepfad vorsehen, damit aktiv
   sichtbar bleibt, ob PSRAM vorhanden und frei ist.
4. Fuer HTTP-Import/Export sowie Backups einen PSRAM-Stagingpfad einziehen,
   bevor aus dem aktuellen Servicepfad ein Produktpfad wird.
5. Diagnose- und Session-Historien zuerst als RAM-/PSRAM-Ringbuffer auslegen;
   Persistenz bleibt Exportfunktion, nicht Default-Livepfad.

## Schlussfolgerung

Part2 soll vorhandenen RAM nicht aus dem Blick verlieren. Die vorhandenen
Speicherreserven am Teensy und die PSRAM-Faehigkeit des ESP32-S3 sind Teil der
Architektur, nicht bloss ein Datenblattmerkmal.

Die Leitfrage fuer kuenftige Slices lautet deshalb nicht nur
"Wie sparen wir Speicher?", sondern genauso:

- "Welche Daten muessen intern-heiss bleiben?"
- "Welche kalten oder grossen Daten koennen bewusst in externen RAM?"
- "Wo verhindert RAM einen unnoetigen Write auf SD oder Flash?"
- "Wo ist mehr Historie, Staging oder Diagnose fachlich wertvoller als ein
  kuenstlich kleiner Speicher-Footprint?"

Wenn diese Fragen aktiv gestellt werden, nutzt Part2 den vorhandenen Speicher
zielfuehrend statt ihn ungenutzt liegen zu lassen.
