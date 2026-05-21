# Dukatimer-Part2 – Schritt 13 Review (2026-05-04)

Geprüfter Stand: `0.2.33-dev`, Stage `LVGL/EEZ PSRAM busy screen`  
Build: `teensy41` **SUCCESS** — RAM1 free `23552`, RAM2 variables `166016`, EXTRAM variables `22144`  
Problems-Check: alle geänderten Dateien sauber.

---

## Positive Umsetzung

| Befund | Schwere | Priorität | Erläuterung | Lösungsweg |
| --- | --- | --- | --- | --- |
| LVGL-Heap korrekt nach PSRAM verschoben | Positiv Hoch | P0 erledigt | `lv_conf.h` nutzt `LV_MEM_CUSTOM`, `lvgl_psram_alloc.h` bindet `extmem_malloc()` an. Entlastet RAM1 und folgt der PSRAM-Strategie. | Beibehalten; der fruehe PSRAM-Runtime-Check im Startpfad ist inzwischen umgesetzt. |
| Draw-Buffer richtig in RAM2/DMAMEM | Positiv Hoch | P0 erledigt | `LvglUi.cpp` nutzt 80 Zeilen; DMA-nahe Buffer bleiben nicht im PSRAM. Gute Trennung zwischen DMA-RAM und kaltem UI-Heap. | So lassen; weitere Vergrößerung nur nach Build- und Displaytest. |
| Busy-Screen hat korrekte Safety-Priorität | Positiv Hoch | P0 erledigt | `LvglUi.cpp` erzwingt `SCREEN_ID_BUSY` für `PreWait`, `Exposing`, `Paused`, `PostWait`. Keine normale Seite kann aktive Exposure überdecken. | Beibehalten; später mit finalem EEZ-Design konsolidieren. |
| Busy-Header vermeidet Widget-Pointer-Kollision | Positiv Mittel | P1 erledigt | `screens.c` baut den Busy-Screen ohne `build_header()`, eigene `busy_hdr_*` bleiben getrennt von normalen `hdr_*`. | Als harte UI-Regel dokumentiert und in Repo-Memory abgelegt. |
| Button-Routing bleibt engine-validiert | Positiv Mittel | P1 erledigt | `main.cpp` pollt Busy-Actions und ruft erst dann `resume()` oder `stopGracefully()`. Die Engine prüft den Zustand selbst. | Später ggf. über InputRouter semantisch vereinheitlichen. |
| Version und Doku synchronisiert | Positiv Mittel | P1 erledigt | Teensy steht auf `0.2.33-dev` in `FirmwareVersion.h`, CHANGELOG dokumentiert PSRAM, Busy-Screen und Speicherstand. | Bei jedem weiteren Runtime-Slice so weiterführen. |

---

## Negative Umsetzung

| Befund | Schwere | Priorität | Erläuterung | Lösungsweg |
| --- | --- | --- | --- | --- |
| PSRAM-Fail-Safe im Startpfad umgesetzt | Positiv Mittel | P1 erledigt | Der Code prueft `external_psram_size == 0` jetzt direkt in `setup()` und haelt den Bootvorgang mit klarer Seriendiagnose an; `LvglUi::begin()` behaelt denselben Guard als zweite Sicherung. | Beibehalten; Hardwaretest fuer falsche oder fehlende PSRAM-Bestueckung bei Gelegenheit gegenpruefen. |
| Busy-Touch-Aktionen umgehen InputRouter | Mittel | P1 | Der C-Callback in `LvglUi.cpp` ist bewusst eng, aber er läuft neben der normalen InputRouterPolicy. Das ist für Pause-Buttons okay, bleibt aber eine Sonderstrecke. | Bei produktiver Pause/Abort-Bedienung als semantische Input-Aktion modellieren oder Guard-Regeln explizit spiegeln. |
| Paper/Setup/Measurement/Remote bleiben Stub-Screens | Mittel | P2 | `offene_punkte.md` nennt den UI-Stand als teilweisen handgeschriebenen EEZ/LVGL-Zwischenstand. | Nächste Slices: Paper, Setup, Measurement, Remote als echte Widget-Bäume. |
| Touch-Parität fachlich festgelegt, Hardware-Abnahme noch offen | Mittel | P2 | Die Bedienung ist jetzt klar gezogen: Vorgang startet über die Start-Taste, im Busy-State pausiert dieselbe Taste und setzt aus `Paused` wieder fort; Abbruch läuft über den Encoder-Button auf der bestehenden Zurück/Undo-Schiene. Die vollständige Hardware-Abnahme für Exposing/Paused/Abort steht aber noch aus. | Hardware-/Input-Abnahme für Exposing- und Paused-State ergänzen; lokalen Encoder-Button als dokumentierten Zurück/Abbruch-Pfad in der Bedien-Doku festhalten. |

---

## Landminen

| Befund | Schwere | Priorität | Erläuterung | Lösungsweg |
| --- | --- | --- | --- | --- |
| `FLASHMEM` gilt auch für Callbacks | Hoch | P0 erledigt | `busy_btn_cb` war zuerst nicht `FLASHMEM`; neue C-Funktionen landen sonst still in ITCM/RAM1. Korrigiert in `screens.c` (Z. 136), auch `create_screens()` ist jetzt FLASHMEM (Z. 674). | Weiterhin jede neue `screens.c`-Funktion gegen `FLASHMEM` prüfen. |
| Shared Header darf nicht vom Busy-Screen gebaut werden | Hoch | P0 | Ein `build_header()` im Busy-Screen würde `g_duka_widgets.hdr_*` auf Busy-Objekte umbiegen und normale Seitenupdates brechen. | Busy immer mit eigenen `busy_hdr_*`-Handles halten; bei EEZ-Regeneration prüfen. |
| Remote-Progress ist nicht deckungsgleich mit Busy | Niedrig–Mittel | P3 | `main.cpp` berechnet Time-Progress nur für `Exposing`; Paused/PostWait können remote `255` bekommen, während Busy lokal nun hält/100 zeigt. | Remote-Progress auf denselben Invariant bringen: PreWait 0, Exposing berechnen, Paused halten oder aus Engine exportieren, PostWait 100. |
| EEZ-generierte Pragma-Warnung bleibt | Niedrig | P3 | Build zeigt bekannte `-Wdangling-pointer`-Pragma-Warnung aus `eez-flow.h`, aber kein Buildfehler. | Bei EEZ-Code-Refresh Generator/Warning-Guard prüfen; aktuell nicht funktional blockierend. |

---

## Logikfehler

| Befund | Schwere | Priorität | Erläuterung | Lösungsweg |
| --- | --- | --- | --- | --- |
| Busy-Zeitfortschritt lief in Pause weiter | Mittel | P1 erledigt | `phaseAgeMs` misst in `Paused` die Pausendauer, nicht echte Belichtungszeit. Dadurch hätte der Balken im Zeitmodus weiterlaufen können. Behoben: `LvglUi.h` speichert `busyProgressPercent_`; `LvglUi.cpp` setzt bei PreWait zurück, aktualisiert nur bei Exposing, hält in Pause und setzt PostWait auf 100. | Erledigt; später besser als Engine-Telemetrie `exposureProgressPercent` exportieren. |
| Start-Taste ist als Pause-/Resume-Producer verdrahtet | Mittel | P1 erledigt | `Start` bleibt der lokale Blindpfad: Der Taster startet im Idle, pausiert waehrend `ExposurePhase::Exposing` und setzt aus `Paused` wieder fort. Touch-`PAUSE` und lokale Tasterbedienung treffen sich an derselben SG-/Engine-Grenze. | Erledigt; Hardwaretest fuer Exposing → Paused → Resume und Encoder-Abbruch noch dokumentieren. |
| Kein neuer P0-Fehler im Exposure-Safety-Pfad gefunden | Info | P0 | `Resume`/`Abort` werden erst durch `ExposureEngine` validiert; `stopGracefully()` akzeptiert nur PreWait/Exposing/Paused, `resume()` nur Paused. | Kein Fix nötig; bei Touch-Erweiterungen diese Engine-Grenze nicht umgehen. |

---

## Geänderte Dateien in diesem Schritt

| Datei | Änderung |
| --- | --- |
| `src/teensy/FirmwareVersion.h` | Version auf `0.2.41-dev`, Stage `early psram startup fail-safe` |
| `src/teensy/LvglUi.h` | `busyProgressPercent_` Member ergänzt |
| `src/teensy/LvglUi.cpp` | Busy-Fortschrittslogik: PreWait=0, Exposing=berechnen, Paused=halten, PostWait=100 |
| `src/teensy/ui/eez_ui/.../screens.c` | `busy_btn_cb` und `create_screens()` mit `FLASHMEM` versehen |
| `CHANGELOG.md` | Schritt 13 plus frueher PSRAM-Startabbruch und Build-Validierung dokumentiert |
| `docs/hardware/ram-und-psram-strategie.md` | LVGL-Heap-PSRAM-Status und jetzt umgesetzter frueher PSRAM-Startcheck |
| `docs/software/eez-studio-step-by-step-und-seitenuebersicht.md` | Ist-Stand-Tabelle, Schritt 7 als Busy-Screen, Exposure-als-Overlay-Abschnitt korrigiert |
| `docs/software/offene_punkte.md` | Version aktualisiert, PSRAM/Busy in code-verifizierter Basis, P2-UI-Status |
| `docs/software/offene_ziele.md` | Z2 mit `SCREEN_ID_BUSY`-Hinweis ergänzt |
