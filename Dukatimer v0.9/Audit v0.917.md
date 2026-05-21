# Dukatimer v0.917 - Bulletproof Recovery & Orphan Audit

## Reparierte Build-Fehler

### 1. Beschädigter HardwareManager
- Ursache: In `src/HardwareManager.cpp` war oberhalb der eigentlichen Includes ein zerstörter Codeblock eingeklebt. Die Datei enthielt eine doppelte, syntaktisch defekte `syncPhysicalLights()`-Implementierung, Verweise auf nicht existierende `setLCDBacklight()`-Hilfen und unvollständige Funktionsköpfe.
- Reparatur:
  - `include/HardwareManager.h` und `src/HardwareManager.cpp` wurden konsistent neu verdrahtet.
  - Die Signatur `syncPhysicalLights(const uint8_t swStableState[3])` ist jetzt wieder identisch zwischen Header und CPP.
  - Die v0.3-Licht-Hierarchie wurde in eine zentrale `renderLightOutputs()`-Pipeline zurückgeführt.
  - Darkness Mode, Safelight-Priorität, Focus-Latch und Roomlight-Relais-Invertierung sind wieder an einer Stelle implementiert.

### 2. InputManager/HardwareManager Signaturbruch
- Ursache: `InputManager::init()` rief `syncPhysicalLights()` ohne Parameter auf, obwohl der Header längst `syncPhysicalLights(const uint8_t[3])` verlangte.
- Reparatur:
  - `src/InputManager.cpp` übergibt jetzt den entprellten Dreifach-Schalterzustand konsistent an den HardwareManager.
  - Die direkte, teilweise widersprüchliche Einzelansteuerung von Safelight/Focus/Room über `handleSwitchState()` wurde wieder auf den zentralen Hierarchiepfad zurückgeführt.

### 3. BW-/SG-Time Kontrollpfad war semantisch falsch verdrahtet
- Ursache:
  - `BWTimeApp` rief `setBWPending(_targetTimeS, _currentGrade)` auf. Das kompilierte zwar durch implizite Typkonversion, war logisch aber falsch: `float` wurde zu `bool`, die Zeit-App startete nicht sauber im Zeitmodus.
  - `SGTimeApp` setzte zwar `setSGTimePending()`, aber die `ExposureEngine` hatte keinen vollständigen Zwei-Phasen-Zeitpfad für den Splitgrade-Zeitmodus.
- Reparatur:
  - `BWTimeApp` startet jetzt bewusst über `setBWPending(true, _targetTimeS)` und markiert den Modus als Zeitbetrieb.
  - `ExposureEngine::tick()` behandelt `MODE_BW` nun als Zeitstartpfad statt irrtümlich als Dosisstart.
  - Für `MODE_SG` wurde ein interner Split-Zeit-Sequencer reaktiviert: Soft-Phase startet zuerst, Hard-Phase wird nach Abschluss automatisch nachgezogen.

### 4. F-Stop Schrittweite war nur im Setup sichtbar, aber nicht in der App wirksam
- Ursache: `SetupApp` konnte `stepMode` ändern, `BWFStopApp` rechnete aber starr mit `12.0f` Ticks pro Blende und ignorierte die Preference vollständig.
- Reparatur:
  - `BWFStopApp` liest die Schrittauflösung jetzt aus `SystemPreferences.stepMode`.
  - Das Tick-Clipping richtet sich wieder an der gewählten Blendenauflösung aus.
  - `SetupApp` zeigt `1/6` korrekt an, statt aus `stepMode == 4` fälschlich `1/4` zu rendern.

### 5. Event-Semantik war partiell verwaist
- Ursache:
  - `EV_GRADE_CLICK` wurde von fast keiner App als BACK/ABORT genutzt.
  - `EV_TIME_CLICK` war nur lokal in einer App als ENTER verdrahtet, während Setup/TestStrip weiter auf `EV_MENU_CLICK` hörten.
- Reparatur:
  - `AppManager` übersetzt `EV_GRADE_CLICK` nun systemweit in `EV_ABORT`.
  - Für `MODE_SETUP` und `MODE_TEST_STRIP` wird `EV_TIME_CLICK` zentral in `EV_MENU_CLICK` überführt.
  - Pending-States (`bwAutoPending`, `sgAutoPending`) blockieren Moduswechsel jetzt ebenfalls, damit keine Zwischenzustände orphaned zurückbleiben.

## Wiederhergestellte Leichen / Routing-Lücken

### 1. Hardware-Schalterpfad reanimiert
- Wiederhergestellt:
  - Entprellte Schalterzustände aus `InputManager`
  - Übergabe an `HardwareManager::syncPhysicalLights()`
  - zentrale Auswertung für:
    - Darkness Mode
    - Safelight-Priorität
    - Focus-Latch (Anti-Flashback)
    - Roomlight-Relais
    - NeoPixel Focus/Safe/Exposure Rendering
- Wirkung: Die physische v0.3/v0.5 Logik hängt nicht mehr in totem Einzelcode, sondern ist wieder an den Runtime-Pfad angeschlossen.

### 2. BWTime / SGTime im Runtime-Pfad wieder funktional verankert
- `AppManager` routet beide Apps bereits, aber die Laufzeitverkabelung war defekt.
- Wiederhergestellt:
  - BW-Time -> Engine `startTime()` via `bwAutoPending`
  - SG-Time -> zweiphasige Zeitsequenz via `sgAutoPending`
  - passende UI-Zustände im DisplayManager für `MODE_BW` und `MODE_SG`

### 3. Darkness Mode bis ins Display re-verkabelt
- `DisplayManager` berücksichtigt jetzt Hardware-Blackout über `HardwareManager::isDarknessModeActive()`.
- Wirkung:
  - Nextion wird auf `dim=0` gezogen.
  - Grove LCD Backlight wird auf schwarz gezogen.
  - Rückkehr aus dem Blackout restauriert die Helligkeit aus `pwmLcd` statt hart auf 100%.

## Befunde aus dem v0.3/v0.5 Feature-Abgleich

## Vollständig bzw. wirksam zurückgeführt
- Safety Latch (Anti-Flashback): wieder zentral im HardwareManager umgesetzt.
- Darkness Mode (Schalter 3): Roomlight-Relais invertiert, Displays werden dunkel, Lichtausgabe wird blockiert.
- Encoder-Skalierung `diff / 2`: bleibt in `InputManager::processEncoders()` aktiv.
- Metronom: weiter aktiv in `ExposureEngine::tick()`.
- BW-/SG-Time Apps: nicht mehr nur kompilierbar, sondern wieder an die Engine angebunden.
- F-Stop Schrittlogik: Setup und Runtime greifen wieder zusammen.

## Weiterhin offene Migrationslücken

### 1. BW-Gradationsabbildung auf Soft/Hard-Split zurückgeführt
- Status: repariert.
- Umsetzung:
  - Die v0.3-Gradationsmathematik wurde architekturkonform in `PaperManager` verankert, nicht in `ExposureEngine` oder `HardwareManager`.
  - `PaperManager` berechnet daraus nun die fertigen Soft-/Hard-Anteile für Zeit und Dosis.
  - `BWTimeApp` und `BWDoseApp` schreiben diese vorberechneten Split-Werte in den `SystemContext`.
  - `ExposureEngine` liest nur noch die fertigen Soft-/Hard-Phasen aus und führt sie sequentiell aus.
- Restgrenze: Fixed-Grade bleibt ein Einphasen-Fall ohne künstliche Aufspaltung.

### 2. Sensor-Fallbacks aus v0.3 sind nicht vollständig modular portiert
- Status: Sensorik ist im neuen `SensorManager` gekapselt, aber ein expliziter Fallbackpfad im Stil der alten lokalen Ersatzmessung ist nicht vollständig nachgewiesen.
- Bedeutung: Bei Sensorverlust ist die Architektur sauberer, aber die alte Ausweichlogik ist noch nicht 1:1 in modulare Dienste überführt.

### 3. Remote-/Blindbedien-Feinheiten aus v0.3/v0.5 sind nur teilweise migriert
- Status: Die Kern-Events `EV_START`, `EV_TIME_CLICK`, `EV_GRADE_CLICK` sind jetzt systemisch sauberer verdrahtet.
- Offen: App-spezifische ENTER-Semantik ist noch nicht in jeder Spezial-App vollständig ausgebaut. Die globale Übersetzung verhindert Leerlauf-Events, ersetzt aber nicht jede mögliche Workflow-Optimierung.

### 4. `SystemContext::triggerAbort()` bleibt Sicherheitskern
- Status: aktiv korrigiert.
- Bedeutung: `triggerAbort()` ist kein Orphan, sondern der harte Laufzeit-Abbruchpfad für den Start-Taster bei aktiver Hardware.
- Semantik:
  - `EV_START` startet im Idle eine Belichtung und stoppt sie hart während laufender Hardwareaktion.
  - `EV_GRADE_CLICK` bleibt ausschließlich UI-Back/Abort und ist nicht der Hardware-Not-Aus.

## Validierung

- PlatformIO Build: erfolgreich
- Umgebung: `esp32-s3-devkitc-1`
- Ergebnis: `firmware.elf` und `firmware.bin` wurden nach den Reparaturen wieder erfolgreich erzeugt.