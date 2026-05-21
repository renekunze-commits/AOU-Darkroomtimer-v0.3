Dukatimer Beta - Master ToDo-Liste

Diese Liste dient als iterativer Fahrplan für das Refactoring von v0.5 zur modularen Beta-Architektur.

Phase 1: Das Fundament (Daten & Services)

Ziel: Eliminierung der Globals.h und Kapselung kritischer Ressourcen mit Hardware-Interlocks.

[ ] 1.1 SystemContext erstellen (SystemContext.h/.cpp)

[ ] Definition der Structs ExposureParams und DeviceStatus.

[ ] Implementierung des Thread-Schutzes (Mutexe für UI-Zugriff, Atomics/Spinlocks für Core 1).

[ ] Migration der ersten Leitvariablen (target_dose, time_bw, etc.) aus Globals.h.

[ ] 1.2 HardwareManager (HAL) erstellen (HardwareManager.h/.cpp)

[ ] Kapselung von Relais, Safelight, Fokuslicht und NeoPixel.

[ ] Implementierung der Funktion setExposureMode(bool active).

[ ] Feature: Sensor-Lockout für BME280/BMP280 auf I2C-Bus 0 während Belichtung.

[ ] 1.3 PaperManager erstellen (PaperManager.h/.cpp)

[ ] Auslagerung der PSRAM-Verwaltung aus Logic_Papers.cpp.

[ ] Feature: Harter Interlock – Verbot von Schreib-/Lesezugriffen während laufender Belichtung.

[ ] Methode getActiveProfileCopy() für thread-sicheren Lesezugriff der Math-Engine.

[ ] 1.4 StorageManager erstellen (StorageManager.h/.cpp)

[ ] Auslagerung von LittleFS und NVS aus Logic_Storage.cpp.

[ ] Feature: Deferred Writing – Verzögertes Schreiben in den Flash-Speicher, das zwingend blockiert wird, solange Licht emittiert wird.

Phase 2: Der Kernel (Orchestrierung)

Ziel: Ein "Betriebssystem", das Events routet und Belichtungen absichert.

[ ] 2.1 App-Interface definieren (IAppMode.h)

[ ] Definition der Basis-Methoden: onEnter, handleInput, onUpdate, onExit.

[ ] 2.2 AppManager erstellen (AppManager.h/.cpp)

[ ] Event-Loop zum Auslesen der xInputQueue auf Core 1 einrichten.

[ ] Feature: Belichtungs-Guard (Verwerfen aller Events außer START_SHORT und START_LONG während EXP_EXPOSING).

[ ] Logik für Moduswechsel (Encoder 4) implementieren.

Phase 3: Ausbaustufe 1 (MVP - Minimum Viable Product)

Ziel: Ein voll nutzbares System für den Standard-Dunkelkammerbetrieb (Prio 1).

[ ] 3.1 SetupApp implementieren (SetupApp.h/.cpp)

[ ] Steuerung der globalen Parameter (globalSet).

[ ] Auswahl des aktiven Papierprofils.

[ ] 3.2 CalibrationApp implementieren (CalibrationApp.h/.cpp)

[ ] Migration des Stouffer-Wizards aus Mode_Calibration.cpp.

[ ] Übergabe der gemessenen Steps an den PaperManager bei Abschluss (mit Statusprüfung: nur im IDLE).

[ ] 3.3 BWApp implementieren (BWApp.h/.cpp)

[ ] Migration der Schwarz-Weiß-Logik.

[ ] Integration der Logic_Measurement für Spot-Messungen.

[ ] Übernahme (Apply) von gepufferten Dosis-Vorschlägen via ENTER.

[ ] 3.4 Das God-Object abreißen (Logic_Timer.cpp bereinigen)

[ ] Löschen der alten processGlobalEvents() und handleInput() switch-case-Blöcke.

[ ] Integration des AppManager in main.cpp (vTaskRealtime).

Phase 4: Ausbaustufe 2 (Erweiterungen)

Ziel: Integration der verbleibenden Spezial-Modi in das neue System.

[ ] 4.1 SGApp implementieren (Splitgrade Automatik & Histogramm).

[ ] 4.2 TestStripApp implementieren (Probestreifen).

[ ] 4.3 BurnApp implementieren (Nachbelichtung).

[ ] 4.4 DensApp implementieren (Densitometer).

[ ] 4.5 Code-Cleanup (Letzte Reste der Globals.h entfernen).