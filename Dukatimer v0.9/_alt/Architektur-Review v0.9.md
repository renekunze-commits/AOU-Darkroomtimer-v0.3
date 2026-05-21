Dukatimer v0.901 — Architektur-Review (Senior C++/ESP32 Audit)
Kontext
v0.3 war ein funktional vollständiges aber monolithisches System mit 96 globalen Variablen in Globals.h, einer zentralen handleInput()-Dispatcher-Kette und keinerlei Ownership-Grenzen. v0.9 adressiert diese Kernprobleme durch klassenbasierte Kapselung. Dieses Review prüft das aktuelle Fundament (Phase 1 der Roadmap in Grundlagen.md) akribisch auf Korrektheit, Thread-Sicherheit, Zukunftsfähigkeit und versteckte Landminen.

A. POSITIV-MATRIX — Was gut gelöst ist
#	Bereich	Detail	Bewertung
P1	Architektur-Schnitt	Klare 4-Schichten-Trennung: Types.h → SystemContext → HardwareManager / PaperManager → StorageManager. Dependency-Richtung ist strikt top-down, keine Zirkelbezüge.	★★★★★
P2	Globals eliminiert	Null extern-Variablen. Jeder Zustand lebt hinter einer Klasse mit definiertem Zugriff. Das war das Hauptproblem von v0.3.	★★★★★
P3	PIMPL für NeoPixelBus	NeoPixelBus-Instanz als static-Variable in HardwareManager.cpp — Header bleibt frei von Template-Heavy-Includes. Compile-Zeiten und Abhängigkeiten stark reduziert.	★★★★★
P4	I2C Exposure-Lock	Doppelter Check-Mechanismus: _exposureLockActive prüfung VOR und NACH Mutex-Akquise. Physische Sicherheit (Dunkelkammer) geht vor.	★★★★☆
P5	GPIO State-Caching	Safelight/Focus/Enlarger/NeoPixel werden nur bei Zustandsänderung geschrieben (HardwareManager.cpp). Vermeidet Relay-Chatter und SPI-Bus-Spam.	★★★★★
P6	FreeRTOS Buzzer-Timer	One-Shot xTimer für nicht-blockierenden Ton (HardwareManager.cpp). Kein delay() im Audio-Pfad.	★★★★★
P7	Deferred Writing	StorageManager.process() schreibt erst nach 2.5s Ruhe und NIE während Belichtung. Schützt Flash-Lebensdauer und verhindert EMV-Störungen.	★★★★☆
P8	CRC-Integrität Settings werden mit FNV-1a-Hash gesichert (StorageManager.cpp). Korrupte Daten werden erkannt.	★★★☆☆
P9	NVS-Backup	Aktives Papierprofil wird parallel zu LittleFS auch in NVS gesichert (StorageManager.cpp). Redundanz bei Flash-Korruption.	★★★★☆
P10	constexpr statt #define	Magic Numbers im StorageManager.h sind static constexpr — typensicher, debugger-sichtbar, kein Namespace-Pollution.	★★★★★
P11	D-logH S-Kurve	Kubische Smoothstep-Approximation in PaperManager.cpp für photometrisch korrekte Gradationsmischung. Physikalisch fundiert.	★★★★★
P12	Write-Lock während Belichtung	_isWriteLocked() verhindert Schreibzugriff auf PaperBank während laufender Belichtung. Konsistenzschutz.	★★★★☆
B. KRITISCH-MATRIX — Fehler, Risiken, Landminen
B1. BUGS (Sofort beheben)
#	Datei	Zeile	Problem	Schwere	Detail

B2. DESIGN-SCHWÄCHEN (Vor Phase 2 beheben)
#	Bereich	Problem	Schwere	Detail
-

B3. ARCHITEKTUR-LÜCKEN (Für Phasen 2–4 essentiell)
#	Fehlendes Modul	Impact	Prio
L1	IAppMode Interface	Ohne eine abstrakte Basis-Klasse (onEnter, handleInput, onUpdate, onExit) kann das "OS + Apps"-Konzept nicht realisiert werden. Jeder Mode wird sonst wie in v0.3 direkt im Dispatcher verdrahtet.	P0
L2	AppManager / Event-Router	Kein InputQueue-Consumer, kein Mode-Switching, kein Belichtungs-Guard. loop() ruft nur storage->process() auf (main.cpp). Das Herz des Systems fehlt.	P0
L3	InputManager / HW_Input	Encoder-Polling, Button-Debounce und InputQueue existieren nicht. Ohne Input kein interaktives System.	P0
L4	DisplayManager	Weder Nextion-UART noch LCD-I2C noch irgendeine Ausgabe ist implementiert. UI ist für alle Modi essentiell.	P1
L5	ExposureEngine	Die zeitpräzise Belichtungssteuerung (esp_timer aus v0.3) fehlt. Dies ist der kritischste Echtzeit-Pfad des gesamten Systems.	P1
L6	SensorManager	TSL2591/2561, BMP280, DS18B20 Treiber fehlen. HardwareManager verwaltet nur Aktoren, keine Sensoren (trotz Wire.begin() in init()).	P1
C. OPTIMIERUNGSPOTENZIAL
#	Bereich	Ist-Zustand	Soll-Zustand	Aufwand
O1	RAII Mutex-Guard	Manuelle xSemaphoreTake/Give-Paare in jeder Methode (~30 Stellen). Bei Exception oder Early-Return Gefahr des Mutex-Leaks.	MutexGuard RAII-Klasse: { MutexGuard lock(mutex, timeout); if (!lock) return false; ... } — Mutex wird im Destruktor immer freigegeben.	Klein
O2	Getter-Fehlerbehandlung	SystemContext-Getter geben bei Timeout stillschweigend stale Daten zurück.	bool getExposure(ExposureParams& out) mit Rückgabewert, oder std::optional<ExposureParams> (C++17). Aufrufer kann reagieren.	Klein
O3	Struct packing	SettingsBlob und PaperProfile haben undefiniertes Padding.	__attribute__((packed)) auf alle persistierten Structs. Alternativ: manuelle Serialisierung mit festen Offsets.	Klein
O4	Separate Dirty-Timestamps	Ein globaler _lastChangeTime für zwei unabhängige Datentöpfe.	_settingsChangedAt und _papersChangedAt separat tracken.	Trivial
O5	Const-Correctness	getActiveIndex(), isFixedGradeActive(), getActiveFixedGradeValue() sind nicht const-qualifiziert.	const wo möglich — ermöglicht Compiler-Optimierungen und dokumentiert Intent.	Trivial
O6	PaperBank CRC	Bank wird ohne Integritäts-Check geschrieben/gelesen.	CRC-Feld in PaperBank Struct + Validierung in _writePapersToFlash() / Load. Analoges Muster wie SettingsBlob.	Klein
O7	double → float Prüfung	PaperProfile nutzt double für LUT-Werte (8 Byte × 22 = 176 Bytes pro Profil). ESP32 hat keinen Hardware-FPU für double.	Prüfen ob float-Präzision (7 Dezimalstellen) für die Photometrie ausreicht. Könnte PaperBank-Größe halbieren und Rechenzeit auf dem Xtensa-Core deutlich senken.	Mittel
O8	Static Allocation	Manager-Objekte werden mit new auf dem Heap alloziert.	Statische Allokation (SystemContext context; ohne Pointer) eliminiert Heap-Fragmentierung und nullptr-Risiken. Für Singletons auf Embedded ideal.	Klein
D. TODO-LISTE MIT PRIORISIERUNG
Priorität 0 — Blocking Bugs (SOFORT)
#	Task	Datei(en)	Detail

T2	updateLiveDose() implementieren	SystemContext.h, SystemContext.cpp	(a) liveDose-Feld in WorkflowFlags oder neues Struct ergänzen. (b) In der Funktion _flags.liveDose = currentDose setzen. Ohne dies kann die ExposureEngine kein Live-Feedback liefern.
T3	SystemContext-Getter Race Condition fixen	SystemContext.cpp	Kopie erst INNERHALB des Mutex-Blocks erstellen, ODER bool-Rückgabewert nutzen um Timeout sichtbar zu machen. Pattern: bool getExposure(ExposureParams& out) { if (take) { out = _exp; give; return true; } return false; }
Priorität 1 — Strukturelle Härtung (Vor Phase 2)
#	Task	Detail
T4	MutexGuard RAII-Klasse erstellen	Einmalig schreiben, ~30 manuelle Take/Give-Paare ersetzen. Verhindert Mutex-Leaks bei Early-Returns oder zukünftigen Code-Änderungen.
T5	__attribute__((packed)) auf SettingsBlob, PaperProfile, PaperBank	Garantiert identisches Memory-Layout über Compiler-Versionen hinweg. Alternativ: statische sizeof-Asserts als Sanity-Check.
T6	CRC zu PaperBank hinzufügen	CRC-Feld in Struct, Berechnung in _writePapersToFlash(), Validierung in init(). Muster von SettingsBlob übernehmen.
T7	_isDirty auf std::atomic<bool> umstellen	In PaperManager.h: #include <atomic> und std::atomic<bool> _isDirty{false};.
T8	Separate Dirty-Timestamps in StorageManager	_settingsChangedAt + _papersChangedAt statt eines gemeinsamen _lastChangeTime.
T9	restoreBankFromStorage() Timeout begrenzen	portMAX_DELAY → pdMS_TO_TICKS(500) mit Fehlerlogging bei Timeout.
T10	Versionsmigration für PaperBank	Bei bank.version != SW_VERSION_HEX: Versuch die Daten struct-weise zu migrieren statt factoryReset(). Mindestens: alte Version lesen, Profile-Array kopieren, neue Felder mit Defaults füllen.
Priorität 2 — Architektur-Kern (Phase 2 — OS-Schicht)
#	Task	Detail
T11	IAppMode Interface definieren	Abstrakte Basisklasse mit: virtual void onEnter(), virtual void handleInput(InputEvent), virtual void onUpdate(), virtual void onExit(), virtual const char* name().
T12	AppManager implementieren	Mode-Switching via Encoder 4, InputQueue-Consumer, Belichtungs-Guard (Events während Exposure filtern), Current-Mode-Pointer.
T13	InputManager / HW_Input portieren	Encoder-Polling (4 Rotary + 4 Switches + 4 Buttons), Debounce, xInputQueue Dispatch. Aus v0.3 HW_Input.cpp adaptieren.
T14	ExposureEngine portieren	esp_timer-basierte Präzisionsbelichtung. Integration mit HardwareManager::setExposureLock() und SystemContext::setExposureState().
Priorität 3 — Peripherie-Integration (Phase 2–3)
#	Task	Detail
T15	SensorManager erstellen	TSL2591/2561 Lux-Messung, BMP280/BME280 Temperatur, DS18B20 via OneWire. Muss HardwareManager::takeI2C()/giveI2C() nutzen.
T16	DisplayManager portieren	Nextion-UART Thread-sicherer Shadow-Buffer + LCD I2C. Eigener Task auf Core 0 wie in v0.3.
T17	double vs float Evaluation	Benchmark der D-logH LUT-Berechnung mit float vs double. ESP32 Xtensa hat nur Single-Precision FPU → double ist Software-emuliert (~10x langsamer).
Priorität 4 — Robustheit (Produktionsreife)
#	Task	Detail
T18	Error-Propagation-System	Aktuell werden Mutex-Timeouts stillschweigend ignoriert. Einführen eines Event-Logs oder Error-Callback-Systems, das UX-seitig sichtbar wird.
T19	Flash-Wear-Monitoring	Schreibzähler in NVS. Warnung bei >100k Zyklen (typische LittleFS-Lebensdauer auf ESP32 Flash).
T20	Watchdog-Integration	FreeRTOS Task-Watchdog für Core 0 und Core 1. Erkennt Deadlocks durch Mutex-Ketten.
T21	Statische Manager-Allokation prüfen	new → Stack/BSS-Allokation für die 4 Singleton-Manager. Eliminiert Heap-Fragmentierung und nullptr-Risiken.
E. ZUSAMMENFASSUNG
Gesamtbewertung: Solides Fundament mit handvoll Landminen.

Das v0.9-Skelett löst das Kernproblem von v0.3 (96-Globals-Monolith) korrekt. Die Schichtentrennung, die PIMPL-Kapselung, das Exposure-Lock-Konzept und das Deferred-Writing sind architektonisch sauber und durchdacht.

Die drei sofort zu behebenden Probleme sind:


updateLiveDose() No-Op (T2) — die zentrale Echtzeit-Funktion macht nichts
Getter Race Condition (T3) — stale Daten bei Mutex-Timeout ohne Fehlersignal
Vor dem Eintritt in Phase 2 (AppManager/IAppMode) sollten die Strukturellen Härtungen T4–T10 abgeschlossen sein, da diese die Grundlage für alle darauf aufbauenden Modi bilden. Ein nachträgliches Einführen von RAII-Guards oder Struct-Packing in einem System mit 8+ Apps wird exponentiell aufwändiger.

