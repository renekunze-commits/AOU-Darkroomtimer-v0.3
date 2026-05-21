---
description: "Use when working on Dukatimer v0.9 embedded C++ architecture, concurrency, persistence, migration safety, and performance-critical ESP32-S3 FreeRTOS code. Triggers: AppManager, SystemContext, HardwareManager, ExposureEngine, Wire/Wire1, MutexGuard, StorageManager, PaperManager, float optimization."
name: "Dukatimer Embedded Architecture Conventions"
applyTo:
  - "include/**/*.h"
  - "src/**/*.cpp"
---
# Dukatimer v0.9 Coding Conventions

Diese Regeln sind verbindlich fuer die Weiterentwicklung von Dukatimer v0.9.
Ziel ist ein stabiles, modulares Kernel-App-System als Nachfolger des v0.3 Prototyps.

## 1. Architektur und Thread-Modell (Concurrency)

- Dual-Core Trennung ist strikt einzuhalten:
  - Core 0: UI (DisplayManager), Eingabe (InputManager), Persistenz (StorageManager).
  - Core 1: Exklusiv fuer zeitkritische Berechnungen (ExposureEngine).
- FreeRTOS-Mutexe sind non-recursive. Daher wird konsequent RAII via MutexGuard verwendet.
- Eine Methode darf intern keine andere oeffentliche Methode derselben Klasse aufrufen, wenn beide denselben Mutex beanspruchen (Deadlock-Gefahr).
- Lock-Free Realtime auf Core 1: I2C-1 (Wire1) wird ohne Mutex betrieben, um Jitter zu vermeiden.
- Core 0 nutzt I2C-0 (Wire) mit explizitem Locking ueber HardwareManager.

## 2. Speicher-Integritaet und Persistenz

- Hash-Payload-Trennung ist Pflicht:
  - Provider (SystemContext, PaperManager) serialisieren niemals den 16-Bit-Hash als Teil der Payload.
  - Nur StorageManager haengt den Hash an und trennt ihn wieder ab.
- OOB-Schutz beim Deserialisieren:
  - Pufferlaenge strikt gegen offsetof(Struct, hash) pruefen.
  - memcpy niemals mit sizeof(Struct) auf unbekannte externe Buffer ausfuehren.
  - memcpy nur mit verifizierter Payload-Laenge ausfuehren.
- Datenmigrationen von Legacy-Strukturen erfolgen immer Field-by-Field mit expliziten Casts.
- Binaere memcpy-Migrationen zwischen Versionsstaenden sind verboten.

## 3. FPU-Optimierung (Performance)

- Single-Precision only: Es werden strikt 32-Bit float-Werte verwendet.
- Alle Float-Konstanten tragen ein f-Suffix (Beispiel: 12.0f).
- Nutze float-Funktionsvarianten:
  - powf() statt pow()
  - exp2f() statt exp2()
  - sqrtf() statt sqrt()
- Double-Pfade sind zu vermeiden, da sie auf ESP32-S3 software-emuliert und deutlich langsamer sind.

## 4. Initialisierung und Typ-Sicherheit

- Bulletproof Init:
  - Keine positionsbasierten Sammel-Initialisierungen fuer komplexe Statusstrukturen.
  - Defaults werden im Header definiert.
  - Im Konstruktor erfolgen explizite, namentliche Zuweisungen (Beispiel: _hw.liveTime = 0.0f;).
- Flags, die zwischen Tasks oder Cores geteilt werden, werden als std::atomic<bool> deklariert.

## 5. Coding-Style und Dokumentation

- Klammersetzung folgt Allman-Style (oeffnende Klammer in neuer Zeile).
- Jede Datei enthaelt einen ausfuehrlichen Header-Kommentar inklusive Versionshistorie.
- Kritische Abschnitte (insbesondere Core-1-to-Core-0 Bridge) werden ausfuehrlich auf Deutsch kommentiert.
- Dependency Injection ist verbindlich:
  - Keine globalen Variablen.
  - Instanzen werden in main.cpp erzeugt und per Pointer in Manager/App-Klassen injiziert.

## Umsetzungscheckliste fuer neue Aenderungen

- Ist die Core-0/Core-1-Zustaendigkeit eingehalten?
- Gibt es einen MutexGuard statt manueller Take/Give-Paare?
- Sind Hash und Payload strikt getrennt?
- Werden nur float + f-Suffix + *f()-Math APIs verwendet?
- Ist Initialisierung explizit und typsicher?
- Ist Allman-Style und deutschsprachige Doku in kritischen Pfaden vorhanden?
- Ist Dependency Injection ohne Globals umgesetzt?
