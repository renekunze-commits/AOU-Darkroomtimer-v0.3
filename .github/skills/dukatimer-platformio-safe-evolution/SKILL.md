---
name: dukatimer-platformio-safe-evolution
description: 'Implementiere und erweitere das Dukatimer-System in einer PlatformIO-Umgebung sicher und nachvollziehbar. Verwende diesen Skill bei Dukatimer-Part2-, Teensy-, ESP32-, HAL-, UI-, Timing-, EV-, Dosis- oder Latenz-Aenderungen, wenn bestehende Signaturen, fotografische Berechnungen und Hardware-Abstraktionen erhalten bleiben muessen.'
argument-hint: 'Beschreibe die geplante Dukatimer-Aenderung, das Zielverhalten und die betroffenen Module.'
---

# Dukatimer PlatformIO Safe Evolution

## Zweck

Nutze diesen Skill fuer die fehlerarme Weiterentwicklung des Dukatimer-Systems in einer PlatformIO-Umgebung mit Zugriff auf den aktuellen Code, historische Vorlagen und begleitende Markdown-Dokumentation.

Der Skill priorisiert Bestandsschutz, technische Nachvollziehbarkeit und saubere Abschlusspruefungen.

## Wann dieser Skill passt

- Neue Firmware-Funktionen oder Bugfixes in Dukatimer-Part2
- Aenderungen an Teensy-, ESP32-, UI-, Timing-, Sensor- oder Kommunikationslogik
- Arbeiten an HAL-nahen Pfaden, bei denen keine Bibliotheksmethoden oder Pins geraten werden duerfen
- Aenderungen mit Bezug auf EV-Logik, Dosis-Integration oder Latenz-Kompensation
- Aufgaben, bei denen Versionierung, Changelog und technische Dokumentation synchron bleiben muessen

## Nicht mit diesem Skill arbeiten

- Wenn Informationen zu Pins, Bibliotheken, Messpfaden oder Hardware-Anschluessen fehlen und nicht im sichtbaren Code oder in der Dokumentation belegbar sind
- Wenn eine echte Refaktorierung oder das Entfernen bestehender Logikpfade verlangt ist, ohne dass dies explizit freigegeben wurde
- Wenn die Aufgabe primaer konzeptionell ist und noch keine konkrete Zielaenderung oder kein konkreter Codepfad benannt werden kann

## Eingaben vor dem Start

Vor der ersten Aenderung muessen folgende Punkte geklaert oder aus Dateien belegt sein:

- Gewuenschtes Zielverhalten oder konkreter Fehler
- Betroffene Umgebung in `platformio.ini` und relevante Build-Ziele
- Betroffene Dateien, Symbole oder der naechste direkte Entscheidungspfad im Code
- Relevante Referenzen aus vorhandenen `.md`-Dokumenten oder historischen Vorlagen
- Ob eine Refaktorierung ausdruecklich erlaubt ist

Wenn ein Punkt nicht belegbar ist, benenne die Luecke offen und fordere die fehlenden Daten an.

## Arbeitsablauf

1. Starte immer am konkretesten Anker.
   Nutze zuerst eine benannte Datei, ein Symbol, einen fehlgeschlagenen Build, einen Test oder einen direkt benachbarten Implementierungspfad. Wenn die Anfrage noch zu grob ist, suche nur so weit, bis ein lokaler Eigentuermer der Logik identifiziert ist.

2. Leite eine lokale, falsifizierbare Arbeitshypothese ab.
   Lies nur den engsten relevanten Code, `platformio.ini` und bei Bedarf die naechste fachliche Referenz. Formuliere genau eine Hypothese dazu, wie das Verhalten aktuell gesteuert wird oder warum es fehlschlaegt, plus genau eine billige Gegenprobe, die diese Hypothese widerlegen koennte.

3. Schuetze Bestand und Schnittstellen.
   Entferne keinen vorhandenen Code und keine etablierten Logikpfade, solange nicht explizit zur Refaktorierung aufgefordert wurde. Bestehende Funktionen muessen in Signatur und Kernfunktion erhalten bleiben. Eingriffe in HAL- oder fotografische Kernlogik sind nur erlaubt, wenn die Aufgabe das zwingend erfordert und die Faktenlage aus Dateien ausreicht.

4. Implementiere die kleinste belastbare Aenderung.
   Arbeite modular und im bestehenden Dukatimer-Stil. Erfinde keine Pins, Register, Sensorgrenzen oder Bibliotheks-APIs. Wenn eine Annahme nicht aus Code, `platformio.ini` oder Dokumentation ableitbar ist, stoppe die Implementation an dieser Stelle und fordere die benoetigte Information an.

5. Kommentiere neue Logik auf Deutsch mit Fokus auf das Warum.
   Jeder neue oder wesentlich geaenderte Logikabschnitt braucht lueckenlose, gut lesbare deutsche Kommentare, die Motivation, Randbedingungen und Schutzgedanken erklaeren. Kommentare duerfen nicht nur die sichtbaren Codezeilen umschreiben.

6. Pruefe mathematische und physikalische Konsistenz.
   Jeder neue Rechenpfad ist gegen EV-Logik, Dosis-Integration und Latenz-Kompensation zu auditieren. Vermeide Integer-Divisionen bei Mittelwerten. Verwende fuer Rundungen `std::lround`, wenn ganzzahlige Ausgabe fachlich erforderlich ist. Achte auf Einheiten, Reihenfolge der Integration und Plausibilitaet von Grenzfaellen.

7. Synchronisiere Versionierung und Dokumentation.
   Erhoehe bei jeder signifikanten Firmware-Aenderung die Version in der passenden `FirmwareVersion.h`. Aktualisiere die naechste relevante Markdown-Dokumentation oder den Changelog im selben Arbeitsgang. Dokumentiere die vorgenommenen Aenderungen strukturiert in einem Abschlussblock oder einer separaten Datei, wenn dies zur Nachvollziehbarkeit beitraegt.

8. Validiere direkt nach der ersten substantiellen Aenderung.
   Fuehre sofort die engste verfuegbare Pruefung fuer den geaenderten Bereich aus. Bevorzuge in dieser Reihenfolge: betroffener Test, enger Build- oder Typecheck fuer das betroffene Slice, dann erst breitere Builds. Fuer PlatformIO-Builds nutze die passenden Workspace-Tasks fuer die betroffenen Umgebungen statt ungezielter Vollbauten.

9. Schliesse mit einem Audit-Fazit ab.
   Halte fest, welche Dateien geaendert wurden, welche Vertrage bewusst erhalten blieben, welche Validierungen gelaufen sind und welche Punkte offen oder unbelegt geblieben sind.

## Entscheidungsregeln

- Wenn der sichtbare Code nur weiterleitet oder registriert, springe genau einen Schritt weiter zur Stelle, die Verhalten wirklich berechnet oder mutiert.
- Wenn mehrere lokale Pfade moeglich sind, waehle den mit der kleinsten testbaren Aenderung und der billigsten Gegenprobe.
- Wenn historische Vorlagen von aktuellem Code abweichen, nutze sie nur als Referenz fuer Absicht und Entwicklungslinie, nicht als Beweis fuer aktuelle Laufzeitlogik.
- Wenn Dokumentation und Code kollidieren, hat der sichtbare aktuelle Code Vorrang. Halte die Abweichung fest und aktualisiere die Doku nur, wenn die beabsichtigte Zielrichtung klar belegt ist.
- Wenn mehrere Board-Umgebungen betroffen sein koennten, validiere mindestens die direkt betroffene Umgebung und nenne ungetestete Umgebungen explizit.
- Wenn kein `FirmwareVersion.h` im betroffenen Slice auffindbar ist, dokumentiere diese Feststellung explizit, statt eine Datei zu erfinden.

## Fertig, wenn alle Punkte erfuellt sind

- Die Aenderung ist auf sichtbare Fakten aus Code, `platformio.ini` und vorhandener Dokumentation gestuetzt.
- Bestehende Signaturen, Kernfunktionen und stabile Logikpfade wurden nicht unbegruendet aufgebrochen.
- Neue oder geaenderte Logik ist modular umgesetzt und auf Deutsch mit dem Warum kommentiert.
- Rechenpfade wurden auf EV-, Dosis- und Latenz-Konsistenz geprueft.
- Relevante Versionierung und Dokumentation wurden synchron nachgezogen oder die Luecke wurde offen benannt.
- Mindestens eine enge technische Validierung wurde nach der Aenderung ausgefuehrt.
- Offene Risiken, fehlende Fakten oder ungetestete Umgebungen wurden explizit benannt.

## Beispielanfragen

- Erweitere die Messlogik im Dukatimer-Part2, ohne die bestehende HAL oder EV-Berechnung zu beschaedigen.
- Implementiere einen Bugfix fuer die Teensy-Timing-Kette und pruefe dabei Latenz-Kompensation und Dokumentation.
- Analysiere einen PlatformIO-Buildfehler im ESP32-Pfad, ohne Pins oder Bibliotheksmethoden zu raten, und halte bestehende Signaturen stabil.