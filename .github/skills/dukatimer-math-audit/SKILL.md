---
name: dukatimer-math-audit
description: 'Fuehre mathematische Audits fuer EV-Logik, Dosisberechnung, Integrationspfade, Splitgrade-Mathematik und Latenz-Kompensation im Dukatimer-System durch. Verwende diesen Skill bei Formel-Aenderungen, Audit-Reviews, Rundungsfragen, Einheitenpruefungen, Mittelwertbildung, Profilmathematik oder Verdacht auf mathematische Drift.'
argument-hint: 'Beschreibe die Formel, den Rechenpfad oder den vermuteten mathematischen Fehler.'
---

# Dukatimer Mathematical Audit

## Zweck

Nutze diesen Skill fuer fachlich strenge Audits von EV-, Dosis- und Integrationslogik im Dukatimer-System.

Der Skill soll mathematische Korrektheit, Einheitenkonsistenz, Rundungsverhalten, Driftfreiheit und die Trennung zwischen fotografischer Logik und technischer Laufzeittelemetrie pruefen.

## Was dieser Skill liefert

- Eine eng gefuehrte Analyse des direkten Rechenpfads
- Eine Liste der verwendeten Groessen, Einheiten, Rundungen und Invarianten
- Konkrete Audit-Befunde mit Risikoabschaetzung
- Falls gewuenscht: die kleinste fachlich begruendete Korrektur statt breiter Refaktorierung

## Wann dieser Skill passt

- EV-Umrechnungen, Belichtungsstufen, Zielwert- oder Korrekturformeln werden geaendert oder erscheinen verdaechtig
- Dosisberechnung, Belichtungszeit-Summen, Akkumulatoren oder Integrationsfenster sollen geprueft werden
- Splitgrade-, Papierprofil- oder Kalibrierungsmathematik wird angepasst
- Mittelwerte, Glattungen, Clamps, Quantisierung oder Rundungen koennen das Ergebnis verfalschen
- Es gibt Hinweise auf numerische Drift nach wiederholten Umrechnungen oder Grade-Roundtrips

## Nicht mit diesem Skill arbeiten

- Fuer Pin-Mapping, Bibliotheks-APIs, Treiberanschluesse oder HAL-Fragen ohne mathemischen Kern
- Fuer reine UI- oder Routing-Themen ohne fachliche Rechenlogik
- Fuer technische Timing-Metriken in us/ms, wenn diese nicht in eine fotografische Korrektur oder Integrationsentscheidung eingehen

## Dukatimer-spezifische Leitplanken

- Historisch liegt fotografische Kompensation oberhalb des Treibers. Raw-RGB-Ausgabe oder Bus-Telemetrie sind nicht automatisch Teil der fotografischen Mathematik.
- Splitgrade-Mathematik kann papierprofilgetrieben sein. Ein gemeinsamer Basistarget-Wert und profilabhaengige Soft/Hard-Anteile duerfen nicht durch wiederholte Float-Normalisierung unbemerkt driften.
- Timing-Telemetrie in us/ms ist von EV-Darstellungen zu trennen. Technische Buslaufzeiten sind nur dann mathematisch relevant, wenn sie bewusst in Latenz-Kompensation oder Abschaltvorlauf eingehen.
- Integer-Divisionen in Mittelwerten sind zu vermeiden. Wenn eine ganzzahlige Ausgabe fachlich notwendig ist, nutze `std::lround` statt stiller Abschneidung.

## Eingaben vor dem Start

Vor der ersten Bewertung muessen folgende Punkte aus Code oder Dokumentation belegbar sein:

- Die konkrete Formel, Funktion oder der Akkumulator, der geprueft werden soll
- Die fachlich erwarteten Invarianten oder Grenzfaelle
- Die Einheiten aller Eingangs- und Ausgangsgroessen
- Der betroffene Board- oder Workflow-Kontext
- Vorhandene Referenzen aus `platformio.ini`, `.md`-Dokumenten oder historischen Vorlagen

Wenn eine Groesse, Einheit oder Referenz nicht sichtbar ist, benenne die Luecke offen und rate nichts hinzu.

## Audit-Ablauf

1. Starte am direkten Rechenpfad.
   Suche die Funktion oder den kleinsten Codeblock, der die fachliche Groesse wirklich berechnet, integriert, normiert oder rundet. Wenn ein sichtbarer Pfad nur weiterleitet, gehe genau einen Schritt weiter zur eigentlichen Mathematik.

2. Erstelle ein Groessen- und Einheitenmodell.
   Liste alle Eingaben, Zwischenwerte, Ausgaben, Defaultwerte und Clamps auf. Notiere fuer jede Groesse die Einheit, den erlaubten Wertebereich und ob die Groesse kontinuierlich, diskret oder bereits quantisiert ist.

3. Formuliere die fachlichen Invarianten.
   Leite fuer den konkreten Pfad pruefbare Aussagen ab, zum Beispiel Monotonie, Erhaltung des Gesamttargets, Nichtnegativitaet, Symmetrie, Idempotenz, Roundtrip-Stabilitaet oder Konstanz ueber eine Null-Aenderung.

4. Pruefe EV-Logik getrennt von technischen Hilfswerten.
   Verifiziere, dass EV-Stufen, Zielwerte und Umrechnungen nur mit den dafuer vorgesehenen fotografischen Groessen arbeiten. Timing-, Bus- oder Treibertelemetrie duerfen nicht versehentlich in fotografische EV-Rechnung einsickern.

5. Pruefe Dosis- und Integrationslogik.
   Untersuche Reihenfolge, Zeitbasis und Summation. Frage immer: Welche Groesse wird ueber welche Zeit integriert, in welcher Einheit liegt der Akkumulator vor und wann wird gerundet oder geklemmt? Suche gezielt nach ms/us-Verwechslungen, impliziten Resets, Doppelzaehlungen und fehlender Flaechen- oder Zeitnormalisierung.

6. Pruefe Rundung, Mittelwertbildung und Drift.
   Suche nach Integer-Division, frueher Quantisierung, stiller Typkonversion, mehrfacher Normalisierung und Float-Roundtrips. Wiederholte Umrechnungsketten duerfen keine fachlich relevante Drift erzeugen, besonders bei Grade-Roundtrips, Profilanteilen und Basisziel-Splits.

7. Vergleiche gegen Dukatimer-Referenzen.
   Nutze vorhandene Dokumentation und historische Vorlagen nur, um Absicht, Einheiten und bekannte mathematische Vertrage zu bestaetigen. Wenn Referenzen voneinander abweichen, hat der sichtbare aktuelle Code Vorrang, aber die Abweichung muss als Audit-Befund festgehalten werden.

8. Korrigiere nur gezielt und nur bei klarer Faktenlage.
   Falls die Aufgabe Codeaenderungen einschliesst, fuehre nur die kleinste fachlich notwendige Korrektur aus. Bestehende Signaturen, HAL-Pfade und nicht betroffene Logik bleiben unveraendert. Ohne klare Faktenlage bleibt das Ergebnis ein Audit-Befund statt einer spekulativen Aenderung.

9. Validiere die Invarianten direkt.
   Nutze die engste verfuegbare Pruefung: vorhandene Tests, kleine Rechenbeispiele, spezialisierte Harnesses oder einen engen Build fuer den betroffenen Slice. Wiederhole die pruefbaren Grenzfaelle, die die urspruengliche Hypothese bestaetigen oder widerlegen koennen.

10. Schliesse mit einem Audit-Bericht ab.
   Dokumentiere geprueften Pfad, Invarianten, numerische Risiken, gefundene Verletzungen, getroffene Korrekturen, verbleibende Unsicherheiten und die ausgefuehrten Validierungen.

## Verbindliches Audit-Ausgabeformat

Jeder Abschluss soll diese Struktur enthalten:

1. Gepruefter Pfad
   Benenne Datei, Funktion, Rechenpfad und fachlichen Zweck.

2. Groessen und Einheiten
   Liste Eingaben, Zwischenwerte, Ausgaben, Einheiten, Wertebereiche und relevante Clamps oder Defaults auf.

3. Invarianten
   Halte die fachlichen Soll-Eigenschaften fest, zum Beispiel Monotonie, Gesamttarget-Erhaltung, Roundtrip-Stabilitaet, Nichtnegativitaet oder Reset-Verhalten.

4. Befunde
   Liste verletzte Invarianten, Driftquellen, Einheitenfehler, Rundungsprobleme, Integrationsfehler oder unbelegte Annahmen auf.

5. Korrektur
   Wenn geaendert wurde, beschreibe die kleinste mathematisch begruendete Korrektur und warum sie die Befunde adressiert. Wenn nichts geaendert wurde, begruende den read-only Abschluss.

6. Validierung
   Nenne die ausgefuehrten Checks, Grenzfaelle, Roundtrips, Tests oder Builds und das daraus folgende Audit-Fazit.

7. Offene Risiken
   Halte fehlende Daten, ungetestete Umgebungen oder fachliche Restunsicherheiten explizit fest.

## Typische Audit-Fragen

- Bleibt das Gesamttarget bei Soft/Hard-Aufteilung erhalten?
- Fuehrt ein Grade-Roundtrip auf denselben fachlichen Zustand zurueck?
- Ist die Reihenfolge von Integration, Glattung, Clamp und Rundung fachlich korrekt?
- Werden technische Laufzeiten nur dort verwendet, wo eine begruendete Latenz-Kompensation vorgesehen ist?
- Ist ein Mittelwert wirklich ein Mittelwert oder nur eine abgeschnittene Ganzzahldivision?
- Verliert ein Akkumulator bei Grenzfaellen Vorzeichen, Praezision oder Einheitenkonsistenz?

## Fertig, wenn alle Punkte erfuellt sind

- Der direkte mathematische Eigentuermer wurde identifiziert.
- Alle relevanten Groessen, Einheiten und Invarianten wurden explizit benannt.
- EV-, Dosis- und Integrationspfade wurden auf Einheiten, Reihenfolge, Rundung und Drift geprueft.
- Technische Telemetrie wurde sauber von fotografischer Mathematik getrennt oder ihre Kopplung wurde fachlich begruendet.
- Jede Korrektur ist minimal, begruendet und auf sichtbare Fakten gestuetzt.
- Mindestens eine enge Validierung fuer den geprueften Pfad wurde ausgefuehrt.
- Offene Unsicherheiten oder fehlende Daten wurden explizit benannt.

## Beispielanfragen

- Pruefe, ob die EV-Umrechnung in diesem Kalibrierungspfad bei wiederholten Grade-Roundtrips driftet.
- Auditier die Dosis-Integration dieses Belichtungsakkumulators auf Einheitenfehler, Reset-Probleme und falsche Rundung.
- Untersuche, ob Soft/Hard-Anteile aus dem Papierprofil das Gesamttarget stabil erhalten.
- Pruefe, ob die Latenz-Kompensation fachlich von der fotografischen EV-Mathematik getrennt bleibt.