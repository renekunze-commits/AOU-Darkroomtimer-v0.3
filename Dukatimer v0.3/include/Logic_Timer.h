/* Logic_Timer.h - v0.5 Root Cause Edition
   
   Diese Datei deklariert die Kern-Engine des Dukatimers.
   Sie bildet die Brücke zwischen der physikalischen Dosis-Berechnung 
   und der zeitbasierten Belichtungssteuerung.
*/

#ifndef LOGIC_TIMER_H
#define LOGIC_TIMER_H

#include <Arduino.h>
#include "Types.h"

// =============================================================================
// ENGINE INITIALISIERUNG & TASKS
// =============================================================================

/** Initialisiert den FreeRTOS-Task für die Closed-Loop Messung auf Core 1 */
void initClosedLoopTask();

/** Initialisiert den Dosis-Status basierend auf den aktuell eingestellten Zeiten (Boot-Sync) */
void initializeDoseStateFromCurrentTimes();


// =============================================================================
// ASYNCHRONES INPUT-HANDLING (v0.5 Architektur)
// =============================================================================

/** Verarbeitet die Input-Queue auf Core 1 (Taster, Encoder, Hardware-Schalter) */
void handleInput();

/** * Die zentrale "Schutzmembran" API: Ändert die Belichtung um EV-Werte.
 * Entscheidet anhand von hwSwitchDoseMode, ob Zeit oder Dosis geändert wird.
 */
void modifyExposureByEV(double evDelta);

/** Synchronisiert Anzeige-Sekunden und Zieldosis für das UI */
void refreshDisplayVariables();


// =============================================================================
// BELICHTUNGSSTEUERUNG (Lifecycle)
// =============================================================================

/** Startet die Belichtungssequenz (inkl. Pre-Wait Phase) */
void startTimer();

/** Stoppt die Belichtung sofort (inkl. Post-Wait Phase) */
void stopTimer();

/** * Wird zyklisch im TaskRealtime aufgerufen. 
 * Überwacht Abbruchbedingungen (Zeit oder Dosis). 
 */
void handleTimer();

/** Deterministisches Metronom für akustisches Feedback */
void handleExposureMetronome(unsigned long elapsedMs);


// =============================================================================
// MODUL-STEUERUNG & LEGACY SUPPORT
// =============================================================================

/** Führt die Standard-Bedienlogik aus (für Abwärtskompatibilität in Wizards) */
void runStandardTimerLoop(char key);

/** Manuelle Skalierung der Dosis (wird von TestStrip benötigt) */
void scaleDoseBWByEV(double evDelta);
void scaleDoseSoftByEV(double evDelta);
void scaleDoseHardByEV(double evDelta);

/** Setter für absolute Dosen */
void setDoseBW(float dose);
void setDoseSoft(float dose);
void setDoseHard(float dose);

/** Getter für aktuelle Dosen */
double getDoseBW();
double getDoseSoft();
double getDoseHard();

/** Berechnet die Schätzzeit für die UI im Dosis-Modus */
float getDisplaySecondsBW();
float getDisplaySecondsSoft();
float getDisplaySecondsHard();

#endif // LOGIC_TIMER_H