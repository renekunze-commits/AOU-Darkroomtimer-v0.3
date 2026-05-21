/* Logic_Math.h - v0.5 Root Cause Edition
  
  Verantwortlichkeit: 
  Zentrale mathematische Transformationen zwischen physikalischen 
  Einheiten (EV, Dosis, Zeit) und spektralen Mischungen (Gradation).
*/

#ifndef LOGIC_MATH_H
#define LOGIC_MATH_H

#include <Arduino.h>

// BETA-FIX: Standard-Zielzone fuer BW-Messungen (Zone V)
// Diese Zone ist der mathematische Ankerpunkt fuer die Belichtungsberechnung
// im BW-Modus. Jeder gemessene Spot wird rechnerisch relativ zu dieser Zone
// verschoben, damit aus der gemessenen Helligkeit eine konsistente Ziel-Dosis
// abgeleitet werden kann.
static constexpr double BW_TARGET_ZONE_DEFAULT = 5.0;

/**
 * Berechnet die getrennten Belichtungszeiten für Grün und Blau
 * basierend auf der Basiszeit, der Ziel-Gradation und dem aktiven Papierprofil.
 * v0.5: Nutzt double für absolute Präzision im Zeit-Modus.
 */
void calculateSplitTimes(double baseTime, double grade, double &timeGreen, double &timeBlue);

/**
 * Berechnet die PWM-Werte für die NeoPixel (pwmValGreen/pwmValBlue) 
 * basierend auf der aktuellen grade_bw.
 * Wird gerufen, wenn sich die Gradation im BW-Modus ändert.
 */
void updateGradeMath();

/**
 * Ermittelt den Index (0-10) in der multival-Matrix basierend auf einem Gradationswert (0.0-5.0).
 */
int gradeIndex(double g);

/**
 * Verschiebt die Gradation im Splitgrade-Modus (Sicherheits-Shift).
 */
void applyGradeShift(int direction);

/**
 * Konvertiert eine Dichte-Differenz (EV) in einen Zeit/Dosis-Multiplikator.
 * Hilfsfunktion für die Root Cause Engine.
 */
double evToFactor(double ev);

/**
 * Empirische Gradationsschätzung aus dem gemessenen Kontrastumfang.
 */
double calculateIdealGrade(double measuredRange);

/**
 * Analysiert das Zonen-Histogramm und leitet Zeit- sowie Gradationsvorschlag ab.
 */
void calculateAutoSG(double &suggestedTime, double &suggestedGrade);

/**
 * BETA-FIX: Berechnet eine vorgeschlagene BW-Dosis aus einer Spotmessung.
 * Der Messpunkt wird ueber einen EV-Shift auf die definierte Zielzone
 * (Standard: Zone V) transformiert und in eine Dosis-Empfehlung rueckgerechnet.
 */
void calculateAutoBW(float measuredLux, double &suggestedDose);

/**
 * Berechnet die Netto-Nachbelichtungszeit relativ zur Basiszeit.
 * Formel: t_burn = t_base * (2^burnEv - 1.0)
 */
double getEffectiveBurnTime();

#endif // LOGIC_MATH_H