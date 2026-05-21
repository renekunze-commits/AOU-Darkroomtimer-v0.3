/* Logic_Measurement.h - v0.5 Root Cause Edition
   
   Verantwortlichkeit: 
   Ablaufsteuerung für Spot-Messungen und Spektralanalyse (C6 Handgerät).
   Implementiert das Zonen-Histogramm für die visuelle Belichtungskontrolle.
*/

#ifndef LOGIC_MEASUREMENT_H
#define LOGIC_MEASUREMENT_H

#include <Arduino.h>

// --- Session Management ---
void startMeteringSession();
void finalizeMeteringSession();
void undoLastMeasurement();
bool isMeteringActive();

/** * Verarbeitet die Eingaben während einer Mess-Session.
 * Wird vom TaskRealtime gerufen.
 * 
 * PFLICHTENHEFT FIX: Parameter-Reihenfolge an die Implementierung in
 * Logic_Measurement.cpp angeglichen. Die Deklaration hatte eine andere
 * Reihenfolge als die Definition, was zu latentem Fehlverhalten bei
 * künftigen Aufrufen geführt hätte (C++ bindet positional, nicht per Name).
 * VORHER: (bool addSpot, bool saveApply, bool toggleChannel, bool resetAll, bool cancel)
 * NEU:    (bool actionSave, bool actionCancel, bool actionToggle, bool actionReset, bool actionUndo)
 */
void handleMeteringSession(bool actionSave, bool actionCancel, bool actionToggle, bool actionReset, bool actionUndo);

// --- Spectral Handshake (Wireless C6) ---
bool triggerSpectralMeasurement();
bool handleMeasurementStateMachine(uint8_t evt, float luxG0, float luxG5);
void abortMeasurementWithError(const char* line1);

// --- Math & Processing ---
void processSpotMeasurement(float luxG0, float luxG5);
bool isMeasurementActive();

#endif // LOGIC_MEASUREMENT_H