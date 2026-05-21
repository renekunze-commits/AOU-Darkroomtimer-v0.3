/* Logic_Papers.h - v0.5 Root Cause Edition
   Zentrale Verwaltung der Papierdatenbank im PSRAM.
*/

#ifndef LOGIC_PAPERS_H
#define LOGIC_PAPERS_H

#include <Arduino.h>
#include "Types.h"

// API Methoden
void initPapers();                    // Beim Start laden
void selectPaper(uint8_t index);      // Papier wechseln (Thread-safe)
PaperProfile& getActivePaper();       // Aktuelles Profil holen
PaperProfile& getPaper(uint8_t index);// Spezifisches Profil holen
uint8_t getActivePaperIndex();        // Index des aktiven Papiers

// Messwerte speichern (für manuelle 11-Stufen Gradations-Kalibrierung)
void setPaperGradeMeasurement(uint8_t gradeIdx, double kSoft, double kHard);

// Automatische Profil-Erstellung (Stouffer Wedge Analyse)
void calculateAndSavePaperProfile(int stepWhiteG0, int stepBlackG0, int stepWhiteG5, int stepBlackG5);

// Speicherverwaltung
void savePapers();
void loadPapers();

// Reset/Defaults
void defaultPapers();

#endif