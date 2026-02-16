#ifndef LOGIC_PAPERS_H
#define LOGIC_PAPERS_H

#include "Types.h"

// Zugriff auf die globale Papierbank (PSRAM-Pointer, siehe Globals.h)
// Das Makro "paperBank" wird in Globals.h definiert und leitet auf (*paperBankPtr).
extern PaperBank* paperBankPtr;

// API Methoden
void initPapers();                    // Beim Start laden
void selectPaper(uint8_t index);      // Papier wechseln
PaperProfile& getActivePaper();       // Aktuelles Profil holen
PaperProfile& getPaper(uint8_t index);// Spezifisches Profil holen
uint8_t getActivePaperIndex();        // Index des aktiven Papiers

// Messwerte speichern (fuer Gradations-Kalibrierung)
void setPaperGradeMeasurement(uint8_t grade, double luxSeconds);

// Speicherverwaltung
void savePapers();
void loadPapers();

// Reset/Defaults
void defaultPapers();

#endif
