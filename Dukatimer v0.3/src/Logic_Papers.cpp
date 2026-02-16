/*
  Logic_Papers.cpp - Papier-Profil Verwaltung
  
  Zweck: Ausgelagerte Verwaltung von bis zu 20 Papierprofilen
  Ermöglicht spätere App-Anbindung (WLAN/BT) ohne SettingsObject zu überschreiben

  PSRAM-OPTIMIERUNG:
  paperBankPtr wird in initPSRAMStructures() (main.cpp) im externen PSRAM
  allokiert. Das spart ~2.4 KB internen SRAM, der für zeitkritische
  FreeRTOS-Stacks und I2C-Interrupt-Buffer benötigt wird.
  Das Makro "paperBank" in Globals.h leitet alle Zugriffe transparent um.
*/

#include <Arduino.h>
#include "Globals.h"
#include "Logic_Papers.h"
#include <EEPROM.h>

// =============================================================================
// PSRAM-ALLOKATION
// paperBankPtr wird in main.cpp::initPSRAMStructures() zugewiesen.
// Bis dahin ist der Pointer NULL – kein Code darf vor setup() zugreifen!
// =============================================================================
PaperBank* paperBankPtr = nullptr;

// EEPROM Adresse für PaperBank (nach SettingsObject)
#define ADDR_PAPERS 512

extern void updateGradeMath();
extern void markDirty();

void initPapers() {
    loadPapers();
    // Fallback falls ungültig
    if (paperBank.version == 0xFFFF || paperBank.version == 0) {
        defaultPapers();
        savePapers();
    }
    // Sicherstellen, dass Index gültig ist
    if (paperBank.activeIndex >= 20) paperBank.activeIndex = 0;
}

PaperProfile& getActivePaper() {
    return paperBank.profiles[paperBank.activeIndex];
}

PaperProfile& getPaper(uint8_t index) {
    if (index >= 20) index = 0;
    return paperBank.profiles[index];
}

uint8_t getActivePaperIndex() {
    return paperBank.activeIndex;
}

void selectPaper(uint8_t index) {
    if (index < 20) {
        paperBank.activeIndex = index;
        updateGradeMath(); 
        savePapers();
    }
}

void setPaperGradeMeasurement(uint8_t grade, double kVal) {
    if (grade > 5) return;
    PaperProfile& p = getActivePaper();
    p.gradeK[grade] = kVal;
    p.calibrated = true;
    savePapers();
}

void defaultPapers() {
    paperBank.version = 1;
    paperBank.activeIndex = 0;
    
    // Beispiel: Slot 0 mit Defaults füllen
    PaperProfile def;
    memset(&def, 0, sizeof(PaperProfile));
    strcpy(def.name, "Default");
    def.Ksoft = 3.0;
    def.Khard = 12.0;
    def.Kbw = 4.0;
    def.calibrated = true;
    def.flashEnable = false;
    def.flashLevel = 2;
    def.flashColor = 0;
    def.flashThreshS = 0.0;
    def.flashFactor = 1.0;
    
    // Alle Slots initialisieren
    for(int i=0; i<20; i++) {
        paperBank.profiles[i] = def;
        sprintf(paperBank.profiles[i].name, "Paper %d", i+1);
        // Grade-Array nullen (bedeutet: nutze Standard-Berechnung)
        for(int g=0; g<6; g++) paperBank.profiles[i].gradeK[g] = 0.0;
    }
}

void savePapers() {
    EEPROM.put(ADDR_PAPERS, paperBank);
    EEPROM.commit();
}

void loadPapers() {
    EEPROM.get(ADDR_PAPERS, paperBank);
}
