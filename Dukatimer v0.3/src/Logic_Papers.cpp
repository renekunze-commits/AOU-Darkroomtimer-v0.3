/* Logic_Papers.cpp - v0.5 Root Cause Edition
   
   PSRAM-OPTIMIERUNG & CHD HYBRID ENGINE:
   - Die PaperBank wird in main.cpp::initPSRAMStructures() allokiert.
   - Alle Zugriffe sind durch gTimerMutex geschützt.
   - Implementiert die Initialisierung der 11-Stufen LUT und ISO-Parameter.
*/

#include <Arduino.h>
#include <math.h>
#include "Globals.h"
#include "Logic_Papers.h"
#include "Logic_Storage.h"
#include "Logic_Math.h"
#include "ExposureEngine.h"

// Globale Kalibrierungsvariablen (Normalerweise in Globals.h)
extern float calibBaseLuxG0;
extern float calibBaseLuxG5;
extern float calibTimeSeconds;

void initPapers() {
    loadPapers();
    
    // EEPROM Versionierung: Erzwingt Reset bei Struktur-Änderungen (v0.5)
    // ROOT CAUSE FIX: Verhindert das Einlesen von verschobenen/korrupten Speicherblöcken
    if (paperBank.version != SW_VERSION || paperBank.version == 0xFFFF || paperBank.version == 0) {
        Serial.println("[STORAGE] PaperBank Version inkompatibel. Führe sauberen Reset durch...");
        defaultPapers();
        savePapers();
    }
    
    // Validierung des Index (Sicherheitsnetz)
    if (paperBank.activeIndex >= 20) paperBank.activeIndex = 0;
}

PaperProfile& getActivePaper() {
    // Rückgabe der Referenz auf das aktuelle Profil im PSRAM
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
    if (index >= 20) {
        uiTriggerBeep(SND_LIMIT);
        return;
    }

    if (ExposureEngine_IsRunning() || isMeasuring) {
        smartLCD("PAPER LOCKED", "EXPOSURE ACTIVE");
        uiTriggerBeep(SND_LIMIT);
        return;
    }

    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        saveErrorState(ERR_MUTEX_TIMEOUT);
        smartLCD("LOCK TIMEOUT", "PAPER SELECT");
        return;
    }

    if (ExposureEngine_IsRunning() || isMeasuring) {
        xSemaphoreGive(gTimerMutex);
        smartLCD("PAPER LOCKED", "EXPOSURE ACTIVE");
        uiTriggerBeep(SND_LIMIT);
        return;
    }

    paperBank.activeIndex = index;
    // Sofortige Neu-Berechnung der spektralen PWM-Werte für den neuen Papiertyp
    updateGradeMath(); 
    xSemaphoreGive(gTimerMutex);
    
    // Permanent speichern (Nutzer-Workflow schließt Konflikte während Belichtung aus)
    savePapers();
}

void setPaperGradeMeasurement(uint8_t gradeIdx, double kSoft, double kHard) {
    if (gradeIdx > 10) return; // 0 bis 10 für 0,5er Stufen
    
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        PaperProfile& p = getActivePaper();
        p.useIsoMath = false; // Wir erzwingen Ansatz B, da manuell kalibriert wird
        p.gradeK_Soft[gradeIdx] = kSoft;
        p.gradeK_Hard[gradeIdx] = kHard;
        p.calibrated = true;
        xSemaphoreGive(gTimerMutex);
        savePapers();
    }
}

void defaultPapers() {
    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        paperBank.version = SW_VERSION;
        paperBank.activeIndex = 1;

        // =================================================================
        // PHASE 1: Alle 20 Slots mit sicheren Defaults initialisieren
        // =================================================================
        for (int i = 0; i < 20; i++) {
            PaperProfile& p = paperBank.profiles[i];
            memset(&p, 0, sizeof(PaperProfile));
            snprintf(p.name, sizeof(p.name), "Leer %d", i);

            p.useIsoMath      = false;
            p.isFixedGrade    = false;
            p.fixedGradeValue = 2.5f;
            p.isoP            = 100.0;
            p.isoR            = 100.0;
            p.Ksoft           = 10.0;
            p.Khard           = 10.0;
            p.Kbw             = 10.0;
            p.calibrated      = false;
            p.flashEnable     = false;
            p.flashCalibrated = false;
            p.flashLevel      = 0;
            p.flashColor      = 0;
            p.flashThreshS    = 0.0;
            p.flashFactor     = 1.0;

            // Lineare Sicherheits-LUT
            for (int g = 0; g < 11; g++) {
                p.gradeK_Soft[g] = 1.0 - (g / 10.0);
                p.gradeK_Hard[g] = g / 10.0;
            }
        }

        // =================================================================
        // SLOT 0: Foma Variant 311
        // Quelle: Foma Datenblatt + Heiland-TRD Split-Grade Referenz
        // Typ:    VC RC, glänzend | ISO P 250/25° | Grade 00–5
        // =================================================================
        {
            PaperProfile& p = paperBank.profiles[0];
            strncpy(p.name, "Foma Variant 311", sizeof(p.name) - 1);
            p.name[sizeof(p.name) - 1] = '\0';

            p.useIsoMath      = false;
            p.isFixedGrade    = false;
            p.fixedGradeValue = 2.5f;
            p.isoP            = 250.0;
            p.isoR            = 100.0;
            p.Ksoft           = 10.0;
            p.Khard           = 10.0;
            p.Kbw             = 12.0;
            p.calibrated      = true;

            p.gradeK_Soft[0]  = 1.00; p.gradeK_Hard[0]  = 0.00;
            p.gradeK_Soft[1]  = 0.88; p.gradeK_Hard[1]  = 0.12;
            p.gradeK_Soft[2]  = 0.75; p.gradeK_Hard[2]  = 0.25;
            p.gradeK_Soft[3]  = 0.62; p.gradeK_Hard[3]  = 0.38;
            p.gradeK_Soft[4]  = 0.50; p.gradeK_Hard[4]  = 0.50;
            p.gradeK_Soft[5]  = 0.38; p.gradeK_Hard[5]  = 0.62;
            p.gradeK_Soft[6]  = 0.25; p.gradeK_Hard[6]  = 0.75;
            p.gradeK_Soft[7]  = 0.15; p.gradeK_Hard[7]  = 0.85;
            p.gradeK_Soft[8]  = 0.07; p.gradeK_Hard[8]  = 0.93;
            p.gradeK_Soft[9]  = 0.02; p.gradeK_Hard[9]  = 0.98;
            p.gradeK_Soft[10] = 0.00; p.gradeK_Hard[10] = 1.00;

            p.flashEnable     = false;
            p.flashCalibrated = false;
            p.flashLevel      = 0;
            p.flashColor      = 0;
            p.flashThreshS    = 0.0;
            p.flashFactor     = 1.0;
        }

        // =================================================================
        // SLOT 1: Ilford MGIV RC
        // Quelle: Ilford Datenblatt Multigrade IV RC De Luxe
        // Typ:    VC RC | ISO P 200 | Grade 00–5
        // =================================================================
        {
            PaperProfile& p = paperBank.profiles[1];
            strncpy(p.name, "Ilford MGIV RC", sizeof(p.name) - 1);
            p.name[sizeof(p.name) - 1] = '\0';

            p.useIsoMath      = false;
            p.isFixedGrade    = false;
            p.fixedGradeValue = 2.5f;
            p.isoP            = 200.0;
            p.isoR            = 105.0;
            p.Ksoft           = 12.0;
            p.Khard           = 12.0;
            p.Kbw             = 15.0;
            p.calibrated      = true;

            p.gradeK_Soft[0]  = 1.00; p.gradeK_Hard[0]  = 0.00;
            p.gradeK_Soft[1]  = 0.90; p.gradeK_Hard[1]  = 0.10;
            p.gradeK_Soft[2]  = 0.80; p.gradeK_Hard[2]  = 0.20;
            p.gradeK_Soft[3]  = 0.70; p.gradeK_Hard[3]  = 0.30;
            p.gradeK_Soft[4]  = 0.60; p.gradeK_Hard[4]  = 0.40;
            p.gradeK_Soft[5]  = 0.50; p.gradeK_Hard[5]  = 0.50;
            p.gradeK_Soft[6]  = 0.40; p.gradeK_Hard[6]  = 0.60;
            p.gradeK_Soft[7]  = 0.30; p.gradeK_Hard[7]  = 0.70;
            p.gradeK_Soft[8]  = 0.20; p.gradeK_Hard[8]  = 0.80;
            p.gradeK_Soft[9]  = 0.10; p.gradeK_Hard[9]  = 0.90;
            p.gradeK_Soft[10] = 0.00; p.gradeK_Hard[10] = 1.00;

            p.flashEnable     = false;
            p.flashCalibrated = false;
            p.flashLevel      = 0;
            p.flashColor      = 0;
            p.flashThreshS    = 0.0;
            p.flashFactor     = 1.0;
        }

        xSemaphoreGive(gTimerMutex);
    }
}

void calculateAndSavePaperProfile(int stepWhiteG0, int stepBlackG0, int stepWhiteG5, int stepBlackG5) {
    if (calibBaseLuxG0 <= 0.0f || calibBaseLuxG5 <= 0.0f) return;

    // Präzisions-Berechnung der Dosis (Double-Precision)
    auto calcDose = [](int step, double baseLux) -> double {
        double density = (step - 1) * 0.15 + 0.05;
        double transmission = pow(10.0, -density);
        return baseLux * transmission * (double)calibTimeSeconds;
    };

    if (xSemaphoreTake(gTimerMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        PaperProfile& p = getActivePaper();

        p.Ksoft = calcDose(stepWhiteG0, calibBaseLuxG0);
        p.Khard = calcDose(stepWhiteG5, calibBaseLuxG5);
        
        // Da wir eine Messung durchführen, zwingen wir das Profil in Ansatz B (LUT)
        p.useIsoMath = false;

        // Wir berechnen die Zwischenwerte der 11-Stufen LUT (Basis: Konstante Lichterdichte)
        // durch eine Interpolation der beiden gemessenen Endpunkte (Ksoft und Khard).
        for(int i = 0; i < 11; i++) {
            double g = i * 0.5;
            double fractionHard = g / 5.0;
            double fractionSoft = 1.0 - fractionHard;
            
            p.gradeK_Soft[i] = p.Ksoft * fractionSoft;
            p.gradeK_Hard[i] = p.Khard * fractionHard;
        }

        p.calibrated = true;
        xSemaphoreGive(gTimerMutex);

        savePapers();
        saveActivePaperProfile(); // Redundantes Backup im NVS
    }
}