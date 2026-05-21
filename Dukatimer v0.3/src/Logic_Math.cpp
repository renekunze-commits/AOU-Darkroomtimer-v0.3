/* Logic_Math.cpp - v0.5 Root Cause Edition
   
   Verantwortlichkeit: 
   Präzisions-Mathematik für die Belichtungstransformation (Constant Highlight Density). 
   Implementiert das Hybrid-Modell: Ansatz B (11-Stufen LUT) & Ansatz C (ISO-Datenblatt).
*/

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "Globals.h"
#include "Types.h"
#include "Logic_Math.h"
#include "Logic_Papers.h"

// =============================================================================
// HILFSFUNKTIONEN (Intern)
// =============================================================================

static constexpr TickType_t MATH_MUTEX_TIMEOUT = pdMS_TO_TICKS(100);
static constexpr double SAFE_DEFAULT_TIME_S = 10.0;
static constexpr double SAFE_DEFAULT_GRADE = 2.5;
static constexpr double SAFE_MIN_DIVISOR = 0.000001;

static double clampDouble(double val, double minV, double maxV) {
    if (val < minV) return minV;
    if (val > maxV) return maxV;
    return val;
}

static bool isFiniteNumber(double value) {
    return !isnan(value) && !isinf(value);
}

static void reportMathFailure(const char* context) {
    saveErrorState(ERR_MATH_INVALID);
    Serial.printf("[MATH] Ungueltige Eingabedaten in %s. Nutze Safe-Fallback.\n", context);
}

static void reportMutexFailure(const char* context) {
    saveErrorState(ERR_MUTEX_TIMEOUT);
    Serial.printf("[MATH] Mutex-Timeout in %s. Operation abgebrochen.\n", context);
}

static void setSafeSuggestion(double &suggestedTime, double &suggestedGrade) {
    suggestedTime = SAFE_DEFAULT_TIME_S;
    suggestedGrade = SAFE_DEFAULT_GRADE;
}

static bool timerMutexOwnedByCurrentTask() {
    if (gTimerMutex == NULL) return false;
    return xSemaphoreGetMutexHolder(gTimerMutex) == xTaskGetCurrentTaskHandle();
}

static bool lockTimerMutexIfNeeded(TickType_t timeout, bool &lockedHere, const char* context) {
    lockedHere = false;
    if (gTimerMutex == NULL) return true;
    if (timerMutexOwnedByCurrentTask()) return true;

    lockedHere = (xSemaphoreTake(gTimerMutex, timeout) == pdTRUE);
    if (!lockedHere) {
        reportMutexFailure(context);
    }
    return lockedHere;
}

static void unlockTimerMutexIfNeeded(bool lockedHere) {
    if (lockedHere && gTimerMutex != NULL) {
        xSemaphoreGive(gTimerMutex);
    }
}

static double sanitizeFinite(double value, double fallback, const char* context) {
    if (!isFiniteNumber(value)) {
        reportMathFailure(context);
        return fallback;
    }
    return value;
}

// =============================================================================
// VERTRAGS-IMPLEMENTIERUNG (Logic_Math.h)
// =============================================================================

double evToFactor(double ev) {
    ev = sanitizeFinite(ev, 0.0, "evToFactor.ev");

    double factor = pow(2.0, ev);
    if (!isFiniteNumber(factor) || factor <= 0.0) {
        reportMathFailure("evToFactor.pow");
        return 1.0;
    }
    return factor;
}

double calculateIdealGrade(double measuredRange) {
    measuredRange = sanitizeFinite(measuredRange, 1.5, "calculateIdealGrade.range");

    double grade = 5.0 * (1.5 - measuredRange) / 0.9;
    if (!isFiniteNumber(grade)) {
        reportMathFailure("calculateIdealGrade.result");
        return SAFE_DEFAULT_GRADE;
    }
    return clampDouble(grade, 0.0, 5.0);
}

int gradeIndex(double g) {
    g = sanitizeFinite(g, SAFE_DEFAULT_GRADE, "gradeIndex.grade");
    // Wandelt 0.0-5.0 in Index 0-10 um (0,5er Schritte)
    // +0.1 als Schutz vor Float-Rundungsfehlern (z.B. 2.4999 * 2 = 4 statt 5)
    int idx = (int)((clampDouble(g, 0.0, 5.0) * 2.0) + 0.1); 
    if (idx < 0) idx = 0;
    if (idx > 10) idx = 10;
    return idx;
}

void updateGradeMath() {
    // Wandelt die Gradation (0.0 - 5.0) in PWM-Werte für den BW-Modus um.
    // v0.5: Nutzt direkt die CHD-Logik des aktuellen Papierprofils!

    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "updateGradeMath")) {
        return;
    }

    double g = sanitizeFinite((double)grade_bw, SAFE_DEFAULT_GRADE, "updateGradeMath.grade_bw");
    PaperProfile paper = getActivePaper();

    if (paper.isFixedGrade) {
        pwmValGreen = 255;
        pwmValBlue = 255;
        unlockTimerMutexIfNeeded(lockedHere);
        return;
    }
    
    double kSoft = 0.0;
    double kHard = 0.0;

    if (paper.useIsoMath) {
        // --- ANSATZ C (ISO) für PWM ---
        // Lineare Interpolation als Baseline für die Farbmischung
        kHard = clampDouble(g, 0.0, 5.0) / 5.0;
        kSoft = 1.0 - kHard;
    } else {
        // --- ANSATZ B (LUT) für PWM ---
        int idx = gradeIndex(g);
        kSoft = sanitizeFinite(paper.gradeK_Soft[idx], 0.0, "updateGradeMath.gradeK_Soft");
        kHard = sanitizeFinite(paper.gradeK_Hard[idx], 0.0, "updateGradeMath.gradeK_Hard");
        
        // Fallback-Schutz für unkalibrierte oder korrupte Papiere
        if (kSoft <= 0.0001 && kHard <= 0.0001) {
            kSoft = 1.0 - (clampDouble(g, 0.0, 5.0) / 5.0);
            kHard = clampDouble(g, 0.0, 5.0) / 5.0;
        }
    }

    if (!isFiniteNumber(kSoft) || !isFiniteNumber(kHard)) {
        reportMathFailure("updateGradeMath.mix");
        pwmValGreen = 255;
        pwmValBlue = 255;
        unlockTimerMutexIfNeeded(lockedHere);
        return;
    }

    // PWM Normalisierung: 
    // Wir wollen maximales Licht im BW-Modus. Der stärkere Kanal wird auf 255 gesetzt,
    // der schwächere exakt proportional gedimmt. Das erhält das spektrale Verhältnis.
    double maxK = fmax(kSoft, kHard);
    if (!isFiniteNumber(maxK) || maxK < SAFE_MIN_DIVISOR) {
        reportMathFailure("updateGradeMath.maxK");
        maxK = 1.0;
    }

    double greenRatio = (kSoft / maxK) * 255.0;
    double blueRatio = (kHard / maxK) * 255.0;

    if (!isFiniteNumber(greenRatio) || !isFiniteNumber(blueRatio)) {
        reportMathFailure("updateGradeMath.ratio");
        pwmValGreen = 255;
        pwmValBlue = 255;
    } else {
        pwmValGreen = (uint8_t)clampDouble(greenRatio, 0.0, 255.0);
        pwmValBlue  = (uint8_t)clampDouble(blueRatio, 0.0, 255.0);
    }

    unlockTimerMutexIfNeeded(lockedHere);
}

void calculateSplitTimes(double baseTime, double grade, double &timeGreen, double &timeBlue) {
    baseTime = sanitizeFinite(baseTime, SAFE_DEFAULT_TIME_S, "calculateSplitTimes.baseTime");
    grade = sanitizeFinite(grade, SAFE_DEFAULT_GRADE, "calculateSplitTimes.grade");
    grade = clampDouble(grade, 0.0, 5.0);

    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "calculateSplitTimes")) {
        timeGreen = SAFE_DEFAULT_TIME_S;
        timeBlue = 0.0;
        return;
    }

    PaperProfile paper = getActivePaper();

    if (paper.isFixedGrade) {
        timeGreen = clampDouble(baseTime, TIME_MIN_S, TIME_MAX_S);
        timeBlue = 0.0;
        unlockTimerMutexIfNeeded(lockedHere);
        return;
    }

    if (paper.useIsoMath) {
        // =========================================================================
        // ANSATZ C: ISO DATENBLATT MATHEMATIK (Heiland / RH Designs Prinzip)
        // =========================================================================
        double speedP = sanitizeFinite(paper.isoP, 100.0, "calculateSplitTimes.isoP");
        if (speedP <= 10.0) {
            reportMathFailure("calculateSplitTimes.isoP.range");
            speedP = 100.0;
        }

        double chdFactor = 100.0 / speedP;
        double chdBaseTime = baseTime * chdFactor;

        double fractionHard = grade / 5.0;
        double fractionSoft = 1.0 - fractionHard;

        timeGreen = chdBaseTime * fractionSoft;
        timeBlue  = chdBaseTime * fractionHard;
    } else {
        // =========================================================================
        // ANSATZ B: DIREKTE 11-STUFEN LUT (Kalibriertes Papier)
        // =========================================================================
        int idx = gradeIndex(grade);
        
        double kSoft = sanitizeFinite(paper.gradeK_Soft[idx], 0.0, "calculateSplitTimes.gradeK_Soft");
        double kHard = sanitizeFinite(paper.gradeK_Hard[idx], 0.0, "calculateSplitTimes.gradeK_Hard");
        
        // Fallback, falls das Array an dieser Stelle 0 ist (z.B. nach Reset)
        if (kSoft <= 0.0001 && kHard <= 0.0001) {
            kSoft = 1.0 - (grade / 5.0);
            kHard = grade / 5.0;
        }

        timeGreen = baseTime * kSoft;
        timeBlue  = baseTime * kHard;
    }

    if (!isFiniteNumber(timeGreen) || !isFiniteNumber(timeBlue)) {
        reportMathFailure("calculateSplitTimes.result");
        timeGreen = SAFE_DEFAULT_TIME_S;
        timeBlue = 0.0;
    } else {
        timeGreen = clampDouble(timeGreen, 0.0, TIME_MAX_S);
        timeBlue = clampDouble(timeBlue, 0.0, TIME_MAX_S);
    }

    unlockTimerMutexIfNeeded(lockedHere);
}

void applyGradeShift(int direction) {
    // v0.5 Update: Gradation in fixen 0,5er Schritten!
    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "applyGradeShift")) {
        return;
    }

    double current = sanitizeFinite((double)grade_bw, SAFE_DEFAULT_GRADE, "applyGradeShift.current");
    bool hitLimit = false;
    current = round(current * 2.0) / 2.0;

    double next = current + (direction * 0.5);
    if (!isFiniteNumber(next)) {
        reportMathFailure("applyGradeShift.next");
        next = SAFE_DEFAULT_GRADE;
    }
    if (next < 0.0 || next > 5.0) {
        hitLimit = true;
    }
    grade_bw = (float)clampDouble(next, 0.0, 5.0);
    updateGradeMath();
    unlockTimerMutexIfNeeded(lockedHere);

    if (hitLimit) uiTriggerBeep(SND_LIMIT);
}

// =============================================================================
// SYSTEM-LOGIK (Status & Validierung)
// =============================================================================

void validateTimes() {
    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "validateTimes")) {
        return;
    }

    time_bw   = (float)clampDouble(sanitizeFinite((double)time_bw, SAFE_DEFAULT_TIME_S, "validateTimes.time_bw"), TIME_MIN_S, TIME_MAX_S);
    time_soft = (float)clampDouble(sanitizeFinite((double)time_soft, SAFE_DEFAULT_TIME_S, "validateTimes.time_soft"), TIME_MIN_S, TIME_MAX_S);
    // M05 Fix: time_hard darf im BW-Modus 0.0 sein – nur im SG-Modus clampen
    if (currentMode == MODE_SG) {
        time_hard = (float)clampDouble(sanitizeFinite((double)time_hard, SAFE_DEFAULT_TIME_S, "validateTimes.time_hard"), TIME_MIN_S, TIME_MAX_S);
    }
    grade_bw  = (float)clampDouble(sanitizeFinite((double)grade_bw, SAFE_DEFAULT_GRADE, "validateTimes.grade_bw"), 0.0, 5.0);

    unlockTimerMutexIfNeeded(lockedHere);
}

void resetTracking() {
    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "resetTracking")) {
        return;
    }

    ts = TS_OFF;
    tsK = 0;
    tsSum = 0.0;
    tsActiveExposure = false;

    unlockTimerMutexIfNeeded(lockedHere);
}

double getEffectiveBurnTime() {
    double base = sanitizeFinite((double)time_bw, SAFE_DEFAULT_TIME_S, "getEffectiveBurnTime.base");
    double ev = sanitizeFinite((double)burnEv, 0.0, "getEffectiveBurnTime.ev");
    double factor = pow(2.0, ev);

    if (!isFiniteNumber(factor)) {
        reportMathFailure("getEffectiveBurnTime.pow");
        return SAFE_DEFAULT_TIME_S;
    }

    double burnTime = base * (factor - 1.0);
    if (!isFiniteNumber(burnTime)) {
        reportMathFailure("getEffectiveBurnTime.result");
        return SAFE_DEFAULT_TIME_S;
    }
    return burnTime;
}

long secondsToUnits(double s, int tgl) {
    s = sanitizeFinite(s, SAFE_DEFAULT_TIME_S, "secondsToUnits.seconds");
    if (tgl == 0) return (long)(s * 10.0);
    if (s <= 0.1) return 0;

    double units = log2(s) * 10.0;
    if (!isFiniteNumber(units)) {
        reportMathFailure("secondsToUnits.log2");
        units = log2(SAFE_DEFAULT_TIME_S) * 10.0;
    }
    return (long)units;
}

void calculateAutoSG(double &suggestedTime, double &suggestedGrade) {
    uint8_t histogramSnapshot[11] = {0};
    double baseTime = 0.0;
    double currentGrade = SAFE_DEFAULT_GRADE;

    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "calculateAutoSG.read")) {
        setSafeSuggestion(suggestedTime, suggestedGrade);
        return;
    }

    memcpy(histogramSnapshot, currentZoneHistogram, sizeof(histogramSnapshot));
    baseTime = sanitizeFinite((double)timer_base_seconds, SAFE_DEFAULT_TIME_S, "calculateAutoSG.timer_base_seconds");
    currentGrade = sanitizeFinite((double)grade_bw, SAFE_DEFAULT_GRADE, "calculateAutoSG.grade_bw");
    unlockTimerMutexIfNeeded(lockedHere);

    int zoneLow = -1;
    int zoneHigh = -1;
    int pointCount = 0;

    for (int i = 0; i < 11; i++) {
        uint8_t bucket = histogramSnapshot[i];
        if (bucket > 0) {
            if (zoneLow < 0) zoneLow = i;
            zoneHigh = i;
        }
        pointCount += ((int)bucket / 20);
    }

    if (zoneLow < 0 || zoneHigh < 0 || pointCount < 2) {
        suggestedTime = 0.0;
        suggestedGrade = currentGrade;
        return;
    }

    const double evRange = (double)(zoneHigh - zoneLow);
    const double measuredRange = evRange / 3.32;
    if (!isFiniteNumber(measuredRange)) {
        reportMathFailure("calculateAutoSG.measuredRange");
        setSafeSuggestion(suggestedTime, suggestedGrade);
        return;
    }

    const double bestGrade = calculateIdealGrade(measuredRange);

    if (!isFiniteNumber(baseTime) || baseTime <= 0.0) {
        double soft = sanitizeFinite((double)time_soft, SAFE_DEFAULT_TIME_S, "calculateAutoSG.time_soft");
        double hard = sanitizeFinite((double)time_hard, SAFE_DEFAULT_TIME_S, "calculateAutoSG.time_hard");
        baseTime = (soft + hard) * 0.5;
    }
    if (baseTime < TIME_MIN_S) baseTime = TIME_MIN_S;

    // CHD-Anker: Lichter auf Zone VIII legen
    const double chdAnchorZone = 8.0;
    const double anchorShiftEV = chdAnchorZone - (double)zoneLow;
    double t = baseTime * evToFactor(anchorShiftEV);
    if (!isFiniteNumber(t)) {
        reportMathFailure("calculateAutoSG.time");
        setSafeSuggestion(suggestedTime, suggestedGrade);
        return;
    }
    t = clampDouble(t, TIME_MIN_S, TIME_MAX_S);

    suggestedTime = t;
    suggestedGrade = clampDouble(bestGrade, 0.0, 5.0);

    lockedHere = false;
    if (lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "calculateAutoSG.write")) {
        trackDensityEV = measuredRange;
        trackGradeSteps = suggestedGrade;
        unlockTimerMutexIfNeeded(lockedHere);
    }
}

void calculateAutoBW(float measuredLux, double &suggestedDose) {
    // BETA-FIX: Diese Funktion koppelt Spotmessung -> BW-Zieldosis.
    // BETA-FIX: [Phase 2] Mathematischer Anker ist jetzt dynamisch
    // ueber globalSet.bwTargetZone konfigurierbar (Setup-Menue).

    double lux = sanitizeFinite((double)measuredLux, 0.0, "calculateAutoBW.measuredLux");
    if (lux <= 0.01) {
        reportMathFailure("calculateAutoBW.lux.range");
        // BETA-FIX: Bei ungueltigem Lux niemals aggressiv hochregeln.
        suggestedDose = sanitizeFinite((double)target_dose, SAFE_DEFAULT_TIME_S, "calculateAutoBW.target_dose_fallback");
        return;
    }

    bool lockedHere = false;
    if (!lockTimerMutexIfNeeded(MATH_MUTEX_TIMEOUT, lockedHere, "calculateAutoBW.read")) {
        suggestedDose = sanitizeFinite((double)target_dose, SAFE_DEFAULT_TIME_S, "calculateAutoBW.mutex_fallback");
        return;
    }

    // BETA-FIX: Aktives Papierprofil als notwendiger Kontext fuer BW-Kopplung.
    // Hier wird nur auf existierende Felder zugegriffen; keine erfundenen Konstanten.
    PaperProfile paper = getActivePaper();
    double baseDose = sanitizeFinite((double)dose_bw, SAFE_DEFAULT_TIME_S, "calculateAutoBW.dose_bw");

    // BETA-FIX: [Phase 2] Zielzone thread-sicher aus globalSet uebernehmen.
    // Sicherheitskorridor: 4.0..9.0, bei Speicherfehlern Fallback auf 8.0.
    double targetZone = 8.0;
    double configuredTargetZone = sanitizeFinite((double)globalSet.bwTargetZone, 8.0, "calculateAutoBW.globalSet.bwTargetZone");
    if (configuredTargetZone >= 4.0 && configuredTargetZone <= 9.0) {
        targetZone = configuredTargetZone;
    }

    // BETA-FIX: Papierprofil als Dosis-Anker nutzen, wenn kalibriert vorhanden.
    // Kbw repraesentiert die BW-Grunddosis des aktiven Papiers und ist damit der
    // naheliegende profilabhaengige Startwert fuer die Rueckrechnung.
    if (paper.Kbw > 0.0001) {
        baseDose = paper.Kbw;
    }
    if (baseDose <= 0.0) {
        baseDose = SAFE_DEFAULT_TIME_S;
    }
    unlockTimerMutexIfNeeded(lockedHere);

    // BETA-FIX: Logarithmierung (Lux -> EV) mit Basis 2.
    // Dadurch wird jede Verdopplung/Halbierung der Beleuchtungsstaerke zu +/-1 EV.
    double measuredEv = log2(lux);
    measuredEv = sanitizeFinite(measuredEv, targetZone, "calculateAutoBW.measuredEv");

    // BETA-FIX: EV-Shift zur Zielzone.
    // BETA-FIX: [Phase 2] Positive Werte bedeuten: Messpunkt liegt unter
    // der eingestellten BW-Zielzone -> mehr Dosis noetig.
    // Negative Werte bedeuten: Messpunkt liegt ueber der Zielzone -> weniger Dosis noetig.
    double evShiftToTarget = targetZone - measuredEv;
    evShiftToTarget = sanitizeFinite(evShiftToTarget, 0.0, "calculateAutoBW.evShiftToTarget");

    // BETA-FIX: Rueckrechnung EV-Shift -> Dosisfaktor mit 2^EV.
    //  +1 EV => Faktor 2.0, -1 EV => Faktor 0.5
    double doseFactor = evToFactor(evShiftToTarget);

    // BETA-FIX: Konnektivitaets-Luecke
    // Es fehlt eine explizite globale Referenzgroesse fuer den absoluten Lux->Zone-
    // Bezug (z.B. kalibrierter Referenz-Lux fuer Zone V am gleichen Messaufbau).
    // Daher erfolgt die absolute Dosisableitung relativ zur aktuellen BW-Basisdosis
    // (bzw. Kbw des aktiven Papiers) und wird nur ueber den EV-Shift korrigiert.
    double calculatedDose = (baseDose * doseFactor);

    calculatedDose = sanitizeFinite(calculatedDose, baseDose, "calculateAutoBW.calculatedDose");
    suggestedDose = clampDouble(calculatedDose, 0.1, 999.0);
}