#include <Arduino.h>
#include <math.h>
#include "Globals.h"
#include "Logic_Measurement.h"
#include "Logic_Papers.h"

namespace {
  unsigned long msgUntilMs = 0;
  char msgLine1[17] = {0};
  char msgLine2[17] = {0};
  unsigned long measureDeadlineMs = 0;
  float spectralLuxG0 = 0.0f;

  void setMessage(const char *l1, const char *l2, unsigned long durationMs) {
    snprintf(msgLine1, sizeof(msgLine1), "%-16s", l1 ? l1 : "");
    snprintf(msgLine2, sizeof(msgLine2), "%-16s", l2 ? l2 : "");
    msgUntilMs = millis() + durationMs;
  }

  void clearMeasures(bool allChannels) {
    if (currentMode == MODE_BW) {
      measBWSum = 0.0;
      measBWCount = 0;
      return;
    }
    if (allChannels) {
      measSoftSum = 0.0; measSoftCount = 0;
      measHardSum = 0.0; measHardCount = 0;
      return;
    }
    if (measureMode == MM_APPLY_SG_G0) {
      measSoftSum = 0.0; measSoftCount = 0;
    } else if (measureMode == MM_APPLY_SG_G5) {
      measHardSum = 0.0; measHardCount = 0;
    }
  }

  void renderUI() {
    char l1[17];
    char l2[17];

    if (millis() < msgUntilMs) {
      smartLCD(msgLine1, msgLine2);
      return;
    }

    if (currentMode == MODE_BW) {
      double avg = (measBWCount > 0) ? (measBWSum / (double)measBWCount) : NAN;
      snprintf(l1, sizeof(l1), "BW n%02d", measBWCount);
      if (measBWCount > 0 && isfinite(avg)) {
        snprintf(l2, sizeof(l2), "avg %.2f OK=E", avg);
      } else {
        snprintf(l2, sizeof(l2), "ADD=G OK=E");
      }
      smartLCD(l1, l2);
      return;
    }

    const bool isSoft = (measureMode == MM_APPLY_SG_G0);
    const int count = isSoft ? measSoftCount : measHardCount;
    const double avg = (count > 0) ? ((isSoft ? measSoftSum : measHardSum) / (double)count) : NAN;
    snprintf(l1, sizeof(l1), "SG %s n%02d", isSoft ? "SOFT" : "HARD", count);
    if (count > 0 && isfinite(avg)) {
      snprintf(l2, sizeof(l2), "avg %.2f OK=E", avg);
    } else {
      snprintf(l2, sizeof(l2), "ADD=G TOG=B");
    }
    smartLCD(l1, l2);
  }

  void restorePreviousLights() {
    // Kein direkter Pin-Zugriff hier: nur Override freigeben.
    // Die zentrale handleLights()-State-Machine übernimmt im nächsten Loop
    // wieder vollständig anhand der realen Schalterzustände.
    isMeasuring = false;
    probeFlashActive = false;
    measurementOverrideActive = false;
  }

  void sendRenderPacketToC6(const char* header, const char* line1, const char* line2, uint8_t haptic) {
    uint8_t pMode = (currentMode == MODE_BW) ? PMODE_METER_BW : PMODE_METER_SG;
    sendProbeRender(header, line1, line2, currentZoneHistogram, haptic, pMode);
  }
}

bool triggerSpectralMeasurement() {
  if (currentMeasureState != MEASURE_IDLE) return false;

  isMeasuring = true;
  probeFlashActive = true;

  setLEDMode(LED_GREEN);
  sendProbeMeasureCmd(CMD_MEASURE_G0);
  currentMeasureState = MEASURE_G0_WAIT_DATA;
  measureDeadlineMs = millis() + 3000;
  sendRenderPacketToC6("[ MEASURE ]", "GREEN phase", "Wait data...", HAPTIC_CLICK);
  return true;
}

void abortMeasurementWithError(const char* line1) {
  currentMeasureState = MEASURE_ERROR;
  restorePreviousLights();
  sendRenderPacketToC6("[ ERROR ]", line1 ? line1 : "Measure fail", "", HAPTIC_ERROR);
  beepWarnLong();
  currentMeasureState = MEASURE_IDLE;
}

bool handleMeasurementStateMachine(uint8_t evt, float luxG0, float luxG5) {
  if (currentMeasureState == MEASURE_IDLE) return false;

  if (evt == EVT_T1_CLICK || evt == EVT_T2_CLICK) {
    abortMeasurementWithError("ABORT");
    return true;
  }

  if (currentMeasureState == MEASURE_G0_WAIT_DATA) {
    if (evt == EVT_LUX_DATA) {
      spectralLuxG0 = luxG0;
      setLEDMode(LED_BLUE);
      sendProbeMeasureCmd(CMD_MEASURE_G5);
      currentMeasureState = MEASURE_G5_WAIT_DATA;
      measureDeadlineMs = millis() + 3000;
      sendRenderPacketToC6("[ MEASURE ]", "BLUE phase", "Wait data...", HAPTIC_CLICK);
      return true;
    }
    if ((long)(millis() - measureDeadlineMs) >= 0) {
      abortMeasurementWithError("Timeout G0");
    }
    return false;
  }

  if (currentMeasureState == MEASURE_G5_WAIT_DATA) {
    if (evt == EVT_LUX_DATA) {
      processSpotMeasurement(spectralLuxG0, luxG5);
      // Render nur hier zentral senden (State-Machine Owner), nicht im Math-Kern.
      char l1[16] = {0};
      char l2[16] = {0};
      if (currentMode == MODE_BW) {
        snprintf(l1, sizeof(l1), "T %.2fs", time_bw);
        snprintf(l2, sizeof(l2), "Z8 ref");
      } else {
        snprintf(l1, sizeof(l1), "Tg %.2fs", time_soft);
        snprintf(l2, sizeof(l2), "Tb %.2fs", time_hard);
      }
      sendRenderPacketToC6("[ ZONE VIII ]", l1, l2, HAPTIC_DONE);
      restorePreviousLights();
      currentMeasureState = MEASURE_IDLE;
      return true;
    }
    if ((long)(millis() - measureDeadlineMs) >= 0) {
      abortMeasurementWithError("Timeout G5");
    }
  }
  return false;
}

void processSpotMeasurement(float luxG0, float luxG5) {
  if (!isfinite(luxG0) || !isfinite(luxG5) || luxG0 <= 0.001f || luxG5 <= 0.001f) return;

  const PaperProfile &activePaperProfile = getActivePaper();

  if (currentMeasureFocus == FOCUS_HIGHLIGHTS) {
    // Lichter-Messung: definiert die Basis-Belichtungszeit (Zone VIII Referenz).
    targetDoseSoft = (activePaperProfile.Ksoft > 0.0) ? activePaperProfile.Ksoft : 10.0;
    targetDoseHard = (activePaperProfile.Khard > 0.0) ? activePaperProfile.Khard : 10.0;

    timer_base_seconds = targetDoseSoft / (double)luxG0;
    if (!isfinite(timer_base_seconds) || timer_base_seconds <= 0.0) return;

    if (currentMode == MODE_BW) {
      time_bw = timer_base_seconds;
      measBWSum += luxG0;
      measBWCount++;
    } else {
      time_soft = timer_base_seconds;
      time_hard = targetDoseHard / (double)luxG5;
      measSoftSum += luxG0;
      measSoftCount++;
      measHardSum += luxG5;
      measHardCount++;
    }
    validateTimes();

    memset(currentZoneHistogram, 0, sizeof(currentZoneHistogram));
    currentZoneHistogram[8] = 255;
    beepOk();
    return;
  }

  // Schatten-Messung: nur gültig, wenn zuvor eine Basis-Zeit über Lichter gesetzt wurde.
  if (!isfinite(timer_base_seconds) || timer_base_seconds <= 0.0) return;

  const double doseMinG0 = (activePaperProfile.Ksoft > 0.0) ? activePaperProfile.Ksoft : 10.0;
  // Da keine expliziten doseMax-Felder existieren, robust aus Khard/Ksoft ableiten.
  const double doseMaxG0 = (activePaperProfile.Khard > doseMinG0)
    ? activePaperProfile.Khard
    : (doseMinG0 * 4.0);

  const double range = doseMaxG0 - doseMinG0;
  if (!isfinite(range) || range <= 0.0) return;

  const double spotDose = (double)luxG0 * timer_base_seconds;
  if (!isfinite(spotDose) || spotDose <= 0.0) return;

  const double ratio = spotDose / doseMinG0;
  if (!isfinite(ratio) || ratio <= 0.0) return;
  const double deltaEV = log2(ratio);

  const double rangeEV = log2(doseMaxG0 / doseMinG0);
  if (!isfinite(rangeEV) || rangeEV <= 0.0) return;

  // Zone 8 entspricht doseMinG0. Nach unten Richtung Zone 0, nach oben Richtung 10.
  const double zoneFloat = 8.0 + (deltaEV * (2.0 / rangeEV));
  int zoneIndex = (int)lround(zoneFloat);
  zoneIndex = constrain(zoneIndex, 0, 10);

  memset(currentZoneHistogram, 0, sizeof(currentZoneHistogram));
  currentZoneHistogram[zoneIndex] = 255;
  beepOk();
}

bool isMeteringActive() {
  return (measureMode != MM_OFF);
}

void startMeteringSession() {
  if (starttime != 0 || currentMode == MODE_DENS) {
    beepWarnLong();
    setMessage("BUSY", "STOP TIMER", 800);
    return;
  }

  measureMode = (currentMode == MODE_BW) ? MM_APPLY_BW : MM_APPLY_SG_G0;
  measSoftSum = 0.0; measSoftCount = 0;
  measHardSum = 0.0; measHardCount = 0;
  measBWSum = 0.0;  measBWCount = 0;
  measureUiSinceMs = millis();
  beepNav();
  setMessage("METER MODE", "ADD=G OK=E", 600);
}

void handleMeteringSession(bool addSpot, bool saveApply, bool toggleChannel, bool resetAll, bool cancel) {
  if (measureMode == MM_OFF) return;

  // --- Async Spot Tick (non-blocking Messung läuft) ---
  if (isAsyncSpotBusy()) {
    if (tickAsyncSpot()) {
      // Messung fertig → Ergebnis verarbeiten
      isMeasuring = false;
      handleLights();
      double avg = getAsyncSpotResult();

      if (!isfinite(avg) || avg <= 0.0) {
        beepWarnLong();
        setMessage("MEASURE FAIL", "NO SENSOR", 700);
      } else if (currentMode == MODE_BW) {
        measBWSum += avg; measBWCount++;
        beepOk();
      } else if (measureMode == MM_APPLY_SG_G0) {
        measSoftSum += avg; measSoftCount++;
        beepOk();
      } else if (measureMode == MM_APPLY_SG_G5) {
        measHardSum += avg; measHardCount++;
        beepOk();
      }
    }
    renderUI();
    return; // Während Messung: alle Inputs ignorieren
  }

  if (cancel) {
    cancelAsyncSpot();
    measureMode = MM_OFF;
    isMeasuring = false;
    handleLights();
    beepNav();
    triggerInfo();
    return;
  }

  if (resetAll) {
    clearMeasures(true);
    beepValue();
    setMessage("MEASURE", "CLEARED", 600);
  }

  if (toggleChannel && currentMode == MODE_SG) {
    measureMode = (measureMode == MM_APPLY_SG_G0) ? MM_APPLY_SG_G5 : MM_APPLY_SG_G0;
    beepNav();
    setMessage(measureMode == MM_APPLY_SG_G0 ? "SOFT" : "HARD", "CHANNEL", 400);
  }

  if (addSpot) {
    // Non-blocking: Licht aus, Sensor-Settle abwarten, dann N Samples
    isMeasuring = true;
    handleLights();
    lcd.setRGB(0, 0, 0);
    updateNextionUI(true);
    startAsyncSpot(7, 100, 150); // 150ms settle + 7×100ms sampling
    renderUI();
    return;
  }

  if (saveApply) {
    const PaperProfile &paper = getActivePaper();
    bool applied = false;

    if (currentMode == MODE_BW) {
      if (measBWCount == 0) {
        beepWarnLong();
        setMessage("NO MEASURES", "ADD=G", 800);
      } else if (!(paper.Kbw > 0.0)) {
        beepWarnLong();
        setMessage("NO K DATA", "RUN TEACH", 900);
      } else {
        double avg = measBWSum / (double)measBWCount;
        time_bw = paper.Kbw / avg;
        applied = true;
      }
    } else {
      if (measSoftCount == 0 && measHardCount == 0) {
        beepWarnLong();
        setMessage("NO MEASURES", "ADD=G", 800);
      } else if ((measSoftCount > 0 && !(paper.Ksoft > 0.0)) ||
                 (measHardCount > 0 && !(paper.Khard > 0.0))) {
        beepWarnLong();
        setMessage("NO K DATA", "RUN TEACH", 900);
      } else {
        if (measSoftCount > 0 && paper.Ksoft > 0.0) {
          double avg = measSoftSum / (double)measSoftCount;
          time_soft = paper.Ksoft / avg;
          applied = true;
        }
        if (measHardCount > 0 && paper.Khard > 0.0) {
          double avg = measHardSum / (double)measHardCount;
          time_hard = paper.Khard / avg;
          applied = true;
        }
      }
    }

    if (applied) {
      validateTimes();
      markDirty();
      resetTracking();
      measureMode = MM_OFF;
      beepOk();
      setMessage("MEASURE", "APPLIED", 700);
      triggerInfo();
      return;
    }
  }

  renderUI();
}
