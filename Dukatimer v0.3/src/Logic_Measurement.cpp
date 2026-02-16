#include <Arduino.h>
#include <math.h>
#include "Globals.h"
#include "Logic_Measurement.h"
#include "Logic_Papers.h"

namespace {
  unsigned long msgUntilMs = 0;
  char msgLine1[17] = {0};
  char msgLine2[17] = {0};

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

  if (cancel) {
    measureMode = MM_OFF;
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
    isMeasuring = true;
    handleLights();
    lcd.setRGB(0, 0, 0);
    updateNextionUI(true);
    double avg = takeAveragedLux(7, 110);
    isMeasuring = false;
    handleLights();
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
