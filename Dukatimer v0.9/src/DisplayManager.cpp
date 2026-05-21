#include "DisplayManager.h"
#include <cstdio>
#include <cstring>

/* =============================================================================
 * DisplayManager.cpp - DUKATIMER BETA (v0.912)
 * * Zentrale Ausgabeschicht für LCD (I2C-0) und Nextion (UART2).
 * (+ FIX: SmartMessage-Blockade (Licht-Leck während Belichtung) behoben)
 * (+ FIX: Error-Handling integriert (System reagiert nun auf Overheat etc.))
 * (+ FIX: Einheitliches Shadow-Buffer-Routing etabliert)
 * ========================================================================== */

DisplayManager::DisplayManager(SystemContext *ctx, HardwareManager *hw)
    : _ctx(ctx), _hw(hw), _initialized(false),
      _smartPending(false), _smartMessageActive(false), _smartMessageStartMs(0),
      _blackoutActive(false),
      _lcdShadowR(0), _lcdShadowG(0), _lcdShadowB(0), _lcdShadowValid(false),
      _nextionShadowMode(MODE_BW), _nextionShadowModeValid(false),
      _nextionShadowModeTextValid(false), _nextionShadowStatusTextValid(false),
      _nextionShadowValue1Valid(false), _nextionShadowValue2Valid(false),
      _nextionLastUpdateMs(0)
{
    std::memset(_smartLine1, 0, sizeof(_smartLine1));
    std::memset(_smartLine2, 0, sizeof(_smartLine2));
    std::memset(_lcdShadowLine1, 0, sizeof(_lcdShadowLine1));
    std::memset(_lcdShadowLine2, 0, sizeof(_lcdShadowLine2));
    std::memset(_nextionShadowModeText, 0, sizeof(_nextionShadowModeText));
    std::memset(_nextionShadowStatusText, 0, sizeof(_nextionShadowStatusText));
    std::memset(_nextionShadowValue1, 0, sizeof(_nextionShadowValue1));
    std::memset(_nextionShadowValue2, 0, sizeof(_nextionShadowValue2));
}

void DisplayManager::init()
{
    if (_initialized)
        return;

    Serial2.begin(115200, SERIAL_8N1, NEXTION_RX, NEXTION_TX);

    if (_hw->takeI2C())
    {
        _lcd.begin(16, 2);
        _lcd.clear();
        _lcd.setRGB(255, 255, 255);
        _lcd.setCursor(0, 0);
        _lcd.print("Dukatimer v0.912");
        _lcd.setCursor(0, 1);
        _lcd.print("Booting...");
        _hw->giveI2C();
    }
    _initialized = true;
}

void DisplayManager::update()
{
    if (!_initialized || !_ctx || !_hw)
        return;

    const unsigned long now = millis();

    // --- PRIORITÄT 1: DATEN-SNAPSHOTS ---
    ExposureParams exp;
    HardwareStatus status;
    WorkflowFlags flags;

    if (!_ctx->getExposure(exp) || !_ctx->getStatus(status) || !_ctx->getFlags(flags))
        return;

    const bool darknessActive = _hw->isDarknessModeActive();
    bool needsBlackout = flags.isExposureRunning || flags.isMeasuring || darknessActive;
    if (needsBlackout != _blackoutActive)
    {
        applyDisplayBlackout(needsBlackout);
    }

    // --- PRIORITÄT 2: SMART-LCD TIMER LOGIK ---
    if (_smartPending)
    {
        _smartPending = false;
        _smartMessageStartMs = now;
        _smartMessageActive = true;
    }

    if (_smartMessageActive && (now - _smartMessageStartMs) >= 3000UL)
    {
        _smartMessageActive = false;
    }

    // --- PRIORITÄT 3: EINHEITLICHES CONTENT-ROUTING ---
    char line1[17] = {0};
    char line2[17] = {0};
    uint8_t r = 255, g = 255, b = 255;

    // A) Höchste Priorität: System-Fehler (Zwingt Anzeige, überschreibt alles)
    if (status.lastError != ERR_NONE)
    {
        std::snprintf(line1, sizeof(line1), "SYSTEM ERROR!");
        switch (status.lastError)
        {
        case ERR_THERMAL_OVERHEAT:
            std::snprintf(line2, sizeof(line2), "OVERHEAT! %2.0fC", status.tempAlu);
            break;
        case ERR_SENSOR_DISCONNECTED:
            std::snprintf(line2, sizeof(line2), "SENSOR LOST!");
            break;
        case ERR_STORAGE_CORRUPTED:
            std::snprintf(line2, sizeof(line2), "STORAGE ERROR!");
            break;
        case ERR_MUTEX_TIMEOUT:
            std::snprintf(line2, sizeof(line2), "SYSTEM LOCKUP!");
            break;
        case ERR_MATH_INVALID:
            std::snprintf(line2, sizeof(line2), "MATH ERROR!");
            break;
        default:
            std::snprintf(line2, sizeof(line2), "CODE: %d", status.lastError);
            break;
        }
    }
    // B) Overlay-Nachricht
    else if (_smartMessageActive)
    {
        safeCopy16(line1, _smartLine1);
        safeCopy16(line2, _smartLine2);
    }
    // C) Regulärer Betriebs-Modus
    else
    {
        buildLcdLines(exp, status, flags, line1, sizeof(line1), line2, sizeof(line2));
    }

    // --- LCD FARBSTEUERUNG (Absolute Priorität bei Hardware-Aktivität) ---
    if (darknessActive)
    {
        line1[0] = '\0';
        line2[0] = '\0';
        r = 0;
        g = 0;
        b = 0;
    }
    else if (flags.isExposureRunning || flags.isMeasuring || status.lastError != ERR_NONE)
    {
        r = 255;
        g = 0;
        b = 0; // Zwingend Rot bei Belichtung, Messung oder Fehler!
    }

    pushLcdIfChanged(line1, line2, r, g, b);

    // --- PRIORITÄT 4: NEXTION RENDERING ---
    if (needsBlackout)
        return; // Nextion wird während Belichtung verschont

    if ((now - _nextionLastUpdateMs) < 200UL)
        return;
    _nextionLastUpdateMs = now;

    if (!_nextionShadowModeValid || _nextionShadowMode != flags.currentMode)
    {
        char cmd[16];
        std::snprintf(cmd, sizeof(cmd), "page %u", modeToNextionPage(flags.currentMode));
        nextionSendCommand(cmd);
        _nextionShadowMode = flags.currentMode;
        _nextionShadowModeValid = true;
    }

    char mTxt[24], sTxt[24], v1[24], v2[24];
    buildNextionPayload(exp, status, flags, mTxt, sizeof(mTxt), sTxt, sizeof(sTxt), v1, sizeof(v1), v2, sizeof(v2));

    // Fehler-Override auch für Nextion!
    if (status.lastError != ERR_NONE)
    {
        std::strcpy(sTxt, "ERROR");
        std::strcpy(v1, "SYS");
        std::strcpy(v2, "HALT");
    }

    if (textChanged(_nextionShadowModeText, mTxt))
    {
        nextionSendText("tMode", mTxt);
        std::strcpy(_nextionShadowModeText, mTxt);
    }
    if (textChanged(_nextionShadowStatusText, sTxt))
    {
        nextionSendText("tStatus", sTxt);
        std::strcpy(_nextionShadowStatusText, sTxt);
    }
    if (textChanged(_nextionShadowValue1, v1))
    {
        nextionSendText("tV1", v1);
        std::strcpy(_nextionShadowValue1, v1);
    }
    if (textChanged(_nextionShadowValue2, v2))
    {
        nextionSendText("tV2", v2);
        std::strcpy(_nextionShadowValue2, v2);
    }

    // --- Optional: Render UI to wireless Handgerät (C6)
    if (_wireless)
    {
        // Best-effort Haptic / DisplayMode Mapping
        uint8_t hapticFeedback = 0;
        ProbeDisplayMode displayMode = PRB_DISP_IDLE;
        switch (flags.currentMode)
        {
        case MODE_BW:
        case MODE_BW_DOSE:
        case MODE_BW_FSTOP:
            displayMode = PRB_DISP_METER_BW; break;
        case MODE_SG:
        case MODE_SG_DOSE:
            displayMode = PRB_DISP_METER_SG; break;
        case MODE_BURN:
            displayMode = PRB_DISP_BURN; break;
        case MODE_CALIBRATION:
            displayMode = PRB_DISP_CALIBRATE; break;
        case MODE_DENSITOMETER:
            displayMode = PRB_DISP_DENSITOM; break;
        default:
            displayMode = PRB_DISP_IDLE; break;
        }

        char header[16]; std::snprintf(header, sizeof(header), "%-15s", modeToShortName(flags.currentMode)); header[15] = '\0';
        _wireless->sendRender(header, line1, line2, flags.zoneHistogram, hapticFeedback, displayMode);
    }
}

void DisplayManager::applyDisplayBlackout(bool active)
{
    _blackoutActive = active;
    if (active)
    {
        nextionSetDim(0);
    }
    else
    {
        SystemPreferences prefs;
        uint8_t dimPercent = 100;
        if (_ctx->getPreferences(prefs))
        {
            dimPercent = static_cast<uint8_t>(map(prefs.pwmLcd, 0, 255, 10, 100));
        }
        _lcdShadowValid = false;
        nextionSetDim(dimPercent);
    }
}

void DisplayManager::smartLCD(const char *l1, const char *l2)
{
    configASSERT(xPortGetCoreID() == 0); // T6: Harter Core-Check
    safeCopy16(_smartLine1, l1);
    safeCopy16(_smartLine2, l2);
    _smartPending = true;
}

const char *DisplayManager::modeToShortName(Mode mode) const
{
    switch (mode)
    {
    case MODE_BW:
        return "BW-TIME";
    case MODE_SG:
        return "SPLIT-G";
    case MODE_BW_DOSE:
        return "BW-DOSE";
    case MODE_SG_DOSE:
        return "SG-DOSE";
    case MODE_BW_FSTOP:
        return "BW-FSTOP";
    case MODE_ZONE:
        return "ZONE-SYS";
    case MODE_SETUP:
        return "SETUP";
    case MODE_CALIBRATION:
        return "CALIB";
    case MODE_DENSITOMETER:
        return "DENS";
    case MODE_PREFLASH:
        return "PREFLASH";
    case MODE_TEST_STRIP:
        return "T-STRIP";
    case MODE_LIVE_VIEW:
        return "LIVE";
    case MODE_BURN:
        return "BURN";
    default:
        return "UNKNOWN";
    }
}

uint8_t DisplayManager::modeToNextionPage(Mode mode) const
{
    switch (mode)
    {
    case MODE_BW:
    case MODE_BW_DOSE:
    case MODE_BW_FSTOP:
        return 1;
    case MODE_SG:
    case MODE_SG_DOSE:
        return 2;
    case MODE_BURN:
        return 3;
    case MODE_CALIBRATION:
        return 4;
    case MODE_TEST_STRIP:
        return 5;
    case MODE_DENSITOMETER:
        return 6;
    case MODE_SETUP:
        return 7;
    case MODE_ZONE:
        return 8;
    default:
        return 0;
    }
}

void DisplayManager::buildLcdLines(const ExposureParams &exp,
                                   const HardwareStatus &status,
                                   const WorkflowFlags &flags,
                                   char *outL1, size_t outL1Size,
                                   char *outL2, size_t outL2Size) const
{
    if (flags.modePreviewActive)
    {
        std::snprintf(outL1, outL1Size, "SELECT MODE:");
        std::snprintf(outL2, outL2Size, "> %-14s", modeToShortName(flags.previewMode));
        return;
    }

    if (flags.currentMode == MODE_SETUP)
    {
        safeCopy16(outL1, flags.appState.setup.line1);
        safeCopy16(outL2, flags.appState.setup.line2);
        return;
    }

    if (flags.isMeasuring)
    {
        std::snprintf(outL1, outL1Size, "SENSOR MEASURE");
        int bars = (flags.measProgress * 10) / 100;
        if (bars > 10)
            bars = 10;
        char barStr[11] = "          ";
        for (int i = 0; i < bars; i++)
            barStr[i] = '=';
        std::snprintf(outL2, outL2Size, "[%s]%3u%%", barStr, flags.measProgress);
        return;
    }

    std::snprintf(outL1, outL1Size, "%-8s %s", modeToShortName(flags.currentMode),
                  flags.isExposureRunning ? "RUNNING" : "IDLE");

    switch (flags.currentMode)
    {
    case MODE_PREFLASH:
    case MODE_BURN:
    case MODE_LIVE_VIEW:
        std::snprintf(outL2, outL2Size, "T:%4.1f D:%4.2f", flags.appState.exposure.remainingTime, status.liveDose);
        break;

    case MODE_BW:
        std::snprintf(outL2, outL2Size, "T:%4.1fs G:%1.1f", flags.appState.exposure.remainingTime, exp.grade);
        break;

    case MODE_SG:
        std::snprintf(outL2, outL2Size, "S:%4.1f H:%4.1f", flags.appState.sgDose.targetSoft, flags.appState.sgDose.targetHard);
        break;

    case MODE_BW_DOSE:
        std::snprintf(outL2, outL2Size, "T:%4.1fs D:%4.2f", flags.appState.bwDose.remainingDose, exp.targetDoseBw);
        break;

    case MODE_SG_DOSE:
        std::snprintf(outL2, outL2Size, "T:%4.1fs %s", flags.appState.sgDose.remainingDose, flags.appState.sgDose.filterState == 0 ? "SOFT" : "HARD");
        break;

    case MODE_BW_FSTOP:
        std::snprintf(outL2, outL2Size, "T:%4.1fs n:%2d", flags.appState.bwFStop.remainingTime, flags.appState.bwFStop.fStopTicks);
        break;

    case MODE_ZONE:
        std::snprintf(outL2, outL2Size, "Zone %u: %4.2fD", flags.appState.zoneMode.selectedZone, flags.appState.zoneMode.calculatedDose);
        break;

    case MODE_TEST_STRIP:
        std::snprintf(outL2, outL2Size, "Step %d (%.2fL)", flags.appState.testStrip.activeStep, flags.appState.testStrip.lastLux);
        break;

    case MODE_CALIBRATION:
        std::snprintf(outL2, outL2Size, "Wizard Step: %d", flags.appState.calibration.wizardState);
        break;

    case MODE_DENSITOMETER:
        if (!flags.appState.densitometer.isZeroed)
        {
            std::snprintf(outL2, outL2Size, "Set Zero Ref!");
        }
        else
        {
            std::snprintf(outL2, outL2Size, "D: %.2f logD", flags.appState.densitometer.currentDensity);
        }
        break;

    default:
        std::snprintf(outL2, outL2Size, "MISSING CASE!");
        break;
    }
}

void DisplayManager::buildNextionPayload(const ExposureParams &exp,
                                         const HardwareStatus &status,
                                         const WorkflowFlags &flags,
                                         char *outMode, size_t outModeSize,
                                         char *outStatus, size_t outStatusSize,
                                         char *outV1, size_t outV1Size,
                                         char *outV2, size_t outV2Size) const
{
    std::strcpy(outMode, modeToShortName(flags.currentMode));
    std::strcpy(outStatus, flags.isExposureRunning ? "RUNNING" : "IDLE");

    switch (flags.currentMode)
    {
    case MODE_BW:
        std::snprintf(outV1, outV1Size, "T:%4.1f", flags.appState.exposure.remainingTime);
        std::snprintf(outV2, outV2Size, "G:%1.1f", exp.grade);
        break;
    case MODE_SG:
        std::snprintf(outV1, outV1Size, "S:%4.1f", flags.appState.sgDose.targetSoft);
        std::snprintf(outV2, outV2Size, "H:%4.1f", flags.appState.sgDose.targetHard);
        break;
    case MODE_BW_DOSE:
        std::snprintf(outV1, outV1Size, "T:%4.1f", flags.appState.bwDose.remainingDose);
        std::snprintf(outV2, outV2Size, "D:%4.2f", exp.targetDoseBw);
        break;
    case MODE_SG_DOSE:
        std::snprintf(outV1, outV1Size, "T:%4.1f", flags.appState.sgDose.remainingDose);
        std::snprintf(outV2, outV2Size, "%s", flags.appState.sgDose.filterState == 0 ? "SOFT" : "HARD");
        break;
    case MODE_BW_FSTOP:
        std::snprintf(outV1, outV1Size, "T:%4.1f", flags.appState.bwFStop.remainingTime);
        std::snprintf(outV2, outV2Size, "n:%2d", flags.appState.bwFStop.fStopTicks);
        break;
    case MODE_ZONE:
        std::snprintf(outV1, outV1Size, "Z:%u", flags.appState.zoneMode.selectedZone);
        std::snprintf(outV2, outV2Size, "D:%4.2f", flags.appState.zoneMode.calculatedDose);
        break;
    case MODE_TEST_STRIP:
        std::snprintf(outV1, outV1Size, "Step");
        std::snprintf(outV2, outV2Size, "%d", flags.appState.testStrip.activeStep);
        break;
    case MODE_CALIBRATION:
        std::snprintf(outV1, outV1Size, "Step");
        std::snprintf(outV2, outV2Size, "%d", flags.appState.calibration.wizardState);
        break;
    case MODE_DENSITOMETER:
        if (!flags.appState.densitometer.isZeroed)
        {
            std::snprintf(outV1, outV1Size, "Set");
            std::snprintf(outV2, outV2Size, "Zero");
        }
        else
        {
            std::snprintf(outV1, outV1Size, "L0:%.1f", flags.appState.densitometer.refLux);
            std::snprintf(outV2, outV2Size, "D:%.2f", flags.appState.densitometer.currentDensity);
        }
        break;
    case MODE_SETUP:
        outV1[0] = '\0';
        outV2[0] = '\0';
        break;
    default:
        std::snprintf(outV1, outV1Size, "T:%4.1f", flags.appState.exposure.remainingTime);
        std::snprintf(outV2, outV2Size, "D:%4.2f", status.liveDose);
        break;
    }
}

void DisplayManager::pushLcdIfChanged(const char *l1, const char *l2, uint8_t r, uint8_t g, uint8_t b)
{
    const bool lineChanged = !_lcdShadowValid || textChanged(_lcdShadowLine1, l1) || textChanged(_lcdShadowLine2, l2);
    const bool colorChanged = !_lcdShadowValid || _lcdShadowR != r || _lcdShadowG != g || _lcdShadowB != b;

    if (!lineChanged && !colorChanged)
        return;

    if (_hw->takeI2C())
    {
        if (lineChanged)
        {
            _lcd.setCursor(0, 0);
            _lcd.print("                ");
            _lcd.setCursor(0, 0);
            _lcd.print(l1);
            _lcd.setCursor(0, 1);
            _lcd.print("                ");
            _lcd.setCursor(0, 1);
            _lcd.print(l2);
        }
        if (colorChanged)
            _lcd.setRGB(r, g, b);

        _hw->giveI2C();

        safeCopy16(_lcdShadowLine1, l1);
        safeCopy16(_lcdShadowLine2, l2);
        _lcdShadowR = r;
        _lcdShadowG = g;
        _lcdShadowB = b;
        _lcdShadowValid = true;
    }
}

// --- NEXTION LOW-LEVEL PROTOKOL ---
void DisplayManager::nextionSendTerminator() { Serial2.print("\xFF\xFF\xFF"); }
void DisplayManager::nextionSendCommand(const char *cmd)
{
    Serial2.print(cmd);
    nextionSendTerminator();
}
void DisplayManager::nextionSendText(const char *obj, const char *value)
{
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, value);
    Serial2.print(buf);
    nextionSendTerminator();
}
void DisplayManager::nextionSetDim(uint8_t level)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "dim=%u", level);
    nextionSendCommand(buf);
}

// --- STRING-HILFSFUNKTIONEN ---
void DisplayManager::safeCopy16(char *dst, const char *src)
{
    if (!dst)
        return;
    if (!src)
    {
        dst[0] = '\0';
        return;
    }
    std::snprintf(dst, 17, "%.16s", src);
}
bool DisplayManager::textChanged(const char *a, const char *b)
{
    if (!a && !b)
        return false;
    if (!a || !b)
        return true;
    return std::strcmp(a, b) != 0;
}