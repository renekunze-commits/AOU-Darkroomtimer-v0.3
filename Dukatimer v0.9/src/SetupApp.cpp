#include "SetupApp.h"
#include "SensorManager.h"
#include <cstdio>
#include <cstring>

/* =============================================================================
 * SetupApp.cpp - DUKATIMER BETA (v0.916.3)
 * * Implementierung des Einstellungsmenüs.
 * (+ FIX: Variable st für das AppSharedState Tagging korrigiert, memset Reihenfolge gefixt)
 * ========================================================================== */

SetupApp::SetupApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm, SensorManager *sm)
    : _ctx(ctx), _hw(hw), _pm(pm), _sm(sm),
      _menuIdx(0), _editActive(false), _subState(0)
{
}

void SetupApp::onEnter()
{
    _menuIdx = 0;
    _editActive = false;
    _subState = 0;

    if (_ctx)
        _ctx->getPreferences(_editPrefs);
    if (_pm)
        _pm->getActiveProfileCopy(_editPaper);

    _hw->playBeep(BEEP_OK);
}

void SetupApp::handleInput(int event)
{
    if (_subState > 0)
    {
        if (_subState == 1)
        {
            if (event == EV_MENU_CLICK)
            {
                if (_sm->startMeasurement(false))
                {
                    _subState = 2;
                    _hw->playBeep(BEEP_OK);
                }
                else
                {
                    _hw->playBeep(BEEP_WARN);
                }
            }
            else if (event == EV_ABORT)
            {
                _subState = 0;
                _hw->playBeep(BEEP_WARN);
            }
        }
        else if (_subState == 2)
        {
            if (event == EV_ABORT)
            {
                if (_sm->isMeasurementRunning())
                    _sm->abortMeasurement();
                _subState = 0;
                _hw->playBeep(BEEP_WARN);
            }
        }
        return;
    }

    if (event == EV_MENU_CLICK)
    {
        if (_editActive)
        {
            _editActive = false;
            _hw->playBeep(BEEP_OK);
        }
        else
        {
            if (_menuIdx == 10)
            {
                _subState = 1;
                _hw->playBeep(BEEP_OK);
            }
            else if (_menuIdx == 11)
            {
                _ctx->setLastError(ERR_NONE);
                _hw->playBeep(BEEP_OK);
            }
            else if (_menuIdx == 12)
            {
                _ctx->setPreferences(_editPrefs);
                _pm->updateActiveProfile(_editPaper);
                _hw->playBeep(BEEP_SAVE);
            }
            else
            {
                _editActive = true;
                _hw->playBeep(BEEP_OK);
            }
        }
        return;
    }

    if (event == EV_ABORT)
    {
        if (_editActive)
        {
            if (_ctx)
                _ctx->getPreferences(_editPrefs);
            if (_pm)
                _pm->getActiveProfileCopy(_editPaper);
            _editActive = false;
            _hw->playBeep(BEEP_WARN);
        }
        return;
    }

    int delta = 0;
    if (event == EV_MENU_UP)
        delta = 1;
    else if (event == EV_MENU_DOWN)
        delta = -1;

    if (delta != 0)
    {
        if (_editActive)
        {
            applyValueChange(delta);
        }
        else
        {
            int next = static_cast<int>(_menuIdx) + delta;
            if (next < 0)
                _menuIdx = 0;
            else if (next >= MAX_MENU_ITEMS)
                _menuIdx = MAX_MENU_ITEMS - 1;
            else
                _menuIdx = static_cast<uint8_t>(next);
        }
        _hw->playBeep(BEEP_TICK);
    }
}

void SetupApp::applyValueChange(int delta)
{
    switch (_menuIdx)
    {
    case 0:
        _editPrefs.stdTime = constrain(_editPrefs.stdTime + (delta * 0.5f), 1.0f, 120.0f);
        break;
    case 1:
        _editPrefs.splitModeAuto = !_editPrefs.splitModeAuto;
        break;
    case 2:
    {
        int s = _editPrefs.soundMode + delta;
        if (s > 2)
            s = 0;
        if (s < 0)
            s = 2;
        _editPrefs.soundMode = s;
    }
    break;
    case 3:
    {
        int m = _editPrefs.stepMode + delta;
        if (m > 4)
            m = 1;
        if (m < 1)
            m = 4;
        _editPrefs.stepMode = m;
    }
    break;
    case 4:
        _editPrefs.bwTargetZone = constrain(_editPrefs.bwTargetZone + (delta * 0.5f), 0.0f, 10.0f);
        break;
    case 5:
        _editPrefs.useWirelessProbe = !_editPrefs.useWirelessProbe;
        break;
    case 6:
        _editPrefs.pwmLcd = constrain(_editPrefs.pwmLcd + (delta * 5), 10, 255);
        break;
    case 7:
        _editPrefs.pwmSafe = constrain(_editPrefs.pwmSafe + (delta * 5), 0, 255);
        break;
    case 8:
        _editPrefs.pwmFocus = constrain(_editPrefs.pwmFocus + (delta * 5), 0, 255);
        break;
    case 9:
        _editPrefs.pwmMax = constrain(_editPrefs.pwmMax + (delta * 5), 0, 255);
        break;
    }
}

void SetupApp::onUpdate()
{
    if (_subState == 2)
    {
        if (_sm->isMeasurementDone())
        {
            _editPrefs.probeDarkLux = _sm->getMeasurementResult();
            _ctx->setPreferences(_editPrefs);

            Serial.printf("[SETUP] Dark Calibration saved: %.4f Lux\n", _editPrefs.probeDarkLux);
            _hw->playBeep(BEEP_OK);
            _subState = 0;
        }
    }
    renderUI();
}

void SetupApp::renderUI()
{
    AppSharedState st{};             // Value-Initialisierung, memset ist nicht mehr nötig.
    st.activeStateMode = MODE_SETUP; // FIX: Richtige Variable und korrektes Reihenfolge-Tagging

    const char *eM = _editActive ? ">" : " ";

    if (_subState == 1)
    {
        std::strcpy(st.setup.line1, "DARK CALIBRAT.");
        std::strcpy(st.setup.line2, "Cover! > ENTER");
    }
    else if (_subState == 2)
    {
        std::strcpy(st.setup.line1, "DARK CALIBRAT.");
        std::strcpy(st.setup.line2, "Measuring...  ");
    }
    else
    {
        switch (_menuIdx)
        {
        case 0:
            std::snprintf(st.setup.line1, 17, "STD TIME BW");
            std::snprintf(st.setup.line2, 17, "%s%.1fs", eM, _editPrefs.stdTime);
            break;
        case 1:
            std::snprintf(st.setup.line1, 17, "SPLIT AUTO");
            std::snprintf(st.setup.line2, 17, "%s%s", eM, _editPrefs.splitModeAuto ? "ON" : "OFF");
            break;
        case 2:
            std::snprintf(st.setup.line1, 17, "SOUND MODE");
            std::snprintf(st.setup.line2, 17, "%s%d", eM, _editPrefs.soundMode);
            break;
        case 3:
            std::snprintf(st.setup.line1, 17, "F-STOP STEP");
            std::snprintf(st.setup.line2, 17, "%s%s", eM, stepNames[constrain(_editPrefs.stepMode, 1, 4) - 1]);
            break;
        case 4:
            std::snprintf(st.setup.line1, 17, "BW TGT ZONE");
            std::snprintf(st.setup.line2, 17, "%s%.1f", eM, _editPrefs.bwTargetZone);
            break;
        case 5:
            std::snprintf(st.setup.line1, 17, "WIRELESS PROBE");
            std::snprintf(st.setup.line2, 17, "%s%s", eM, _editPrefs.useWirelessProbe ? "ON" : "OFF");
            break;
        case 6:
            std::snprintf(st.setup.line1, 17, "PWM LCD BRIGHT");
            std::snprintf(st.setup.line2, 17, "%s%d", eM, _editPrefs.pwmLcd);
            break;
        case 7:
            std::snprintf(st.setup.line1, 17, "PWM SAFELIGHT");
            std::snprintf(st.setup.line2, 17, "%s%d", eM, _editPrefs.pwmSafe);
            break;
        case 8:
            std::snprintf(st.setup.line1, 17, "PWM FOCUS LAMP");
            std::snprintf(st.setup.line2, 17, "%s%d", eM, _editPrefs.pwmFocus);
            break;
        case 9:
            std::snprintf(st.setup.line1, 17, "PWM MAX EXPOSURE");
            std::snprintf(st.setup.line2, 17, "%s%d", eM, _editPrefs.pwmMax);
            break;
        case 10:
            std::strcpy(st.setup.line1, "DARK CALIBRAT.");
            std::snprintf(st.setup.line2, 17, " Current: %.4f", _editPrefs.probeDarkLux);
            break;
        case 11:
            std::strcpy(st.setup.line1, "RESET ERRORS");
            std::strcpy(st.setup.line2, " ENTER = RESET");
            break;
        case 12:
            std::strcpy(st.setup.line1, "SAVE & EXIT");
            std::strcpy(st.setup.line2, " ENTER = SAVE");
            break;
        }
    }

    _ctx->setAppState(st);
}

void SetupApp::onExit()
{
    if (_sm->isMeasurementRunning())
    {
        _sm->abortMeasurement();
    }
}