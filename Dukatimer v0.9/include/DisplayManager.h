/* =============================================================================
 * DisplayManager.h - DUKATIMER BETA (v0.912)
 * * Zentrale Ausgabesteuerung für Grove RGB LCD (I2C-0) und Nextion (UART2).
 * * ARCHITEKTUR-GRUNDSÄTZE:
 * 1. Low-Traffic Design: Nutzt Shadow-Buffering (Vergleich von Soll/Ist-Zustand).
 * 2. Bus-Guarding: LCD-Zugriffe über HardwareManager-Mutex abgesichert.
 * 3. Non-Blocking UI: Core 0 wird niemals blockiert.
 * (+ FIX: Case-Sensitivity "Types.h" behoben)
 * (+ FIX: pushSmartLcdNow Bypass entfernt für einheitliches Routing)
 * ========================================================================== */

#pragma once

#include <Arduino.h>
#include <rgb_lcd.h> 
#include "SystemContext.h"
#include "HardwareManager.h"
#include "Types.h" // KORREKTUR: Großes 'T' für Linux/CI-Kompatibilität
#include "WirelessManager.h"

class DisplayManager
{
public:
    DisplayManager(SystemContext *ctx, HardwareManager *hw);

    void init();
    void update();

    // Forciert eine asynchrone Overlay-Nachricht für 3000ms.
    // Darf nur von Core 0 gerufen werden!
    void smartLCD(const char *l1, const char *l2);

private:
    SystemContext   *_ctx;
    HardwareManager *_hw;
    rgb_lcd          _lcd;
    
    bool _initialized;

    // --- Smart-LCD Management ---
    char          _smartLine1[17];
    char          _smartLine2[17];
    bool          _smartPending;
    bool          _smartMessageActive;
    unsigned long _smartMessageStartMs;

    // --- Blackout Management ---
    bool _blackoutActive;

    // --- LCD Shadow Buffer ---
    char    _lcdShadowLine1[17];
    char    _lcdShadowLine2[17];
    uint8_t _lcdShadowR, _lcdShadowG, _lcdShadowB;
    bool    _lcdShadowValid;

    // --- Nextion Shadow Buffer ---
    Mode _nextionShadowMode;
    bool _nextionShadowModeValid;
    char _nextionShadowModeText[24];
    char _nextionShadowStatusText[24];
    char _nextionShadowValue1[24];
    char _nextionShadowValue2[24];
    bool _nextionShadowModeTextValid;
    bool _nextionShadowStatusTextValid;
    bool _nextionShadowValue1Valid;
    bool _nextionShadowValue2Valid;

    unsigned long _nextionLastUpdateMs;

    // --- Interne Logik-Bausteine ---
    const char *modeToShortName(Mode mode) const;
    uint8_t     modeToNextionPage(Mode mode) const;

    void buildLcdLines(const ExposureParams &exp, 
                       const HardwareStatus &status, 
                       const WorkflowFlags &flags, 
                       char *outL1, size_t outL1Size, 
                       char *outL2, size_t outL2Size) const;

    void buildNextionPayload(const ExposureParams &exp, 
                             const HardwareStatus &status, 
                             const WorkflowFlags &flags, 
                             char *outMode, size_t outModeSize, 
                             char *outStatus, size_t outStatusSize, 
                             char *outV1, size_t outV1Size, 
                             char *outV2, size_t outV2Size) const;

    void applyDisplayBlackout(bool active);
    void nextionSetDim(uint8_t level);
    void pushLcdIfChanged(const char *l1, const char *l2, uint8_t r, uint8_t g, uint8_t b);

    // Nextion Kommunikations-Primitives
    void nextionSendTerminator();
    void nextionSendCommand(const char *cmd);
    void nextionSendText(const char *obj, const char *value);

    static void safeCopy16(char *dst, const char *src);
    static bool textChanged(const char *a, const char *b);

public:
    // Optional: Verknüpft einen WirelessManager zur Anzeige an das Handgerät
    void setWirelessManager(WirelessManager* w) { _wireless = w; }

private:
    WirelessManager* _wireless = nullptr;
};