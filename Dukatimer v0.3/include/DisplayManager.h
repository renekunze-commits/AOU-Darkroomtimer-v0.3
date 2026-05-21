/* =============================================================================
 * DisplayManager.h - DUKATIMER BETA (v0.905)
 *
 * Zweck:
 * - Zentrale Ausgabe-Schicht fuer zwei Display-Technologien:
 * 1) Grove RGB LCD 2x16 am I2C-0 Bus (primaere, zeitkritische Anzeige)
 * 2) Nextion am UART2 (Informationsanzeige, aktuell nur TX-Output)
 *
 * Design-Prinzipien (Gehärtetes Fundament):
 * - Dependency Injection (SystemContext, HardwareManager)
 * - Keine Globals, keine Arduino String Klasse (Heap-Sicherheit)
 * - Shadow Buffering zur Reduktion von I2C/UART Traffic
 * - Strikt nicht-blockierende update() Methode für Core-0
 * ========================================================================== */

#pragma once

#include <Arduino.h>
// WICHTIG: Erfordert externe Bibliothek "seeed-studio/Grove - LCD RGB Backlight" in platformio.ini!
#include <rgb_lcd.h> 
#include "SystemContext.h"
#include "HardwareManager.h"
#include "config.h" // KORREKTUR: Case-Sensitivity behoben (config.h -> korrekt))

class DisplayManager
{
public:
    DisplayManager(SystemContext *ctx, HardwareManager *hw);

    void init();

    // Zyklisches Display-Update (Aufruf im Main-Loop auf Core 0).
    void update();

    // Externe Sofortmeldung: Text wird intern gespeichert und beim naechsten
    // update() mit Prioritaet und I2C-Lock auf dem LCD dargestellt.
    void smartLCD(const char *l1, const char *l2);

private:
    SystemContext *_ctx;
    HardwareManager *_hw;
    rgb_lcd _lcd;
    bool _initialized;

    // ---------- Smart-LCD Override ----------
    char _smartLine1[17];
    char _smartLine2[17];
    bool _smartPending;

    // ---------- LCD Shadow Buffer ----------
    // Speichert den letzten an das LCD gesendeten Stand, um redundante
    // langsame I2C-Transaktionen zu vermeiden.
    char _lcdShadowLine1[17];
    char _lcdShadowLine2[17];
    uint8_t _lcdShadowR;
    uint8_t _lcdShadowG;
    uint8_t _lcdShadowB;
    bool _lcdShadowValid;

    // ---------- Nextion Shadow Buffer ----------
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

    // Nextion Drosselung (max. 5 Hz => 200 ms).
    unsigned long _nextionLastUpdateMs;

    // ---------- Hilfsfunktionen ----------
    const char *modeToShortName(Mode mode) const;
    uint8_t modeToNextionPage(Mode mode) const;

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

    void selectLcdColor(const WorkflowFlags &flags, uint8_t &r, uint8_t &g, uint8_t &b) const;

    void pushLcdIfChanged(const char *l1, const char *l2, uint8_t r, uint8_t g, uint8_t b);
    void pushSmartLcdNow();

    void nextionSendTerminator();
    void nextionSendCommand(const char *cmd);
    void nextionSendText(const char *obj, const char *value);

    static void safeCopy16(char *dst, const char *src);
    static bool textChanged(const char *a, const char *b);
};