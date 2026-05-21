#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "AppManager.h"
#include "HardwareManager.h"
#include "config.h"
#include "types.h"

/* =============================================================================
 * InputManager.h - DUKATIMER BETA (v0.916.8)
 * * Hardware-Eingabe-Polling.
 * * HÄRTUNG: Strikte Speichertrennung zwischen Tastern (Momentary) und 
 * * Kippschaltern (Latching), um Out-of-Bounds Memory Corruption zu verhindern.
 * ========================================================================== */

class InputManager
{
public:
    InputManager(AppManager *appMgr, HardwareManager *hwMgr);
    void init();
    void process();

private:
    static constexpr uint8_t kEncoderCount = 4;
    
    // Konstanten für Entprellung und Limits
    static constexpr uint32_t kDebounceMs = 50;
    static constexpr int64_t kMaxEncoderEventsPerCycle = 16;

    AppManager *_appMgr;
    HardwareManager *_hwMgr;

    // Hardware Encoders
    ESP32Encoder _encSoft;
    ESP32Encoder _encHard;
    ESP32Encoder _encGrade;
    ESP32Encoder _encMode;

    ESP32Encoder *_encoders[kEncoderCount];
    int64_t _lastCount[kEncoderCount];

    // --- 1. TASTER (Momentary) ---
    // Index: 0=SoftSW, 1=HardSW, 2=GradeSW, 3=ModeSW, 4=Start
    uint8_t _buttonPins[5];
    uint8_t _btnRawState[5];
    uint8_t _btnStableState[5];
    uint32_t _btnLastChangeMs[5];

    // --- 2. KIPPSCHALTER (Latching) ---
    // Index: 0=Red, 1=White, 2=Room
    uint8_t _switchPins[3];
    uint8_t _swRawState[3];
    uint8_t _swStableState[3];
    uint32_t _swLastChangeMs[3];

    // Hardware State Tracker
    bool _stateRed;
    bool _stateWhite;
    bool _initialized;

    // Liefert true zurück, wenn das Event erfolgreich eingereiht wurde.
    bool dispatchEvent(InputEvent event);

    // Polling Sub-Routinen
    void processEncoders();
    void processButtons();
    void processSwitches();

    // Event-Mapping
    void handleButtonState(uint8_t idx);
    void handleSwitchState(uint8_t idx, bool active);
    InputEvent mapEncoderEvent(uint8_t encoderIndex, bool positiveDirection) const;
};