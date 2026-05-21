/* HW_Input.cpp - v0.5 Root Cause Edition
   
   ARCHITEKTUR-MAXIME:
   - Deterministik: Encoder-Polling über Hardware-Counter (PCNT).
   - Taster: Absolute Verriegelung (Temporal Lockout) im Realtime-Task.
     Schützt zu 100% vor Feder-Prellen schwerer mechanischer Schalter.
   - Grounding: Direkte Event-Generierung für die xInputQueue.
*/

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "Config.h"
#include "Globals.h"
#include "Types.h"

// =============================================================================
// GLOBALE OBJEKTE & SPEICHER
// =============================================================================

// Encoder-Instanzen (Nutzen ESP32 Hardware Pulse Counter)
ESP32Encoder encSoft;  // Encoder 1 (Links)
ESP32Encoder encHard;  // Encoder 2 (Mitte)
ESP32Encoder encGrade; // Encoder 3 (Rechts)
ESP32Encoder encMode;  // Encoder 4 (Ganz Rechts) - Master Mode Dial

// Delta-Tracking für Encoder
static long lastPosSoft = 0;
static long lastPosHard = 0;
static long lastPosGrade = 0;
static long lastPosMode = 0;

// Legacy-Stati für Abwärtskompatibilität
BtnState sEnter;
BtnState sBack;
BtnState btnStart;
BtnState btnEnc3;
BtnState btnRedLed;
BtnState btnWhiteLed;

// =============================================================================
// ROBUSTES TASTER-POLLING (Temporal Lockout / Absolute Sperrzeit)
// =============================================================================

// ROOT CAUSE FIX: 200ms absolute Totzeit (Taubheit) nach jedem Flankenwechsel.
// Verhindert zu 100% doppelte Start-Events beim Loslassen des Tasters.
static constexpr uint32_t LOCKOUT_MS = 200; 

struct PolledButton {
    int pin;
    InputEventType eventType;
    bool stableState;
    uint32_t lastEventTime;
};

// Unsere 3 Haupt-Taster, die gepollt werden
static PolledButton btns[] = {
    { PIN_SW_ENTER, EVT_ENTER_PRESSED, false, 0 },
    { PIN_SW_BACK,  EVT_BACK_PRESSED,  false, 0 },
    { PIN_START,    EVT_START_PRESSED, false, 0 }
};
static const int numBtns = 3;

// =============================================================================
// INITIALISIERUNG
// =============================================================================

void initInput() {
    ESP32Encoder::useInternalWeakPullResistors = UP;
    
    // ROOT CAUSE FIX: Zurück zum HalfQuad-Modus.
    // KY-040 Encoder liegen mechanisch oft so, dass SingleEdge
    // zu verschluckten Erst-Klicks führt.
    encSoft.attachHalfQuad(ENC_SOFT_A, ENC_SOFT_B);
    encHard.attachHalfQuad(ENC_HARD_A, ENC_HARD_B);
    encGrade.attachHalfQuad(ENC_GRADE_A, ENC_GRADE_B);
    encMode.attachHalfQuad(ENC_MODE_A, ENC_MODE_B);

    encSoft.setCount(0);
    encHard.setCount(0);
    encGrade.setCount(0);
    encMode.setCount(0);

    // Taster als reine Inputs (mit Pullup) initialisieren. Keine Interrupts mehr!
    for (int i = 0; i < numBtns; i++) {
        pinMode(btns[i].pin, INPUT_PULLUP);
    }

    sEnter.lastState = true;
    sBack.lastState = true;
    btnStart.lastState = true;
}

// =============================================================================
// INPUT POLLING (Wird exakt alle 1ms vom TaskRealtime auf Core 1 gerufen)
// =============================================================================

void HW_Input_Process() {
    uint32_t now = millis();

    // 1. DREH-ENCODER AUSLESEN
    auto pollEncoder = [](ESP32Encoder& enc, long& lastPos, InputEventType type, uint32_t timestamp) {
        // ROOT CAUSE FIX: Der Software-Teiler (/ 2) wie gestern.
        // Glättet das mechanische Spiel zwischen den Rastungen zuverlässig aus.
        long curPos = enc.getCount() / 2; 
        
        if (curPos != lastPos) {
            InputEvent evt;
            evt.type = type;
            evt.value = (int32_t)(curPos - lastPos);
            evt.timestamp = timestamp;
            xQueueSend(xInputQueue, &evt, 0);
            lastPos = curPos;
        }
    };

    pollEncoder(encSoft,  lastPosSoft,  EVT_ENC_SOFT, now);
    pollEncoder(encHard,  lastPosHard,  EVT_ENC_HARD, now);
    pollEncoder(encGrade, lastPosGrade, EVT_ENC_GRADE, now);
    pollEncoder(encMode,  lastPosMode,  EVT_ENC_MODE, now);

    // 2. TASTER AUSLESEN (Absolute Verriegelung statt wackeligem Edge-Tracking)
    for (int i = 0; i < numBtns; i++) {
        bool rawPressed = (digitalRead(btns[i].pin) == LOW); // LOW = Taster gedrückt

        // Wenn die Sperrzeit (200ms) abgelaufen ist, dürfen wir Statusänderungen akzeptieren
        if (now - btns[i].lastEventTime > LOCKOUT_MS) {
            
            if (rawPressed && !btns[i].stableState) {
                // Taster wurde GANZ NEU gedrückt
                btns[i].stableState = true;
                btns[i].lastEventTime = now; // Sperre (Totzeit) aktivieren!
                
                InputEvent evt;
                evt.type = btns[i].eventType;
                evt.value = 0;
                evt.timestamp = now;
                xQueueSend(xInputQueue, &evt, 0);
            } 
            else if (!rawPressed && btns[i].stableState) {
                // Taster wurde LOSGELASSEN
                btns[i].stableState = false;
                btns[i].lastEventTime = now; // Auch beim Loslassen für 200ms sperren! (Gegen Feder-Prellen)
            }
        }
    }
}

// =============================================================================
// LEGACY COMPATIBILITY LAYER
// =============================================================================
bool checkButtonPress(BtnState &st, int pin) {
    bool currentRaw = (digitalRead(pin) == LOW);
    unsigned long now = millis();
    bool eventTriggered = false;

    // Nutzt für Legacy-Funktionen ebenfalls eine simple Lockout-Variante
    if (now - st.lastDebounceTime > 150) {
        if (currentRaw != st.isPressed) {
            st.isPressed = currentRaw;
            st.lastDebounceTime = now;
            if (st.isPressed) eventTriggered = true;
        }
    }
    return eventTriggered;
}

bool updateButton(BtnState &st, int pin, unsigned long now) {
    (void)now;
    return checkButtonPress(st, pin);
}