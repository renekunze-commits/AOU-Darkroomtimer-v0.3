/* HW_Input.cpp - v0.5 Root Cause Edition
   
   ARCHITEKTUR-MAXIME:
   - Zero-Latency: Interrupt-basierte Erfassung aller Taster.
   - Deterministik: Encoder-Polling über Hardware-Counter (PCNT).
   - Grounding: Direkte Event-Generierung für die xInputQueue.
*/

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "driver/gpio.h"
#include "Config.h"
#include "Globals.h"
#include "Types.h"

// =============================================================================
// GLOBALE OBJEKTE & SPEICHER
// =============================================================================

// Encoder-Instanzen (Nutzen ESP32 Hardware Pulse Counter)
ESP32Encoder encSoft;  // Encoder 1 (Links) - Nutzt Zeit
ESP32Encoder encHard;  // Encoder 2 (Mitte) - Nutzt Split-Hard/Navigation
ESP32Encoder encGrade; // Encoder 3 (Rechts) - Nutzt Gradation
ESP32Encoder encMode;  // Encoder 4 (Ganz Rechts) - Master Mode Dial

// Delta-Tracking für Encoder
static long lastPosSoft = 0;
static long lastPosHard = 0;
static long lastPosGrade = 0;
static long lastPosMode = 0;

// Legacy-Stati für Kompatibilität
BtnState sEnter;
BtnState sBack;
BtnState btnStart;
BtnState btnEnc3;
BtnState btnRedLed;
BtnState btnWhiteLed;

// Entprell-Zeitkonstante (ms)
static constexpr uint32_t DEBOUNCE_MS = 50;

// =============================================================================
// HARDWARE INTERRUPTS (ISRs)
// =============================================================================

void IRAM_ATTR isr_button_handler(void* arg) {
    InputEventType type = (InputEventType)(uint32_t)arg;
    static uint32_t last_fire_ms[16] = {0};
    uint32_t now = millis();

    uint8_t idx = (uint8_t)type;
    if (idx >= 16) idx = 15;

    if (now - last_fire_ms[idx] > DEBOUNCE_MS) {
        InputEvent evt;
        evt.type = type;
        evt.value = 0;
        evt.timestamp = now;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xInputQueue, &evt, &xHigherPriorityTaskWoken);
        
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
        last_fire_ms[idx] = now;
    }
}

// =============================================================================
// INITIALISIERUNG
// =============================================================================

void initInput() {
    // Sicherstellen, dass alle Encoder-Pins im sauberen Zustand sind
    gpio_reset_pin((gpio_num_t)ENC_SOFT_A);
    gpio_reset_pin((gpio_num_t)ENC_SOFT_B);
    gpio_reset_pin((gpio_num_t)ENC_HARD_A);
    gpio_reset_pin((gpio_num_t)ENC_HARD_B);
    gpio_reset_pin((gpio_num_t)ENC_GRADE_A);
    gpio_reset_pin((gpio_num_t)ENC_GRADE_B);
    gpio_reset_pin((gpio_num_t)ENC_MODE_A);
    gpio_reset_pin((gpio_num_t)ENC_MODE_B);

    ESP32Encoder::useInternalWeakPullResistors = UP;

    // Wieder auf HalfQuad zurückstellen (sorgt für saubere Zustandsübergänge),
    // da mechanische Raster (Detents) oft 2 halbe Phasen erzeugen.
    encSoft.attachHalfQuad(ENC_SOFT_A, ENC_SOFT_B);
    encHard.attachHalfQuad(ENC_HARD_A, ENC_HARD_B);
    encGrade.attachHalfQuad(ENC_GRADE_A, ENC_GRADE_B);
    encMode.attachHalfQuad(ENC_MODE_A, ENC_MODE_B); // Der 4. Encoder!

    // Hardware-Debouncing (Glitch Filter im ESP32 PCNT Modul)
    // Ignoriert Signale, die kürzer als 1023 APB-Ticks (~12.8us) sind.
    encSoft.setFilter(1023);
    encHard.setFilter(1023);
    encGrade.setFilter(1023);
    encMode.setFilter(1023);

    encSoft.setCount(0);
    encHard.setCount(0);
    encGrade.setCount(0);
    encMode.setCount(0);
    // Taster anbinden
    // Enc1/2/3 und START werden per Polling in HW_Input_Process() erfasst
    // (robuste Entprellung + Long-Press). Nur MODE bleibt auf ISR.
    pinMode(PIN_SW_ENTER, INPUT_PULLUP);
    pinMode(PIN_SW_BACK, INPUT_PULLUP);
    pinMode(PIN_SW_GRADE, INPUT_PULLUP);

    pinMode(PIN_SW_MODE, INPUT_PULLUP);
    attachInterruptArg(PIN_SW_MODE, isr_button_handler, (void*)EVT_MODE_PRESSED, FALLING);

    pinMode(PIN_START, INPUT_PULLUP);

    pinMode(PIN_SW_FOCUS, INPUT_PULLUP);
    pinMode(PIN_SW_SAFE, INPUT_PULLUP);
    pinMode(PIN_SW_ROOMLIGHT, INPUT_PULLUP);
}

// =============================================================================
// ENCODER POLLING + LONG-PRESS DETECTION (Core 1)
// =============================================================================

static constexpr uint32_t LONG_PRESS_MS = 600;

void HW_Input_Process() {
    // --- Encoder Polling ---
    auto pollEncoder = [](ESP32Encoder& enc, long& lastPos, InputEventType type) {
        long rawPos = enc.getCount();
        long curPos = rawPos / 2; 
        if (curPos != lastPos) {
            InputEvent evt;
            evt.type = type;
            evt.value = (int32_t)(curPos - lastPos);
            evt.timestamp = millis();
            xQueueSend(xInputQueue, &evt, 0);
            lastPos = curPos;
        }
    };

    pollEncoder(encSoft,  lastPosSoft,  EVT_ENC_SOFT);
    pollEncoder(encHard,  lastPosHard,  EVT_ENC_HARD);
    pollEncoder(encGrade, lastPosGrade, EVT_ENC_GRADE);
    pollEncoder(encMode,  lastPosMode,  EVT_ENC_MODE);

    // --- H02 FIX: Long-Press-Erkennung für Enc1/2/3 Buttons ---
    static bool btnWasBack = false, btnWasEnter = false, btnWasGrade = false;
    static uint32_t btnTimeBack = 0, btnTimeEnter = 0, btnTimeGrade = 0;
    static bool btnLongBack = false, btnLongEnter = false, btnLongGrade = false;

    auto pollLongPress = [](uint8_t pin, bool& wasPressed, uint32_t& pressTime, 
                            bool& longConsumed, InputEventType shortEvt, InputEventType longEvt) {
        bool pressed = (digitalRead(pin) == LOW);
        uint32_t now = millis();
        if (pressed && !wasPressed) {
            pressTime = now;
            longConsumed = false;
        } else if (pressed && wasPressed && !longConsumed && (now - pressTime) >= LONG_PRESS_MS) {
            longConsumed = true;
            InputEvent evt = { longEvt, 0, now };
            xQueueSend(xInputQueue, &evt, 0);
        } else if (!pressed && wasPressed) {
            if (!longConsumed && (now - pressTime) >= DEBOUNCE_MS) {
                InputEvent evt = { shortEvt, 0, now };
                xQueueSend(xInputQueue, &evt, 0);
            }
        }
        wasPressed = pressed;
    };

    pollLongPress(PIN_SW_BACK,  btnWasBack,  btnTimeBack,  btnLongBack,  EVT_BACK_PRESSED,  EVT_BACK_LONG);
    pollLongPress(PIN_SW_ENTER, btnWasEnter, btnTimeEnter, btnLongEnter, EVT_ENTER_PRESSED, EVT_ENTER_LONG);
    pollLongPress(PIN_SW_GRADE, btnWasGrade, btnTimeGrade, btnLongGrade, EVT_GRADE_PRESSED, EVT_GRADE_LONG);

    // START: stabile Polling-Entprellung + Kurz/Langdruck
    static bool startStablePressed = false;
    static bool startLastRawPressed = false;
    static uint32_t startRawChangedAt = 0;
    static uint32_t startPressTime = 0;
    static bool startLongConsumed = false;

    const bool startRawPressed = (digitalRead(PIN_START) == LOW);
    const uint32_t now = millis();

    if (startRawPressed != startLastRawPressed) {
        startLastRawPressed = startRawPressed;
        startRawChangedAt = now;
    }

    if ((now - startRawChangedAt) >= DEBOUNCE_MS && startStablePressed != startRawPressed) {
        bool wasStablePressed = startStablePressed;
        startStablePressed = startRawPressed;

        if (startStablePressed && !wasStablePressed) {
            startPressTime = now;
            startLongConsumed = false;
        } else if (!startStablePressed && wasStablePressed) {
            if (!startLongConsumed) {
                InputEvent evt = { EVT_START_PRESSED, 0, now };
                xQueueSend(xInputQueue, &evt, 0);
            }
        }
    }

    if (startStablePressed && !startLongConsumed && (now - startPressTime) >= LONG_PRESS_MS) {
        startLongConsumed = true;
        InputEvent evt = { EVT_START_LONG, 0, now };
        xQueueSend(xInputQueue, &evt, 0);
    }
}

// =============================================================================
// LEGACY COMPATIBILITY LAYER
// =============================================================================
bool checkButtonPress(BtnState &st, int pin) {
    bool currentRaw = (digitalRead(pin) == LOW);
    unsigned long now = millis();
    bool eventTriggered = false;

    if (currentRaw != st.lastState) {
        st.lastDebounceTime = now;
        st.lastState = currentRaw;
    }

    if ((now - st.lastDebounceTime) > DEBOUNCE_MS) {
        if (st.isPressed != currentRaw) {
            st.isPressed = currentRaw;
            if (st.isPressed) eventTriggered = true;
        }
    }
    return eventTriggered;
}

bool updateButton(BtnState &st, int pin, unsigned long now) {
    (void)now;
    return checkButtonPress(st, pin);


   

}