#include "input.h"
#include "config.h"
#include "DukatimerProtocol.h"

#include <Arduino.h>
#include <RotaryEncoder.h>

// =============================================================================
// ENCODER (interrupt-basiert, plattformagnostisch)
// =============================================================================
RotaryEncoder encoder(PIN_ENC_CLK, PIN_ENC_DT, RotaryEncoder::LatchMode::TWO03);

static portMUX_TYPE inputMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int16_t encoderDelta = 0;
static volatile int64_t encoderLastStep = 0;
static volatile float latestLuxValue = 0.0f;
static volatile uint32_t latestLuxTimestampMs = 0;

namespace {

void IRAM_ATTR checkPosition() {
    encoder.tick();
}

int16_t clampEncoderDelta(int64_t value) {
    if (value > 32767ll) {
        return 32767;
    }
    if (value < -32768ll) {
        return -32768;
    }
    return static_cast<int16_t>(value);
}

void syncEncoderDeltaFromHardware() {
    const int64_t currentStep = static_cast<int64_t>(encoder.getPosition());
    portENTER_CRITICAL(&inputMux);
    const int64_t stepDelta = currentStep - encoderLastStep;
    if (stepDelta != 0) {
        encoderLastStep = currentStep;
        encoderDelta = clampEncoderDelta(static_cast<int64_t>(encoderDelta) + stepDelta);
    }
    portEXIT_CRITICAL(&inputMux);
}

}  // namespace

// =============================================================================
// TASTER: Continuous Debounce (identisch mit S3 Kippschalter-Logik)
// Jeder Flankenwechsel im Rohsignal startet den 50ms Timer neu.
// Erst nach 50ms absoluter Stille wird der neue Zustand übernommen.
// =============================================================================
struct ButtonState {
    bool stablePressed;       // Letzter stabiler (entprellter) Zustand
    bool prevRaw;             // Vorheriger Rohwert (Flankentracking)
    unsigned long edgeMs;     // Zeitstempel des letzten Flankenwechsels
    unsigned long pressStartMs;
    bool longFired;
    bool shortPending;
    bool longPending;
};

static ButtonState btnT1  = { false, false, 0, 0, false, false, false };
static ButtonState btnT2  = { false, false, 0, 0, false, false, false };
static ButtonState btnEnc = { false, false, 0, 0, false, false, false };

static void updateButtonState(ButtonState &st, int pin) {
    const unsigned long DEBOUNCE_MS = 50;
    const unsigned long LONG_PRESS_MS = 600;
    const bool rawPressed = (digitalRead(pin) == LOW);
    const unsigned long now = millis();

    if (rawPressed != st.prevRaw) {
        st.prevRaw = rawPressed;
        st.edgeMs = now;
    }

    if (st.edgeMs != 0 && (now - st.edgeMs) >= DEBOUNCE_MS) {
        if (rawPressed != st.stablePressed) {
            st.stablePressed = rawPressed;
            if (st.stablePressed) {
                st.pressStartMs = now;
                st.longFired = false;
            } else if (!st.longFired) {
                st.shortPending = true;
            }
        }
    }

    if (st.stablePressed && !st.longFired && st.pressStartMs != 0 && (now - st.pressStartMs) >= LONG_PRESS_MS) {
        st.longFired = true;
        st.longPending = true;
    }
}

static void pollButtonStates() {
    updateButtonState(btnT1, PIN_BTN_1);
    updateButtonState(btnT2, PIN_BTN_2);
    updateButtonState(btnEnc, PIN_ENC_SW);
}

void initInput() {
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);
    pinMode(PIN_BTN_1, INPUT_PULLUP);
    pinMode(PIN_BTN_2, INPUT_PULLUP);
    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), checkPosition, CHANGE);
    encoderLastStep = static_cast<int64_t>(encoder.getPosition());
}

static bool consumeShortPress(ButtonState &st) {
    if (st.shortPending) {
        st.shortPending = false;
        return true;
    }
    return false;
}

static bool consumeLongPress(ButtonState &st) {
    if (st.longPending) {
        st.longPending = false;
        return true;
    }
    return false;
}

int16_t getAndClearEncoderDelta() {
    syncEncoderDeltaFromHardware();
    portENTER_CRITICAL(&inputMux);
    const int16_t delta = encoderDelta;
    encoderDelta = 0;
    portEXIT_CRITICAL(&inputMux);
    return delta;
}

void restoreEncoderDelta(int16_t delta) {
    if (delta == 0) {
        return;
    }

    portENTER_CRITICAL(&inputMux);
    encoderDelta = clampEncoderDelta(static_cast<int64_t>(encoderDelta) + delta);
    portEXIT_CRITICAL(&inputMux);
}

uint8_t getActiveButtonMask() {
    pollButtonStates();
    uint8_t buttonMask = dukatimer::protocol::kWirelessRemoteButtonNone;
    if (btnT2.stablePressed) {
        buttonMask |= dukatimer::protocol::kWirelessRemoteButtonMeasure;
    }
    if (btnT1.stablePressed) {
        buttonMask |= dukatimer::protocol::kWirelessRemoteButtonBack;
    }
    if (btnEnc.stablePressed) {
        buttonMask |= dukatimer::protocol::kWirelessRemoteButtonEncoder;
    }
    return buttonMask;
}

float getLuxValue() {
    portENTER_CRITICAL(&inputMux);
    const float lux = latestLuxValue;
    portEXIT_CRITICAL(&inputMux);
    return lux;
}

uint32_t getLuxSampleAgeMs(uint32_t nowMs) {
    if (nowMs == 0u) {
        nowMs = millis();
    }

    portENTER_CRITICAL(&inputMux);
    const uint32_t sampleTimestampMs = latestLuxTimestampMs;
    portEXIT_CRITICAL(&inputMux);

    if (sampleTimestampMs == 0u || sampleTimestampMs > nowMs) {
        return 0u;
    }

    return nowMs - sampleTimestampMs;
}

void publishLuxValue(float lux, uint32_t sampleTimestampMs) {
    if (sampleTimestampMs == 0u) {
        sampleTimestampMs = millis();
    }

    portENTER_CRITICAL(&inputMux);
    latestLuxValue = lux;
    latestLuxTimestampMs = sampleTimestampMs;
    portEXIT_CRITICAL(&inputMux);
}

bool isT1Pressed()  {
    pollButtonStates();
    return consumeShortPress(btnT1);
}

bool isT2Pressed()  {
    pollButtonStates();
    return consumeShortPress(btnT2);
}

bool isEncPressed() {
    pollButtonStates();
    return consumeShortPress(btnEnc);
}

bool isEncLongPressed() {
    pollButtonStates();
    return consumeLongPress(btnEnc);
}