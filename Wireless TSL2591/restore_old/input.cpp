#include "input.h"
#include "config.h"
#include <Arduino.h>
#include <RotaryEncoder.h>

// =============================================================================
// ENCODER (Interrupt-basiert, plattformagnostisch)
// =============================================================================
RotaryEncoder encoder(PIN_ENC_CLK, PIN_ENC_DT, RotaryEncoder::LatchMode::TWO03);

void IRAM_ATTR checkPosition() {
    encoder.tick();
}

// =============================================================================
// TASTER: Continuous Debounce (identisch mit S3 Kippschalter-Logik)
// Jeder Flankenwechsel im Rohsignal startet den 50ms Timer neu.
// Erst nach 50ms absoluter Stille wird der neue Zustand übernommen.
// =============================================================================
struct ButtonState {
    bool stablePressed;      // Letzter stabiler (entprellter) Zustand
    bool prevRaw;            // Vorheriger Rohwert (Flankentracking)
    unsigned long edgeMs;    // Zeitstempel des letzten Flankenwechsels
    bool eventFired;         // Einmalige Flanke wurde gemeldet
};

static ButtonState btnT1  = { false, false, 0, false };
static ButtonState btnT2  = { false, false, 0, false };
static ButtonState btnEnc = { false, false, 0, false };

void initInput() {
    // 1. Encoder Initialisierung
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);

    // 2. Dedizierte Taster Initialisierung
    pinMode(PIN_BTN_1, INPUT_PULLUP);
    pinMode(PIN_BTN_2, INPUT_PULLUP);

    // 3. Output Initialisierung
    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);

    // Encoder Interrupts anhängen (reagiert auf jede Flanke)
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), checkPosition, CHANGE);
}

long getEncoderValue() {
    return encoder.getPosition();
}

// Generische Continuous-Debounce Funktion für alle Taster
static bool checkButton(ButtonState &st, int pin) {
    const unsigned long DEBOUNCE_MS = 50;
    bool rawPressed = (digitalRead(pin) == LOW);
    unsigned long now = millis();

    // Bei jedem Flankenwechsel im Rohsignal: Timer neu starten
    if (rawPressed != st.prevRaw) {
        st.prevRaw = rawPressed;
        st.edgeMs = now;
        st.eventFired = false;  // Neue Flanke → Event darf wieder feuern
    }

    // Erst nach 50ms absoluter Stille neuen Zustand übernehmen
    if (st.edgeMs != 0 && (now - st.edgeMs) >= DEBOUNCE_MS) {
        if (rawPressed != st.stablePressed) {
            st.stablePressed = rawPressed;
            // Nur die Press-Down Flanke als Event melden (einmalig)
            if (st.stablePressed && !st.eventFired) {
                st.eventFired = true;
                return true;
            }
        }
    }

    return false;
}

bool isT1Pressed()  { return checkButton(btnT1,  PIN_BTN_1); }
bool isT2Pressed()  { return checkButton(btnT2,  PIN_BTN_2); }
bool isEncPressed() { return checkButton(btnEnc, PIN_ENC_SW);   }