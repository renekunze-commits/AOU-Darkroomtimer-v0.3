#include "input.h"
#include "config.h"
#include <ESP32Encoder.h>

// =============================================================================
// ENCODER (Hardware PCNT - verliert nie Schritte)
// =============================================================================
ESP32Encoder encoder;

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
    // Taster Pins
    pinMode(PIN_BTN_T1, INPUT_PULLUP);
    pinMode(PIN_BTN_T2, INPUT_PULLUP);
    pinMode(ENC1_BTN, INPUT_PULLUP);
    
    // Encoder mit Hardware Pulse Counter (PCNT)
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachHalfQuad(ENC1_A, ENC1_B);
    encoder.setCount(0);
}

long getEncoderValue() {
    return encoder.getCount() / 2; // Durch 2 für saubere Rastung
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

bool isT1Pressed()  { return checkButton(btnT1,  PIN_BTN_T1); }
bool isT2Pressed()  { return checkButton(btnT2,  PIN_BTN_T2); }
bool isEncPressed() { return checkButton(btnEnc, ENC1_BTN);   }