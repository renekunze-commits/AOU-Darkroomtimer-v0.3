#include <Arduino.h>
#include <ESP32Encoder.h> // Library: ESP32Encoder by Kevin Harrington
#include "Config.h"
#include "Globals.h"
#include "Types.h"

/*
  HW_Input.cpp
  - Initialisiert Encoder und Taster
  - Stellt Entprellungslogik bereit (BtnState)
  - Bietet kleine Helper für Mode-Module (updateButton / checkButtonPress)

  Encoder-Konvention:
  - encSoft: Navigation / Zeitänderung
  - encHard: Wertänderung / Menü
  - encGrade: Gradation / Burn
*/

// =============================================================================
// GLOBALE OBJEKTE
// =============================================================================

ESP32Encoder encSoft;  // Encoder 1 (Links): Zeit / Soft
ESP32Encoder encHard;  // Encoder 2 (Mitte): Hard / Menü
ESP32Encoder encGrade; // Encoder 3 (Rechts): Gradation / Burn

// Speicher für alte Positionen (für Delta-Berechnung)
long oldPosSoft = 0;
long oldPosHard = 0;
long oldPosGrade = 0;

// Taster Zustände (Entprellung) - tatsächliche Definitionen
BtnState sEnter;  // Extern deklariert in `Globals.h`
BtnState sBack;
BtnState btnStart;  // Wird auch in main.cpp verwendet
BtnState btnEnc3; // Grade/Shift (intern)
BtnState btnRedLed; // Pin 21 - NeoPixel Red LED
BtnState btnWhiteLed; // Pin 14 - NeoPixel White LED

// =============================================================================
// HILFSFUNKTIONEN
// =============================================================================

// Initialisierung (wird im setup() aufgerufen)
void initInput() {
    // 1. Encoder Hardware aktivieren
    // WICHTIG für S3: Interne Pullups aktivieren!
    ESP32Encoder::useInternalWeakPullResistors = UP;

    // Pins zuweisen (Halb-Quad ist meist besser für Rasterung)
    encSoft.attachHalfQuad(ENC_SOFT_A, ENC_SOFT_B);
    encHard.attachHalfQuad(ENC_HARD_A, ENC_HARD_B);
    encGrade.attachHalfQuad(ENC_GRADE_A, ENC_GRADE_B);

    // Startwerte nullen
    encSoft.setCount(0);
    encHard.setCount(0);
    encGrade.setCount(0);
    
    // Taster Pins sind schon im main.cpp setup() als INPUT_PULLUP gesetzt,
    // aber sicher ist sicher:
    pinMode(PIN_SW_ENTER, INPUT_PULLUP);
    pinMode(PIN_SW_BACK, INPUT_PULLUP);
    pinMode(PIN_SW_GRADE, INPUT_PULLUP);
    // PIN_START ist auch schon gesetzt.

    // Initialisiere Taster-Stati (sicherstellen, dass Felder definiert sind)
    sEnter.lastState = true; sEnter.isPressed = false; sEnter.lastDebounceTime = 0;
    sEnter.pressStartMs = 0; sEnter.lastRepeatMs = 0; sEnter.intervalMs = 0; sEnter.longPressDuration = 500;

    sBack = sEnter;
    btnStart = sEnter;
    btnEnc3 = sEnter;
    btnRedLed = sEnter;
    btnWhiteLed = sEnter;
}

// Button Update Helper (Entprellung)
// Gibt true zurück, wenn der Taster frisch gedrückt wurde (Falling Edge).
// Zusätzlich werden Long-Press/Repeat-Parameter im BtnState vorbereitet
// (z. B. pressStartMs, lastRepeatMs), die bei Bedarf später genutzt werden können.
bool checkButtonPress(BtnState &st, int pin) {
    bool currentRaw = (digitalRead(pin) == LOW); // LOW = Gedrückt
    unsigned long now = millis();
    bool eventTriggered = false;

    // Raw-change: start debounce timer
    if (currentRaw != st.lastState) {
        st.lastDebounceTime = now;
        st.lastState = currentRaw;
    }

    // Debounce abgeschlossen?
    if ((now - st.lastDebounceTime) > 50) { // 50ms Debounce
        if (st.isPressed != currentRaw) {
            st.isPressed = currentRaw;
            if (st.isPressed) {
                // Neuer Druck erkannt
                st.pressStartMs = now;
                st.lastRepeatMs = now;
                eventTriggered = true;
            }
        }
    }
    return eventTriggered;
}

// updateButton ist eine kleine Kompatibilitäts-Hülle damit alte Signaturen
// in Mode_*.cpp unverändert bleiben. Sie delegiert an checkButtonPress.
bool updateButton(BtnState &st, int pin, unsigned long now) {
    (void)now; // zur Kompatibilität mit Signatur
    return checkButtonPress(st, pin);
} 