#include "input.h"
#include "config.h"
#include <ESP32Encoder.h>

ESP32Encoder encoder;
bool wasPressed = false;

void initInput() {
    pinMode(ENC_BTN, INPUT_PULLUP);
    
    // Encoder Setup
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachHalfQuad(ENC_A, ENC_B);
    encoder.setCount(0);
}

long getEncoderValue() {
    return encoder.getCount() / 2; // Durch 2 für saubere Rastung
}

bool isButtonPressed() {
    bool currentState = (digitalRead(ENC_BTN) == LOW);
    bool event = false;

    // Einfache Entprellung / Flankenerkennung
    if (currentState && !wasPressed) {
        event = true;
        wasPressed = true;
        delay(10); // Minimales Debounce
    } else if (!currentState) {
        wasPressed = false;
    }
    
    return event;
}