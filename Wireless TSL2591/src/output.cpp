#include "output.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TwoWire WireOLED = TwoWire(1);
Adafruit_SSD1306 displayLeft(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SSD1306 displayRight(SCREEN_WIDTH, SCREEN_HEIGHT, &WireOLED, -1);

#define BUZZER_CHANNEL 0

// Hardware-Offsets für 0.42" Displays (72x40 auf 128x64 Controller)
#define OFFSET_X 28
#define OFFSET_Y 12 

// Hilfsfunktion: Cursor setzen inkl. Totbereich-Übersprung
void setCursorOffset(Adafruit_SSD1306 &disp, int x, int y) {
    disp.setCursor(x + OFFSET_X, y + OFFSET_Y);
}

// Hilfsfunktion: Linien zeichnen inkl. Totbereich-Übersprung
void drawLineOffset(Adafruit_SSD1306 &disp, int x0, int y0, int x1, int y1, uint16_t color) {
    disp.drawLine(x0 + OFFSET_X, y0 + OFFSET_Y, x1 + OFFSET_X, y1 + OFFSET_Y, color);
}

void initOutput() {
    ledcSetup(BUZZER_CHANNEL, 2000, 8); 
    ledcAttachPin(PIN_BUZZER, BUZZER_CHANNEL);
    ledcWriteTone(BUZZER_CHANNEL, 0);   
    
    WireOLED.begin(I2C_SDA_OLED, I2C_SCL_OLED);

    bool leftOK = displayLeft.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    bool rightOK = displayRight.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

    if(!leftOK || !rightOK) {
        for(int i=0; i<3; i++) { beep(2000, 100); delay(100); }
    }

    displayLeft.clearDisplay();
    displayLeft.setTextColor(SSD1306_WHITE);
    displayLeft.display();

    displayRight.clearDisplay();
    displayRight.setTextColor(SSD1306_WHITE);
    displayRight.display();
}

void showStartup() {
    displayLeft.clearDisplay();
    displayLeft.setTextSize(1);
    setCursorOffset(displayLeft, 0, 10);
    displayLeft.print(F("Probe"));
    displayLeft.display();

    displayRight.clearDisplay();
    displayRight.setTextSize(1);
    setCursorOffset(displayRight, 0, 10);
    displayRight.print(F("Init..."));
    displayRight.display();
    
    beep(1000, 100);
    delay(100);
    beep(1500, 100);
}

void showError(const char* msg) {
    displayLeft.clearDisplay();
    setCursorOffset(displayLeft, 0, 10);
    displayLeft.setTextSize(1);
    displayLeft.print(F("ERROR"));
    displayLeft.display();

    displayRight.clearDisplay();
    setCursorOffset(displayRight, 0, 10);
    displayRight.setTextSize(1);
    displayRight.print(msg);
    displayRight.display();
}

void updateDisplay(float lux, long encVal, uint32_t seq, bool txOK) {
    // --- LINKES DISPLAY: Messwerte ---
    displayLeft.clearDisplay();
    displayLeft.setTextSize(1);
    setCursorOffset(displayLeft, 0, 0);
    displayLeft.print(F("Lux:"));
    
    if (lux >= 100000.0) {
        displayLeft.setTextSize(1);
    } else {
        displayLeft.setTextSize(2);
    }
    setCursorOffset(displayLeft, 0, 16);
    
    if (lux < 100.0) {
        displayLeft.print(lux, 1); 
    } else {
        displayLeft.print(lux, 0); 
    }

    // --- NEU: Dynamischer ESP-NOW Indikator unten (Y=36) ---
    if (txOK) {
        // Durchgehender Pfeil nach rechts (Senden erfolgreich)
        drawLineOffset(displayLeft, 0, 36, 68, 36, SSD1306_WHITE); // Hauptlinie
        drawLineOffset(displayLeft, 64, 33, 68, 36, SSD1306_WHITE); // Pfeilspitze oben
        drawLineOffset(displayLeft, 64, 39, 68, 36, SSD1306_WHITE); // Pfeilspitze unten
    } else {
        // Gestrichelte Linie ohne Pfeilspitze (Fehler/Keine Verbindung)
        for (int i = 0; i < 68; i += 8) {
            drawLineOffset(displayLeft, i, 36, i + 4, 36, SSD1306_WHITE);
        }
    }

    displayLeft.display();

    // --- RECHTES DISPLAY: Status & UI ---
    displayRight.clearDisplay();
    
    displayRight.setTextSize(1);
    setCursorOffset(displayRight, 0, 0);
    displayRight.print(txOK ? F("TX OK") : F("TX ERR"));
    
    setCursorOffset(displayRight, 0, 16);
    displayRight.print(F("Enc: "));
    displayRight.print(encVal);

    setCursorOffset(displayRight, 0, 32);
    displayRight.print(F("Seq: "));
    displayRight.print(seq % 100);
    
    displayRight.display();
}

void beep(int freq, int duration) {
    ledcWriteTone(BUZZER_CHANNEL, freq); 
    delay(duration);                     
    ledcWriteTone(BUZZER_CHANNEL, 0);    
}

void clickSound() {
    ledcWriteTone(BUZZER_CHANNEL, 2000);
    delay(5);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}