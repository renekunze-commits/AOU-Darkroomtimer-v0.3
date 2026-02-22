#include "output.h"
#include <U8g2lib.h>
#include <Wire.h>

// I2C Pins für den C6 (gemäß deiner Config)
#define OLED_SDA 6
#define OLED_SCL 7

// Initialisierung des Displays (SSD1306, 128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

// Globale Variablen für den Display-Inhalt (werden von main.cpp befüllt)
char currentHeader[16] = "[ BEREIT ]";
char currentLine1[16] = "Warte auf S3...";
char currentLine2[16] = "";
uint8_t currentHistogram[11] = {0}; // Zonen 0 bis X

// State-Variablen für das Display-Management
unsigned long lastDisplayUpdate = 0;
bool displayNeedsUpdate = true;
bool isDisplaySleeping = false;
const unsigned long DISPLAY_SLEEP_TIMEOUT = 30000; // 30 Sekunden Inaktivität

void setupDisplay() {
    Wire.begin(OLED_SDA, OLED_SCL);
    u8g2.begin();
    u8g2.clearBuffer();
    
    // Boot Screen
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(10, 30, "DUKATIMER C6");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 45, "Verbinde mit S3...");
    u8g2.sendBuffer();
}

void drawHistogram() {
    // Parameter für das Balkendiagramm im unteren Display-Bereich
    const int startX = 2;       // Links einrücken
    const int startY = 63;      // Unten anfangen (Y wächst nach unten!)
    const int maxBarHeight = 24; // Maximale Höhe der Balken
    const int barWidth = 9;     // Breite eines Balkens
    const int barSpacing = 2;   // Abstand zwischen Balken
    
    // Grundlinie zeichnen
    u8g2.drawHLine(0, startY, 128);
    
    // Ziffern 0, 5, X (10) als Orientierungshilfe ganz unten (optional, falls Platz ist)
    // Wir setzen sie knapp über die Grundlinie
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(2, 62, "0");
    u8g2.drawStr(58, 62, "V");
    u8g2.drawStr(115, 62, "X");

    // Die 11 Zonen iterieren
    for (int i = 0; i < 11; i++) {
        // Wert aus Array holen (0-255)
        uint8_t val = currentHistogram[i];
        if (val == 0) continue; // Nichts zu zeichnen
        
        // Höhe mappen: 0-255 auf 1-24 Pixel
        int h = map(val, 0, 255, 1, maxBarHeight);
        
        // X-Position berechnen
        int x = startX + (i * (barWidth + barSpacing));
        
        // Y-Position der oberen linken Ecke des Balkens
        int y = startY - h;
        
        // Balken zeichnen (gefülltes Rechteck)
        u8g2.drawBox(x, y, barWidth, h);
    }
}

void updateDisplay() {
    // Nichts tun, wenn das Display schläft (Stromsparen)
    if (isDisplaySleeping) return;

    // Wir rendern nur neu, wenn es auch neue Daten gab
    if (!displayNeedsUpdate) return;

    u8g2.clearBuffer();
    
    // 1. HEADER (Top-Bar, invertiert oder umrahmt für gute Lesbarkeit)
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0); // Schriftfarbe auf Schwarz (Invertiert)
    
    // Text zentrieren (rudimentär)
    int headerWidth = u8g2.getStrWidth(currentHeader);
    int startX = (128 - headerWidth) / 2;
    u8g2.drawStr(startX, 9, currentHeader);
    
    u8g2.setDrawColor(1); // Zurück auf Weiß für den Rest
    
    // 2. HAUPT-TEXTE (Zeile 1 & 2)
    u8g2.setFont(u8g2_font_helvB10_tr); // Größere Schrift für die Zeit
    u8g2.drawStr(5, 25, currentLine1);
    
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(5, 38, currentLine2);
    
    // 3. ZONEN-HISTOGRAMM
    drawHistogram();
    
    // Puffer auf das Display schieben
    u8g2.sendBuffer();
    
    displayNeedsUpdate = false;
}

void wakeDisplay() {
    if (isDisplaySleeping) {
        u8g2.setPowerSave(0); // Display wieder einschalten
        isDisplaySleeping = false;
        displayNeedsUpdate = true; // Einmal sofort neu zeichnen
    }
    lastDisplayUpdate = millis(); // Timeout zurücksetzen
}

void checkDisplaySleep() {
    if (!isDisplaySleeping && (millis() - lastDisplayUpdate > DISPLAY_SLEEP_TIMEOUT)) {
        u8g2.setPowerSave(1); // OLED in Tiefschlaf versetzen
        isDisplaySleeping = true;
    }
}