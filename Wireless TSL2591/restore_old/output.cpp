#include "output.h"
#include <U8g2lib.h>
#include <Wire.h>
#include "config.h"

// Nutze die Standard I2C Schnittstelle. Die Pins wurden bereits global in main.cpp über Wire.begin() gesetzt.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Kapselung: Nur innerhalb DIESER Datei sichtbar
static char currentHeader[16] = "[ BEREIT ]";
static char currentLine1[16] = "Warte auf S3...";
static char currentLine2[16] = "";
static uint8_t currentHistogram[11] = {0}; 

static unsigned long lastDisplayUpdate = 0;
static bool displayNeedsUpdate = true;
static bool isDisplaySleeping = false;

void setupDisplay() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(10, 30, "DUKATIMER C6");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 45, "Verbinde mit S3...");
    u8g2.sendBuffer();
}

void setDisplayData(const char* header, const char* line1, const char* line2, const uint8_t* histogram) {
    strncpy(currentHeader, header, sizeof(currentHeader) - 1);
    currentHeader[sizeof(currentHeader) - 1] = '\0';
    
    strncpy(currentLine1, line1, sizeof(currentLine1) - 1);
    currentLine1[sizeof(currentLine1) - 1] = '\0';
    
    strncpy(currentLine2, line2, sizeof(currentLine2) - 1);
    currentLine2[sizeof(currentLine2) - 1] = '\0';

    if (histogram != nullptr) {
        memcpy(currentHistogram, histogram, 11);
    }

    displayNeedsUpdate = true;
    wakeDisplay(); 
}

void drawHistogram() {
    const int startX = 2;       
    const int startY = 63;      
    const int maxBarHeight = 24; 
    const int barWidth = 9;     
    const int barSpacing = 2;   
    
    u8g2.drawHLine(0, startY, 128);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(2, 62, "0");
    u8g2.drawStr(58, 62, "V");
    u8g2.drawStr(115, 62, "X");

    for (int i = 0; i < 11; i++) {
        uint8_t val = currentHistogram[i];
        if (val == 0) continue; 
        
        int h = map(val, 0, 255, 1, maxBarHeight);
        int x = startX + (i * (barWidth + barSpacing));
        int y = startY - h;
        
        u8g2.drawBox(x, y, barWidth, h);
    }
}

void updateDisplay() {
    if (isDisplaySleeping || !displayNeedsUpdate) return;

    u8g2.clearBuffer();
    
    // Header
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0); 
    int headerWidth = u8g2.getStrWidth(currentHeader);
    int startX = (128 - headerWidth) / 2;
    u8g2.drawStr(startX, 9, currentHeader);
    u8g2.setDrawColor(1); 
    
    // Texte
    u8g2.setFont(u8g2_font_helvB10_tr); 
    u8g2.drawStr(5, 25, currentLine1);
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(5, 38, currentLine2);
    
    // Histogramm
    drawHistogram();
    
    u8g2.sendBuffer();
    displayNeedsUpdate = false;
    lastDisplayUpdate = millis();
}

void wakeDisplay() {
    if (isDisplaySleeping) {
        u8g2.setPowerSave(0); 
        isDisplaySleeping = false;
        displayNeedsUpdate = true; 
    }
    lastDisplayUpdate = millis(); 
}

void checkDisplaySleep() {
    if (!isDisplaySleeping && (millis() - lastDisplayUpdate > DISPLAY_SLEEP_TIMEOUT)) {
        u8g2.setPowerSave(1); 
        isDisplaySleeping = true;
    }
}

// ===== KOMPATIBILITÄTS-FUNKTIONEN (für bestehenden Code) =====

void initOutput() {
    u8g2.setI2CAddress(0x3C * 2); 
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(10, 30, "DUKATIMER C6");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 45, "Verbinde mit S3...");
    u8g2.sendBuffer();
    pinMode(PIN_VIB_MOTOR, OUTPUT);
    digitalWrite(PIN_VIB_MOTOR, LOW);
}

void showStartup() {
    uint8_t emptyHisto[11] = {0};
    setDisplayData("[ START ]", "Initialisiere...", "", emptyHisto);
    updateDisplay();
}

void showError(const char* message) {
    uint8_t emptyHisto[11] = {0};
    setDisplayData("[ FEHLER ]", message, "", emptyHisto);
    updateDisplay();
}

void showStandby(float lux, uint32_t seq, bool connected) {
    char luxStr[16];
    char seqStr[16];
    uint8_t emptyHisto[11] = {0};
    
    // ROOT CAUSE FIX: Dynamische Präzision passend zum Auto-Ranging
    if (lux < 1.0f) {
        snprintf(luxStr, sizeof(luxStr), "Lux: %.3f", lux);
    } else if (lux < 10.0f) {
        snprintf(luxStr, sizeof(luxStr), "Lux: %.2f", lux);
    } else {
        snprintf(luxStr, sizeof(luxStr), "Lux: %.1f", lux);
    }
    
    snprintf(seqStr, sizeof(seqStr), "Seq: %lu", seq);
    
    setDisplayData(connected ? "[ VERBUNDEN ]" : "[ WARTE ]", luxStr, seqStr, emptyHisto);
    updateDisplay();
}

void renderFromPacket(const ProbeRenderPacket& packet) {
    setDisplayData(packet.header_text, packet.line1_text, packet.line2_text, packet.zone_histogram);
    updateDisplay();
    
    if (packet.haptic_feedback == HAPTIC_CLICK) {
        clickSound();
    }
}

void clickSound() {
    // ROOT CAUSE FIX: "tone()" entfernt. 
    // Ein ERM Vibrationsmotor dreht bei 2000Hz Rechteckspannung nicht an.
    digitalWrite(PIN_VIB_MOTOR, HIGH);
    delay(30); // 30ms für klares, knackiges Feedback ohne Nachschwingen
    digitalWrite(PIN_VIB_MOTOR, LOW);
}