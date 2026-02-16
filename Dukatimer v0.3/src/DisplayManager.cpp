#include "DisplayManager.h"
#include "Config.h"
#include <Arduino.h>

/*
  DisplayManager.cpp - Nextion Communication
  FIX: Leitet Touch-Events jetzt korrekt an HW_Display weiter!
*/

// Externe Funktionen aus HW_Display.cpp bekannt machen
extern void handleNextionTouch(uint8_t page, uint8_t id, uint8_t event);
extern void refreshPage(uint8_t pageId);

// We use Serial2 for Nextion
#define NexSerial Serial2

// DM_init: Initialisiert die serielle Verbindung
void DM_init() {
  // Standard 9600 baud
  NexSerial.begin(9600, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  delay(200);
  
  // Nextion konfigurieren für Touch-Events
  DM_sendCommand("bkcmd=3");
  delay(50);
}

#ifdef DEBUG_ENABLED
static bool dmDebug = true;
#else
static bool dmDebug = false;
#endif

// Receive buffer
static uint8_t recvBuf[64];
static int recvPos = 0;

static void processNextionPacket(const uint8_t *buf, int len) {
  if (len <= 0) return;

  uint8_t cmd = buf[0];

  // Fall A: Touch Event (0x65 + Page + ID + Event + End)
  if (cmd == 0x65 && len >= 4) {
      uint8_t page = buf[1];
      uint8_t id   = buf[2];
      uint8_t evt  = buf[3]; // 1=Press, 0=Release
      
      // Weiterleiten an HW_Display!
      handleNextionTouch(page, id, evt);
  }
  
  // Fall B: Sendme / Page Change (0x66 + Page + End)
  // Wird gesendet, wenn du im Editor bei einer Seite "Send Page ID" aktivierst
  else if (cmd == 0x66 && len >= 2) {
      uint8_t page = buf[1];
      refreshPage(page);
  }
}

// DM_loop: Liest Daten und sucht nach dem Terminator FF FF FF
void DM_loop() {
  while (NexSerial.available()) {
    int b = NexSerial.read();
    
    // Buffer Überlaufschutz
    if (recvPos < (int)sizeof(recvBuf)) {
        recvBuf[recvPos++] = (uint8_t)b;
    } else {
        recvPos = 0;
    }

    // Check for Terminator (FF FF FF)
    if (recvPos >= 3 && 
        recvBuf[recvPos-1] == 0xFF && 
        recvBuf[recvPos-2] == 0xFF && 
        recvBuf[recvPos-3] == 0xFF) {
      
      // Paket gefunden! Verarbeitung aufrufen
      // Länge ist recvPos - 3 (ohne die FFs)
      processNextionPacket(recvBuf, recvPos-3);
      
      recvPos = 0; // Reset für nächstes Paket
    }
  }
}

// DM_testConnection: Send "connect"
bool DM_testConnection(uint16_t timeoutMs) {
  while (NexSerial.available()) NexSerial.read(); // Clear input
  DM_sendCommand("connect");

  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (NexSerial.available()) return true;
    delay(1);
  }
  return false;
}

void DM_sendCommand(const String &cmd) {
  NexSerial.print(cmd);
  NexSerial.write(0xFF);
  NexSerial.write(0xFF);
  NexSerial.write(0xFF);
  
  #ifdef DEBUG_ENABLED
  if (dmDebug) { Serial.println("Nex TX: " + cmd); }
  #endif
}

void DM_sendCommand(const char *cmd) {
  if (!cmd) return;
  NexSerial.print(cmd);
  NexSerial.write(0xFF);
  NexSerial.write(0xFF);
  NexSerial.write(0xFF);

  #ifdef DEBUG_ENABLED
  if (dmDebug) { Serial.println(String("Nex TX: ") + cmd); }
  #endif
}

// API Wrapper
void DM_page(int page) { DM_sendCommand("page " + String(page)); }
void DM_setText(const String &objName, const String &text) { DM_sendCommand(objName + ".txt=\"" + text + "\""); }
void DM_setText(const char *objName, const char *text) {
  if (!objName) return;
  char cmd[96];
  snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", objName, text ? text : "");
  DM_sendCommand(cmd);
}
void DM_setNumber(const String &objName, int value) { DM_sendCommand(objName + ".val=" + String(value)); }
void DM_setPicture(const String &objName, int picID) { DM_sendCommand(objName + ".pic=" + String(picID)); }
void DM_setVisible(const String &objName, bool visible) { DM_sendCommand("vis " + objName + "," + String(visible ? 1 : 0)); }
void DM_setBackgroundColor(const String &objName, uint16_t color) { DM_sendCommand(objName + ".bco=" + String(color)); }
void DM_setForegroundColor(const String &objName, uint16_t color) { DM_sendCommand(objName + ".pco=" + String(color)); }
void DM_setDebug(bool on) { dmDebug = on; }