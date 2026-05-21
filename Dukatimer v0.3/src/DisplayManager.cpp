/*
  DisplayManager.cpp - Nextion Communication HAL (v0.5 Root Cause Edition)
  
  Architektur-Regel: 
  - JEDER Zugriff auf NexSerial MUSS durch xNexMutex geschützt sein.
  - Diese Datei ist die einzige Instanz, die physisch mit Serial2 spricht.
*/

#include "DisplayManager.h"
#include "Config.h"
#include <Arduino.h>

// Definition des globalen Mutex-Handles
SemaphoreHandle_t xNexMutex = NULL;

extern void handleNextionCommand(const char* cmd);

// Wir nutzen Serial2 für das Nextion (Pins in Config.h definiert)
#define NexSerial Serial2

// Hilfsfunktion zur Terminierung von Nextion-Befehlen (Interne Nutzung)
static void terminateCommand() {
  NexSerial.write(0xFF);
  NexSerial.write(0xFF);
  NexSerial.write(0xFF);
}

void DM_init() {
  // Erstellen des Mutex für Thread-Sicherheit
  if (xNexMutex == NULL) {
    xNexMutex = xSemaphoreCreateMutex();
  }

  // Nextion-Panel kann noch auf 9600 stehen. Daher einmalig umstellen und
  // danach dauerhaft mit 115200 arbeiten.
  NexSerial.begin(9600, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  delay(120);
  NexSerial.print("bauds=115200");
  terminateCommand();
  delay(80);
  NexSerial.print("baud=115200");
  terminateCommand();
  delay(80);
  NexSerial.flush();
  NexSerial.end();

  NexSerial.begin(115200, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  delay(150);
  
  // Konfiguration: Touch-Events automatisch senden
  DM_sendCommand("bauds=115200");
  DM_sendCommand("bkcmd=1"); 
}

void DM_sendCommand(const char* cmd) {
  if (xNexMutex == NULL) return;
  if (xSemaphoreTake(xNexMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    NexSerial.print(cmd);
    terminateCommand();
    xSemaphoreGive(xNexMutex);
  }
}

void DM_setText(const char* obj, const char* text) {
  if (xNexMutex == NULL) return;
  if (xSemaphoreTake(xNexMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    NexSerial.print(obj);
    NexSerial.print(".txt=\"");
    NexSerial.print(text);
    NexSerial.print("\"");
    terminateCommand();
    xSemaphoreGive(xNexMutex);
  }
}

// Überladung für String-Objekte (Delegiert an const char* Pfad)
void DM_setText(const char* obj, String text) {
  DM_setText(obj, text.c_str());
}

void DM_setNumber(const char* obj, int val) {
  if (xNexMutex == NULL) return;
  if (xSemaphoreTake(xNexMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    NexSerial.print(obj);
    NexSerial.print(".val=");
    NexSerial.print(val);
    terminateCommand();
    xSemaphoreGive(xNexMutex);
  }
}

void DM_setDimming(uint8_t dimValue) {
  if (dimValue > 100) dimValue = 100;
  if (xNexMutex == NULL) return;
  
  if (xSemaphoreTake(xNexMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    NexSerial.print("dim=");
    NexSerial.print(dimValue);
    terminateCommand();
    xSemaphoreGive(xNexMutex);
  }
}

void DM_loop() {
  static char rxBuf[32];
  static uint8_t rxLen = 0;
  static uint8_t ffCount = 0;
  static uint8_t bootZeroPrefix = 0;
  static bool bootMarker88Seen = false;

  while (NexSerial.available() > 0) {
    uint8_t b = (uint8_t)NexSerial.read();

    if (b == 0xFF) {
      ffCount++;
      if (ffCount >= 3) {
        if (rxLen < sizeof(rxBuf)) {
          rxBuf[rxLen] = '\0';
        } else {
          rxBuf[sizeof(rxBuf) - 1] = '\0';
        }

        const bool bootFrameDetected = (rxLen == 0) && (bootZeroPrefix >= 3 || bootMarker88Seen);

        if (rxLen > 0) {
          handleNextionCommand(rxBuf);
        } else if (bootFrameDetected) {
          // Nextion hat neu gestartet (power-on/reboot frame) -> UI sofort wieder synchronisieren.
          extern void updateNextionUI(bool force);
          updateNextionUI(true);
        }

        rxLen = 0;
        ffCount = 0;
        bootZeroPrefix = 0;
        bootMarker88Seen = false;
      }
      continue;
    }

    ffCount = 0;

    if (rxLen == 0) {
      if (b == 0x00 && bootZeroPrefix < 3) {
        bootZeroPrefix++;
      } else if (b == 0x88) {
        bootMarker88Seen = true;
      } else {
        bootZeroPrefix = 0;
        bootMarker88Seen = false;
      }
    }

    if (b >= 0x20 && b <= 0x7E) {
      if (rxLen < (sizeof(rxBuf) - 1)) {
        rxBuf[rxLen++] = (char)b;
      }
    }
  }
}