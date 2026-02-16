#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>

/*
  DisplayManager.h - Minimal Nextion Communication API
  
  - Grundlegende UART-Kommunikation (RX=1, TX=2)
  - Senden von Befehlen an Nextion
  - Empfangen und Dekodieren von Nextion-Paketen (Touch-Events, etc.)
  - Debug-Logging aller RX/TX Daten
*/

// Initialize Nextion UART connection
void DM_init();

// Process incoming Nextion data (call in loop)
void DM_loop();

// Test Nextion connection (returns true if any response is received)
bool DM_testConnection(uint16_t timeoutMs);

// Send raw command to Nextion (terminators added automatically)
void DM_sendCommand(const String &cmd);
void DM_sendCommand(const char *cmd);

// High-level commands
void DM_page(int page);                                        // Change page
void DM_setText(const String &objName, const String &text);   // Set text field
void DM_setText(const char *objName, const char *text);       // Set text field
void DM_setNumber(const String &objName, int value);          // Set number field
void DM_setPicture(const String &objName, int picID);         // Set picture/icon
void DM_setVisible(const String &objName, bool visible);      // Show/hide component
void DM_setBackgroundColor(const String &objName, uint16_t color); // Set background color
void DM_setForegroundColor(const String &objName, uint16_t color); // Set foreground color

// Debug control
void DM_setDebug(bool on);  // Enable/disable debug logging

#endif