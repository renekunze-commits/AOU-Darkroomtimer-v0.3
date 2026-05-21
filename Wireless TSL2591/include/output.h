#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>
#include "DukatimerProtocol.h"
#include "config.h"

// Hardware-Initialisierung
void initOutput();

void wakeDisplay();
void checkDisplaySleep();

// View-Funktionen (Konkrete Screens)
void showStartup();
void showError(const char* message);
void observeRemoteDiagnostic(const dukatimer::protocol::DiagnosticPayload& payload);
void observeLocalRenderDiagnostics(uint32_t staleRenderCount, uint32_t renderTimeoutCount, bool renderTimeoutActive);
void renderSlaveMode(const dukatimer::protocol::RemoteDisplayPayload& data);
void renderRemoteOfflineMode(float lux);
void renderRemoteMeasurementMode(float lux, bool hardPhase);
void renderStandAloneMode(float lux);
float calculateEV(float lux);

void triggerHaptic(dukatimer::protocol::RemoteHapticFeedback feedback);
void serviceOutput();
void clickSound();

#endif