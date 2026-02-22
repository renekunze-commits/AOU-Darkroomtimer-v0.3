#ifndef LOGIC_MEASUREMENT_H
#define LOGIC_MEASUREMENT_H

#include <Arduino.h>

void startMeteringSession();
bool isMeteringActive();
void handleMeteringSession(bool addSpot, bool saveApply, bool toggleChannel, bool resetAll, bool cancel);
bool triggerSpectralMeasurement();
bool handleMeasurementStateMachine(uint8_t evt, float luxG0, float luxG5);
void abortMeasurementWithError(const char* line1);
void processSpotMeasurement(float luxG0, float luxG5);

#endif
