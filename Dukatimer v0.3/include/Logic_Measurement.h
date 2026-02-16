#ifndef LOGIC_MEASUREMENT_H
#define LOGIC_MEASUREMENT_H

#include <Arduino.h>

void startMeteringSession();
bool isMeteringActive();
void handleMeteringSession(bool addSpot, bool saveApply, bool toggleChannel, bool resetAll, bool cancel);

#endif
