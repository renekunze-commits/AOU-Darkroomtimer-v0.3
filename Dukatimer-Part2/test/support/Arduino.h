#pragma once

#include <stdint.h>

// Der AP-11-Harness braucht nur `millis()` als kleinsten gemeinsamen Arduino-
// Vertrag. Weitere HAL-Symbole bleiben absichtlich draussen, damit der Test
// keine versehentliche Laufzeitnaehe vorgaukelt.
unsigned long millis();