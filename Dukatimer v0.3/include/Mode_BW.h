/* Mode_BW.h - Modus 1: Schwarz-Weiß-Belichtung (Zeit/Dosis)

   Verantwortlichkeit:
   Encoder- und Taster-Logik für den BW-Modus, ausgelagert aus Logic_Timer.cpp.
   Respektiert hwSwitchDoseMode: Zeit- vs. Dosis-Betrieb bleibt unangetastet.
*/

#ifndef MODE_BW_H
#define MODE_BW_H

#include "Types.h"

/**
 * Verarbeitet ein einzelnes InputEvent im BW-Modus.
 * Wird von handleInput() in Logic_Timer.cpp aufgerufen,
 * wenn currentMode == MODE_BW.
 *
 * @param evt     Das zu verarbeitende Event aus xInputQueue
 * @param evStep  Die aktuelle EV-Schrittweite (abhängig von globalStepMode)
 * @return true wenn das Event konsumiert wurde, false wenn nicht
 */
bool handleBWInput(const InputEvent& evt, double evStep);

#endif // MODE_BW_H
