/* Mode_SG.h - Modus 2: Splitgrade-Belichtung (Soft/Hard getrennt)

   Verantwortlichkeit:
   Encoder- und Taster-Logik für den SG-Modus, ausgelagert aus Logic_Timer.cpp.
   Respektiert hwSwitchDoseMode: Zeit- vs. Dosis-Betrieb bleibt unangetastet.
*/

#ifndef MODE_SG_H
#define MODE_SG_H

#include "Types.h"

/**
 * Verarbeitet ein einzelnes InputEvent im Splitgrade-Modus.
 * Wird von handleInput() in Logic_Timer.cpp aufgerufen,
 * wenn currentMode == MODE_SG.
 *
 * @param evt     Das zu verarbeitende Event aus xInputQueue
 * @param evStep  Die aktuelle EV-Schrittweite (abhängig von globalStepMode)
 * @return true wenn das Event konsumiert wurde, false wenn nicht
 */
bool handleSGInput(const InputEvent& evt, double evStep);

#endif // MODE_SG_H
