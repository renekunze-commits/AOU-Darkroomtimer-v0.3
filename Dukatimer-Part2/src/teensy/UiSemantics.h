#pragma once

#include "ExposureRuntimeState.h"
#include "SensorRuntimeStatus.h"

namespace dukatimer {
namespace ui_semantics {

/*
 * ui_semantics
 *
 * Zentrale Kurzlabel fuer die aktuelle eingebettete UI und Diagnoseausgaben.
 * Die Funktionen verdichten technische Zustandswerte in kleine, stabile Strings,
 * damit diese Zuordnung nicht mehrfach in LvglUi oder anderen Ansichten dupli-
 * ziert werden muss.
 */

const char* exposurePhaseLabel(ExposurePhase phase);
const char* exposureModeLabel(ExposureControlMode controlMode);
const char* exposureFaultLabel(ExposureFaultReason faultReason);
const char* exposureFaultDomainLabel(ExposureFaultReason faultReason);
const char* faultLatchLabel(bool faultLatched);
const char* onOffLabel(bool enabled);

const char* sensorHealthLabel(SensorHealth health);
const char* luxValidityLabel(LuxSampleValidity sampleValidity);
const char* thermalStateLabel(ThermalState thermalState);
const char* sampleFreshLabel(bool sampleFresh);
const char* tslDiagnosticReasonLabel(Tsl2561DiagnosticReason diagnosticReason);
const char* ds18DiagnosticReasonLabel(Ds18b20DiagnosticReason diagnosticReason);

}  // namespace ui_semantics
}  // namespace dukatimer
