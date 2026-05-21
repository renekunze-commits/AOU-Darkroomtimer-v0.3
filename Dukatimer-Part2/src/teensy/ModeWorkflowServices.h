#pragma once

#include "HeadTimingDiagnosticsCommandPort.h"
#include "HeadTimingDiagnosticsQueryPort.h"
#include "MeasurementCommandPort.h"
#include "MeasurementQueryPort.h"
#include "PaperProfileCommandPort.h"
#include "PaperProfileQueryPort.h"
#include "SystemSettingsCommandPort.h"
#include "SystemSettingsQueryPort.h"

namespace dukatimer {

/*
 * ModeWorkflowServices
 *
 * Minimales AP-04-Skelett fuer einmalig injizierte Workflow-Dienste. Weitere
 * Ports kommen spaeter hinzu, aber neue Workflows sollen ab hier nicht mehr
 * ueber implizite globale Querzugriffe an Measurement- oder Papierdaten kommen.
 */
struct ModeWorkflowServices {
	HeadTimingDiagnosticsCommandPort* headTimingDiagnosticsCommands = nullptr;
	const HeadTimingDiagnosticsQueryPort* headTimingDiagnosticsQuery = nullptr;
	const MeasurementQueryPort* measurementQuery = nullptr;
	MeasurementCommandPort* measurementCommands = nullptr;
	PaperProfileCommandPort* paperProfileCommands = nullptr;
	const PaperProfileQueryPort* paperProfiles = nullptr;
	SystemSettingsCommandPort* systemSettingsCommands = nullptr;
	const SystemSettingsQueryPort* systemSettingsQuery = nullptr;
};

}  // namespace dukatimer