#pragma once

#include "SystemSnapshot.h"

namespace dukatimer {

/**
 * @brief UiPresenter translates SystemSnapshot state into UI-ready strings.
 *
 * This class decouples the UI framework (LVGL) from the logic of how
 * system states are described in text. It is additive and purely functional
 * for now, but provides the infrastructure for complex UI mapping.
 */
class UiPresenter {
public:
	UiPresenter() = default;

	// SG Mode Labels
	const char* getSgHeader(const SystemSnapshot& snapshot);
	const char* getSgTargets(const SystemSnapshot& snapshot);
	const char* getSgExposureMain(const SystemSnapshot& snapshot);

	// BW Mode Labels (dedicated getters for SCREEN_ID_PAGE_SPLITGRADE BW reuse and future PageBlackWhite)
	const char* getBwHeader(const SystemSnapshot& snapshot);
	const char* getBwTargets(const SystemSnapshot& snapshot);
	const char* getBwExposureMain(const SystemSnapshot& snapshot);

	// Diagnostic Labels
	const char* getModeInfo(const SystemSnapshot& snapshot);
	const char* getExposureInfo(const SystemSnapshot& snapshot);
	const char* getSensorInfo(const SystemSnapshot& snapshot);
	const char* getRemoteCommandInfo(const SystemSnapshot& snapshot);
	const char* getGatewayInfo(const SystemSnapshot& snapshot);
	const char* getEncoderInfo(const SystemSnapshot& snapshot);
	const char* getSwitchInfo(const SystemSnapshot& snapshot);
	const char* getTouchInfo(const SystemSnapshot& snapshot);
	const char* getMeasurementSources(const SystemSnapshot& snapshot);
	const char* getMeasurementMain(const SystemSnapshot& snapshot);
	const char* getMeasurementMeta(const SystemSnapshot& snapshot);
	const char* getMeasurementReference(const SystemSnapshot& snapshot);
	const char* getMeasurementRange(const SystemSnapshot& snapshot);
	const char* getMeasurementStatusLeft(const SystemSnapshot& snapshot);
	const char* getMeasurementStatusCenter(const SystemSnapshot& snapshot);
	const char* getMeasurementStatusRight(const SystemSnapshot& snapshot);
	const char* getMeasurementUndoChip(const SystemSnapshot& snapshot);
	const char* getDmaInfo(const SystemSnapshot& snapshot, bool fbEnabled, bool dmaSupported, bool dmaActive);

	// PageMeasurement widget labels
	const char* getPageMeasLocalLux(const SystemSnapshot& snapshot);
	const char* getPageMeasWirelessLux(const SystemSnapshot& snapshot);
	const char* getPageMeasSourceChip(const SystemSnapshot& snapshot);
	const char* getPageMeasLuxMain(const SystemSnapshot& snapshot);
	const char* getPageMeasLuxAge(const SystemSnapshot& snapshot);
	const char* getPageMeasRefLux(const SystemSnapshot& snapshot);
	const char* getPageMeasEvDiff(const SystemSnapshot& snapshot);
	const char* getPageMeasSession(const SystemSnapshot& snapshot);

	// Title / Subtitle
	const char* getTitle(const SystemSnapshot& snapshot);
	const char* getSubtitle(const SystemSnapshot& snapshot);

	// Overlay (Special case as it returns text and potentially color state in future)
	const char* getOverlayText(const SystemSnapshot& snapshot);

private:
	char titleBuffer_[64] = {};
	char overlayBuffer_[224] = {};
	char sgHeaderBuffer_[96] = {};
	char sgTargetsBuffer_[128] = {};
	char sgExposureMainBuffer_[128] = {};
};

} // namespace dukatimer
