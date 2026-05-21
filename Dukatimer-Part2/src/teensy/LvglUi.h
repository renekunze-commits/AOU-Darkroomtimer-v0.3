#pragma once

#include <lvgl.h>

#include "SystemSnapshot.h"
#include "TftDisplayHal.h"
#include "TouchSampler.h"
#include "UiModalAction.h"
#include "UiPresenter.h"

namespace dukatimer {

struct TouchCalibration {
	int16_t rawMinX = 200;
	int16_t rawMaxX = 3900;
	int16_t rawMinY = 200;
	int16_t rawMaxY = 3900;
	bool swapAxes = false;
	bool invertX = false;
	bool invertY = false;
};

/*
 * LvglUi
 *
 * Zweck:
 * - bildet die lokale LVGL-Huelle ueber Display, Touch und sichtbaren Snapshot
 * - konsumiert ausschliesslich SystemSnapshot und praesentiert daraus Labels,
 *   Overlays und Diagnosezeilen
 * - besitzt die Display-/Touch-Treiberobjekte, aber keine Fachlogik der Modi
 */
class LvglUi {
public:
	// Global sichtbare Modal-Buttons liefern nur noch eine fachliche Aktion.
	// main.cpp speist sie daraus als synthetische lokale Input-Events in die
	// zentrale Normalizer-/Router-/Workflow-Kette ein.

	LvglUi(const TftDisplayConfig& displayConfig, TouchSampler& touchSampler, UiPresenter& presenter,
	       const TouchCalibration& touchCalibration = TouchCalibration{});
	LvglUi(const LvglUi&) = delete;
	LvglUi& operator=(const LvglUi&) = delete;
	LvglUi(LvglUi&&) = delete;
	LvglUi& operator=(LvglUi&&) = delete;

	bool begin();
	void tick(uint32_t elapsedMs);
	void service();
	void updateSnapshot(const SystemSnapshot& snapshot);

	// Liest eine ausstehende globale Modal-Button-Aktion aus und setzt sie zurueck.
	// Muss jeden Loop-Durchlauf in main.cpp abgefragt werden, damit der zentrale
	// Input-Dispatch die Aktion semantisch weiterreichen kann.
	UiModalAction pollModalAction();

	// Wird vom C-Callback lvgl_ui_modal_action_callback() aus screens.c gesetzt.
	void setPendingModalAction(UiModalAction action) { pendingModalAction_ = action; }

	bool dmaFlushSupported() const;
	bool dmaFlushActive() const;

private:
	static void flushDisplay(lv_disp_drv_t* dispDrv, const lv_area_t* area, lv_color_t* colorPtr);
	static void waitForDisplay(lv_disp_drv_t* dispDrv);
	static void readTouch(lv_indev_drv_t* indevDrv, lv_indev_data_t* data);
	static void handleDisplayFlushComplete(void* context);

	void flushDisplayImpl(lv_disp_drv_t* dispDrv, const lv_area_t* area, lv_color_t* colorPtr);
	void waitForDisplayImpl() const;
	void handleDisplayFlushCompleteImpl();
	void readTouchImpl(lv_indev_data_t* data) const;
	uint16_t mapTouchAxis(int16_t rawValue, int16_t rawMin, int16_t rawMax, uint16_t pixelMax, bool invert) const;
	void pushWidgetsFromSnapshot(const SystemSnapshot& snapshot, uint32_t overlayColorHex);

	TftDisplayHal display_;
	TouchSampler& touchSampler_;
	UiPresenter& presenter_;
	TouchCalibration touchCalibration_;
	lv_disp_draw_buf_t drawBuffer_{};
	lv_disp_drv_t displayDriver_{};
	lv_indev_drv_t touchDriver_{};
	lv_disp_t* displayHandle_ = nullptr;
	lv_indev_t* touchHandle_ = nullptr;
	lv_disp_drv_t* pendingFlushDriver_ = nullptr;
	bool flushPending_ = false;
	int16_t currentScreenId_ = -1;
	char dmaText_[96] = {};
	int32_t busyProgressPercent_ = 0;
	UiModalAction pendingModalAction_ = UiModalAction::None;
	bool initialized_ = false;
	// Setup-Screen Lazy-Loading: cache of the last rendered setup state.
	// The setup block skips all LVGL calls when nothing has changed, saving
	// ~200 snprintf + lv_label_set_text calls per second when Setup is visible.
	SetupModeRuntimeState setupStateCache_ = {};
	bool setupThermalCache_ = false;
	float setupOutputCache_ = 1.0f;
};

}  // namespace dukatimer