
#include "LvglUi.h"
#include <Arduino.h>
#include <cstdio>
#include "InputRouterPolicy.h"
#include "ui/eez_ui/DukatimerPart2TeensyUi/src/ui/eez-flow.h"
#include "ui/eez_ui/DukatimerPart2TeensyUi/src/ui/ui.h"
#include "ui/eez_ui/DukatimerPart2TeensyUi/src/ui/screens.h"
#include "ui/eez_ui/DukatimerPart2TeensyUi/src/ui/vars.h"

extern uint8_t external_psram_size;

namespace {
// Double-buffer je 80 Zeilen = 2 × 75 KB in DMAMEM/RAM2 (war: 40 Zeilen = 2 × 37.5 KB).
// Weniger Render-Passes pro Frame → flüssigeres Scrolling / komplexe Screens.
// RAM2 hat nach letztem Build 435 KB frei; 150 KB Gesamtbedarf ist unkritisch.
constexpr uint16_t kLvglDrawBufferLines = 80;
constexpr uint16_t kLvglMaxHorizontalResolution = 480;
constexpr size_t kLvglDrawBufferPixelCapacity = static_cast<size_t>(kLvglMaxHorizontalResolution) * kLvglDrawBufferLines;
DMAMEM static lv_color_t sLvglDrawBuffer1[kLvglDrawBufferPixelCapacity];
DMAMEM static lv_color_t sLvglDrawBuffer2[kLvglDrawBufferPixelCapacity];

// Overlay-Palette — Runtime-Fallback fuer das Meldungsband.
// Werte muessen den EEZ-Projektfarben C_MSG_* entsprechen (Abschnitt 4.1 der Doku).
// Zielvertrag: EEZ Studio als Farb-Owner. Diese Konstanten bleiben Fallback,
// bis das Meldungsband vollstaendig ueber EEZ-Flow-Bindings gespeist wird.
// EEZ-Name            → C++ Konstante
// C_MSG_NORMAL        → kOverlayColorNormal
// C_MSG_FAULT         → kOverlayColorWorkflowFault
// C_MSG_CONFIRM       → kOverlayColorWorkflowConfirm
// C_MSG_WAIT          → kOverlayColorWorkflowWait
// C_MSG_FALLBACK      → kOverlayColorSensorFallback
// C_MSG_COMPLETED     → kOverlayColorSgCompleted
// C_MSG_ABORTED       → kOverlayColorSgAborted
// C_MSG_LINK_LOST     → kOverlayColorLinkDegraded
// C_MSG_SG_DIRTY      → kOverlayColorSgDirty
constexpr uint32_t kOverlayColorNormal = 0x120606u;
constexpr uint32_t kOverlayColorWorkflowFault = 0x3A0506u;
constexpr uint32_t kOverlayColorWorkflowConfirm = 0x2A100Au;
constexpr uint32_t kOverlayColorWorkflowWait = 0x1C0B0Bu;
constexpr uint32_t kOverlayColorSensorFallback = 0x34120Cu;
constexpr uint32_t kOverlayColorSgCompleted = 0x1B100Cu;
constexpr uint32_t kOverlayColorSgAborted = 0x241112u;
constexpr uint32_t kOverlayColorLinkDegraded = 0x2D120Eu;
constexpr uint32_t kOverlayColorSgDirty = 0x1F0C0Cu;

const dukatimer::PaperSlotUiSummary* resolveSelectedPaperSlotSummary(
    const dukatimer::SystemSnapshot& snapshot) {
    const uint8_t selectedSlot = snapshot.modeState.paper.selectedSlot;
    if (selectedSlot >= snapshot.paperSlotCount || selectedSlot >= dukatimer::kPaperSlotCount) {
        return nullptr;
    }

    const dukatimer::PaperSlotUiSummary& summary = snapshot.paperSlotSummaries[selectedSlot];
    return summary.available ? &summary : nullptr;
}

int16_t computeTargetScreen(const dukatimer::SystemSnapshot& snapshot) {
    // Belichtungsaktive Phasen haben absolute Priorität — kein anderer Screen-
    // Wechsel während Belichtung. PostWait bleibt auf Busy bis Done abgeschlossen.
    const auto expPhase = snapshot.exposureState.phase;
    if (expPhase == dukatimer::ExposurePhase::PreWait ||
        expPhase == dukatimer::ExposurePhase::Exposing ||
        expPhase == dukatimer::ExposurePhase::Paused ||
        expPhase == dukatimer::ExposurePhase::PostWait) {
        return SCREEN_ID_BUSY;
    }

    const auto mode = snapshot.modeState.activeMode;
    if (mode == dukatimer::ModeId::None) {
        return SCREEN_ID_BOOT;
    }
    if (mode == dukatimer::ModeId::Paper) {
        return SCREEN_ID_PAGE_PAPER_WORKSPACE;
    }
    if (mode == dukatimer::ModeId::Setup) {
        return SCREEN_ID_PAGE_SETUP;
    }
    if (mode == dukatimer::ModeId::Splitgrade || mode == dukatimer::ModeId::BlackWhite) {
        bool measurementVisible = false;
        if (mode == dukatimer::ModeId::Splitgrade) {
            measurementVisible =
                snapshot.modeState.splitgrade.panel == dukatimer::SplitgradePanel::Measurement &&
                snapshot.exposureState.phase == dukatimer::ExposurePhase::Idle;
        } else {
            measurementVisible =
                snapshot.modeState.bw.panel == dukatimer::BwPanel::Measurement &&
                snapshot.exposureState.phase == dukatimer::ExposurePhase::Idle;
        }
        return measurementVisible ? SCREEN_ID_PAGE_MEASUREMENT : SCREEN_ID_PAGE_SPLITGRADE;
    }
    return SCREEN_ID_BOOT;
}

int32_t computeWorkflowFamily(const dukatimer::SystemSnapshot& snapshot) {
    // UI-Familienvertrag (EEZ-ModeTabs): 0=PAPER, 1=MEAS, 2=PRINT, 3=SETUP
    // ModeId::None (Boot) hat keinen aktiven Tab; 0 als neutraler Standardwert.
    const auto mode = snapshot.modeState.activeMode;
    if (mode == dukatimer::ModeId::Paper) return 0;
    if (mode == dukatimer::ModeId::Splitgrade) {
        return (snapshot.modeState.splitgrade.panel == dukatimer::SplitgradePanel::Measurement)
            ? 1 : 2;
    }
    if (mode == dukatimer::ModeId::BlackWhite) {
        return (snapshot.modeState.bw.panel == dukatimer::BwPanel::Measurement)
            ? 1 : 2;
    }
    if (mode == dukatimer::ModeId::Setup) return 3;
    return 0; // ModeId::None (Boot) — kein aktiver ModeTab
}
}

// Singleton-Zeiger fuer den C-Callback aus screens.c (busy_btn_cb).
// Ausserhalb des anonymous namespace damit extern "C" keine Linkage-Konflikte erzeugt.
static dukatimer::LvglUi* s_lvglInstance = nullptr;

// C-Linkage: wird aus screens.c aufgerufen, sobald ein globaler Modal-Button
// eine fachliche Aktion wie Resume, Abort oder kuenftig Pause meldet.
extern "C" void lvgl_ui_modal_action_callback(uint8_t action) {
    if (s_lvglInstance) {
        s_lvglInstance->setPendingModalAction(
            dukatimer::decodeUiModalAction(static_cast<int16_t>(action)));
    }
}

namespace dukatimer {

LvglUi::LvglUi(const TftDisplayConfig& displayConfig,
                             TouchSampler& touchSampler,
                             UiPresenter& presenter,
                             const TouchCalibration& touchCalibration)
        : display_(displayConfig),
            touchSampler_(touchSampler),
            presenter_(presenter),
            touchCalibration_(touchCalibration) {}

FLASHMEM bool LvglUi::begin() {
if (initialized_) return true;
// Der aktuelle LVGL-Heap ist voll auf EXTRAM/PSRAM umgestellt. Ohne vom
// Teensy-Core erkanntes PSRAM wuerde der Custom-Allocator spaeter nur NULL
// liefern; deshalb lehnt der Start diese Hardwarekonstellation bewusst frueh ab.
if (external_psram_size == 0u) {
    Serial.println("[UI] external PSRAM missing - LVGL startup rejected");
    return false;
}
lv_init();
if (!display_.begin()) return false;
lv_disp_draw_buf_init(&drawBuffer_, sLvglDrawBuffer1, sLvglDrawBuffer2, kLvglDrawBufferPixelCapacity);
lv_disp_drv_init(&displayDriver_);
displayDriver_.hor_res = static_cast<lv_coord_t>(display_.driver().width());
displayDriver_.ver_res = static_cast<lv_coord_t>(display_.driver().height());
displayDriver_.flush_cb = flushDisplay;
displayDriver_.wait_cb = waitForDisplay;
displayDriver_.draw_buf = &drawBuffer_;
displayDriver_.user_data = this;
displayHandle_ = lv_disp_drv_register(&displayDriver_);
lv_indev_drv_init(&touchDriver_);
touchDriver_.type = LV_INDEV_TYPE_POINTER;
touchDriver_.read_cb = readTouch;
touchDriver_.user_data = this;
touchHandle_ = lv_indev_drv_register(&touchDriver_);
ui_init();
s_lvglInstance = this;
eez_flow_set_screen(SCREEN_ID_BOOT, LV_SCR_LOAD_ANIM_NONE, 0, 0);
currentScreenId_ = SCREEN_ID_BOOT;
initialized_ = true;
return true;
}

void LvglUi::tick(uint32_t elapsedMs) {
// Nur LVGL-Tick-Zaehler. EEZ-Flow-Tick (eez_flow_tick + tick_screen) liegt in service().
if (!initialized_) return;
lv_tick_inc(elapsedMs);
}

UiModalAction LvglUi::pollModalAction() {
    const UiModalAction action = pendingModalAction_;
    pendingModalAction_ = UiModalAction::None;
    return action;
}
void LvglUi::service() {
if (!initialized_) return;
lv_timer_handler();
ui_tick(); // EEZ-Flow-Tick: eez_flow_tick() + tick_screen(g_currentScreen)
}

void LvglUi::flushDisplay(lv_disp_drv_t* dispDrv, const lv_area_t* area, lv_color_t* colorPtr) {
static_cast<LvglUi*>(dispDrv->user_data)->flushDisplayImpl(dispDrv, area, colorPtr);
}
void LvglUi::waitForDisplay(lv_disp_drv_t* dispDrv) {
static_cast<LvglUi*>(dispDrv->user_data)->waitForDisplayImpl();
}
void LvglUi::readTouch(lv_indev_drv_t* indevDrv, lv_indev_data_t* data) {
static_cast<LvglUi*>(indevDrv->user_data)->readTouchImpl(data);
}

void LvglUi::flushDisplayImpl(lv_disp_drv_t* dispDrv, const lv_area_t* area, lv_color_t* colorPtr) {
const int16_t x = static_cast<int16_t>(area->x1);
const int16_t y = static_cast<int16_t>(area->y1);
const int16_t w = static_cast<int16_t>(area->x2 - area->x1 + 1);
const int16_t h = static_cast<int16_t>(area->y2 - area->y1 + 1);
pendingFlushDriver_ = dispDrv;
flushPending_ = true;
display_.writeRect(x, y, w, h, &colorPtr->full);
flushPending_ = false;
lv_disp_flush_ready(dispDrv);
}

void LvglUi::waitForDisplayImpl() const {
while (flushPending_) { yield(); }
}

void LvglUi::handleDisplayFlushCompleteImpl() {
if (flushPending_ && pendingFlushDriver_) {
flushPending_ = false;
lv_disp_flush_ready(pendingFlushDriver_);
}
}

// ---------------------------------------------------------------------------
// Direct widget push — called from updateSnapshot() every UI tick.
// Reads the SystemSnapshot that was already formatted for EEZ globals and
// pushes text / colour changes directly to the hand-crafted LVGL handles.
// FLASHMEM keeps this large function out of ITCM.
// ---------------------------------------------------------------------------
FLASHMEM void LvglUi::pushWidgetsFromSnapshot(const SystemSnapshot& snapshot,
                                               uint32_t overlayColorHex) {
    DukaWidgets& w = g_duka_widgets;

    // ---- Busy screen (fruehzeitige Rueckkehr — eigener Header, kein shared hdr) --
    // B7-BLOCKIERT (Prompt B7, BusyScreen): Safety-Gate 1 (Prerequisite: min.
    // 1 nicht-sicherheitskritische Seite migriert) und Gate 2 (tick_screen_busy
    // kann EEZ propagieren) nicht erfuellt. Direktpfad bleibt aktiv und ist
    // safety-korrekt (Pause/Resume/Abort nur Event-Buffer, ExposureEngine
    // unveraendert). Busy-Header getrennt von normalen hdr_* Handles (kein
    // build_header()-Aufruf). Abschnitt 13.12.
    if (currentScreenId_ == SCREEN_ID_BUSY) {
        const ExposureRuntimeState& exp = snapshot.exposureState;

        // Phasenlabel
        if (w.busy_hdr_phase_lbl) {
            const char* phaseText = "BELICHTUNG";
            if (exp.phase == ExposurePhase::PreWait)       phaseText = "VORBEREITUNG";
            else if (exp.phase == ExposurePhase::Paused)   phaseText = "PAUSE";
            else if (exp.phase == ExposurePhase::PostWait) phaseText = "NACHWARTEZEIT";
            lv_label_set_text_static(w.busy_hdr_phase_lbl, phaseText);
        }
        // Message-Band Hintergrundfarbe (Fallback / Fault Signalfarbe)
        if (w.busy_hdr_msg_cont) {
            lv_obj_set_style_bg_color(w.busy_hdr_msg_cont, lv_color_hex(overlayColorHex), LV_PART_MAIN);
        }
        if (w.busy_hdr_msg_lbl) {
            lv_label_set_text_static(w.busy_hdr_msg_lbl, presenter_.getOverlayText(snapshot));
        }
        if (w.busy_hdr_thermal_lbl) {
            if (exp.thermalDeratingActive) {
                lv_obj_clear_flag(w.busy_hdr_thermal_lbl, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(w.busy_hdr_thermal_lbl, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // Zeitanzeige
        if (w.busy_time_lbl) {
            if (exp.phase == ExposurePhase::PreWait || exp.phase == ExposurePhase::PostWait) {
                lv_label_set_text_static(w.busy_time_lbl, "...");
            } else if (exp.controlMode == ExposureControlMode::Dose) {
                std::snprintf(dmaText_, sizeof(dmaText_), "%.3f / %.3f lx\xB7s",
                    static_cast<double>(exp.currentDose), static_cast<double>(exp.targetDose));
                lv_label_set_text(w.busy_time_lbl, dmaText_);
            } else {
                std::snprintf(dmaText_, sizeof(dmaText_), "%.1f s",
                    static_cast<double>(exp.remainingTimeSeconds));
                lv_label_set_text(w.busy_time_lbl, dmaText_);
            }
        }

        // Fortschrittsbalken + %
        if (w.busy_bar) {
            int32_t pct = busyProgressPercent_;
            if (exp.phase == ExposurePhase::PreWait) {
                pct = 0;
                busyProgressPercent_ = pct;
            } else if (exp.phase == ExposurePhase::PostWait) {
                pct = 100;
                busyProgressPercent_ = pct;
            } else if (exp.phase == ExposurePhase::Exposing) {
                if (exp.controlMode == ExposureControlMode::Dose && exp.targetDose > 0.001f) {
                    pct = static_cast<int32_t>(exp.currentDose / exp.targetDose * 100.0f + 0.5f);
                } else {
                    // Zeitfortschritt aus phaseAgeMs + Restzeit berechnen
                    const float elapsedMs  = static_cast<float>(exp.phaseAgeMs);
                    const float remainMs   = exp.remainingTimeSeconds * 1000.0f;
                    const float totalMs    = elapsedMs + remainMs;
                    if (totalMs > 0.0f) {
                        pct = static_cast<int32_t>(elapsedMs / totalMs * 100.0f + 0.5f);
                    }
                }
                if (pct < 0)   pct = 0;
                if (pct > 100) pct = 100;
                busyProgressPercent_ = pct;
            }
            lv_bar_set_value(w.busy_bar, pct, LV_ANIM_OFF);
            if (w.busy_pct_lbl) {
                std::snprintf(dmaText_, sizeof(dmaText_), "%d%%", static_cast<int>(pct));
                lv_label_set_text(w.busy_pct_lbl, dmaText_);
            }
        }

        // SG-Detail: Soft/Hard-Restzeiten aufschlüsseln
        if (w.busy_sg_detail_lbl) {
            const bool isSg = (snapshot.modeState.activeMode == ModeId::Splitgrade ||
                                snapshot.modeState.activeMode == ModeId::BlackWhite);
            if (!isSg) {
                lv_obj_add_flag(w.busy_sg_detail_lbl, LV_OBJ_FLAG_HIDDEN);
            } else if (snapshot.modeState.activeMode == ModeId::BlackWhite) {
                lv_obj_clear_flag(w.busy_sg_detail_lbl, LV_OBJ_FLAG_HIDDEN);
                const BwModeRuntimeState& bw = snapshot.modeState.bw;
                const bool bwRunDose = (exp.controlMode == ExposureControlMode::Dose);
                if (bw.whiteLight) {
                    if (bwRunDose) {
                        std::snprintf(dmaText_, sizeof(dmaText_),
                            "BW WEISSLICHT  G%.1f  %.2f / %.2f lx\xB7s",
                            static_cast<double>(bw.grade),
                            static_cast<double>(exp.currentDose),
                            static_cast<double>(exp.targetDose));
                    } else {
                        std::snprintf(dmaText_, sizeof(dmaText_), "BW WEISSLICHT  G%.1f  %.1f s offen",
                            static_cast<double>(bw.grade),
                            static_cast<double>(exp.remainingTimeSeconds));
                    }
                } else {
                    if (bwRunDose) {
                        std::snprintf(dmaText_, sizeof(dmaText_),
                            "BW G%.1f  Soft:%.0f%%  Hard:%.0f%%  %.2f / %.2f lx\xB7s",
                            static_cast<double>(bw.grade),
                            static_cast<double>(bw.softMix * 100.0f),
                            static_cast<double>(bw.hardMix * 100.0f),
                            static_cast<double>(exp.currentDose),
                            static_cast<double>(exp.targetDose));
                    } else {
                        std::snprintf(dmaText_, sizeof(dmaText_),
                            "BW G%.1f  Soft:%.0f%%  Hard:%.0f%%  %.1f s",
                            static_cast<double>(bw.grade),
                            static_cast<double>(bw.softMix * 100.0f),
                            static_cast<double>(bw.hardMix * 100.0f),
                            static_cast<double>(exp.remainingTimeSeconds));
                    }
                }
                lv_label_set_text(w.busy_sg_detail_lbl, dmaText_);
            } else {
                lv_obj_clear_flag(w.busy_sg_detail_lbl, LV_OBJ_FLAG_HIDDEN);
                const SplitgradeModeRuntimeState& sg = snapshot.modeState.splitgrade;
                const bool isHardPhase =
                    (sg.executionState == SplitgradeExecutionState::ExposingHard) ||
                    (sg.executionState == SplitgradeExecutionState::ArmingHard);
                if (isHardPhase) {
                    std::snprintf(dmaText_, sizeof(dmaText_),
                        "SOFT: erledigt  \xB7  HARD: %.1f s offen",
                        static_cast<double>(exp.remainingTimeSeconds));
                } else {
                    std::snprintf(dmaText_, sizeof(dmaText_),
                        "SOFT: %.1f s offen  \xB7  HARD: %.1f s bereit",
                        static_cast<double>(exp.remainingTimeSeconds),
                        static_cast<double>(sg.hardTarget));
                }
                lv_label_set_text(w.busy_sg_detail_lbl, dmaText_);
            }
        }

        // Solange die Engine aktiv exponiert, darf die UI explizit PAUSE senden.
        // Resume/Abort bleiben bewusst ein separates Overlay fuer den Paused-Zustand.
        if (w.busy_run_overlay) {
            if (exp.phase == ExposurePhase::Exposing) {
                lv_obj_clear_flag(w.busy_run_overlay, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(w.busy_run_overlay, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // Pause-Overlay sichtbar nur bei ExposurePhase::Paused
        if (w.busy_pause_overlay) {
            if (exp.phase == ExposurePhase::Paused) {
                lv_obj_clear_flag(w.busy_pause_overlay, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(w.busy_pause_overlay, LV_OBJ_FLAG_HIDDEN);
            }
        }
        return;
    }

    // ---- Header (shared across all non-busy content screens) ----------------
    if (w.hdr_title) {
        lv_label_set_text_static(w.hdr_title, presenter_.getTitle(snapshot));
    }
    if (w.hdr_msg_lbl) {
        lv_label_set_text_static(w.hdr_msg_lbl, presenter_.getOverlayText(snapshot));
    }
    if (w.hdr_msg_cont) {
        lv_obj_set_style_bg_color(w.hdr_msg_cont, lv_color_hex(overlayColorHex), LV_PART_MAIN);
    }
    if (w.hdr_thermal_lbl) {
        if (snapshot.exposureState.thermalDeratingActive) {
            lv_obj_clear_flag(w.hdr_thermal_lbl, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(w.hdr_thermal_lbl, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // ---- ModeTabs (highlight active family) ---------------------------------
    // Familienvertrag: 0=PAPER, 1=MEAS, 2=PRINT, 3=SETUP (EEZ-Conditions-Vertrag A1)
    // computeWorkflowFamily() kodiert MEAS bereits (SG/BW im Measurement-Panel -> 1).
    const int32_t family = computeWorkflowFamily(snapshot);
    struct { lv_obj_t *lbl; bool active; } tabs[4] = {
        { w.tab_paper, family == 0 },
        { w.tab_meas,  family == 1 },
        { w.tab_print, family == 2 },
        { w.tab_setup, family == 3 },
    };
    for (int i = 0; i < 4; ++i) {
        if (!tabs[i].lbl) continue;
        lv_obj_t *cont = lv_obj_get_parent(tabs[i].lbl);
        if (cont) {
            lv_obj_set_style_bg_color(cont,
                tabs[i].active ? lv_color_hex(0x2A0C0Cu) : lv_color_hex(0x0B0404u),
                LV_PART_MAIN);
        }
        lv_obj_set_style_text_color(tabs[i].lbl,
            tabs[i].active ? lv_color_hex(0xF1E7E0u) : lv_color_hex(0x8A6A64u),
            LV_PART_MAIN);
    }

    // ---- Boot screen (aktiver Update-Pfad bis EEZ-Bindung aktiv ist) ----------
    // B1: EEZ-Bindungen fuer Boot-Labels sind in der assets[]-Binary kodiert
    // (boot_titleText=var147, boot_subtitleText=var148, global_overlayText=var2),
    // aber da create_screen_boot() handgeschrieben ist, hat eez_flow_tick() keine
    // Kenntnis von den LVGL-Label-Handles. Daher bleibt dieser Direktpfad aktiv.
    // Die flow::setGlobalVariable()-Aufrufe fuer Boot weiter unten sind bereits
    // die Infrastruktur fuer den kuenftigen EEZ-gefuehrten Pfad.
    if (w.boot_title_lbl) {
        lv_label_set_text_static(w.boot_title_lbl, presenter_.getTitle(snapshot));
    }
    if (w.boot_subtitle_lbl) {
        lv_label_set_text_static(w.boot_subtitle_lbl, presenter_.getSubtitle(snapshot));
    }
    if (w.boot_status_lbl) {
        lv_label_set_text_static(w.boot_status_lbl, presenter_.getOverlayText(snapshot));
    }

    // ---- PageSplitgrade (only update when screen is active, SG mode) -------
    if (currentScreenId_ == SCREEN_ID_PAGE_SPLITGRADE &&
        snapshot.modeState.activeMode == ModeId::Splitgrade) {

    const SplitgradeModeRuntimeState& sg = snapshot.modeState.splitgrade;
    const ExposureRuntimeState& exp = snapshot.exposureState;

    if (w.sg_grade_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "G %.1f", sg.grade);
        lv_label_set_text(w.sg_grade_lbl, dmaText_);
    }
    if (w.sg_soft_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "SOFT %.1f", sg.softTarget);
        lv_label_set_text(w.sg_soft_lbl, dmaText_);
    }
    if (w.sg_hard_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "HARD %.1f", sg.hardTarget);
        lv_label_set_text(w.sg_hard_lbl, dmaText_);
    }
    if (w.sg_ctrl_mode_lbl) {
        lv_label_set_text_static(w.sg_ctrl_mode_lbl,
            sg.controlMode == ExposureControlMode::Dose ? "DOSE" : "ZEIT");
    }
    if (w.sg_exec_state_lbl) {
        lv_label_set_text_static(w.sg_exec_state_lbl, presenter_.getSgHeader(snapshot));
    }
    if (w.sg_paper_lbl) {
        const char* dirtyMark = sg.parametersDirty ? " *" : "";
        const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
        const unsigned activeSlot = slotCount > 0u
            ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u : 0u;
        std::snprintf(dmaText_, sizeof(dmaText_), "P%u/%u %.14s%s",
            activeSlot, slotCount, snapshot.paperActiveSlotName, dirtyMark);
        lv_label_set_text(w.sg_paper_lbl, dmaText_);
    }
    if (w.sg_time_lbl) {
        if (exp.phase == ExposurePhase::Idle || exp.phase == ExposurePhase::Done) {
            std::snprintf(dmaText_, sizeof(dmaText_), "%.1f s", sg.softTarget + sg.hardTarget);
        } else {
            std::snprintf(dmaText_, sizeof(dmaText_), "%.1f s", exp.remainingTimeSeconds);
        }
        lv_label_set_text(w.sg_time_lbl, dmaText_);
    }
    if (w.sg_dose_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "%.2f / %.2f lx-s",
            exp.currentDose, exp.targetDose);
        lv_label_set_text(w.sg_dose_lbl, dmaText_);
    }
    if (w.sg_lux_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "%.2f lx", exp.measuredLux);
        lv_label_set_text(w.sg_lux_lbl, dmaText_);
    }
    if (w.sg_head_lbl) {
        const unsigned headPct = static_cast<unsigned>(exp.runtimeOutputLimit * 100.0f + 0.5f);
        const bool headWarn = exp.thermalDeratingActive && headPct < 100u;
        std::snprintf(dmaText_, sizeof(dmaText_), "HEAD %u%%%s", headPct,
            headWarn ? " THERM" : "");
        lv_label_set_text(w.sg_head_lbl, dmaText_);
        lv_obj_set_style_text_color(w.sg_head_lbl,
            headWarn ? lv_color_hex(0xE0A06Eu) : lv_color_hex(0xB8948Au),
            LV_PART_MAIN);
    }
    if (w.sg_context_lbl) {
        if (exp.sensorFallbackActive) {
            lv_label_set_text_static(w.sg_context_lbl, "SENSOR FALLBACK - kein Closed Loop");
        } else if (exp.faultLatched) {
            lv_label_set_text_static(w.sg_context_lbl, "FAULT - START zum Quittieren");
        } else {
            lv_label_set_text_static(w.sg_context_lbl,
                "E1=GRADE  E2=MODUS  E3=PANEL  E3-LONG=SETUP");
        }
    }

    } // end SCREEN_ID_PAGE_SPLITGRADE (SG)

    // ---- PageSplitgrade reused for BW (when BW mode active) ----------------
    if (currentScreenId_ == SCREEN_ID_PAGE_SPLITGRADE &&
        snapshot.modeState.activeMode == ModeId::BlackWhite) {

    const BwModeRuntimeState& bw  = snapshot.modeState.bw;
    const ExposureRuntimeState& exp = snapshot.exposureState;

    if (w.sg_grade_lbl) {
        if (bw.gradeEditable) {
            std::snprintf(dmaText_, sizeof(dmaText_), "G %.1f", bw.grade);
        } else {
            std::snprintf(dmaText_, sizeof(dmaText_), "G %.1f FIX", bw.grade);
        }
        lv_label_set_text(w.sg_grade_lbl, dmaText_);
    }
    if (w.sg_soft_lbl) {
        if (bw.whiteLight) {
            lv_label_set_text_static(w.sg_soft_lbl, "WEISS");
        } else {
            std::snprintf(dmaText_, sizeof(dmaText_), "SOFT %.0f%%",
                static_cast<double>(bw.softMix * 100.0f));
            lv_label_set_text(w.sg_soft_lbl, dmaText_);
        }
    }
    if (w.sg_hard_lbl) {
        if (bw.whiteLight) {
            lv_label_set_text_static(w.sg_hard_lbl, "LICHT");
        } else {
            std::snprintf(dmaText_, sizeof(dmaText_), "HARD %.0f%%",
                static_cast<double>(bw.hardMix * 100.0f));
            lv_label_set_text(w.sg_hard_lbl, dmaText_);
        }
    }
    if (w.sg_ctrl_mode_lbl) {
        lv_label_set_text_static(w.sg_ctrl_mode_lbl,
            bw.controlMode == ExposureControlMode::Dose ? "DOSE" : "ZEIT");
    }
    if (w.sg_exec_state_lbl) {
        lv_label_set_text_static(w.sg_exec_state_lbl, presenter_.getBwHeader(snapshot));
    }
    if (w.sg_paper_lbl) {
        const char* dirtyMark = bw.parametersDirty ? " *" : "";
        const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
        const unsigned activeSlot = slotCount > 0u
            ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u : 0u;
        std::snprintf(dmaText_, sizeof(dmaText_), "P%u/%u %.14s%s",
            activeSlot, slotCount, snapshot.paperActiveSlotName, dirtyMark);
        lv_label_set_text(w.sg_paper_lbl, dmaText_);
    }
    if (w.sg_time_lbl) {
        const bool bwDose = (bw.controlMode == ExposureControlMode::Dose);
        const bool isIdle = (exp.phase == ExposurePhase::Idle || exp.phase == ExposurePhase::Done);
        if (bwDose) {
            if (isIdle) {
                std::snprintf(dmaText_, sizeof(dmaText_), "%.2f lx\xB7s", bw.targetValue);
            } else {
                std::snprintf(dmaText_, sizeof(dmaText_), "%.2f / %.2f lx\xB7s",
                    exp.currentDose, exp.targetDose);
            }
        } else {
            std::snprintf(dmaText_, sizeof(dmaText_), "%.1f s",
                isIdle ? bw.targetValue : exp.remainingTimeSeconds);
        }
        lv_label_set_text(w.sg_time_lbl, dmaText_);
    }
    if (w.sg_dose_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "%.2f / %.2f lx-s",
            exp.currentDose, exp.targetDose);
        lv_label_set_text(w.sg_dose_lbl, dmaText_);
    }
    if (w.sg_lux_lbl) {
        std::snprintf(dmaText_, sizeof(dmaText_), "%.2f lx", exp.measuredLux);
        lv_label_set_text(w.sg_lux_lbl, dmaText_);
    }
    if (w.sg_head_lbl) {
        const unsigned headPct = static_cast<unsigned>(exp.runtimeOutputLimit * 100.0f + 0.5f);
        const bool headWarn = exp.thermalDeratingActive && headPct < 100u;
        std::snprintf(dmaText_, sizeof(dmaText_), "HEAD %u%%%s", headPct,
            headWarn ? " THERM" : "");
        lv_label_set_text(w.sg_head_lbl, dmaText_);
        lv_obj_set_style_text_color(w.sg_head_lbl,
            headWarn ? lv_color_hex(0xE0A06Eu) : lv_color_hex(0xB8948Au),
            LV_PART_MAIN);
    }
    if (w.sg_context_lbl) {
        if (exp.sensorFallbackActive) {
            lv_label_set_text_static(w.sg_context_lbl, "SENSOR FALLBACK - kein Closed Loop");
        } else if (exp.faultLatched) {
            lv_label_set_text_static(w.sg_context_lbl, "FAULT - START zum Quittieren");
        } else {
            lv_label_set_text_static(w.sg_context_lbl,
                "E1=ZIEL  E2=MODUS  E3=PANEL  E3-LONG=SETUP");
        }
    }

    } // end BW on SCREEN_ID_PAGE_SPLITGRADE

    // ---- PagePaperWorkspace (only update when screen is active) -------------
    // B2: create_screen_page_paper_workspace() ist handgeschrieben → tick stub →
    // EEZ hat keine Kenntnis von den Paper-Widget-Handles. Aktiver Pfad:
    // Direktzugriff via g_duka_widgets.paper_* (dieser Block).
    // EEZ-Datenschicht vollstaendig: 33 Flow-Globals (Indices 17-49) geschrieben
    // in updateSnapshot(). Fehlende EEZ-Vars fuer vollen CAL-Pfad:
    // PAPER_STEP_WHITE, PAPER_STEP_BLACK + EEZ-Export/Merge gemaess 13.4.
    // Details: Abschnitt 13.8 der EEZ-Doku.
    if (currentScreenId_ == SCREEN_ID_PAGE_PAPER_WORKSPACE) {
        const PaperModeRuntimeState& paper = snapshot.modeState.paper;
        const bool isCal = (paper.panel == PaperWorkspacePanel::Calibrate);

        // Panel info strip
        if (w.paper_panel_lbl) {
            const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
            const unsigned selectedSlot = (slotCount > 0u)
                ? static_cast<unsigned>(paper.selectedSlot) + 1u : 0u;
            if (isCal) {
                const char* stateLabel = paper.editingActive ? "EDIT"
                    : (paper.parametersDirty ? "DIRTY" : "NAV");
                std::snprintf(dmaText_, sizeof(dmaText_), "CAL  Slot %u/%u  %s",
                              selectedSlot, slotCount, stateLabel);
            } else {
                const unsigned activeSlot = (slotCount > 0u)
                    ? static_cast<unsigned>(snapshot.paperActiveSlot) + 1u : 0u;
                std::snprintf(dmaText_, sizeof(dmaText_), "SELECT  Slot %u/%u  Aktiv: %u",
                              selectedSlot, slotCount, activeSlot);
            }
            lv_label_set_text(w.paper_panel_lbl, dmaText_);
        }

        // Item/slot labels (string literals, index must match PaperCalibrationItem enum order)
        static const char* const kCalItemLabels[] = {
            "GRADE MODE",        // GradeMode      = 0
            "FIXED GRADE",       // FixedGradeValue = 1
            "ISO MATH",          // IsoMath         = 2
            "ISO-P",             // IsoP            = 3
            "ISO-R",             // IsoR            = 4
            "K-BW",              // KBw             = 5
            "K-SOFT",            // KSoft           = 6
            "K-HARD",            // KHard           = 7
            "CALIBRATED",        // Calibrated      = 8
            "WEISSPUNKT (N)",    // StepWhite       = 9
            "SCHWARZPUNKT (M)",  // StepBlack       = 10
            "APPLY",             // Apply           = 11
            "DISCARD",           // Discard         = 12
        };
        constexpr int kCalItemLabelCount =
            static_cast<int>(sizeof(kCalItemLabels) / sizeof(kCalItemLabels[0]));

        const int itemCount = static_cast<int>(paper.itemCount);
        const int selectedItemIdx = static_cast<int>(paper.selectedItem);
        const unsigned slotCount = static_cast<unsigned>(snapshot.paperSlotCount);
        const int selectedSlotIdx = static_cast<int>(paper.selectedSlot);

        // Compute 7-row sliding window (center on selected item/slot)
        int windowStart;
        int listCount;
        if (isCal) {
            listCount = itemCount;
            windowStart = selectedItemIdx - 3;
        } else {
            listCount = static_cast<int>(slotCount);
            windowStart = selectedSlotIdx - 3;
        }
        if (windowStart < 0) windowStart = 0;
        if (windowStart + 7 > listCount) windowStart = listCount - 7;
        if (windowStart < 0) windowStart = 0;

        for (int row = 0; row < 7; ++row) {
            lv_obj_t* cont = w.paper_cal_row_cont[row];
            lv_obj_t* lbl  = w.paper_cal_row_lbl[row];
            lv_obj_t* val  = w.paper_cal_row_val[row];
            if (!cont) continue;

            const int listIdx = windowStart + row;
            const bool visible = (listIdx < listCount);
            if (!visible) {
                lv_obj_add_flag(cont, LV_OBJ_FLAG_HIDDEN);
                continue;
            }
            lv_obj_clear_flag(cont, LV_OBJ_FLAG_HIDDEN);

            if (isCal) {
                // ---- CAL panel: calibration items ----
                const bool isSelected = (listIdx == selectedItemIdx);
                const bool isEditing  = isSelected && paper.editingActive;

                lv_obj_set_style_bg_color(cont, lv_color_hex(
                    isEditing  ? 0x0A1A08u :
                    isSelected ? 0x1A0A0Au : 0x0B0404u), LV_PART_MAIN);

                if (lbl) {
                    const char* itemLabel = (listIdx < kCalItemLabelCount)
                        ? kCalItemLabels[listIdx] : "?";
                    lv_label_set_text_static(lbl, itemLabel);
                    lv_obj_set_style_text_color(lbl,
                        lv_color_hex(isSelected ? 0xF5E8E0u : 0x9A8080u), LV_PART_MAIN);
                }

                if (val) {
                    char valBuf[24] = {};
                    const auto& sp = paper.stagedProfile;
                    const PaperCalibrationItem item =
                        static_cast<PaperCalibrationItem>(listIdx);
                    switch (item) {
                        case PaperCalibrationItem::GradeMode:
                            std::snprintf(valBuf, sizeof(valBuf), "%s",
                                sp.gradeMode == PaperGradeMode::FixedGrade ? "FG" : "MG");
                            break;
                        case PaperCalibrationItem::FixedGradeValue:
                            std::snprintf(valBuf, sizeof(valBuf), "%.1f",
                                static_cast<double>(sp.fixedGradeValue));
                            break;
                        case PaperCalibrationItem::IsoMath:
                            std::snprintf(valBuf, sizeof(valBuf), "%s",
                                sp.useIsoMath ? "EIN" : "AUS");
                            break;
                        case PaperCalibrationItem::IsoP:
                            std::snprintf(valBuf, sizeof(valBuf), "%.0f",
                                static_cast<double>(sp.isoP));
                            break;
                        case PaperCalibrationItem::IsoR:
                            std::snprintf(valBuf, sizeof(valBuf), "%.0f",
                                static_cast<double>(sp.isoR));
                            break;
                        case PaperCalibrationItem::KBw:
                            std::snprintf(valBuf, sizeof(valBuf), "%.3f",
                                static_cast<double>(sp.kBw));
                            break;
                        case PaperCalibrationItem::KSoft:
                            std::snprintf(valBuf, sizeof(valBuf), "%.2f",
                                static_cast<double>(sp.kSoft));
                            break;
                        case PaperCalibrationItem::KHard:
                            std::snprintf(valBuf, sizeof(valBuf), "%.2f",
                                static_cast<double>(sp.kHard));
                            break;
                        case PaperCalibrationItem::Calibrated:
                            std::snprintf(valBuf, sizeof(valBuf), "%s",
                                sp.calibrated ? "CAL" : "RAW");
                            break;
                        case PaperCalibrationItem::StepWhite:
                            std::snprintf(valBuf, sizeof(valBuf), "N=%u",
                                static_cast<unsigned>(paper.stepWhite));
                            break;
                        case PaperCalibrationItem::StepBlack:
                            std::snprintf(valBuf, sizeof(valBuf), "M=%u",
                                static_cast<unsigned>(paper.stepBlack));
                            break;
                        case PaperCalibrationItem::Apply:
                            std::snprintf(valBuf, sizeof(valBuf), "-> SD");
                            break;
                        case PaperCalibrationItem::Discard:
                            std::snprintf(valBuf, sizeof(valBuf), "verwerfen");
                            break;
                        default:
                            break;
                    }
                    lv_label_set_text(val, valBuf);
                    lv_obj_set_style_text_color(val, lv_color_hex(
                        isEditing  ? 0x88E060u :
                        isSelected ? 0xF0C890u : 0x7A6868u), LV_PART_MAIN);
                }

            } else {
                // ---- SELECT panel: paper slots ----
                const bool isSelected = (listIdx == selectedSlotIdx);
                const bool isActive   = (static_cast<unsigned>(listIdx) ==
                                         static_cast<unsigned>(snapshot.paperActiveSlot));

                lv_obj_set_style_bg_color(cont, lv_color_hex(
                    isSelected ? 0x1A0A0Au : 0x0B0404u), LV_PART_MAIN);

                if (lbl) {
                    std::snprintf(dmaText_, sizeof(dmaText_), "Slot %d%s",
                                  listIdx + 1, isActive ? "  [AKT]" : "");
                    lv_label_set_text(lbl, dmaText_);
                    lv_obj_set_style_text_color(lbl, lv_color_hex(
                        isActive   ? 0xF0C890u :
                        isSelected ? 0xF5E8E0u : 0x9A8080u), LV_PART_MAIN);
                }

                if (val) {
                    const PaperSlotUiSummary* slotSum = (listIdx < (int)snapshot.paperSlotCount)
                        ? &snapshot.paperSlotSummaries[listIdx] : nullptr;
                    if (slotSum != nullptr && slotSum->available) {
                        const char* calStr = slotSum->calibrated ? "CAL" : "RAW";
                        const char* gmStr  = (slotSum->gradeMode == PaperGradeMode::FixedGrade)
                            ? "FG" : "MG";
                        std::snprintf(dmaText_, sizeof(dmaText_), "%s/%s  %.14s",
                                      calStr, gmStr, slotSum->name);
                    } else {
                        std::snprintf(dmaText_, sizeof(dmaText_), "---");
                    }
                    lv_label_set_text(val, dmaText_);
                    lv_obj_set_style_text_color(val, lv_color_hex(
                        isSelected ? 0xB8948Au : 0x7A6868u), LV_PART_MAIN);
                }
            }
        } // for row

        // Hint bar
        if (w.paper_hint_lbl) {
            if (isCal) {
                lv_label_set_text_static(w.paper_hint_lbl,
                    paper.editingActive
                        ? "E1=WERT  E1-PRESS=FERTIG  E3-LONG=SELECT"
                        : "E1=ITEM  E1-PRESS=EDIT  E3-LONG=SELECT");
            } else {
                lv_label_set_text_static(w.paper_hint_lbl,
                    "E1=SLOT  E1-PRESS=AKTIVIEREN  E3-LONG=CAL");
            }
        }

        // Panel-Switcher-Chips: aktiven hervorheben
        if (w.paper_chip_select) {
            lv_obj_set_style_text_color(w.paper_chip_select,
                !isCal ? lv_color_hex(0xF1E7E0u) : lv_color_hex(0x8A6A64u), LV_PART_MAIN);
        }
        if (w.paper_chip_cal) {
            lv_obj_set_style_text_color(w.paper_chip_cal,
                isCal ? lv_color_hex(0xF1E7E0u) : lv_color_hex(0x8A6A64u), LV_PART_MAIN);
        }

    } // end SCREEN_ID_PAGE_PAPER_WORKSPACE

    // ---- PageSetup (only update when screen is active) ----------------------
    // B4: create_screen_page_setup() ist handgeschrieben -> tick stub -> EEZ hat
    // keine Kenntnis von den Setup-Widget-Handles. Aktiver Pfad: Direktzugriff
    // via g_duka_widgets.setup_* (dieser Block, mit Lazy-Loading-Guard).
    // Action-Buttons haben LV_OBJ_FLAG_CLICKABLE, aber keinen LVGL-Event-
    // Callback -> Touch auf Apply/Discard/SafetyDefaults ist derzeit inaktiv.
    // Firmwarepfad: SetupWorkflow verarbeitet Enc3/Enc4 + E1-PRESS exklusiv.
    // EEZ-Datenschicht: 19 Flow-Globals (71-89) geschrieben; SETUP_HEAD_TIMING_
    // DIAGNOSTICS_ENABLED fehlt in vars.h (benoetigt EEZ-Projektaenderung).
    // Details: Abschnitt 13.9 der EEZ-Doku.
    if (currentScreenId_ == SCREEN_ID_PAGE_SETUP) {
        const SetupModeRuntimeState& setup = snapshot.modeState.setup;
        const bool thermalActive = snapshot.exposureState.thermalDeratingActive;
        const float outputLimit  = snapshot.exposureState.runtimeOutputLimit;

        // Lazy-Loading guard: skip all LVGL widget updates when the setup state
        // and thermal fields are identical to the last rendered frame.
        if (setup == setupStateCache_ &&
            thermalActive == setupThermalCache_ &&
            outputLimit   == setupOutputCache_) {
            // Nothing changed — no LVGL work needed.
        } else {

        setupStateCache_   = setup;
        setupThermalCache_ = thermalActive;
        setupOutputCache_  = outputLimit;

        const int selectedIdx = static_cast<int>(setup.selectedItem);
        const int totalItems  = static_cast<int>(SetupMenuItem::Count);

        static const char* const kSetupItemLabels[] = {
            "SOUND MODUS",      // SoundMode = 0
            "LAUTSTAERKE",      // SoundVolume
            "VIBRATION",        // Vibration
            "HEAD MAX %",       // MaxHeadBrightness
            "HEAD DIAG",        // HeadTimingDiagnostics
            "THERM START C",    // ThermalDeratingStart
            "THERM STOP C",     // ThermalHardStop
            "UEBERNEHMEN",      // Apply
            "VERWERFEN",        // Discard
            "WERKSSTANDARD",    // SafetyDefaults
        };

        // Sliding window: 6 Zeilen, zentriert auf selectedItem
        int windowStart = selectedIdx - 3;
        if (windowStart < 0) windowStart = 0;
        if (windowStart + 6 > totalItems) windowStart = totalItems - 6;
        if (windowStart < 0) windowStart = 0;

        for (int row = 0; row < 6; ++row) {
            lv_obj_t* cont = w.setup_item_row_cont[row];
            lv_obj_t* lbl  = w.setup_item_lbl[row];
            lv_obj_t* val  = w.setup_item_val[row];
            if (!cont) continue;

            const int listIdx = windowStart + row;
            if (listIdx >= totalItems) {
                lv_obj_add_flag(cont, LV_OBJ_FLAG_HIDDEN);
                continue;
            }
            lv_obj_clear_flag(cont, LV_OBJ_FLAG_HIDDEN);

            const bool isSelected = (listIdx == selectedIdx);
            const bool isEditing  = isSelected && setup.editingActive;

            lv_obj_set_style_bg_color(cont, lv_color_hex(
                isEditing  ? 0x0A1A08u :
                isSelected ? 0x1A0A0Au : 0x0B0404u), LV_PART_MAIN);

            if (lbl) {
                lv_label_set_text_static(lbl, kSetupItemLabels[listIdx]);
                lv_obj_set_style_text_color(lbl,
                    lv_color_hex(isSelected ? 0xF5E8E0u : 0x9A8080u), LV_PART_MAIN);
            }

            if (val) {
                char valBuf[20] = {};
                const auto& ss = setup.stagedSettings;
                switch (static_cast<SetupMenuItem>(listIdx)) {
                    case SetupMenuItem::SoundMode:
                        switch (ss.soundMode) {
                            case SoundFeedbackMode::Off:
                                std::snprintf(valBuf, sizeof(valBuf), "AUS"); break;
                            case SoundFeedbackMode::FaultsOnly:
                                std::snprintf(valBuf, sizeof(valBuf), "FEHLER"); break;
                            default:
                                std::snprintf(valBuf, sizeof(valBuf), "NORMAL"); break;
                        }
                        break;
                    case SetupMenuItem::SoundVolume:
                        switch (ss.soundVolume) {
                            case SoundVolumeLevel::Low:
                                std::snprintf(valBuf, sizeof(valBuf), "LEISE"); break;
                            case SoundVolumeLevel::High:
                                std::snprintf(valBuf, sizeof(valBuf), "LAUT"); break;
                            default:
                                std::snprintf(valBuf, sizeof(valBuf), "MITTEL"); break;
                        }
                        break;
                    case SetupMenuItem::Vibration:
                        std::snprintf(valBuf, sizeof(valBuf),
                            ss.vibrationEnabled ? "EIN" : "AUS"); break;
                    case SetupMenuItem::MaxHeadBrightness:
                        std::snprintf(valBuf, sizeof(valBuf), "%u%%",
                            static_cast<unsigned>(ss.maxHeadBrightnessPercent)); break;
                    case SetupMenuItem::HeadTimingDiagnostics:
                        std::snprintf(valBuf, sizeof(valBuf),
                            setup.headTimingDiagnosticsEnabled ? "EIN" : "AUS"); break;
                    case SetupMenuItem::ThermalDeratingStart:
                        std::snprintf(valBuf, sizeof(valBuf), "%.0f",
                            static_cast<double>(ss.thermalProtection.deratingStartCelsius)); break;
                    case SetupMenuItem::ThermalHardStop:
                        std::snprintf(valBuf, sizeof(valBuf), "%.0f",
                            static_cast<double>(ss.thermalProtection.hardStopCelsius)); break;
                    case SetupMenuItem::Apply:
                        std::snprintf(valBuf, sizeof(valBuf), "-> SD"); break;
                    case SetupMenuItem::Discard:
                        std::snprintf(valBuf, sizeof(valBuf), "rueckgaengig"); break;
                    case SetupMenuItem::SafetyDefaults:
                        std::snprintf(valBuf, sizeof(valBuf), "Werksstandard"); break;
                    default: break;
                }
                lv_label_set_text(val, valBuf);
                lv_obj_set_style_text_color(val, lv_color_hex(
                    isEditing  ? 0x88E060u :
                    isSelected ? 0xF0C890u : 0x7A6868u), LV_PART_MAIN);
            }
        } // for row

        // Edit-Block: Name + Wert des selektierten Items
        const SetupMenuItem selItem = setup.selectedItem;
        const bool isMaxHead = (selItem == SetupMenuItem::MaxHeadBrightness);

        if (w.setup_edit_name_lbl && selectedIdx < totalItems) {
            lv_label_set_text_static(w.setup_edit_name_lbl, kSetupItemLabels[selectedIdx]);
        }
        if (w.setup_edit_val_lbl) {
            char editValBuf[24] = {};
            const auto& ss = setup.stagedSettings;
            switch (selItem) {
                case SetupMenuItem::SoundMode:
                    switch (ss.soundMode) {
                        case SoundFeedbackMode::Off:
                            std::snprintf(editValBuf, sizeof(editValBuf), "AUS"); break;
                        case SoundFeedbackMode::FaultsOnly:
                            std::snprintf(editValBuf, sizeof(editValBuf), "FEHLER"); break;
                        default:
                            std::snprintf(editValBuf, sizeof(editValBuf), "NORMAL"); break;
                    }
                    break;
                case SetupMenuItem::SoundVolume:
                    switch (ss.soundVolume) {
                        case SoundVolumeLevel::Low:
                            std::snprintf(editValBuf, sizeof(editValBuf), "LEISE"); break;
                        case SoundVolumeLevel::High:
                            std::snprintf(editValBuf, sizeof(editValBuf), "LAUT"); break;
                        default:
                            std::snprintf(editValBuf, sizeof(editValBuf), "MITTEL"); break;
                    }
                    break;
                case SetupMenuItem::Vibration:
                    std::snprintf(editValBuf, sizeof(editValBuf),
                        ss.vibrationEnabled ? "EIN" : "AUS"); break;
                case SetupMenuItem::MaxHeadBrightness:
                    std::snprintf(editValBuf, sizeof(editValBuf), "%u %%",
                        static_cast<unsigned>(ss.maxHeadBrightnessPercent)); break;
                case SetupMenuItem::HeadTimingDiagnostics:
                    std::snprintf(editValBuf, sizeof(editValBuf),
                        setup.headTimingDiagnosticsEnabled ? "EIN" : "AUS"); break;
                case SetupMenuItem::ThermalDeratingStart:
                    std::snprintf(editValBuf, sizeof(editValBuf), "%.0f deg",
                        static_cast<double>(ss.thermalProtection.deratingStartCelsius)); break;
                case SetupMenuItem::ThermalHardStop:
                    std::snprintf(editValBuf, sizeof(editValBuf), "%.0f deg",
                        static_cast<double>(ss.thermalProtection.hardStopCelsius)); break;
                case SetupMenuItem::Apply:
                    std::snprintf(editValBuf, sizeof(editValBuf), "-> SD"); break;
                case SetupMenuItem::Discard:
                    std::snprintf(editValBuf, sizeof(editValBuf), "rueckgaengig"); break;
                case SetupMenuItem::SafetyDefaults:
                    std::snprintf(editValBuf, sizeof(editValBuf), "Werksstandard"); break;
                default: break;
            }
            lv_label_set_text(w.setup_edit_val_lbl, editValBuf);
        }

        // CAP-Label: nur bei MaxHeadBrightness sichtbar
        if (w.setup_edit_cap_lbl) {
            if (isMaxHead) {
                lv_obj_clear_flag(w.setup_edit_cap_lbl, LV_OBJ_FLAG_HIDDEN);
                std::snprintf(dmaText_, sizeof(dmaText_), "CAP %u%%",
                    static_cast<unsigned>(setup.stagedSettings.maxHeadBrightnessPercent));
                lv_label_set_text(w.setup_edit_cap_lbl, dmaText_);
            } else {
                lv_obj_add_flag(w.setup_edit_cap_lbl, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // LIVE-Label: nur wenn thermisches Derating aktiv
        if (w.setup_edit_live_lbl) {
            if (snapshot.exposureState.thermalDeratingActive) {
                lv_obj_clear_flag(w.setup_edit_live_lbl, LV_OBJ_FLAG_HIDDEN);
                const unsigned livePct = static_cast<unsigned>(
                    snapshot.exposureState.runtimeOutputLimit * 100.0f + 0.5f);
                std::snprintf(dmaText_, sizeof(dmaText_), "LIVE %u%%", livePct);
                lv_label_set_text(w.setup_edit_live_lbl, dmaText_);
            } else {
                lv_obj_add_flag(w.setup_edit_live_lbl, LV_OBJ_FLAG_HIDDEN);
            }
        }

        // Range-Hinweis
        if (w.setup_edit_range_lbl) {
            const char* rangeText = "";
            if      (selItem == SetupMenuItem::MaxHeadBrightness)   rangeText = "10 - 100 %";
            else if (selItem == SetupMenuItem::ThermalDeratingStart) rangeText = "35 - 70 C";
            else if (selItem == SetupMenuItem::ThermalHardStop)      rangeText = "45 - 80 C";
            else if (selItem == SetupMenuItem::SoundVolume)          rangeText = "LEISE / MITTEL / LAUT";
            lv_label_set_text_static(w.setup_edit_range_lbl, rangeText);
        }

        // Aktions-Buttons: sichtbar wenn editingActive
        if (w.setup_btn_apply) {
            if (setup.editingActive) lv_obj_clear_flag(w.setup_btn_apply, LV_OBJ_FLAG_HIDDEN);
            else                      lv_obj_add_flag(w.setup_btn_apply,  LV_OBJ_FLAG_HIDDEN);
        }
        if (w.setup_btn_discard) {
            if (setup.editingActive) lv_obj_clear_flag(w.setup_btn_discard, LV_OBJ_FLAG_HIDDEN);
            else                      lv_obj_add_flag(w.setup_btn_discard,  LV_OBJ_FLAG_HIDDEN);
        }
        if (w.setup_btn_safety) {
            if (setup.parametersDirty) lv_obj_clear_flag(w.setup_btn_safety, LV_OBJ_FLAG_HIDDEN);
            else                        lv_obj_add_flag(w.setup_btn_safety,  LV_OBJ_FLAG_HIDDEN);
        }

        // Encoder-Hinweis
        if (w.setup_hint_lbl) {
            lv_label_set_text_static(w.setup_hint_lbl,
                setup.editingActive
                    ? "E1=WERT  E1-PRESS=FERTIG  E3-LONG=BEENDEN"
                    : "E1=WERT  E3=ITEM  E3-LONG=BEENDEN");
        }

        } // end lazy-loading else
    } // end SCREEN_ID_PAGE_SETUP

    // ---- PageMeasurement (only update when screen is active) ----------------
    // B5-BLOCKIERT (Prompt B5, PageMeasurement): create_screen_page_measurement()
    // ist handgeschrieben → tick_screen_page_measurement() ist Stub → EEZ-Runtime
    // kann Flow-Vars 105-122 nicht in Widget-Schreibvorgaenge uebersetzen.
    // Aktiver Pfad (direkt): 7 Label-Handles via presenter_.getPageMeas*() +
    // 22 Histogramm-Handles via lv_obj_set_y/lv_obj_set_height().
    // A5 abgeschlossen: kein snprintf() fuer Measurement-Text in LvglUi.cpp.
    // EEZ-Datenschicht vollstaendig: 18 Flow-Globals (105-122) alle in
    // updateSnapshot() geschrieben. Fehlende EEZ-Vars fuer formatierten Text:
    // keine MEAS_*_TEXT-Vars — intentional, da Presenter-Muster traegt.
    // Histogramm-Geometrie nicht durch EEZ-Text-Bindung abdeckbar.
    // latestZoneIndex-Markierung: direkt implementiert — aktive Spalte C_WARN
    // (0xE0A06E), restliche C_ACCENT_PROGRESS (0xC05030), Guard sampleCount>0.
    // Kein Touch-Callback (actions.h leer). Details: Abschnitt 13.10.
    if (currentScreenId_ == SCREEN_ID_PAGE_MEASUREMENT) {
        const MeasurementRuntimeStatus& meas = snapshot.measurementStatus;

        // Quellen-Labels (links) — Text aus Presenter/Formatter
        if (w.meas_local_lbl) {
            lv_label_set_text_static(w.meas_local_lbl, presenter_.getPageMeasLocalLux(snapshot));
        }
        if (w.meas_wireless_lbl) {
            lv_label_set_text_static(w.meas_wireless_lbl, presenter_.getPageMeasWirelessLux(snapshot));
        }
        if (w.meas_source_chip_lbl) {
            lv_label_set_text_static(w.meas_source_chip_lbl, presenter_.getPageMeasSourceChip(snapshot));
        }

        // Haupt-Messwert (rechts) — Text aus Presenter/Formatter
        if (w.meas_lux_main_lbl) {
            lv_label_set_text_static(w.meas_lux_main_lbl, presenter_.getPageMeasLuxMain(snapshot));
        }
        if (w.meas_lux_age_lbl) {
            lv_label_set_text_static(w.meas_lux_age_lbl, presenter_.getPageMeasLuxAge(snapshot));
        }

        // Referenz-Block — Text aus Presenter/Formatter
        if (w.meas_ref_lux_lbl) {
            lv_label_set_text_static(w.meas_ref_lux_lbl, presenter_.getPageMeasRefLux(snapshot));
        }
        if (w.meas_ev_diff_lbl) {
            lv_label_set_text_static(w.meas_ev_diff_lbl, presenter_.getPageMeasEvDiff(snapshot));
        }

        // Histogramm: 11 Balken proportional zur Häufigkeit (wachsen von unten).
        // latestZoneIndex-Spalte wird mit C_WARN (0xE0A06E) hervorgehoben,
        // alle anderen Spalten mit C_ACCENT_PROGRESS (0xC05030).
        // Markierung nur wenn sampleCount > 0, damit Zone-0 bei leerer
        // Session nicht faelschlich hervorgehoben wird.
        {
            const auto& hist = meas.session.zoneHistogram;
            const uint8_t latestZone = meas.session.latestSample.zoneIndex;
            const bool hasAny = (meas.session.sampleCount > 0u);
            uint8_t maxCount = 1u;
            for (size_t i = 0u; i < kMeasurementHistogramBucketCount; ++i) {
                if (hist[i] > maxCount) maxCount = hist[i];
            }
            for (size_t i = 0u; i < kMeasurementHistogramBucketCount; ++i) {
                lv_obj_t* fill = w.meas_hist_fill[i];
                if (!fill) continue;
                const int32_t fill_h = static_cast<int32_t>((hist[i] * 44u) / maxCount);
                lv_obj_set_y(fill, 44 - fill_h);
                lv_obj_set_height(fill, fill_h);
                const bool isLatest = hasAny && (i < kMeasurementHistogramBucketCount)
                                      && (static_cast<uint8_t>(i) == latestZone);
                lv_obj_set_style_bg_color(fill,
                    isLatest ? lv_color_hex(0xE0A06Eu)   // C_WARN: aktive Zone
                             : lv_color_hex(0xC05030u),  // C_ACCENT_PROGRESS: Histogramm-Fuell
                    LV_PART_MAIN);
            }
        }

        // Session-Kontext — Text aus Presenter/Formatter
        if (w.meas_session_lbl) {
            lv_label_set_text_static(w.meas_session_lbl, presenter_.getPageMeasSession(snapshot));
        }

    } // end SCREEN_ID_PAGE_MEASUREMENT

    // ---- PageWirelessRemote (only update when screen is active) -------------
    // B6-GEPARKT (Prompt B6, PageWirelessRemote): computeTargetScreen() gibt
    // niemals SCREEN_ID_PAGE_WIRELESS_REMOTE zurueck — Seite ist runtime-seitig
    // unerreichbar. Dieser Block ist toter Code bis der Runtime-Hook
    // implementiert wird (Abschnitt 13.11: bool showWirelessDiag in Snapshot,
    // SETUP-Submodus, Enc3-Back als Rueckkehrpfad).
    // Direktpfad ist vollstaendig (16 Handles, snprintf-basiert) — aber
    // Presenter-Migration fehlt noch (kein A5-Aequivalent fuer Remote-Texte).
    if (currentScreenId_ == SCREEN_ID_PAGE_WIRELESS_REMOTE) {
        const EspLinkRuntimeStatus& esp   = snapshot.espLinkStatus;
        const WirelessGatewayStatus& wl   = esp.wireless;
        const EspDiagnosticStatus& diag   = esp.diagnostic;
        const EspTxQueueStatus& txq       = esp.txQueue;
        const RemoteCommandTrackerStatus& rmt = snapshot.remoteCommandTrackerStatus;

        // LINK-CHIP: Textfarbe je nach Gesundheit
        if (w.remote_link_chip_lbl) {
            const char* healthText = "ESP ----";
            uint32_t healthColor = 0xB8948Au; // C_TEXT_SEC
            switch (esp.health) {
                case EspLinkHealth::Online:
                    healthText = "ESP ONLINE"; healthColor = 0x40C060u; break;
                case EspLinkHealth::Stale:
                    healthText = "ESP STALE";  healthColor = 0xE0A06Eu; break;
                case EspLinkHealth::Lost:
                    healthText = "ESP LOST";   healthColor = 0xE05050u; break;
                default: break;
            }
            lv_label_set_text_static(w.remote_link_chip_lbl, healthText);
            lv_obj_set_style_text_color(w.remote_link_chip_lbl,
                lv_color_hex(healthColor), LV_PART_MAIN);
        }

        // Rx/Tx-Alter
        if (w.remote_rx_tx_lbl) {
            std::snprintf(dmaText_, sizeof(dmaText_), "RX %lums  TX %lums",
                static_cast<unsigned long>(esp.lastRxAgeMs),
                static_cast<unsigned long>(esp.lastTxAgeMs));
            lv_label_set_text(w.remote_rx_tx_lbl, dmaText_);
        }

        // Remote-Uptime
        if (w.remote_uptime_lbl) {
            const uint32_t upSec = esp.remoteUptimeMs / 1000u;
            std::snprintf(dmaText_, sizeof(dmaText_), "LAUFZEIT %lus",
                static_cast<unsigned long>(upSec));
            lv_label_set_text(w.remote_uptime_lbl, dmaText_);
        }

        // PEER-CHIP: Wireless-Terminal-Status
        if (w.remote_peer_chip_lbl) {
            const char* peerText = "PEER ----";
            switch (wl.peerState) {
                case protocol::WirelessPeerState::Online:    peerText = "PEER ONLINE";   break;
                case protocol::WirelessPeerState::Measuring: peerText = "PEER MESSUNG";  break;
                case protocol::WirelessPeerState::Fault:     peerText = "PEER FEHLER";   break;
                case protocol::WirelessPeerState::Offline:   peerText = "PEER OFFLINE";  break;
                default: break;
            }
            lv_label_set_text_static(w.remote_peer_chip_lbl, peerText);
        }

        // Akku + Alter
        if (w.remote_battery_lbl) {
            std::snprintf(dmaText_, sizeof(dmaText_), "Akku %u%%  Alter %lums",
                static_cast<unsigned>(wl.batteryPercent),
                static_cast<unsigned long>(wl.lastSeenAgeMs));
            lv_label_set_text(w.remote_battery_lbl, dmaText_);
        }

        // Letzter Lux + Sequenz
        if (w.remote_peer_lux_lbl) {
            std::snprintf(dmaText_, sizeof(dmaText_), "%.2f lx  #%lu",
                static_cast<double>(wl.lastLux),
                static_cast<unsigned long>(wl.measurementSequence));
            lv_label_set_text(w.remote_peer_lux_lbl, dmaText_);
        }

        // Diagnose
        if (w.remote_diag_lbl) {
            std::snprintf(dmaText_, sizeof(dmaText_), "DIAG %u:%u  n=%lu",
                static_cast<unsigned>(diag.code),
                static_cast<unsigned>(diag.detail),
                static_cast<unsigned long>(diag.counter));
            lv_label_set_text(w.remote_diag_lbl, dmaText_);
        }

        // TxQueue-Statistik
        if (w.remote_txqueue_lbl) {
            std::snprintf(dmaText_, sizeof(dmaText_),
                "TXQ pend=%u  dropR=%lu  dropC=%lu",
                static_cast<unsigned>(txq.pendingFrameCount),
                static_cast<unsigned long>(txq.droppedRenderCount),
                static_cast<unsigned long>(txq.droppedCommandCount));
            lv_label_set_text(w.remote_txqueue_lbl, dmaText_);
        }

        // RMT-Block
        if (w.remote_rmt_lbl) {
            std::snprintf(dmaText_, sizeof(dmaText_),
                "RMT fly=%u  rtr=%lu  to=%lu  sat=%lu",
                static_cast<unsigned>(rmt.inFlightCount),
                static_cast<unsigned long>(rmt.retryCount),
                static_cast<unsigned long>(rmt.timeoutCount),
                static_cast<unsigned long>(rmt.saturationCount));
            lv_label_set_text(w.remote_rmt_lbl, dmaText_);
        }

    } // end SCREEN_ID_PAGE_WIRELESS_REMOTE
}

void LvglUi::readTouchImpl(lv_indev_data_t* data) const {
if (!initialized_) {
data->state = LV_INDEV_STATE_RELEASED;
return;
}
const auto& ts = touchSampler_.state();
if (!ts.active) {
data->state = LV_INDEV_STATE_RELEASED;
return;
}
int16_t rawX = ts.rawX;
int16_t rawY = ts.rawY;
if (touchCalibration_.swapAxes) { const int16_t temp = rawX; rawX = rawY; rawY = temp; }
data->state = LV_INDEV_STATE_PRESSED;
data->point.x = mapTouchAxis(rawX, touchCalibration_.rawMinX, touchCalibration_.rawMaxX,
                             static_cast<uint16_t>(displayDriver_.hor_res - 1), touchCalibration_.invertX);
data->point.y = mapTouchAxis(rawY, touchCalibration_.rawMinY, touchCalibration_.rawMaxY,
                             static_cast<uint16_t>(displayDriver_.ver_res - 1), touchCalibration_.invertY);
}

FLASHMEM void LvglUi::updateSnapshot(const SystemSnapshot& snapshot) {
    if (!initialized_) return;

    // Screen switching
    const int16_t targetScreen = computeTargetScreen(snapshot);
    if (targetScreen != currentScreenId_) {
        eez_flow_set_screen(targetScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0);
        currentScreenId_ = targetScreen;
    }

    using namespace eez;

    // Overlay color bleibt nur Runtime-Fallback. Die Werte muessen trotzdem der
    // EEZ-Palette folgen, damit handgeschriebene Screens dieselbe Safety-Sprache
    // sprechen wie spaetere Designer-Bindings.
    uint32_t overlayColorHex = kOverlayColorNormal;
    const InputModalState modalState = static_cast<InputModalState>(snapshot.inputModalStateCode);
    if (modalState == InputModalState::WorkflowFault) {
        overlayColorHex = kOverlayColorWorkflowFault;
    } else if (modalState == InputModalState::WorkflowConfirm) {
        overlayColorHex = kOverlayColorWorkflowConfirm;
    } else if (modalState == InputModalState::WorkflowWait) {
        overlayColorHex = kOverlayColorWorkflowWait;
    } else if (snapshot.modeState.activeMode == ModeId::Setup && snapshot.modeState.setup.persistFailed) {
        overlayColorHex = kOverlayColorWorkflowFault;
    } else if (snapshot.modeState.activeMode == ModeId::Setup && snapshot.modeState.setup.parametersDirty) {
        overlayColorHex = kOverlayColorWorkflowConfirm;
    } else if (snapshot.exposureState.sensorFallbackActive) {
        overlayColorHex = kOverlayColorSensorFallback;
    } else if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
               snapshot.modeState.splitgrade.executionState == SplitgradeExecutionState::Completed) {
        overlayColorHex = kOverlayColorSgCompleted;
    } else if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
               snapshot.modeState.splitgrade.executionState == SplitgradeExecutionState::Aborted) {
        overlayColorHex = kOverlayColorSgAborted;
    } else if (snapshot.espLinkStatus.health == EspLinkHealth::Lost ||
               snapshot.espLinkStatus.health == EspLinkHealth::Stale) {
        overlayColorHex = kOverlayColorLinkDegraded;
    } else if (snapshot.modeState.activeMode == ModeId::Splitgrade &&
               snapshot.modeState.splitgrade.parametersDirty) {
        overlayColorHex = kOverlayColorSgDirty;
    }

    // Global variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_TITLE_TEXT,
        StringValue(presenter_.getTitle(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_SUBTITLE_TEXT,
        StringValue(presenter_.getSubtitle(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_OVERLAY_TEXT,
        StringValue(presenter_.getOverlayText(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_OVERLAY_COLOR,
        IntegerValue(static_cast<int32_t>(overlayColorHex)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_OVERLAY_STATE,
        IntegerValue(static_cast<int32_t>(snapshot.inputModalStateCode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_ACTIVE_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.activeMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_ACTIVE_WORKFLOW_FAMILY,
        IntegerValue(computeWorkflowFamily(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_MODAL_STATE,
        IntegerValue(static_cast<int32_t>(snapshot.inputModalStateCode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_LINK_HEALTH,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.health)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_THERMAL_DERATING_ACTIVE,
        BooleanValue(snapshot.exposureState.thermalDeratingActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_SERVICE_SENSOR_FLAGS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.serviceSensors.sensorFlags)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_AHT_TEMPERATURE_CELSIUS,
        FloatValue(snapshot.espLinkStatus.serviceSensors.ahtTemperatureCelsius));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_AHT_HUMIDITY_PERCENT,
        FloatValue(snapshot.espLinkStatus.serviceSensors.ahtHumidityPercent));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_BMP_TEMPERATURE_CELSIUS,
        FloatValue(snapshot.espLinkStatus.serviceSensors.bmpTemperatureCelsius));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_BMP_PRESSURE_HPA,
        FloatValue(snapshot.espLinkStatus.serviceSensors.bmpPressureHpa));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_TOUCH_ACTIVE,
        BooleanValue(snapshot.touchState.active));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_START_BUTTON_ACTIVE,
        BooleanValue(snapshot.startButtonActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_PAPER_ACTIVE_SLOT,
        IntegerValue(static_cast<int32_t>(snapshot.paperActiveSlot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_PAPER_SLOT_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.paperSlotCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_PAPER_ACTIVE_SLOT_NAME,
        StringValue(snapshot.paperActiveSlotName));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_PAPER_ACTIVE_SLOT_CALIBRATED,
        BooleanValue(snapshot.paperActiveSlotCalibrated));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_GLOBAL_PAPER_ACTIVE_GRADE_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.paperActiveGradeMode)));

    const PaperSlotUiSummary* selectedPaperSummary = resolveSelectedPaperSlotSummary(snapshot);
    const char* selectedPaperSlotName = selectedPaperSummary != nullptr ? selectedPaperSummary->name : "";
    const PaperGradeMode selectedPaperSlotGradeMode = selectedPaperSummary != nullptr
        ? selectedPaperSummary->gradeMode
        : PaperGradeMode::Multigrade;
    const bool selectedPaperSlotCalibrated = selectedPaperSummary != nullptr && selectedPaperSummary->calibrated;
    const bool selectedPaperSlotUseIsoMath = selectedPaperSummary != nullptr && selectedPaperSummary->useIsoMath;
    const float selectedPaperSlotFixedGradeValue = selectedPaperSummary != nullptr
        ? selectedPaperSummary->fixedGradeValue
        : 2.5f;

    // Paper page variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_PANEL,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.panel)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.selectedSlot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTION_DIRTY,
        BooleanValue(snapshot.modeState.paper.selectionDirty));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_ITEM,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.selectedItem)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_ITEM_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.itemCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_EDITING_ACTIVE,
        BooleanValue(snapshot.modeState.paper.editingActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_PARAMETERS_DIRTY,
        BooleanValue(snapshot.modeState.paper.parametersDirty));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_PERSIST_FAILED,
        BooleanValue(snapshot.modeState.paper.persistFailed));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.selectedSlotAgeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_ITEM_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.selectedItemAgeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STORAGE_ERROR_CODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.storageErrorCode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STORAGE_ERROR_DETAIL,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.storageErrorDetail)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SLOT_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.paperSlotCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_ACTIVE_SLOT,
        IntegerValue(static_cast<int32_t>(snapshot.paperActiveSlot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT_NAME,
        StringValue(selectedPaperSlotName));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT_GRADE_MODE,
        IntegerValue(static_cast<int32_t>(selectedPaperSlotGradeMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT_CALIBRATED,
        BooleanValue(selectedPaperSlotCalibrated));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT_USE_ISO_MATH,
        BooleanValue(selectedPaperSlotUseIsoMath));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_SELECTED_SLOT_FIXED_GRADE_VALUE,
        FloatValue(selectedPaperSlotFixedGradeValue));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_GRADE_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.paper.stagedProfile.gradeMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_FIXED_GRADE_VALUE,
        FloatValue(snapshot.modeState.paper.stagedProfile.fixedGradeValue));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_USE_ISO_MATH,
        BooleanValue(snapshot.modeState.paper.stagedProfile.useIsoMath));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_ISO_P,
        FloatValue(snapshot.modeState.paper.stagedProfile.isoP));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_ISO_R,
        FloatValue(snapshot.modeState.paper.stagedProfile.isoR));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_KBW,
        FloatValue(snapshot.modeState.paper.stagedProfile.kBw));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_KSOFT,
        FloatValue(snapshot.modeState.paper.stagedProfile.kSoft));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_KHARD,
        FloatValue(snapshot.modeState.paper.stagedProfile.kHard));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_PAPER_STAGED_CALIBRATED,
        BooleanValue(snapshot.modeState.paper.stagedProfile.calibrated));

    // Splitgrade page variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_HEADER_TEXT,
        StringValue(presenter_.getSgHeader(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_TARGETS_TEXT,
        StringValue(presenter_.getSgTargets(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_EXPOSURE_MAIN_TEXT,
        StringValue(presenter_.getSgExposureMain(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_PANEL,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.splitgrade.panel)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_EXECUTION_STATE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.splitgrade.executionState)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_CONTROL_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.splitgrade.controlMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_SOFT_TARGET,
        FloatValue(snapshot.modeState.splitgrade.softTarget));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_HARD_TARGET,
        FloatValue(snapshot.modeState.splitgrade.hardTarget));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_GRADE,
        FloatValue(snapshot.modeState.splitgrade.grade));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_PARAMETERS_DIRTY,
        BooleanValue(snapshot.modeState.splitgrade.parametersDirty));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_EXPOSURE_PHASE,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.phase)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_REMAINING_TIME_SECONDS,
        FloatValue(snapshot.exposureState.remainingTimeSeconds));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_CURRENT_DOSE,
        FloatValue(snapshot.exposureState.currentDose));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_TARGET_DOSE,
        FloatValue(snapshot.exposureState.targetDose));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_MEASURED_LUX,
        FloatValue(snapshot.exposureState.measuredLux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_RUNTIME_OUTPUT_LIMIT,
        FloatValue(snapshot.exposureState.runtimeOutputLimit));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_RUNTIME_HEAD_BUS_LATENCY_MS,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.runtimeHeadBusLatencyMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_FAULT_REASON,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.faultReason)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_FAULT_LATCHED,
        BooleanValue(snapshot.exposureState.faultLatched));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_SENSOR_FALLBACK_ACTIVE,
        BooleanValue(snapshot.exposureState.sensorFallbackActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SG_THERMAL_DERATING_ACTIVE,
        BooleanValue(snapshot.exposureState.thermalDeratingActive));

    // Setup page variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_SELECTED_ITEM,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.selectedItem)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_ITEM_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.itemCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_EDITING_ACTIVE,
        BooleanValue(snapshot.modeState.setup.editingActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_PARAMETERS_DIRTY,
        BooleanValue(snapshot.modeState.setup.parametersDirty));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_PERSIST_FAILED,
        BooleanValue(snapshot.modeState.setup.persistFailed));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_SELECTED_ITEM_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.selectedItemAgeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_STORAGE_ERROR_CODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.storageErrorCode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_STORAGE_ERROR_DETAIL,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.storageErrorDetail)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_SOUND_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.stagedSettings.soundMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_SOUND_VOLUME,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.setup.stagedSettings.soundVolume)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_VIBRATION_ENABLED,
        BooleanValue(snapshot.modeState.setup.stagedSettings.vibrationEnabled != 0u));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_MAX_HEAD_BRIGHTNESS_PERCENT,
        FloatValue(static_cast<float>(snapshot.modeState.setup.stagedSettings.maxHeadBrightnessPercent)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_THERMAL_DERATING_START_CELSIUS,
        FloatValue(snapshot.modeState.setup.stagedSettings.thermalProtection.deratingStartCelsius));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_THERMAL_HARD_STOP_CELSIUS,
        FloatValue(snapshot.modeState.setup.stagedSettings.thermalProtection.hardStopCelsius));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_RUNTIME_OUTPUT_LIMIT,
        FloatValue(snapshot.exposureState.runtimeOutputLimit));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_THERMAL_DERATING_ACTIVE,
        BooleanValue(snapshot.exposureState.thermalDeratingActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_HEADER_TEXT,
        StringValue(presenter_.getSgHeader(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_VALUE_TEXT,
        StringValue(presenter_.getSgExposureMain(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_SETUP_OVERLAY_TEXT,
        StringValue(presenter_.getOverlayText(snapshot)));

    // Exposure overlay variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_PHASE,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.phase)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_CONTROL_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.controlMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_TARGET_DOSE,
        FloatValue(snapshot.exposureState.targetDose));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_CURRENT_DOSE,
        FloatValue(snapshot.exposureState.currentDose));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_REMAINING_DOSE,
        FloatValue(snapshot.exposureState.remainingDose));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_REMAINING_TIME_SECONDS,
        FloatValue(snapshot.exposureState.remainingTimeSeconds));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_MEASURED_LUX,
        FloatValue(snapshot.exposureState.measuredLux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_RUNTIME_OUTPUT_LIMIT,
        FloatValue(snapshot.exposureState.runtimeOutputLimit));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_RUNTIME_HEAD_BUS_LATENCY_MS,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.runtimeHeadBusLatencyMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_THERMAL_DERATING_ACTIVE,
        BooleanValue(snapshot.exposureState.thermalDeratingActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_SENSOR_FALLBACK_ACTIVE,
        BooleanValue(snapshot.exposureState.sensorFallbackActive));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_SENSOR_FALLBACK_REASON,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.sensorFallbackReason)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_FAULT_REASON,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.faultReason)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_FAULT_LATCHED,
        BooleanValue(snapshot.exposureState.faultLatched));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_EXPOSURE_PHASE_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.exposureState.phaseAgeMs)));

    // Measurement page variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_ACTIVE_SOURCE,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.activeLux.source)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_ACTIVE_LUX_VALID,
        BooleanValue(snapshot.measurementStatus.activeLux.valid));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_ACTIVE_LUX,
        FloatValue(snapshot.measurementStatus.activeLux.lux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_ACTIVE_LUX_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.activeLux.ageMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_ACTIVE_LUX_SEQUENCE,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.activeLux.sequence)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_REFERENCE_VALID,
        BooleanValue(snapshot.measurementStatus.activeReference.valid));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_REFERENCE_LUX,
        FloatValue(snapshot.measurementStatus.activeReference.lux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_REFERENCE_SEQUENCE,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.activeReference.sequence)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_RELATIVE_EV_VALID,
        BooleanValue(snapshot.measurementStatus.activeRelativeEvValid));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_RELATIVE_EV_STOPS,
        FloatValue(snapshot.measurementStatus.activeRelativeEvStops));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_LOCAL_LUX,
        FloatValue(snapshot.measurementStatus.localLux.lux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_WIRELESS_LUX,
        FloatValue(snapshot.measurementStatus.wirelessLux.lux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_SAMPLE_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.session.sampleCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_CAPTURED_SAMPLE_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.session.capturedSampleCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_LATEST_ZONE_INDEX,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.session.latestSample.zoneIndex)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_UNDO_DEPTH,
        IntegerValue(static_cast<int32_t>(snapshot.measurementStatus.session.undoDepth)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_CAN_UNDO,
        BooleanValue(snapshot.measurementStatus.session.canUndo));
    {
        ArrayOfInteger histogram(kMeasurementHistogramBucketCount);
        for (size_t i = 0u; i < kMeasurementHistogramBucketCount; ++i) {
            histogram.at(static_cast<int>(i),
                static_cast<int>(snapshot.measurementStatus.session.zoneHistogram[i]));
        }
        flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_MEAS_ZONE_HISTOGRAM, histogram.value);
    }

    // Wireless remote page variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_LINK_HEALTH,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.health)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_HEARTBEAT_SEEN,
        BooleanValue(snapshot.espLinkStatus.heartbeatSeen));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_LAST_RX_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.lastRxAgeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_LAST_TX_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.lastTxAgeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_REMOTE_UPTIME_MS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.remoteUptimeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_PEER_STATE,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.peerState)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_BATTERY_PERCENT,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.batteryPercent)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_LAST_SEEN_AGE_MS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.lastSeenAgeMs)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_LAST_LUX,
        FloatValue(snapshot.espLinkStatus.wireless.lastLux));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_MEASUREMENT_SEQUENCE,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.measurementSequence)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_COMMAND_ACK_SEQUENCE,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.commandAckSequence)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_RENDER_STATUS_FLAGS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.renderStatusFlags)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_STALE_RENDER_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.staleRenderCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_RENDER_TIMEOUT_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.wireless.renderTimeoutCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_DIAGNOSTIC_CODE,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.diagnostic.code)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_DIAGNOSTIC_DETAIL,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.diagnostic.detail)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_DIAGNOSTIC_COUNTER,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.diagnostic.counter)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_TX_PENDING_FRAME_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.txQueue.pendingFrameCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_TX_DROPPED_RENDER_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.txQueue.droppedRenderCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_TX_DROPPED_COMMAND_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.txQueue.droppedCommandCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_RMT_IN_FLIGHT_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.remoteCommandTrackerStatus.inFlightCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_RMT_RETRY_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.remoteCommandTrackerStatus.retryCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_RMT_TIMEOUT_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.remoteCommandTrackerStatus.timeoutCount)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_REMOTE_RMT_SATURATION_COUNT,
        IntegerValue(static_cast<int32_t>(snapshot.remoteCommandTrackerStatus.saturationCount)));

    // Boot page variables
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_TITLE_TEXT,
        StringValue(presenter_.getTitle(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_SUBTITLE_TEXT,
        StringValue(presenter_.getSubtitle(snapshot)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_ACTIVE_MODE,
        IntegerValue(static_cast<int32_t>(snapshot.modeState.activeMode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_LINK_HEALTH,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.health)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_PAPER_SLOT_STORAGE_ERROR_CODE,
        IntegerValue(static_cast<int32_t>(snapshot.paperSlotStorageErrorCode)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_PAPER_SLOT_STORAGE_ERROR_DETAIL,
        IntegerValue(static_cast<int32_t>(snapshot.paperSlotStorageErrorDetail)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_REMOTE_CAPABILITY_BITS,
        IntegerValue(static_cast<int32_t>(snapshot.espLinkStatus.remoteCapabilityBits)));
    flow::setGlobalVariable(FLOW_GLOBAL_VARIABLE_BOOT_TOUCH_ACTIVE,
        BooleanValue(snapshot.touchState.active));

    // Direct widget push (hand-crafted screens without EEZ flow bindings)
    pushWidgetsFromSnapshot(snapshot, overlayColorHex);
}
uint16_t LvglUi::mapTouchAxis(int16_t rawValue, int16_t rawMin, int16_t rawMax, uint16_t pixelMax, bool invert) const {
if (rawMax <= rawMin) return 0;
long clamped = rawValue;
if (clamped < rawMin) clamped = rawMin;
if (clamped > rawMax) clamped = rawMax;
long mapped = map(clamped, rawMin, rawMax, 0, pixelMax);
if (invert) mapped = pixelMax - mapped;
return static_cast<uint16_t>(mapped);
}

bool LvglUi::dmaFlushSupported() const { return display_.asyncFlushSupported(); }
bool LvglUi::dmaFlushActive() const { return display_.asyncFlushActive(); }

}  // namespace dukatimer
