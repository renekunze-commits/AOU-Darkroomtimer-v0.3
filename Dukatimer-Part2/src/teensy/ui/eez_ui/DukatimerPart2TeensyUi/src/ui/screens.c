#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

/* Keep screen creation functions in Flash, not ITCM */
#ifndef FLASHMEM
#define FLASHMEM __attribute__((section(".text.flashmem")))
#endif

objects_t objects;

// Hand-crafted widget handles
DukaWidgets g_duka_widgets;

static const char *screen_names[] = { "Boot", "PageSplitgrade", "PagePaperWorkspace", "PageSetup", "PageMeasurement", "PageWirelessRemote", "Busy" };
static const char *object_names[] = { "boot", "page_splitgrade", "page_paper_workspace", "page_setup", "page_measurement", "page_wireless_remote", "busy" };

// ---------------------------------------------------------------------------
// Colour constants (doc section 4)
// EEZ-Projekt ist Zielvertrag fuer alle C_BG/C_TEXT/C_MSG_* Farben.
// Die Defines hier sind Runtime-Fallback fuer handgeschriebene Screens.
// EEZ-Namen vs. Runtime-Namen: C_TAB_ACTIVE_FG=C_TAB_ACTIVE_TEXT,
//   C_TAB_INACT_BG=C_TAB_IDLE_BG, C_TAB_INACT_FG=C_TAB_IDLE_TEXT (gleiche Werte).
// ---------------------------------------------------------------------------

// -- EEZ-Palette (Zielvertrag) --
#define C_BG            lv_color_hex(0x050202u)  /* EEZ: C_BG */
#define C_TEXT          lv_color_hex(0xF1E7E0u)  /* EEZ: C_TEXT */
#define C_TEXT_SEC      lv_color_hex(0xB8948Au)  /* EEZ: C_TEXT_SEC */
#define C_MSG_NORMAL    lv_color_hex(0x120606u)  /* EEZ: C_MSG_NORMAL */
#define C_MSG_FAULT     lv_color_hex(0x3A0506u)  /* EEZ: C_MSG_FAULT */
#define C_MSG_CONFIRM   lv_color_hex(0x2A100Au)  /* EEZ: C_MSG_CONFIRM */
#define C_WARN          lv_color_hex(0xE0A06Eu)  /* EEZ: C_WARN */
#define C_TAB_ACTIVE_BG lv_color_hex(0x2A0C0Cu)  /* EEZ: C_TAB_ACTIVE_BG */
#define C_TAB_INACT_BG  lv_color_hex(0x0B0404u)  /* EEZ: C_TAB_IDLE_BG */
#define C_TAB_ACTIVE_FG lv_color_hex(0xF1E7E0u)  /* EEZ: C_TAB_ACTIVE_TEXT */
#define C_TAB_INACT_FG  lv_color_hex(0x8A6A64u)  /* EEZ: C_TAB_IDLE_TEXT */

// -- Runtime-Fallback: strukturelle Border-/BG-Farben (nicht in EEZ-Palette) --
#define C_SECTION_BORDER lv_color_hex(0x2A1010u)  /* Header-/Abschnittstrennlinie */
#define C_PANEL_BORDER   lv_color_hex(0x1C0808u)  /* innere Panel-/Block-Rahmen */
#define C_BLOCK_BG       lv_color_hex(0x0D0505u)  /* leicht aufgehellter BG fuer Quell-/Diag-Bloecke */
#define C_ACCENT_PROGRESS lv_color_hex(0xC05030u) /* Fortschrittsbalken / Histogramm-Fuell (warm) */

// -- Runtime-Fallback: Aktions-Button-Farben (Diagnose-Ausnahme, nicht in EEZ-Palette) --
// Gruen/Rot/Amber sind Ausnahmen fuer Aktion-Buttons (Apply, Discard, Safety/Pause).
// Nicht auf Produktfarben ausweiten; Zielvertrag ist spaetere EEZ-Style-Migration.
#define C_BTN_CONFIRM_BG  lv_color_hex(0x0D2E18u)  /* Apply / Fortsetzen: Hintergrund */
#define C_BTN_CONFIRM_BD  lv_color_hex(0x1A5C30u)  /* Apply / Fortsetzen: Rahmen */
#define C_BTN_CONFIRM_FG  lv_color_hex(0x7FD4A0u)  /* Apply / Fortsetzen: Text */
#define C_BTN_DISCARD_BG  lv_color_hex(0x2E0D0Du)  /* Discard / Abbrechen: Hintergrund */
#define C_BTN_DISCARD_BD  lv_color_hex(0x5C1A1Au)  /* Discard / Abbrechen: Rahmen */
#define C_BTN_DISCARD_FG  lv_color_hex(0xD47F7Fu)  /* Discard / Abbrechen: Text */
#define C_BTN_SAFETY_BG   lv_color_hex(0x2F2610u)  /* SafetyDefaults / Pause: Hintergrund */
#define C_BTN_SAFETY_BD   lv_color_hex(0x7A6730u)  /* SafetyDefaults / Pause: Rahmen */
#define C_BTN_SAFETY_FG   lv_color_hex(0xE5D08Au)  /* SafetyDefaults / Pause: Text */

// ---------------------------------------------------------------------------
// Shared header builder (y=0, h=48)
// ---------------------------------------------------------------------------
static FLASHMEM void build_header(lv_obj_t *parent) {
    // Background strip
    lv_obj_t *hdr = lv_obj_create(parent);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_size(hdr, 480, 48);
    lv_obj_set_style_bg_color(hdr, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(hdr, C_SECTION_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(hdr, 0, LV_PART_MAIN);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    // Left: title / mode label (x=0, w=140)
    lv_obj_t *title = lv_label_create(hdr);
    lv_obj_set_pos(title, 4, 14);
    lv_obj_set_size(title, 132, 24);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(title, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(title, "DUKATIMER");
    lv_label_set_long_mode(title, LV_LABEL_LONG_CLIP);
    g_duka_widgets.hdr_title = title;

    // Centre: message band (x=140, w=230, h=48)
    lv_obj_t *msg_cont = lv_obj_create(hdr);
    lv_obj_set_pos(msg_cont, 140, 0);
    lv_obj_set_size(msg_cont, 230, 48);
    lv_obj_set_style_bg_color(msg_cont, C_MSG_NORMAL, LV_PART_MAIN);
    lv_obj_set_style_radius(msg_cont, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_cont, 6, LV_PART_MAIN);
    lv_obj_clear_flag(msg_cont, LV_OBJ_FLAG_SCROLLABLE);
    g_duka_widgets.hdr_msg_cont = msg_cont;

    lv_obj_t *msg_lbl = lv_label_create(msg_cont);
    lv_obj_set_size(msg_lbl, 218, 36);
    lv_obj_align(msg_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(msg_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(msg_lbl, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(msg_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(msg_lbl, "");
    lv_label_set_long_mode(msg_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.hdr_msg_lbl = msg_lbl;

    // Right: thermal chip (x=370, w=110) — hidden until derating active
    lv_obj_t *therm = lv_label_create(hdr);
    lv_obj_set_pos(therm, 374, 14);
    lv_obj_set_size(therm, 102, 24);
    lv_obj_set_style_text_font(therm, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(therm, C_WARN, LV_PART_MAIN);
    lv_obj_set_style_text_align(therm, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(therm, "THERM");
    lv_obj_add_flag(therm, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.hdr_thermal_lbl = therm;
}

// ---------------------------------------------------------------------------
// Shared ModeTab bar builder (y=284, h=36)
// ---------------------------------------------------------------------------
static FLASHMEM void build_modetabs(lv_obj_t *parent) {
    static const char *labels[4] = { "PAPER", "MEAS", "PRINT", "SETUP" };
    lv_obj_t **handles[4] = {
        &g_duka_widgets.tab_paper,
        &g_duka_widgets.tab_meas,
        &g_duka_widgets.tab_print,
        &g_duka_widgets.tab_setup,
    };
    const lv_coord_t tab_w = 120;
    for (int i = 0; i < 4; ++i) {
        lv_obj_t *tab = lv_obj_create(parent);
        lv_obj_set_pos(tab, (lv_coord_t)(i * tab_w), 284);
        lv_obj_set_size(tab, tab_w, 36);
        lv_obj_set_style_bg_color(tab, C_TAB_INACT_BG, LV_PART_MAIN);
        lv_obj_set_style_radius(tab, 3, LV_PART_MAIN);
        lv_obj_set_style_border_side(tab, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
        lv_obj_set_style_border_width(tab, 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(tab, C_PANEL_BORDER, LV_PART_MAIN);
        lv_obj_set_style_pad_all(tab, 0, LV_PART_MAIN);
        lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *lbl = lv_label_create(tab);
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl, C_TAB_INACT_FG, LV_PART_MAIN);
        lv_label_set_text(lbl, labels[i]);

        *handles[i] = lbl;
    }
}

//
// Event handlers
//

lv_obj_t *tick_value_change_obj;

// Forward declaration: implemented in LvglUi.cpp and wired at begin() time.
// Delivers global modal button actions (1=Resume, 2=Abort, 3=Pause) without
// coupling this C file to C++ classes.
extern void lvgl_ui_modal_action_callback(uint8_t action);

static FLASHMEM void busy_btn_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lvgl_ui_modal_action_callback((uint8_t)(uintptr_t)lv_event_get_user_data(e));
    }
}

//
// Screens
//

FLASHMEM void create_screen_boot() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.boot = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    // Title — large centred
    lv_obj_t *title = lv_label_create(obj);
    lv_obj_set_pos(title, 20, 60);
    lv_obj_set_size(title, 440, 36);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(title, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(title, "DUKATIMER");
    g_duka_widgets.boot_title_lbl = title;

    // Subtitle
    lv_obj_t *sub = lv_label_create(obj);
    lv_obj_set_pos(sub, 20, 110);
    lv_obj_set_size(sub, 440, 24);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(sub, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(sub, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(sub, "Initialisierung...");
    g_duka_widgets.boot_subtitle_lbl = sub;

    // Status line
    lv_obj_t *status = lv_label_create(obj);
    lv_obj_set_pos(status, 20, 200);
    lv_obj_set_size(status, 440, 48);
    lv_obj_set_style_text_font(status, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(status, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(status, "");
    lv_label_set_long_mode(status, LV_LABEL_LONG_WRAP);
    g_duka_widgets.boot_status_lbl = status;

    tick_screen_boot();
}

FLASHMEM void tick_screen_boot() {
    // B1-Blocker: create_screen_boot() ist handgeschrieben und registriert keine
    // EEZ-Widget-Binding-States fuer boot_title_lbl, boot_subtitle_lbl,
    // boot_status_lbl. Solange das so ist, kann eez_flow_tick() die Labels nicht
    // automatisch aktualisieren. Aktiver Update-Pfad: LvglUi::pushWidgetsFromSnapshot()
    // via g_duka_widgets.boot_*_lbl (Direktzugriff auf LVGL-Handles).
    //
    // Voraussetzung fuer EEZ-gefuehrtes Boot:
    //   create_screen_boot() muss aus einem EEZ-Export kommen, der die
    //   Binding-States fuer die drei Labels registriert (Abschnitt 13.4 EEZ-Doc).
    //   Exportpfad: Staging-Verzeichnis gemaess B0-Exportvertrag.
    void *flowState = getFlowState(0, 0);
    (void)flowState;
}

FLASHMEM void create_screen_page_splitgrade() {
    void *flowState = getFlowState(0, 1);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.page_splitgrade = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    build_header(obj);
    build_modetabs(obj);

    // ---- SG-Panel links (y=48, h=96, w=220) --------------------------------
    lv_obj_t *sg_left = lv_obj_create(obj);
    lv_obj_set_pos(sg_left, 0, 48);
    lv_obj_set_size(sg_left, 220, 96);
    lv_obj_set_style_bg_color(sg_left, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(sg_left, LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_border_width(sg_left, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(sg_left, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(sg_left, 4, LV_PART_MAIN);
    lv_obj_clear_flag(sg_left, LV_OBJ_FLAG_SCROLLABLE);

    // Grade (row 0, big)
    lv_obj_t *grade = lv_label_create(sg_left);
    lv_obj_set_pos(grade, 4, 2);
    lv_obj_set_size(grade, 212, 42);
    lv_obj_set_style_text_font(grade, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(grade, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(grade, "G -.-");
    g_duka_widgets.sg_grade_lbl = grade;

    // Soft target (row 1)
    lv_obj_t *soft = lv_label_create(sg_left);
    lv_obj_set_pos(soft, 4, 46);
    lv_obj_set_size(soft, 212, 22);
    lv_obj_set_style_text_font(soft, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(soft, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(soft, "SOFT --");
    g_duka_widgets.sg_soft_lbl = soft;

    // Hard target (row 2)
    lv_obj_t *hard = lv_label_create(sg_left);
    lv_obj_set_pos(hard, 4, 70);
    lv_obj_set_size(hard, 160, 22);
    lv_obj_set_style_text_font(hard, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(hard, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(hard, "HARD --");
    g_duka_widgets.sg_hard_lbl = hard;

    // ControlMode chip (right of hard)
    lv_obj_t *ctrl = lv_label_create(sg_left);
    lv_obj_set_pos(ctrl, 168, 70);
    lv_obj_set_size(ctrl, 48, 22);
    lv_obj_set_style_text_font(ctrl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(ctrl, C_WARN, LV_PART_MAIN);
    lv_obj_set_style_text_align(ctrl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(ctrl, "DOSE");
    g_duka_widgets.sg_ctrl_mode_lbl = ctrl;

    // ---- Execution / Exposure block (y=144, h=104) --------------------------
    // Left (x=0, w=220, h=104): exec state + paper info
    lv_obj_t *exec_block = lv_obj_create(obj);
    lv_obj_set_pos(exec_block, 0, 144);
    lv_obj_set_size(exec_block, 220, 104);
    lv_obj_set_style_bg_color(exec_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(exec_block, LV_BORDER_SIDE_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_border_width(exec_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(exec_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(exec_block, 4, LV_PART_MAIN);
    lv_obj_clear_flag(exec_block, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *exec_state = lv_label_create(exec_block);
    lv_obj_set_pos(exec_state, 4, 4);
    lv_obj_set_size(exec_state, 212, 28);
    lv_obj_set_style_text_font(exec_state, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(exec_state, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(exec_state, "IDLE");
    g_duka_widgets.sg_exec_state_lbl = exec_state;

    lv_obj_t *paper = lv_label_create(exec_block);
    lv_obj_set_pos(paper, 4, 36);
    lv_obj_set_size(paper, 212, 60);
    lv_obj_set_style_text_font(paper, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(paper, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(paper, "PAPER --");
    lv_label_set_long_mode(paper, LV_LABEL_LONG_WRAP);
    g_duka_widgets.sg_paper_lbl = paper;

    // Right (x=220, w=260, h=104): time, dose, lux, head
    lv_obj_t *exp_block = lv_obj_create(obj);
    lv_obj_set_pos(exp_block, 220, 144);
    lv_obj_set_size(exp_block, 260, 104);
    lv_obj_set_style_bg_color(exp_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(exp_block, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(exp_block, 4, LV_PART_MAIN);
    lv_obj_clear_flag(exp_block, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *time_lbl = lv_label_create(exp_block);
    lv_obj_set_pos(time_lbl, 4, 2);
    lv_obj_set_size(time_lbl, 252, 42);
    lv_obj_set_style_text_font(time_lbl, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(time_lbl, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(time_lbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(time_lbl, "0.0 s");
    g_duka_widgets.sg_time_lbl = time_lbl;

    lv_obj_t *dose_lbl = lv_label_create(exp_block);
    lv_obj_set_pos(dose_lbl, 4, 48);
    lv_obj_set_size(dose_lbl, 252, 20);
    lv_obj_set_style_text_font(dose_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(dose_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(dose_lbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(dose_lbl, "0.00 / 0.00 lux-s");
    g_duka_widgets.sg_dose_lbl = dose_lbl;

    lv_obj_t *lux_lbl = lv_label_create(exp_block);
    lv_obj_set_pos(lux_lbl, 4, 70);
    lv_obj_set_size(lux_lbl, 128, 20);
    lv_obj_set_style_text_font(lux_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(lux_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(lux_lbl, "LUX --");
    g_duka_widgets.sg_lux_lbl = lux_lbl;

    lv_obj_t *head_lbl = lv_label_create(exp_block);
    lv_obj_set_pos(head_lbl, 136, 70);
    lv_obj_set_size(head_lbl, 120, 20);
    lv_obj_set_style_text_font(head_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(head_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(head_lbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(head_lbl, "HEAD --");
    g_duka_widgets.sg_head_lbl = head_lbl;

    // ---- Context bar (y=248, h=36) -----------------------------------------
    lv_obj_t *ctx_bar = lv_obj_create(obj);
    lv_obj_set_pos(ctx_bar, 0, 248);
    lv_obj_set_size(ctx_bar, 480, 36);
    lv_obj_set_style_bg_color(ctx_bar, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(ctx_bar, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ctx_bar, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(ctx_bar, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ctx_bar, 4, LV_PART_MAIN);
    lv_obj_clear_flag(ctx_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ctx_lbl = lv_label_create(ctx_bar);
    lv_obj_set_pos(ctx_lbl, 4, 4);
    lv_obj_set_size(ctx_lbl, 472, 28);
    lv_obj_set_style_text_font(ctx_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(ctx_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(ctx_lbl, "E1=GRADE  E2=MODUS  E3=PANEL  E3-LONG=SETUP");
    lv_label_set_long_mode(ctx_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.sg_context_lbl = ctx_lbl;

    tick_screen_page_splitgrade();
}

FLASHMEM void tick_screen_page_splitgrade() {
    void *flowState = getFlowState(0, 1);
    (void)flowState;
}

FLASHMEM void create_screen_page_paper_workspace() {
    void *flowState = getFlowState(0, 2);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.page_paper_workspace = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    build_header(obj);
    build_modetabs(obj);

    // ---- Panel info strip (y=48, h=24) — panel type + slot + state --------
    lv_obj_t *panel_strip = lv_obj_create(obj);
    lv_obj_set_pos(panel_strip, 0, 48);
    lv_obj_set_size(panel_strip, 480, 24);
    lv_obj_set_style_bg_color(panel_strip, lv_color_hex(0x0E0505u), LV_PART_MAIN);
    lv_obj_set_style_border_side(panel_strip, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel_strip, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel_strip, C_SECTION_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel_strip, 0, LV_PART_MAIN);
    lv_obj_clear_flag(panel_strip, LV_OBJ_FLAG_SCROLLABLE);

    // Linker Textteil: Slot-Nr. + Name (x=4, w=280)
    lv_obj_t *panel_lbl = lv_label_create(panel_strip);
    lv_obj_set_pos(panel_lbl, 4, 3);
    lv_obj_set_size(panel_lbl, 280, 18);
    lv_obj_set_style_text_font(panel_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(panel_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(panel_lbl, "PAPER");
    lv_label_set_long_mode(panel_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.paper_panel_lbl = panel_lbl;

    // Panel-Switcher-Chips rechts im Strip: SELECT (x=290) und CAL (x=376)
    // LvglUi hebt den aktiven Chip hervor (helle Textfarbe + Unterrand).
    lv_obj_t *chip_select = lv_label_create(panel_strip);
    lv_obj_set_pos(chip_select, 290, 3);
    lv_obj_set_size(chip_select, 82, 18);
    lv_obj_set_style_text_font(chip_select, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(chip_select, C_TAB_INACT_FG, LV_PART_MAIN);
    lv_obj_set_style_text_align(chip_select, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(chip_select, "SELECT");
    g_duka_widgets.paper_chip_select = chip_select;

    lv_obj_t *chip_cal = lv_label_create(panel_strip);
    lv_obj_set_pos(chip_cal, 376, 3);
    lv_obj_set_size(chip_cal, 82, 18);
    lv_obj_set_style_text_font(chip_cal, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(chip_cal, C_TAB_INACT_FG, LV_PART_MAIN);
    lv_obj_set_style_text_align(chip_cal, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(chip_cal, "CAL");
    g_duka_widgets.paper_chip_cal = chip_cal;

    // ---- 7 item/slot rows (y=72, each 26 px → y=72..254) ------------------
    // Sliding-window list shared by CAL panel (calibration items) and
    // SELECT panel (paper slots). LvglUi::pushWidgetsFromSnapshot() decides
    // which data drives each row based on the active panel.
    for (int i = 0; i < 7; ++i) {
        int y = 72 + i * 26;

        lv_obj_t *row = lv_obj_create(obj);
        lv_obj_set_pos(row, 2, y);
        lv_obj_set_size(row, 476, 25);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x0B0404u), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(row, 3, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        g_duka_widgets.paper_cal_row_cont[i] = row;

        lv_obj_t *lbl = lv_label_create(row);
        lv_obj_set_pos(lbl, 12, 4);
        lv_obj_set_size(lbl, 248, 17);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0x9A8080u), LV_PART_MAIN);
        lv_label_set_text(lbl, "---");
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
        g_duka_widgets.paper_cal_row_lbl[i] = lbl;

        lv_obj_t *val = lv_label_create(row);
        lv_obj_set_pos(val, 260, 4);
        lv_obj_set_size(val, 216, 17);
        lv_obj_set_style_text_font(val, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(val, lv_color_hex(0x7A6868u), LV_PART_MAIN);
        lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
        lv_label_set_text(val, "");
        lv_label_set_long_mode(val, LV_LABEL_LONG_CLIP);
        g_duka_widgets.paper_cal_row_val[i] = val;
    }

    // ---- Hint bar (y=258, h=22) — context-sensitive encoder hints ---------
    lv_obj_t *hint = lv_label_create(obj);
    lv_obj_set_pos(hint, 0, 258);
    lv_obj_set_size(hint, 480, 22);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(hint, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(hint, "E1=ITEM  E1-PRESS=EDIT  E3-LONG=SELECT");
    lv_label_set_long_mode(hint, LV_LABEL_LONG_CLIP);
    g_duka_widgets.paper_hint_lbl = hint;

    tick_screen_page_paper_workspace();
}

FLASHMEM void tick_screen_page_paper_workspace() {
    // B2-Blocker: create_screen_page_paper_workspace() ist handgeschrieben und
    // registriert keine EEZ-Widget-Binding-States fuer paper_panel_lbl,
    // paper_cal_row_cont/lbl/val[0-6], paper_hint_lbl, paper_chip_select,
    // paper_chip_cal. Aktiver Update-Pfad: LvglUi::pushWidgetsFromSnapshot()
    // via g_duka_widgets.paper_* (Direktzugriff auf LVGL-Handles).
    //
    // EEZ-Datenschicht: 33 Paper-Flow-Globals (Indices 17-49) vollstaendig in
    // LvglUi::updateSnapshot() geschrieben. Nicht in vars.h (benoetigen
    // EEZ-Projektaenderung + Re-Export fuer vollstaendigen CAL-Pfad):
    //   PAPER_STEP_WHITE / PAPER_STEP_BLACK (CAL-Panel Stufenwerte)
    //   PAPER_PANEL_TEXT, PAPER_HINT_TEXT (formatierte Ausgabestrings)
    //   Per-Slot-Summaries fuer SELECT-Panel-List (nur ausgewaehlter Slot in EEZ)
    //
    // Vollstaendige Migration: EEZ Studio Export gemaess B0-Vertrag (Abschnitt 13.4).
    // Ergebnis dokumentiert in Abschnitt 13.8 der EEZ-Doku.
    void *flowState = getFlowState(0, 2);
    (void)flowState;
}

// ---------------------------------------------------------------------------
// Setup-Screen — SETUP-ITEM-LISTE + EDIT-BLOCK (Schritt 6 der Spec)
// ---------------------------------------------------------------------------
FLASHMEM void create_screen_page_setup() {
    void *flowState = getFlowState(0, 3);
    (void)flowState;

    lv_obj_t *obj = lv_obj_create(0);
    objects.page_setup = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    build_header(obj);
    build_modetabs(obj);

    // ---- SETUP-ITEM-LISTE (y=48, 6 Zeilen à 22px + 6px Abstand = 28px) ---
    // Zeilen y=50..217 (6 * 28 = 168px, letzter Abschluss bei y=218)
    static const char *item_names[6] = {
        "Tonmodus",
        "Lautstaerke",
        "Vibration",
        "Max. Lampenhelligkeit",
        "Therm. Drosselung ab",
        "Therm. Abschaltschwelle",
    };
    for (int i = 0; i < 6; ++i) {
        int y = 50 + i * 28;

        lv_obj_t *row = lv_obj_create(obj);
        lv_obj_set_pos(row, 2, y);
        lv_obj_set_size(row, 476, 26);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x0B0404u), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(row, 3, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        g_duka_widgets.setup_item_row_cont[i] = row;

        // Cursor-Markierung als linker farbiger Balken (initial versteckt)
        lv_obj_t *cursor = lv_obj_create(row);
        lv_obj_set_pos(cursor, 0, 0);
        lv_obj_set_size(cursor, 3, 26);
        lv_obj_set_style_bg_color(cursor, C_WARN, LV_PART_MAIN);
        lv_obj_set_style_border_width(cursor, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(cursor, 0, LV_PART_MAIN);
        lv_obj_add_flag(cursor, LV_OBJ_FLAG_HIDDEN);
        // Kein Handle noetig; LvglUi steuert ueber row-Hintergrundfarbe

        lv_obj_t *lbl = lv_label_create(row);
        lv_obj_set_pos(lbl, 10, 4);
        lv_obj_set_size(lbl, 240, 18);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl, C_TEXT_SEC, LV_PART_MAIN);
        lv_label_set_text(lbl, item_names[i]);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
        g_duka_widgets.setup_item_lbl[i] = lbl;

        lv_obj_t *val = lv_label_create(row);
        lv_obj_set_pos(val, 254, 4);
        lv_obj_set_size(val, 218, 18);
        lv_obj_set_style_text_font(val, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_style_text_color(val, C_TEXT, LV_PART_MAIN);
        lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
        lv_label_set_text(val, "---");
        lv_label_set_long_mode(val, LV_LABEL_LONG_CLIP);
        g_duka_widgets.setup_item_val[i] = val;
    }

    // ---- Trennlinie zwischen Liste und Edit-Block (y=220) ------------------
    lv_obj_t *sep = lv_obj_create(obj);
    lv_obj_set_pos(sep, 0, 218);
    lv_obj_set_size(sep, 480, 1);
    lv_obj_set_style_bg_color(sep, C_SECTION_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sep, 0, LV_PART_MAIN);
    lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);

    // ---- EDIT-BLOCK (y=220, h=64) ------------------------------------------
    lv_obj_t *edit = lv_obj_create(obj);
    lv_obj_set_pos(edit, 0, 220);
    lv_obj_set_size(edit, 480, 64);
    lv_obj_set_style_bg_color(edit, lv_color_hex(0x0D0606u), LV_PART_MAIN);
    lv_obj_set_style_border_width(edit, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(edit, 0, LV_PART_MAIN);
    lv_obj_clear_flag(edit, LV_OBJ_FLAG_SCROLLABLE);

    // Item-Name (Zeile 1, y=222, Montserrat 14)
    lv_obj_t *edit_name = lv_label_create(obj);
    lv_obj_set_pos(edit_name, 8, 222);
    lv_obj_set_size(edit_name, 260, 20);
    lv_obj_set_style_text_font(edit_name, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(edit_name, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(edit_name, "---");
    lv_label_set_long_mode(edit_name, LV_LABEL_LONG_CLIP);
    g_duka_widgets.setup_edit_name_lbl = edit_name;

    // Aktueller Wert (Zeile 1 rechts, Montserrat 20 bold-aehnlich)
    lv_obj_t *edit_val = lv_label_create(obj);
    lv_obj_set_pos(edit_val, 270, 220);
    lv_obj_set_size(edit_val, 202, 28);
    lv_obj_set_style_text_font(edit_val, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(edit_val, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(edit_val, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(edit_val, "---");
    lv_label_set_long_mode(edit_val, LV_LABEL_LONG_CLIP);
    g_duka_widgets.setup_edit_val_lbl = edit_val;

    // CAP-Anzeige (y=244, Zeile 2 links): konfigurierte Helligkeitsgrenze
    lv_obj_t *cap_lbl = lv_label_create(obj);
    lv_obj_set_pos(cap_lbl, 8, 244);
    lv_obj_set_size(cap_lbl, 116, 18);
    lv_obj_set_style_text_font(cap_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(cap_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(cap_lbl, "CAP ---%");
    lv_obj_add_flag(cap_lbl, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.setup_edit_cap_lbl = cap_lbl;

    // LIVE-Anzeige (y=244, Zeile 2 Mitte): effektive Laufzeitleistung
    lv_obj_t *live_lbl = lv_label_create(obj);
    lv_obj_set_pos(live_lbl, 128, 244);
    lv_obj_set_size(live_lbl, 116, 18);
    lv_obj_set_style_text_font(live_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(live_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(live_lbl, "LIVE ---%");
    lv_obj_add_flag(live_lbl, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.setup_edit_live_lbl = live_lbl;

    // Bereich / Thermal-Hinweis (y=244 rechts)
    lv_obj_t *range_lbl = lv_label_create(obj);
    lv_obj_set_pos(range_lbl, 248, 244);
    lv_obj_set_size(range_lbl, 224, 18);
    lv_obj_set_style_text_font(range_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(range_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(range_lbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(range_lbl, "");
    lv_label_set_long_mode(range_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.setup_edit_range_lbl = range_lbl;

    // Encoder-Hilfe (y=264, letzte Zeile vor ModeTab)
    lv_obj_t *hint = lv_label_create(obj);
    lv_obj_set_pos(hint, 8, 265);
    lv_obj_set_size(hint, 472, 16);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(hint, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(hint, "E1=WERT  E3=ITEM  E3-LONG=BEENDEN");
    lv_label_set_long_mode(hint, LV_LABEL_LONG_CLIP);
    g_duka_widgets.setup_hint_lbl = hint;

    // ---- Action-Buttons: Apply / Discard / SafetyDefaults ------------------
    // Positionierung: x=2..160, x=162..318, x=320..478; y=48+6*28+2 = 218; aber
    // Edit-Block beginnt bei y=220, also Buttons in die untere Haelfte der Liste
    // oder als separate Zeile. Loesung: Buttons als letzte "Zeile" der Liste
    // bei y=220-48 nicht mehr moeglich -> Spec sagt "in SETUP-ITEM-LISTE unten".
    // Kompromiss: Buttons in den Edit-Block eingebettet, als zweite Zeile neben
    // Name/Wert (y=220, ausgeblendet wenn editingActive==false).
    // Hier separate Positionierung auf y=220..256 links, vertikal zentriert.
    // Tatsaechliche Sichtbarkeit steuert LvglUi per Hidden-Flag je nach editingActive.

    // Apply (links)
    lv_obj_t *btn_apply = lv_obj_create(edit);
    lv_obj_set_pos(btn_apply, 2, 4);
    lv_obj_set_size(btn_apply, 140, 56);
    lv_obj_set_style_bg_color(btn_apply, C_BTN_CONFIRM_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn_apply, C_BTN_CONFIRM_BD, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn_apply, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(btn_apply, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn_apply, 0, LV_PART_MAIN);
    lv_obj_clear_flag(btn_apply, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_apply, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(btn_apply, C_BTN_CONFIRM_BD, LV_PART_MAIN | LV_STATE_PRESSED); // C3: pressed visual (inaktiver Hook)
    lv_obj_add_flag(btn_apply, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *apply_lbl = lv_label_create(btn_apply);
    lv_obj_align(apply_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(apply_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(apply_lbl, C_BTN_CONFIRM_FG, LV_PART_MAIN);
    lv_label_set_text(apply_lbl, "UEBERNEHMEN");
    g_duka_widgets.setup_btn_apply = btn_apply;

    // Discard (Mitte)
    lv_obj_t *btn_discard = lv_obj_create(edit);
    lv_obj_set_pos(btn_discard, 146, 4);
    lv_obj_set_size(btn_discard, 140, 56);
    lv_obj_set_style_bg_color(btn_discard, C_BTN_DISCARD_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn_discard, C_BTN_DISCARD_BD, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn_discard, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(btn_discard, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn_discard, 0, LV_PART_MAIN);
    lv_obj_clear_flag(btn_discard, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_discard, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(btn_discard, C_BTN_DISCARD_BD, LV_PART_MAIN | LV_STATE_PRESSED); // C3: pressed visual (inaktiver Hook)
    lv_obj_add_flag(btn_discard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *discard_lbl = lv_label_create(btn_discard);
    lv_obj_align(discard_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(discard_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(discard_lbl, C_BTN_DISCARD_FG, LV_PART_MAIN);
    lv_label_set_text(discard_lbl, "VERWERFEN");
    g_duka_widgets.setup_btn_discard = btn_discard;

    // SafetyDefaults (rechts)
    lv_obj_t *btn_safety = lv_obj_create(edit);
    lv_obj_set_pos(btn_safety, 290, 4);
    lv_obj_set_size(btn_safety, 186, 56);
    lv_obj_set_style_bg_color(btn_safety, C_BTN_SAFETY_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn_safety, C_BTN_SAFETY_BD, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn_safety, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(btn_safety, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn_safety, 0, LV_PART_MAIN);
    lv_obj_clear_flag(btn_safety, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn_safety, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(btn_safety, C_BTN_SAFETY_BD, LV_PART_MAIN | LV_STATE_PRESSED); // C3: pressed visual (inaktiver Hook)
    lv_obj_add_flag(btn_safety, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *safety_lbl = lv_label_create(btn_safety);
    lv_obj_align(safety_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(safety_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(safety_lbl, C_BTN_SAFETY_FG, LV_PART_MAIN);
    lv_label_set_text(safety_lbl, "SICHERHEITSSTANDARDS");
    g_duka_widgets.setup_btn_safety = btn_safety;

    tick_screen_page_setup();
}

FLASHMEM void tick_screen_page_setup() {
    // B4-Blocker: create_screen_page_setup() ist handgeschrieben und registriert
    // keine EEZ-Widget-Binding-States fuer setup_item_row_cont/lbl/val[0-5],
    // setup_edit_name_lbl, setup_edit_val_lbl, setup_edit_cap_lbl,
    // setup_edit_live_lbl, setup_edit_range_lbl, setup_hint_lbl,
    // setup_btn_apply, setup_btn_discard, setup_btn_safety.
    // Aktiver Update-Pfad: LvglUi::pushWidgetsFromSnapshot() via g_duka_widgets.setup_*.
    //
    // Action-Buttons: setup_btn_apply/discard/safety haben LV_OBJ_FLAG_CLICKABLE,
    // aber keinen LVGL-Event-Callback (actions.h ist leer). Firmwarepfad:
    // Apply/Discard/SafetyDefaults werden ausschliesslich ueber Encoder ausgeloest
    // (SetupWorkflow verarbeitet SetupMenuItem::Apply, Discard, SafetyDefaults).
    // Touch-Hooks fuer die drei Buttons sind offene Luecke (kein Guard vorhanden).
    //
    // EEZ-Datenschicht: 19 Setup-Flow-Globals (Indices 71-89) vollstaendig
    // geschrieben. Fehlende EEZ-Var: SETUP_HEAD_TIMING_DIAGNOSTICS_ENABLED
    // (setup.headTimingDiagnosticsEnabled im HeadTimingDiagnostics-Renderpfad).
    // Ergebnis dokumentiert in Abschnitt 13.9 der EEZ-Doku.
    void *flowState = getFlowState(0, 3);
    (void)flowState;
}

// ---------------------------------------------------------------------------
// Measurement-Screen — QUELLEN + HAUPT + REFERENZ + HISTOGRAMM + SESSION
// Spec Schritt 8; kein neuer Font noetig (Hauptmesswert: Montserrat 28)
// ---------------------------------------------------------------------------
FLASHMEM void create_screen_page_measurement() {
    void *flowState = getFlowState(0, 4);
    (void)flowState;

    lv_obj_t *obj = lv_obj_create(0);
    objects.page_measurement = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    build_header(obj);
    build_modetabs(obj);

    // =========================================================================
    // QUELLEN-BLOCK (x=0, y=48, w=180, h=92)
    // =========================================================================
    lv_obj_t *src_block = lv_obj_create(obj);
    lv_obj_set_pos(src_block, 0, 48);
    lv_obj_set_size(src_block, 180, 92);
    lv_obj_set_style_bg_color(src_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(src_block, LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_border_width(src_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(src_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(src_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(src_block, LV_OBJ_FLAG_SCROLLABLE);

    // Quellen-Chip (Zeile 1)
    lv_obj_t *src_chip = lv_label_create(src_block);
    lv_obj_set_pos(src_chip, 0, 0);
    lv_obj_set_size(src_chip, 168, 18);
    lv_obj_set_style_text_font(src_chip, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(src_chip, C_WARN, LV_PART_MAIN);
    lv_label_set_text(src_chip, "QUELLE: ---");
    lv_label_set_long_mode(src_chip, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_source_chip_lbl = src_chip;

    // Lokal-Lux (Zeile 2)
    lv_obj_t *local_lbl = lv_label_create(src_block);
    lv_obj_set_pos(local_lbl, 0, 24);
    lv_obj_set_size(local_lbl, 168, 20);
    lv_obj_set_style_text_font(local_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(local_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(local_lbl, "LOKAL  ---");
    lv_label_set_long_mode(local_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_local_lbl = local_lbl;

    // Wireless-Lux (Zeile 3)
    lv_obj_t *wireless_lbl = lv_label_create(src_block);
    lv_obj_set_pos(wireless_lbl, 0, 48);
    lv_obj_set_size(wireless_lbl, 168, 20);
    lv_obj_set_style_text_font(wireless_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(wireless_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(wireless_lbl, "FUNK   ---");
    lv_label_set_long_mode(wireless_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_wireless_lbl = wireless_lbl;

    // =========================================================================
    // HAUPTMESSWERT (x=180, y=48, w=300, h=92)
    // =========================================================================
    lv_obj_t *main_block = lv_obj_create(obj);
    lv_obj_set_pos(main_block, 180, 48);
    lv_obj_set_size(main_block, 300, 92);
    lv_obj_set_style_bg_color(main_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(main_block, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(main_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(main_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(main_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(main_block, LV_OBJ_FLAG_SCROLLABLE);

    // Aktiv-Lux (grosse Zahl, Zeile 1 + 2)
    lv_obj_t *lux_main = lv_label_create(main_block);
    lv_obj_set_pos(lux_main, 0, 0);
    lv_obj_set_size(lux_main, 288, 52);
    lv_obj_set_style_text_font(lux_main, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(lux_main, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(lux_main, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(lux_main, "--- lx");
    lv_label_set_long_mode(lux_main, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_lux_main_lbl = lux_main;

    // Alter / Sequenz (Zeile 3)
    lv_obj_t *age_lbl = lv_label_create(main_block);
    lv_obj_set_pos(age_lbl, 0, 56);
    lv_obj_set_size(age_lbl, 288, 18);
    lv_obj_set_style_text_font(age_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(age_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(age_lbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(age_lbl, "--- ms  #---");
    lv_label_set_long_mode(age_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_lux_age_lbl = age_lbl;

    // =========================================================================
    // REFERENZ-BLOCK (x=0, y=140, w=480, h=50)
    // =========================================================================
    lv_obj_t *ref_block = lv_obj_create(obj);
    lv_obj_set_pos(ref_block, 0, 140);
    lv_obj_set_size(ref_block, 480, 50);
    lv_obj_set_style_bg_color(ref_block, C_BLOCK_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(ref_block, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(ref_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(ref_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ref_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(ref_block, LV_OBJ_FLAG_SCROLLABLE);

    // Referenz-Label (links)
    lv_obj_t *ref_hdr = lv_label_create(ref_block);
    lv_obj_set_pos(ref_hdr, 0, 0);
    lv_obj_set_size(ref_hdr, 60, 16);
    lv_obj_set_style_text_font(ref_hdr, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(ref_hdr, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(ref_hdr, "REF");

    lv_obj_t *ref_lux = lv_label_create(ref_block);
    lv_obj_set_pos(ref_lux, 0, 16);
    lv_obj_set_size(ref_lux, 200, 22);
    lv_obj_set_style_text_font(ref_lux, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(ref_lux, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(ref_lux, "--- lx");
    lv_label_set_long_mode(ref_lux, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_ref_lux_lbl = ref_lux;

    // EV-Abstand (rechts)
    lv_obj_t *ev_hdr = lv_label_create(ref_block);
    lv_obj_set_pos(ev_hdr, 240, 0);
    lv_obj_set_size(ev_hdr, 228, 16);
    lv_obj_set_style_text_font(ev_hdr, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(ev_hdr, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(ev_hdr, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(ev_hdr, "ABSTAND ZUR REFERENZ");

    lv_obj_t *ev_lbl = lv_label_create(ref_block);
    lv_obj_set_pos(ev_lbl, 240, 16);
    lv_obj_set_size(ev_lbl, 228, 22);
    lv_obj_set_style_text_font(ev_lbl, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(ev_lbl, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(ev_lbl, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(ev_lbl, "--- BL");
    lv_label_set_long_mode(ev_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_ev_diff_lbl = ev_lbl;

    // =========================================================================
    // HISTOGRAMM (x=0, y=190, w=480, h=50) — 11 Zonen-Spalten
    // Spaltenbreite: 41px, Abstand: 2px, x-Start: 9px
    // → 11*41 + 10*2 = 451+20 = 471 ≤ 472 (480 - 2*4)
    // =========================================================================
    {
        const lv_coord_t hist_y    = 190;
        const lv_coord_t col_h     = 44;   // innere Spaltenhoehe (inkl. Fuell-Puffer)
        const lv_coord_t col_w     = 41;
        const lv_coord_t col_gap   = 2;
        const lv_coord_t x_start   = 9;

        for (int z = 0; z < 11; ++z) {
            lv_coord_t cx = (lv_coord_t)(x_start + z * (col_w + col_gap));

            // Aeusserer Rahmen-Container (grauer Hintergrund = leere Zone)
            lv_obj_t *col = lv_obj_create(obj);
            lv_obj_set_pos(col, cx, hist_y + 3);
            lv_obj_set_size(col, col_w, col_h);
            lv_obj_set_style_bg_color(col, lv_color_hex(0x1A0A0Au), LV_PART_MAIN);
            lv_obj_set_style_border_width(col, 0, LV_PART_MAIN);
            lv_obj_set_style_radius(col, 2, LV_PART_MAIN);
            lv_obj_set_style_pad_all(col, 0, LV_PART_MAIN);
            lv_obj_set_style_clip_corner(col, true, LV_PART_MAIN);
            lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);
            g_duka_widgets.meas_hist_col[z] = col;

            // Fuell-Rechteck (startet bei voller Hoehe = unsichtbar h=0)
            lv_obj_t *fill = lv_obj_create(col);
            lv_obj_set_pos(fill, 0, col_h);   // initial komplett unten = unsichtbar
            lv_obj_set_size(fill, col_w, 0);
            lv_obj_set_style_bg_color(fill, C_ACCENT_PROGRESS, LV_PART_MAIN);
            lv_obj_set_style_border_width(fill, 0, LV_PART_MAIN);
            lv_obj_set_style_radius(fill, 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(fill, 0, LV_PART_MAIN);
            lv_obj_clear_flag(fill, LV_OBJ_FLAG_SCROLLABLE);
            g_duka_widgets.meas_hist_fill[z] = fill;
        }
    }

    // =========================================================================
    // SESSION-KONTEXT (x=0, y=240, w=480, h=44)
    // =========================================================================
    lv_obj_t *session_block = lv_obj_create(obj);
    lv_obj_set_pos(session_block, 0, 240);
    lv_obj_set_size(session_block, 480, 44);
    lv_obj_set_style_bg_color(session_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(session_block, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_border_width(session_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(session_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(session_block, 4, LV_PART_MAIN);
    lv_obj_clear_flag(session_block, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *session_lbl = lv_label_create(session_block);
    lv_obj_set_pos(session_lbl, 0, 4);
    lv_obj_set_size(session_lbl, 472, 32);
    lv_obj_set_style_text_font(session_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(session_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(session_lbl, "0 Proben  UNDO: ---  E4=MENUE  C6=ZONE");
    lv_label_set_long_mode(session_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.meas_session_lbl = session_lbl;

    tick_screen_page_measurement();
}

FLASHMEM void tick_screen_page_measurement() {
    // B5-BLOCKIERT (Prompt B5, PageMeasurement): create_screen_page_measurement()
    // ist handgeschrieben — keine EEZ-Bindungszustaende registriert.
    // tick_screen_page_measurement() ist ein Stub; EEZ-Runtime kann keine
    // Flow-Variable (MEAS_ACTIVE_SOURCE…MEAS_ZONE_HISTOGRAM, Idx 105-122) in
    // Widget-Updates uebersetzen.
    // Aktiver Pfad: pushWidgetsFromSnapshot() schreibt alle 7 Label-Handles und
    // 22 Histogramm-Handles direkt via lv_label_set_text_static() /
    // lv_obj_set_y() / lv_obj_set_height() / lv_obj_set_style_bg_color().
    // Alle Texte kommen aus UiPresenter/MeasurementValueFormatter
    // (A5 abgeschlossen; kein snprintf() mehr in LvglUi.cpp fuer Measurement).
    // latestZoneIndex-Markierung: direkt implementiert — aktive Spalte C_WARN,
    // restliche C_ACCENT_PROGRESS, Guard sampleCount>0 (kein false highlight).
    // Histogramm-Geometrie/Farbe nicht durch EEZ-Text-Vars abdeckbar —
    // erfordert Presenter-Aufruf im EEZ-Tick oder dedizierte EEZ-Custom-Action.
    // Kein Touch-Callback (actions.h leer) — Touch auf Measurement-Screen inaktiv.
    // Migrationsvoraussetzung: EEZ-Export → Staging → Merge (Abschnitt 13.4).
    void *flowState = getFlowState(0, 4);
    (void)flowState;
}

// ---------------------------------------------------------------------------
// WirelessRemote-Screen — Diagnoseseite fuer ESP32-S3-Gateway und C6-Terminal
// Spec Schritt 9; kein Encoder-Editing, reines Read-only-Layout
// ---------------------------------------------------------------------------
FLASHMEM void create_screen_page_wireless_remote() {
    void *flowState = getFlowState(0, 5);
    (void)flowState;

    lv_obj_t *obj = lv_obj_create(0);
    objects.page_wireless_remote = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    build_header(obj);
    build_modetabs(obj);

    // =========================================================================
    // LINK-BLOCK  (x=0, y=48, w=200, h=92)
    // =========================================================================
    lv_obj_t *link_block = lv_obj_create(obj);
    lv_obj_set_pos(link_block, 0, 48);
    lv_obj_set_size(link_block, 200, 92);
    lv_obj_set_style_bg_color(link_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(link_block, LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_border_width(link_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(link_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(link_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(link_block, LV_OBJ_FLAG_SCROLLABLE);

    // LinkHealth-Chip (Zeile 1)
    lv_obj_t *link_chip = lv_label_create(link_block);
    lv_obj_set_pos(link_chip, 0, 0);
    lv_obj_set_size(link_chip, 188, 20);
    lv_obj_set_style_text_font(link_chip, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(link_chip, C_WARN, LV_PART_MAIN);
    lv_label_set_text(link_chip, "ESP --- ");
    lv_label_set_long_mode(link_chip, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_link_chip_lbl = link_chip;

    // RxAge / TxAge (Zeile 2)
    lv_obj_t *rx_tx = lv_label_create(link_block);
    lv_obj_set_pos(rx_tx, 0, 26);
    lv_obj_set_size(rx_tx, 188, 18);
    lv_obj_set_style_text_font(rx_tx, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(rx_tx, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(rx_tx, "RX ---ms  TX ---ms");
    lv_label_set_long_mode(rx_tx, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_rx_tx_lbl = rx_tx;

    // Remote-Uptime (Zeile 3)
    lv_obj_t *uptime = lv_label_create(link_block);
    lv_obj_set_pos(uptime, 0, 50);
    lv_obj_set_size(uptime, 188, 18);
    lv_obj_set_style_text_font(uptime, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(uptime, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(uptime, "LAUFZEIT ---");
    lv_label_set_long_mode(uptime, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_uptime_lbl = uptime;

    // =========================================================================
    // PEER-BLOCK  (x=200, y=48, w=280, h=92)
    // =========================================================================
    lv_obj_t *peer_block = lv_obj_create(obj);
    lv_obj_set_pos(peer_block, 200, 48);
    lv_obj_set_size(peer_block, 280, 92);
    lv_obj_set_style_bg_color(peer_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(peer_block, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(peer_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(peer_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(peer_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(peer_block, LV_OBJ_FLAG_SCROLLABLE);

    // PeerState-Chip (Zeile 1)
    lv_obj_t *peer_chip = lv_label_create(peer_block);
    lv_obj_set_pos(peer_chip, 0, 0);
    lv_obj_set_size(peer_chip, 268, 20);
    lv_obj_set_style_text_font(peer_chip, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(peer_chip, C_WARN, LV_PART_MAIN);
    lv_obj_set_style_text_align(peer_chip, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(peer_chip, "C6 ---");
    lv_label_set_long_mode(peer_chip, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_peer_chip_lbl = peer_chip;

    // Akku + Zuletzt gesehen (Zeile 2)
    lv_obj_t *battery = lv_label_create(peer_block);
    lv_obj_set_pos(battery, 0, 26);
    lv_obj_set_size(battery, 268, 18);
    lv_obj_set_style_text_font(battery, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(battery, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(battery, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(battery, "AKKU ---%  ZULETZT ---ms");
    lv_label_set_long_mode(battery, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_battery_lbl = battery;

    // Letzter Lux + Sequenz (Zeile 3)
    lv_obj_t *peer_lux = lv_label_create(peer_block);
    lv_obj_set_pos(peer_lux, 0, 50);
    lv_obj_set_size(peer_lux, 268, 18);
    lv_obj_set_style_text_font(peer_lux, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(peer_lux, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(peer_lux, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(peer_lux, "LUX ---  #---");
    lv_label_set_long_mode(peer_lux, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_peer_lux_lbl = peer_lux;

    // =========================================================================
    // DIAGNOSE-BLOCK  (x=0, y=140, w=480, h=58)
    // =========================================================================
    lv_obj_t *diag_block = lv_obj_create(obj);
    lv_obj_set_pos(diag_block, 0, 140);
    lv_obj_set_size(diag_block, 480, 58);
    lv_obj_set_style_bg_color(diag_block, C_BLOCK_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(diag_block, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(diag_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(diag_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(diag_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(diag_block, LV_OBJ_FLAG_SCROLLABLE);

    // Sektions-Header
    lv_obj_t *diag_hdr = lv_label_create(diag_block);
    lv_obj_set_pos(diag_hdr, 0, 0);
    lv_obj_set_size(diag_hdr, 472, 14);
    lv_obj_set_style_text_font(diag_hdr, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(diag_hdr, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(diag_hdr, "DIAGNOSE");

    // DiagCode + Detail + Counter
    lv_obj_t *diag = lv_label_create(diag_block);
    lv_obj_set_pos(diag, 0, 16);
    lv_obj_set_size(diag, 232, 32);
    lv_obj_set_style_text_font(diag, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(diag, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(diag, "CODE ---\nDETAIL ---  N=---");
    lv_label_set_long_mode(diag, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_diag_lbl = diag;

    // TxQueue-Statistik (rechts)
    lv_obj_t *txq = lv_label_create(diag_block);
    lv_obj_set_pos(txq, 236, 16);
    lv_obj_set_size(txq, 236, 32);
    lv_obj_set_style_text_font(txq, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(txq, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(txq, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(txq, "TX WARTEND ---\nRENDER --- CMD ---");
    lv_label_set_long_mode(txq, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_txqueue_lbl = txq;

    // =========================================================================
    // RMT-BLOCK  (x=0, y=198, w=480, h=40)
    // =========================================================================
    lv_obj_t *rmt_block = lv_obj_create(obj);
    lv_obj_set_pos(rmt_block, 0, 198);
    lv_obj_set_size(rmt_block, 480, 40);
    lv_obj_set_style_bg_color(rmt_block, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(rmt_block, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(rmt_block, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(rmt_block, C_PANEL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(rmt_block, 6, LV_PART_MAIN);
    lv_obj_clear_flag(rmt_block, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *rmt_hdr = lv_label_create(rmt_block);
    lv_obj_set_pos(rmt_hdr, 0, 0);
    lv_obj_set_size(rmt_hdr, 100, 14);
    lv_obj_set_style_text_font(rmt_hdr, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(rmt_hdr, C_TEXT_SEC, LV_PART_MAIN);
    lv_label_set_text(rmt_hdr, "RMT-KANAL");

    lv_obj_t *rmt = lv_label_create(rmt_block);
    lv_obj_set_pos(rmt, 0, 16);
    lv_obj_set_size(rmt, 472, 18);
    lv_obj_set_style_text_font(rmt, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(rmt, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(rmt, "AKTIV ---  RETRY ---  TIMEOUT ---  SAETT ---");
    lv_label_set_long_mode(rmt, LV_LABEL_LONG_CLIP);
    g_duka_widgets.remote_rmt_lbl = rmt;

    // =========================================================================
    // HINWEIS-ZEILE  (x=0, y=240, w=480, h=44)
    // =========================================================================
    lv_obj_t *hint = lv_label_create(obj);
    lv_obj_set_pos(hint, 8, 250);
    lv_obj_set_size(hint, 464, 16);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(hint, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(hint, "E3-PRESS = ZURUECK");
    lv_label_set_long_mode(hint, LV_LABEL_LONG_CLIP);

    tick_screen_page_wireless_remote();
}

FLASHMEM void tick_screen_page_wireless_remote() {
    // B6-GEPARKT (Prompt B6, PageWirelessRemote): Seite ist designseitig vorhanden
    // und hat einen vollstaendigen pushWidgetsFromSnapshot()-Direktpfad (16 Handles,
    // snprintf-basiert, keine Presenter-Methode), aber computeTargetScreen()
    // gibt niemals SCREEN_ID_PAGE_WIRELESS_REMOTE zurueck.
    // Kein sicherer Servicepfad vorhanden:
    // - Kein ModeId::WirelessRemote in der Firmware (nicht einfuehren gemaess Prompt).
    // - Kein showWirelessDiag-Flag in SystemSnapshot oder ModeRuntimeState.
    // - Keine Long-Press- oder Gesture-Infrastruktur in InputRouterPolicy.
    // - Kein Debounce/Guard-Konzept fuer Touch-Geste.
    // Definierter spaeterer Runtime-Hook: Bool-Flag `showWirelessDiag` in
    // SystemSnapshot, gesetzt als SETUP-Submodus; computeTargetScreen() prueft
    // nach Setup-Routing; SETUP-Tab bleibt aktiv (kein eigener ModeTab).
    // Rueckkehr: Enc3-Back/Cancel setzt Flag zurueck. Abschnitt 13.11.
    void *flowState = getFlowState(0, 5);
    (void)flowState;
}

// ---------------------------------------------------------------------------
// Busy screen — eigenständiger Header (keine build_header()-Überschreibung der
// SG-Handle-Pointer), Zeit/Fortschritt/SG-Detail/Pause-Overlay.
// ---------------------------------------------------------------------------
FLASHMEM void create_screen_busy() {
    void *flowState = getFlowState(0, 6);
    (void)flowState;

    lv_obj_t *obj = lv_obj_create(0);
    objects.busy = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    // ---- Eigener Header-Streifen (y=0 h=48) --------------------------------
    // Kein build_header()-Aufruf: dieser Screen darf hdr_* nicht überschreiben.
    lv_obj_t *hdr = lv_obj_create(obj);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_size(hdr, 480, 48);
    lv_obj_set_style_bg_color(hdr, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(hdr, C_SECTION_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(hdr, 0, LV_PART_MAIN);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *phase_lbl = lv_label_create(hdr);
    lv_obj_set_pos(phase_lbl, 4, 14);
    lv_obj_set_size(phase_lbl, 132, 24);
    lv_obj_set_style_text_font(phase_lbl, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(phase_lbl, C_TEXT, LV_PART_MAIN);
    lv_label_set_text(phase_lbl, "BELICHTUNG");
    lv_label_set_long_mode(phase_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.busy_hdr_phase_lbl = phase_lbl;

    lv_obj_t *msg_cont = lv_obj_create(hdr);
    lv_obj_set_pos(msg_cont, 140, 0);
    lv_obj_set_size(msg_cont, 230, 48);
    lv_obj_set_style_bg_color(msg_cont, C_MSG_NORMAL, LV_PART_MAIN);
    lv_obj_set_style_radius(msg_cont, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(msg_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(msg_cont, 6, LV_PART_MAIN);
    lv_obj_clear_flag(msg_cont, LV_OBJ_FLAG_SCROLLABLE);
    g_duka_widgets.busy_hdr_msg_cont = msg_cont;

    lv_obj_t *msg_lbl = lv_label_create(msg_cont);
    lv_obj_set_size(msg_lbl, 218, 36);
    lv_obj_align(msg_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(msg_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(msg_lbl, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(msg_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(msg_lbl, "");
    lv_label_set_long_mode(msg_lbl, LV_LABEL_LONG_CLIP);
    g_duka_widgets.busy_hdr_msg_lbl = msg_lbl;

    lv_obj_t *therm = lv_label_create(hdr);
    lv_obj_set_pos(therm, 374, 14);
    lv_obj_set_size(therm, 102, 24);
    lv_obj_set_style_text_font(therm, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(therm, C_WARN, LV_PART_MAIN);
    lv_obj_set_style_text_align(therm, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_label_set_text(therm, "THERM");
    lv_obj_add_flag(therm, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.busy_hdr_thermal_lbl = therm;

    // ---- Zeit-Anzeige (y=58, Montserrat 28, zentriert) ---------------------
    lv_obj_t *time_lbl = lv_label_create(obj);
    lv_obj_set_pos(time_lbl, 0, 58);
    lv_obj_set_size(time_lbl, 480, 56);
    lv_obj_set_style_text_font(time_lbl, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(time_lbl, C_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_align(time_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(time_lbl, "---");
    g_duka_widgets.busy_time_lbl = time_lbl;

    // ---- Fortschrittsbalken (y=122, h=18) -----------------------------------
    lv_obj_t *bar = lv_bar_create(obj);
    lv_obj_set_pos(bar, 40, 122);
    lv_obj_set_size(bar, 400, 18);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x1A0808u), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, C_ACCENT_PROGRESS, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
    lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
    g_duka_widgets.busy_bar = bar;

    // ---- Prozentlabel (y=146) -----------------------------------------------
    lv_obj_t *pct_lbl = lv_label_create(obj);
    lv_obj_set_pos(pct_lbl, 0, 146);
    lv_obj_set_size(pct_lbl, 480, 24);
    lv_obj_set_style_text_font(pct_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(pct_lbl, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(pct_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(pct_lbl, "0%");
    g_duka_widgets.busy_pct_lbl = pct_lbl;

    // ---- SG-Detail (y=178, 2-zeilig, hidden when non-SG) -------------------
    lv_obj_t *sg_detail = lv_label_create(obj);
    lv_obj_set_pos(sg_detail, 20, 178);
    lv_obj_set_size(sg_detail, 440, 38);
    lv_obj_set_style_text_font(sg_detail, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(sg_detail, C_TEXT_SEC, LV_PART_MAIN);
    lv_obj_set_style_text_align(sg_detail, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_long_mode(sg_detail, LV_LABEL_LONG_WRAP);
    lv_label_set_text(sg_detail, "");
    lv_obj_add_flag(sg_detail, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.busy_sg_detail_lbl = sg_detail;

    // ---- Run-Overlay (y=228, h=64, hidden until actively Exposing) ---------
    lv_obj_t *run_overlay = lv_obj_create(obj);
    lv_obj_set_pos(run_overlay, 0, 228);
    lv_obj_set_size(run_overlay, 480, 64);
    lv_obj_set_style_bg_color(run_overlay, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(run_overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(run_overlay, 0, LV_PART_MAIN);
    lv_obj_clear_flag(run_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(run_overlay, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.busy_run_overlay = run_overlay;

    // PAUSE (zentral, bernsteinfarbener Sicherheitsakzent, user_data=3)
    lv_obj_t *pause_btn = lv_obj_create(run_overlay);
    lv_obj_set_pos(pause_btn, 10, 6);
    lv_obj_set_size(pause_btn, 460, 52);
    lv_obj_set_style_bg_color(pause_btn, C_BTN_SAFETY_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(pause_btn, C_BTN_SAFETY_BD, LV_PART_MAIN);
    lv_obj_set_style_border_width(pause_btn, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(pause_btn, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pause_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(pause_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(pause_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(pause_btn, busy_btn_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)3u);
    lv_obj_set_style_bg_color(pause_btn, C_BTN_SAFETY_BD, LV_PART_MAIN | LV_STATE_PRESSED); // C3: pressed visual
    lv_obj_t *pause_lbl = lv_label_create(pause_btn);
    lv_obj_align(pause_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(pause_lbl, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(pause_lbl, C_BTN_SAFETY_FG, LV_PART_MAIN);
    lv_label_set_text(pause_lbl, "PAUSE");

    // ---- Pause-Overlay (y=228, h=64, hidden until Paused) ------------------
    lv_obj_t *overlay = lv_obj_create(obj);
    lv_obj_set_pos(overlay, 0, 228);
    lv_obj_set_size(overlay, 480, 64);
    lv_obj_set_style_bg_color(overlay, C_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(overlay, 0, LV_PART_MAIN);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);
    g_duka_widgets.busy_pause_overlay = overlay;

    // FORTSETZEN (links, grüner Ton, user_data=1)
    lv_obj_t *resume_btn = lv_obj_create(overlay);
    lv_obj_set_pos(resume_btn, 10, 6);
    lv_obj_set_size(resume_btn, 215, 52);
    lv_obj_set_style_bg_color(resume_btn, C_BTN_CONFIRM_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(resume_btn, C_BTN_CONFIRM_BD, LV_PART_MAIN);
    lv_obj_set_style_border_width(resume_btn, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(resume_btn, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(resume_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(resume_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(resume_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(resume_btn, busy_btn_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)1u);
    lv_obj_set_style_bg_color(resume_btn, C_BTN_CONFIRM_BD, LV_PART_MAIN | LV_STATE_PRESSED); // C3: pressed visual
    lv_obj_t *resume_lbl = lv_label_create(resume_btn);
    lv_obj_align(resume_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(resume_lbl, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(resume_lbl, C_BTN_CONFIRM_FG, LV_PART_MAIN);
    lv_label_set_text(resume_lbl, "FORTSETZEN");

    // ABBRECHEN (rechts, roter Ton, user_data=2)
    lv_obj_t *abort_btn = lv_obj_create(overlay);
    lv_obj_set_pos(abort_btn, 255, 6);
    lv_obj_set_size(abort_btn, 215, 52);
    lv_obj_set_style_bg_color(abort_btn, C_BTN_DISCARD_BG, LV_PART_MAIN);
    lv_obj_set_style_border_color(abort_btn, C_BTN_DISCARD_BD, LV_PART_MAIN);
    lv_obj_set_style_border_width(abort_btn, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(abort_btn, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(abort_btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(abort_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(abort_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(abort_btn, busy_btn_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)2u);
    lv_obj_set_style_bg_color(abort_btn, C_BTN_DISCARD_BD, LV_PART_MAIN | LV_STATE_PRESSED); // C3: pressed visual
    lv_obj_t *abort_lbl = lv_label_create(abort_btn);
    lv_obj_align(abort_lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(abort_lbl, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(abort_lbl, C_BTN_DISCARD_FG, LV_PART_MAIN);
    lv_label_set_text(abort_lbl, "ABBRECHEN");

    tick_screen_busy();
}

FLASHMEM void tick_screen_busy() {
    // B7-BLOCKIERT (Prompt B7, BusyScreen): Safety-Gate-Pruefung ergab zwei
    // nicht erfuellte Gates -- keine EEZ-Migration ausgefuehrt.
    //
    // Gate 1 (Prerequisite): Keine nicht-sicherheitskritische Seite wurde
    // bisher erfolgreich in die EEZ-Runtime migriert (B1/B2/B4/B5: BLOCKIERT,
    // B6: GEPARKT). Migration von Busy erfordert eine funktionierende Pilot-
    // seite als Nachweis des EEZ-Tick-Pfades.
    //
    // Gate 2 (tick_screen_busy kann EEZ propagieren): create_screen_busy()
    // ist handgeschrieben -- identischer Blocker wie B1-B5. EEZ-Runtime kann
    // keine Flow-Variable in Widget-Updates uebersetzen, da kein EEZ-generiertes
    // Layout mit Binding-Tabelle vorliegt. Direktpfad in
    // pushWidgetsFromSnapshot() bleibt aktiv und safety-korrekt.
    //
    // Passierte Gates (dokumentiert):
    // - busy_hdr_* Handles getrennt von normalen hdr_* Handles (kein build_header()).
    // - Pause/Resume/Abort puffern nur Ereignisse: busy_btn_cb -->
    //   lvgl_ui_modal_action_callback --> setPendingModalAction --> pollModalAction().
    //   ExposureEngine wird nicht direkt beruehrt.
    // - computeTargetScreen() gibt SCREEN_ID_BUSY absolute Prioritaet
    //   (PreWait/Exposing/Paused/PostWait vor allen anderen Branches).
    //
    // Definierter spaeterer Migration-Hook: Abschnitt 13.12.
    void *flowState = getFlowState(0, 6);
    (void)flowState;
}

typedef void (*tick_screen_func_t)(void);
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_boot,
    tick_screen_page_splitgrade,
    tick_screen_page_paper_workspace,
    tick_screen_page_setup,
    tick_screen_page_measurement,
    tick_screen_page_wireless_remote,
    tick_screen_busy,
};
FLASHMEM void tick_screen(int screen_index) {
    if (screen_index >= 0 && screen_index < 7) {
        tick_screen_funcs[screen_index]();
    }
}
FLASHMEM void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen(screenId - 1);
}

//
// Fonts
//

ext_font_desc_t fonts[] = {
#if LV_FONT_MONTSERRAT_8
    { "MONTSERRAT_8", &lv_font_montserrat_8 },
#endif
#if LV_FONT_MONTSERRAT_10
    { "MONTSERRAT_10", &lv_font_montserrat_10 },
#endif
#if LV_FONT_MONTSERRAT_12
    { "MONTSERRAT_12", &lv_font_montserrat_12 },
#endif
#if LV_FONT_MONTSERRAT_14
    { "MONTSERRAT_14", &lv_font_montserrat_14 },
#endif
#if LV_FONT_MONTSERRAT_16
    { "MONTSERRAT_16", &lv_font_montserrat_16 },
#endif
#if LV_FONT_MONTSERRAT_18
    { "MONTSERRAT_18", &lv_font_montserrat_18 },
#endif
#if LV_FONT_MONTSERRAT_20
    { "MONTSERRAT_20", &lv_font_montserrat_20 },
#endif
#if LV_FONT_MONTSERRAT_22
    { "MONTSERRAT_22", &lv_font_montserrat_22 },
#endif
#if LV_FONT_MONTSERRAT_24
    { "MONTSERRAT_24", &lv_font_montserrat_24 },
#endif
#if LV_FONT_MONTSERRAT_26
    { "MONTSERRAT_26", &lv_font_montserrat_26 },
#endif
#if LV_FONT_MONTSERRAT_28
    { "MONTSERRAT_28", &lv_font_montserrat_28 },
#endif
#if LV_FONT_MONTSERRAT_30
    { "MONTSERRAT_30", &lv_font_montserrat_30 },
#endif
#if LV_FONT_MONTSERRAT_32
    { "MONTSERRAT_32", &lv_font_montserrat_32 },
#endif
#if LV_FONT_MONTSERRAT_34
    { "MONTSERRAT_34", &lv_font_montserrat_34 },
#endif
#if LV_FONT_MONTSERRAT_36
    { "MONTSERRAT_36", &lv_font_montserrat_36 },
#endif
#if LV_FONT_MONTSERRAT_38
    { "MONTSERRAT_38", &lv_font_montserrat_38 },
#endif
#if LV_FONT_MONTSERRAT_40
    { "MONTSERRAT_40", &lv_font_montserrat_40 },
#endif
#if LV_FONT_MONTSERRAT_42
    { "MONTSERRAT_42", &lv_font_montserrat_42 },
#endif
#if LV_FONT_MONTSERRAT_44
    { "MONTSERRAT_44", &lv_font_montserrat_44 },
#endif
#if LV_FONT_MONTSERRAT_46
    { "MONTSERRAT_46", &lv_font_montserrat_46 },
#endif
#if LV_FONT_MONTSERRAT_48
    { "MONTSERRAT_48", &lv_font_montserrat_48 },
#endif
};

//
//
//

FLASHMEM void create_screens() {
    
    eez_flow_init_fonts(fonts, sizeof(fonts) / sizeof(ext_font_desc_t));

// Set default LVGL theme
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    // Initialize screens
    eez_flow_init_screen_names(screen_names, sizeof(screen_names) / sizeof(const char *));
    eez_flow_init_object_names(object_names, sizeof(object_names) / sizeof(const char *));
    
    // Create screens
    create_screen_boot();
    create_screen_page_splitgrade();
    create_screen_page_paper_workspace();
    create_screen_page_setup();
    create_screen_page_measurement();
    create_screen_page_wireless_remote();
    // Busy-Screen wird NACH allen normalen Screens erstellt damit hdr_*-Zeiger
    // aus create_screen_page_splitgrade() gueltig bleiben.
    create_screen_busy();
}