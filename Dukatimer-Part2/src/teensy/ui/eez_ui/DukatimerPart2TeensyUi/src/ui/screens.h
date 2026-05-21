#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_BOOT = 1,
    SCREEN_ID_PAGE_SPLITGRADE = 2,
    SCREEN_ID_PAGE_PAPER_WORKSPACE = 3,
    SCREEN_ID_PAGE_SETUP = 4,
    SCREEN_ID_PAGE_MEASUREMENT = 5,
    SCREEN_ID_PAGE_WIRELESS_REMOTE = 6,
    SCREEN_ID_BUSY = 7,
    _SCREEN_ID_LAST = 7
};

typedef struct _objects_t {
    lv_obj_t *boot;
    lv_obj_t *page_splitgrade;
    lv_obj_t *page_paper_workspace;
    lv_obj_t *page_setup;
    lv_obj_t *page_measurement;
    lv_obj_t *page_wireless_remote;
    lv_obj_t *busy;
} objects_t;

extern objects_t objects;

// Hand-crafted widget handles exposed for direct update from LvglUi::updateSnapshot().
// Zeroed at startup; non-null only after the respective create_screen_xxx() has run.
typedef struct {
    // Shared header (present on every content screen)
    lv_obj_t *hdr_title;         // left: mode/title label
    lv_obj_t *hdr_msg_cont;      // center: message band container (bg colour changes)
    lv_obj_t *hdr_msg_lbl;       // center: message text
    lv_obj_t *hdr_thermal_lbl;   // right: THERM warning chip (hidden when not derating)

    // ModeTabs (shared bottom bar, present on every content screen)
    lv_obj_t *tab_paper;
    lv_obj_t *tab_meas;
    lv_obj_t *tab_print;
    lv_obj_t *tab_setup;

    // PageSplitgrade — SG-panel left (y=48, h=96, w=220)
    lv_obj_t *sg_grade_lbl;      // Grade value, Montserrat 28 (36 gesperrt bis RAM1-Check)
    lv_obj_t *sg_soft_lbl;       // Soft target
    lv_obj_t *sg_hard_lbl;       // Hard target
    lv_obj_t *sg_ctrl_mode_lbl;  // ZEIT / DOSIS chip

    // PageSplitgrade — execution / exposure block (y=144, h=104)
    lv_obj_t *sg_exec_state_lbl; // execution state text (left)
    lv_obj_t *sg_paper_lbl;      // active paper slot + dirty flag (left)
    lv_obj_t *sg_time_lbl;       // remaining time, Montserrat 36 (right)
    lv_obj_t *sg_dose_lbl;       // current/target dose (right)
    lv_obj_t *sg_head_lbl;       // HEAD xx% (right)
    lv_obj_t *sg_lux_lbl;        // measured lux (right)

    // PageSplitgrade — context bar (y=248, h=36)
    lv_obj_t *sg_context_lbl;    // encoder hint / fallback / fault text

    // Boot screen
    lv_obj_t *boot_title_lbl;
    lv_obj_t *boot_subtitle_lbl;
    lv_obj_t *boot_status_lbl;

    // Busy screen — own header (no shared build_header; separate pointers to avoid collision)
    lv_obj_t *busy_hdr_phase_lbl;    // left strip: "BELICHTUNG" / "PAUSE" / ...
    lv_obj_t *busy_hdr_msg_cont;     // center: message band container (dynamic bg)
    lv_obj_t *busy_hdr_msg_lbl;      // center: fault/fallback info text
    lv_obj_t *busy_hdr_thermal_lbl;  // right: THERM chip (hidden when not derating)
    lv_obj_t *busy_time_lbl;         // remaining time or dose progress, Montserrat 28
    lv_obj_t *busy_bar;              // LVGL bar widget 0–100
    lv_obj_t *busy_pct_lbl;          // "47%" label
    lv_obj_t *busy_sg_detail_lbl;    // "SOFT 8.5s offen  ·  HARD 6.2s bereit" (hidden non-SG)
    lv_obj_t *busy_run_overlay;      // container with PAUSE (hidden unless actively Exposing)
    lv_obj_t *busy_pause_overlay;    // container with FORTSETZEN + ABBRECHEN (hidden unless Paused)

    // PagePaperWorkspace — hand-crafted widgets (7-row sliding window for CAL + SELECT panels)
    lv_obj_t *paper_panel_lbl;          // top strip: panel type + slot + state
    lv_obj_t *paper_cal_row_cont[7];    // row background containers (hidden/shown per panel)
    lv_obj_t *paper_cal_row_lbl[7];     // row item label / slot number
    lv_obj_t *paper_cal_row_val[7];     // row value / slot info
    lv_obj_t *paper_hint_lbl;           // bottom encoder-hint bar

    // PageSetup — SETUP-ITEM-LISTE (6 Zeilen) und EDIT-BLOCK
    lv_obj_t *setup_item_row_cont[6];   // Zeilenhintergrund-Container
    lv_obj_t *setup_item_lbl[6];        // Item-Name links
    lv_obj_t *setup_item_val[6];        // Item-Wert rechts
    lv_obj_t *setup_edit_name_lbl;      // Aktuelles Item (Montserrat 14) im Edit-Block
    lv_obj_t *setup_edit_val_lbl;       // Aktueller Wert (Montserrat 20) im Edit-Block
    lv_obj_t *setup_edit_cap_lbl;       // CAP xx% — konfigurierte Helligkeitsgrenze
    lv_obj_t *setup_edit_live_lbl;      // LIVE yy% — effektive Laufzeitleistung
    lv_obj_t *setup_edit_range_lbl;     // Erlaubter Bereich / Thermal-Hinweis
    lv_obj_t *setup_btn_apply;          // Apply-Button
    lv_obj_t *setup_btn_discard;        // Discard-Button
    lv_obj_t *setup_btn_safety;         // SafetyDefaults-Button
    lv_obj_t *setup_hint_lbl;           // Encoder-Hilfe-Zeile im Edit-Block

    // PageMeasurement — QUELLEN-BLOCK, HAUPTMESSWERT, REFERENZ, HISTOGRAMM, SESSION
    lv_obj_t *meas_local_lbl;           // Lokal-Lux-Wert im Quellen-Block
    lv_obj_t *meas_wireless_lbl;        // Wireless-Lux-Wert im Quellen-Block
    lv_obj_t *meas_source_chip_lbl;     // Aktive Quelle (LOKAL / WIRELESS / KEINE)
    lv_obj_t *meas_lux_main_lbl;        // Hauptmesswert, Montserrat 28
    lv_obj_t *meas_lux_age_lbl;         // Alter in ms / Sequenznummer
    lv_obj_t *meas_ref_lux_lbl;         // Referenz-Lux
    lv_obj_t *meas_ev_diff_lbl;         // Relativer EV-Abstand in Blenden
    lv_obj_t *meas_hist_col[11];        // Histogramm-Spalten-Container (fixer Rahmen)
    lv_obj_t *meas_hist_fill[11];       // Histogramm-Fuellrechtecke (Hoehe variabel)
    lv_obj_t *meas_session_lbl;         // Probenzaehler / Undo-Chip / Hinweis

    // PagePaperWorkspace — Panel-Switcher-Chips (SELECT / CAL)
    lv_obj_t *paper_chip_select;        // SELECT-Chip (aktiv = hell, inaktiv = gedaempft)
    lv_obj_t *paper_chip_cal;           // CAL-Chip

    // PageWirelessRemote — Diagnoseseite (read-only)
    lv_obj_t *remote_link_chip_lbl;     // LinkHealth-Chip (VERBUNDEN / DEGRADIERT / GETRENNT)
    lv_obj_t *remote_rx_tx_lbl;         // RxAge ms / TxAge ms
    lv_obj_t *remote_uptime_lbl;        // Remote-Uptime
    lv_obj_t *remote_peer_chip_lbl;     // PeerState-Chip (ONLINE / OFFLINE)
    lv_obj_t *remote_battery_lbl;       // Akku % + Zuletzt gesehen
    lv_obj_t *remote_peer_lux_lbl;      // Letzter Lux-Wert + Messequenz
    lv_obj_t *remote_diag_lbl;          // DiagCode + Detail + Counter (kombiniert)
    lv_obj_t *remote_txqueue_lbl;       // TxQueue: Pending / Render-Verluste / Cmd-Verluste
    lv_obj_t *remote_rmt_lbl;           // RMT: InFlight / Retry / Timeout / Saettigung
} DukaWidgets;

extern DukaWidgets g_duka_widgets;

void create_screen_boot();
void tick_screen_boot();

void create_screen_page_splitgrade();
void tick_screen_page_splitgrade();

void create_screen_page_paper_workspace();
void tick_screen_page_paper_workspace();

void create_screen_page_setup();
void tick_screen_page_setup();

void create_screen_page_measurement();
void tick_screen_page_measurement();

void create_screen_page_wireless_remote();
void tick_screen_page_wireless_remote();

void create_screen_busy();
void tick_screen_busy();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/