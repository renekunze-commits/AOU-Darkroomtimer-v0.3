#ifndef UI_MAP_H
#define UI_MAP_H

/* * UI_Map.h - v0.5.10 "Beta Grounding"
 * Zentrale Definitionen für Nextion Pages 0-9 und deren Variablen.
 * Kompatibel mit funktionalem UI-Konzept und HW_Display v0.5.10.
 */

// SEITEN (IDs entsprechen der Reihenfolge im Editor)
// Wir verwenden numerische IDs statt Seitennamen, damit der Seitenwechsel auch dann
// funktioniert, wenn Seiten im HMI umbenannt wurden.
#define PAGE_BOOT       "0"  // Boot Diagnostik
#define PAGE_MAIN       "1"  // BW Modus
#define PAGE_SG         "2"  // Splitgrade Modus
#define PAGE_BURN       "3"  // Burn Modus
#define PAGE_CALIB      "4"  // Papier-Kalibrierung
#define PAGE_TESTSTRIP  "5"  // Probestreifen
#define PAGE_WIZARD     "6"  // Densitometer
#define PAGE_MENU       "7"  // System Setup
#define PAGE_LOCK       "8"  // Belichtungs-Sperrbildschirm
#define PAGE_DIAG       "9"  // Wireless / Bridge Diagnose

// PAGE 0: BOOT DIAGNOSTIK
#define TXT_BOOT_SENS   "tSens"
#define TXT_BOOT_ENV    "tEnv"
#define TXT_BOOT_SW     "tSw"
#define TXT_BOOT_RELAY  "tRelay"
#define TXT_BOOT_C6     "tC6"

// PAGE 1: MAIN (BW)
#define TXT_MAIN_TIME   "tTime"
#define TXT_MAIN_GRADE  "tGrade"
#define NUM_MAIN_SCR    "nScr"

// PAGE 2: SG
#define OBJ_SG_PHASE    "tPhase"
#define OBJ_SG_ZONE     "nZone"
#define OBJ_SG_HIST     "tHist"

// PAGE 3: BURN
#define TXT_BURN_TIME   "tTime"
#define TXT_BURN_GRADE  "tGrade"
#define TXT_BURN_EV     "tEv"

// PAGE 4: CALIB
#define TXT_CAL_STATUS  "tStatus"
#define TXT_CAL_INST    "tInstruction"
#define TXT_CAL_STEP    "tStep"

// PAGE 5: TESTSTRIP
#define TXT_TS_STATUS   "tStatus"
#define TXT_TS_INFO1    "tInfo1"
#define TXT_TS_INFO2    "tInfo2"

// PAGE 6: DENSITOMETER (WIZARD)
#define TXT_DENS_STATUS "tStatus"
#define TXT_DENS_LUX    "tLux"
#define TXT_DENS_VAL    "tDens"

// PAGE 7: SETUP (MENU)
#define TXT_SET_CAT     "tCat"
#define TXT_SET_ITEM    "tItem"
#define TXT_SET_VAL     "tVal"

// PAGE 8: LOCK (BELICHTUNG)
#define TXT_LOCK_STATUS "tStatus"
#define TXT_LOCK_TIME   "tTime"
#define OBJ_LOCK_PROG   "jProgress"

// PAGE 9: DIAGNOSE
#define TXT_DIAG_STATUS "tStatus"
#define TXT_DIAG_G0     "tLuxG0"
#define TXT_DIAG_G5     "tLuxG5"
#define TXT_DIAG_VAL    "tDiag"

// EVENT STRINGS (Vom Nextion zum S3)
#define EVT_MAIN_START   "main_start"
#define EVT_MAIN_SAFE    "main_safe"
#define EVT_MAIN_FOCUS   "main_focus"
#define EVT_MAIN_SCREEN  "main_screen"
#define EVT_MAIN_DENS    "main_dens"
#define EVT_MAIN_APPLY   "main_apply"
#define EVT_MAIN_BRIDGE  "main_bridge"

#define EVT_SG_MEASURE   "sg_measure"
#define EVT_SG_START     "sg_start"
#define EVT_SG_RESET     "sg_reset"

// LEGACY COMMANDS (Support für v0.5.8 HMI)
#define EVT_CMD_START   "bStart"
#define EVT_CMD_SAFE    "bSafe"
#define EVT_CMD_FOCUS   "bFoc"
#define EVT_CMD_SCREEN  "bScr"
#define EVT_CMD_APPLY   "bHash"
#define EVT_CMD_DENS    "bDens"

#endif