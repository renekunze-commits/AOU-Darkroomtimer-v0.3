#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

// GCC LTO demotes static-const font glyph_bitmap arrays from .rodata to .data
// (DTCM), wasting ~76KB of RAM1 on Teensy 4.1. On Teensy 4.x the linker
// script maps .rodata into the .data segment (copied to DTCM), so .rodata is
// NOT Flash-only. Use .progmem which stays in Flash (> FLASH, not AT> FLASH),
// saving the DTCM cost and serving glyph bitmaps via DCACHE from 0x60xxxxxx.
#ifndef LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_CONST __attribute__((section(".progmem")))
#endif

#if defined(__IMXRT1062__)
#ifndef LV_ATTRIBUTE_LARGE_RAM_ARRAY
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY __attribute__((section(".externalram")))
#endif
#endif

#define LV_MEM_CUSTOM 1
// LVGL-Heap liegt im Teensy-4.1-PSRAM (EXTRAM) statt in RAM1/DTCM.
// extmem_malloc fällt bei fehlendem PSRAM auf NULL zurück — Board hat 16 MB bestückt.
#define LV_MEM_CUSTOM_INCLUDE "lvgl_psram_alloc.h"
#define LV_MEM_CUSTOM_ALLOC   lv_psram_malloc
#define LV_MEM_CUSTOM_REALLOC lv_psram_realloc
#define LV_MEM_CUSTOM_FREE    lv_psram_free

#define LV_DISP_DEF_REFR_PERIOD 16
#define LV_INDEV_DEF_READ_PERIOD 12

#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1

#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0

#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1
#define LV_THEME_DEFAULT_GROW 1

#define LV_USE_FLEX 1
#define LV_USE_GRID 1

#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_28 1

#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_OBJ 1

#endif