#pragma once
// LVGL Custom-Allocator — leitet den LVGL-Heap auf den Teensy-4.1-PSRAM (EXTRAM).
//
// Warum:
//   Ohne Custom-Allocator verbraucht der statische LV_MEM_SIZE-Pool 128 KB RAM1/DTCM.
//   Mit extmem_malloc landen Widget-Objekte, Stile und LVGL-interne Puffer in PSRAM
//   statt in RAM1 — spart ~128 KB RAM1 und macht z. B. Montserrat 36 wieder nutzbar.
//
// Voraussetzung: Teensy 4.1 mit bestücktem PSRAM (hier 16 MB, via external_psram_size
//   ≥ 1 verifizierbar). Auf einem Board ohne PSRAM würde extmem_malloc auf NULL
//   zurückfallen; in diesem Fall muss der Build-Flag abgesichert werden.
//
// Einbindung:
//   lv_conf.h setzt:
//     #define LV_MEM_CUSTOM           1
//     #define LV_MEM_CUSTOM_INCLUDE   "lvgl_psram_alloc.h"
//     #define LV_MEM_CUSTOM_ALLOC     lv_psram_malloc
//     #define LV_MEM_CUSTOM_REALLOC   lv_psram_realloc
//     #define LV_MEM_CUSTOM_FREE      lv_psram_free

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Teensyduino-Core (imxrt_psram / teensy41) exportiert diese Symbole.
// Explizite Forward-Deklaration damit dieser Header in C- und C++-TUs
// eingebunden werden kann, ohne Arduino.h zu benötigen.
void* extmem_malloc(size_t size);
void* extmem_realloc(void* ptr, size_t size);
void  extmem_free(void* ptr);

#ifdef __cplusplus
}
#endif

#define lv_psram_malloc(size)        extmem_malloc(size)
#define lv_psram_realloc(ptr, size)  extmem_realloc((ptr), (size))
#define lv_psram_free(ptr)           extmem_free(ptr)
