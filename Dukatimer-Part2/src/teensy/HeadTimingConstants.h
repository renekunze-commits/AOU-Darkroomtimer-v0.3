/*
 * HeadTimingConstants
 *
 * Gemeinsame Timing-Konstanten fuer den Lichtkopf.
 * Diese Werte muessen zwischen ExposureEngine (praediktiver Shutoff)
 * und NeoPixelHead (tatsaechliche Fade-/Ausgabezeiten) konsistent bleiben.
 */
#pragma once

#include <stdint.h>

namespace dukatimer {

// Beobachtete Bus-/Latch-Latenz bis ein Abschaltbefehl am Kopf wirksam ist.
constexpr uint32_t kHeadBusLatencyMs = 8;

// Vorgabe gemaess Laufzeitspezifikation/Audit:
// Fade-Out-Vorlauf fuer den Head-Shutoff wird explizit mit 100 ms gefuehrt.
constexpr uint32_t kHeadFadeOutMs = 100;

// Gemeinsame Soft-Stop-Dauer des Heads (Transition auf Off).
constexpr uint16_t kHeadSoftStopMs = static_cast<uint16_t>(kHeadFadeOutMs);

// Baseline-Lead fuer den praediktiven Shutoff aus Bus-Latenz + Fade-Out.
constexpr uint32_t kHeadPredictiveShutoffBaseLeadMs = kHeadBusLatencyMs + kHeadFadeOutMs;

// Konservative Obergrenze fuer gemessene present()-Dauern (us -> ms),
// damit sporadische Ausreisser die Laufzeit-Latenz nicht unbegrenzt aufblasen.
constexpr uint32_t kHeadPresentDurationClampMs = 250;

static_assert(kHeadFadeOutMs <= 0xFFFFu, "kHeadFadeOutMs must fit into uint16_t");

}  // namespace dukatimer
