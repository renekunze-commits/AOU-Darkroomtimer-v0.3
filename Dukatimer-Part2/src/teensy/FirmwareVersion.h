#pragma once

namespace dukatimer::build {

// Zentrale Versionsquelle fuer die aktuelle Teensy-Basis.
// Diese Konstanten werden bewusst an einer Stelle gepflegt, damit Debug-UI,
// Serial-Bootlog und spaetere Diagnosepfade dieselbe Versionskennung zeigen.
inline constexpr const char* kFirmwareName = "Dukatimer Part2 Teensy";
inline constexpr const char* kFirmwareVersion = "0.2.47-dev";
inline constexpr const char* kFirmwareStage = "RAM1 audit + EEZ Busy-Screen";
inline constexpr const char* kDisplayBackend = "ILI9488_t3_mm shim";
inline constexpr const char* kUiRuntime = "LVGL 8.x";
inline constexpr const char* kUiDesigner = "EEZ Studio ready";

inline constexpr unsigned kVersionMajor = 0;
inline constexpr unsigned kVersionMinor = 2;
inline constexpr unsigned kVersionPatch = 46;

}  // namespace dukatimer::build