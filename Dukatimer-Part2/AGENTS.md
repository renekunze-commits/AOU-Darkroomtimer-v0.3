# Dukatimer-Part2 Guide

## Project Shape

- This is a dual-MCU PlatformIO project. [platformio.ini](platformio.ini) builds [src/teensy](src/teensy/) for `teensy41` and [src/esp32](src/esp32/) for `esp32s3_n16r8` via `build_src_filter`.
- Teensy 4.1 owns UI/TFT/Touch, local encoders 1-3, `ExposureEngine`, NeoPixel/SSR output, paper/storage state, and safety-critical runtime logic.
- ESP32-S3 owns encoder 4, 1-Wire and extra service sensors, service I/O, and the ESP-NOW gateway.
- [lib/SharedProtocol](lib/SharedProtocol/) is the versioned cross-MCU boundary. If you change payloads, enums, or message flow, check both sides.

## Guardrails

- Keep exposure, safety, and other authoritative darkroom logic on the Teensy. The ESP is a service and gateway MCU, not a second owner of exposure state.
- Wireless remote paths stay dumb-terminal style: transport events, render payloads, and measurements, but not their own paper logic or exposure math.
- Do not reintroduce per-workflow EV or F-stop math or UI-side measurement formatting. Reuse the central ownership model in [docs/software/erledigt/dukatimer-part2-architekturzugriffspunkte-math-sensorik-und-workflows.md](docs/software/erledigt/dukatimer-part2-architekturzugriffspunkte-math-sensorik-und-workflows.md).
- [historischer Ursprung](historischer%20Ursprung/) is reference-only unless the user explicitly asks to edit it.

## Validation

- Prefer targeted builds over `default_envs`: use the VS Code tasks in [.vscode/tasks.json](.vscode/tasks.json) or run `pio run -e teensy41` for Teensy work and `pio run -e esp32s3_n16r8` for ESP work.
- If you touch [lib/SharedProtocol](lib/SharedProtocol/), cross-MCU routing, or shared headers under [include](include/), build both environments.
- Automated tests are not established yet; [test/README](test/README) is still the default PlatformIO placeholder. For firmware changes, at least build the affected environment and call out missing hardware validation.
- The Teensy build now enforces the storage policy in [src/teensy/TeensyStoragePolicy.h](src/teensy/TeensyStoragePolicy.h): `SD.h`, `FS.h`, direct `SdFat.h` includes outside the policy header, and bare `File` are not allowed in Teensy sources.

## Docs To Consult First

- Requirements and MCU split: [docs/software/dukatimer-part2-pflichtenheft.md](docs/software/dukatimer-part2-pflichtenheft.md)
- Input, UI, and wireless architecture: [docs/software/dukatimer-part2-grundarchitektur-input-ui-wireless.md](docs/software/dukatimer-part2-grundarchitektur-input-ui-wireless.md)
- Active backlog and slice ordering: [docs/software/dukatimer-part2-migrations-backlog.md](docs/software/dukatimer-part2-migrations-backlog.md) and [docs/software/dukatimer-part2-arbeitspakete-2026-04-29.md](docs/software/dukatimer-part2-arbeitspakete-2026-04-29.md)
- Timing, I2C, and runtime caveats: [docs/head_timing_and_i2c_timeout.md](docs/head_timing_and_i2c_timeout.md)
- Recent implemented slices and validation notes: [CHANGELOG.md](CHANGELOG.md)

## Documentation Notes

- Most project docs are in German while code identifiers are English. Keep new customization text concise and link to the source docs instead of restating them.
