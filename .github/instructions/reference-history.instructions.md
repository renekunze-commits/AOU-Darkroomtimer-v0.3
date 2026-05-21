---
description: "Use when editing or reasoning about historical Dukatimer reference code in Dukatimer v0.3, Dukatimer v0.9, the historischer Ursprung sketch, or Wireless TSL2591. These folders are reference-only unless the user explicitly requests changes there."
name: "Historical Reference Code"
applyTo: ["Dukatimer v0.3/**", "Dukatimer v0.9/**", "Dukatimer-Part2/historischer Ursprung/**", "Wireless TSL2591/**"]
---

# Historical Reference Code

- Treat these folders as behavior and protocol references, not the default implementation target.
- When porting behavior, apply the change in `Dukatimer-Part2/` unless the user explicitly asked for a historical branch or the remote probe firmware.
- Use the historical code to confirm edge cases, prior workflows, and protocol intent before copying logic forward.
- If the active implementation target is unclear, move to `Dukatimer-Part2/src/teensy`, `Dukatimer-Part2/src/esp32`, or `Dukatimer-Part2/lib/SharedProtocol` and verify the current Part2 docs first.