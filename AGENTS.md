# Workspace Guide

This workspace contains active firmware, historical reference trees, and sibling embedded projects.

## Default Target

- New Dukatimer firmware work belongs in [Dukatimer-Part2](Dukatimer-Part2/).
- Read [Dukatimer-Part2/AGENTS.md](Dukatimer-Part2/AGENTS.md) before editing Part2 files.

## Folder Intent

- [Dukatimer v0.3](Dukatimer%20v0.3/) and [Dukatimer v0.9](Dukatimer%20v0.9/) are reference code for legacy behavior and architecture.
- [Dukatimer-Part2/historischer Ursprung](Dukatimer-Part2/historischer%20Ursprung/) is archival source material, not the active firmware line.
- [Wireless TSL2591](Wireless%20TSL2591/), [BeWeMe-C6](BeWeMe-C6/), and [BEWeMe-S3](BEWeMe-S3/) are separate firmware projects. Do not edit them unless the user names that project.

## Working Defaults

- Port behavior forward into Part2 instead of backporting fixes into historical trees.
- Prefer linking to existing docs over copying long project history into new customization files.
- When a task crosses both MCUs, treat `Dukatimer-Part2/lib/SharedProtocol/` as the contract surface and validate both firmware environments.
