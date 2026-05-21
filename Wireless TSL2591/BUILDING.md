# Isolierte C6-Build-Lane

Dieses Projekt nutzt fuer `c6_zero` einen eigenen pioarduino-Core unter
`.pioarduino-core/`. Dadurch bleibt der globale PlatformIO-Core fuer andere
Workspace-Projekte wie Dukatimer-Part2 unberuehrt.

## Einstiegspunkte

- `Bootstrap isolated pioarduino core`: legt den projektlokalen Core an.
- `Resolve Wireless TSL2591 packages (isolated C6)`: zieht Plattform und
  Bibliotheken fuer `c6_zero` in den isolierten Core.
- `Build Wireless TSL2591 (isolated C6)`: normaler Build fuer das C6-Projekt.
- `Upload Wireless TSL2591 (isolated C6)`: Upload ueber dieselbe isolierte Lane.

## Technische Guardrails

- Alle VS-Code-Tasks rufen `tools/invoke_pioarduino_core.ps1` auf statt eines
  globalen `pio` oder `platformio.exe`.
- `tools/invoke_pioarduino_core.ps1` bootstrapt den pioarduino-Core nur in
  `.pioarduino-core/` und setzt fuer jeden Aufruf `PLATFORMIO_CORE_DIR` passend.
- Der Wrapper repariert die projektlokale `penv` bei Bedarf ueber
  `tools/ensure_pioarduino_penv_deps.py`, weil der pioarduino-Builder in dieser
  Lane den eigenen Python-Dependency-Install nicht immer stabil selbst beendet.
- Unter Windows legt derselbe Wrapper fuer Child-Prozesse zusaetzlich einen
  kurzen `subst`-Alias auf das Projekt an, damit das Entpacken langer
  `esp32-core-*-libs`-Pfade unterhalb der `MAX_PATH`-Grenze bleibt.
- Falls das entpackte `framework-arduinoespressif32-libs`-Paket ohne
  top-level `package.json` ankommt, erzeugt der Wrapper ein minimales Manifest
  und startet den PlatformIO-Aufruf einmal neu.
- `tools/pio_riscv_toolchain_path.py` normalisiert weiterhin den RISC-V-
  Toolchain-Pfad innerhalb des isolierten Cores.

## Reset

Zum sauberen Neuaufsetzen reicht es, `.pioarduino-core/` zu loeschen und danach
erneut `Bootstrap isolated pioarduino core` auszufuehren.
