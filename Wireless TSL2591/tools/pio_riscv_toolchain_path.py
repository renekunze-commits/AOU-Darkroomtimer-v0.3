from pathlib import Path
import os

Import("env")


def _compiler_exists(bin_dir: Path) -> bool:
	return (bin_dir / "riscv32-esp-elf-g++.exe").exists() or (bin_dir / "riscv32-esp-elf-g++").exists()


def _candidate_bin_dirs() -> list[Path]:
	platform = env.PioPlatform()
	package_dir = platform.get_package_dir("toolchain-riscv32-esp")
	project_core_dir = Path(env.subst("$PROJECT_CORE_DIR"))
	candidates: list[Path] = []

	if package_dir:
		toolchain_root = Path(package_dir)
		candidates.extend(
			[
				toolchain_root / "bin",
				toolchain_root / "riscv32-esp-elf" / "bin",
			]
		)

	# pioarduino legt die RISC-V-Toolchain je nach Release und Installer unter
	# `packages` oder `tools` ab. Das Projekt normalisiert beide Layouts lokal,
	# damit Builds nicht von einer manuell gesetzten Shell-PATH abhaengen.
	candidates.extend(
		[
			project_core_dir / "packages" / "toolchain-riscv32-esp" / "bin",
			project_core_dir / "packages" / "toolchain-riscv32-esp" / "riscv32-esp-elf" / "bin",
			project_core_dir / "tools" / "toolchain-riscv32-esp" / "bin",
			project_core_dir / "tools" / "toolchain-riscv32-esp" / "riscv32-esp-elf" / "bin",
		]
	)
	return candidates


def _prepend_bin_dir(bin_dir: Path) -> None:
	path_value = str(bin_dir)
	env.PrependENVPath("PATH", path_value)
	os.environ["PATH"] = path_value + os.pathsep + os.environ.get("PATH", "")
	print(f"Resolved RISC-V toolchain path: {path_value}")


seen_paths: set[str] = set()
for candidate in _candidate_bin_dirs():
	normalized = os.path.normcase(os.path.normpath(str(candidate)))
	if normalized in seen_paths:
		continue
	seen_paths.add(normalized)
	if _compiler_exists(candidate):
		_prepend_bin_dir(candidate)
		break