from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


def run(command: list[str]) -> int:
	print("Running:", " ".join(command))
	completed = subprocess.run(command, cwd=PROJECT_ROOT)
	return completed.returncode


def find_host_compiler() -> list[str] | None:
	for candidate in ("g++", "clang++"):
		resolved = shutil.which(candidate)
		if resolved:
			return [resolved]

	resolved = shutil.which("cl.exe")
	if resolved:
		return [resolved]

	return None


def run_with_gnu_compiler(compiler: list[str]) -> int:
	output_dir = PROJECT_ROOT / ".pio" / "build" / "ap11_measurement_domain"
	output_dir.mkdir(parents=True, exist_ok=True)
	output_binary = output_dir / ("ap11_measurement_domain.exe" if sys.platform.startswith("win") else "ap11_measurement_domain")
	compile_command = compiler + [
		"-std=c++17",
		"-Wall",
		"-Wextra",
		"-pedantic",
		"-I",
		str(PROJECT_ROOT / "test" / "support"),
		"-I",
		str(PROJECT_ROOT / "lib" / "SharedProtocol"),
		"-I",
		str(PROJECT_ROOT / "src" / "teensy"),
		str(PROJECT_ROOT / "test" / "ap11_measurement_domain" / "test_main.cpp"),
		"-o",
		str(output_binary),
	]
	if run(compile_command) != 0:
		return 1
	return run([str(output_binary)])


def run_with_msvc(compiler: list[str]) -> int:
	output_dir = PROJECT_ROOT / ".pio" / "build" / "ap11_measurement_domain"
	output_dir.mkdir(parents=True, exist_ok=True)
	output_binary = output_dir / "ap11_measurement_domain.exe"
	compile_command = compiler + [
		"/nologo",
		"/std:c++17",
		"/EHsc",
		"/W4",
		f"/I{PROJECT_ROOT / 'test' / 'support'}",
		f"/I{PROJECT_ROOT / 'lib' / 'SharedProtocol'}",
		f"/I{PROJECT_ROOT / 'src' / 'teensy'}",
		str(PROJECT_ROOT / "test" / "ap11_measurement_domain" / "test_main.cpp"),
		f"/Fe:{output_binary}",
	]
	if run(compile_command) != 0:
		return 1
	return run([str(output_binary)])


def run_syntax_only_fallback() -> int:
	compiler = PROJECT_ROOT / ".pio" / "build" / "teensy41" / ".." / ".." / ".." / ".." / \
		".platformio" / "packages" / "toolchain-gccarmnoneeabi-teensy" / "bin" / "arm-none-eabi-g++.exe"
	compiler = compiler.resolve()
	if not compiler.exists():
		compiler = Path.home() / ".platformio" / "packages" / "toolchain-gccarmnoneeabi-teensy" / "bin" / "arm-none-eabi-g++.exe"
	if not compiler.exists():
		print("No host compiler and no Teensy syntax compiler found.", file=sys.stderr)
		return 1

	compile_command = [
		str(compiler),
		"-std=gnu++17",
		"-fsyntax-only",
		"-Wall",
		"-Wextra",
		"-I",
		str(PROJECT_ROOT / "test" / "support"),
		"-I",
		str(PROJECT_ROOT / "lib" / "SharedProtocol"),
		"-I",
		str(PROJECT_ROOT / "src" / "teensy"),
		str(PROJECT_ROOT / "test" / "ap11_measurement_domain" / "test_main.cpp"),
	]
	result = run(compile_command)
	if result == 0:
		print("Host compiler not found; AP11 harness passed syntax-only validation with Teensy ARM compiler.")
	return result


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parent.parent


def main() -> int:
	# Der Runner bevorzugt einen echten Host-Compiler, damit der Harness seine
	# Invarianten sofort ausfuehrt. Falls die Arbeitsstation nur die PlatformIO-
	# Toolchain besitzt, faellt er bewusst auf Syntax-Only zurueck statt den Test
	# ganz unbemerkt auszulassen.
	host_compiler = find_host_compiler()
	if host_compiler is None:
		return run_syntax_only_fallback()
	if host_compiler[0].lower().endswith("cl.exe"):
		return run_with_msvc(host_compiler)
	return run_with_gnu_compiler(host_compiler)


if __name__ == "__main__":
	sys.exit(main())