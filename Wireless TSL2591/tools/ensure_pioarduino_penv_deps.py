from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path


def executable_in_penv(core_dir: Path, name: str) -> Path:
	exe_suffix = ".exe" if os.name == "nt" else ""
	scripts_dir = "Scripts" if os.name == "nt" else "bin"
	return core_dir / "penv" / scripts_dir / f"{name}{exe_suffix}"


def load_builder_module(builder_script: Path):
	spec = importlib.util.spec_from_file_location("pioarduino_penv_setup", builder_script)
	if spec is None or spec.loader is None:
		raise RuntimeError(f"Cannot load builder helper from {builder_script}")
	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module


def ensure_uv_available(penv_python: Path, penv_uv: Path) -> None:
	if penv_uv.exists():
		return

	# uv-created venvironments koennen ohne pip starten. Der Helper stellt pip nur
	# fuer den Bootstrap sicher und installiert uv dann projektlokal nach.
	subprocess.run([str(penv_python), "-m", "ensurepip", "--default-pip"], check=False)
	subprocess.check_call(
		[
			str(penv_python),
			"-m",
			"pip",
			"install",
			"--disable-pip-version-check",
			"uv>=0.1.0",
		]
	)


def installed_packages(builder_module, penv_uv: Path, penv_python: Path) -> dict[str, object]:
	result = subprocess.run(
		[
			str(penv_uv),
			"pip",
			"list",
			f"--python={penv_python}",
			"--format=json",
		],
		check=True,
		capture_output=True,
		text=True,
		encoding="utf-8",
	)
	packages = json.loads(result.stdout)
	installed: dict[str, object] = {}
	for package_info in packages:
		installed[package_info["name"].lower()] = builder_module.pepver_to_semver(package_info["version"])
	if "platformio" not in installed:
		for alias_name in ("pioarduino-core", "pioarduino"):
			if alias_name in installed:
				installed["platformio"] = installed[alias_name]
				break
	return installed


def packages_to_install(builder_module, installed: dict[str, object]) -> list[str]:
	resolved: list[str] = []
	for package_name in builder_module.get_packages_to_install(builder_module.python_deps, installed):
		spec = builder_module.python_deps[package_name]
		if spec.startswith(("http://", "https://", "git+", "file://")):
			resolved.append(spec)
		else:
			resolved.append(f"{package_name}{spec}")
	return resolved


def ensure_dependencies(core_dir: Path) -> int:
	builder_script = core_dir / "platforms" / "espressif32" / "builder" / "penv_setup.py"
	if not builder_script.exists():
		print(f"Skip pioarduino penv dependency repair: builder script not found at {builder_script}")
		return 0

	penv_python = executable_in_penv(core_dir, "python")
	if not penv_python.exists():
		print(f"Isolated penv python missing: {penv_python}", file=sys.stderr)
		return 1

	penv_uv = executable_in_penv(core_dir, "uv")
	ensure_uv_available(penv_python, penv_uv)
	builder_module = load_builder_module(builder_script)
	current_packages = installed_packages(builder_module, penv_uv, penv_python)
	missing = packages_to_install(builder_module, current_packages)
	if not missing:
		print("pioarduino penv dependencies already satisfied.")
		return 0

	uv_cache_dir = core_dir / ".cache" / "uv"
	uv_cache_dir.mkdir(parents=True, exist_ok=True)
	install_env = dict(os.environ)
	install_env["UV_CACHE_DIR"] = str(uv_cache_dir)
	print("Installing missing pioarduino penv dependencies:")
	for dependency in missing:
		print(f"- {dependency}")
	subprocess.check_call(
		[
			str(penv_uv),
			"pip",
			"install",
			f"--python={penv_python}",
			"--upgrade",
			*missing,
		],
		env=install_env,
	)
	print("pioarduino penv dependencies repaired.")
	return 0


def main() -> int:
	parser = argparse.ArgumentParser()
	parser.add_argument("--core-dir", required=True)
	args = parser.parse_args()
	return ensure_dependencies(Path(args.core_dir).resolve())


if __name__ == "__main__":
	sys.exit(main())