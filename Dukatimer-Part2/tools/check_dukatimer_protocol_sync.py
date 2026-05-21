from __future__ import annotations

import sys
from pathlib import Path


env = None


def normalized_text(path: Path) -> str:
	return path.read_text(encoding="utf-8").replace("\r\n", "\n")


def resolve_dukatimer_root() -> Path | None:
	project_dir = None
	if env is not None:
		project_dir = Path(env.subst("$PROJECT_DIR")).resolve()

	candidates = []
	if project_dir is not None:
		candidates.append(project_dir)
		candidates.append(project_dir.parent / "Dukatimer-Part2")
	if "__file__" in globals():
		candidates.append(Path(__file__).resolve().parent.parent)
	current_dir = Path.cwd().resolve()
	candidates.append(current_dir)
	candidates.append(current_dir / "Dukatimer-Part2")
	candidates.append(current_dir.parent / "Dukatimer-Part2")

	for candidate in candidates:
		if (candidate / "lib" / "SharedProtocol" / "DukatimerProtocol.h").exists():
			return candidate

	return None


def verify_sync() -> int:
	# Der C6 baut historisch gegen eine eigene Headerkopie. Dieser Guard stoppt
	# beide Projekte sofort, sobald SharedProtocol und Wireless-Kopie fachlich
	# auseinanderlaufen und damit still unterschiedliche ESP-NOW-ABIs behaupten.
	dukatimer_root = resolve_dukatimer_root()
	if dukatimer_root is None:
		print("Unable to resolve Dukatimer-Part2 root for SharedProtocol sync check.", file=sys.stderr)
		return 1
	shared_header = dukatimer_root / "lib" / "SharedProtocol" / "DukatimerProtocol.h"
	wireless_header = dukatimer_root.parent / "Wireless TSL2591" / "include" / "DukatimerProtocol.h"

	if not shared_header.exists():
		print(f"SharedProtocol header missing: {shared_header}", file=sys.stderr)
		return 1
	if not wireless_header.exists():
		print(f"Wireless protocol header missing: {wireless_header}", file=sys.stderr)
		return 1

	if normalized_text(shared_header) == normalized_text(wireless_header):
		print("SharedProtocol sync check passed.")
		return 0

	print("SharedProtocol sync check failed.", file=sys.stderr)
	print(f"Update {wireless_header} to match {shared_header} exactly.", file=sys.stderr)
	return 1


def main() -> int:
	return verify_sync()


if __name__ == "__main__":
	sys.exit(main())


try:
	Import("env")  # type: ignore[name-defined]
except Exception:
	pass
else:
	if verify_sync() != 0:
		try:
			from SCons.Script import Exit
		except Exception:
			raise SystemExit(1)
		Exit(1)