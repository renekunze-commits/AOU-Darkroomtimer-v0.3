Import("env")

from pathlib import Path
import re


PROJECT_DIR = Path(env["PROJECT_DIR"])
TEENSY_DIR = PROJECT_DIR / "src" / "teensy"

FORBIDDEN_INCLUDE_PATTERNS = (
    re.compile(r"^\s*#\s*include\s*<SD\.h>", re.MULTILINE),
    re.compile(r"^\s*#\s*include\s*<FS\.h>", re.MULTILINE),
    re.compile(r"^\s*#\s*include\s*<SdFat\.h>", re.MULTILINE),
)
FORBIDDEN_FILE_TOKEN = re.compile(r"\bFile\b")


def is_ignored_line(line: str) -> bool:
    stripped = line.strip()
    return (
        not stripped
        or stripped.startswith("//")
        or stripped.startswith("/*")
        or stripped.startswith("*")
        or stripped.startswith("#")
    )


violations = []
for path in sorted(TEENSY_DIR.rglob("*")):
    if path.suffix not in {".h", ".hpp", ".cpp", ".ino"}:
        continue

    text = path.read_text(encoding="utf-8")
    relative = path.relative_to(PROJECT_DIR).as_posix()
    if path.name == "TeensyStoragePolicy.h":
        continue

    for pattern in FORBIDDEN_INCLUDE_PATTERNS:
        if pattern.search(text):
            violations.append(f"{relative}: forbidden include matching '{pattern.pattern}'")

    for line_number, line in enumerate(text.splitlines(), start=1):
        if is_ignored_line(line):
            continue
        if FORBIDDEN_FILE_TOKEN.search(line):
            violations.append(f"{relative}:{line_number}: forbidden bare File usage -> {line.strip()}")


if violations:
    joined = "\n".join(violations)
    raise SystemExit(
        "Teensy storage policy violation detected. Use src/teensy/TeensyStoragePolicy.h with "
        "TeensyStorageVolume/TeensyStorageFile and avoid SD.h, FS.h, direct SdFat.h includes "
        "or bare File in teensy sources.\n"
        f"{joined}"
    )