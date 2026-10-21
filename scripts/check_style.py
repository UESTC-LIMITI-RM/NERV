#!/usr/bin/env python3
"""Check NERV text files for encoding, line ending, tabs, trailing whitespace and punctuation rules."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SKIP_DIRS = {".git", "build", "__pycache__"}
BINARY_SUFFIXES = {
    ".png", ".jpg", ".jpeg", ".gif", ".pdf", ".zip", ".7z", ".pyc",
}

# 这些区间内的字符属于全角标点, 弯引号, 破折号, 省略号与非半角空白等, 禁止出现在代码, 注释, 提交信息与文档中.
FORBIDDEN_RANGES = (
    (0x00A0, 0x00A0, "non-breaking space"),
    (0x200B, 0x200D, "zero-width character"),
    (0x3000, 0x303F, "CJK punctuation"),
    (0xFEFF, 0xFEFF, "byte order mark"),
    (0xFF01, 0xFF5E, "full-width ASCII"),
    (0x2012, 0x2015, "dash"),
    (0x2018, 0x201F, "curly quote"),
    (0x2026, 0x2026, "ellipsis"),
)


def is_forbidden(char: str) -> bool:
    codepoint = ord(char)
    for start, end, _name in FORBIDDEN_RANGES:
        if start <= codepoint <= end:
            return True
    return False


def check_file(path: Path) -> list[tuple[int, int, str]]:
    errors: list[tuple[int, int, str]] = []
    data = path.read_bytes()

    if data.startswith(b"\xef\xbb\xbf"):
        errors.append((1, 1, "UTF-8 BOM is not allowed"))
        data = data[3:]

    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        errors.append((1, 1, f"not valid UTF-8: {exc}"))
        return errors

    if text and not text.endswith("\n"):
        errors.append((text.count("\n") + 1, 1, "missing final newline"))

    for line_number, line in enumerate(text.split("\n"), 1):
        if "\r" in line:
            errors.append((line_number, 1, "CR or CRLF line ending found"))
        if "\t" in line:
            column = line.index("\t") + 1
            errors.append((line_number, column, "tab character found"))
        if line != line.rstrip():
            errors.append((line_number, len(line.rstrip()) + 1, "trailing whitespace"))
        for column, char in enumerate(line, 1):
            if is_forbidden(char):
                errors.append(
                    (line_number, column, f"forbidden character U+{ord(char):04X}")
                )
                break

    return errors


def iter_files(root: Path):
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        if path.suffix.lower() in BINARY_SUFFIXES:
            continue
        yield path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", default=["."], help="files or directories to check")
    args = parser.parse_args()

    roots = [Path(item) for item in args.paths]
    files: list[Path] = []
    for root in roots:
        if root.is_file():
            files.append(root)
        elif root.is_dir():
            files.extend(iter_files(root))
        else:
            print(f"error: path not found: {root}")
            return 2

    total_errors = 0
    for path in files:
        for line, column, message in check_file(path):
            print(f"{path}:{line}:{column}: {message}")
            total_errors += 1

    if total_errors:
        print(f"style check failed: {total_errors} error(s) in {len(files)} file(s)")
        return 1

    print(f"style check passed: {len(files)} file(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
