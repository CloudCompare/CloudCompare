#!/usr/bin/env python3

from __future__ import annotations

import re
import sys
from pathlib import Path


BUNDLES = (
    (
        "CloudCompare",
        Path("qCC/Mac/CloudCompare.plist"),
        Path("qCC/Mac/CMakeLists.txt"),
    ),
    (
        "ccViewer",
        Path("ccViewer/Mac/ccViewer.plist"),
        Path("ccViewer/Mac/CMakeLists.txt"),
    ),
)

PLACEHOLDER = "${MACOSX_BUNDLE_GUI_IDENTIFIER}"
IDENTIFIER_RE = re.compile(r'MACOSX_BUNDLE_GUI_IDENTIFIER\s+"[^"]+"')


def main() -> int:
    repo_root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.cwd()
    errors: list[str] = []

    for bundle_name, plist_relpath, cmake_relpath in BUNDLES:
        plist_path = repo_root / plist_relpath
        cmake_path = repo_root / cmake_relpath

        plist_text = plist_path.read_text(encoding="utf-8")
        if PLACEHOLDER not in plist_text:
            errors.append(
                f"{bundle_name}: expected {plist_relpath} to reference {PLACEHOLDER}"
            )

        cmake_text = cmake_path.read_text(encoding="utf-8")
        if not IDENTIFIER_RE.search(cmake_text):
            errors.append(
                f"{bundle_name}: expected {cmake_relpath} to set a non-empty "
                "MACOSX_BUNDLE_GUI_IDENTIFIER"
            )

    if errors:
        print("macOS bundle identifier verification failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print("macOS bundle identifiers are configured for all checked app bundles.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
