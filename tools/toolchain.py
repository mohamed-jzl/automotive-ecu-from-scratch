"""
Find a host C compiler, so the unit tests and the SIL library build on any PC.

Tried in order:
    1. the CC environment variable, if set
    2. gcc, then clang, on the PATH
    3. zig's bundled clang, via `python -m ziglang cc`

The third option exists for Windows. `pip install ziglang` downloads a
complete, self-contained C compiler into the Python environment - no MSYS2,
no Visual Studio, no administrator rights, nothing added to the system.
"""

from __future__ import annotations

import importlib.util
import os
import shlex
import shutil
import subprocess
import sys
from typing import Optional


def find_c_compiler() -> Optional[list[str]]:
    """Return the compiler as an argument list, or None if none was found."""
    if os.environ.get("CC"):
        return shlex.split(os.environ["CC"])

    for name in ("gcc", "clang"):
        path = shutil.which(name)
        if path:
            return [path]

    if importlib.util.find_spec("ziglang") is not None:
        return [sys.executable, "-m", "ziglang", "cc"]

    return None


def require_c_compiler() -> list[str]:
    compiler = find_c_compiler()
    if compiler is None:
        raise SystemExit(
            "No C compiler found.\n"
            "  Linux/macOS:  install gcc or clang\n"
            "  Windows:      pip install ziglang   (self-contained, no admin needed)\n"
            "  or set the CC environment variable."
        )
    return compiler


def compile_c(compiler: list[str], args: list[str]) -> subprocess.CompletedProcess:
    """Run the compiler and return the completed process (output captured)."""
    return subprocess.run(compiler + args, capture_output=True, text=True)
