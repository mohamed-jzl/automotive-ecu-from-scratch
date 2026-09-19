#!/usr/bin/env python3
"""
Build and run every unit test suite - without make.

Equivalent to `cd tests && make`, for machines that have a C compiler but no
make (typically Windows). Uses the same flags as tests/Makefile, including
-Werror, so a clean run here means a clean run in CI.

    python tools/run_unit_tests.py              # all suites
    python tools/run_unit_tests.py isotp uds    # suites whose name matches

On Windows with no compiler at all:  pip install ziglang
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

from toolchain import compile_c, require_c_compiler

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"
FW = ROOT / "firmware" / "automotive-body-ecu" / "src"

# Keep in sync with SRC_<suite> in tests/Makefile.
SUITES: dict[str, list[Path]] = {
    "test_vehicle_state_machine": [FW / "app" / "vehicle_state_machine.c"],
    "test_fault_manager":         [FW / "services" / "fault_manager.c"],
    "test_can_signals":           [FW / "services" / "can_signals.c"],
    "test_adc_conversion":        [],
    "test_isotp":                 [FW / "diag" / "isotp.c"],
    "test_uds_server":            [FW / "diag" / "uds_server.c",
                                   FW / "diag" / "dtc_manager.c",
                                   FW / "diag" / "uds_security.c"],
    "test_uds_security":          [FW / "diag" / "uds_security.c"],
    "test_dtc_manager":           [FW / "diag" / "dtc_manager.c"],
    "test_nvm_store":             [FW / "services" / "nvm_store.c"],
}

CFLAGS = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-g", "-O0",
          f"-I{TESTS / 'framework'}"] + \
         [f"-I{FW / d}" for d in ("app", "diag", "services", "drivers", "config")]


def main() -> int:
    compiler = require_c_compiler()
    filters = [a.lower() for a in sys.argv[1:]]
    selected = [s for s in SUITES if not filters or any(f in s for f in filters)]

    print(f"Compiler: {' '.join(compiler)}\n")
    failed: list[str] = []

    with tempfile.TemporaryDirectory() as build_dir:
        for suite in selected:
            exe = Path(build_dir) / (suite + (".exe" if sys.platform == "win32" else ""))
            sources = [TESTS / f"{suite}.c", TESTS / "framework" / "unity_min.c"] + SUITES[suite]

            result = compile_c(compiler, CFLAGS + ["-o", str(exe)] + [str(s) for s in sources])
            if result.returncode != 0:
                print(f"  BUILD FAILED  {suite}\n{result.stdout}{result.stderr}")
                failed.append(suite)
                continue

            run = subprocess.run([str(exe)], capture_output=True, text=True)
            # Print only the suite header, failures and the summary line.
            for line in run.stdout.splitlines():
                if "[PASS]" not in line:
                    print(line)
            if run.returncode != 0:
                failed.append(suite)

    total = len(selected)
    print(f"{total - len(failed)} of {total} suites passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
