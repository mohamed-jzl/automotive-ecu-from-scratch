#!/usr/bin/env python3
"""
Test runner for the Automotive Body Control ECU.

Usage
-----
    # Develop the framework with no hardware connected
    python run_tests.py --interface virtual

    # Run against a real ECU through a CANable / CANtact adapter
    python run_tests.py --interface slcan --channel COM5

    # Run against a PEAK PCAN-USB
    python run_tests.py --interface pcan --channel PCAN_USBBUS1

    # Linux, native or virtual CAN
    python run_tests.py --interface socketcan --channel can0

    # Run a subset and write an HTML report
    python run_tests.py --interface slcan --channel COM5 \
                        --filter TC-00 --report report.html

Exit codes
----------
    0   every test that ran, passed
    1   at least one test failed or errored
    2   the bus could not be opened

The exit code is what lets this run unattended in a pipeline. A test suite
that only prints its results, and never fails a build, protects nothing.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime

from can_interface import EcuCanInterface
from report_generator import write_html_report
from test_cases import ALL_TESTS, ERROR, FAIL, PASS, SKIP, TestResult

# ANSI colours, disabled automatically when the output is redirected to a file.
_COLOUR = sys.stdout.isatty()


def _paint(text: str, code: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _COLOUR else text


STATUS_STYLE = {
    PASS:  lambda s: _paint(s, "32"),   # green
    FAIL:  lambda s: _paint(s, "31"),   # red
    SKIP:  lambda s: _paint(s, "33"),   # yellow
    ERROR: lambda s: _paint(s, "35"),   # magenta
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="System test suite for the Body Control ECU",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--interface", default="virtual",
        choices=["virtual", "slcan", "pcan", "socketcan", "kvaser", "ixxat"],
        help="python-can backend (default: virtual, needs no hardware)",
    )
    parser.add_argument(
        "--channel", default="test",
        help="adapter channel, e.g. COM5, can0, PCAN_USBBUS1",
    )
    parser.add_argument(
        "--bitrate", type=int, default=500_000,
        help="bus bit rate in bit/s (default: 500000)",
    )
    parser.add_argument(
        "--filter", default="",
        help="run only tests whose id or name contains this text",
    )
    parser.add_argument(
        "--report", default="",
        help="write an HTML report to this path",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="print the evidence collected by each test",
    )
    return parser.parse_args()


def print_header(args: argparse.Namespace, test_count: int) -> None:
    print()
    print("=" * 72)
    print("  Automotive Body Control ECU - System Test Suite")
    print("=" * 72)
    print(f"  Interface : {args.interface} on '{args.channel}' @ {args.bitrate} bit/s")
    print(f"  Started   : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Tests     : {test_count}")
    print("=" * 72)
    print()

    if args.interface == "virtual":
        print("  NOTE: the virtual backend has no ECU attached, so every test")
        print("        will report SKIP. This mode exists to exercise the test")
        print("        framework itself, not to verify the ECU.")
        print()


def print_result(result: TestResult, verbose: bool) -> None:
    style = STATUS_STYLE.get(result.status, lambda s: s)

    print(f"  [{style(result.status):<4}] {result.test_id}  {result.name}")
    print(f"         requirement {result.requirement}   ({result.duration_s:.2f} s)")

    if verbose and result.evidence:
        for line in result.evidence:
            print(f"         | {line}")

    if result.message:
        for line in result.message.splitlines():
            print(f"         > {line}")

    print()


def print_summary(results: list[TestResult]) -> None:
    counts = {status: sum(1 for r in results if r.status == status)
              for status in (PASS, FAIL, SKIP, ERROR)}

    print("-" * 72)
    print(f"  {len(results)} tests: "
          f"{counts[PASS]} passed, {counts[FAIL]} failed, "
          f"{counts[SKIP]} skipped, {counts[ERROR]} errors")

    # Requirement coverage is the number the V-model actually cares about:
    # how many requirements have been demonstrated, not how many tests ran.
    verified = {r.requirement for r in results if r.status == PASS}
    total = {r.requirement for r in results}
    print(f"  requirements verified: {len(verified)} of {len(total)}")

    if counts[FAIL] or counts[ERROR]:
        verdict = _paint("FAILED", "31")
    elif counts[PASS] == 0:
        verdict = _paint("INCONCLUSIVE (nothing was verified)", "33")
    else:
        verdict = _paint("PASSED", "32")

    print(f"  RESULT: {verdict}")
    print("=" * 72)
    print()


def main() -> int:
    args = parse_arguments()

    selected = [
        test for test in ALL_TESTS
        if args.filter.lower() in (test.test_id + " " + test.name).lower()
    ]

    if not selected:
        print(f"No tests match filter '{args.filter}'", file=sys.stderr)
        return 2

    print_header(args, len(selected))

    try:
        ecu = EcuCanInterface(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
        )
        ecu.connect()
    except Exception as exc:                            # noqa: BLE001
        print(f"  Could not open the CAN bus: {exc}", file=sys.stderr)
        print("\n  Check that the adapter is plugged in, that the channel name "
              "is correct,\n  and that no other program is holding the port.\n",
              file=sys.stderr)
        return 2

    results: list[TestResult] = []

    try:
        for test in selected:
            # Clear buffered traffic between tests so one test's stimulus can
            # never satisfy the next test's assertion.
            ecu.flush()

            result = test.run(ecu)
            results.append(result)
            print_result(result, args.verbose)
    finally:
        ecu.disconnect()

    print_summary(results)

    if args.report:
        write_html_report(args.report, results, vars(args))
        print(f"  HTML report written to {args.report}\n")

    failed = any(r.status in (FAIL, ERROR) for r in results)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
