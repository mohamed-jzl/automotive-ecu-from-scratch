#!/usr/bin/env python3
"""
UDS diagnostic system test runner.

    # Software-in-the-loop: the production C stack on this PC, no hardware
    python run_diag_tests.py --sil

    # Hardware-in-the-loop: the real ECU through a CAN adapter
    python run_diag_tests.py --interface slcan --channel COM5

    # A subset, with an HTML report
    python run_diag_tests.py --sil --filter DG-01 --report diag_report.html

Exit code 0 only if every test that ran passed. On SIL, each test starts from
a factory-fresh ECU (power cycle, empty flash), so results never depend on the
order the tests run in. On hardware, each test starts by returning to the
default session.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

import can

from diag_client import SESSION_DEFAULT, DiagClient
from diag_test_cases import DIAG_TESTS, DiagTarget
from report_generator import write_html_report
from run_tests import print_result, print_summary
from test_cases import ERROR, FAIL, TestResult

SIL_CHANNEL = "sil"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UDS diagnostic system tests")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--sil", action="store_true",
                        help="test the production C stack compiled for this PC")
    target.add_argument("--interface", choices=["slcan", "pcan", "socketcan", "kvaser", "ixxat"],
                        help="python-can backend for a real ECU")
    parser.add_argument("--channel", default="COM5")
    parser.add_argument("--bitrate", type=int, default=500_000)
    parser.add_argument("--filter", default="")
    parser.add_argument("--report", default="")
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()
    if not args.sil and not args.interface:
        parser.error("choose --sil or --interface")
    return args


def main() -> int:
    args = parse_arguments()

    # udsoncan logs every negative response. Many tests provoke NRCs on
    # purpose and check them explicitly, so that output would only be noise.
    logging.disable(logging.ERROR)

    selected = [t for t in DIAG_TESTS
                if args.filter.lower() in (t.test_id + " " + t.name).lower()]

    print()
    print("=" * 72)
    print("  Body Control ECU - UDS Diagnostic Test Suite")
    print("=" * 72)
    print(f"  Target : {'SIL (production C code on this PC)' if args.sil else f'HIL {args.interface}:{args.channel}'}")
    print(f"  Tester : udsoncan + can-isotp (independent ISO 14229 / 15765 implementations)")
    print(f"  Tests  : {len(selected)}")
    print("=" * 72)
    print()

    sil = None
    if args.sil:
        from sil_bridge import SilEcu
        sil = SilEcu(channel=SIL_CHANNEL).start()
        bus = can.Bus(interface="virtual", channel=SIL_CHANNEL)
    else:
        bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)

    results: list[TestResult] = []
    try:
        with DiagClient(bus) as client:
            target = DiagTarget(client=client, sil=sil)

            for test in selected:
                if sil is not None:
                    sil.power_cycle(erase_nvm=True)
                    sil.reset_count = 0
                    time.sleep(0.05)
                else:
                    try:
                        client.session(SESSION_DEFAULT)
                    except Exception:           # noqa: BLE001 - best effort
                        pass

                result = test.run(target)
                results.append(result)
                print_result(result, args.verbose)
    finally:
        bus.shutdown()
        if sil is not None:
            sil.stop()

    print_summary(results)

    if args.report:
        write_html_report(args.report, results,
                          {"target": "SIL" if args.sil else f"HIL {args.interface}:{args.channel}",
                           "tester": "udsoncan + can-isotp", "suite": "UDS diagnostics"})
        print(f"  HTML report written to {args.report}\n")

    return 1 if any(r.status in (FAIL, ERROR) for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
