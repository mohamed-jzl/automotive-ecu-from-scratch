#!/usr/bin/env python3
"""
Command-line diagnostic tester for the Body Control ECU - a small workshop tool.

    python diag_tool.py --sil demo                       # guided demo, no hardware
    python diag_tool.py --interface slcan --channel COM5 info
    python diag_tool.py --interface slcan --channel COM5 dtc
    python diag_tool.py --interface slcan --channel COM5 clear
    python diag_tool.py --interface slcan --channel COM5 write-vin WDB2030461A123456
    python diag_tool.py --interface slcan --channel COM5 lamp-test
    python diag_tool.py --interface slcan --channel COM5 reset

Commercial equivalents are Vector CANape/CANoe diagnostic consoles or a
dealer tester; this does the same basic jobs on top of the open-source
udsoncan library.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

import can
from udsoncan.exceptions import NegativeResponseException

from diag_client import (
    DID_ACTIVE_SESSION, DID_BATTERY_VOLTAGE, DID_ECU_SERIAL, DID_FAULT_BITMASK,
    DID_SOFTWARE_VERSION, DID_UPTIME, DID_VEHICLE_STATE, DID_VIN,
    FAULT_BATTERY_UNDERVOLTAGE, RID_LAMP_SELF_TEST, VEHICLE_STATES, DiagClient,
    describe_status, nrc_name, nrc_of,
)


def show_info(c: DiagClient) -> None:
    v = c.read_many([DID_VIN, DID_SOFTWARE_VERSION, DID_ECU_SERIAL, DID_ACTIVE_SESSION])
    live = c.read_many([DID_BATTERY_VOLTAGE, DID_VEHICLE_STATE, DID_FAULT_BITMASK, DID_UPTIME])
    major, minor, patch = v[DID_SOFTWARE_VERSION]
    serial = v[DID_ECU_SERIAL]

    print("  ECU identification")
    print(f"    VIN               {v[DID_VIN]}")
    print(f"    Software version  {major}.{minor}.{patch}")
    print(f"    Serial number     {serial.decode('ascii', 'replace') if serial.isascii() else serial.hex().upper()}")
    print(f"    Session           0x{v[DID_ACTIVE_SESSION]:02X}")
    print("  Live data")
    print(f"    Battery           {live[DID_BATTERY_VOLTAGE] / 1000:.3f} V")
    print(f"    Vehicle state     {VEHICLE_STATES.get(live[DID_VEHICLE_STATE], '?')}")
    print(f"    Active faults     0x{live[DID_FAULT_BITMASK]:04X}")
    print(f"    Uptime            {live[DID_UPTIME]} s")


def show_dtcs(c: DiagClient) -> None:
    dtcs = [d for d in c.read_dtcs(0xFF) if d.status & 0x2C]   # pending/confirmed/failedSinceClear
    if not dtcs:
        print("  No DTCs stored.")
        return
    print(f"  {len(dtcs)} DTC(s) stored:")
    for d in dtcs:
        print(f"    {d.text}  {d.description}")
        print(f"      status 0x{d.status:02X}: {', '.join(describe_status(d.status))}")
        snapshot = c.read_snapshot(d.code)
        if snapshot:
            state = VEHICLE_STATES.get(snapshot.get("vehicle_state"), "?")
            print(f"      freeze frame: battery {snapshot.get('battery_mv', 0) / 1000:.3f} V, "
                  f"vehicle state {state}")
        extended = c.read_extended_data(d.code) or {}
        print(f"      occurrences: {extended.get('occurrence_counter', '?')}, "
              f"clean cycles since last failure: {extended.get('aging_counter', '?')}")


def run_demo(c: DiagClient, sil) -> None:
    def step(title: str) -> None:
        print(f"\n--- {title} " + "-" * max(0, 60 - len(title)))

    step("1. Read identification and live data")
    show_info(c)

    step("2. Try to write the VIN without security access")
    c.session(0x03)
    try:
        c.write_vin("WDB2030461A123456")
    except NegativeResponseException as exc:
        print(f"  Refused, as it should be: {nrc_name(nrc_of(exc))}")

    step("3. Unlock with seed/key and write the VIN")
    c.unlock()
    c.write_vin("WDB2030461A123456")
    print(f"  VIN is now {c.read(DID_VIN)}")

    if sil is not None:
        step("4. Simulate a flat battery (SIL fault injection)")
        sil.set_battery_mv(10350)
        sil.set_vehicle_state(0)
        sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
        sil.set_battery_mv(12600)
        sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, False)
        print("  Battery dipped to 10.350 V, then recovered.")

        step("5. Read DTC memory")
        show_dtcs(c)

        step("6. Reset the ECU - DTCs and VIN must survive")
        c.ecu_reset()
        time.sleep(1.0)
        print(f"  VIN after reset: {c.read(DID_VIN)}")
        show_dtcs(c)

        step("7. Clear DTCs")
        c.clear_dtcs()
        show_dtcs(c)


def main() -> int:
    parser = argparse.ArgumentParser(description="Body Control ECU diagnostic tool")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--sil", action="store_true")
    target.add_argument("--interface", choices=["slcan", "pcan", "socketcan", "kvaser", "ixxat"])
    parser.add_argument("--channel", default="COM5")
    parser.add_argument("--bitrate", type=int, default=500_000)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("info", "dtc", "clear", "lamp-test", "reset", "demo"):
        sub.add_parser(name)
    sub.add_parser("write-vin").add_argument("vin")
    args = parser.parse_args()

    logging.disable(logging.ERROR)

    sil = None
    if args.sil:
        from sil_bridge import SilEcu
        sil = SilEcu(channel="sil").start()
        bus = can.Bus(interface="virtual", channel="sil")
    else:
        bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)

    try:
        with DiagClient(bus) as c:
            if args.command == "info":
                show_info(c)
            elif args.command == "dtc":
                show_dtcs(c)
            elif args.command == "clear":
                c.clear_dtcs()
                print("  DTCs cleared.")
            elif args.command == "write-vin":
                c.unlock()
                c.write_vin(args.vin.upper())
                print(f"  VIN written: {c.read(DID_VIN)}")
            elif args.command == "lamp-test":
                c.unlock()
                c.client.start_routine(RID_LAMP_SELF_TEST)
                print("  Lamp self-test running for 3 s - all exterior lamps on.")
            elif args.command == "reset":
                c.ecu_reset()
                print("  ECU reset requested.")
            elif args.command == "demo":
                run_demo(c, sil)
    except NegativeResponseException as exc:
        print(f"  ECU refused the request: {nrc_name(nrc_of(exc))}", file=sys.stderr)
        return 1
    finally:
        bus.shutdown()
        if sil is not None:
            sil.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
