#!/usr/bin/env python3
"""
Software model of the Body Control ECU, for developing tests without hardware.

What this is for
----------------
Two problems it solves:

  1. The test framework needs to be testable. A test that has never passed is
     not a test - it is an untested assertion. Running the suite against a
     model that is known-good proves the framework can distinguish pass from
     fail, so that when it runs against real hardware a failure means the ECU
     is wrong rather than the test.

  2. Test development can start before the hardware exists. In a real project
     the test bench and the ECU are built by different teams in parallel, and
     waiting for silicon before writing tests wastes months.

What this is NOT
----------------
It is not the ECU. It reimplements the observable CAN behaviour in Python and
shares no code with the firmware, so it proves nothing at all about the C. A
test passing here and failing on the board means the firmware is wrong; that
is exactly the comparison that makes the model useful, and it stops working
the moment the model is derived from the firmware rather than from the
specification.

In industry this is called a model-in-the-loop (MIL) or software-in-the-loop
(SIL) test target, as opposed to hardware-in-the-loop (HIL) against a real ECU.

Usage
-----
    # Terminal 1
    python ecu_simulator.py --interface virtual

    # Terminal 2
    python run_tests.py --interface virtual

Note that python-can's `virtual` backend only connects buses inside a single
process. To run the two across separate terminals use a real virtual driver -
`vcan0` on Linux, or point both at the same adapter. The --self-test option
below runs both sides in one process, which is what CI uses.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

from ecu_signals import (
    CAN_ID_BCM_DIAGNOSTICS,
    CAN_ID_BCM_VEHICLE_STATUS,
    CAN_ID_PCM_ENGINE_STATUS,
    VEHICLE_STATE_RUN,
    CrcError,
    Diagnostics,
    EngineStatus,
    VehicleStatus,
)

try:
    import can
except ImportError:                                     # pragma: no cover
    raise SystemExit("python-can is not installed.  pip install -r requirements.txt")


class EcuSimulator:
    """
    Transmits the ECU's periodic messages and consumes what it subscribes to.

    Mirrors the firmware's schedule: BCM_VehicleStatus every 100 ms and
    BCM_DIAGNOSTICS every 500 ms, each with its own alive counter.
    """

    VEHICLE_STATUS_PERIOD_S = 0.100
    DIAGNOSTICS_PERIOD_S    = 0.500
    INDICATOR_HALF_PERIOD_S = 0.333     # 1.5 Hz, matching ECU_INDICATOR_HALF_PERIOD_MS

    def __init__(self, bus: can.BusABC, *, indicator_left: bool = False) -> None:
        self.bus = bus
        self.indicator_left = indicator_left

        self._alive_status = 0
        self._alive_diag = 0
        self._started_at = time.monotonic()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self.engine_frames_received = 0
        self.crc_errors = 0

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def __enter__(self) -> "EcuSimulator":
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()

    # -- message construction --------------------------------------------

    @property
    def uptime_s(self) -> float:
        return time.monotonic() - self._started_at

    def _indicator_phase(self) -> bool:
        """Flash phase derived from elapsed time, as the firmware does."""
        return int(self.uptime_s / self.INDICATOR_HALF_PERIOD_S) % 2 == 0

    def _build_vehicle_status(self) -> bytes:
        self._alive_status = (self._alive_status + 1) & 0xFF

        return VehicleStatus(
            vehicle_state=VEHICLE_STATE_RUN,
            ignition_on=True,
            brake_active=False,
            # Reports the LAMP, not the switch: the bit flashes while the
            # indicator is active, which is what TC-012 verifies.
            indicator_left=self.indicator_left and self._indicator_phase(),
            indicator_right=False,
            headlight_on=True,
            battery_mv=12_600,
            fault_count=0,
            fault_bitmask=0x0000,
            alive_counter=self._alive_status,
        ).pack()

    def _build_diagnostics(self) -> bytes:
        self._alive_diag = (self._alive_diag + 1) & 0xFF

        return Diagnostics(
            uptime_seconds=int(self.uptime_s),
            task_overrun_count=0,
            max_task_duration_ms=1,
            can_tx_failures=0,
            alive_counter=self._alive_diag,
        ).pack()

    def _send(self, can_id: int, payload: bytes) -> None:
        self.bus.send(
            can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
        )

    # -- main loop -------------------------------------------------------

    def _run(self) -> None:
        next_status = time.monotonic()
        next_diag = time.monotonic()

        while not self._stop.is_set():
            now = time.monotonic()

            if now >= next_status:
                self._send(CAN_ID_BCM_VEHICLE_STATUS, self._build_vehicle_status())
                # Advance from the scheduled time, not from now, so the average
                # period stays exact - the same reasoning as the firmware's
                # scheduler.
                next_status += self.VEHICLE_STATUS_PERIOD_S

            if now >= next_diag:
                self._send(CAN_ID_BCM_DIAGNOSTICS, self._build_diagnostics())
                next_diag += self.DIAGNOSTICS_PERIOD_S

            message = self.bus.recv(timeout=0.005)
            if message is not None and message.arbitration_id == CAN_ID_PCM_ENGINE_STATUS:
                try:
                    EngineStatus.unpack(bytes(message.data))
                    self.engine_frames_received += 1
                except CrcError:
                    self.crc_errors += 1


# =============================================================================
#  Entry point
# =============================================================================

def main() -> int:
    parser = argparse.ArgumentParser(description="Body Control ECU simulator")
    parser.add_argument("--interface", default="virtual")
    parser.add_argument("--channel", default="test")
    parser.add_argument("--bitrate", type=int, default=500_000)
    parser.add_argument("--indicator-left", action="store_true",
                        help="simulate the left turn indicator being active")
    parser.add_argument("--self-test", action="store_true",
                        help="run the system test suite against this simulator "
                             "in the same process")
    args = parser.parse_args()

    kwargs = {"interface": args.interface, "channel": args.channel}
    if args.interface != "virtual":
        kwargs["bitrate"] = args.bitrate

    if args.self_test:
        return _self_test(kwargs)

    print(f"ECU simulator running on {args.interface}:{args.channel} - Ctrl-C to stop")

    with can.Bus(**kwargs) as bus, EcuSimulator(bus, indicator_left=args.indicator_left):
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nstopped")

    return 0


def _self_test(bus_kwargs: dict) -> int:
    """
    Run the simulator and the test suite together in one process.

    This is what proves the framework works: the same suite that reports
    INCONCLUSIVE against an empty bus must report PASSED against a known-good
    model. A framework that cannot tell those two apart is worthless.
    """
    from can_interface import EcuCanInterface
    from test_cases import ALL_TESTS, ERROR, FAIL, PASS, SKIP

    print("\nSelf-test: running the system test suite against the simulator\n")

    with can.Bus(**bus_kwargs) as sim_bus:
        with EcuSimulator(sim_bus, indicator_left=True):
            time.sleep(0.3)     # let a few frames reach the bus first

            tester = EcuCanInterface(**{
                "interface": bus_kwargs["interface"],
                "channel": bus_kwargs["channel"],
            })
            tester.connect()

            results = []
            try:
                for test in ALL_TESTS:
                    tester.flush()
                    result = test.run(tester)
                    results.append(result)
                    print(f"  [{result.status:<5}] {result.test_id}  {result.name}")
                    if result.message:
                        print(f"          {result.message.splitlines()[0]}")
            finally:
                tester.disconnect()

    counts = {s: sum(1 for r in results if r.status == s)
              for s in (PASS, FAIL, SKIP, ERROR)}

    print(f"\n  {len(results)} tests: {counts[PASS]} passed, {counts[FAIL]} failed, "
          f"{counts[SKIP]} skipped, {counts[ERROR]} errors")

    # The framework is only proven if tests genuinely pass AND none error out.
    ok = counts[PASS] > 0 and counts[FAIL] == 0 and counts[ERROR] == 0
    print(f"  SELF-TEST: {'OK' if ok else 'FAILED'}\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
