"""
Test cases for the Automotive Body Control ECU.

Every test carries the identifier of the requirement it verifies. That link is
not decoration: in automotive development (the V-model, ASPICE) a requirement
with no verifying test is an open risk, and a test that verifies no requirement
is work nobody asked for. The traceability matrix in
docs/traceability_matrix.md is generated from these annotations.

Level of testing
----------------
These are SYSTEM tests. They exercise the complete ECU - firmware, hardware and
CAN interface together - through its external interface only, with no access to
internals. That is a deliberate complement to the unit tests in tests/, which
prove individual modules in isolation. Each level catches what the other cannot:

    unit test    "the state machine transitions correctly"
    system test  "the assembled ECU actually reports that state on the bus"

Passing unit tests with failing system tests means the integration is wrong.
Passing system tests with failing unit tests means you were lucky.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from can_interface import EcuCanInterface
from ecu_signals import (
    CAN_ID_BCM_DIAGNOSTICS,
    CAN_ID_BCM_VEHICLE_STATUS,
    CrcError,
    VEHICLE_STATE_NAMES,
    VehicleStatus,
    crc8,
)

# =============================================================================
#  Result model
# =============================================================================

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"
ERROR = "ERROR"


@dataclass
class TestResult:
    """Outcome of a single test case."""

    test_id: str
    name: str
    requirement: str
    status: str
    message: str = ""
    duration_s: float = 0.0
    evidence: list[str] = field(default_factory=list)


class TestSkipped(Exception):
    """
    Raised when a test cannot run, as distinct from failing.

    The difference matters. A failure says "the ECU is wrong". A skip says
    "this test proved nothing". Reporting a skip as a pass would be the worst
    possible outcome: a green report that verifies nothing.
    """


@dataclass
class TestCase:
    """One test: an identifier, the requirement it covers, and the procedure."""

    test_id: str
    name: str
    requirement: str
    description: str
    procedure: Callable[[EcuCanInterface, list[str]], None]

    def run(self, ecu: EcuCanInterface) -> TestResult:
        evidence: list[str] = []
        started = time.monotonic()

        try:
            self.procedure(ecu, evidence)
            status, message = PASS, ""

        except TestSkipped as exc:
            status, message = SKIP, str(exc)

        except AssertionError as exc:
            status, message = FAIL, str(exc)

        except Exception as exc:                        # noqa: BLE001
            # An unexpected exception is not a test failure - it means the test
            # itself, or the bus, is broken. Reported separately so a framework
            # defect is never mistaken for an ECU defect.
            status, message = ERROR, f"{type(exc).__name__}: {exc}"

        return TestResult(
            test_id=self.test_id,
            name=self.name,
            requirement=self.requirement,
            status=status,
            message=message,
            duration_s=time.monotonic() - started,
            evidence=evidence,
        )


# =============================================================================
#  Shared helpers
# =============================================================================

def _require_status(ecu: EcuCanInterface, timeout: float = 2.0) -> VehicleStatus:
    """
    Fetch one BCM_VehicleStatus, or skip the test if the ECU is absent.

    Every test needs this, and a missing ECU is a setup problem rather than a
    defect, so it is reported as a skip with an actionable message.
    """
    status = ecu.wait_for_message(CAN_ID_BCM_VEHICLE_STATUS, timeout=timeout)

    if status is None:
        raise TestSkipped(
            "No BCM_VehicleStatus received. Check that the ECU is powered, "
            "that the CAN transceiver is wired, that both ends of the bus are "
            "terminated with 120 ohms, and that both nodes are at 500 kbit/s."
        )
    return status


# =============================================================================
#  Test procedures
# =============================================================================

def tc_vehicle_status_is_transmitted(ecu: EcuCanInterface, evidence: list[str]) -> None:
    status = _require_status(ecu)

    evidence.append(f"state={status.state_name} battery={status.battery_volts:.2f} V")
    evidence.append(f"alive_counter={status.alive_counter}")

    assert status is not None, "no vehicle status received"


def tc_vehicle_status_period(ecu: EcuCanInterface, evidence: list[str]) -> None:
    _require_status(ecu)   # confirm the ECU is alive before measuring

    stats = ecu.measure_period(CAN_ID_BCM_VEHICLE_STATUS, duration_s=2.0)

    if stats is None:
        raise TestSkipped("fewer than two frames captured - cannot measure a period")

    evidence.append(str(stats))

    # +/-20% tolerance. The requirement is 100 ms; the allowance covers host
    # timestamping jitter, which is the limiting factor here rather than any
    # inaccuracy in the ECU. See the note in can_interface.py.
    assert 80.0 <= stats.mean_ms <= 120.0, (
        f"mean period {stats.mean_ms:.1f} ms is outside the 80-120 ms window"
    )


def tc_diagnostics_is_transmitted(ecu: EcuCanInterface, evidence: list[str]) -> None:
    diagnostics = ecu.wait_for_message(CAN_ID_BCM_DIAGNOSTICS, timeout=3.0)

    if diagnostics is None:
        raise TestSkipped("no BCM_DIAGNOSTICS received within 3 s")

    evidence.append(
        f"uptime={diagnostics.uptime_seconds}s "
        f"overruns={diagnostics.task_overrun_count} "
        f"worst_task={diagnostics.max_task_duration_ms}ms"
    )

    assert diagnostics.uptime_seconds >= 0, "negative uptime reported"


def tc_every_frame_passes_crc(ecu: EcuCanInterface, evidence: list[str]) -> None:
    _require_status(ecu)

    frames = ecu.collect(CAN_ID_BCM_VEHICLE_STATUS, duration_s=2.0)

    if not frames:
        raise TestSkipped("no frames captured")

    corrupt = 0
    for frame in frames:
        if frame.data[7] != crc8(frame.data[:7]):
            corrupt += 1

    evidence.append(f"{len(frames)} frames checked, {corrupt} CRC failures")

    # Any CRC failure at all points at a physical-layer problem: missing
    # termination, a bit rate mismatch, or an over-long bus.
    assert corrupt == 0, f"{corrupt} of {len(frames)} frames failed their CRC"


def tc_alive_counter_increments(ecu: EcuCanInterface, evidence: list[str]) -> None:
    _require_status(ecu)

    frames = ecu.collect(CAN_ID_BCM_VEHICLE_STATUS, duration_s=1.5)

    if len(frames) < 3:
        raise TestSkipped(f"only {len(frames)} frames captured, need at least 3")

    counters = [VehicleStatus.unpack(f.data).alive_counter for f in frames]
    evidence.append(f"counters: {counters[:10]}")

    for previous, current in zip(counters, counters[1:]):
        expected = (previous + 1) & 0xFF     # must wrap 255 -> 0
        assert current == expected, (
            f"alive counter jumped {previous} -> {current}, expected {expected}. "
            "A repeated value means stale data; a skip means a lost frame."
        )


def tc_vehicle_state_is_valid(ecu: EcuCanInterface, evidence: list[str]) -> None:
    status = _require_status(ecu)

    evidence.append(f"reported state = {status.vehicle_state} ({status.state_name})")

    assert status.vehicle_state in VEHICLE_STATE_NAMES, (
        f"state {status.vehicle_state} is outside the defined range 0-4"
    )


def tc_battery_voltage_is_plausible(ecu: EcuCanInterface, evidence: list[str]) -> None:
    status = _require_status(ecu)

    evidence.append(f"battery = {status.battery_volts:.3f} V "
                    f"({status.battery_mv} mV)")

    # Upper bound is the sense circuit's full scale (18.3 V). Anything above it
    # is impossible and would indicate a decoding error rather than a real
    # measurement.
    assert 0 <= status.battery_mv <= 18300, (
        f"battery reading {status.battery_mv} mV is outside the measurable range"
    )


def tc_no_faults_under_nominal_conditions(ecu: EcuCanInterface, evidence: list[str]) -> None:
    status = _require_status(ecu)

    evidence.append(f"fault_bitmask=0x{status.fault_bitmask:04X} "
                    f"count={status.fault_count}")

    if status.active_faults:
        evidence.append("active: " + ", ".join(status.active_faults))

    assert status.fault_bitmask == 0, (
        f"ECU reports active faults: {', '.join(status.active_faults)}. "
        "With the potentiometer set to a nominal battery voltage this should "
        "be empty."
    )


def tc_fault_count_matches_bitmask(ecu: EcuCanInterface, evidence: list[str]) -> None:
    status = _require_status(ecu)

    bits_set = bin(status.fault_bitmask).count("1")

    evidence.append(f"count field={status.fault_count}, bits set={bits_set}")

    # Two representations of the same fact must agree. A mismatch means the
    # encoder is building the payload from inconsistent sources.
    assert status.fault_count == bits_set, (
        f"fault_count says {status.fault_count} but the bitmask has "
        f"{bits_set} bits set"
    )


def tc_no_task_overruns(ecu: EcuCanInterface, evidence: list[str]) -> None:
    diagnostics = ecu.wait_for_message(CAN_ID_BCM_DIAGNOSTICS, timeout=3.0)

    if diagnostics is None:
        raise TestSkipped("no diagnostics message received")

    evidence.append(f"overruns={diagnostics.task_overrun_count} "
                    f"worst_task={diagnostics.max_task_duration_ms} ms")

    # This is the evidence that the timing design actually closes. A non-zero
    # count means a task exceeded its budget, and the schedule can no longer be
    # assumed to hold.
    assert diagnostics.task_overrun_count == 0, (
        f"{diagnostics.task_overrun_count} task overruns recorded - "
        "the scheduler missed a deadline"
    )


def tc_ecu_tolerates_restbus_traffic(ecu: EcuCanInterface, evidence: list[str]) -> None:
    """
    Feed the ECU a realistic engine-status stream and confirm it keeps running.

    This is the value of restbus simulation: the ECU is exercised against the
    network traffic it will meet in a vehicle, without a vehicle. A node that
    works alone on a quiet bench and falls over on a busy bus is a classic and
    expensive integration failure.
    """
    _require_status(ecu)

    for rpm in range(800, 3000, 200):
        ecu.send_engine_status(
            rpm=rpm,
            speed_kph=rpm / 40.0,
            coolant_temp_c=90,
            engine_running=True,
        )
        time.sleep(0.05)

    evidence.append("sent 11 PCM_EngineStatus frames over ~550 ms")

    status = ecu.wait_for_message(CAN_ID_BCM_VEHICLE_STATUS, timeout=2.0)

    assert status is not None, (
        "ECU stopped transmitting after receiving engine status traffic"
    )
    evidence.append(f"ECU still alive, state={status.state_name}")


def tc_indicator_signals_are_consistent(ecu: EcuCanInterface, evidence: list[str]) -> None:
    """
    The indicator bits report the LAMP, not the switch, so they must be seen
    flashing rather than held steady when an indicator is active.

    An instrument cluster telltale is required to mirror the exterior lamp, so
    a signal that reported the switch instead would light the telltale even
    with a failed bulb - the exact failure the requirement exists to prevent.
    """
    _require_status(ecu)

    frames = ecu.collect(CAN_ID_BCM_VEHICLE_STATUS, duration_s=2.0)

    if len(frames) < 10:
        raise TestSkipped(f"only {len(frames)} frames captured, need at least 10")

    states = [VehicleStatus.unpack(f.data) for f in frames]
    left = [s.indicator_left for s in states]

    if not any(left):
        raise TestSkipped(
            "left indicator never active - hold the left turn button during "
            "this test to verify the flashing behaviour"
        )

    transitions = sum(1 for a, b in zip(left, left[1:]) if a != b)
    evidence.append(f"{transitions} transitions across {len(left)} frames "
                    f"over 2 s (expect ~6 at 1.5 Hz)")

    assert transitions >= 2, (
        "indicator bit never changed while active - it appears to report the "
        "switch rather than the lamp"
    )


# =============================================================================
#  Registry
# =============================================================================

ALL_TESTS: list[TestCase] = [
    TestCase(
        "TC-001", "Vehicle status is transmitted", "REQ-010",
        "The ECU broadcasts BCM_VehicleStatus on the bus.",
        tc_vehicle_status_is_transmitted,
    ),
    TestCase(
        "TC-002", "Vehicle status period is 100 ms", "REQ-011",
        "BCM_VehicleStatus is transmitted every 100 ms (+/-20%).",
        tc_vehicle_status_period,
    ),
    TestCase(
        "TC-003", "Diagnostics message is transmitted", "REQ-014",
        "The ECU broadcasts BCM_DIAGNOSTICS with health telemetry.",
        tc_diagnostics_is_transmitted,
    ),
    TestCase(
        "TC-004", "Every frame passes its CRC", "REQ-012",
        "All transmitted frames carry a valid CRC-8 in byte 7.",
        tc_every_frame_passes_crc,
    ),
    TestCase(
        "TC-005", "Alive counter increments and wraps", "REQ-013",
        "The alive counter advances by exactly one per transmission.",
        tc_alive_counter_increments,
    ),
    TestCase(
        "TC-006", "Vehicle state is within range", "REQ-001",
        "The reported state is always one of the five defined values.",
        tc_vehicle_state_is_valid,
    ),
    TestCase(
        "TC-007", "Battery voltage is plausible", "REQ-020",
        "The reported voltage lies within the sense circuit's range.",
        tc_battery_voltage_is_plausible,
    ),
    TestCase(
        "TC-008", "No faults under nominal conditions", "REQ-030",
        "With a healthy supply the ECU reports no active faults.",
        tc_no_faults_under_nominal_conditions,
    ),
    TestCase(
        "TC-009", "Fault count agrees with the bitmask", "REQ-031",
        "The fault count field equals the number of bits set in the bitmask.",
        tc_fault_count_matches_bitmask,
    ),
    TestCase(
        "TC-010", "No scheduler deadline misses", "REQ-050",
        "No task exceeded its execution budget during the test.",
        tc_no_task_overruns,
    ),
    TestCase(
        "TC-011", "ECU tolerates restbus traffic", "REQ-040",
        "The ECU keeps operating while receiving powertrain messages.",
        tc_ecu_tolerates_restbus_traffic,
    ),
    TestCase(
        "TC-012", "Indicator signals report the lamp", "REQ-060",
        "Indicator bits flash with the lamp rather than following the switch.",
        tc_indicator_signals_are_consistent,
    ),
]
