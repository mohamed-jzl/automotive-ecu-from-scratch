"""
UDS diagnostic system tests.

Run against either target with the same code:

    SIL   the production C stack compiled for the PC (sil_bridge.py)
    HIL   the real STM32 over a USB-CAN adapter

The tester is udsoncan + can-isotp: independent open-source implementations
of ISO 14229 and ISO 15765-2. A pass therefore means "a standard UDS client
accepts this ECU", not merely "our client agrees with our ECU".

A few tests need to inject a fault on demand. Only the SIL target can do
that; on hardware they are reported as SKIP with instructions to provoke the
fault physically (turn the battery potentiometer down). A skip is never
counted as a pass.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from udsoncan.exceptions import NegativeResponseException

from diag_client import (
    DID_ACTIVE_SESSION, DID_BATTERY_VOLTAGE, DID_ECU_SERIAL, DID_SOFTWARE_VERSION,
    DID_UPTIME, DID_VEHICLE_STATE, DID_VIN, FAULT_BATTERY_UNDERVOLTAGE,
    RID_LAMP_SELF_TEST, SESSION_DEFAULT, SESSION_EXTENDED, DiagClient,
    describe_status, dtc_to_string, nrc_name, nrc_of,
)
from sil_bridge import SilEcu
from test_cases import TestCase, TestSkipped

DTC_P0562 = 0x056200        # system voltage low, lights the warning lamp

S3_TIMEOUT_S = 5.0
LOCKOUT_S = 10.0


@dataclass
class DiagTarget:
    """What a diagnostic test runs against."""

    client: DiagClient
    sil: Optional[SilEcu] = None        # None when testing real hardware

    def require_sil(self, why: str) -> SilEcu:
        if self.sil is None:
            raise TestSkipped(f"needs SIL fault injection ({why}); on hardware, "
                              "provoke the fault physically and re-run")
        return self.sil


# =============================================================================
#  Helpers
# =============================================================================

def expect_nrc(action, expected: int, evidence: list[str]) -> None:
    """Run a udsoncan call that must be refused with a specific NRC."""
    try:
        action()
    except NegativeResponseException as exc:
        got = nrc_of(exc)
        evidence.append(f"refused with {nrc_name(got)}")
        assert got == expected, f"expected {nrc_name(expected)}, got {nrc_name(got)}"
        return
    raise AssertionError(f"request was accepted; expected {nrc_name(expected)}")


def expect_raw_nrc(target: DiagTarget, request: bytes, expected: int,
                   evidence: list[str]) -> None:
    response = target.client.raw(request)
    evidence.append(f"{request.hex(' ')}  ->  {response.hex(' ') if response else 'no response'}")
    assert response is not None, "no response at all"
    assert response[0] == 0x7F and response[1] == request[0], "not a negative response"
    assert response[2] == expected, f"expected {nrc_name(expected)}, got {nrc_name(response[2])}"


def wait_for_reboot(target: DiagTarget, seconds: float = 1.0) -> None:
    """Give the ECU time to send its response, reset and come back."""
    time.sleep(seconds)


# =============================================================================
#  Sessions and timing
# =============================================================================

def dg_default_session_at_power_up(t: DiagTarget, ev: list[str]) -> None:
    session = t.client.read(DID_ACTIVE_SESSION)
    ev.append(f"DID 0xF186 = 0x{session:02X}")
    assert session == SESSION_DEFAULT, "ECU did not start in the default session"


def dg_extended_session_reports_timing(t: DiagTarget, ev: list[str]) -> None:
    response = t.client.raw(bytes([0x10, 0x03]))
    ev.append(f"10 03  ->  {response.hex(' ') if response else 'none'}")
    assert response == bytes([0x50, 0x03, 0x00, 0x32, 0x01, 0xF4]), \
        "expected 50 03 | P2 0x0032 (50 ms) | P2* 0x01F4 (5000 ms)"


def dg_s3_timeout_returns_to_default(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_EXTENDED)
    time.sleep(S3_TIMEOUT_S + 1.0)          # tester stays silent past S3
    session = t.client.read(DID_ACTIVE_SESSION)
    ev.append(f"after {S3_TIMEOUT_S + 1:.0f} s of silence, session = 0x{session:02X}")
    assert session == SESSION_DEFAULT, "extended session survived the S3 timeout"


def dg_tester_present_keeps_session(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_EXTENDED)
    for _ in range(4):                       # 4 x 2 s = 8 s, well past S3
        time.sleep(2.0)
        reply = t.client.raw(bytes([0x3E, 0x80]), timeout=0.3)
        assert reply is None, "3E 80 (suppressed) must not be answered"
    session = t.client.read(DID_ACTIVE_SESSION)
    ev.append(f"8 s with TesterPresent every 2 s, session = 0x{session:02X}")
    assert session == SESSION_EXTENDED, "session dropped despite TesterPresent"


# =============================================================================
#  Reading data
# =============================================================================

def dg_identification_dids(t: DiagTarget, ev: list[str]) -> None:
    values = t.client.read_many([DID_VIN, DID_SOFTWARE_VERSION, DID_ECU_SERIAL])
    ev.append(f"VIN={values[DID_VIN]!r} SW={values[DID_SOFTWARE_VERSION]} "
              f"serial={values[DID_ECU_SERIAL].hex()}")
    assert len(values[DID_VIN]) == 17
    assert values[DID_SOFTWARE_VERSION] == (0, 2, 0), "software version is not 0.2.0"
    assert len(values[DID_ECU_SERIAL]) == 12


def dg_live_data_dids(t: DiagTarget, ev: list[str]) -> None:
    battery = t.client.read(DID_BATTERY_VOLTAGE)
    state = t.client.read(DID_VEHICLE_STATE)
    ev.append(f"battery = {battery} mV, vehicle state = {state}")
    assert 0 <= battery <= 18300, "battery reading outside the sense circuit range"
    assert 0 <= state <= 4, "vehicle state outside 0..4"


def dg_multi_frame_response(t: DiagTarget, ev: list[str]) -> None:
    """Five DIDs in one request: a 41-byte response, carried by ISO-TP as FF + 5 CFs."""
    dids = [DID_VIN, DID_SOFTWARE_VERSION, DID_ECU_SERIAL, DID_BATTERY_VOLTAGE, DID_UPTIME]
    values = t.client.read_many(dids)
    ev.append(f"{len(values)} DIDs decoded from one segmented response")
    assert set(values) == set(dids)


def dg_unsupported_did(t: DiagTarget, ev: list[str]) -> None:
    expect_raw_nrc(t, bytes([0x22, 0x12, 0x34]), 0x31, ev)


def dg_protocol_errors(t: DiagTarget, ev: list[str]) -> None:
    expect_raw_nrc(t, bytes([0xAA, 0x00]), 0x11, ev)          # unknown service
    expect_raw_nrc(t, bytes([0x22, 0xF1]), 0x13, ev)          # odd length
    expect_raw_nrc(t, bytes([0x10, 0x02]), 0x12, ev)          # programming session


def dg_session_restrictions(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_DEFAULT)
    expect_raw_nrc(t, bytes([0x31, 0x01, 0x02, 0x01]), 0x7F, ev)   # routine
    expect_raw_nrc(t, bytes([0x27, 0x01]), 0x7F, ev)               # security


def dg_functional_addressing(t: DiagTarget, ev: list[str]) -> None:
    silent = t.client.raw_functional(bytes([0x3E, 0x80]))
    answered = t.client.raw_functional(bytes([0x3E, 0x00]))
    unsupported = t.client.raw_functional(bytes([0xAA, 0x00]))
    ev.append(f"7DF 3E 80 -> {silent}, 3E 00 -> {answered.hex(' ') if answered else None}, "
              f"AA 00 -> {unsupported}")
    assert silent is None, "suppressed positive response was sent"
    assert answered == bytes([0x7E, 0x00]), "functional TesterPresent not answered"
    assert unsupported is None, "functional 'not supported' NRC must be suppressed"


# =============================================================================
#  Security and writing
# =============================================================================

def dg_write_needs_security(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_EXTENDED)
    expect_nrc(lambda: t.client.write_vin("WDB00000000000001"), 0x33, ev)


def dg_security_unlock(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_EXTENDED)
    seed_rsp = t.client.raw(bytes([0x27, 0x01]))
    assert seed_rsp is not None and seed_rsp[:2] == bytes([0x67, 0x01])
    seed = int.from_bytes(seed_rsp[2:6], "big")
    ev.append(f"seed 0x{seed:08X}")
    assert seed != 0, "a locked ECU must never send an all-zero seed"

    from diag_client import compute_key
    key = compute_key(seed)
    reply = t.client.raw(bytes([0x27, 0x02]) + key.to_bytes(4, "big"))
    ev.append(f"key 0x{key:08X} -> {reply.hex(' ') if reply else None}")
    assert reply == bytes([0x67, 0x02]), "correct key was not accepted"

    seed_again = t.client.raw(bytes([0x27, 0x01]))
    assert seed_again[2:6] == bytes(4), "unlocked ECU must answer with seed 00000000"


def dg_write_and_read_back_vin(t: DiagTarget, ev: list[str]) -> None:
    vin = "WDB2030461A123456"
    t.client.unlock()
    t.client.write_vin(vin)
    read_back = t.client.read(DID_VIN)
    ev.append(f"wrote {vin}, read {read_back}")
    assert read_back == vin


def dg_invalid_vin_rejected(t: DiagTarget, ev: list[str]) -> None:
    t.client.unlock()
    # 'I' is illegal in a VIN (ISO 3779): too easily read as '1'.
    expect_nrc(lambda: t.client.write_vin("WDBI030461A123456"), 0x31, ev)


def dg_vin_persists_across_reset(t: DiagTarget, ev: list[str]) -> None:
    vin = "VF1RFB00X56789012"
    t.client.unlock()
    t.client.write_vin(vin)
    t.client.ecu_reset()
    ev.append("ECUReset (hard) acknowledged")
    wait_for_reboot(t)

    if t.sil is not None:
        ev.append(f"SIL reboots observed: {t.sil.reset_count}")
        assert t.sil.reset_count >= 1, "ECU acknowledged but never reset"

    session = t.client.read(DID_ACTIVE_SESSION)
    read_back = t.client.read(DID_VIN)
    ev.append(f"after reset: session 0x{session:02X}, VIN {read_back}")
    assert session == SESSION_DEFAULT, "reset did not return to the default session"
    assert read_back == vin, "VIN was lost across the reset"


# =============================================================================
#  DTCs
# =============================================================================

def dg_dtc_confirmed_after_fault(t: DiagTarget, ev: list[str]) -> None:
    sil = t.require_sil("undervoltage")
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)

    dtcs = {d.code: d for d in t.client.read_dtcs(0x08)}      # confirmed only
    assert DTC_P0562 in dtcs, "P0562-00 not reported as confirmed"
    entry = dtcs[DTC_P0562]
    ev.append(f"{entry.text} status 0x{entry.status:02X}: {', '.join(describe_status(entry.status))}")
    assert entry.status == 0xAF, "expected 0xAF = failing, confirmed, warning lamp on"

    count = t.client.count_dtcs(0x08)
    assert count == 1, f"19 01 reported {count} confirmed DTCs, expected 1"


def dg_snapshot_records_conditions(t: DiagTarget, ev: list[str]) -> None:
    sil = t.require_sil("undervoltage")
    sil.set_battery_mv(10400)
    sil.set_vehicle_state(3)                  # RUN
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)

    sil.set_battery_mv(12600)                 # conditions change afterwards...
    snapshot = t.client.read_snapshot(DTC_P0562)
    ev.append(f"freeze frame: {snapshot}")
    assert snapshot == {"battery_mv": 10400, "vehicle_state": 3}, \
        "freeze frame must hold the conditions at the moment of failure"


def dg_intermittent_fault_history(t: DiagTarget, ev: list[str]) -> None:
    sil = t.require_sil("undervoltage")
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, False)
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, False)

    entry = {d.code: d for d in t.client.read_dtcs(0xFF)}[DTC_P0562]
    extended = t.client.read_extended_data(DTC_P0562)
    ev.append(f"status 0x{entry.status:02X}, extended data {extended}")
    assert entry.status & 0x01 == 0, "fault is gone: testFailed must be 0"
    assert entry.status & 0x08, "history must remain: confirmedDTC must be 1"
    assert extended.get("occurrence_counter") == 2


def dg_clear_dtcs(t: DiagTarget, ev: list[str]) -> None:
    if t.sil is not None:
        t.sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
        t.sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, False)
    t.client.clear_dtcs()
    remaining = t.client.read_dtcs(0x08)
    ev.append(f"confirmed after clear: {[d.text for d in remaining]}")
    assert not remaining, "confirmed DTCs survived ClearDiagnosticInformation"


def dg_dtcs_persist_across_reset(t: DiagTarget, ev: list[str]) -> None:
    sil = t.require_sil("undervoltage")
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
    t.client.ecu_reset()
    wait_for_reboot(t)

    codes = [dtc_to_string(d.code) for d in t.client.read_dtcs(0x08)]
    ev.append(f"confirmed after reset: {codes}")
    assert "P0562-00" in codes, "DTC was lost across the reset"


def dg_dtc_setting_off(t: DiagTarget, ev: list[str]) -> None:
    sil = t.require_sil("undervoltage")
    t.client.session(SESSION_EXTENDED)
    t.client.client.control_dtc_setting(0x02)               # off
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
    frozen = t.client.read_dtcs(0x08)
    ev.append(f"with DTC setting OFF: {[d.text for d in frozen]}")
    assert not frozen, "a DTC was stored while DTC setting was off"

    t.client.session(SESSION_DEFAULT)                       # re-enables it
    sil.report_fault(FAULT_BATTERY_UNDERVOLTAGE, True)
    stored = t.client.read_dtcs(0x08)
    ev.append(f"back in default session: {[d.text for d in stored]}")
    assert stored, "DTC setting was not re-enabled by leaving the extended session"


# =============================================================================
#  Routine
# =============================================================================

def dg_lamp_self_test(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_EXTENDED)
    expect_nrc(lambda: t.client.client.start_routine(RID_LAMP_SELF_TEST), 0x33, ev)

    t.client.unlock()
    t.client.client.start_routine(RID_LAMP_SELF_TEST)
    result = t.client.client.get_routine_result(RID_LAMP_SELF_TEST)
    status = result.service_data.routine_status_record
    ev.append(f"routine status record: {status.hex()}  (01 = running)")
    assert status == b"\x01"

    if t.sil is not None:
        assert t.sil.lamp_test_active(), "SIL reports the lamps are not on"


# =============================================================================
#  Security lockout - LAST, because it blocks unlocking for 10 s afterwards
# =============================================================================

def dg_security_lockout(t: DiagTarget, ev: list[str]) -> None:
    t.client.session(SESSION_EXTENDED)
    for attempt in range(1, 4):
        t.client.raw(bytes([0x27, 0x01]))
        reply = t.client.raw(bytes([0x27, 0x02, 0xDE, 0xAD, 0xBE, 0xEF]))
        ev.append(f"wrong key #{attempt}: {reply.hex(' ') if reply else None}")
        expected = 0x36 if attempt == 3 else 0x35
        assert reply == bytes([0x7F, 0x27, expected])

    expect_raw_nrc(t, bytes([0x27, 0x01]), 0x37, ev)       # no seed during delay


# =============================================================================
#  Registry
# =============================================================================

DIAG_TESTS: list[TestCase] = [
    TestCase("DG-001", "Default session at power-up", "REQ-110",
             "Reads DID 0xF186 and expects the default session.", dg_default_session_at_power_up),
    TestCase("DG-002", "Extended session reports P2/P2*", "REQ-111",
             "10 03 answered with the timing parameters.", dg_extended_session_reports_timing),
    TestCase("DG-003", "S3 timeout returns to default", "REQ-112",
             "An idle extended session falls back after 5 s.", dg_s3_timeout_returns_to_default),
    TestCase("DG-004", "TesterPresent keeps the session", "REQ-113",
             "3E 80 every 2 s holds the extended session.", dg_tester_present_keeps_session),
    TestCase("DG-005", "Identification DIDs", "REQ-120",
             "VIN, software version and serial number.", dg_identification_dids),
    TestCase("DG-006", "Live data DIDs", "REQ-121",
             "Battery voltage and vehicle state.", dg_live_data_dids),
    TestCase("DG-007", "Multi-frame response", "REQ-160",
             "Five DIDs in one ISO-TP segmented response.", dg_multi_frame_response),
    TestCase("DG-008", "Unsupported DID refused", "REQ-122",
             "NRC 0x31 for an unknown DID.", dg_unsupported_did),
    TestCase("DG-009", "Protocol errors", "REQ-101",
             "NRC 0x11, 0x13 and 0x12 where the standard requires them.", dg_protocol_errors),
    TestCase("DG-010", "Session restrictions", "REQ-104",
             "Extended-only services refused in default with NRC 0x7F.", dg_session_restrictions),
    TestCase("DG-011", "Functional addressing", "REQ-103",
             "Broadcast requests and response suppression.", dg_functional_addressing),
    TestCase("DG-012", "Writing needs security", "REQ-131",
             "WriteDataByIdentifier refused with NRC 0x33 while locked.", dg_write_needs_security),
    TestCase("DG-013", "Seed/key unlock", "REQ-130",
             "Correct key unlocks; seed is 0 once unlocked.", dg_security_unlock),
    TestCase("DG-014", "Write and read back VIN", "REQ-123",
             "A valid VIN is stored and returned.", dg_write_and_read_back_vin),
    TestCase("DG-015", "Invalid VIN rejected", "REQ-124",
             "A VIN containing 'I' is refused with NRC 0x31.", dg_invalid_vin_rejected),
    TestCase("DG-016", "VIN survives ECUReset", "REQ-141",
             "Reset is performed and the VIN is kept in flash.", dg_vin_persists_across_reset),
    TestCase("DG-017", "DTC confirmed after a fault", "REQ-150",
             "Undervoltage produces P0562-00 with status 0xAF.", dg_dtc_confirmed_after_fault),
    TestCase("DG-018", "Freeze frame", "REQ-151",
             "Snapshot holds battery voltage and state at failure.", dg_snapshot_records_conditions),
    TestCase("DG-019", "Intermittent fault history", "REQ-150",
             "Healed fault stays confirmed; occurrence counter = 2.", dg_intermittent_fault_history),
    TestCase("DG-020", "Clear DTCs", "REQ-152",
             "0x14 FFFFFF removes every confirmed DTC.", dg_clear_dtcs),
    TestCase("DG-021", "DTCs survive ECUReset", "REQ-141",
             "A confirmed DTC is still present after reset.", dg_dtcs_persist_across_reset),
    TestCase("DG-022", "ControlDTCSetting", "REQ-153",
             "No DTC stored while off; re-enabled in default session.", dg_dtc_setting_off),
    TestCase("DG-023", "Lamp self-test routine", "REQ-170",
             "Needs security; runs and reports status 'running'.", dg_lamp_self_test),
    TestCase("DG-024", "Security lockout", "REQ-132",
             "3 wrong keys -> NRC 0x36, then 0x37 during the delay.", dg_security_lockout),
]
