"""
UDS diagnostic client for the Body Control ECU.

Built on two open-source libraries used across the industry for tooling:

    can-isotp   ISO 15765-2 transport (segmentation, flow control)
    udsoncan    ISO 14229-1 services (sessions, DIDs, DTCs, security)

Why use libraries here, when the ECU side was written from scratch?
Because an INDEPENDENT implementation is what makes the tests meaningful. If
our C ISO-TP layer framed a multi-frame response slightly wrong, a Python
client written by the same person would likely make the matching mistake and
pass. udsoncan and can-isotp were written by other people, against the
standards, and are used by many others: when they accept our ECU's answers,
that is evidence of interoperability, not of self-consistency.

This file also carries the pieces a client must agree on with the ECU:
the seed/key algorithm, the DID layout, the DTC codes. Each is written from
the specification in docs/uds_specification.md, not copied from the C.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Optional

import can
import isotp
import udsoncan
import udsoncan.configs
from udsoncan import DidCodec
from udsoncan.client import Client
from udsoncan.connections import PythonIsoTpConnection
from udsoncan.exceptions import NegativeResponseException, TimeoutException

# =============================================================================
#  Addressing (docs/uds_specification.md section 2)
# =============================================================================

PHYSICAL_REQUEST_ID = 0x7E0
FUNCTIONAL_REQUEST_ID = 0x7DF
RESPONSE_ID = 0x7E8

# =============================================================================
#  Identifiers
# =============================================================================

DID_VIN = 0xF190
DID_SOFTWARE_VERSION = 0xF195
DID_ECU_SERIAL = 0xF18C
DID_ACTIVE_SESSION = 0xF186
DID_BATTERY_VOLTAGE = 0x0100
DID_VEHICLE_STATE = 0x0101
DID_DIGITAL_INPUTS = 0x0102
DID_FAULT_BITMASK = 0x0103
DID_UPTIME = 0x0104

RID_LAMP_SELF_TEST = 0x0201

SESSION_DEFAULT = 0x01
SESSION_EXTENDED = 0x03

# Fault ids in the order of FaultId_t (used for SIL fault injection).
FAULT_BATTERY_UNDERVOLTAGE = 0
FAULT_BATTERY_OVERVOLTAGE = 1
FAULT_ADC_FAILURE = 2
FAULT_CAN_BUS_OFF = 3
FAULT_CAN_TX_FAILURE = 4
FAULT_TASK_OVERRUN = 5

DTC_NAMES = {
    0x056200: "System voltage low",
    0x056300: "System voltage high",
    0x900149: "Battery sense circuit - internal failure",
    0xC00188: "High-speed CAN - bus off",
    0xC00100: "High-speed CAN - general failure",
    0xF00049: "Control module - internal failure",
}

STATUS_BITS = [
    (0x01, "testFailed"),
    (0x02, "testFailedThisOperationCycle"),
    (0x04, "pendingDTC"),
    (0x08, "confirmedDTC"),
    (0x10, "testNotCompletedSinceLastClear"),
    (0x20, "testFailedSinceLastClear"),
    (0x40, "testNotCompletedThisOperationCycle"),
    (0x80, "warningIndicatorRequested"),
]

VEHICLE_STATES = {0: "OFF", 1: "ACC", 2: "ON", 3: "RUN", 4: "FAULT"}

NRC_NAMES = {
    0x10: "generalReject", 0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported", 0x13: "incorrectMessageLengthOrInvalidFormat",
    0x14: "responseTooLong", 0x22: "conditionsNotCorrect",
    0x24: "requestSequenceError", 0x31: "requestOutOfRange",
    0x33: "securityAccessDenied", 0x35: "invalidKey",
    0x36: "exceededNumberOfAttempts", 0x37: "requiredTimeDelayNotExpired",
    0x72: "generalProgrammingFailure", 0x78: "responsePending",
    0x7E: "subFunctionNotSupportedInActiveSession",
    0x7F: "serviceNotSupportedInActiveSession",
}

# =============================================================================
#  Seed / key (docs/uds_specification.md section 6)
# =============================================================================

_MASK32 = 0xFFFFFFFF


def compute_key(seed: int) -> int:
    """
    Educational seed/key algorithm - independently implemented from the spec.

    NOT secure: see firmware/.../uds_security.h. Real ECUs use AES-CMAC with a
    key held in a hardware security module.
    """
    key = seed ^ 0xA5C3F00F
    key = ((key << 7) | (key >> 25)) & _MASK32        # rotate left 7
    key = (key + 0x1D2B3C4D) & _MASK32
    key ^= key >> 11
    return key


def _security_algo(level: int, seed: bytes, params=None) -> bytes:
    return compute_key(int.from_bytes(seed, "big")).to_bytes(4, "big")


# =============================================================================
#  DTC helpers
# =============================================================================

def dtc_to_string(code: int) -> str:
    """
    Format a 3-byte DTC the way technicians write it, e.g. 0x056200 -> P0562-00.

    Top two bits select the letter (P/C/B/U); the next 14 bits are four digits,
    the first of which is limited to 0-3; the last byte is the failure type.
    """
    letter = "PCBU"[(code >> 22) & 0x3]
    first_digit = (code >> 20) & 0x3
    rest = (code >> 8) & 0xFFF
    failure_type = code & 0xFF
    return f"{letter}{first_digit}{rest:03X}-{failure_type:02X}"


def describe_status(status: int) -> list[str]:
    return [name for bit, name in STATUS_BITS if status & bit]


# =============================================================================
#  DID codecs for udsoncan
# =============================================================================

class RawCodec(DidCodec):
    """Pass bytes through unchanged (for binary DIDs like the chip UID)."""

    def __init__(self, length: int) -> None:
        self.length = length

    def encode(self, value: bytes) -> bytes:
        return bytes(value)

    def decode(self, payload: bytes) -> bytes:
        return bytes(payload)

    def __len__(self) -> int:
        return self.length


# UDS data is big-endian: note the '>' in every struct format.
DID_CODECS = {
    DID_VIN: udsoncan.AsciiCodec(17),
    DID_SOFTWARE_VERSION: ">BBB",
    DID_ECU_SERIAL: RawCodec(12),
    DID_ACTIVE_SESSION: ">B",
    DID_BATTERY_VOLTAGE: ">H",
    DID_VEHICLE_STATE: ">B",
    DID_DIGITAL_INPUTS: ">B",
    DID_FAULT_BITMASK: ">H",
    DID_UPTIME: ">I",
}


@dataclass
class DtcEntry:
    code: int
    status: int

    @property
    def text(self) -> str:
        return dtc_to_string(self.code)

    @property
    def description(self) -> str:
        return DTC_NAMES.get(self.code, "unknown")


# =============================================================================
#  Client
# =============================================================================

class DiagClient:
    """A UDS tester connected to the ECU through one python-can bus."""

    def __init__(self, bus: can.BusABC, request_timeout: float = 3.0) -> None:
        self.bus = bus

        address = isotp.Address(isotp.AddressingMode.Normal_11bits,
                                txid=PHYSICAL_REQUEST_ID, rxid=RESPONSE_ID)
        self.stack = isotp.CanStack(
            bus,
            address=address,
            params={
                "tx_padding": 0xCC,          # full 8-byte frames, like the ECU
                "tx_data_min_length": 8,
                "blocking_send": False,
                "stmin": 0,
                "blocksize": 0,
            },
        )
        self.connection = PythonIsoTpConnection(self.stack)

        config = copy.deepcopy(udsoncan.configs.default_client_config)
        config["data_identifiers"] = DID_CODECS
        config["security_algo"] = _security_algo
        config["request_timeout"] = request_timeout
        # The ECU advertises P2 = 50 ms. Adopting it here would make the tester
        # as strict as the ECU is fast; on a PC with ~15 ms timer granularity
        # talking to a simulated ECU, that produces false timeouts. The ECU's
        # real timing is verified on hardware, not through this client.
        config["use_server_timing"] = False
        config["p2_timeout"] = 1.0
        self.client = Client(self.connection, config=config)

    def open(self) -> "DiagClient":
        self.client.open()
        return self

    def close(self) -> None:
        self.client.close()

    def __enter__(self) -> "DiagClient":
        return self.open()

    def __exit__(self, *exc_info) -> None:
        self.close()

    # -- raw access, for negative tests and byte-exact checks ------------

    def raw(self, payload: bytes, timeout: float = 1.0) -> Optional[bytes]:
        """Send raw request bytes; return the raw response, or None if none came."""
        self.connection.empty_rxqueue()
        self.connection.send(bytes(payload))
        try:
            return self.connection.wait_frame(timeout=timeout, exception=True)
        except TimeoutException:
            return None

    def raw_functional(self, payload: bytes, timeout: float = 0.5) -> Optional[bytes]:
        """Broadcast a single-frame request on 0x7DF; return any response."""
        if len(payload) > 7:
            raise ValueError("functional requests must fit in a single frame")
        self.connection.empty_rxqueue()
        frame = bytes([len(payload)]) + bytes(payload)
        self.bus.send(can.Message(arbitration_id=FUNCTIONAL_REQUEST_ID,
                                  data=frame.ljust(8, b"\xCC"),
                                  is_extended_id=False))
        try:
            return self.connection.wait_frame(timeout=timeout, exception=True)
        except TimeoutException:
            return None

    # -- convenience ------------------------------------------------------

    def session(self, session: int) -> None:
        self.client.change_session(session)

    def unlock(self) -> None:
        """Extended session + SecurityAccess level 1."""
        self.client.change_session(SESSION_EXTENDED)
        self.client.unlock_security_access(1)

    @staticmethod
    def _unwrap(value):
        """udsoncan returns single-field struct DIDs as a 1-tuple: (12600,)."""
        return value[0] if isinstance(value, tuple) and len(value) == 1 else value

    def read(self, did: int):
        return self._unwrap(self.client.read_data_by_identifier(did).service_data.values[did])

    def read_many(self, dids: list[int]) -> dict:
        values = self.client.read_data_by_identifier(dids).service_data.values
        return {did: self._unwrap(value) for did, value in values.items()}

    def write_vin(self, vin: str) -> None:
        self.client.write_data_by_identifier(DID_VIN, vin)

    def read_dtcs(self, status_mask: int = 0xFF) -> list[DtcEntry]:
        response = self.client.get_dtc_by_status_mask(status_mask)
        return [DtcEntry(d.id, d.status.get_byte_as_int())
                for d in response.service_data.dtcs]

    def count_dtcs(self, status_mask: int) -> int:
        return self.client.get_number_of_dtc_by_status_mask(status_mask).service_data.dtc_count

    def read_snapshot(self, dtc: int) -> Optional[dict]:
        """
        Read the freeze frame of one DTC (0x19 0x04, record 0x01), decoded by hand.

        Response: 59 04 <DTC:3> <status> [01 <n> (<DID:2> <data>)...]
        """
        response = self.raw(bytes([0x19, 0x04, (dtc >> 16) & 0xFF,
                                   (dtc >> 8) & 0xFF, dtc & 0xFF, 0x01]))
        if response is None or response[0] != 0x59 or len(response) < 6:
            return None
        if len(response) == 6:
            return {}                                   # no freeze frame stored
        data = response[8:]                             # skip record no. and count
        snapshot = {}
        while len(data) >= 3:
            did = (data[0] << 8) | data[1]
            if did == DID_BATTERY_VOLTAGE:
                snapshot["battery_mv"] = (data[2] << 8) | data[3]
                data = data[4:]
            elif did == DID_VEHICLE_STATE:
                snapshot["vehicle_state"] = data[2]
                data = data[3:]
            else:
                break
        return snapshot

    def read_extended_data(self, dtc: int) -> Optional[dict]:
        """0x19 0x06 record 0xFF: occurrence counter and aging counter."""
        response = self.raw(bytes([0x19, 0x06, (dtc >> 16) & 0xFF,
                                   (dtc >> 8) & 0xFF, dtc & 0xFF, 0xFF]))
        if response is None or response[0] != 0x59:
            return None
        records = response[6:]
        result = {}
        for i in range(0, len(records) - 1, 2):
            if records[i] == 0x01:
                result["occurrence_counter"] = records[i + 1]
            elif records[i] == 0x02:
                result["aging_counter"] = records[i + 1]
        return result

    def clear_dtcs(self) -> None:
        self.client.clear_dtc(0xFFFFFF)

    def ecu_reset(self) -> None:
        self.client.ecu_reset(0x01)


def nrc_of(exception: NegativeResponseException) -> int:
    """The Negative Response Code carried by a udsoncan exception."""
    return exception.response.code


def nrc_name(code: int) -> str:
    return f"0x{code:02X} {NRC_NAMES.get(code, 'unknown')}"
