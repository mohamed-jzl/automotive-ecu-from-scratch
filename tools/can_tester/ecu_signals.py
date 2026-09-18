"""
CAN signal encoding and decoding - Python mirror of firmware/src/services/can_signals.c

Why this file exists twice, in two languages
--------------------------------------------
The ECU packs signals into bytes in C. The test framework must unpack those
same bytes in Python to check them. Both implementations follow
docs/can_specification.md, and they are written *independently* from that
document rather than one being a translation of the other.

That independence is the point. If the C encoder has a bug, a Python decoder
derived from the same buggy code would decode it "correctly" and the test
would pass. Two implementations from one written specification means a
mismatch reveals a real defect - in the C, in the Python, or in the
specification itself. Any of those three is worth finding.

In production this duplication is avoided by generating both sides from a
single DBC database file (Vector CANdb++, or the open-source `cantools`).
Writing it out by hand here makes the mechanism visible, and is exactly what
you would be asked to explain in an interview.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

# =============================================================================
#  Message identifiers - must match can_signals.h
# =============================================================================

CAN_ID_BCM_VEHICLE_STATUS = 0x100   # transmitted by the Body ECU, every 100 ms
CAN_ID_BCM_DIAGNOSTICS    = 0x101   # transmitted by the Body ECU, every 500 ms
CAN_ID_PCM_ENGINE_STATUS  = 0x200   # received by the Body ECU, every 100 ms

CAN_MESSAGE_DLC = 8

# Bit positions inside byte 0 of BCM_VehicleStatus
VS_STATE_MASK          = 0x07
VS_IGNITION_BIT        = 0x08
VS_BRAKE_BIT           = 0x10
VS_INDICATOR_LEFT_BIT  = 0x20
VS_INDICATOR_RIGHT_BIT = 0x40
VS_HEADLIGHT_BIT       = 0x80

VS_FAULT_COUNT_MASK    = 0x0F
ES_ENGINE_RUNNING_BIT  = 0x01

# Coolant temperature is sent as an unsigned byte with a +40 offset, so the
# automotive range of -40..+215 degrees C fits without sign handling.
ES_COOLANT_TEMP_OFFSET = 40

# =============================================================================
#  Vehicle states - must match VehicleState_t in vehicle_state_machine.h
# =============================================================================

VEHICLE_STATE_OFF   = 0
VEHICLE_STATE_ACC   = 1
VEHICLE_STATE_ON    = 2
VEHICLE_STATE_RUN   = 3
VEHICLE_STATE_FAULT = 4

VEHICLE_STATE_NAMES = {
    VEHICLE_STATE_OFF:   "OFF",
    VEHICLE_STATE_ACC:   "ACC",
    VEHICLE_STATE_ON:    "ON",
    VEHICLE_STATE_RUN:   "RUN",
    VEHICLE_STATE_FAULT: "FAULT",
}

# =============================================================================
#  Fault identifiers - must match FaultId_t in fault_manager.h
# =============================================================================

FAULT_BATTERY_UNDERVOLTAGE = 0
FAULT_BATTERY_OVERVOLTAGE  = 1
FAULT_ADC_FAILURE          = 2
FAULT_CAN_BUS_OFF          = 3
FAULT_CAN_TX_FAILURE       = 4
FAULT_TASK_OVERRUN         = 5

FAULT_NAMES = {
    FAULT_BATTERY_UNDERVOLTAGE: "BATT_UNDERVOLT",
    FAULT_BATTERY_OVERVOLTAGE:  "BATT_OVERVOLT",
    FAULT_ADC_FAILURE:          "ADC_FAILURE",
    FAULT_CAN_BUS_OFF:          "CAN_BUS_OFF",
    FAULT_CAN_TX_FAILURE:       "CAN_TX_FAIL",
    FAULT_TASK_OVERRUN:         "TASK_OVERRUN",
}


def decode_fault_bitmask(bitmask: int) -> list[str]:
    """Turn a fault bitmask into a list of readable fault names."""
    return [name for bit, name in FAULT_NAMES.items() if bitmask & (1 << bit)]


# =============================================================================
#  CRC-8 / SAE J1850
# =============================================================================

def crc8(data: bytes) -> int:
    """
    CRC-8 with SAE J1850 parameters: polynomial 0x1D, init 0xFF, final XOR 0xFF.

    These are the parameters used by AUTOSAR E2E Profile 1. The polynomial has
    a Hamming distance of 4 across an 8-byte payload, so it detects every
    1-, 2- and 3-bit error.

    Verified against the published check value: crc8(b"123456789") == 0x4B.
    """
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


class CrcError(ValueError):
    """Raised when a received payload fails its end-to-end CRC check."""


def _append_crc(payload: bytearray) -> bytes:
    """Fill byte 7 with the CRC of bytes 0..6."""
    payload[7] = crc8(bytes(payload[:7]))
    return bytes(payload)


def _verify_crc(payload: bytes) -> None:
    """Raise CrcError if byte 7 does not match the CRC of bytes 0..6."""
    if len(payload) != CAN_MESSAGE_DLC:
        raise CrcError(f"expected {CAN_MESSAGE_DLC} bytes, got {len(payload)}")

    expected = crc8(payload[:7])
    if payload[7] != expected:
        raise CrcError(
            f"CRC mismatch: frame carries 0x{payload[7]:02X}, "
            f"computed 0x{expected:02X}"
        )


# =============================================================================
#  BCM_VehicleStatus (0x100)
# =============================================================================

@dataclass
class VehicleStatus:
    """Contents of the BCM_VehicleStatus message."""

    vehicle_state: int = VEHICLE_STATE_OFF
    ignition_on: bool = False
    brake_active: bool = False
    indicator_left: bool = False
    indicator_right: bool = False
    headlight_on: bool = False
    battery_mv: int = 0
    fault_count: int = 0
    fault_bitmask: int = 0
    alive_counter: int = 0

    @property
    def state_name(self) -> str:
        return VEHICLE_STATE_NAMES.get(self.vehicle_state, "INVALID")

    @property
    def active_faults(self) -> list[str]:
        return decode_fault_bitmask(self.fault_bitmask)

    @property
    def battery_volts(self) -> float:
        return self.battery_mv / 1000.0

    def pack(self) -> bytes:
        """Encode into the 8-byte wire format, CRC included."""
        payload = bytearray(CAN_MESSAGE_DLC)

        byte0 = self.vehicle_state & VS_STATE_MASK
        if self.ignition_on:
            byte0 |= VS_IGNITION_BIT
        if self.brake_active:
            byte0 |= VS_BRAKE_BIT
        if self.indicator_left:
            byte0 |= VS_INDICATOR_LEFT_BIT
        if self.indicator_right:
            byte0 |= VS_INDICATOR_RIGHT_BIT
        if self.headlight_on:
            byte0 |= VS_HEADLIGHT_BIT
        payload[0] = byte0

        payload[1] = self.battery_mv & 0xFF          # little-endian
        payload[2] = (self.battery_mv >> 8) & 0xFF
        payload[3] = self.fault_count & VS_FAULT_COUNT_MASK
        payload[4] = self.fault_bitmask & 0xFF
        payload[5] = (self.fault_bitmask >> 8) & 0xFF
        payload[6] = self.alive_counter & 0xFF

        return _append_crc(payload)

    @classmethod
    def unpack(cls, payload: bytes) -> "VehicleStatus":
        """
        Decode from the wire format.

        Raises CrcError if the payload is corrupt. Raising rather than
        returning a partially-decoded object is deliberate: a caller cannot
        accidentally use data that failed its integrity check.
        """
        _verify_crc(payload)

        byte0 = payload[0]
        return cls(
            vehicle_state=byte0 & VS_STATE_MASK,
            ignition_on=bool(byte0 & VS_IGNITION_BIT),
            brake_active=bool(byte0 & VS_BRAKE_BIT),
            indicator_left=bool(byte0 & VS_INDICATOR_LEFT_BIT),
            indicator_right=bool(byte0 & VS_INDICATOR_RIGHT_BIT),
            headlight_on=bool(byte0 & VS_HEADLIGHT_BIT),
            battery_mv=payload[1] | (payload[2] << 8),
            fault_count=payload[3] & VS_FAULT_COUNT_MASK,
            fault_bitmask=payload[4] | (payload[5] << 8),
            alive_counter=payload[6],
        )


# =============================================================================
#  BCM_DIAGNOSTICS (0x101)
# =============================================================================

@dataclass
class Diagnostics:
    """Contents of the BCM_DIAGNOSTICS message - ECU health telemetry."""

    uptime_seconds: int = 0
    task_overrun_count: int = 0
    max_task_duration_ms: int = 0
    can_tx_failures: int = 0
    alive_counter: int = 0

    def pack(self) -> bytes:
        payload = bytearray(CAN_MESSAGE_DLC)
        payload[0] = self.uptime_seconds & 0xFF
        payload[1] = (self.uptime_seconds >> 8) & 0xFF
        payload[2] = self.task_overrun_count & 0xFF
        payload[3] = (self.task_overrun_count >> 8) & 0xFF
        payload[4] = self.max_task_duration_ms & 0xFF
        payload[5] = self.can_tx_failures & 0xFF
        payload[6] = self.alive_counter & 0xFF
        return _append_crc(payload)

    @classmethod
    def unpack(cls, payload: bytes) -> "Diagnostics":
        _verify_crc(payload)
        return cls(
            uptime_seconds=payload[0] | (payload[1] << 8),
            task_overrun_count=payload[2] | (payload[3] << 8),
            max_task_duration_ms=payload[4],
            can_tx_failures=payload[5],
            alive_counter=payload[6],
        )


# =============================================================================
#  PCM_EngineStatus (0x200)
# =============================================================================

@dataclass
class EngineStatus:
    """
    Contents of the PCM_EngineStatus message.

    The test framework *sends* this to stimulate the Body ECU, standing in for
    a powertrain ECU that is not on the bench. Simulating a neighbouring ECU
    in software is called restbus simulation, and it is how automotive
    integration testing is done without assembling an entire vehicle.
    """

    engine_rpm: int = 0
    vehicle_speed_kph_x10: int = 0
    coolant_temp_c: int = 0
    engine_running: bool = False
    alive_counter: int = 0

    @property
    def vehicle_speed_kph(self) -> float:
        return self.vehicle_speed_kph_x10 / 10.0

    def pack(self) -> bytes:
        payload = bytearray(CAN_MESSAGE_DLC)
        payload[0] = self.engine_rpm & 0xFF
        payload[1] = (self.engine_rpm >> 8) & 0xFF
        payload[2] = self.vehicle_speed_kph_x10 & 0xFF
        payload[3] = (self.vehicle_speed_kph_x10 >> 8) & 0xFF
        payload[4] = (self.coolant_temp_c + ES_COOLANT_TEMP_OFFSET) & 0xFF
        payload[5] = ES_ENGINE_RUNNING_BIT if self.engine_running else 0x00
        payload[6] = self.alive_counter & 0xFF
        return _append_crc(payload)

    @classmethod
    def unpack(cls, payload: bytes) -> "EngineStatus":
        _verify_crc(payload)
        return cls(
            engine_rpm=payload[0] | (payload[1] << 8),
            vehicle_speed_kph_x10=payload[2] | (payload[3] << 8),
            coolant_temp_c=payload[4] - ES_COOLANT_TEMP_OFFSET,
            engine_running=bool(payload[5] & ES_ENGINE_RUNNING_BIT),
            alive_counter=payload[6],
        )


# =============================================================================
#  Dispatch helper
# =============================================================================

DECODERS = {
    CAN_ID_BCM_VEHICLE_STATUS: VehicleStatus,
    CAN_ID_BCM_DIAGNOSTICS:    Diagnostics,
    CAN_ID_PCM_ENGINE_STATUS:  EngineStatus,
}


def decode(can_id: int, payload: bytes) -> Optional[object]:
    """
    Decode any known message by identifier.

    Returns None for an unknown identifier - on a real vehicle bus most
    traffic belongs to other nodes and is simply not our concern.
    Raises CrcError if a known message fails its integrity check.
    """
    decoder = DECODERS.get(can_id)
    return decoder.unpack(payload) if decoder else None
