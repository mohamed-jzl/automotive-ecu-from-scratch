"""
CAN bus interface for the ECU test framework.

Wraps python-can so the test cases deal in ECU concepts ("wait for the next
vehicle status message") rather than in bus plumbing.

Supported backends
------------------
Selected with --interface on the command line:

    virtual   An in-process bus with no hardware at all. Nothing is connected,
              so the ECU never answers - but every piece of the framework
              (encoding, timing, assertions, reporting) still runs. Use it to
              develop and debug the tests themselves before touching a board.

    slcan     Serial-line CAN adapters (CANable, CANtact, USBtin). The cheap,
              widely available option - about 15 USD.
              Example:  --interface slcan --channel COM5

    pcan      PEAK PCAN-USB. The industry standard, and what you will meet in
              a professional test bench.
              Example:  --interface pcan --channel PCAN_USBBUS1

    socketcan Linux native CAN, including the `vcan0` virtual interface.
              Example:  --interface socketcan --channel can0

A note on timing accuracy
-------------------------
Timestamps come from the PC, not from the CAN controller, so they include USB
latency and host scheduler jitter - typically a few milliseconds. That is fine
for verifying a 100 ms transmission period. It is NOT sufficient to verify a
microsecond-level requirement; that needs a hardware-timestamping interface or
a CAN analyser. Knowing the limits of your measurement equipment is part of
being a test engineer, so the measured jitter is reported rather than hidden.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import Optional

try:
    import can
except ImportError as exc:                              # pragma: no cover
    raise SystemExit(
        "python-can is not installed.\n"
        "  pip install -r requirements.txt"
    ) from exc

from ecu_signals import (
    CAN_ID_PCM_ENGINE_STATUS,
    CrcError,
    EngineStatus,
    decode,
)


@dataclass
class ReceivedFrame:
    """One frame as it came off the bus, with its host-side timestamp."""

    can_id: int
    data: bytes
    timestamp: float

    def __str__(self) -> str:
        payload = " ".join(f"{b:02X}" for b in self.data)
        return f"0x{self.can_id:03X}  [{len(self.data)}]  {payload}"


@dataclass
class PeriodStatistics:
    """Measured transmission timing of a periodic message."""

    sample_count: int
    mean_ms: float
    min_ms: float
    max_ms: float
    jitter_ms: float        # max - min, the peak-to-peak spread

    def __str__(self) -> str:
        return (
            f"n={self.sample_count}  mean={self.mean_ms:.1f} ms  "
            f"min={self.min_ms:.1f}  max={self.max_ms:.1f}  "
            f"jitter={self.jitter_ms:.1f} ms"
        )


class EcuCanInterface:
    """Connection to the ECU under test."""

    def __init__(
        self,
        interface: str = "virtual",
        channel: str = "test",
        bitrate: int = 500_000,
    ) -> None:
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self._bus: Optional[can.BusABC] = None
        self._alive_counter = 0

    # -- connection ------------------------------------------------------

    def connect(self) -> None:
        """Open the bus. Raises if the adapter is missing or already in use."""
        kwargs = {"interface": self.interface, "channel": self.channel}

        # The virtual backend has no wire, so a bit rate would be meaningless.
        if self.interface != "virtual":
            kwargs["bitrate"] = self.bitrate

        self._bus = can.Bus(**kwargs)

        # Discard anything that arrived before the test started. Without this,
        # a test could "pass" on a frame produced by whatever ran previously.
        self.flush()

    def disconnect(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self) -> "EcuCanInterface":
        self.connect()
        return self

    def __exit__(self, *exc_info) -> None:
        self.disconnect()

    @property
    def bus(self) -> can.BusABC:
        if self._bus is None:
            raise RuntimeError("Bus is not connected - call connect() first")
        return self._bus

    def flush(self) -> int:
        """Drain every frame currently buffered. Returns how many were dropped."""
        dropped = 0
        while self.bus.recv(timeout=0.0) is not None:
            dropped += 1
        return dropped

    # -- transmit --------------------------------------------------------

    def send(self, can_id: int, payload: bytes) -> None:
        """Send one raw frame with a standard 11-bit identifier."""
        self.bus.send(
            can.Message(
                arbitration_id=can_id,
                data=payload,
                is_extended_id=False,
            )
        )

    def send_engine_status(
        self,
        rpm: int = 0,
        speed_kph: float = 0.0,
        coolant_temp_c: int = 20,
        engine_running: bool = False,
    ) -> EngineStatus:
        """
        Send PCM_EngineStatus, impersonating the powertrain ECU.

        This is restbus simulation: standing in for a node that is not on the
        bench so the ECU under test sees the network it expects.

        The alive counter advances automatically, because a receiver that
        implements E2E protection is entitled to reject a frame whose counter
        has not moved - and would be right to.
        """
        self._alive_counter = (self._alive_counter + 1) & 0xFF

        status = EngineStatus(
            engine_rpm=rpm,
            vehicle_speed_kph_x10=int(round(speed_kph * 10)),
            coolant_temp_c=coolant_temp_c,
            engine_running=engine_running,
            alive_counter=self._alive_counter,
        )

        self.send(CAN_ID_PCM_ENGINE_STATUS, status.pack())
        return status

    # -- receive ---------------------------------------------------------

    def receive(self, timeout: float = 1.0) -> Optional[ReceivedFrame]:
        """Wait for any frame. Returns None on timeout."""
        message = self.bus.recv(timeout=timeout)
        if message is None:
            return None

        return ReceivedFrame(
            can_id=message.arbitration_id,
            data=bytes(message.data),
            timestamp=message.timestamp,
        )

    def wait_for_id(self, can_id: int, timeout: float = 2.0) -> Optional[ReceivedFrame]:
        """
        Wait for a frame with a specific identifier, ignoring all others.

        The deadline is absolute rather than per-frame. A per-frame timeout
        would let heavy unrelated bus traffic extend the wait indefinitely,
        so a test could hang instead of failing.
        """
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            frame = self.receive(timeout=max(0.0, remaining))

            if frame is None:
                return None
            if frame.can_id == can_id:
                return frame

        return None

    def wait_for_message(self, can_id: int, timeout: float = 2.0):
        """
        Wait for a message and decode it.

        Returns the decoded dataclass, or None on timeout.
        Raises CrcError if the frame arrived but failed its integrity check -
        a corrupt frame is a test failure, not a timeout, and the two must
        never be confused when diagnosing a problem.
        """
        frame = self.wait_for_id(can_id, timeout)
        return None if frame is None else decode(frame.can_id, frame.data)

    def collect(self, can_id: int, duration_s: float) -> list[ReceivedFrame]:
        """Collect every frame with the given identifier for a fixed window."""
        deadline = time.monotonic() + duration_s
        frames: list[ReceivedFrame] = []

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            frame = self.receive(timeout=max(0.0, remaining))

            if frame is not None and frame.can_id == can_id:
                frames.append(frame)

        return frames

    # -- timing ----------------------------------------------------------

    def measure_period(
        self,
        can_id: int,
        duration_s: float = 2.0,
    ) -> Optional[PeriodStatistics]:
        """
        Measure the transmission period of a periodic message.

        Verifying that a message is sent *on time* matters as much as verifying
        its contents. A receiver that expects a 100 ms heartbeat will declare
        the sender dead if it drifts to 400 ms, no matter how correct the
        payload is.

        Returns None if fewer than two frames arrived - one frame gives no
        interval to measure, and reporting a period from it would be a lie.
        """
        frames = self.collect(can_id, duration_s)

        if len(frames) < 2:
            return None

        intervals_ms = [
            (frames[i].timestamp - frames[i - 1].timestamp) * 1000.0
            for i in range(1, len(frames))
        ]

        return PeriodStatistics(
            sample_count=len(intervals_ms),
            mean_ms=statistics.fmean(intervals_ms),
            min_ms=min(intervals_ms),
            max_ms=max(intervals_ms),
            jitter_ms=max(intervals_ms) - min(intervals_ms),
        )
