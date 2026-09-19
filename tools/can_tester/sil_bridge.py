"""
Software-in-the-loop bridge: runs the production C diagnostic stack on a PC
and connects it to a python-can virtual bus.

    +----------------------+     virtual CAN bus      +--------------------------+
    |  udsoncan + can-isotp | <---------------------> |  SilEcu (this file)       |
    |  (tester, Python)     |   0x7E0 / 0x7DF / 0x7E8 |    ctypes                 |
    +----------------------+                          |    sil_ecu.dll / .so      |
                                                      |    = src/diag/*.c         |
                                                      |    + nvm_store.c          |
                                                      +--------------------------+

The C library is compiled on demand from the firmware sources (see
build_sil_library), so the SIL target can never drift away from the code that
runs on the STM32: it IS that code.

Timing note: the bridge polls the stack every ~2 ms, but Windows timers often
have ~15 ms granularity. That is fine for a 50 ms P2 deadline and the protocol
timeouts, and it is one reason SIL results are about logic, not timing -
timing is verified on the real ECU (HIL).
"""

from __future__ import annotations

import ctypes
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import can

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "firmware" / "automotive-body-ecu" / "src"
SIL_DIR = ROOT / "tools" / "sil"

sys.path.insert(0, str(ROOT / "tools"))
from toolchain import compile_c, require_c_compiler  # noqa: E402

# The production sources compiled into the SIL library.
PRODUCTION_SOURCES = [
    FW / "diag" / "isotp.c",
    FW / "diag" / "uds_server.c",
    FW / "diag" / "uds_security.c",
    FW / "diag" / "dtc_manager.c",
    FW / "diag" / "diag_manager.c",
    FW / "services" / "nvm_store.c",
]

_TX_CALLBACK = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_uint32,
                                ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8)


def _library_name() -> str:
    if sys.platform == "win32":
        return "sil_ecu.dll"
    if sys.platform == "darwin":
        return "sil_ecu.dylib"
    return "sil_ecu.so"


def build_sil_library(force: bool = False) -> Path:
    """
    Compile the SIL shared library, unless it is already newer than every source.

    Uses -Werror like the unit tests: the SIL build is held to the same
    standard as the rest of the host-compiled code.
    """
    output = SIL_DIR / "build" / _library_name()
    sources = PRODUCTION_SOURCES + [SIL_DIR / "sil_ecu.c"]
    headers = list(FW.rglob("*.h"))

    if (not force) and output.exists():
        newest_input = max(p.stat().st_mtime for p in sources + headers)
        if output.stat().st_mtime >= newest_input:
            return output

    output.parent.mkdir(parents=True, exist_ok=True)
    compiler = require_c_compiler()

    args = ["-shared", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
            *(f"-I{FW / d}" for d in ("diag", "services", "config")),
            "-o", str(output), *(str(s) for s in sources)]
    if sys.platform != "win32":
        args.insert(0, "-fPIC")

    result = compile_c(compiler, args)
    if result.returncode != 0:
        raise RuntimeError(f"SIL build failed:\n{result.stdout}{result.stderr}")
    return output


class SilEcu:
    """The simulated ECU. Use as a context manager."""

    def __init__(self, channel: str = "sil", erase_nvm: bool = True) -> None:
        self.channel = channel
        self._erase_nvm = erase_nvm
        self._lib = ctypes.CDLL(str(build_sil_library()))
        self._declare_signatures()

        self._bus: Optional[can.BusABC] = None
        self._lock = threading.RLock()          # C code is not thread-safe
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._t0 = time.monotonic()
        self.reset_count = 0

        # Keep a reference: if Python garbage-collected the callback object,
        # the C side would call freed memory.
        self._tx_callback = _TX_CALLBACK(self._on_tx)
        self._lib.sil_set_tx_callback(self._tx_callback)

    def _declare_signatures(self) -> None:
        lib = self._lib
        lib.sil_set_tx_callback.argtypes = [_TX_CALLBACK]
        lib.sil_boot.argtypes = [ctypes.c_uint32, ctypes.c_int]
        lib.sil_boot.restype = ctypes.c_int
        lib.sil_rx_frame.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8),
                                     ctypes.c_uint8, ctypes.c_uint32]
        lib.sil_poll.argtypes = [ctypes.c_uint32]
        lib.sil_take_reset.restype = ctypes.c_int
        lib.sil_set_battery_mv.argtypes = [ctypes.c_uint16]
        lib.sil_set_vehicle_state.argtypes = [ctypes.c_uint8]
        lib.sil_report_fault.argtypes = [ctypes.c_uint8, ctypes.c_int]
        lib.sil_lamp_test_active.restype = ctypes.c_int

    # -- time and frames -------------------------------------------------

    def now_ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF

    def _on_tx(self, can_id, data_ptr, dlc) -> int:
        payload = bytes(data_ptr[i] for i in range(dlc))
        self._bus.send(can.Message(arbitration_id=can_id, data=payload,
                                   is_extended_id=False))
        return 1

    def _run(self) -> None:
        while not self._stop.is_set():
            message = self._bus.recv(timeout=0.002)
            with self._lock:
                now = self.now_ms()
                if message is not None and not message.is_extended_id:
                    buffer = (ctypes.c_uint8 * 8)(*bytes(message.data).ljust(8, b"\x00"))
                    self._lib.sil_rx_frame(message.arbitration_id, buffer,
                                           message.dlc, now)
                self._lib.sil_poll(now)

                # The stack asked for an ECU reset: simulate a reboot. The
                # flash is kept, exactly as on real hardware.
                if self._lib.sil_take_reset():
                    self.reset_count += 1
                    self._lib.sil_boot(self.now_ms(), 0)

    # -- lifecycle -------------------------------------------------------

    def start(self) -> "SilEcu":
        self._bus = can.Bus(interface="virtual", channel=self.channel)
        with self._lock:
            self._lib.sil_boot(self.now_ms(), 1 if self._erase_nvm else 0)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sil-ecu", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self) -> "SilEcu":
        return self.start()

    def __exit__(self, *exc_info) -> None:
        self.stop()

    # -- test hooks (things only a simulation can do) --------------------

    def power_cycle(self, erase_nvm: bool = False) -> None:
        """Cold restart. erase_nvm=True gives a factory-fresh ECU."""
        with self._lock:
            self._lib.sil_boot(self.now_ms(), 1 if erase_nvm else 0)

    def set_battery_mv(self, millivolts: int) -> None:
        with self._lock:
            self._lib.sil_set_battery_mv(millivolts)

    def set_vehicle_state(self, state: int) -> None:
        with self._lock:
            self._lib.sil_set_vehicle_state(state)

    def report_fault(self, fault_id: int, failed: bool) -> None:
        """Inject a matured fault result, as body_control.c would report it."""
        with self._lock:
            self._lib.sil_report_fault(fault_id, 1 if failed else 0)

    def start_operation_cycle(self) -> None:
        with self._lock:
            self._lib.sil_start_operation_cycle()

    def lamp_test_active(self) -> bool:
        with self._lock:
            return bool(self._lib.sil_lamp_test_active())
