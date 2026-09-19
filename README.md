# Automotive Body Control ECU — STM32

[![CI](https://github.com/mohamed-jzl/automotive-ecu-from-scratch/actions/workflows/ci.yml/badge.svg)](https://github.com/mohamed-jzl/automotive-ecu-from-scratch/actions/workflows/ci.yml)
![Platform](https://img.shields.io/badge/platform-STM32F446RE-blue)
![Language](https://img.shields.io/badge/language-Embedded%20C-green)
![Diagnostics](https://img.shields.io/badge/diagnostics-UDS%20ISO%2014229-purple)
![Transport](https://img.shields.io/badge/transport-ISO--TP%2015765--2-purple)
![Tests](https://img.shields.io/badge/tests-186%20unit%20%2B%2036%20system-brightgreen)

A simplified automotive **Body Control Module** built from scratch on an STM32
Nucleo-F446RE: layered bare-metal firmware, a time-triggered scheduler, fault
management, CAN communication with end-to-end protection, and a complete
**UDS diagnostic stack** (ISO 14229-1 over ISO-TP) with persistent DTC memory,
verified by 186 unit tests and by software-in-the-loop system tests driven by
an independent, open-source UDS client.

> **Scope, stated honestly.** This is an educational prototype, not production
> firmware. It is not ISO 26262 certified, not AUTOSAR conformant, and its
> seed/key algorithm is deliberately simple and not secure. What it does is
> apply the *engineering practices* those standards exist to enforce —
> requirements traceability, layered architecture, verification at several
> levels, honest coverage reporting — at a scale one person can build and
> fully explain.

---

## What it does

| | |
|---|---|
| **Vehicle state machine** | `OFF → ACC → ON → RUN`, plus a `FAULT` state reachable from anywhere and overridable by nothing |
| **Body functions** | Ignition, turn indicators at 1.5 Hz (UNECE R6), hazard mode, headlights, PWM brake light at two intensities |
| **Battery monitoring** | 12-bit ADC, 8× oversampling, under/overvoltage detection with hysteresis |
| **Fault management** | Maturation and healing — quick to suspect a fault, slow to dismiss one |
| **CAN** | 500 kbit/s, interrupt-driven reception, CRC-8 and alive counter on every frame, receive timeouts |
| **UDS diagnostics** | 10 services, 9 DIDs, sessions, seed/key security with lockout, ECU reset, lamp self-test routine |
| **DTC memory** | 6 DTCs (P0562, P0563, U0001…), full ISO 14229 status byte, freeze frames, occurrence counters, 40-cycle aging |
| **Persistence** | VIN and DTCs survive resets in a power-loss-safe flash log; flash is never erased at runtime |
| **Reliability** | Independent watchdog, reset-cause reporting, worst-case execution time measurement |

### A real diagnostic session

Captured from the production C code, byte for byte:

```
0x7E0  02 10 03 CC CC CC CC CC     extended session
0x7E8  06 50 03 00 32 01 F4 CC       -> OK, P2 = 50 ms, P2* = 5 s
0x7E0  02 27 01 CC CC CC CC CC     request seed
0x7E8  06 67 01 21 21 03 83 CC       -> seed 0x21210383
0x7E0  06 27 02 8E B4 D6 2F CC     send key
0x7E8  02 67 02 CC CC CC CC CC       -> unlocked
0x7E0  10 14 2E F1 90 57 44 42     write VIN: 20 bytes, ISO-TP First Frame
0x7E8  30 00 05 CC CC CC CC CC       -> Flow Control: go, 5 ms between frames
0x7E0  21 32 30 33 30 34 36 31       Consecutive Frame 1
0x7E0  22 41 31 32 33 34 35 36       Consecutive Frame 2
0x7E8  03 6E F1 90 CC CC CC CC       -> VIN written
0x7E0  03 19 02 08 CC CC CC CC     which DTCs are confirmed?
0x7E8  07 59 02 FF 05 62 00 AF       -> P0562-00 (voltage low), status 0xAF
```

```
$ python diag_tool.py --sil dtc
  1 DTC(s) stored:
    P0562-00  System voltage low
      status 0x2E: testFailedThisOperationCycle, pendingDTC, confirmedDTC, testFailedSinceLastClear
      freeze frame: battery 10.350 V, vehicle state OFF
      occurrences: 1, clean cycles since last failure: 0
```

Full interface: **[docs/uds_specification.md](docs/uds_specification.md)**

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  APPLICATION    body_control · vehicle_state_machine · diag_app  │
├──────────────────────────────────────────────────────────────────┤
│  SERVICES       scheduler · fault_manager · logger · can_manager │
│                 can_signals · nvm_store                          │
│  DIAGNOSTICS    isotp · uds_server · dtc_manager · uds_security  │
│                 diag_manager                                     │
├──────────────────────────────────────────────────────────────────┤
│  DRIVERS        gpio · adc · pwm · uart · can · watchdog         │
│                 flash · mcu        — the ONLY layer calling HAL  │
├──────────────────────────────────────────────────────────────────┤
│  STM32 HAL      ST-generated                                     │
└──────────────────────────────────────────────────────────────────┘
```

The whole diagnostic stack, plus the state machine, fault manager, CAN
encoding and flash store, is **pure**: no HAL, no registers, no clock. Hardware
access arrives as function pointers (dependency injection). That single
decision is what lets the same C code run on the STM32, in 186 unit tests, and
as a software-in-the-loop target on a PC.

Full rationale: **[docs/architecture/software_architecture.md](docs/architecture/software_architecture.md)**

---

## Verification

| Level | Count | Runs on | What it proves |
|---|---|---|---|
| Unit tests | 186 | PC, no hardware | Each pure module is correct in isolation |
| Diagnostic system tests (SIL) | 24 | PC — **production C stack** vs **udsoncan** | The UDS stack interoperates with an independent tester |
| Body system tests | 12 | Real ECU over CAN (or simulator) | The assembled ECU behaves correctly on the bus |
| Bench integration | 16 | Real hardware, manual | Wiring, interrupts, flash timing, lamps |

```bash
python tools/run_unit_tests.py                         # 186 unit tests (make-free; or: cd tests && make)
cd tools/can_tester
python run_diag_tests.py --sil --report report.html    # 24 UDS tests, no hardware needed
python diag_tool.py --sil demo                         # guided diagnostic demo
```

```
  [PASS] DG-013  Seed/key unlock                 REQ-130
  [PASS] DG-016  VIN survives ECUReset           REQ-141
  [PASS] DG-017  DTC confirmed after a fault     REQ-150
  [PASS] DG-024  Security lockout                REQ-132
  ...
  24 tests: 24 passed, 0 failed, 0 skipped, 0 errors
  requirements verified: 22 of 22
```

Highlights:

- **An independent tester.** The diagnostic tests use udsoncan and can-isotp,
  open-source ISO 14229 / 15765 implementations written by other people. A
  hand-written client would share the ECU's misunderstandings; a third-party
  one does not.
- **Power cuts on demand.** The flash store is tested against a RAM model that
  obeys flash physics, with writes cut off mid-record — the previous record
  must survive.
- **Standard check vectors.** CRC-8/SAE-J1850 (`0x4B`) and CRC-32 (`0xCBF43926`)
  pinned to their published values.
- **Byte-exact expectations**, computed independently from the specifications.
- **Honest results.** A skipped test is never counted as a pass; an empty bus
  reports `INCONCLUSIVE`, not `PASSED`.

---

## Requirements traceability

67 requirements, each linked to its implementation and its tests in
**[docs/traceability_matrix.md](docs/traceability_matrix.md)**.

| | |
|---|---|
| Covered by automated tests | 56 (84%) |
| Covered by bench tests only | 8 (12%) |
| Covered by code review only | 3 (4%) |

The uncovered ones are named, with what it would take to cover each.

---

## Getting started

**Without hardware** (any PC with Python 3.10+):

```bash
pip install -r tools/can_tester/requirements.txt
pip install ziglang            # only if you have no C compiler (Windows)
python tools/run_unit_tests.py
cd tools/can_tester && python diag_tool.py --sil demo
```

**With hardware:** STM32 Nucleo-F446RE, a few LEDs, buttons and a potentiometer
(~30 USD); a CAN transceiver and USB-CAN adapter for the CAN and diagnostic
parts. Wiring, bill of materials and bring-up checklist:
**[docs/hardware_setup.md](docs/hardware_setup.md)**.

```
STM32CubeIDE:  File → Import → Existing Projects into Workspace
                 → firmware/automotive-body-ecu     then Ctrl+B, F11
```

Serial console at **115200 8N1**:

```
===========================================
  Automotive Body Control ECU
[          8][INFO ] Firmware v0.2.0  target STM32F446RE
[         10][INFO ] Reset cause: power-on or manual reset
===========================================
[         14][INFO ] Diagnostics ready - UDS on 0x7E0/0x7E8, NVM 0% used
[         15][INFO ] Body control initialised - 5 tasks registered
[       1420][INFO ] State: OFF -> ACC  (event IGN_SHORT)
[       8100][ERROR] Fault SET:     BATT_UNDERVOLT
```

### Resource usage

```
   text    data     bss
  45796     144    4440
```

17.5% of the 256 KB code region (sectors 6-7 are reserved for DTC storage),
3.4% of RAM. **No dynamic allocation anywhere.**

---

## Repository layout

```
firmware/automotive-body-ecu/src/
  app/         body_control, vehicle_state_machine, diag_app (DIDs, DTCs, routine)
  diag/        isotp, uds_server, uds_security, dtc_manager, diag_manager
  services/    scheduler, fault_manager, logger, can_signals, can_manager, nvm_store
  drivers/     gpio, adc, pwm, uart, can, watchdog, flash, mcu
  config/      ecu_config.h, pin_config.h  ← every tunable value, with its derivation

tests/                186 unit tests (9 suites) + Unity-compatible framework
tools/run_unit_tests.py   make-free unit test runner
tools/sil/            SIL target: production diagnostic code built for the PC
tools/can_tester/     Python system tests, UDS client, diag_tool, SIL bridge, reports
docs/                 requirements, architecture, CAN + UDS specs, test plan, traceability
```

---

## Documentation

| Document | Contents |
|---|---|
| [System requirements](docs/requirements/system_requirements.md) | 67 testable requirements, scope exclusions, known limitations |
| [Software architecture](docs/architecture/software_architecture.md) | Layers, dependency injection, execution model, flash memory map |
| [UDS specification](docs/uds_specification.md) | Services, DIDs, DTCs, status bits, security, real CAN trace |
| [CAN specification](docs/can_specification.md) | Bit timing, message layouts, E2E protection, DBC equivalent |
| [Test plan](docs/test_plan.md) | Unit, SIL, HIL and bench levels — and what each cannot prove |
| [Traceability matrix](docs/traceability_matrix.md) | Requirement → implementation → test |
| [Hardware setup](docs/hardware_setup.md) | Wiring, bring-up checklist, diagnostics on the bench, troubleshooting |
| [Lessons learned](docs/lessons_learned.md) | Real problems hit, design decisions, what would be done differently |

---

## Concepts demonstrated

Embedded C · bare-metal firmware · layered architecture · dependency injection ·
GPIO, ADC, PWM, timers, UART · CAN 2.0A · interrupts and lock-free ring buffers ·
state machines · cooperative scheduling · watchdog · fault maturation ·
**UDS (ISO 14229-1)** · **ISO-TP (ISO 15765-2)** · DTC status and aging · freeze
frames · seed/key security · flash EEPROM emulation · power-loss safety ·
CRC and end-to-end protection · unit testing · **software-in-the-loop** ·
system testing with an independent tester · requirements traceability · CI

## Roadmap

- **v0.3 — UDS bootloader**: programming session and firmware download
  (`0x34`/`0x36`/`0x37`) with a signed-image check.
- HIL test bench: a second Nucleo generating the battery voltage and button
  inputs, so the bench tests run unattended.

---

MIT licence — see [LICENSE](LICENSE).
**Mohamed Amine Jazoul** — Electrical Engineering, ENSAM Rabat
