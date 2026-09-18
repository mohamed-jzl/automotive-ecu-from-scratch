# Automotive Body Control ECU — STM32

[![CI](https://github.com/YOUR_USERNAME/automotive-ecu-from-scratch/actions/workflows/ci.yml/badge.svg)](https://github.com/YOUR_USERNAME/automotive-ecu-from-scratch/actions/workflows/ci.yml)
![Platform](https://img.shields.io/badge/platform-STM32F446RE-blue)
![Language](https://img.shields.io/badge/language-Embedded%20C-green)
![Bus](https://img.shields.io/badge/bus-CAN%20500kbit%2Fs-orange)
![Tests](https://img.shields.io/badge/tests-67%20unit%20%2B%2012%20system-brightgreen)

A simplified automotive **Body Control Module** built from scratch on an STM32
Nucleo-F446RE: layered bare-metal firmware, a time-triggered scheduler, fault
management with diagnostic maturation, CAN communication with end-to-end
protection, a host-side unit test suite, and a Python system-test framework
that drives the ECU over the bus and produces an HTML report.

> **Scope, stated honestly.** This is an educational prototype, not production
> firmware. It is not ISO 26262 certified, not AUTOSAR conformant, and claims
> no ASIL. What it does is apply the *engineering practices* those standards
> exist to enforce — requirements traceability, layered architecture,
> verification at multiple levels, honest coverage reporting — at a scale one
> person can build and fully explain.

---

## What it does

| | |
|---|---|
| **Vehicle state machine** | `OFF → ACC → ON → RUN`, plus a `FAULT` state reachable from anywhere and overridable by nothing |
| **Digital inputs** | Ignition, brake, turn indicators — non-blocking debounce, short-press vs long-press discrimination |
| **Lighting outputs** | Headlights, turn indicators flashing at 1.5 Hz (UNECE R6), hazard mode, PWM brake light at two intensities |
| **Battery monitoring** | 12-bit ADC behind a resistor divider, 8× oversampling, under/overvoltage detection with hysteresis |
| **Fault management** | Maturation and healing counters — quick to suspect a fault, slow to dismiss one |
| **CAN communication** | 500 kbit/s, 3 messages, CRC-8 and alive counters on every frame, receive-timeout handling |
| **Reliability** | Independent watchdog, reset-cause reporting, worst-case execution time measurement |
| **Diagnostics** | Timestamped severity-tagged UART logging, status LED that encodes the operating mode |

---

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  APPLICATION      body_control.c · vehicle_state_machine.c   │
├──────────────────────────────────────────────────────────────┤
│  SERVICES         scheduler · fault_manager · logger         │
│                   can_signals · can_manager                  │
├──────────────────────────────────────────────────────────────┤
│  DRIVERS          gpio · adc · pwm · uart · can · watchdog   │
│                   the ONLY layer permitted to call the HAL   │
├──────────────────────────────────────────────────────────────┤
│  STM32 HAL        ST-generated                               │
├──────────────────────────────────────────────────────────────┤
│  HARDWARE         GPIO · TIM3 · ADC1 · USART2 · CAN1 · IWDG  │
└──────────────────────────────────────────────────────────────┘
```

Four modules are deliberately **pure** — no HAL include, no register access,
no time source: the state machine, the fault manager, the CAN signal encoding,
and the ADC conversion. That is what makes them testable on a PC, and it is
the single most important design decision in the project.

**Execution model:** cooperative time-triggered scheduler, no RTOS. Four tasks
at 10/50/100/500 ms. Tasks run to completion, so there are no races between
them, no mutexes, and one stack. The scheduler measures every activation and
reports the worst case — which is the evidence that the timing design actually
closes rather than an assumption that it does.

Full rationale: **[docs/architecture/software_architecture.md](docs/architecture/software_architecture.md)**

---

## Verification

| Level | Count | Runs on | What it proves |
|---|---|---|---|
| Unit tests | 67 | Host PC, no hardware | Each pure module is correct in isolation |
| Integration | 13 | Bench, manual | Pins are wired as specified, peripherals configured |
| System tests | 12 | Real ECU over CAN | The assembled ECU behaves correctly from outside |

```bash
cd tests && make          # 67 unit tests, -Wall -Wextra -Werror
```

Highlights worth a look:

- **Exhaustive sweeps.** All 25 state/event combinations; all 4096 ADC readings
  checked for monotonicity. Affordable only because the modules are pure.
- **Independently derived expectations.** The CAN byte-layout tests assert
  against arrays computed in Python from the written specification, not against
  the encoder's own output — so a bug cannot hide by appearing in both.
- **Standard check vectors.** The CRC is pinned to the published
  CRC-8/SAE-J1850 check value (`CRC("123456789") == 0x4B`), not to our own
  interpretation of the standard.
- **The framework proves itself.** The system suite must report `INCONCLUSIVE`
  against an empty bus and `12/12 PASSED` against a known-good software model.
  A framework that cannot distinguish those two verifies nothing — CI asserts
  both on every commit.

```bash
cd tools/can_tester
pip install -r requirements.txt
python ecu_simulator.py --self-test          # prove the framework, no hardware
python run_tests.py --interface slcan --channel COM5 --report report.html
```

```
  [PASS] TC-001  Vehicle status is transmitted          REQ-010
  [PASS] TC-002  Vehicle status period is 100 ms        REQ-011
  [PASS] TC-004  Every frame passes its CRC             REQ-012
  [PASS] TC-005  Alive counter increments and wraps     REQ-013
  ...
  12 tests: 12 passed, 0 failed, 0 skipped, 0 errors
  requirements verified: 12 of 12
```

---

## Requirements traceability

36 requirements, each linked to the code implementing it and the test verifying
it in **[docs/traceability_matrix.md](docs/traceability_matrix.md)**.

| | |
|---|---|
| Covered by automated tests | 27 (75%) |
| Covered by manual bench tests | 6 (17%) |
| Covered by code review only | 3 (8%) |

The three untested requirements are named explicitly, with what it would take
to cover each. A matrix showing 100% coverage usually means the requirements
were written to match the tests rather than the other way round.

---

## Getting started

### You need

STM32 Nucleo-F446RE (~17 USD), four LEDs and resistors, three buttons, a
potentiometer. A CAN transceiver and adapter are needed only for the CAN
phases — everything up to fault management works without them. Full bill of
materials and wiring: **[docs/hardware_setup.md](docs/hardware_setup.md)**.

### Build the firmware

```
In STM32CubeIDE:  File → Import → Existing Projects into Workspace
                    → firmware/automotive-body-ecu
Then Ctrl+B to build, F11 to flash.
```

Connect a serial terminal at **115200 8N1** to the ST-Link virtual COM port:

```
===========================================
  Automotive Body Control ECU
[          8][INFO ] Firmware v0.1.0  target STM32F446RE
[          9][INFO ] System clock 84000000 Hz
[         10][INFO ] Reset cause: power-on or manual reset
===========================================
[         12][INFO ] Body control initialised - 4 tasks registered
[       1420][INFO ] State: OFF -> ACC  (event IGN_SHORT)
[       3350][INFO ] State: ON -> RUN  (event IGN_LONG)
[       8100][ERROR] Fault SET:     BATT_UNDERVOLT
[       8102][INFO ] State: RUN -> FAULT  (event FAULT_SET)
```

### Resource usage

```
   text    data     bss     dec     hex
  29056     104    2488   31648    7ba0
```

5.5% of flash, 1.9% of RAM. **No dynamic allocation anywhere** — `malloc` is
never called, so memory use is fixed at link time and cannot fail at runtime.

---

## Repository layout

```
firmware/automotive-body-ecu/
  src/app/         body_control.c, vehicle_state_machine.c
  src/services/    scheduler, fault_manager, logger, can_signals, can_manager
  src/drivers/     gpio, adc, pwm, uart, can, watchdog
  src/config/      ecu_config.h, pin_config.h   ← every tunable value
  Core/, Drivers/  CubeMX-generated startup and ST HAL

tests/             67 unit tests + a Unity-compatible framework
tools/can_tester/  Python system tests, ECU simulator, HTML reporting
docs/              requirements, architecture, CAN spec, test plan, traceability
.github/workflows/ CI: firmware build, unit tests, framework self-check, cppcheck
```

---

## Documentation

| Document | Contents |
|---|---|
| [System requirements](docs/requirements/system_requirements.md) | 36 testable requirements, plus explicit scope exclusions and known limitations |
| [Software architecture](docs/architecture/software_architecture.md) | Layering, execution model, why cooperative rather than an RTOS |
| [CAN specification](docs/can_specification.md) | Bit timing, message layouts, E2E protection, DBC equivalent |
| [Test plan](docs/test_plan.md) | Verification strategy, what is deliberately not unit tested, and why |
| [Traceability matrix](docs/traceability_matrix.md) | Requirement → implementation → test, with honest coverage gaps |
| [Hardware setup](docs/hardware_setup.md) | Bill of materials, wiring diagrams, bring-up checklist, troubleshooting |
| [Lessons learned](docs/lessons_learned.md) | Design decisions, mistakes made, and what would be done differently |

---

## Concepts demonstrated

Embedded C · bare-metal firmware · layered architecture · hardware abstraction ·
GPIO and debouncing · ADC and signal conditioning · PWM · hardware timers ·
UART · CAN 2.0A · finite state machines · cooperative scheduling · worst-case
execution time · watchdog and fault recovery · diagnostic fault maturation ·
CRC and end-to-end protection · unit testing · system testing · restbus
simulation · requirements traceability · continuous integration · static analysis

---

## Licence

MIT — see [LICENSE](LICENSE).

**Mohamed Amine Jazoul** — Electrical Engineering, ENSAM Rabat
