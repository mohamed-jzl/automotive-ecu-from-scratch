# Automotive Body Control ECU — STM32

Educational simulation of an automotive Body Control Module (BCM)
built on an STM32 Nucleo-F446RE.

Developed as a structured learning project covering embedded C,
STM32 peripherals, automotive CAN communication, fault management,
unit testing, and Python-based test automation.

---

## Hardware

- STM32 Nucleo-F446RE
- MCP2515 CAN module x2
- LEDs, push buttons, potentiometer (sensor simulation)

## Software Stack

- STM32CubeIDE (compiler, debugger, flash tool)
- Embedded C (bare-metal, no RTOS)
- Unity (C unit testing framework)
- Python 3 + python-can (test automation)
- GitHub Actions (CI)

---

## Repository Structure

| Folder | Purpose |
|--------|---------|
| src/app/ | Application layer: body control logic, state machine |
| src/services/ | Service layer: fault manager, logger, CAN manager |
| src/drivers/ | Hardware drivers: GPIO, ADC, PWM, CAN, UART |
| src/config/ | Project-wide configuration and pin assignments |
| tests/unit/ | Unit tests (Unity framework) |
| tests/integration/ | Integration tests |
| tools/can_tester/ | Python CAN test automation |
| docs/ | Requirements, architecture, CAN specification |

---

## Development Phases

| Phase | Topic | Status |
|-------|-------|--------|
| 0 | Project foundation | Done |
| 1 | Development environment | In progress |
| 2 | First embedded C program | Pending |
| 3 | GPIO: ignition and lights | Pending |
| 4 | Software architecture | Pending |
| 5 | State machine | Pending |
| 6 | Timers | Pending |
| 7 | Interrupts | Pending |
| 8 | ADC | Pending |
| 9 | PWM | Pending |
| 10 | Watchdog | Pending |
| 11 | UART logging | Pending |
| 12 | Fault management | Pending |
| 13 | CAN communication | Pending |
| 14 | Multi-ECU network | Pending |
| 15 | Embedded unit testing | Pending |
| 16 | Python test automation | Pending |

---

## Author

Mohamed Amine Jazoul
Electrical Engineering — ENSAM Rabat
