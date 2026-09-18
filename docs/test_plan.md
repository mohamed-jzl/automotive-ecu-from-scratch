# Test Plan

**Project:** Automotive Body Control ECU
**Version:** 0.1.0

---

## 1. Verification vs validation

Two words that are used interchangeably in conversation and mean different
things in engineering:

| | Question | Our answer |
|---|---|---|
| **Verification** | Did we build the product *right*? | Unit and system tests against `system_requirements.md` |
| **Validation** | Did we build the *right* product? | Manual bench review against the project objective |

A system can pass every verification test and still be the wrong system. That
is why the requirements document opens with a rationale for each group, not
just a list of shalls.

## 2. Test levels

```
                                              ┌────────────────────┐
  System requirements  ──────────────────────▶│   SYSTEM TESTS     │
                                              │ tools/can_tester/  │
                                              │ Black-box, over CAN│
                                              └────────────────────┘
                                                        ▲
                                              ┌────────────────────┐
  Architecture  ───────────────────────────▶  │ INTEGRATION TESTS  │
                                              │ Manual, on bench   │
                                              └────────────────────┘
                                                        ▲
                                              ┌────────────────────┐
  Module design  ──────────────────────────▶  │    UNIT TESTS      │
                                              │     tests/         │
                                              │  Host PC, no HW    │
                                              └────────────────────┘
```

Each level catches what the others cannot:

- Unit tests prove a module is correct **in isolation**. They cannot tell you
  the modules are wired together correctly.
- System tests prove the assembled ECU behaves correctly **from outside**.
  They cannot tell you *which* module is wrong when they fail.
- Passing unit tests with failing system tests means the integration is wrong.
  Passing system tests with failing unit tests means you were lucky.

---

## 3. Unit tests

**Location:** `tests/`
**Runs on:** host PC, no hardware
**Framework:** `tests/framework/unity_min.h` (Unity-compatible API)
**Command:** `cd tests && make`

### 3.1 Scope

Only modules that are **pure** — no HAL, no register access, no time source:

| Suite | Module under test | Tests |
|---|---|---|
| `test_vehicle_state_machine` | `vehicle_state_machine.c` | 25 |
| `test_fault_manager` | `fault_manager.c` | 16 |
| `test_can_signals` | `can_signals.c` | 18 |
| `test_adc_conversion` | `AdcDriver_RawToBatteryMv` | 8 |

The tests link the **real production source**, never a copy. A test that
passes against a duplicated implementation proves nothing about the shipped
code.

### 3.2 Explicitly out of scope

`gpio_driver.c`, `adc_driver.c`, `pwm_driver.c`, `uart_driver.c`,
`can_driver.c` and `watchdog_driver.c` are **not** unit tested.

This is a deliberate decision, not an omission. These modules exist to write to
hardware registers. Testing them without hardware means mocking the entire
STM32 HAL — a large amount of work whose main result is proving that the mocks
agree with the mocks. They are verified on the bench instead (section 4), and
the requirements they implement are covered by system tests.

Knowing what *not* to unit test is as much a part of test design as knowing
what to.

### 3.3 Notable test techniques

**Exhaustive sweeps.** `test_no_event_can_produce_an_invalid_state` crosses
all 5 states with all 5 events — the module's complete input space.
`test_conversion_is_monotonic` sweeps all 4096 possible ADC readings. Both are
affordable only because the modules are pure; on hardware they would be weeks
of test cases.

**Independent expected values.** The CAN byte-layout tests assert against
hard-coded arrays computed in Python from the written specification, not
against the encoder's own output. A bug in the C encoder cannot hide by also
appearing in the expectation.

**Standard check vectors.** The CRC is verified against the published check
value for CRC-8/SAE-J1850 (`CRC("123456789") == 0x4B`). This pins the
implementation to the standard rather than to our interpretation of it.

**Negative testing.** Out-of-range enums, NULL pointers, corrupted payloads,
illegal state transitions. Roughly a third of the tests check that wrong input
is *rejected*, not that right input is accepted.

**Timing without a clock.** The fault manager counts calls rather than
milliseconds, so maturation and healing timing is tested exactly. Provoking
the same coverage on hardware would mean holding a battery voltage at a
marginal level for a precise number of cycles.

---

## 4. Integration tests (manual, on bench)

Executed by hand against the checklist in
[hardware_setup.md](hardware_setup.md#6-bring-up-checklist). These cover what
the automated tests cannot reach: that the pins are wired as the pin map says,
that the peripherals were configured correctly, and that the lamps physically
light.

| ID | Check | Requirement | Method |
|---|---|---|---|
| IT-01 | Serial banner appears at reset | REQ-080, REQ-081 | Terminal at 115200 8N1 |
| IT-02 | Status LED flashes 1 Hz in OFF | REQ-083 | Visual |
| IT-03 | Ignition advances OFF→ACC→ON | REQ-002 | Press B1, read the log |
| IT-04 | Long press enters RUN | REQ-002 | Hold B1 >1 s from ON |
| IT-05 | Short press from RUN returns to OFF | REQ-003 | Press B1 |
| IT-06 | Indicators flash at 1.5 Hz | REQ-060 | Stopwatch over 10 cycles |
| IT-07 | Both indicators flash in phase | REQ-061 | Visual, both buttons held |
| IT-08 | Brake light dim → bright | REQ-062 | Visual, press brake button |
| IT-09 | All lamps dark in FAULT | REQ-063 | Provoke undervoltage |
| IT-10 | Battery reading tracks the pot | REQ-020 | Sweep, watch DEBUG log |
| IT-11 | Undervoltage matures, not instant | REQ-032 | Sweep down, time the latch (~1.5 s) |
| IT-12 | Fault clears more slowly than it set | REQ-033 | Sweep back up, time the clear (~2.5 s) |
| IT-13 | Watchdog resets a hung ECU | REQ-070, REQ-072 | Temporarily add `while(1);` to a task; the banner must report a watchdog reset |

> IT-13 is the only test that deliberately breaks the firmware. It is the only
> way to prove the watchdog works: a watchdog that has never fired is an
> untested safety mechanism. Revert the change immediately afterwards.

---

## 5. System tests (automated, over CAN)

**Location:** `tools/can_tester/`
**Runs against:** the assembled ECU, black-box, over the CAN bus
**Command:** `python run_tests.py --interface slcan --channel COM5`

### 5.1 Test cases

| ID | Test | Requirement |
|---|---|---|
| TC-001 | Vehicle status is transmitted | REQ-010 |
| TC-002 | Vehicle status period is 100 ms ±20% | REQ-011 |
| TC-003 | Diagnostics message is transmitted | REQ-014 |
| TC-004 | Every frame passes its CRC | REQ-012 |
| TC-005 | Alive counter increments and wraps | REQ-013 |
| TC-006 | Vehicle state is within range | REQ-001 |
| TC-007 | Battery voltage is plausible | REQ-020 |
| TC-008 | No faults under nominal conditions | REQ-030 |
| TC-009 | Fault count agrees with the bitmask | REQ-031 |
| TC-010 | No scheduler deadline misses | REQ-050 |
| TC-011 | ECU tolerates restbus traffic | REQ-040 |
| TC-012 | Indicator signals report the lamp | REQ-060 |

### 5.2 Restbus simulation

TC-011 sends `PCM_EngineStatus` frames while the ECU is running, standing in
for a powertrain ECU that is not on the bench. This is called **restbus
simulation** — simulating the "rest of the bus" so a single ECU can be tested
against the network traffic it will meet in a vehicle.

A node that works alone on a quiet bench and falls over on a busy bus is a
classic and expensive integration failure, usually discovered very late.

### 5.3 Pass, fail, skip — and why the distinction matters

| Result | Meaning |
|---|---|
| **PASS** | The requirement was demonstrated |
| **FAIL** | The ECU behaved incorrectly |
| **SKIP** | The test could not run and **proved nothing** |
| **ERROR** | The test framework or bus failed, not the ECU |

Reporting a skip as a pass would be the worst possible outcome: a green report
that verifies nothing. The runner therefore reports **INCONCLUSIVE** rather
than **PASSED** when every test skipped, and the summary counts *requirements
verified* rather than tests executed.

### 5.4 Proving the framework itself works

A test that has never passed is an untested assertion. The framework is
therefore validated against a software model of the ECU:

```
python ecu_simulator.py --self-test
```

The same suite must report **INCONCLUSIVE** against an empty bus and
**12/12 PASSED** against the known-good model. A framework that cannot tell
those two apart is worthless, so this is run in CI on every commit.

The simulator shares no code with the firmware — it reimplements the
observable behaviour from the specification. That independence is what makes
it useful: a test passing against the model and failing on the board means the
firmware is wrong, which is exactly the comparison you want.

### 5.5 Measurement limitations

Timestamps come from the PC, not the CAN controller, so they include USB
latency and host scheduler jitter — typically a few milliseconds. That is why
TC-002 allows ±20% on a 100 ms period.

**The tolerance reflects the measurement equipment, not the ECU.** This setup
cannot verify a requirement tighter than a few milliseconds; that needs a
hardware-timestamping interface or a CAN analyser. Knowing the limits of your
instruments is part of being a test engineer, so the measured jitter is
reported alongside every timing result rather than hidden.

---

## 6. Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request:

| Job | What it proves |
|---|---|
| Firmware build | The ECU image compiles and links for ARM, with size reported |
| Unit tests | All 67 tests pass, compiled with `-Wall -Wextra -Werror` |
| Framework self-test | The Python suite passes against the simulator |
| Static analysis | `cppcheck` finds no defects in application code |

`-Werror` means a compiler warning fails the build. In embedded C a warning is
usually a real defect — an implicit conversion that truncates, a comparison
that is always true, an uninitialised variable — and a warning that nobody is
forced to look at is a warning that nobody looks at.

## 7. Entry and exit criteria

**Entry (before testing starts):** the firmware builds with no warnings, the
bench is wired per `hardware_setup.md`, and the requirements baseline is
agreed.

**Exit (testing is complete):**

- All unit tests pass.
- All bench integration checks pass.
- All system tests pass, or any failure is documented with an accepted
  justification.
- Every requirement in `system_requirements.md` traces to at least one passing
  test in `traceability_matrix.md`.

The last criterion is the one that matters. A requirement with no passing test
is an unverified claim, however confident anyone is about it.
