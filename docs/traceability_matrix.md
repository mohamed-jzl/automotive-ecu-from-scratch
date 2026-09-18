# Requirements Traceability Matrix

Links every requirement to the code that implements it and the test that
verifies it.

## Why this table exists

In automotive development (ASPICE, the V-model, ISO 26262) traceability is a
process requirement, not documentation for its own sake. It answers three
questions that are otherwise unanswerable:

1. **Is every requirement implemented?** A requirement with no code is a
   feature that was specified and forgotten.
2. **Is every requirement verified?** A requirement with no passing test is an
   unverified claim, however confident anyone is about it.
3. **Is every piece of code justified?** Code that traces to no requirement is
   either an undocumented feature or dead weight — and both are defects.

It also answers the question that comes up during a change: *if I modify this
function, which requirements am I at risk of breaking, and which tests must I
re-run?* Without the table, the honest answer is "all of them".

## Legend

| Level | Meaning |
|---|---|
| **U** | Unit test — host PC, automated |
| **S** | System test — over CAN against the real ECU, automated |
| **I** | Integration test — manual, on the bench |

---

## Matrix

### Vehicle state management

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-001 | State is always one of five values | `vehicle_state_machine.c` | `test_no_event_can_produce_an_invalid_state`, TC-006 | U, S |
| REQ-002 | OFF→ACC→ON on short press; ON→RUN on long press | `vehicle_state_machine.c:VSM_TRANSITION_TABLE` | `test_full_startup_sequence`, IT-03, IT-04 | U, I |
| REQ-003 | Short press from ON/RUN returns to OFF | `vehicle_state_machine.c:VSM_TRANSITION_TABLE` | `test_shutdown_from_run_is_single_action`, `test_shutdown_from_on_is_single_action`, IT-05 | U, I |
| REQ-004 | FAULT reachable from anywhere; driver input ignored | `vehicle_state_machine.c:HandleEvent` | `test_fault_is_reachable_from_every_state`, `test_driver_input_cannot_override_a_fault` | U |
| REQ-005 | FAULT→OFF on recovery, never FAULT→RUN | `vehicle_state_machine.c:VSM_TRANSITION_TABLE` | `test_fault_recovery_returns_to_off_not_to_run` | U |

### Digital inputs

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-006 | Inputs sampled every 10 ms | `body_control.c:task_10ms` | IT-03 | I |
| REQ-007 | New level accepted after 3 stable samples | `gpio_driver.c:SampleInputs` | IT-03 | I |
| REQ-008 | Debouncing must not block | `gpio_driver.c` (counter-based, no delay) | Code review; TC-010 (no overruns) | S |

### Lighting outputs

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-060 | Indicators flash at 1.5 Hz ±10% | `body_control.c:indicator_phase` | TC-012, IT-06 | S, I |
| REQ-061 | Both indicators flash in phase | `body_control.c:task_50ms` (shared phase) | IT-07 | I |
| REQ-062 | Brake light 20% running, 100% braking | `body_control.c:task_50ms`, `pwm_driver.c` | IT-08 | I |
| REQ-063 | All lamps off when lighting not permitted | `body_control.c:task_50ms` (early return) | `test_fault_grants_nothing`, IT-09 | U, I |

### Battery monitoring

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-020 | Measure every 500 ms, report in mV | `body_control.c:task_500ms`, `adc_driver.c` | TC-007, IT-10 | S, I |
| REQ-021 | Detect undervoltage below 11 000 mV | `body_control.c:is_undervoltage` | `test_fault_thresholds_are_inside_the_measurable_range`, IT-11 | U, I |
| REQ-022 | Detect overvoltage above 15 500 mV | `body_control.c:is_overvoltage` | `test_fault_thresholds_are_inside_the_measurable_range` | U |
| REQ-023 | 300 mV hysteresis on voltage faults | `body_control.c:is_undervoltage/is_overvoltage` | `test_hysteresis_does_not_overlap_the_thresholds`, IT-12 | U, I |
| REQ-024 | Average 8 conversions per reading | `adc_driver.c:ReadBatteryMv` | Code review | — |

### Fault management

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-030 | No faults under nominal conditions | `fault_manager.c` | `test_nothing_is_active_after_init`, TC-008 | U, S |
| REQ-031 | Fault count equals bits set in bitmask | `fault_manager.c:GetActiveCount/GetActiveBitmask` | `test_every_fault_id_maps_to_a_distinct_bit`, TC-009 | U, S |
| REQ-032 | Latch after 3 consecutive detections | `fault_manager.c:Update` | `test_single_detection_does_not_latch`, `test_fault_latches_on_the_configured_count`, IT-11 | U, I |
| REQ-033 | Clear after 5 consecutive clean evaluations | `fault_manager.c:Update` | `test_fault_clears_only_after_full_healing`, IT-12 | U, I |
| REQ-034 | Any contrary evaluation resets the counter | `fault_manager.c:Update` | `test_interrupted_maturation_restarts_from_zero`, `test_interrupted_healing_restarts_from_zero`, `test_chattering_condition_stays_latched` | U |

### CAN communication

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-010 | Transmit BCM_VehicleStatus | `can_manager.c:SendVehicleStatus` | `test_vehicle_status_byte_layout_running`, TC-001 | U, S |
| REQ-011 | 100 ms ±20% period | `body_control.c:task_100ms`, `scheduler.c` | TC-002 | S |
| REQ-012 | CRC-8 over bytes 0–6 in byte 7 | `can_signals.c:Crc8` | `test_crc_matches_the_published_check_value`, `test_crc_detects_every_single_bit_error`, TC-004 | U, S |
| REQ-013 | Alive counter increments and wraps | `can_manager.c` (owns the counters) | `test_alive_counter_is_carried_through_unchanged`, TC-005 | U, S |
| REQ-014 | Transmit BCM_Diagnostics every 500 ms | `body_control.c:task_500ms` | `test_diagnostics_round_trip`, TC-003 | U, S |
| REQ-015 | Reject received frames failing CRC | `can_signals.c:UnpackEngineStatus` | `test_corrupted_payload_is_rejected`, `test_rejected_payload_does_not_modify_the_output` | U |
| REQ-016 | Engine data older than 300 ms is invalid | `can_manager.c:IsEngineDataStale` | Code review | — |
| REQ-040 | Operate normally under bus load | `can_manager.c:ProcessReceived` | TC-011 | S |
| REQ-041 | 500 kbit/s, sample point 75–90% | `ecu_config.h:ECU_CAN_*`, `can_driver.c` | TC-001 (frames decode at all) | S |

### Timing and scheduling

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-050 | No task exceeds 5 ms | `scheduler.c:Run` | TC-010 | S |
| REQ-051 | Count and report overruns | `scheduler.c:GetOverrunCount` | TC-010 | S |
| REQ-052 | Measure and report worst-case execution time | `scheduler.c:GetMaxDurationMs` | TC-003 (field is populated) | S |

### Reliability

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-070 | Independent watchdog, 500 ms nominal | `watchdog_driver.c` | IT-13 | I |
| REQ-071 | Refresh only from the main loop | `main.c` while loop | Code review; IT-13 | I |
| REQ-072 | Detect and report a watchdog reset | `watchdog_driver.c:WasResetByWatchdog`, `logger.c:PrintBanner` | IT-13 | I |
| REQ-073 | Outputs inactive before being enabled | `gpio_driver.c:Init` (writes before configuring) | Code review | — |

### Diagnostics

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-080 | UART log at 115200 8N1 | `uart_driver.c`, `logger.c` | IT-01 | I |
| REQ-081 | Timestamp and severity on every line | `logger.c:Print` | IT-01 | I |
| REQ-082 | Log transitions, not steady state | `body_control.c:update_fault`, `fault_manager.c:Update` return value | `test_update_reports_the_latching_transition`, `test_repeated_fault_set_does_not_report_a_change` | U |
| REQ-083 | Status LED encodes the operating mode | `body_control.c:update_status_led` | IT-02 | I |

---

## Coverage summary

| | Count |
|---|---|
| Requirements defined | 36 |
| Covered by automated tests (U or S) | 27 (75%) |
| Covered by manual bench tests only (I) | 6 (17%) |
| Covered by code review only | 3 (8%) |

### The three requirements with no test

Named explicitly rather than buried, because an unverified requirement that
nobody has noticed is far more dangerous than one that is documented as such:

| Req | Why it is not automated | What would be needed |
|---|---|---|
| REQ-024 (average 8 conversions) | The averaging is internal to the driver and not observable from outside | Refactor the averaging into a pure function, as was done for the voltage conversion |
| REQ-016 (300 ms RX staleness) | Requires suppressing a periodic sender mid-test | A test that stops the restbus simulation and asserts the ECU stops using engine data |
| REQ-073 (outputs safe before enable) | The window is microseconds wide at startup | An oscilloscope on the output pins during reset |

All three are genuinely testable with more effort. They are listed as an
honest coverage gap, which is what a real project's matrix looks like — a
matrix showing 100% coverage usually means the requirements were written to
match the tests rather than the other way round.

## Maintaining this matrix

When you add a requirement, add a row. When you add a test, annotate it with
its requirement ID — the `TestCase` entries in `tools/can_tester/test_cases.py`
carry a `requirement` field for exactly this purpose, and the generated HTML
report counts *requirements verified*, not tests executed.

A matrix that is not updated alongside the code becomes actively misleading:
it claims coverage that no longer exists.
