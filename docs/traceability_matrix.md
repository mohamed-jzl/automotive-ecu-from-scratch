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
| **SIL** | Diagnostic system test — production C stack on a PC, driven by udsoncan (`run_diag_tests.py --sil`); the same tests run on hardware |

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

### UDS diagnostics (v0.2)

Unit test suites: `test_isotp` (ISO-TP), `test_uds_server` (UDS),
`test_uds_security`, `test_dtc_manager`, `test_nvm_store`.
System tests: `DG-0xx` in `tools/can_tester/diag_test_cases.py`.

| Req | Requirement | Implementation | Verified by | Level |
|---|---|---|---|---|
| REQ-100 | Physical `7E0`, functional `7DF`, response `7E8` | `ecu_config.h:ECU_DIAG_CAN_ID_*`, `isotp.c:IsoTp_OnFrame` | `test_foreign_identifier_is_not_consumed`, every DG test | U, SIL |
| REQ-101 | NRC `11` / `12` / `13` | `uds_server.c:UdsServer_ProcessRequest` + handlers | `test_unknown_service_is_rejected`, `test_programming_session_is_not_supported`, DG-009 | U, SIL |
| REQ-102 | ISO 14229 NRC order | `uds_server.c:UdsServer_ProcessRequest` | `test_service_in_wrong_session_is_rejected`, `test_write_unknown_did_is_out_of_range` | U |
| REQ-103 | Functional suppression, suppress bit | `uds_server.c:negative_response` | `test_functional_not_supported_errors_are_silent`, `test_suppress_bit_never_hides_an_error`, DG-011 | U, SIL |
| REQ-104 | Extended-only services refused with `7F` | `uds_server.c:SERVICE_TABLE` | `test_write_not_allowed_in_default_session`, DG-010 | U, SIL |
| REQ-110 | Default session after reset | `uds_server.c:UdsServer_Init` | `test_starts_in_default_session_locked`, DG-001, DG-016 | U, SIL |
| REQ-111 | P2 = 50 ms, P2\* = 5000 ms reported | `uds_server.c:handle_session_control` | `test_extended_session_response_carries_timing`, DG-002 | U, SIL |
| REQ-112 | S3 fallback and re-lock | `uds_server.c:UdsServer_Poll` | `test_s3_timeout_returns_to_default_session`, DG-003 | U, SIL |
| REQ-113 | TesterPresent restarts S3 | `uds_server.c:UdsServer_ProcessRequest` | `test_tester_present_keeps_the_session_alive`, DG-004 | U, SIL |
| REQ-120 | Identification DIDs | `diag_app.c:DID_TABLE` | DG-005 | SIL |
| REQ-121 | Live data DIDs | `diag_app.c:DID_TABLE` | DG-006 | SIL |
| REQ-122 | No supported DID → `31` | `uds_server.c:handle_read_did` | `test_read_only_unsupported_dids_is_out_of_range`, DG-008 | U, SIL |
| REQ-123 | VIN write needs extended + unlock | `uds_server.c:handle_write_did` | `test_write_requires_security`, DG-012, DG-014 | U, SIL |
| REQ-124 | VIN validated (ISO 3779) | `diag_app.c:write_vin` | DG-015 | SIL |
| REQ-130 | Correct key unlocks | `uds_server.c:handle_security_access`, `uds_security.c` | `test_correct_key_unlocks`, `test_known_key_vectors`, DG-013 | U, SIL |
| REQ-131 | Protected services → `33` | `uds_server.c:handle_write_did`, `handle_routine_control` | `test_protected_routine_needs_security`, DG-012, DG-023 | U, SIL |
| REQ-132 | 3 wrong keys → `36`, then `37` for 10 s | `uds_server.c:handle_security_access` | `test_three_wrong_keys_trigger_a_lockout`, DG-024 | U, SIL |
| REQ-133 | Seed non-zero, single use, zero when unlocked | `uds_security.c:UdsSecurity_NextSeed`, `uds_server.c` | `test_generator_never_returns_zero`, `test_wrong_key_is_rejected_and_consumes_the_seed`, `test_seed_is_zero_when_already_unlocked`, DG-013 | U, SIL |
| REQ-140 | Respond before resetting | `diag_manager.c:DiagManager_Poll` | `test_hard_reset_is_acknowledged_then_requested`, DG-016 | U, SIL |
| REQ-141 | VIN and DTCs survive reset | `diag_manager.c:save_if_needed`, `restore_from_nvm` | DG-016, DG-021 | SIL |
| REQ-142 | Power loss during write is safe | `nvm_store.c:write_slot` (magic written last, CRC-32) | `test_torn_write_keeps_the_previous_record`, `test_corrupted_record_is_ignored` | U |
| REQ-143 | No erase after watchdog armed | `nvm_store.c:NvmStore_Write` (never erases), `main.c` init order | `test_full_region_switches_without_erasing`, `test_both_regions_full_reports_full` | U |
| REQ-150 | Status byte per ISO 14229 Annex D | `dtc_manager.c:DtcManager_ReportResult` | `test_failure_sets_the_expected_bits`, `test_pass_after_failure_keeps_the_history`, DG-017, DG-019 | U, SIL |
| REQ-151 | Freeze frame at first confirmation | `dtc_manager.c`, `uds_server.c:handle_read_dtc` | `test_snapshot_is_captured_at_first_failure_only`, `test_snapshot_record`, DG-018 | U, SIL |
| REQ-152 | Clear all or one DTC | `dtc_manager.c:DtcManager_Clear` | `test_clear_all_restores_initial_status`, `test_clear_single_dtc_leaves_the_others`, DG-020 | U, SIL |
| REQ-153 | ControlDTCSetting, restored in default | `uds_server.c:enter_session`, `dtc_manager.c` | `test_dtc_setting_off_freezes_dtcs_until_default_session`, DG-022 | U, SIL |
| REQ-154 | Aging after 40 clean cycles | `dtc_manager.c:DtcManager_StartOperationCycle` | `test_confirmed_dtc_ages_out_after_the_threshold`, `test_failing_again_restarts_aging` | U |
| REQ-155 | Operation cycle starts at OFF → ACC | `body_control.c:task_10ms` | IT-14 | I |
| REQ-160 | ISO-TP segmentation, FC, timeouts | `isotp.c` | 25 tests in `test_isotp`, DG-007, DG-014 | U, SIL |
| REQ-161 | Interrupt-driven 32-frame RX queue | `can_driver.c:HAL_CAN_RxFifo0MsgPendingCallback` | IT-15 | I |
| REQ-170 | Lamp self-test routine | `diag_app.c:lamp_test_*`, `body_control.c:task_50ms` | `test_routine_start_and_results`, DG-023, IT-16 | U, SIL, I |

---

## Coverage summary

| | Count |
|---|---|
| Requirements defined | 67 |
| Covered by automated tests (U, S or SIL) | 56 (84%) |
| Covered by manual bench tests only (I) | 8 (12%) |
| Covered by code review only | 3 (4%) |

The two new bench-only requirements are both about *integration*: REQ-155
depends on the real state machine driving the DTC manager, and REQ-161 on a
real interrupt firing under real bus load. Neither exists in the SIL build,
which deliberately contains only the diagnostic stack. That boundary is
stated here so nobody mistakes the SIL results for more than they prove.

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
