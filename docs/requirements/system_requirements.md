# System Requirements Specification

**Project:** Automotive Body Control ECU
**Target:** STM32 Nucleo-F446RE
**Version:** 0.2.0 (adds section 4.10, UDS diagnostics)
**Status:** Baseline

---

## 1. Purpose

This document defines what the ECU must do. Every requirement is testable, has
a unique identifier, and is linked to the code that implements it and the test
that verifies it in [traceability_matrix.md](../traceability_matrix.md).

## 2. Why requirements come first

In automotive development the V-model pairs every specification activity with a
corresponding verification activity:

```
  System requirements  ─────────────────────▶  System tests
       │                                              ▲
       ▼                                              │
   Architecture  ──────────────────────────▶  Integration tests
       │                                              ▲
       ▼                                              │
   Module design  ─────────────────────────▶  Unit tests
       │                                              ▲
       └──────────────▶  Implementation  ─────────────┘
```

Two rules follow, and both are audited in a real project:

- A requirement with no verifying test is an **unverified claim**.
- A test that verifies no requirement is **work nobody asked for**.

A requirement written as "the battery voltage should be monitored" cannot be
tested, because nothing distinguishes a pass from a fail. Every requirement
below states a threshold, a period, or an observable output.

## 3. Notation

The keyword **shall** marks a binding requirement. Anything expressed as
"should" or "may" is guidance, not a requirement, and is not tested.

---

## 4. Requirements

### 4.1 Vehicle state management

| ID | Requirement |
|----|-------------|
| **REQ-001** | The ECU **shall** maintain a vehicle state that is always exactly one of: OFF, ACC, ON, RUN, FAULT. |
| **REQ-002** | The ECU **shall** advance OFF → ACC → ON on successive short presses of the ignition input, and transition ON → RUN on a press held for at least 1000 ms. |
| **REQ-003** | The ECU **shall** return to OFF on a short ignition press while in ON or RUN. |
| **REQ-004** | The ECU **shall** enter FAULT from any state when at least one fault is latched, and **shall** ignore all driver input while in FAULT. |
| **REQ-005** | The ECU **shall** transition FAULT → OFF, and never FAULT → RUN, when the last active fault clears. |

*Rationale for REQ-004 and REQ-005:* a fail-safe state that a driver could
override by pressing a button is not fail-safe. Recovering directly to RUN
would restart a vehicle function with no deliberate driver action.

### 4.2 Digital inputs

| ID | Requirement |
|----|-------------|
| **REQ-006** | The ECU **shall** sample all digital inputs every 10 ms. |
| **REQ-007** | The ECU **shall** accept a new input level only after it has been stable for 3 consecutive samples (30 ms). |
| **REQ-008** | Input debouncing **shall not** block execution of any other task. |

*Rationale for REQ-008:* a blocking delay inside an input read makes the 10 ms
control deadline unachievable. The debounce must count samples across task
activations, never stall inside one.

### 4.3 Lighting outputs

| ID | Requirement |
|----|-------------|
| **REQ-060** | While a turn indicator is requested and lighting is permitted, the ECU **shall** flash the corresponding lamp at 1.5 Hz ± 10% (within the 60–120 cycles/min band of UNECE R6). |
| **REQ-061** | When both turn indicators are requested simultaneously, the ECU **shall** flash both lamps in phase (hazard function). |
| **REQ-062** | The ECU **shall** drive the brake light at 20% duty as a running light, and at 100% duty while the brake input is active. |
| **REQ-063** | The ECU **shall** de-energise every exterior lamp in any state that does not permit lighting, including FAULT. |

### 4.4 Battery monitoring

| ID | Requirement |
|----|-------------|
| **REQ-020** | The ECU **shall** measure the battery voltage every 500 ms and report it in millivolts. |
| **REQ-021** | The ECU **shall** detect undervoltage below 11 000 mV. |
| **REQ-022** | The ECU **shall** detect overvoltage above 15 500 mV. |
| **REQ-023** | Voltage fault detection **shall** apply 300 mV of hysteresis so that a voltage at a threshold does not set and clear the fault repeatedly. |
| **REQ-024** | Each reported measurement **shall** be the average of 8 conversions. |

### 4.5 Fault management

| ID | Requirement |
|----|-------------|
| **REQ-030** | The ECU **shall** report no active faults when all monitored conditions are within their normal range. |
| **REQ-031** | The reported fault count **shall** equal the number of bits set in the reported fault bitmask. |
| **REQ-032** | A fault **shall** latch only after its condition has been present for 3 consecutive evaluations. |
| **REQ-033** | A latched fault **shall** clear only after its condition has been absent for 5 consecutive evaluations. |
| **REQ-034** | Any single evaluation to the contrary **shall** reset the corresponding counter to zero. |

*Rationale:* REQ-032 prevents a single noisy reading from putting the vehicle
in limp mode. REQ-033 being stricter than REQ-032 means the ECU is quick to
suspect a fault and slow to dismiss one — the safe asymmetry.

### 4.6 CAN communication

| ID | Requirement |
|----|-------------|
| **REQ-010** | The ECU **shall** transmit BCM_VehicleStatus (0x100) containing vehicle state, output states, battery voltage and fault status. |
| **REQ-011** | BCM_VehicleStatus **shall** be transmitted every 100 ms ± 20%. |
| **REQ-012** | Every transmitted frame **shall** carry a CRC-8 (SAE J1850) over bytes 0–6 in byte 7. |
| **REQ-013** | Every transmitted frame **shall** carry an alive counter in byte 6, incremented by exactly one per transmission and wrapping 255 → 0. |
| **REQ-014** | The ECU **shall** transmit BCM_DIAGNOSTICS (0x101) every 500 ms containing uptime, scheduler overrun count and worst-case task duration. |
| **REQ-015** | The ECU **shall** receive PCM_EngineStatus (0x200) and **shall** reject any frame whose CRC does not match. |
| **REQ-016** | The ECU **shall** treat received engine data older than 300 ms as invalid and **shall not** use it for any decision. |
| **REQ-040** | The ECU **shall** continue normal operation while receiving CAN traffic at the bus's nominal load. |
| **REQ-041** | The bus **shall** operate at 500 kbit/s with a sample point between 75% and 90%. |

*Rationale for REQ-016:* absent data and zero data are different things. A
receiver that keeps using the last value from a powertrain ECU that has lost
power believes the engine is still turning at the last-reported speed.

### 4.7 Timing and scheduling

| ID | Requirement |
|----|-------------|
| **REQ-050** | No task **shall** exceed an execution time of 5 ms. |
| **REQ-051** | The ECU **shall** count and report every task activation that exceeds its budget. |
| **REQ-052** | The ECU **shall** measure and report the worst-case execution time observed. |

*Rationale:* REQ-052 is the evidence that the schedule closes. Without a
measured worst case, the timing design is an assumption rather than a result.

### 4.8 Reliability

| ID | Requirement |
|----|-------------|
| **REQ-070** | The ECU **shall** enable an independent watchdog with a nominal timeout of 500 ms. |
| **REQ-071** | The watchdog **shall** be refreshed only from the main loop, after the scheduler has serviced its due tasks. |
| **REQ-072** | The ECU **shall** determine whether its last reset was caused by the watchdog, and **shall** report it at startup. |
| **REQ-073** | The ECU **shall** drive all outputs to their inactive state before enabling them during initialisation. |

*Rationale for REQ-071:* refreshing the watchdog from a timer interrupt keeps
petting it while the main loop is dead, which defeats the mechanism entirely.
This is the most common way watchdogs are rendered useless in practice, so it
is stated as a requirement rather than left to implementation judgement.

### 4.9 Diagnostics

| ID | Requirement |
|----|-------------|
| **REQ-080** | The ECU **shall** emit human-readable log output over UART at 115200 8N1. |
| **REQ-081** | Every log line **shall** carry a millisecond timestamp and a severity level. |
| **REQ-082** | The ECU **shall** log every vehicle state transition and every fault transition, and **shall not** log unchanged state. |
| **REQ-083** | The status LED **shall** indicate the operating mode: 1 Hz flash in OFF, steady in ACC/ON/RUN, 5 Hz flash in FAULT. |

### 4.10 UDS diagnostics (ISO 14229-1 over ISO 15765-2)

Interface details — byte formats, DIDs, DTC codes — are in
[uds_specification.md](../uds_specification.md). The requirements below state
what must hold; the specification states exactly how it looks on the wire.

**Protocol and addressing**

| ID | Requirement |
|----|-------------|
| **REQ-100** | The ECU **shall** accept physical requests on `0x7E0` and functional requests on `0x7DF`, and **shall** respond on `0x7E8`. |
| **REQ-101** | The ECU **shall** answer an unsupported service with NRC `0x11`, an unsupported sub-function with `0x12`, and a request of wrong length with `0x13`. |
| **REQ-102** | When a request violates several rules, the ECU **shall** report them in the ISO 14229-1 order: service support, session, length, then service-specific checks. |
| **REQ-103** | For functionally addressed requests the ECU **shall not** send NRCs `0x11`, `0x12`, `0x31`, `0x7E` or `0x7F`; when bit 7 of the sub-function is set it **shall not** send a positive response, but **shall** still send any other negative response. |
| **REQ-104** | Services restricted to the extended session **shall** be refused in the default session with NRC `0x7F`. |

**Sessions**

| ID | Requirement |
|----|-------------|
| **REQ-110** | The ECU **shall** start in the default session after every reset. |
| **REQ-111** | On entering a session the ECU **shall** report P2server = 50 ms and P2\*server = 5000 ms. |
| **REQ-112** | The ECU **shall** return to the default session, and re-lock security, when no request is received for 5000 ms in a non-default session (S3). |
| **REQ-113** | TesterPresent (`0x3E`) **shall** restart the S3 timer. |

**Data identifiers**

| ID | Requirement |
|----|-------------|
| **REQ-120** | The ECU **shall** provide VIN (`F190`), software version (`F195`), serial number (`F18C`) and active session (`F186`). |
| **REQ-121** | The ECU **shall** provide battery voltage, vehicle state, inputs, active faults and uptime as DIDs `0100`–`0104`. |
| **REQ-122** | A request containing no supported DID **shall** be refused with NRC `0x31`. |
| **REQ-123** | The VIN **shall** be writable only in the extended session with security access unlocked. |
| **REQ-124** | The ECU **shall** reject a VIN containing any character outside ISO 3779 (`0-9`, `A-Z` except `I`, `O`, `Q`) with NRC `0x31`. |

**Security access**

| ID | Requirement |
|----|-------------|
| **REQ-130** | The ECU **shall** unlock security level 1 when it receives the key matching the most recent seed. |
| **REQ-131** | WriteDataByIdentifier and the lamp self-test routine **shall** be refused with NRC `0x33` while security is locked. |
| **REQ-132** | After 3 consecutive invalid keys the ECU **shall** answer NRC `0x36` and refuse seed requests with NRC `0x37` for 10 s. |
| **REQ-133** | A seed **shall** never be zero while locked, **shall** be valid for one key attempt only, and **shall** be `00000000` when already unlocked. |

**Reset and persistence**

| ID | Requirement |
|----|-------------|
| **REQ-140** | On ECUReset the ECU **shall** transmit its positive response before resetting. |
| **REQ-141** | The VIN and all DTC information **shall** be retained across resets and power cycles. |
| **REQ-142** | An interruption of power during a non-volatile write **shall not** corrupt or lose the previously stored data. |
| **REQ-143** | The ECU **shall not** erase flash after the watchdog has been armed. |

**DTC management**

| ID | Requirement |
|----|-------------|
| **REQ-150** | Each DTC **shall** carry a status byte whose eight bits behave as defined in ISO 14229-1 Annex D. |
| **REQ-151** | When a DTC is first confirmed, the ECU **shall** store a snapshot of battery voltage and vehicle state. |
| **REQ-152** | ClearDiagnosticInformation **shall** reset all DTCs, or one DTC, to the initial status `0x50`. |
| **REQ-153** | While ControlDTCSetting is off, DTC status **shall not** change; the setting **shall** return to on when the default session is entered. |
| **REQ-154** | A confirmed DTC **shall** be erased after 40 consecutive operation cycles without failure. |
| **REQ-155** | An operation cycle **shall** begin at each transition from OFF to ACC. |

**Transport and reception**

| ID | Requirement |
|----|-------------|
| **REQ-160** | The ECU **shall** send and receive UDS messages of up to 256 bytes using ISO-TP segmentation, flow control and the N_Cr / N_Bs timeouts. |
| **REQ-161** | CAN frames **shall** be received by interrupt into a queue of at least 32 frames, so that no frame of a diagnostic request is lost between task activations. |

**Routines**

| ID | Requirement |
|----|-------------|
| **REQ-170** | Routine `0x0201` **shall** light all exterior lamps for 3 s, **shall** be refused with NRC `0x22` while the engine runs, and **shall** report its status. |

*Rationale for REQ-143:* erasing a 128 KB flash sector stalls the CPU for 1-2
seconds, longer than the watchdog timeout. Any design that erases at runtime
eventually resets the ECU in the middle of a write.

*Rationale for REQ-140:* if the ECU reset before its response left the bus,
the tester would time out and could not tell a successful reset from a lost
request.

---

## 5. Explicitly out of scope

Stating what a system does *not* do is as important as stating what it does.
These are deliberate exclusions, not oversights:

- No programming session or bootloader: UDS services `0x34`/`0x36`/`0x37`
  (download), `0x28`, `0x2F` and `0x29` are not implemented. See
  [uds_specification.md](../uds_specification.md#12-not-implemented).
- No production-grade security: the seed/key algorithm is educational and
  offers no protection against a determined attacker.
- No ISO 26262 functional safety development; no ASIL is claimed.
- No AUTOSAR conformance. The architecture is *inspired by* its layering.
- No bootloader or firmware update capability.
- No low-power or sleep modes; a real BCM must sleep at microamp level to
  avoid draining the battery, which is a significant design area on its own.
- No bus-load management or network-management (NM) protocol.

## 6. Known limitations

Recorded so that a reviewer is not left to discover them:

| Limitation | Consequence |
|---|---|
| UART logging is blocking | A long log line inside a fast task can itself cause the deadline miss it was added to diagnose. A ring buffer with DMA is the correct fix. |
| IWDG timeout tolerance is ±50% | The LSI is an untrimmed RC oscillator. The true timeout lies between roughly 330 ms and 1 s. A safety-critical design would measure LSI against a precise clock and compensate. |
| ~~CAN reception is polled~~ | **Resolved in 0.2.0**: reception is interrupt-driven into a 32-frame queue (REQ-161). |
| No hardware RNG for security seeds | The STM32F446 has none; seeds come from a PRNG seeded by the chip ID and request timing. Not cryptographically secure. |
| DTC warning lamp follows the live fault | Production strategies usually keep the lamp lit for several clean cycles after the fault disappears. |
| Timing is measured with 1 ms resolution | A task taking 0.4 ms and one taking 1.4 ms are indistinguishable. A cycle counter (DWT) would give sub-microsecond resolution. |
| Headlights are driven by vehicle state, not a switch | There are no spare inputs on the bench. A real BCM has a light switch and an ambient sensor. |
