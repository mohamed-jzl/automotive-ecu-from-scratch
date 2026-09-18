# CAN Interface Specification

**Bus:** Body CAN
**Bit rate:** 500 000 bit/s
**Frame format:** CAN 2.0A, 11-bit standard identifiers
**Byte order:** Little-endian (Intel) for all multi-byte signals

---

## 1. Bit timing

| Parameter | Value |
|---|---|
| CAN clock source | APB1 @ 42 MHz |
| Prescaler | 6 |
| Synchronisation jump width | 1 tq |
| Phase segment 1 (BS1) | 11 tq |
| Phase segment 2 (BS2) | 2 tq |
| Total bit time | 1 + 11 + 2 = **14 tq** |
| Resulting bit rate | 42 MHz ÷ (6 × 14) = **500 000 bit/s** |
| Sample point | (1 + 11) ÷ 14 = **85.7%** |

The sample point is where each node reads the bus level within a bit. CiA 301
recommends 75–90%: late enough that the signal has settled across a long
cable, early enough to leave margin for resynchronisation. Every node on the
bus must agree on both the bit rate and, approximately, the sample point — a
mismatch produces intermittent errors that look like noise and are
notoriously hard to diagnose.

## 2. Physical layer

```
   Body ECU              Cluster ECU            Powertrain ECU
  ┌─────────┐            ┌─────────┐            ┌─────────┐
  │ STM32   │            │ STM32   │            │  PC +   │
  │  CAN1   │            │  CAN1   │            │ adapter │
  └────┬────┘            └────┬────┘            └────┬────┘
       │ TX/RX (3.3 V logic)  │                      │
  ┌────▼────┐            ┌────▼────┐            ┌────▼────┐
  │transceiv│            │transceiv│            │transceiv│
  └──┬───┬──┘            └──┬───┬──┘            └──┬───┬──┘
     │   │                  │   │                  │   │
 ────┴───┼──────────────────┴───┼──────────────────┴───┼──── CAN_H
  ┌──┐   │                      │                      │  ┌──┐
  │120     │                    │                      │  │120
  │Ω │   │                      │                      │  │Ω │
  └──┘   │                      │                      │  └──┘
 ────────┴──────────────────────┴──────────────────────┴──── CAN_L
```

**Termination:** exactly two 120 Ω resistors, one at each physical end of the
bus — never one per node. The value matches the twisted pair's characteristic
impedance so signal reflections do not corrupt the waveform. A bus with one
resistor, or with four, will appear to work at low bit rates and fail
intermittently at 500 kbit/s. Measuring 60 Ω across an unpowered bus (the two
120 Ω in parallel) is the quickest way to confirm the termination is right.

The STM32 provides logic-level TX and RX only. An external transceiver
(MCP2551, TJA1050, SN65HVD230) is required to drive the differential pair.

## 3. Message overview

| ID | Name | Direction | Period | DLC | Sender |
|---|---|---|---|---|---|
| 0x100 | BCM_VehicleStatus | TX | 100 ms | 8 | Body ECU |
| 0x101 | BCM_Diagnostics | TX | 500 ms | 8 | Body ECU |
| 0x200 | PCM_EngineStatus | RX | 100 ms | 8 | Powertrain ECU |

A lower identifier wins CAN arbitration, so the numbering encodes priority. In
a production vehicle the safety-relevant powertrain messages would be given
the lower identifiers; here the body messages use 0x1xx purely so this ECU's
traffic is easy to pick out of a bus trace.

## 4. End-to-end protection

Every message reserves the last two bytes:

| Byte | Field | Purpose |
|---|---|---|
| 6 | Alive counter | Increments by 1 per transmission, wraps 255 → 0 |
| 7 | CRC-8 | Computed over bytes 0–6 |

**CRC-8 parameters (SAE J1850, as used by AUTOSAR E2E Profile 1):**

| | |
|---|---|
| Polynomial | 0x1D |
| Initial value | 0xFF |
| Final XOR | 0xFF |
| Reflection | none |
| Check value | `CRC("123456789")` = **0x4B** |

This polynomial has a Hamming distance of 4 over an 8-byte payload: it detects
every 1-, 2- and 3-bit error.

**Why this is needed when CAN already has a CRC.** The CAN controller's own
15-bit CRC protects the frame between two transceivers. It does not protect
against a sender that has crashed and is transmitting a stale buffer, a
gateway that duplicates or reorders a frame, or corruption introduced inside
a routing node *after* the frame was checked. The alive counter answers "is
this fresh?"; the payload CRC answers "did anything change it since the
sender built it?" CAN's CRC answers neither.

---

## 5. Message definitions

### 5.1 BCM_VehicleStatus — 0x100

Transmitted by the Body ECU every 100 ms. The primary broadcast: what mode the
vehicle is in and what the body outputs are doing.

| Byte | Bits | Signal | Type | Range | Unit |
|---|---|---|---|---|---|
| 0 | 0–2 | `VehicleState` | uint3 | 0–4 | enum |
| 0 | 3 | `IgnitionOn` | bool | 0–1 | — |
| 0 | 4 | `BrakeActive` | bool | 0–1 | — |
| 0 | 5 | `IndicatorLeft` | bool | 0–1 | — |
| 0 | 6 | `IndicatorRight` | bool | 0–1 | — |
| 0 | 7 | `HeadlightOn` | bool | 0–1 | — |
| 1–2 | — | `BatteryVoltage` | uint16 LE | 0–18300 | mV |
| 3 | 0–3 | `FaultCount` | uint4 | 0–15 | — |
| 3 | 4–7 | *reserved* | — | 0 | — |
| 4–5 | — | `FaultBitmask` | uint16 LE | — | bitfield |
| 6 | — | `AliveCounter` | uint8 | 0–255 | — |
| 7 | — | `CRC8` | uint8 | — | — |

`VehicleState` enumeration:

| Value | Name | Meaning |
|---|---|---|
| 0 | OFF | Dormant |
| 1 | ACC | Accessories powered |
| 2 | ON | Full electrical system powered |
| 3 | RUN | Engine running |
| 4 | FAULT | Fail-safe, degraded |

`FaultBitmask` bit assignments:

| Bit | Fault | Meaning |
|---|---|---|
| 0 | `BATT_UNDERVOLT` | Supply below 11 000 mV |
| 1 | `BATT_OVERVOLT` | Supply above 15 500 mV |
| 2 | `ADC_FAILURE` | Conversion timed out |
| 3 | `CAN_BUS_OFF` | Controller disconnected itself |
| 4 | `CAN_TX_FAIL` | Frames rejected by the mailbox |
| 5 | `TASK_OVERRUN` | A task exceeded its budget |

> Bit positions are part of the network interface. **Append new faults; never
> renumber existing ones.** A receiving ECU decodes by bit position, so
> renumbering silently changes the meaning of a fault for every other node.

**Worked example.** Engine running, ignition and brake active, headlights on,
battery 12.600 V, no faults, alive counter 7:

```
byte 0 = 0x03 (RUN) | 0x08 (ignition) | 0x10 (brake) | 0x80 (headlight) = 0x9B
byte 1 = 12600 & 0xFF        = 0x38
byte 2 = (12600 >> 8) & 0xFF = 0x31
byte 6 = 0x07
byte 7 = CRC8(9B 38 31 00 00 00 07) = 0x07

  9B 38 31 00 00 00 07 07
```

This exact vector is asserted in `tests/test_can_signals.c` and reproduced
independently by `tools/can_tester/ecu_signals.py`.

### 5.2 BCM_Diagnostics — 0x101

Transmitted every 500 ms. Health telemetry about the ECU itself.

| Byte | Signal | Type | Range | Unit |
|---|---|---|---|---|
| 0–1 | `UptimeSeconds` | uint16 LE | 0–65535 | s |
| 2–3 | `TaskOverrunCount` | uint16 LE | 0–65535 | — |
| 4 | `MaxTaskDurationMs` | uint8 | 0–255 | ms |
| 5 | `CanTxFailures` | uint8 | 0–255 | — |
| 6 | `AliveCounter` | uint8 | 0–255 | — |
| 7 | `CRC8` | uint8 | — | — |

All counters **saturate** at their maximum rather than wrapping. A counter
that has exceeded its range should report "at least 255", never fold back to a
small and reassuring number.

### 5.3 PCM_EngineStatus — 0x200

Received from the powertrain ECU every 100 ms. Treated as invalid if not
refreshed within 300 ms (three times its period).

| Byte | Signal | Type | Range | Resolution | Offset |
|---|---|---|---|---|---|
| 0–1 | `EngineRpm` | uint16 LE | 0–8000 | 1 rpm | 0 |
| 2–3 | `VehicleSpeed` | uint16 LE | 0–2500 | 0.1 km/h | 0 |
| 4 | `CoolantTemp` | uint8 | 0–255 | 1 °C | **−40** |
| 5 bit 0 | `EngineRunning` | bool | 0–1 | — | — |
| 6 | `AliveCounter` | uint8 | 0–255 | — | — |
| 7 | `CRC8` | uint8 | — | — | — |

**The coolant temperature offset.** CAN signals are unsigned by default.
Adding 40 shifts the automotive range of −40…+215 °C into 0…255, so it fits
one byte with no sign handling:

```
physical = raw − 40        raw = physical + 40
  −40 °C  ⇄  raw 0
    0 °C  ⇄  raw 40
   90 °C  ⇄  raw 130
```

This "physical = (raw × factor) + offset" convention is used throughout real
DBC files, and getting the sign wrong is a classic defect: a cold-start
reading of −30 °C decoding as +226 °C is invisible in a warm workshop and
obvious on a winter morning. `tests/test_can_signals.c` covers it explicitly.

---

## 6. DBC equivalent

A real project keeps this specification in a DBC database and generates both
encoder and decoder from it. The equivalent definition:

```
BO_ 256 BCM_VehicleStatus: 8 BCM
 SG_ VehicleState   :  0|3@1+ (1,0)     [0|4]     ""  CLUSTER
 SG_ IgnitionOn     :  3|1@1+ (1,0)     [0|1]     ""  CLUSTER
 SG_ BrakeActive    :  4|1@1+ (1,0)     [0|1]     ""  CLUSTER
 SG_ IndicatorLeft  :  5|1@1+ (1,0)     [0|1]     ""  CLUSTER
 SG_ IndicatorRight :  6|1@1+ (1,0)     [0|1]     ""  CLUSTER
 SG_ HeadlightOn    :  7|1@1+ (1,0)     [0|1]     ""  CLUSTER
 SG_ BatteryVoltage :  8|16@1+ (0.001,0) [0|18.3] "V" CLUSTER
 SG_ FaultCount     : 24|4@1+ (1,0)     [0|15]    ""  CLUSTER
 SG_ FaultBitmask   : 32|16@1+ (1,0)    [0|65535] ""  CLUSTER
 SG_ AliveCounter   : 48|8@1+ (1,0)     [0|255]   ""  CLUSTER
 SG_ CRC8           : 56|8@1+ (1,0)     [0|255]   ""  CLUSTER

BO_ 512 PCM_EngineStatus: 8 PCM
 SG_ EngineRpm      :  0|16@1+ (1,0)    [0|8000]  "rpm"  BCM
 SG_ VehicleSpeed   : 16|16@1+ (0.1,0)  [0|250]   "km/h" BCM
 SG_ CoolantTemp    : 32|8@1+ (1,-40)   [-40|215] "degC" BCM
 SG_ EngineRunning  : 40|1@1+ (1,0)     [0|1]     ""     BCM
 SG_ AliveCounter   : 48|8@1+ (1,0)     [0|255]   ""     BCM
 SG_ CRC8           : 56|8@1+ (1,0)     [0|255]   ""     BCM
```

Reading a `SG_` line: `startbit|length@byteorder±  (factor,offset)  [min|max]  "unit"  receivers`,
where `@1` means little-endian and `+` means unsigned.

## 7. Implementation references

| Concern | File |
|---|---|
| C encoder/decoder | `firmware/automotive-body-ecu/src/services/can_signals.c` |
| C unit tests | `tests/test_can_signals.c` |
| Python mirror | `tools/can_tester/ecu_signals.py` |
| Peripheral driver | `firmware/automotive-body-ecu/src/drivers/can_driver.c` |

The C and Python implementations are written independently from this document
rather than one being translated from the other. If they disagree, one of the
three — the C, the Python, or this specification — is wrong, and all three are
worth finding.
