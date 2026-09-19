# UDS Diagnostic Specification

**Standard:** ISO 14229-1 (UDS) over ISO 15765-2 (ISO-TP) on classic CAN
**ECU:** Body Control ECU, firmware 0.2.0
**Implementation:** `firmware/automotive-body-ecu/src/diag/`

This document is the contract between the ECU and any diagnostic tester. The
C implementation and the Python tester (`tools/can_tester/diag_client.py`)
were both written from it, independently.

---

## 1. What UDS is, in one trace

A tester asks, the ECU answers. Every request starts with a Service Identifier
(SID). A positive answer echoes `SID + 0x40`; a refusal is always
`7F <SID> <NRC>`. This is a real capture from the SIL target, taken from the
production C code:

```
# 10 03  DiagnosticSessionControl -> extended session
  0x7E0  02 10 03 CC CC CC CC CC        request  (SF, 2 bytes)
  0x7E8  06 50 03 00 32 01 F4 CC        positive (0x10 + 0x40 = 0x50), P2 = 50 ms, P2* = 5 s

# 27 01 / 27 02  SecurityAccess
  0x7E0  02 27 01 CC CC CC CC CC        request seed
  0x7E8  06 67 01 21 21 03 83 CC        seed = 0x21210383
  0x7E0  06 27 02 8E B4 D6 2F CC        key  = f(seed)
  0x7E8  02 67 02 CC CC CC CC CC        unlocked

# 2E F1 90  WriteDataByIdentifier VIN - 20 bytes, so ISO-TP splits it
  0x7E0  10 14 2E F1 90 57 44 42        First Frame: total length 0x014 = 20
  0x7E8  30 00 05 CC CC CC CC CC        Flow Control: continue, BS 0, STmin 5 ms
  0x7E0  21 32 30 33 30 34 36 31        Consecutive Frame 1
  0x7E0  22 41 31 32 33 34 35 36        Consecutive Frame 2
  0x7E8  03 6E F1 90 CC CC CC CC        positive (0x2E + 0x40 = 0x6E)

# 19 02 08  ReadDTCInformation - which DTCs are confirmed?
  0x7E0  03 19 02 08 CC CC CC CC
  0x7E8  07 59 02 FF 05 62 00 AF        P0562-00, status 0xAF
```

Reading that last line: `59 02` = positive response to report type 0x02; `FF` =
every status bit is supported; `05 62 00` = DTC **P0562-00** (system voltage
low); `AF` = failing now, confirmed, warning lamp on (section 8).

---

## 2. Addressing

| Direction | CAN ID | Use |
|---|---|---|
| Tester → ECU, physical | `0x7E0` | Addressed to this ECU only |
| Tester → ECU, functional | `0x7DF` | Broadcast to every ECU; **single frame only** |
| ECU → tester | `0x7E8` | All responses |

11-bit identifiers at 500 kbit/s, the ISO 15765-4 convention for the first ECU
on a bus. Functional requests must fit in one frame because a multi-frame
exchange needs one specific receiver to send flow control.

## 3. Transport (ISO-TP)

| Parameter | Value | Meaning |
|---|---|---|
| Frame length | always 8 bytes | unused bytes padded with `0xCC` |
| Maximum message | 256 bytes | longer requests get Flow Control `OVERFLOW` |
| Block size (BS) sent by ECU | 0 | tester may send all CFs without waiting |
| STmin sent by ECU | 5 ms | minimum gap between the tester's CFs |
| N_Cr | 1000 ms | max wait for the next CF when receiving |
| N_Bs | 1000 ms | max wait for a Flow Control when sending |
| WAIT frames accepted | 10 | then the transfer is abandoned |

Frame types (high nibble of byte 0): `0` Single, `1` First, `2` Consecutive
(sequence number 1..15, 0, 1..), `3` Flow Control (`30` continue, `31` wait,
`32` overflow).

## 4. Timing

| Parameter | Value | Reported in |
|---|---|---|
| P2server | 50 ms | `10 xx` response, bytes 2-3 (ms) |
| P2*server | 5000 ms | `10 xx` response, bytes 4-5 (units of 10 ms) |
| S3server | 5000 ms | — |

The ECU processes diagnostics in a 5 ms task, so a single-frame request is
answered within one to two task periods, well inside P2.

**S3:** in any non-default session, if no request arrives for 5 s the ECU
returns to the default session and re-locks security. Send `3E 80` (suppressed
TesterPresent) every ~2 s to hold a session open.

## 5. Sessions and services

| SID | Service | Default | Extended | Sub-functions / format |
|---|---|:---:|:---:|---|
| `10` | DiagnosticSessionControl | ✓ | ✓ | `01` default, `03` extended (`02` programming → NRC 12) |
| `11` | ECUReset | ✓ | ✓ | `01` hard, `03` soft |
| `14` | ClearDiagnosticInformation | ✓ | ✓ | `14 FF FF FF` all, or one 3-byte DTC |
| `19` | ReadDTCInformation | ✓ | ✓ | `01` `02` `04` `06` `0A` (section 8) |
| `22` | ReadDataByIdentifier | ✓ | ✓ | up to 8 DIDs per request |
| `27` | SecurityAccess | | ✓ | `01` request seed, `02` send key |
| `2E` | WriteDataByIdentifier | | ✓ 🔒 | writable DIDs only |
| `31` | RoutineControl | | ✓ | `01` start, `02` stop, `03` results |
| `3E` | TesterPresent | ✓ | ✓ | `00` |
| `85` | ControlDTCSetting | | ✓ | `01` on, `02` off |

🔒 requires security access. Setting bit 7 of a sub-function
(`suppressPosRspMsgIndicationBit`, e.g. `3E 80`) suppresses the positive
response; negative responses are never suppressed by it.

**Every session change re-locks security.** Returning to the default session
also re-enables DTC setting, so `85 02` cannot outlive the tester that sent it.

## 6. Security access (0x27)

```
tester              ECU
  27 01    ─────▶
           ◀─────   67 01 <seed: 4 bytes, big-endian>
  27 02 <key>  ──▶
           ◀─────   67 02                       (or 7F 27 35 invalid key)
```

| Rule | Behaviour |
|---|---|
| Already unlocked | seed is `00 00 00 00` |
| Seed | never `00000000` while locked; single use |
| Key without a fresh seed | NRC `24` requestSequenceError |
| Wrong key, attempts 1-2 | NRC `35` invalidKey |
| Wrong key, attempt 3 | NRC `36` exceededNumberOfAttempts, 10 s lockout starts |
| Seed request during lockout | NRC `37` requiredTimeDelayNotExpired |

**Key algorithm (educational, not secure):**

```
key = seed XOR 0xA5C3F00F
key = rotate_left(key, 7)
key = (key + 0x1D2B3C4D) mod 2^32
key = key XOR (key >> 11)
```

Test vectors: `0x00000001 → 0xFF3CA7F7`, `0x12345678 → 0x18FD67E7`,
`0xDEADBEEF → 0xD44826DF`.

> This algorithm can be recovered from three seed/key pairs. It demonstrates
> the protocol mechanics only. Production ECUs use AES-128-CMAC with a
> per-vehicle key in a hardware security module, or ISO 14229 service 0x29.

The seed generator (xorshift32) is seeded from the STM32's 96-bit unique ID
and mixes in the millisecond timestamp of each request, so the first seed
after power-up is not the same on every boot.

## 7. Data identifiers

All multi-byte values are **big-endian**, the UDS convention (unlike this
project's CAN signals, which are little-endian — byte order is a property of
each protocol).

| DID | Name | Length | Access | Encoding |
|---|---|---|---|---|
| `F190` | VIN | 17 | read, write 🔒 | ASCII, ISO 3779 characters only |
| `F195` | Software version | 3 | read | major, minor, patch |
| `F18C` | ECU serial number | 12 | read | STM32 96-bit unique ID |
| `F186` | Active session | 1 | read | `01` / `03` |
| `0100` | Battery voltage | 2 | read | mV |
| `0101` | Vehicle state | 1 | read | 0 OFF, 1 ACC, 2 ON, 3 RUN, 4 FAULT |
| `0102` | Digital inputs | 1 | read | bit 0 ignition, 1 brake, 2 left, 3 right |
| `0103` | Active fault bitmask | 2 | read | bit N = `FaultId_t` N |
| `0104` | Uptime | 4 | read | seconds since reset |

`F1xx` identifiers are standardised by ISO 14229-1 Annex C; `01xx` are this
project's own. A VIN containing `I`, `O` or `Q`, or any lower-case or
non-alphanumeric character, is refused with NRC `31`.

## 8. DTCs

### 8.1 DTC table

| Fault | DTC | Bytes | Warning lamp | Meaning |
|---|---|---|:---:|---|
| `BATT_UNDERVOLT` | P0562-00 | `05 62 00` | ✓ | System voltage low |
| `BATT_OVERVOLT` | P0563-00 | `05 63 00` | ✓ | System voltage high |
| `ADC_FAILURE` | B1001-49 | `90 01 49` | ✓ | Battery sense – internal electronic failure |
| `CAN_BUS_OFF` | U0001-88 | `C0 01 88` | | High-speed CAN – bus off |
| `CAN_TX_FAIL` | U0001-00 | `C0 01 00` | | High-speed CAN – general failure |
| `TASK_OVERRUN` | U3000-49 | `F0 00 49` | | Control module – internal electronic failure |

Decoding the first two bytes (SAE J2012): bits 15-14 give the letter (`00` P,
`01` C, `10` B, `11` U), bits 13-12 the first digit (0-3), and the remaining
12 bits three hex digits. The third byte is the failure type (J2012-DA:
`0x49` internal electronic failure, `0x88` bus off).

### 8.2 Status byte (ISO 14229-1 Annex D)

| Bit | Name | Set when |
|---|---|---|
| 0 | testFailed | failing at the last evaluation |
| 1 | testFailedThisOperationCycle | failed at least once this ignition cycle |
| 2 | pendingDTC | failed this or the previous cycle |
| 3 | confirmedDTC | stored (the fault manager already matured it) |
| 4 | testNotCompletedSinceLastClear | not evaluated since the last clear |
| 5 | testFailedSinceLastClear | failed at least once since the last clear |
| 6 | testNotCompletedThisOperationCycle | not evaluated yet this cycle |
| 7 | warningIndicatorRequested | failing, and the DTC lights the lamp |

| Status | Reading |
|---|---|
| `0x50` | never tested since clear (initial) |
| `0x00` | tested, passing, no history |
| `0xAF` | failing now, confirmed, lamp on |
| `0x2E` | **intermittent**: failed this cycle, confirmed, but not failing now |

**Operation cycle** = one ignition cycle (OFF → ACC). **Aging:** a confirmed
DTC that completes 40 cycles without failing is erased automatically. The
cycle in which it failed does not count.

### 8.3 ReadDTCInformation report types

| Request | Response |
|---|---|
| `19 01 <mask>` | `59 01 FF 01 <count:2>` — number of DTCs where `status & mask ≠ 0` |
| `19 02 <mask>` | `59 02 FF` then `<DTC:3> <status>` for each match |
| `19 0A` | `59 0A FF` then every supported DTC with its status |
| `19 04 <DTC:3> 01` | `59 04 <DTC> <status>` + snapshot record (below) |
| `19 06 <DTC:3> FF` | `59 06 <DTC> <status> 01 <occurrences> 02 <aging counter>` |

**Snapshot (freeze frame), record `01`**, captured the first time the DTC is
confirmed:

```
01 02  01 00 <battery mV:2>  01 01 <vehicle state:1>
│  │   └ DID 0x0100          └ DID 0x0101
│  └ 2 identifiers
└ record number
```

## 9. Routine: lamp self-test (RID `0x0201`)

| Request | Response | Notes |
|---|---|---|
| `31 01 02 01` | `71 01 02 01` | All exterior lamps on for 3 s. Needs security. NRC `22` while RUN. |
| `31 02 02 01` | `71 02 02 01` | Stop early |
| `31 03 02 01` | `71 03 02 01 <s>` | `s` = `00` never run, `01` running, `02` completed |

## 10. Negative Response Codes used

| NRC | Name | Typical cause here |
|---|---|---|
| `11` | serviceNotSupported | unknown SID |
| `12` | subFunctionNotSupported | e.g. `10 02` (programming session) |
| `13` | incorrectMessageLengthOrInvalidFormat | wrong byte count |
| `14` | responseTooLong | response would exceed 256 bytes |
| `22` | conditionsNotCorrect | lamp test while engine runs |
| `24` | requestSequenceError | key sent without a seed |
| `31` | requestOutOfRange | unknown DID/DTC/RID, invalid VIN |
| `33` | securityAccessDenied | protected service while locked |
| `35` | invalidKey | wrong key |
| `36` | exceededNumberOfAttempts | third wrong key |
| `37` | requiredTimeDelayNotExpired | seed requested during lockout |
| `7F` | serviceNotSupportedInActiveSession | e.g. `27` in the default session |

**Functional requests:** NRCs `11`, `12`, `31`, `7E` and `7F` are **not sent**
for requests received on `0x7DF`. Otherwise one broadcast would trigger a
refusal from every ECU that simply does not implement the service.

## 11. Persistence

The VIN and all DTC records (status, counters, freeze frame) are stored in
flash sectors 6 and 7 (`0x08040000`–`0x0807FFFF`), excluded from the code
region by the linker script. Records are appended; power loss mid-write is
detected by CRC-32 and the previous record is used. Sectors are only erased
at start-up, before the watchdog is armed. See `src/services/nvm_store.h`.

`11 01` (ECUReset) forces a save before resetting, and only resets after the
positive response has left the CAN controller.

## 12. Not implemented

| Service / feature | Why not |
|---|---|
| Programming session, `34`/`36`/`37` download | needs a bootloader — a natural next project |
| `28` CommunicationControl | no second network to control |
| `2F` InputOutputControlByIdentifier | the lamp-test routine covers actuator testing |
| `29` Authentication | certificate-based security; out of scope |
| `78` responsePending | every handler completes well within P2 |
| DoIP (ISO 13400) | diagnostics over Ethernet; CAN only here |
