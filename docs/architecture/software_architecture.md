# Software Architecture

**Project:** Automotive Body Control ECU
**Target:** STM32F446RE (ARM Cortex-M4F @ 84 MHz, 512 KB flash, 128 KB SRAM)

---

## 1. Layered structure

```
┌──────────────────────────────────────────────────────────────┐
│  main.c                                                      │
│  Entry point. Two statements in the loop: run the scheduler, │
│  refresh the watchdog.                                       │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  APPLICATION LAYER            src/app/                       │
│                                                              │
│  body_control.c           The four periodic tasks. Decides   │
│                           what the vehicle should do.        │
│  vehicle_state_machine.c  Operating modes. PURE - no HAL.    │
└──────────┬──────────────────────────────────┬────────────────┘
           │                                  │
┌──────────▼──────────────────────────────────▼────────────────┐
│  SERVICE LAYER                src/services/  +  src/diag/    │
│                                                              │
│  scheduler.c       Time-triggered task dispatch + WCET       │
│  fault_manager.c   Maturation and healing.      PURE         │
│  can_signals.c     Byte-level packing + CRC.    PURE         │
│  can_manager.c     Alive counters, RX timeout, staleness     │
│  logger.c          Severity-tagged UART output               │
│  nvm_store.c       Power-loss-safe flash log.   PURE         │
│                                                              │
│  diagnostic stack (v0.2) - all PURE, also runs on a PC:      │
│  isotp.c           ISO 15765-2 segmentation / flow control   │
│  uds_server.c      ISO 14229-1 services, sessions, NRCs      │
│  uds_security.c    Seed/key (educational)                    │
│  dtc_manager.c     DTC status bits, aging, freeze frames     │
│  diag_manager.c    Glue: CAN -> ISO-TP -> UDS, persistence   │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  DRIVER LAYER                 src/drivers/                   │
│  The ONLY layer permitted to call the STM32 HAL.             │
│                                                              │
│  gpio_driver.c   uart_driver.c   adc_driver.c                │
│  pwm_driver.c    can_driver.c    watchdog_driver.c           │
│  flash_driver.c  mcu_driver.c                         (v0.2) │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  STM32 HAL                    Drivers/   (ST-generated)      │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  HARDWARE     GPIO · TIM3 · ADC1 · USART2 · CAN1 · IWDG      │
└──────────────────────────────────────────────────────────────┘
```

**The rule:** each layer calls only the layer directly below it.
`body_control.c` never calls the HAL. `gpio_driver.c` never calls
`fault_manager.c`. The one deliberate exception is `HAL_GetTick()`, used by
the application and service layers for timing; wrapping it would add a layer
of indirection that buys nothing.

## 2. Why this structure

### Portability

Moving to a different microcontroller means rewriting `src/drivers/`. The
application layer, the state machine, the fault manager and the CAN signal
encoding are plain C and do not change at all. That is roughly 60% of the
logic that survives a hardware change untouched.

### Testability

This is the payoff that matters most day to day. Four modules are **pure** —
no HAL include, no register access, no time source:

| Module | Why it is pure | What that buys |
|---|---|---|
| `vehicle_state_machine.c` | Maps (state, event) → state | All 25 state/event combinations tested in microseconds |
| `fault_manager.c` | Counts calls, not milliseconds | Maturation timing tested exactly, with no hardware |
| `can_signals.c` | Bytes in, bytes out | Exact wire layout asserted; bit-flip corruption simulated |
| `AdcDriver_RawToBatteryMv` | Pure inline arithmetic | All 4096 possible inputs swept for monotonicity |
| `isotp.c` | Frames in via a call, out via a function pointer, time as a parameter | Lost frames, wrong sequence numbers, timeouts replayed exactly |
| `uds_server.c` | Request bytes in, response bytes out | Every NRC path tested byte-for-byte |
| `dtc_manager.c` | Counts operation cycles, not days | 40-cycle aging tested in a loop |
| `nvm_store.c` | All flash access through function pointers | Power cuts mid-write simulated on every run |
| `diag_manager.c` | Hardware reached only through `DiagConfig_t` | The whole diagnostic stack runs on a PC as a SIL target |

### Dependency injection: how the diagnostic stack stays pure

The diagnostic stack needs to send CAN frames, erase flash and reset the MCU,
yet contains no HAL call. It receives those abilities as **function pointers**
at initialisation (`DiagConfig_t` in `diag_manager.h`):

```
                 DiagConfig_t
               ┌───────────────┐
 diag_app.c ──▶│ send_frame    │──▶ CanDriver_Transmit      (on the STM32)
 (target)      │ erase_region  │──▶ FlashDriver_EraseSector
               │ program_word  │──▶ FlashDriver_ProgramWord
               │ system_reset  │──▶ McuDriver_SystemReset
               └───────────────┘

 sil_ecu.c  ──▶  the same fields ──▶ Python callback / RAM "flash" / flag
 (PC)
```

Same diagnostic code, two platforms. This is also the shape of AUTOSAR's
split between the generic diagnostic module (DCM) and the ECU-specific data it
serves: `src/diag/` would work unchanged in a door module; `diag_app.c` is
what makes it a *Body Control* ECU (its DIDs, DTCs and routine).

Provoking the same coverage on hardware would mean a marginal battery voltage
held at exactly the right level for exactly the right number of cycles. In
software it is a loop.

**Purity is not an aesthetic preference. It is the difference between a module
you can prove correct and one you can only hope about.**

### Reviewability

The transition table in `vehicle_state_machine.c` is six rows. A reviewer
reads them and knows every legal move in the system. The equivalent written as
nested `switch` statements spreads the same information across branches, where
a missing case looks exactly like a deliberate omission.

## 3. Execution model

Cooperative, time-triggered. No RTOS, no pre-emption, one stack.

```
main()
  └── while (1)
        ├── Scheduler_Run()          runs whatever is due, returns immediately
        └── WatchdogDriver_Refresh() only reached if the scheduler returned
```

| Task | Period | Responsibility |
|---|---|---|
| `diag_rx` | 5 ms | Drain the CAN receive queue, run ISO-TP and UDS, save DTCs (v0.2) |
| `input_sm` | 10 ms | Sample inputs, debounce, generate events, run the state machine |
| `outputs` | 50 ms | Status LED, lamp self-test, headlights, indicators, brake PWM |
| `can_comms` | 100 ms | Transmit BCM_VehicleStatus |
| `diag` | 500 ms | Measure battery, evaluate voltage faults, transmit diagnostics |

One interrupt exists alongside the tasks: **CAN receive** (`CAN1_RX0_IRQHandler`).
It only copies frames from the 3-message hardware FIFO into a 32-frame
software queue; all processing happens in `diag_rx`. The queue is a
single-producer / single-consumer ring buffer, safe without locks because each
index has exactly one writer — see the comment block in `can_driver.c`.

### Why cooperative rather than an RTOS

A task runs to completion; nothing interrupts it. Consequences:

- **No race conditions between tasks.** `s_battery_mv` is written by the
  500 ms task and read by the 100 ms task with no mutex and no atomics,
  because one cannot interrupt the other. Under pre-emption this would be a
  genuine bug.
- **One stack.** No per-task stack sizing, and no stack-overflow class of
  failure.
- **Hand-analysable timing.** Worst case is the sum of the tasks that can
  coincide, which is why the scheduler measures and reports it.

The cost: a long task delays every other task. Every task must be short and
must never block. That constraint is what the 5 ms budget (REQ-050) enforces.

An RTOS becomes worth its complexity when tasks have genuinely different
criticality, or when one must block on I/O. Neither applies here, and adopting
one would add pre-emption hazards in exchange for nothing.

### Period selection

Each period comes from the physics of the signal, not from a round number:

- **10 ms** — below the ~100 ms threshold at which a human perceives lag, and
  it sets the debounce time base (3 × 10 ms = 30 ms).
- **50 ms** — resolves the 333 ms indicator half-period to within 15%.
- **100 ms** — the conventional automotive period for vehicle status.
- **500 ms** — battery voltage changes over seconds; sampling faster would
  cost CPU time and add noise without adding information.

## 4. Data flow

```
  buttons ──▶ gpio_driver ──▶ body_control ──▶ vehicle_state_machine
                 (debounce)     (events)            (transitions)
                                     │                     │
                                     │                     ▼
                                     │              capabilities
                                     │                     │
                                     ▼                     ▼
  ADC ────────▶ adc_driver ──▶ fault_manager ────▶ output control
                (oversample)    (maturation)             │
                                     │                   ├──▶ gpio_driver ──▶ lamps
                                     ▼                   └──▶ pwm_driver ──▶ brake light
                               can_manager
                                     │
                                     ▼
                              can_signals ──▶ can_driver ──▶ CAN bus
                              (pack + CRC)
```

## 5. Memory

Measured from the linked image (`arm-none-eabi-size`):

| Section | Bytes | Of available | Contents |
|---|---|---|---|
| `.text` | 45 796 | 17.5% of the 256 KB code region | Code and constants |
| `.data` | 144 | — | Initialised globals (in flash, copied to RAM at boot) |
| `.bss` | 4 440 | 3.4% of 128 KB SRAM | Zero-initialised globals (incl. two 256-byte ISO-TP buffers and the 32-frame CAN queue) |

v0.1 was 29 KB of code; the diagnostic stack added about 16 KB.

### Flash memory map

```
0x08000000 ┌────────────────────────────┐
           │ sectors 0-5   256 KB       │  code region (linker script FLASH)
           │ vector table, code, consts │
0x08040000 ├────────────────────────────┤
           │ sector 6      128 KB       │  NVM region A  ┐ DTCs + VIN,
0x08060000 ├────────────────────────────┤                │ ping-pong
           │ sector 7      128 KB       │  NVM region B  ┘ (nvm_store.c)
0x0807FFFF └────────────────────────────┘
```

The linker script caps the code region at 256 KB. Without that cap the linker
could legally place code in sectors 6-7, and the first DTC save would erase
part of the running firmware.

**No dynamic allocation anywhere.** `malloc` is never called. Every buffer is
statically sized, so memory use is known at link time and cannot fail at
runtime. This is standard practice in automotive firmware for two reasons:
heap exhaustion is an unrecoverable runtime failure with no safe response, and
fragmentation makes worst-case behaviour unprovable.

All lookup tables — the state transition table, the GPIO descriptor tables,
the fault and state name arrays — are `const`, so they live in flash and cost
no RAM.

## 6. Naming conventions

| Pattern | Meaning | Example |
|---|---|---|
| `ModuleName_FunctionName` | Public API | `GpioDriver_SetOutput` |
| `module_name_helper` | File-local (`static`) | `body_control_read_ignition_event` |
| `s_variable` | File-local state (`static`) | `s_battery_mv` |
| `TypeName_t` | Type definition | `VehicleState_t` |
| `MODULE_CONSTANT` | Macro constant | `ECU_TASK_PERIOD_10MS` |

The prefix makes the owning module visible at every call site. Reading
`FaultManager_IsActive(...)` in `body_control.c` tells you where to look
without opening a header.

## 7. Configuration

Every tunable value lives in `src/config/ecu_config.h`, and every pin in
`src/config/pin_config.h`. No magic numbers anywhere else.

Each derived constant carries its derivation as a comment — the PWM prescaler
shows the division from the 84 MHz timer clock, the CAN prescaler shows the
bit-time arithmetic, the watchdog reload shows the LSI division. A number
whose origin is not written down cannot be safely changed by the next person,
and on this project the next person is you in six months.
