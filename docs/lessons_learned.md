# Lessons Learned

Design decisions, problems actually hit during development, and what would be
done differently. Written because the reasoning behind a choice is worth more
six months later than the choice itself — and because a project that reports
only its successes teaches nobody anything.

Sections 1-4 cover v0.1 (the body control functions). Section 5 covers v0.2
(UDS diagnostics).

---

## 1. Problems hit during development

### 1.1 STM32CubeIDE's "Empty Project" produces no `.ioc`

**Symptom.** A project created through *File → New → STM32 Project →
STM32CubeIDE Empty Project* contained a startup file and a linker script but
no `.ioc` configuration file and no `Drivers/` folder. Every HAL call failed
to compile with `unknown type name 'UART_HandleTypeDef'`.

**Cause.** In CubeIDE 2.2.0 that wizard entry creates a genuinely *empty*
project — correct startup code for the chosen part, but no CubeMX integration
and no HAL. The HAL only appears after configuring peripherals in CubeMX and
generating code, which cannot happen without an `.ioc`.

**Resolution.** Configure the project in **standalone STM32CubeMX**, set
*Project Manager → Toolchain/IDE* to *STM32CubeIDE*, generate, then import the
result into CubeIDE. This is a normal professional workflow — CubeMX owns
configuration, CubeIDE owns editing and debugging — and it sidesteps the IDE's
project wizard entirely.

**Lesson.** Several hours went into fighting a tool before questioning whether
that tool was the right entry point. When a standard workflow repeatedly fails
in the same way, the assumption to re-examine first is that it is the standard
workflow.

### 1.2 HAL modules are opt-in, and the sources are not copied

**Symptom.** After the project was generated correctly, ADC, CAN, TIM, UART
and IWDG code still would not compile.

**Cause.** Two separate things, both easy to miss:

1. `Core/Inc/stm32f4xx_hal_conf.h` ships with most `HAL_*_MODULE_ENABLED`
   defines commented out. A module that is not enabled has its entire header
   `#ifdef`-ed away, so its types simply do not exist.
2. CubeMX only copies the HAL `.c` files for peripherals it was told about.
   Peripherals configured in driver code rather than in CubeMX have no sources
   in the project at all.

**Resolution.** Uncomment the five module defines, and copy the corresponding
`.c` and `.h` files from the STM32Cube firmware package
(`~/STM32Cube/Repository/STM32Cube_FW_F4_*/`).

**Lesson.** "Undefined type" in an STM32 project almost always means a missing
`MODULE_ENABLED`, not a missing include. Recognising the signature of a class
of error saves far more time than debugging each instance from scratch.

### 1.3 A new source folder is excluded from the build by default

**Symptom.** `gpio_driver.c` compiled fine, but the link failed with
`undefined reference to GPIO_Driver_Init`.

**Cause.** CubeIDE marks newly created folders as excluded from the build. The
header was found, so every call site compiled; the `.c` was never compiled, so
the linker had no function body to resolve against.

**Resolution.** Right-click the folder → *Resource Configurations → Exclude
from Build* → uncheck both Debug and Release.

**Lesson.** This is the clearest illustration of the compiler/linker boundary
available. A *compiler* error means the code is malformed. A *linker* error
means something was promised in a header and never delivered in an object
file. Reading which of the two you have immediately narrows the search from
the whole codebase to the build configuration.

### 1.4 `CAN_DLC_x` constants do not exist on STM32F4

**Symptom.** A lookup table mapping byte counts to symbolic HAL constants
failed with `'CAN_DLC_0' undeclared`.

**Cause.** The assumption was wrong. On STM32F4's bxCAN, `CAN_TxHeaderTypeDef.DLC`
takes a plain integer 0–8. The symbolic codes exist on CAN-FD peripherals,
where DLC values above 8 are non-linear (9 means 12 bytes, 10 means 16).

**Resolution.** Assign the byte count directly, and document *why* no mapping
is needed — including the CAN-FD case where it would be.

**Lesson.** The original code carried a comment explaining that the explicit
mapping avoided "relying on a coincidence". The comment was confidently
written and entirely wrong. **A comment justifying an assumption is not
evidence for it.** Checking the header took thirty seconds.

### 1.5 The system clock is 84 MHz, not the 180 MHz the part supports

**Symptom.** Nothing failed. That is what makes it worth recording.

**Cause.** CubeMX's automatic clock configuration settled on 84 MHz. The
STM32F446 can reach 180 MHz, and every early timing calculation had assumed it
did.

**Resolution.** Derive every peripheral constant from named macros —
`ECU_APB1_HZ`, `ECU_APB1_TIMER_HZ` — defined once in `ecu_config.h`, with the
arithmetic written out beside each one.

**Lesson.** A PWM frequency that is silently wrong by a factor of two is a
much worse defect than one that fails to compile, because nothing draws
attention to it. Every derived constant in this project now shows its
derivation, so an incorrect clock assumption becomes visible at review rather
than at measurement.

---

## 2. Design decisions and their reasoning

### 2.1 Drivers configure their own peripherals

Rather than relying on CubeMX-generated `MX_*_Init()` functions, each driver
enables its own clocks and configures its own peripheral.

**Why:** the ADC's sampling time sits next to the code that explains why that
sampling time is correct for a 1.8 kΩ source impedance. In CubeMX it would be a
dropdown in a GUI, invisible in the repository and lost on the next
regeneration.

**Cost:** the `.ioc` no longer describes the full hardware configuration, so a
newcomer opening CubeMX sees less than the truth. Acceptable here because
`pin_config.h` is the single authority and says so; on a larger team the
divergence between tool and code would be a real hazard.

### 2.2 Purity as an explicit architectural goal

Four modules are kept free of the HAL, of hardware, and of any time source —
not as a stylistic preference, but so their behaviour can be proven.

The fault manager is the clearest case. It counts *calls*, not milliseconds.
That one decision makes maturation timing exactly testable: 3 calls latches, 5
clean calls clears, and an interrupted sequence restarts. Reading a clock
instead would have made the same behaviour require a marginal battery voltage
held at exactly the right level for exactly the right duration — essentially
untestable, and therefore in practice untested.

**Purity is the difference between a module you can prove and one you can only
hope about.**

### 2.3 Fault maturation is asymmetric on purpose

Three consecutive detections latch a fault; five consecutive clean evaluations
clear it.

Symmetric counters would let a condition sitting exactly on its threshold
oscillate between set and clear indefinitely. Making healing stricter than
latching means the ECU is quick to suspect a fault and slow to dismiss one —
the safe direction to be wrong in. This mirrors the DTC maturation strategies
in ISO 14229.

### 2.4 A transition table, not nested switch statements

The state machine's entire legal behaviour is six `const` rows. A reviewer
reads them and knows every possible move in the system.

The equivalent written as nested `switch` statements spreads the same
information across branches where a missing case is indistinguishable from a
deliberate omission. The table also lives in flash and costs no RAM.

**Exception:** fault entry applies from *every* state and is handled as an
explicit rule before the table lookup. Encoding it as four near-identical rows
would invite one of them to be forgotten when a fifth state is added.

### 2.5 Cooperative scheduling instead of an RTOS

Tasks run to completion. No pre-emption, no mutexes, one stack.

The concrete payoff: `s_battery_mv` is written by the 500 ms task and read by
the 100 ms task with no synchronisation at all, because one cannot interrupt
the other. Under an RTOS that is a genuine race condition requiring a mutex or
an atomic.

An RTOS earns its complexity when tasks have genuinely different criticality,
or when one must block on I/O. Neither applies here, and adopting one would add
pre-emption hazards in exchange for nothing.

### 2.6 The test framework validates itself

The system test suite is run against a software model of the ECU in CI, and
must report `INCONCLUSIVE` against an empty bus and `12/12 PASSED` against the
model.

A test suite that has never passed is an untested assertion. Without this
check, a bug that made every test skip would leave CI green while verifying
absolutely nothing — the worst possible failure mode for a test framework,
because it actively creates false confidence.

The simulator deliberately shares no code with the firmware. It reimplements
the observable behaviour from the specification, which is what makes a
disagreement between them meaningful.

---

## 3. What would be done differently

| | Current | Better | Why it was not done |
|---|---|---|---|
| **UART logging** | Blocking transmit, ~87 µs per character | Ring buffer + DMA | Blocking is simpler to reason about while learning. It is also a real hazard: a 50-character line costs 4.3 ms of a 10 ms budget, so logging inside a fast task causes the deadline miss it was added to diagnose. |
| **Timing resolution** | 1 ms, from `HAL_GetTick()` | DWT cycle counter, ~12 ns at 84 MHz | Tasks taking 0.4 ms and 1.4 ms are currently indistinguishable, which makes the reported worst-case execution time far coarser than the analysis deserves. |
| **CAN reception** | ~~Polled every 100 ms~~ | Interrupt-driven | **Done in v0.2** - a diagnostic tester's multi-frame request made the change necessary rather than optional. See 5.2. |
| **Watchdog timeout** | Nominal 500 ms, LSI ±50% | Measure LSI against a precise clock and compensate | The true timeout lies between roughly 330 ms and 1 s. Fine for a teaching project, unacceptable for a safety function. |
| **CAN encode/decode** | Hand-written in C and Python | Generated from a DBC file with `cantools` | Writing it by hand makes the bit packing visible, which is the educational point. A production project generates both sides from one source so they cannot diverge. |
| **Test framework** | Custom `unity_min.h` | Vendored ThrowTheSwitch Unity | The API is deliberately identical, so migration is one submodule and a Makefile line. Keeping it small avoided burying the tests under 2000 lines of third-party source. |

---

## 4. Things that turned out to matter more than expected

**Naming is load-bearing.** `BUTTON_PRESSED_STATE` instead of
`GPIO_PIN_RESET` turns a comparison into a sentence, and localises the
active-low assumption to a single line. Changing to an active-high button is a
one-line edit rather than an audit.

**Writing the derivation next to the constant.** Every timing value in
`ecu_config.h` shows its arithmetic. This caught the 84 MHz assumption, and it
means the next person can change the clock without reverse-engineering six
prescalers.

**Reporting skips honestly.** The distinction between "the ECU is wrong" and
"this test proved nothing" is the most important thing the test runner does.
It would have been easy — and much more satisfying — to print a green summary
for a run in which nothing was verified.

**Stating what the project is not.** The requirements document lists explicit
scope exclusions and known limitations. This is more convincing than claiming
completeness, and it is exactly what a reviewer looks for: an engineer who
knows the boundaries of their own work is far easier to trust than one whose
work appears to have none.

---

## 5. Version 0.2 — UDS diagnostics

### 5.1 Running the tests for the first time found a defect in v0.1

**Symptom.** v0.1 shipped 67 unit tests that had only ever been *compiled*,
never run, because the development PC had no host C compiler. When v0.2
brought one in (clang, via `pip install ziglang`), `test_can_signals.c` failed
to build: an uninitialised struct was passed to a function by pointer.

**Cause.** GCC does not warn about this case; clang does
(`-Wuninitialized-const-pointer`). It was harmless in practice - the callee
returned on its other, NULL, argument before reading the struct - but it is
undefined behaviour, and `-Werror` correctly refused it.

**Lesson.** Tests that have never been executed are not evidence of anything,
however carefully written. And two compilers see more than one: CI now uses
GCC while local runs use clang, and every host-compiled file is checked with
both before a push.

### 5.2 The IDE build could not have worked

**Symptom.** While adding `src/diag/` to the CubeIDE project, the `.cproject`
turned out to list only `../src/drivers` as an include path. `src/app`,
`src/services` and `src/config` - all of v0.1's application code - were
missing from both the Debug and Release configurations.

**Cause.** Every v0.1 build had been done from the command line with explicit
`-I` flags, which hid the problem completely. Importing the project into
CubeIDE and pressing Ctrl+B would have failed on the first `#include`.

**Lesson.** Build the way your users build. A README that says "import into
CubeIDE and press Ctrl+B" is a claim, and the only way to know it is true is
to do exactly that.

### 5.3 CI had never run

The v0.1 commit existed locally but had never been pushed, so the CI pipeline
described in the README had not executed once. A pipeline that is never
triggered protects nothing, and a green badge on an old commit proves nothing
about the current one.

### 5.4 A failing test that was right about the code and wrong about itself

**Symptom.** Two DTC aging tests failed: after "40 clean cycles" the DTC was
still confirmed, and an aging counter read 29 where 30 was expected.

**Cause.** The code was correct and the tests were not. The operation cycle in
which a DTC *fails* is not a clean cycle. The first `StartOperationCycle()`
after a failure closes that failing cycle without aging; only later cycles
count.

**Lesson.** When a test fails, the question is "which of the two is wrong?",
not "how do I make it pass?". Re-reading the standard settled it in a minute.
Changing the code to match the test would have introduced a real defect in
order to silence a correct alarm.

### 5.5 The ECU was right; the test harness leaked state

**Symptom.** In the first full SIL run, 23 of 24 diagnostic tests passed. The
lamp self-test was refused with NRC `0x22` conditionsNotCorrect.

**Cause.** An earlier test had set the simulated vehicle state to RUN, and the
lamp test is - correctly - forbidden while the engine runs. The SIL power
cycle reset the diagnostic stack but not the simulated application, so one
test's state leaked into the next. On a real ECU a reset always returns to OFF.

**Lesson.** Test isolation is part of the test, not a convenience. It also
illustrates why the ECU refusing was a *good* result: a stack that had allowed
the routine would have hidden the harness bug and shipped a safety defect.

### 5.6 Flash erase time shaped the whole persistence design

Erasing a 128 KB sector blocks the CPU for 1-2 s. The watchdog times out after
~0.5 s. The obvious design - erase, then rewrite the DTC table - therefore
resets the ECU in the middle of every save.

The append-only store follows directly from that single number: records are
appended (microseconds per word, no erase), two sectors alternate, and erasing
happens only at start-up before the watchdog is armed. Understanding the
physical constraint first made the design almost obvious; starting from the
data structure would have led to a design that works on the bench until the
day the timing lines up badly.

### 5.7 A deterministic generator makes replay possible

The first seed/key version seeded its random generator once at boot, from the
chip's unique ID and the boot time. Both are the same on every power-up, so
the first seed after every boot was identical - and a key recorded once could
be replayed forever. The fix mixes the millisecond timestamp of each seed
request into the generator. It is still not cryptography, and the
documentation says so, but it removes an attack that needed no skill at all.

### 5.8 The first CI run failed - on the build system, not the code

**Symptom.** The very first CI run after pushing v0.2: four jobs green
(firmware build, SIL diagnostics, test framework, static analysis), one red.
`cd tests && make` exited with code 2:

```
make: Circular build <- build/test_vehicle_state_machine dependency dropped.
/usr/bin/ld: cannot open output file build/test_fault_manager: No such file or directory
```

**Cause.** The Makefile used the name `build` for two different things: a
phony target meaning "compile but do not run", and the output *directory* that
every executable needs. Each executable therefore had `build` as an
order-only prerequisite, while `build` depended on every executable. Make
detected the cycle, broke it by dropping the directory dependency, and the
linker had nowhere to write.

The fix is to create the directory inside the recipe (`mkdir -p $(@D)`)
instead of making it a prerequisite.

**Why it was not caught earlier.** Every local run used
`tools/run_unit_tests.py`, which needs no make. The Makefile had been checked
by reading it, not by running it - on Windows there was no `make` to run it
with. Reproducing meant installing gcc and make inside WSL and running exactly
what CI runs.

**Lesson.** This is section 5.2 all over again, and it is worth noticing that
it repeated: *a build path nobody executes is a build path that does not
work*. There are now two ways to run the tests, and both are exercised - the
Python runner locally, the Makefile in CI.

It is also a good result in its own right: the pipeline caught a real defect on
its first run, which is exactly what it exists for. Four green jobs and one
red is far more informative than a green badge nobody has tested.

### 5.9 Testing against someone else's implementation

The UDS tester is not hand-written: it is udsoncan and can-isotp, maintained
by other people and used widely. That was deliberate. A client written by the
same person as the server tends to share its misreadings of the standard, so
the two agree and every test passes. When an independent implementation
accepts our segmented responses, our NRCs and our flow control, that is
evidence the ECU follows the standard - not merely that it agrees with itself.
