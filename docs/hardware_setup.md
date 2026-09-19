# Hardware Setup

How to wire the bench so the firmware in this repository runs and can be tested.

---

## 1. Bill of materials

| Item | Qty | Purpose | ~Cost (USD) |
|---|---|---|---|
| STM32 Nucleo-F446RE | 1 | The ECU (2 for the multi-node setup) | 17 |
| MCP2551 or SN65HVD230 CAN transceiver module | 1–2 | Logic level → differential bus | 3 |
| 5 mm LED, any colour | 4 | Headlight, 2 × indicator, brake light | 1 |
| 220 Ω resistor | 4 | LED current limiting | <1 |
| Tactile push button | 3 | Brake, turn left, turn right | 1 |
| 10 kΩ linear potentiometer | 1 | Simulates battery voltage | 1 |
| 10 kΩ resistor | 1 | Divider upper leg | <1 |
| 2.2 kΩ resistor | 1 | Divider lower leg | <1 |
| 120 Ω resistor | 2 | CAN bus termination | <1 |
| Breadboard + jumper wires | 1 | — | 4 |
| USB A → Mini-B cable | 1 | Power, flashing, debug, UART | — |

**Total: roughly 30 USD.** CAN parts are only needed from Phase 13 onward;
everything up to and including fault management works with LEDs and buttons.

## 2. Pin assignment

| Function | Pin | Direction | Notes |
|---|---|---|---|
| Ignition switch | PC13 | Input | Onboard blue button B1, active low |
| Brake pedal | PC0 | Input | External button to GND, internal pull-up |
| Turn left | PC1 | Input | External button to GND, internal pull-up |
| Turn right | PC2 | Input | External button to GND, internal pull-up |
| Status LED | PA5 | Output | Onboard green LED LD2 |
| Headlight | PB0 | Output | External LED + 220 Ω |
| Indicator left | PB1 | Output | External LED + 220 Ω |
| Indicator right | PB2 | Output | External LED + 220 Ω |
| Brake light (PWM) | PB4 | Output | TIM3_CH1, AF2, LED + 220 Ω |
| Battery sense | PA0 | Analog | ADC1_IN0, behind the divider |
| UART TX | PA2 | AF7 | To ST-Link virtual COM port |
| UART RX | PA3 | AF7 | To ST-Link virtual COM port |
| CAN RX | PB8 | AF9 | To transceiver RXD |
| CAN TX | PB9 | AF9 | To transceiver TXD |

**Pins to avoid:** PA13 (SWDIO), PA14 (SWCLK) and PB3 (SWO) carry the debug
interface. Reassigning them makes the board unprogrammable through the
ST-Link, which is a tedious mistake to recover from.

## 3. Wiring

### Buttons (×3)

```
    3.3 V  ──── internal pull-up (enabled in firmware)
                        │
     PC0 / PC1 / PC2  ──┤
                        │
                     [button]
                        │
                       GND
```

Active low: the pin idles high through the internal pull-up and is pulled to
ground when pressed. No external resistor is required — the firmware enables
the internal one. Without any pull-up the pin would float and read noise.

### LEDs (×4)

```
  PB0 / PB1 / PB2 / PB4 ───[ 220 Ω ]───▶|───── GND
                                       LED
```

Current: (3.3 V − 2.0 V forward drop) ÷ 220 Ω ≈ **6 mA**. Comfortably inside
the STM32's 20 mA per-pin limit, and bright enough to read on a bench.

### Battery voltage divider

This is the circuit the ADC conversion arithmetic in `ecu_config.h` assumes.
Getting it wrong makes every voltage reading wrong by a constant factor.

```
    3.3 V ─────┐
               │
          [ 10 kΩ ]   R1
               │
               ├───────────▶ PA0   (ADC1_IN0)
               │
          [ 2.2 kΩ ]   R2
               │
              GND
```

Transfer function:

```
  V_pin = V_in × R2 ÷ (R1 + R2) = V_in × 2200 ÷ 12200
  V_in  = V_pin × 12200 ÷ 2200
```

At full scale, 3.3 V at the pin corresponds to **18.3 V** at the simulated
battery — enough headroom for the whole automotive range, from a 9 V cranking
dip to a 15 V charging voltage.

**For bench testing**, replace the fixed divider with a potentiometer so the
voltage can be swept by hand to provoke the under- and overvoltage faults:

```
    3.3 V ────[ 10 kΩ pot ]──── GND
                    │
                  wiper ─────▶ PA0
```

The full sweep maps to 0 → 18.3 V reported. Turn it down slowly and watch the
status LED switch to its 5 Hz fault flash as the undervoltage fault matures.

> **Never connect a real 12 V battery to the ADC pin.** The divider must be
> in place first. 12 V applied directly to a 3.3 V input destroys the pin, and
> usually the microcontroller with it.

### CAN transceiver

```
  STM32                    MCP2551 / SN65HVD230
  ─────                    ────────────────────
  PB9  (CAN1_TX) ────────▶ TXD
  PB8  (CAN1_RX) ◀──────── RXD
  3.3 V or 5 V   ────────▶ VCC     (check your module's rating)
  GND            ────────▶ GND
                           CANH ───┬──── to bus
                           CANL ───┼──── to bus
                                   │
                                [120 Ω]   only at the two physical bus ends
```

**Supply voltage matters.** The MCP2551 is a 5 V part; its RXD output can
exceed the STM32's 3.3 V tolerance on pins that are not 5 V-tolerant. The
SN65HVD230 is a 3.3 V part and connects directly, which is why it is the
easier choice. If you use an MCP2551, check the datasheet for your specific
module and add level shifting if needed.

## 4. Serial console

USART2 is routed to the ST-Link's virtual COM port, so logs arrive over the
same USB cable used for flashing — no second adapter needed.

| Setting | Value |
|---|---|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |

Find the port in Windows Device Manager under *Ports (COM & LPT)*, listed as
**STMicroelectronics STLink Virtual COM Port**. On Linux it is `/dev/ttyACM0`.

Expected output at reset:

```
===========================================
  Automotive Body Control ECU
[          8][INFO ] Firmware v0.2.0  target STM32F446RE
[          9][INFO ] System clock 84000000 Hz
[         10][INFO ] Reset cause: power-on or manual reset
===========================================
[         14][INFO ] Diagnostics ready - UDS on 0x7E0/0x7E8, NVM 0% used
[         15][INFO ] Body control initialised - 5 tasks registered
[       1420][INFO ] State: OFF -> ACC  (event IGN_SHORT)
[       2110][INFO ] State: ACC -> ON  (event IGN_SHORT)
[       3350][INFO ] State: ON -> RUN  (event IGN_LONG)
```

## 5. Multi-node setup

For the two-ECU tests, both boards share one bus:

```
   Nucleo #1                      Nucleo #2
   Body ECU                       Cluster ECU
      │                                │
   transceiver                    transceiver
      │                                │
  ────┼────────────────────────────────┼──── CAN_H
   [120 Ω]                         [120 Ω]
  ────┴────────────────────────────────┴──── CAN_L
```

**Both boards must share a common ground.** CAN is differential, but the
transceivers still need a shared reference; without it the common-mode voltage
can drift outside their input range and communication fails intermittently in
ways that look like software bugs.

## 6. Bring-up checklist

Work through this in order. Each step depends on the previous one, so fixing
them in sequence is much faster than debugging the whole bench at once.

1. **Board powers up** — the red LD3 power LED is lit.
2. **Firmware flashes** — STM32CubeIDE reports success, no ST-Link error.
3. **Serial console works** — the banner appears at reset. If not, check the
   COM port number and the 115200 baud rate before suspecting the firmware.
4. **Status LED flashes at 1 Hz** — the ECU is alive and in OFF. If it is
   dark or steady, the main loop is not running.
5. **Ignition works** — pressing B1 logs `State: OFF -> ACC`.
6. **Long press works** — from ON, holding B1 for over a second logs
   `State: ON -> RUN`.
7. **Lamps respond** — in ON or RUN, the headlight LED lights and the brake
   light sits at a visible dim level.
8. **Indicators flash** — holding a turn button flashes the lamp at 1.5 Hz.
9. **Battery reading is plausible** — sweep the potentiometer and watch
   `battery=...mV` at DEBUG log level.
10. **Faults mature** — turn the potentiometer below the undervoltage
    threshold and confirm the fault latches only after ~1.5 s (3 evaluations
    at 500 ms), not immediately.
11. **CAN transmits** — with an adapter attached,
    `python run_tests.py --interface slcan --channel COM5` reports passes.

## 7. Diagnostics (v0.2)

With a USB-CAN adapter on the bus, the ECU can be diagnosed like a production
module. The same commands work against the SIL target (`--sil`) with no
hardware at all.

```
cd tools/can_tester
python diag_tool.py --interface slcan --channel COM5 info        # identification + live data
python diag_tool.py --interface slcan --channel COM5 dtc         # stored DTCs with freeze frames
python diag_tool.py --interface slcan --channel COM5 lamp-test   # all lamps on for 3 s
python run_diag_tests.py --interface slcan --channel COM5        # the 24 diagnostic system tests
```

**Seeing a real DTC.** Turn the battery potentiometer below ~11 V for a few
seconds, then back up. The status LED flashes at 5 Hz while the fault is
active; afterwards `dtc` shows `P0562-00` with status `0x2E` (not failing now,
but confirmed) and a freeze frame holding the voltage it dropped to. Press the
reset button: the DTC is still there, because it is stored in flash.

**First boot after flashing v0.2** takes up to ~2 s longer than usual while the
NVM sectors are prepared (erased). That happens before the watchdog starts, and
only when the sectors need it.

## 8. Troubleshooting

| Symptom | Most likely cause |
|---|---|
| No serial output | Wrong COM port, or wrong baud rate |
| Status LED dark | Firmware not running, or the main loop hung |
| Status LED flashes fast (5 Hz) | A fault is latched — read the log |
| Board resets repeatedly | Watchdog timeout: a task is hanging |
| Button does nothing | Wired to the wrong pin, or missing ground connection |
| Voltage reading always 0 | ADC pin not connected, or the divider is open |
| Voltage reading always full scale | Divider shorted, or ADC pin tied to 3.3 V |
| No CAN frames | Transceiver unpowered, TX/RX swapped, or missing termination |
| CAN works briefly then stops | Bus-off from a bit rate mismatch, or a missing second node to acknowledge frames |
| Diagnostic tester times out | Adapter not at 500 kbit/s, or tester using 0x7E0/0x7E8 swapped |
| `7F 27 37` on every seed request | Security lockout after 3 wrong keys - wait 10 s |
| `7F xx 7F` | Service needs the extended session: send `10 03` first |
| DTCs lost after reset | NVM failed to initialise - look for "NVM unavailable" in the boot log |

**The single most common CAN mistake:** a CAN transmitter needs at least one
other node to acknowledge each frame. A single node alone on a bus will
retransmit, accumulate errors, and eventually go bus-off — even though the
wiring is perfect. Always have a second node or an adapter listening.
