/**
 * @file    ecu_config.h
 * @brief   Compile-time configuration for the Body Control ECU.
 *
 * Every tunable number in the firmware lives here. No magic constants are
 * allowed anywhere else in the codebase - if a value needs changing, it is
 * changed once, in this file, and the change is visible in code review.
 *
 * All peripheral arithmetic below assumes the clock tree configured by
 * SystemClock_Config() in Core/Src/main.c:
 *
 *     HSE 8 MHz -> PLL -> SYSCLK = HCLK = 84 MHz
 *     APB1 = 42 MHz   (CAN1, USART2)     APB1 timer clock = 84 MHz (TIM3)
 *     APB2 = 84 MHz   (ADC1)
 *
 * If you ever change the clock configuration in CubeMX, revisit every
 * "derivation" comment in this file.
 */

#ifndef CONFIG_ECU_CONFIG_H
#define CONFIG_ECU_CONFIG_H

#include <stdint.h>

/* ========================================================================= */
/* Identity                                                                  */
/* ========================================================================= */

#define ECU_FW_VERSION_MAJOR            0U
#define ECU_FW_VERSION_MINOR            1U
#define ECU_FW_VERSION_PATCH            0U

#define ECU_NAME                        "BodyControlECU"

/* ========================================================================= */
/* Clock tree (mirrored here so drivers can do timing maths)                 */
/* ========================================================================= */

#define ECU_SYSCLK_HZ                   84000000UL
#define ECU_APB1_HZ                     42000000UL  /* CAN1, USART2 */
#define ECU_APB1_TIMER_HZ               84000000UL  /* TIM2..TIM7   */
#define ECU_APB2_HZ                     84000000UL  /* ADC1         */

/* ========================================================================= */
/* Scheduler - cooperative time-triggered task periods                       */
/* ========================================================================= */

/* The main loop runs these four tasks at fixed periods. Periods are chosen
 * as multiples of each other so the schedule is easy to reason about and
 * jitter stays bounded. */
#define ECU_TASK_PERIOD_10MS            10U    /* input sampling + state machine */
#define ECU_TASK_PERIOD_50MS            50U    /* output actuation (indicators)  */
#define ECU_TASK_PERIOD_100MS           100U   /* CAN vehicle status transmit    */
#define ECU_TASK_PERIOD_500MS           500U   /* battery monitor + diagnostics  */

/* A task that takes longer than this is treated as a scheduling fault.
 * Real ECUs monitor this to prove worst-case execution time assumptions. */
#define ECU_TASK_OVERRUN_LIMIT_MS       5U

/* ========================================================================= */
/* Digital input conditioning                                                */
/* ========================================================================= */

/* A mechanical contact bounces for roughly 1-20 ms. We sample every 10 ms and
 * require N consecutive identical samples before accepting a new level.
 * 3 samples x 10 ms = 30 ms of stability required. */
#define ECU_INPUT_DEBOUNCE_SAMPLES      3U

/* Holding the ignition button for this long is interpreted as "crank the
 * engine" rather than "advance the ignition position". */
#define ECU_IGNITION_LONG_PRESS_MS      1000U

/* ========================================================================= */
/* Turn indicators                                                           */
/* ========================================================================= */

/* UNECE Regulation 6 requires automotive turn indicators to flash between
 * 60 and 120 cycles per minute. 1.5 Hz (333 ms on, 333 ms off) sits
 * comfortably in the middle at 90 cycles/min. */
#define ECU_INDICATOR_HALF_PERIOD_MS    333U

/* ========================================================================= */
/* ADC - battery voltage measurement                                         */
/* ========================================================================= */

/* STM32F446 ADC is a 12-bit successive-approximation converter. */
#define ECU_ADC_RESOLUTION_BITS         12U
#define ECU_ADC_MAX_COUNT               4095U   /* 2^12 - 1 */

/* Analog reference. On the Nucleo, VREF+ is tied to the 3.3 V rail. */
#define ECU_ADC_VREF_MV                 3300U

/* External resistor divider that scales battery voltage into the 0-3.3 V
 * ADC input range:
 *
 *      VBAT o---[ R1 = 10k ]---+---[ R2 = 2.2k ]---o GND
 *                              |
 *                              +---> PA0 (ADC1_IN0)
 *
 *   V_adc = V_bat * R2 / (R1 + R2) = V_bat * 2200 / 12200
 *   V_bat = V_adc * (R1 + R2) / R2 = V_adc * 12200 / 2200
 *
 * Full scale: 3.3 V at the pin corresponds to 18.3 V at the battery, which
 * covers the whole automotive range (9 V cranking to 15 V charging) with
 * margin for load-dump clamping.
 */
#define ECU_ADC_DIVIDER_R1_OHM          10000UL
#define ECU_ADC_DIVIDER_R2_OHM          2200UL

/* Number of raw samples averaged per reading. Averaging 8 samples of white
 * noise improves the effective signal-to-noise ratio by sqrt(8) ~ 2.8x. */
#define ECU_ADC_OVERSAMPLE_COUNT        8U

/* ========================================================================= */
/* Battery voltage thresholds (millivolts at the battery, not at the pin)    */
/* ========================================================================= */

#define ECU_BATT_UNDERVOLTAGE_MV        11000U  /* below this, cranking is unsafe */
#define ECU_BATT_OVERVOLTAGE_MV         15500U  /* above this, the alternator regulator has failed */
#define ECU_BATT_NOMINAL_MV             12600U  /* healthy resting voltage        */

/* Hysteresis prevents a voltage hovering exactly on a threshold from
 * setting and clearing a fault repeatedly (fault chattering). */
#define ECU_BATT_HYSTERESIS_MV          300U

/* ========================================================================= */
/* Fault management                                                          */
/* ========================================================================= */

/* A fault condition must persist for this many consecutive evaluations
 * before the fault latches. This is called fault maturation and it prevents
 * a single noisy reading from putting the vehicle into limp mode. */
#define ECU_FAULT_MATURATION_COUNT      3U

/* Once healed, the condition must stay healthy this many cycles before the
 * fault clears. Asymmetric maturation (slow to clear) is standard practice. */
#define ECU_FAULT_HEALING_COUNT         5U

/* ========================================================================= */
/* PWM - brake light                                                         */
/* ========================================================================= */

/* 1 kHz is above the ~100 Hz flicker fusion threshold of human vision, and
 * low enough that switching losses in a real MOSFET driver stay negligible.
 *
 * Derivation with TIM3 clocked at ECU_APB1_TIMER_HZ = 84 MHz:
 *
 *   counter clock = 84 MHz / (PSC + 1) = 84 MHz / 84 = 1 MHz
 *   PWM frequency = 1 MHz / (ARR + 1)  = 1 MHz / 1000 = 1 kHz
 *
 * ARR = 999 also gives exactly 1000 duty steps, so duty percent maps 1:1
 * onto the compare register with no rounding.
 */
#define ECU_PWM_FREQUENCY_HZ            1000U
#define ECU_PWM_PRESCALER               (84U - 1U)
#define ECU_PWM_PERIOD_TICKS            (1000U - 1U)
#define ECU_PWM_DUTY_MAX                1000U

/* Brake light intensity levels, in duty ticks (0 .. ECU_PWM_DUTY_MAX). */
#define ECU_PWM_BRAKE_OFF               0U
#define ECU_PWM_BRAKE_TAIL_LIGHT        200U    /* 20% - running light         */
#define ECU_PWM_BRAKE_FULL              1000U   /* 100% - brake pedal pressed  */

/* ========================================================================= */
/* UART - debug console                                                      */
/* ========================================================================= */

#define ECU_UART_BAUDRATE               115200UL

/* Maximum length of one formatted log line, including the trailing newline.
 * Lines longer than this are truncated rather than overflowing the stack. */
#define ECU_LOG_LINE_MAX                128U

/* Severity levels, defined here as plain integers so that the compile-time
 * verdict below does not depend on logger.h. The LogLevel_t enum in logger.h
 * takes its values from these macros, so the two can never drift apart. */
#define ECU_LOG_LEVEL_DEBUG             0U
#define ECU_LOG_LEVEL_INFO              1U
#define ECU_LOG_LEVEL_WARN              2U
#define ECU_LOG_LEVEL_ERROR             3U
#define ECU_LOG_LEVEL_NONE              4U

/* Compile-time log verbosity. Calls below this level are replaced by the
 * preprocessor with nothing at all, so they cost zero flash and zero cycles
 * in the built image - the format string is not even linked in.
 *
 * Set to ECU_LOG_LEVEL_DEBUG while bringing up new hardware, and to
 * ECU_LOG_LEVEL_WARN or higher for a release build. */
#define ECU_LOG_LEVEL_COMPILED          ECU_LOG_LEVEL_INFO

/* ========================================================================= */
/* CAN bus                                                                   */
/* ========================================================================= */

/* 500 kbit/s is the most common body/comfort-domain CAN rate in production
 * vehicles.
 *
 * Derivation with CAN1 clocked from APB1 at 42 MHz:
 *
 *   nominal bit time = SYNC_SEG + BS1 + BS2 = 1 + 11 + 2 = 14 time quanta
 *   bit rate = APB1 / (Prescaler * quanta) = 42e6 / (6 * 14) = 500000 bit/s
 *
 *   sample point = (SYNC_SEG + BS1) / quanta = 12 / 14 = 85.7%
 *
 * A sample point between 75% and 90% is what CiA 301 recommends: late enough
 * for the signal to settle across a long bus, early enough to leave
 * resynchronisation margin.
 */
#define ECU_CAN_BITRATE_BPS             500000UL
#define ECU_CAN_PRESCALER               6U
#define ECU_CAN_SJW_TQ                  1U
#define ECU_CAN_BS1_TQ                  11U
#define ECU_CAN_BS2_TQ                  2U

/* Transmit mailbox wait. The bxCAN peripheral has 3 TX mailboxes; if all are
 * full this long, the bus is not accepting frames and we raise a fault. */
#define ECU_CAN_TX_TIMEOUT_MS           10U

/* If no frame is received from the powertrain ECU within this window, its
 * data is considered stale and must not be used for any decision.
 * Rule of thumb: 3 x the sender's transmit period. */
#define ECU_CAN_RX_TIMEOUT_MS           300U

/* ========================================================================= */
/* Watchdog                                                                  */
/* ========================================================================= */

/* The independent watchdog runs from the ~32 kHz LSI oscillator, in its own
 * clock domain, so it keeps counting even if the main PLL fails.
 *
 *   tick period = 32 kHz / prescaler = 32000 / 32 = 1000 Hz = 1 ms
 *   timeout     = reload * tick      = 500 * 1 ms = 500 ms
 *
 * 500 ms is comfortably longer than our slowest task period (100 ms) but
 * short enough that a hung ECU recovers before a driver notices.
 *
 * Note: the LSI is an RC oscillator with +/-50% tolerance across temperature,
 * so the real timeout lies between roughly 330 ms and 1 s. Safety-critical
 * designs measure the LSI against a precise clock and compensate; we document
 * the tolerance instead, which is the honest educational equivalent.
 */
#define ECU_WATCHDOG_TIMEOUT_MS         500U
#define ECU_WATCHDOG_LSI_HZ             32000UL
#define ECU_WATCHDOG_PRESCALER_DIV      32UL
#define ECU_WATCHDOG_RELOAD             ((ECU_WATCHDOG_TIMEOUT_MS * ECU_WATCHDOG_LSI_HZ) \
                                         / (ECU_WATCHDOG_PRESCALER_DIV * 1000UL))

#endif /* CONFIG_ECU_CONFIG_H */
