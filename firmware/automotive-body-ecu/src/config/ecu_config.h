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
#define ECU_FW_VERSION_MINOR            2U
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

/* The main loop runs these tasks at fixed periods. Periods are chosen
 * as multiples of each other so the schedule is easy to reason about and
 * jitter stays bounded. */
#define ECU_TASK_PERIOD_5MS             5U     /* CAN reception + diagnostics    */
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

/* ========================================================================= */
/* CAN reception buffer                                                      */
/* ========================================================================= */

/* Frames are moved out of the 3-message hardware FIFO by an interrupt and
 * queued here until the 5 ms task processes them. 32 frames is ~2 ms of a
 * fully loaded 500 kbit/s bus - far more than a diagnostic tester can produce
 * given the STmin we ask it to respect. Must be a power of two. */
#define ECU_CAN_RX_QUEUE_SIZE           32U

/* ========================================================================= */
/* Diagnostics - addressing (ISO 15765-4 conventions for 11-bit IDs)         */
/* ========================================================================= */

/* Physical request: sent by the tester to THIS ECU only.
 * Functional request: broadcast to every ECU on the bus at once.
 * Response: always sent from this ECU's own response identifier.
 * 0x7E0/0x7E8 is the pair conventionally used by the first ECU on a bus. */
#define ECU_DIAG_CAN_ID_PHYSICAL        0x7E0U
#define ECU_DIAG_CAN_ID_FUNCTIONAL      0x7DFU
#define ECU_DIAG_CAN_ID_RESPONSE        0x7E8U

/* ========================================================================= */
/* ISO-TP (ISO 15765-2) - transport of UDS messages longer than 7 bytes      */
/* ========================================================================= */

/* Largest UDS message we accept or send. ISO-TP allows up to 4095 bytes;
 * our longest real message (a full DTC report) is under 40. Requests longer
 * than this are refused with a flow-control OVERFLOW frame, not truncated. */
#define ECU_ISOTP_BUFFER_SIZE           256U

/* Unused bytes of a frame are filled with this value. Always sending 8-byte
 * frames keeps the bit length constant, which simplifies bus-load analysis;
 * 0xCC is a common choice because its alternating bits limit bit stuffing. */
#define ECU_ISOTP_PADDING_BYTE          0xCCU

/* Parameters we send in our Flow Control frame when receiving a long request:
 *   Block size 0 = "send everything, no further flow control needed".
 *   STmin 5 ms   = minimum gap the tester must leave between consecutive
 *                  frames, so our 5 ms task can keep up without the receive
 *                  queue overflowing. */
#define ECU_ISOTP_BLOCK_SIZE            0U
#define ECU_ISOTP_ST_MIN_MS             5U

/* Protocol timeouts (ISO 15765-2 section 9.8). If the next expected frame does
 * not arrive in time, the transfer is abandoned rather than waited on forever.
 *   N_Cr: waiting for the next Consecutive Frame when receiving.
 *   N_Bs: waiting for a Flow Control frame when sending. */
#define ECU_ISOTP_TIMEOUT_N_CR_MS       1000U
#define ECU_ISOTP_TIMEOUT_N_BS_MS       1000U

/* A receiver may answer "WAIT" instead of "continue". Limiting how many WAITs
 * we accept stops a faulty tester from stalling our transmitter forever. */
#define ECU_ISOTP_MAX_WAIT_FRAMES       10U

/* ========================================================================= */
/* UDS (ISO 14229-1) - timing                                                */
/* ========================================================================= */

/* P2server: maximum time between receiving a request and starting the reply.
 * P2*server: extended limit after a "response pending" (NRC 0x78).
 * Both values are reported to the tester in the DiagnosticSessionControl
 * response, so the tester knows how long to wait. */
#define ECU_UDS_P2_SERVER_MS            50U
#define ECU_UDS_P2_STAR_SERVER_MS       5000U

/* S3server: a non-default session falls back to the default session if no
 * request arrives within this window. This is why testers send
 * TesterPresent (0x3E) periodically: an abandoned tester must never leave an
 * ECU unlocked in an extended session. */
#define ECU_UDS_S3_SERVER_MS            5000U

/* ========================================================================= */
/* UDS - security access (service 0x27)                                      */
/* ========================================================================= */

/* After this many wrong keys the ECU refuses new seeds for a time delay.
 * Without it, a 32-bit key could be brute-forced by trying keys in a loop. */
#define ECU_UDS_SECURITY_MAX_ATTEMPTS   3U
#define ECU_UDS_SECURITY_LOCKOUT_MS     10000U

/* ========================================================================= */
/* DTC management                                                            */
/* ========================================================================= */

/* A confirmed DTC that does not fail again for this many operation cycles
 * (ignition cycles here) is considered healed and is removed from memory.
 * This is called DTC aging. 40 is a typical OEM value. */
#define ECU_DTC_AGING_THRESHOLD         40U

/* Maximum number of DTCs the manager can hold. */
#define ECU_DTC_MAX_COUNT               16U

/* ========================================================================= */
/* Non-volatile memory (emulated EEPROM in flash)                            */
/* ========================================================================= */

/* Two 128 KB flash sectors used in ping-pong, excluded from the code region by
 * the linker script. Flash can only be erased a whole sector at a time, and
 * erasing 128 KB blocks the CPU for 1-2 seconds - longer than the watchdog
 * timeout. The store is designed so that erasing only ever happens at startup,
 * before the watchdog is armed. See nvm_store.h for the full reasoning. */
#define ECU_NVM_REGION_A_ADDRESS        0x08040000UL   /* sector 6 */
#define ECU_NVM_REGION_B_ADDRESS        0x08060000UL   /* sector 7 */
#define ECU_NVM_REGION_SIZE             (128UL * 1024UL)

/* At startup, if the active region is fuller than this, the latest record is
 * migrated to the other region and the old one is erased. */
#define ECU_NVM_COMPACT_THRESHOLD_PCT   75U

/* Minimum interval between two flash writes. Flash cells survive about 10 000
 * erase cycles; rate-limiting protects them from a fault that flickers. */
#define ECU_NVM_MIN_WRITE_INTERVAL_MS   1000U

/* ========================================================================= */
/* Diagnostic routines                                                       */
/* ========================================================================= */

/* Duration of the lamp self-test started through RoutineControl (0x31). */
#define ECU_LAMP_TEST_DURATION_MS       3000U

#endif /* CONFIG_ECU_CONFIG_H */
