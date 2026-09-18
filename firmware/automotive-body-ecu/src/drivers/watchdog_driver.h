/**
 * @file    watchdog_driver.h
 * @brief   Independent watchdog (IWDG) - last-resort recovery from a hung ECU.
 *
 * What problem this solves
 * ------------------------
 * Software can stop making progress without crashing: an infinite loop, a
 * deadlock, a corrupted pointer that lands in a tight branch. From the
 * outside the CPU looks alive - it is executing instructions - but the
 * vehicle function it controls is frozen. No amount of defensive C detects
 * this from *inside* the program, because the code that would detect it is
 * the code that stopped running.
 *
 * The watchdog is a hardware timer that resets the MCU unless software
 * periodically refreshes it. Healthy firmware refreshes it on schedule; hung
 * firmware does not, and the chip reboots into a known-good state.
 *
 * Why the *independent* watchdog specifically
 * -------------------------------------------
 * The IWDG is clocked from the LSI RC oscillator, in a completely separate
 * clock domain from the CPU. If the main PLL fails or the external crystal
 * stops, the CPU halts but the IWDG keeps counting and still forces the
 * reset. A watchdog sharing the system clock would freeze alongside the
 * thing it is supposed to be watching.
 *
 * Once started, the IWDG cannot be stopped by software - only by a reset.
 * That is deliberate: a fault must not be able to disable its own safety net.
 *
 * Where the refresh belongs
 * -------------------------
 * The refresh call goes in exactly one place: the main loop, *after* the
 * scheduler has serviced its due tasks. Refreshing from a timer interrupt
 * would defeat the mechanism entirely - the interrupt keeps firing even when
 * the main loop is stuck, so the watchdog would happily pet a dead ECU.
 * This is the single most common way watchdogs are rendered useless in
 * practice.
 */

#ifndef DRIVERS_WATCHDOG_DRIVER_H
#define DRIVERS_WATCHDOG_DRIVER_H

#include <stdbool.h>

/**
 * @brief Start the independent watchdog with the configured timeout.
 *
 * Timeout is ECU_WATCHDOG_TIMEOUT_MS nominal. The LSI oscillator is an
 * untrimmed RC oscillator specified at 32 kHz +/-50% across the full
 * temperature range, so the real timeout lies roughly between 0.66x and 2x
 * the nominal value. The refresh period must therefore be comfortably
 * shorter than the *shortest* possible timeout, not the nominal one.
 *
 * Cannot be undone: once this returns true, the watchdog runs until reset.
 *
 * @return true on success, false if the HAL rejected the configuration.
 */
bool WatchdogDriver_Init(void);

/**
 * @brief Refresh the watchdog counter ("kick" / "pet" the watchdog).
 *
 * Call once per main-loop iteration, from the main loop only.
 */
void WatchdogDriver_Refresh(void);

/**
 * @brief Report whether the previous reset was caused by the watchdog.
 *
 * Must be called early in startup, before WatchdogDriver_ClearResetFlags().
 * A watchdog reset is strong evidence of a software hang and is worth
 * logging and counting - in a production ECU it would be stored in
 * non-volatile memory as a diagnostic trouble code.
 *
 * @return true if the last reset came from an IWDG timeout.
 */
bool WatchdogDriver_WasResetByWatchdog(void);

/**
 * @brief Clear the RCC reset-cause flags.
 *
 * These flags are sticky across resets and are only cleared by software, so
 * failing to clear them makes every subsequent boot look like a watchdog
 * reset. Call once, after reading the cause.
 */
void WatchdogDriver_ClearResetFlags(void);

#endif /* DRIVERS_WATCHDOG_DRIVER_H */
