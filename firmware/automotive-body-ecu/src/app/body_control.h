/**
 * @file    body_control.h
 * @brief   Application layer - the ECU's actual behaviour.
 *
 * This is the only module that knows what the vehicle is supposed to *do*.
 * It reads debounced inputs, turns them into state machine events, evaluates
 * fault conditions, decides what every lamp should be doing, and publishes
 * the result on CAN.
 *
 * It calls services and drivers. Nothing calls it except main().
 *
 * Task structure
 * --------------
 * Behaviour is split across four periodic tasks, registered with the
 * scheduler at startup. Each period is chosen from the physics of the signal
 * it handles, not from convenience:
 *
 *      10 ms   Input sampling and the state machine.
 *              Fast enough that a human never perceives lag (the threshold is
 *              around 100 ms), and it sets the debounce time base.
 *
 *      50 ms   Output actuation.
 *              Indicator flashing needs a 333 ms half-period; 50 ms resolves
 *              that to within 15%, and lamps have no faster requirement.
 *
 *     100 ms   CAN transmission and reception.
 *              The conventional period for vehicle status broadcasts.
 *
 *     500 ms   Battery measurement and diagnostics.
 *              Battery voltage changes over seconds. Sampling it faster would
 *              burn CPU time and add noise without adding information.
 *
 * Choosing a period is an engineering decision with a justification, not a
 * round number someone liked.
 */

#ifndef APP_BODY_CONTROL_H
#define APP_BODY_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise every layer of the ECU and register the periodic tasks.
 *
 * Order matters and is enforced here:
 *   1. Logger first, so every later failure can be reported.
 *   2. Drivers, so the hardware is in a defined state.
 *   3. Services, which depend on the drivers.
 *   4. Application state.
 *   5. Scheduler tasks last - nothing runs until everything exists.
 *
 * @param  was_watchdog_reset  Reset cause, read by main() before anything could
 *                             clear it. Printed in the startup banner, which
 *                             comes first in the log so every later line is
 *                             read in the context of how the ECU started.
 * @return true if every subsystem initialised successfully. A false return
 *         means the ECU is running degraded; the log identifies which
 *         subsystem failed.
 */
bool BodyControl_Init(bool was_watchdog_reset);

/**
 * @brief Report whether initialisation completed without any subsystem failing.
 */
bool BodyControl_IsHealthy(void);

/**
 * @brief Latest battery measurement in millivolts (DID 0x0100, DTC snapshots).
 */
uint32_t BodyControl_GetBatteryMv(void);

#endif /* APP_BODY_CONTROL_H */
