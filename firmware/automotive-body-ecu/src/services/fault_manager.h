/**
 * @file    fault_manager.h
 * @brief   Fault detection, maturation, healing and fail-safe arbitration.
 *
 * Detection is not the hard part
 * ------------------------------
 * Comparing a voltage against a threshold is trivial. The hard part is
 * deciding *when to believe it*. A single reading below 11 V might be a real
 * flat battery, or it might be the voltage dip while the starter motor
 * engages, or a noise spike coupled in from an injector. Reacting to every
 * such sample would put the vehicle in limp mode several times per journey.
 *
 * Maturation and healing
 * ----------------------
 * Every fault is filtered by two asymmetric counters:
 *
 *      condition present  -> maturation counter++ ; at N, the fault LATCHES
 *      condition absent   -> healing counter++    ; at M, the fault CLEARS
 *
 * with M > N deliberately. A fault is confirmed relatively quickly but
 * released slowly, so a condition oscillating around its threshold settles
 * into the safe interpretation rather than chattering between the two. This
 * asymmetry is standard practice in automotive diagnostics and is what the
 * ISO 14229 (UDS) world calls a DTC maturation strategy.
 *
 *      raw:     __/‾‾\__/‾‾‾‾‾‾‾‾‾‾‾‾‾‾\______/‾\____
 *      latched: _____________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__
 *                            ^ 3 samples       ^ 5 clean samples
 *                              confirmed         before release
 *
 * Purity, and why it matters here
 * -------------------------------
 * This module contains no hardware access, no HAL include, and no global
 * time source. It is a pure state machine over integers, which means the
 * whole of it compiles and runs on a host PC under plain gcc. Its entire
 * behaviour - including the maturation timing that would take minutes to
 * provoke on real hardware - is verified in tests/test_fault_manager.c in
 * microseconds.
 *
 * Keeping diagnostic logic free of I/O is not an academic preference. It is
 * the difference between a module you can test and one you can only hope about.
 */

#ifndef SERVICES_FAULT_MANAGER_H
#define SERVICES_FAULT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Every fault the ECU can detect.
 *
 * The numeric value is the bit position in the fault bitmask that is
 * transmitted over CAN, so these values are part of the network interface:
 * changing one silently changes what a receiving ECU decodes. Append new
 * faults at the end; never renumber existing ones.
 */
typedef enum
{
    FAULT_BATTERY_UNDERVOLTAGE = 0, /**< Supply below ECU_BATT_UNDERVOLTAGE_MV  */
    FAULT_BATTERY_OVERVOLTAGE,      /**< Supply above ECU_BATT_OVERVOLTAGE_MV   */
    FAULT_ADC_FAILURE,              /**< Conversion timed out or never completed */
    FAULT_CAN_BUS_OFF,              /**< Controller disconnected itself from the bus */
    FAULT_CAN_TX_FAILURE,           /**< Frames repeatedly rejected by the mailbox */
    FAULT_TASK_OVERRUN,             /**< A task exceeded its execution budget    */
    FAULT_COUNT                     /**< Number of faults - not a real fault     */
} FaultId_t;

/**
 * @brief Reset all fault state. Call once at startup.
 */
void FaultManager_Init(void);

/**
 * @brief Feed one evaluation of a fault condition into the filter.
 *
 * Call this on a fixed period for every fault, whether the condition is
 * present or not. The maturation counters measure *consecutive evaluations*,
 * so skipping a call when the condition is absent would leave a stale count
 * and let an intermittent fault latch when it should not.
 *
 * @param  id                 Which fault is being evaluated.
 * @param  condition_present  true if the raw condition is detected right now.
 * @return true if the fault's latched state changed as a result of this call.
 *         The caller uses this to log transitions rather than steady state -
 *         which is what keeps the log readable instead of flooding it.
 */
bool FaultManager_Update(FaultId_t id, bool condition_present);

/**
 * @brief Query whether a fault is currently latched.
 *
 * @param  id  Fault to query.
 * @return true if latched, false if clear or @p id is out of range.
 */
bool FaultManager_IsActive(FaultId_t id);

/**
 * @brief Query whether any fault at all is latched.
 *
 * This is what drives the vehicle state machine into its FAULT state.
 *
 * @return true if at least one fault is active.
 */
bool FaultManager_IsAnyActive(void);

/**
 * @brief Number of faults currently latched.
 */
uint8_t FaultManager_GetActiveCount(void);

/**
 * @brief All latched faults as a bitmask, ready for CAN transmission.
 *
 * Bit N corresponds to FaultId_t value N. A bitmask lets a receiving ECU
 * learn the complete diagnostic picture from two payload bytes.
 *
 * @return Bitmask of active faults.
 */
uint16_t FaultManager_GetActiveBitmask(void);

/**
 * @brief Short human-readable name of a fault, for logs and diagnostics.
 *
 * @param  id  Fault to name.
 * @return Static string, or "UNKNOWN" if @p id is out of range. Never NULL.
 */
const char *FaultManager_GetName(FaultId_t id);

/**
 * @brief Force-clear every fault and reset all counters.
 *
 * Equivalent to a diagnostic tester issuing "clear DTCs". Used by the test
 * harness; on a real vehicle this is a serviced action, never automatic.
 */
void FaultManager_ClearAll(void);

#endif /* SERVICES_FAULT_MANAGER_H */
