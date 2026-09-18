/**
 * @file    fault_manager.c
 * @brief   Fault maturation implementation - see fault_manager.h for theory.
 *
 * Pure module: no HAL, no hardware, no time source. Every decision depends
 * only on the sequence of calls made to FaultManager_Update(), which is
 * exactly what makes the whole file testable on a host PC.
 */

#include "fault_manager.h"

#include "ecu_config.h"

/**
 * @brief Filter state for one fault.
 */
typedef struct
{
    bool    is_active;          /**< Latched verdict reported to the rest of the ECU */
    uint8_t maturation_count;   /**< Consecutive evaluations with the condition present */
    uint8_t healing_count;      /**< Consecutive evaluations with the condition absent  */
} FaultState_t;

static FaultState_t s_faults[FAULT_COUNT];

/**
 * @brief Diagnostic names, indexed by FaultId_t.
 *
 * Kept short so a log line stays within one terminal width. Designated
 * initialisers tie each string to its enum value, so reordering the enum
 * cannot silently mislabel a fault.
 */
static const char *const FAULT_NAMES[FAULT_COUNT] =
{
    [FAULT_BATTERY_UNDERVOLTAGE] = "BATT_UNDERVOLT",
    [FAULT_BATTERY_OVERVOLTAGE]  = "BATT_OVERVOLT",
    [FAULT_ADC_FAILURE]          = "ADC_FAILURE",
    [FAULT_CAN_BUS_OFF]          = "CAN_BUS_OFF",
    [FAULT_CAN_TX_FAILURE]       = "CAN_TX_FAIL",
    [FAULT_TASK_OVERRUN]         = "TASK_OVERRUN"
};

void FaultManager_Init(void)
{
    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; i++)
    {
        s_faults[i].is_active        = false;
        s_faults[i].maturation_count = 0U;
        s_faults[i].healing_count    = 0U;
    }
}

bool FaultManager_Update(FaultId_t id, bool condition_present)
{
    if (id >= FAULT_COUNT)
    {
        return false;
    }

    FaultState_t *fault        = &s_faults[id];
    const bool    was_active   = fault->is_active;

    if (condition_present)
    {
        /* The condition is present, so any progress toward healing is void.
         * Resetting rather than decrementing means a fault that flickers
         * back on has to serve its full healing period again from zero. */
        fault->healing_count = 0U;

        if (!fault->is_active)
        {
            fault->maturation_count++;

            if (fault->maturation_count >= ECU_FAULT_MATURATION_COUNT)
            {
                fault->is_active        = true;
                fault->maturation_count = 0U;
            }
        }
    }
    else
    {
        /* Symmetrically: one clean evaluation voids progress toward latching. */
        fault->maturation_count = 0U;

        if (fault->is_active)
        {
            fault->healing_count++;

            if (fault->healing_count >= ECU_FAULT_HEALING_COUNT)
            {
                fault->is_active     = false;
                fault->healing_count = 0U;
            }
        }
    }

    return (fault->is_active != was_active);
}

bool FaultManager_IsActive(FaultId_t id)
{
    if (id >= FAULT_COUNT)
    {
        return false;
    }
    return s_faults[id].is_active;
}

bool FaultManager_IsAnyActive(void)
{
    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; i++)
    {
        if (s_faults[i].is_active)
        {
            return true;
        }
    }
    return false;
}

uint8_t FaultManager_GetActiveCount(void)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; i++)
    {
        if (s_faults[i].is_active)
        {
            count++;
        }
    }
    return count;
}

uint16_t FaultManager_GetActiveBitmask(void)
{
    uint16_t mask = 0U;

    for (uint8_t i = 0U; i < (uint8_t)FAULT_COUNT; i++)
    {
        if (s_faults[i].is_active)
        {
            mask |= (uint16_t)(1U << i);
        }
    }
    return mask;
}

const char *FaultManager_GetName(FaultId_t id)
{
    if (id >= FAULT_COUNT)
    {
        return "UNKNOWN";
    }
    return FAULT_NAMES[id];
}

void FaultManager_ClearAll(void)
{
    FaultManager_Init();
}
