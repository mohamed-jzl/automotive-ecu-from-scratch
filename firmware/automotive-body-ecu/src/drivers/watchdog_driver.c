/**
 * @file    watchdog_driver.c
 * @brief   IWDG implementation - see watchdog_driver.h for the rationale.
 */

#include "watchdog_driver.h"

#include "ecu_config.h"
#include "stm32f4xx_hal.h"

static IWDG_HandleTypeDef s_iwdg;
static bool               s_initialised = false;

bool WatchdogDriver_Init(void)
{
    s_iwdg.Instance = IWDG;

    /* Timeout derivation (see ecu_config.h for the full reasoning):
     *
     *   LSI          ~= 32 kHz
     *   Prescaler     = 32     -> counter ticks at 1 kHz, i.e. 1 ms per tick
     *   Reload        = 500    -> timeout = 500 ticks * 1 ms = 500 ms
     *
     * The reload register is 12 bits wide, so 500 is well within range
     * (maximum 4095, which at this prescaler would be a ~4 s timeout).
     */
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_32;
    s_iwdg.Init.Reload    = ECU_WATCHDOG_RELOAD;

    /* HAL_IWDG_Init both configures and *starts* the watchdog. From this
     * point the ECU must refresh it within the timeout or be reset. */
    if (HAL_IWDG_Init(&s_iwdg) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    s_initialised = true;
    return true;
}

void WatchdogDriver_Refresh(void)
{
    if (!s_initialised)
    {
        return;
    }

    (void)HAL_IWDG_Refresh(&s_iwdg);
}

bool WatchdogDriver_WasResetByWatchdog(void)
{
    return (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET);
}

void WatchdogDriver_ClearResetFlags(void)
{
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
