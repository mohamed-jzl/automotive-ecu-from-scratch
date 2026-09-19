/**
 * @file    mcu_driver.c
 * @brief   Reset and unique ID - see mcu_driver.h.
 */

#include "mcu_driver.h"

#include "stm32f4xx_hal.h"

void McuDriver_SystemReset(void)
{
    NVIC_SystemReset();
}

void McuDriver_ReadUniqueId(uint8_t out[MCU_UNIQUE_ID_LENGTH])
{
    /* UID_BASE (0x1FFF7A10 on the STM32F4) is a read-only system memory area.
     * It is read byte by byte through a volatile pointer so the compiler never
     * assumes it can cache or combine the accesses. */
    const volatile uint8_t *uid = (const volatile uint8_t *)UID_BASE;

    for (uint8_t i = 0U; i < MCU_UNIQUE_ID_LENGTH; i++)
    {
        out[i] = uid[i];
    }
}
