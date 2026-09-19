/**
 * @file    mcu_driver.h
 * @brief   Microcontroller-level services: software reset and unique device ID.
 */

#ifndef DRIVERS_MCU_DRIVER_H
#define DRIVERS_MCU_DRIVER_H

#include <stdint.h>

/** Size of the STM32 factory-programmed unique device identifier. */
#define MCU_UNIQUE_ID_LENGTH    12U

/**
 * @brief Reset the whole microcontroller, exactly as the reset pin would.
 *
 * Implemented with the Cortex-M System Control Block (NVIC_SystemReset), so
 * it works identically on any Cortex-M part. Never returns.
 */
void McuDriver_SystemReset(void);

/**
 * @brief Read the 96-bit unique ID burned into every STM32 at the factory.
 *
 * No two chips share it, which makes it a ready-made ECU serial number
 * (DID 0xF18C) and a good source of per-device entropy for the security seed.
 */
void McuDriver_ReadUniqueId(uint8_t out[MCU_UNIQUE_ID_LENGTH]);

#endif /* DRIVERS_MCU_DRIVER_H */
