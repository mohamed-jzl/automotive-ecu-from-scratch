/**
 * @file    flash_driver.h
 * @brief   Erase and program the internal flash sectors reserved for NVM data.
 *
 * Flash rules this driver has to respect
 * --------------------------------------
 *  - The flash is locked after reset. Every write session must unlock it by
 *    writing a key sequence to FLASH->KEYR, and lock it again afterwards so a
 *    wild pointer cannot corrupt it.
 *  - Programming can only change bits from 1 to 0. Only erase sets them back.
 *  - Erase granularity is a whole sector: 128 KB for sectors 6 and 7.
 *  - While flash is being written, any instruction fetch from flash stalls.
 *    The STM32F446 has a single flash bank, so the CPU effectively pauses for
 *    each programmed word (~16 us) and for the whole of a sector erase (1-2 s).
 *
 * The last point is why the NVM store only ever erases at start-up.
 *
 * Voltage range
 * -------------
 * The number of bits programmed per operation depends on the supply voltage.
 * At 2.7-3.6 V (the Nucleo runs at 3.3 V) the flash can be programmed 32 bits
 * at a time, which is what FLASH_VOLTAGE_RANGE_3 selects.
 */

#ifndef DRIVERS_FLASH_DRIVER_H
#define DRIVERS_FLASH_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Erase one flash sector. BLOCKS THE CPU FOR UP TO ~2 SECONDS.
 *
 * @param sector  Sector number: only 6 and 7 are accepted. Refusing every
 *                other sector means a bug can never erase the running code.
 */
bool FlashDriver_EraseSector(uint8_t sector);

/**
 * @brief Program one 32-bit word.
 *
 * @param address  Absolute, 4-byte-aligned address inside sector 6 or 7.
 */
bool FlashDriver_ProgramWord(uint32_t address, uint32_t value);

#endif /* DRIVERS_FLASH_DRIVER_H */
