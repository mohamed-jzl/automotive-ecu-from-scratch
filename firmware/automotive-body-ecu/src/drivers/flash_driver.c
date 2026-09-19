/**
 * @file    flash_driver.c
 * @brief   Flash erase/program for the NVM sectors - see flash_driver.h.
 */

#include "flash_driver.h"

#include "ecu_config.h"
#include "stm32f4xx_hal.h"

#define NVM_FIRST_SECTOR    6U
#define NVM_LAST_SECTOR     7U
#define NVM_AREA_START      ECU_NVM_REGION_A_ADDRESS
#define NVM_AREA_END        (ECU_NVM_REGION_B_ADDRESS + ECU_NVM_REGION_SIZE)

/**
 * @brief Clear error flags left over from a previous operation.
 *
 * A classic trap: if an earlier operation left, say, the programming sequence
 * error flag (PGSERR) set, the HAL refuses every following write until it is
 * cleared - and nothing in the symptom points at the real cause.
 */
static void flash_clear_errors(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

bool FlashDriver_EraseSector(uint8_t sector)
{
    if ((sector < NVM_FIRST_SECTOR) || (sector > NVM_LAST_SECTOR))
    {
        return false;
    }

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t               failed_sector = 0U;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = sector;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;     /* 2.7 - 3.6 V */

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    flash_clear_errors();
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &failed_sector);
    (void)HAL_FLASH_Lock();

    return (status == HAL_OK);
}

bool FlashDriver_ProgramWord(uint32_t address, uint32_t value)
{
    if ((address < NVM_AREA_START) || (address >= NVM_AREA_END) || ((address & 3U) != 0U))
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    flash_clear_errors();
    const HAL_StatusTypeDef status =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t)value);
    (void)HAL_FLASH_Lock();

    /* Read back: the only proof that a write really worked. Programming an
     * already-programmed location, for instance, can "succeed" with the
     * wrong result. */
    return (status == HAL_OK) && (*(volatile const uint32_t *)address == value);
}
