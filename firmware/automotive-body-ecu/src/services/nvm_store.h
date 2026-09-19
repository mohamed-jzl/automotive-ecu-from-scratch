/**
 * @file    nvm_store.h
 * @brief   Power-loss-safe record storage in flash (emulated EEPROM).
 *
 * The problem
 * -----------
 * DTCs must survive a reset: a fault that disappears when the ignition is
 * switched off is exactly the one a technician needs to see. The STM32F446 has
 * no EEPROM, only flash, and flash has three awkward properties:
 *
 *   1. Programming can only turn bits from 1 to 0. Turning a bit back to 1
 *      requires ERASING - and erase works only on a whole sector.
 *   2. Our sectors are 128 KB, and erasing one blocks the CPU for 1-2 seconds.
 *      The watchdog times out after ~0.5 s, and the 10 ms control task would
 *      miss 100+ deadlines. Erasing at runtime is therefore not an option.
 *   3. Each sector survives only ~10 000 erase cycles.
 *
 * The design: an append-only log in two sectors
 * ---------------------------------------------
 * Instead of rewriting data in place, every save APPENDS a new record in the
 * next free slot. The newest valid record wins. Programming a 176-byte slot
 * takes well under a millisecond and needs no erase.
 *
 *      region A (sector 6)                     region B (sector 7)
 *      +--------+--------+--------+-----+      +------------------------+
 *      | seq 1  | seq 2  | seq 3  | FF..|      | FF FF FF FF (erased)   |
 *      +--------+--------+--------+-----+      +------------------------+
 *                          ^ newest            kept erased, ready
 *
 * When region A fills up, the newest record is copied to the already-erased
 * region B and writing continues there. Region A is erased only at the next
 * start-up, before the watchdog is armed, when a 2-second stall harms nothing.
 * A 128 KB region holds ~740 records, so at one save per second at most this
 * is plenty; realistically a region lasts for months of driving.
 *
 * Power-loss safety
 * -----------------
 * Power can fail halfway through writing a record. Every slot therefore ends
 * with a CRC-32, and the "magic" marker that declares a slot valid is the LAST
 * word written. A torn write leaves a slot with no magic or a bad CRC, which
 * the reader simply skips - and the previous record is still intact. During a
 * migration, the new copy is written completely before the old region is ever
 * erased, so at every instant at least one valid copy exists.
 *
 * This is the same idea as ST's application note AN3969 ("EEPROM emulation"),
 * and conceptually close to what AUTOSAR's Fee (Flash EEPROM Emulation)
 * module does.
 *
 * Pure module: all flash access goes through the NvmFlashIf_t function
 * pointers. On the ECU they call the flash driver; in the unit tests they
 * operate on a RAM array that imitates flash rules exactly (programming can
 * only clear bits), which is how torn writes are simulated.
 */

#ifndef SERVICES_NVM_STORE_H
#define SERVICES_NVM_STORE_H

#include <stdbool.h>
#include <stdint.h>

/** Maximum payload bytes per record. */
#define NVM_PAYLOAD_MAX         160U

/** Size of one slot in flash: magic + sequence + length + payload + CRC. */
#define NVM_SLOT_SIZE           (4U + 4U + 4U + NVM_PAYLOAD_MAX + 4U)

/**
 * @brief Access to the two flash regions.
 */
typedef struct
{
    const uint8_t *region_base[2];  /**< Memory-mapped read address of A and B  */
    uint32_t       region_size;     /**< Bytes per region                       */

    /** Erase a whole region (every byte becomes 0xFF). SLOW. Startup only. */
    bool (*erase_region)(uint8_t region, void *context);

    /** Program one 32-bit word at a 4-byte-aligned offset in a region. FAST. */
    bool (*program_word)(uint8_t region, uint32_t offset, uint32_t value,
                         void *context);

    void *context;
} NvmFlashIf_t;

typedef enum
{
    NVM_OK = 0,
    NVM_EMPTY,          /**< No valid record exists yet (first power-up)      */
    NVM_FULL,           /**< No free slot and no erased region to move to     */
    NVM_FLASH_ERROR,    /**< The flash driver reported a failure              */
    NVM_BAD_ARGUMENT
} NvmResult_t;

/**
 * @brief Store state. Allocated by the caller; all fields private.
 */
typedef struct
{
    NvmFlashIf_t flash;
    uint8_t      active_region;
    uint32_t     next_slot;          /**< Next free slot in the active region */
    uint32_t     sequence;           /**< Sequence number of the newest record */
    bool         has_record;
    uint8_t      latest_region;
    uint32_t     latest_slot;
    bool         other_region_erased;
} NvmStore_t;

/**
 * @brief Scan both regions and locate the newest valid record.
 *
 * Read-only: never erases anything.
 */
NvmResult_t NvmStore_Mount(NvmStore_t *store, const NvmFlashIf_t *flash);

/**
 * @brief Start-up housekeeping. MAY ERASE A SECTOR - call before the watchdog.
 *
 * Ensures the inactive region is erased and ready, and migrates to it if the
 * active region is fuller than @p compact_threshold_pct.
 */
NvmResult_t NvmStore_Maintain(NvmStore_t *store, uint8_t compact_threshold_pct);

/**
 * @brief Copy the newest record's payload into @p out.
 *
 * @return NVM_OK, or NVM_EMPTY if nothing has ever been saved.
 */
NvmResult_t NvmStore_Read(const NvmStore_t *store, uint8_t *out,
                          uint16_t out_size, uint16_t *out_length);

/**
 * @brief Append a new record. Never erases, so it is safe at runtime.
 *
 * @return NVM_FULL if both regions are full; the caller keeps its data in RAM
 *         and the next start-up's NvmStore_Maintain() makes room.
 */
NvmResult_t NvmStore_Write(NvmStore_t *store, const uint8_t *data, uint16_t length);

/** @brief Percentage of slots used in the active region. */
uint8_t NvmStore_GetUsagePercent(const NvmStore_t *store);

/**
 * @brief CRC-32 (IEEE 802.3 / zlib: reflected polynomial 0xEDB88320).
 *        Check value: CRC32("123456789") = 0xCBF43926.
 */
uint32_t NvmStore_Crc32(const uint8_t *data, uint32_t length);

#endif /* SERVICES_NVM_STORE_H */
