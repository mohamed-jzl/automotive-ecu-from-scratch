/**
 * @file    nvm_store.c
 * @brief   Append-only flash record store - see nvm_store.h for the design.
 *
 * Slot layout (176 bytes, all words little-endian):
 *
 *   offset   0  magic      0x4E564D31 ("NVM1"), written LAST
 *   offset   4  sequence   increases by one per record; newest wins
 *   offset   8  length     payload bytes (low 16 bits) | format version << 16
 *   offset  12  payload    NVM_PAYLOAD_MAX bytes, unused tail left at 0xFF
 *   offset 172  crc32      over offsets 4..171
 *
 * Writing the magic last is the whole power-loss strategy in one decision:
 * until that final word lands, the slot does not count, and the previous
 * record remains the newest valid one.
 */

#include "nvm_store.h"

#include <string.h>

#define NVM_MAGIC           0x4E564D31UL
#define NVM_FORMAT_VERSION  1U
#define NVM_ERASED_WORD     0xFFFFFFFFUL

#define OFFSET_MAGIC        0U
#define OFFSET_SEQUENCE     4U
#define OFFSET_LENGTH       8U
#define OFFSET_PAYLOAD      12U
#define OFFSET_CRC          (OFFSET_PAYLOAD + NVM_PAYLOAD_MAX)

/* ========================================================================= */
/* Helpers                                                                   */
/* ========================================================================= */

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
    p[2] = (uint8_t)((value >> 16) & 0xFFU);
    p[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint32_t slots_per_region(const NvmStore_t *store)
{
    return store->flash.region_size / NVM_SLOT_SIZE;
}

static const uint8_t *slot_address(const NvmStore_t *store, uint8_t region, uint32_t slot)
{
    return store->flash.region_base[region] + (slot * NVM_SLOT_SIZE);
}

static bool bytes_erased(const uint8_t *p, uint32_t length)
{
    for (uint32_t i = 0U; i < length; i++)
    {
        if (p[i] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

static bool slot_is_erased(const NvmStore_t *store, uint8_t region, uint32_t slot)
{
    return bytes_erased(slot_address(store, region, slot), NVM_SLOT_SIZE);
}

static bool region_is_erased(const NvmStore_t *store, uint8_t region)
{
    return bytes_erased(store->flash.region_base[region], store->flash.region_size);
}

/**
 * @brief A slot is valid only if its magic, length and CRC all check out.
 */
static bool slot_is_valid(const NvmStore_t *store, uint8_t region, uint32_t slot,
                          uint32_t *out_sequence)
{
    const uint8_t *p = slot_address(store, region, slot);

    if (get32(&p[OFFSET_MAGIC]) != NVM_MAGIC)
    {
        return false;
    }

    const uint32_t length_word = get32(&p[OFFSET_LENGTH]);

    if (((length_word & 0xFFFFU) > NVM_PAYLOAD_MAX) ||
        ((length_word >> 16) != NVM_FORMAT_VERSION))
    {
        return false;
    }

    const uint32_t expected = NvmStore_Crc32(&p[OFFSET_SEQUENCE],
                                             OFFSET_CRC - OFFSET_SEQUENCE);
    if (get32(&p[OFFSET_CRC]) != expected)
    {
        return false;       /* torn write or bit rot: ignore this slot */
    }

    if (out_sequence != NULL)
    {
        *out_sequence = get32(&p[OFFSET_SEQUENCE]);
    }
    return true;
}

/**
 * @brief Program one complete record into an erased slot.
 *
 * The record is built in RAM first, then programmed word by word with the
 * magic marker deliberately left for last.
 */
static bool write_slot(NvmStore_t *store, uint8_t region, uint32_t slot,
                       const uint8_t *data, uint16_t length, uint32_t sequence)
{
    uint8_t image[NVM_SLOT_SIZE];

    (void)memset(image, 0xFF, sizeof(image));
    put32(&image[OFFSET_SEQUENCE], sequence);
    put32(&image[OFFSET_LENGTH], (uint32_t)length | ((uint32_t)NVM_FORMAT_VERSION << 16));
    (void)memcpy(&image[OFFSET_PAYLOAD], data, length);
    put32(&image[OFFSET_CRC],
          NvmStore_Crc32(&image[OFFSET_SEQUENCE], OFFSET_CRC - OFFSET_SEQUENCE));

    const uint32_t base = slot * NVM_SLOT_SIZE;

    for (uint32_t offset = OFFSET_SEQUENCE; offset < NVM_SLOT_SIZE; offset += 4U)
    {
        const uint32_t word = get32(&image[offset]);

        /* An erased word already reads 0xFFFFFFFF; programming it is a no-op
         * that would only cost time. */
        if (word == NVM_ERASED_WORD)
        {
            continue;
        }

        if (!store->flash.program_word(region, base + offset, word, store->flash.context))
        {
            return false;
        }
    }

    /* The commit point. Before this line the slot is invisible to readers. */
    return store->flash.program_word(region, base + OFFSET_MAGIC, NVM_MAGIC,
                                     store->flash.context);
}

/** @brief Index just after the last non-erased slot of a region. */
static uint32_t find_next_free_slot(const NvmStore_t *store, uint8_t region)
{
    uint32_t next = 0U;

    for (uint32_t slot = 0U; slot < slots_per_region(store); slot++)
    {
        /* A torn slot is neither valid nor erased: it can never be
         * programmed again, so it counts as used. */
        if (!slot_is_erased(store, region, slot))
        {
            next = slot + 1U;
        }
    }
    return next;
}

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

uint32_t NvmStore_Crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            /* Reflected algorithm: shift right, XOR the reversed polynomial
             * whenever the bit shifted out was a 1. */
            crc = ((crc & 1U) != 0U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }

    return ~crc;
}

NvmResult_t NvmStore_Mount(NvmStore_t *store, const NvmFlashIf_t *flash)
{
    if ((store == NULL) || (flash == NULL) || (flash->region_size < NVM_SLOT_SIZE) ||
        (flash->erase_region == NULL) || (flash->program_word == NULL))
    {
        return NVM_BAD_ARGUMENT;
    }

    (void)memset(store, 0, sizeof(*store));
    store->flash = *flash;

    /* Newest record = highest sequence number across BOTH regions. Searching
     * both covers the moment during a migration where each holds a copy. */
    for (uint8_t region = 0U; region < 2U; region++)
    {
        for (uint32_t slot = 0U; slot < slots_per_region(store); slot++)
        {
            uint32_t sequence = 0U;

            if (slot_is_valid(store, region, slot, &sequence) &&
                ((!store->has_record) || (sequence > store->sequence)))
            {
                store->has_record    = true;
                store->sequence      = sequence;
                store->latest_region = region;
                store->latest_slot   = slot;
            }
        }
    }

    store->active_region       = store->has_record ? store->latest_region : 0U;
    store->next_slot           = find_next_free_slot(store, store->active_region);
    store->other_region_erased = region_is_erased(store, (uint8_t)(1U - store->active_region));

    return NVM_OK;
}

NvmResult_t NvmStore_Maintain(NvmStore_t *store, uint8_t compact_threshold_pct)
{
    if (store == NULL)
    {
        return NVM_BAD_ARGUMENT;
    }

    const uint8_t other = (uint8_t)(1U - store->active_region);

    /* Step 1: make sure the spare region is erased, so that a runtime switch
     * never has to erase anything. */
    if (!store->other_region_erased)
    {
        if (!store->flash.erase_region(other, store->flash.context))
        {
            return NVM_FLASH_ERROR;
        }
        store->other_region_erased = true;
    }

    if (NvmStore_GetUsagePercent(store) < compact_threshold_pct)
    {
        return NVM_OK;
    }

    /* Step 2: the active region is getting full. */
    if (store->has_record)
    {
        uint8_t  payload[NVM_PAYLOAD_MAX];
        uint16_t length = 0U;

        if (NvmStore_Read(store, payload, sizeof(payload), &length) != NVM_OK)
        {
            return NVM_FLASH_ERROR;
        }

        /* Copy first, erase second: at every moment a valid copy exists. */
        if (!write_slot(store, other, 0U, payload, length, store->sequence + 1U))
        {
            return NVM_FLASH_ERROR;
        }

        store->sequence      = store->sequence + 1U;
        store->latest_region = other;
        store->latest_slot   = 0U;
    }

    const uint8_t old = store->active_region;

    if (!store->flash.erase_region(old, store->flash.context))
    {
        return NVM_FLASH_ERROR;
    }

    store->active_region       = other;
    store->next_slot           = store->has_record ? 1U : 0U;
    store->other_region_erased = true;      /* the old region, now erased */

    return NVM_OK;
}

NvmResult_t NvmStore_Read(const NvmStore_t *store, uint8_t *out,
                          uint16_t out_size, uint16_t *out_length)
{
    if ((store == NULL) || (out == NULL) || (out_length == NULL))
    {
        return NVM_BAD_ARGUMENT;
    }

    if (!store->has_record)
    {
        return NVM_EMPTY;
    }

    const uint8_t *p      = slot_address(store, store->latest_region, store->latest_slot);
    const uint16_t length = (uint16_t)(get32(&p[OFFSET_LENGTH]) & 0xFFFFU);

    if (length > out_size)
    {
        return NVM_BAD_ARGUMENT;
    }

    (void)memcpy(out, &p[OFFSET_PAYLOAD], length);
    *out_length = length;
    return NVM_OK;
}

NvmResult_t NvmStore_Write(NvmStore_t *store, const uint8_t *data, uint16_t length)
{
    if ((store == NULL) || (data == NULL) || (length > NVM_PAYLOAD_MAX))
    {
        return NVM_BAD_ARGUMENT;
    }

    uint8_t  region = store->active_region;
    uint32_t slot   = store->next_slot;

    if (slot >= slots_per_region(store))
    {
        /* Active region full. Move to the spare region - allowed at runtime
         * ONLY because it is already erased, so no erase is needed now. */
        if (!store->other_region_erased)
        {
            return NVM_FULL;
        }

        region                     = (uint8_t)(1U - store->active_region);
        slot                       = 0U;
        store->active_region       = region;
        store->other_region_erased = false;     /* the old one is full */
    }

    const bool ok = write_slot(store, region, slot, data, length, store->sequence + 1U);

    /* Even a failed write consumed the slot: partly programmed flash cannot
     * be programmed again until the region is erased. */
    store->next_slot = slot + 1U;

    if (!ok)
    {
        return NVM_FLASH_ERROR;
    }

    store->sequence      = store->sequence + 1U;
    store->has_record    = true;
    store->latest_region = region;
    store->latest_slot   = slot;
    return NVM_OK;
}

uint8_t NvmStore_GetUsagePercent(const NvmStore_t *store)
{
    if ((store == NULL) || (slots_per_region(store) == 0U))
    {
        return 0U;
    }

    const uint32_t used = (store->next_slot > slots_per_region(store))
                              ? slots_per_region(store) : store->next_slot;

    return (uint8_t)((used * 100U) / slots_per_region(store));
}
