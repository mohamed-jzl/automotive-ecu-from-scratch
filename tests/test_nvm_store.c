/**
 * @file    test_nvm_store.c
 * @brief   Unit tests for the power-loss-safe flash record store.
 *
 * The "flash" here is a RAM array that follows the physical rules of real
 * flash exactly:
 *
 *   - erase sets every byte to 0xFF
 *   - programming can only clear bits (new = old AND value), never set them
 *
 * On top of that, a "power budget" makes the N-th programmed word fail, which
 * is how a power cut halfway through a write is simulated. Pulling the plug on
 * a real board at exactly the right microsecond is not a repeatable test;
 * this is.
 *
 * Regions are made tiny (4 slots) so that "region full" and "switch region"
 * cases happen after four writes instead of seven hundred.
 */

#include "unity_min.h"
#include "nvm_store.h"

#include <string.h>

#define SLOTS_PER_REGION  4U
#define REGION_SIZE       (NVM_SLOT_SIZE * SLOTS_PER_REGION)

static uint8_t s_flash[2][REGION_SIZE];
static int     s_erase_count[2];
static int     s_power_budget;           /* words left before "power loss"; -1 = unlimited */

static bool fake_erase(uint8_t region, void *context)
{
    (void)context;
    memset(s_flash[region], 0xFF, REGION_SIZE);
    s_erase_count[region]++;
    return true;
}

static bool fake_program(uint8_t region, uint32_t offset, uint32_t value, void *context)
{
    (void)context;

    if (s_power_budget == 0)
    {
        return false;                    /* power is gone */
    }
    if (s_power_budget > 0)
    {
        s_power_budget--;
    }

    for (uint32_t i = 0; i < 4U; i++)
    {
        /* The physics of flash: a 0 can be written over a 1, never the reverse. */
        s_flash[region][offset + i] &= (uint8_t)((value >> (8U * i)) & 0xFFU);
    }
    return true;
}

static const NvmFlashIf_t FLASH_IF =
{
    .region_base  = { s_flash[0], s_flash[1] },
    .region_size  = REGION_SIZE,
    .erase_region = fake_erase,
    .program_word = fake_program,
    .context      = NULL,
};

static NvmStore_t s_store;

void setUp(void)
{
    memset(s_flash, 0xFF, sizeof(s_flash));
    s_erase_count[0] = 0;
    s_erase_count[1] = 0;
    s_power_budget   = -1;
    (void)NvmStore_Mount(&s_store, &FLASH_IF);
}

void tearDown(void)
{
}

/** Write a small record whose content identifies it. */
static NvmResult_t write_marker(uint8_t marker)
{
    uint8_t data[10];
    memset(data, marker, sizeof(data));
    return NvmStore_Write(&s_store, data, sizeof(data));
}

/** Read the newest record and return its marker byte, or -1. */
static int read_marker(void)
{
    uint8_t  out[NVM_PAYLOAD_MAX];
    uint16_t length = 0;

    if (NvmStore_Read(&s_store, out, sizeof(out), &length) != NVM_OK)
    {
        return -1;
    }
    return out[0];
}

/** Simulate a reset: forget all RAM state and scan the flash again. */
static void reboot(void)
{
    s_power_budget = -1;
    TEST_ASSERT_EQUAL_INT(NVM_OK, NvmStore_Mount(&s_store, &FLASH_IF));
}

/* ========================================================================= */

static void test_crc32_matches_the_published_check_value(void)
{
    const uint8_t input[] = "123456789";
    TEST_ASSERT_EQUAL_UINT32(0xCBF43926UL, NvmStore_Crc32(input, 9));
}

static void test_empty_flash_has_no_record(void)
{
    uint8_t  out[16];
    uint16_t length = 0;

    TEST_ASSERT_EQUAL_INT(NVM_EMPTY, NvmStore_Read(&s_store, out, sizeof(out), &length));
    TEST_ASSERT_EQUAL_UINT8(0, NvmStore_GetUsagePercent(&s_store));
}

static void test_write_then_read_round_trip(void)
{
    const uint8_t data[5] = { 1, 2, 3, 4, 5 };
    TEST_ASSERT_EQUAL_INT(NVM_OK, NvmStore_Write(&s_store, data, sizeof(data)));

    uint8_t  out[16];
    uint16_t length = 0;
    TEST_ASSERT_EQUAL_INT(NVM_OK, NvmStore_Read(&s_store, out, sizeof(out), &length));
    TEST_ASSERT_EQUAL_UINT16(5, length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, out, 5);
}

static void test_newest_record_wins_across_a_reboot(void)
{
    (void)write_marker(0x11);
    (void)write_marker(0x22);
    (void)write_marker(0x33);

    reboot();
    TEST_ASSERT_EQUAL_INT(0x33, read_marker());
}

static void test_torn_write_keeps_the_previous_record(void)
{
    (void)write_marker(0xAA);

    /* Power fails after 3 words: the slot gets its sequence, length and part
     * of the payload, but never its CRC or magic marker. */
    s_power_budget = 3;
    TEST_ASSERT_EQUAL_INT(NVM_FLASH_ERROR, write_marker(0xBB));

    reboot();
    TEST_ASSERT_EQUAL_INT(0xAA, read_marker());

    /* The half-written slot cannot be reprogrammed, so the next write must
     * skip it rather than land on top of it. */
    TEST_ASSERT_EQUAL_INT(NVM_OK, write_marker(0xCC));
    reboot();
    TEST_ASSERT_EQUAL_INT(0xCC, read_marker());
}

static void test_corrupted_record_is_ignored(void)
{
    (void)write_marker(0x01);
    (void)write_marker(0x02);

    /* Bit rot in the second record's payload (slot 1, payload offset 12). */
    s_flash[0][NVM_SLOT_SIZE + 12] ^= 0x01;

    reboot();
    TEST_ASSERT_EQUAL_INT(0x01, read_marker());     /* falls back, CRC caught it */
}

static void test_full_region_switches_without_erasing(void)
{
    for (uint8_t i = 0; i < SLOTS_PER_REGION; i++)
    {
        TEST_ASSERT_EQUAL_INT(NVM_OK, write_marker((uint8_t)(0x10 + i)));
    }
    TEST_ASSERT_EQUAL_UINT8(100, NvmStore_GetUsagePercent(&s_store));

    /* Region A is full; region B is still erased, so writing continues there.
     * Crucially, NOTHING was erased: this path is safe at runtime. */
    TEST_ASSERT_EQUAL_INT(NVM_OK, write_marker(0x99));
    TEST_ASSERT_EQUAL_INT(0, s_erase_count[0] + s_erase_count[1]);

    reboot();
    TEST_ASSERT_EQUAL_INT(0x99, read_marker());
}

static void test_both_regions_full_reports_full(void)
{
    for (uint8_t i = 0; i < 2U * SLOTS_PER_REGION; i++)
    {
        TEST_ASSERT_EQUAL_INT(NVM_OK, write_marker(i));
    }

    /* No erased region left: refuse, and never erase at runtime. */
    TEST_ASSERT_EQUAL_INT(NVM_FULL, write_marker(0xEE));
    TEST_ASSERT_EQUAL_INT(0, s_erase_count[0] + s_erase_count[1]);
    TEST_ASSERT_EQUAL_INT(7, read_marker());         /* last good one intact */
}

static void test_maintain_erases_a_used_spare_region(void)
{
    /* Fill A, spill one record into B, reboot: A is now the spare and holds
     * old data. Maintain must erase it so a future switch needs no erase. */
    for (uint8_t i = 0; i <= SLOTS_PER_REGION; i++)
    {
        (void)write_marker(i);
    }
    reboot();

    TEST_ASSERT_EQUAL_INT(NVM_OK, NvmStore_Maintain(&s_store, 75));
    TEST_ASSERT_EQUAL_INT(1, s_erase_count[0]);
    TEST_ASSERT_EQUAL_INT((int)SLOTS_PER_REGION, read_marker());
}

static void test_maintain_migrates_a_nearly_full_region(void)
{
    for (uint8_t i = 0; i < 3; i++)       /* 3 of 4 slots = 75 % */
    {
        (void)write_marker((uint8_t)(0x40 + i));
    }

    TEST_ASSERT_EQUAL_INT(NVM_OK, NvmStore_Maintain(&s_store, 75));

    /* Latest copied to B first, THEN A erased. */
    TEST_ASSERT_EQUAL_INT(1, s_erase_count[0]);
    TEST_ASSERT_EQUAL_INT(0x42, read_marker());
    TEST_ASSERT_EQUAL_UINT8(25, NvmStore_GetUsagePercent(&s_store));

    reboot();
    TEST_ASSERT_EQUAL_INT(0x42, read_marker());
}

static void test_maintain_below_threshold_does_nothing(void)
{
    (void)write_marker(0x05);
    TEST_ASSERT_EQUAL_INT(NVM_OK, NvmStore_Maintain(&s_store, 75));
    TEST_ASSERT_EQUAL_INT(0, s_erase_count[0] + s_erase_count[1]);
}

static void test_invalid_arguments_are_rejected(void)
{
    uint8_t big[NVM_PAYLOAD_MAX + 1];
    memset(big, 0, sizeof(big));

    TEST_ASSERT_EQUAL_INT(NVM_BAD_ARGUMENT, NvmStore_Write(&s_store, big, sizeof(big)));
    TEST_ASSERT_EQUAL_INT(NVM_BAD_ARGUMENT, NvmStore_Write(&s_store, NULL, 4));
    TEST_ASSERT_EQUAL_INT(NVM_BAD_ARGUMENT, NvmStore_Mount(&s_store, NULL));
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("NVM Store (emulated EEPROM)");

    RUN_TEST(test_crc32_matches_the_published_check_value);
    RUN_TEST(test_empty_flash_has_no_record);
    RUN_TEST(test_write_then_read_round_trip);
    RUN_TEST(test_newest_record_wins_across_a_reboot);
    RUN_TEST(test_torn_write_keeps_the_previous_record);
    RUN_TEST(test_corrupted_record_is_ignored);
    RUN_TEST(test_full_region_switches_without_erasing);
    RUN_TEST(test_both_regions_full_reports_full);
    RUN_TEST(test_maintain_erases_a_used_spare_region);
    RUN_TEST(test_maintain_migrates_a_nearly_full_region);
    RUN_TEST(test_maintain_below_threshold_does_nothing);
    RUN_TEST(test_invalid_arguments_are_rejected);

    return UnityEnd();
}
