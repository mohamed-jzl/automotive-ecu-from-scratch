/**
 * @file    test_uds_security.c
 * @brief   Unit tests for the SecurityAccess seed/key algorithm and seed generator.
 *
 * The expected keys were computed by an independent Python implementation of
 * the same written algorithm (tools/can_tester/diag_client.py), not by running
 * this C code. If the ECU and the tester ever computed different keys, no
 * tester could unlock the ECU - these vectors are what guarantees they agree.
 */

#include "unity_min.h"
#include "uds_security.h"

void setUp(void)
{
    UdsSecurity_SeedRandom(0x12345678UL);
}

void tearDown(void)
{
}

static void test_known_key_vectors(void)
{
    TEST_ASSERT_EQUAL_UINT32(0xFF3CA7F7UL, UdsSecurity_ComputeKey(0x00000001UL));
    TEST_ASSERT_EQUAL_UINT32(0x18FD67E7UL, UdsSecurity_ComputeKey(0x12345678UL));
    TEST_ASSERT_EQUAL_UINT32(0xD44826DFUL, UdsSecurity_ComputeKey(0xDEADBEEFUL));
    TEST_ASSERT_EQUAL_UINT32(0x3B34521CUL, UdsSecurity_ComputeKey(0xFFFFFFFFUL));
    TEST_ASSERT_EQUAL_UINT32(0xC65D8834UL, UdsSecurity_ComputeKey(0x3A91C407UL));
}

static void test_key_is_never_the_seed_itself(void)
{
    /* A key equal to its seed would let a tester "unlock" by echoing the seed. */
    for (uint32_t seed = 1U; seed < 5000U; seed++)
    {
        TEST_ASSERT_TRUE(UdsSecurity_ComputeKey(seed) != seed);
    }
}

static void test_generator_never_returns_zero(void)
{
    /* Seed 0 means "already unlocked" in UDS. The generator must never
     * produce it by chance, or a locked ECU would claim to be unlocked. */
    UdsSecurity_SeedRandom(0U);         /* even from the worst start */

    for (uint32_t i = 0U; i < 100000U; i++)
    {
        TEST_ASSERT_TRUE(UdsSecurity_NextSeed(i) != 0U);
    }
}

static void test_seeds_are_not_repeated_back_to_back(void)
{
    uint32_t previous = UdsSecurity_NextSeed(0U);

    for (uint32_t i = 0U; i < 10000U; i++)
    {
        const uint32_t next = UdsSecurity_NextSeed(0U);
        TEST_ASSERT_TRUE(next != previous);
        previous = next;
    }
}

static void test_generator_first_step_matches_reference(void)
{
    /* xorshift32 from the documented default state, cross-checked in Python. */
    UdsSecurity_SeedRandom(0U);
    TEST_ASSERT_EQUAL_UINT32(0x40AEC71FUL, UdsSecurity_NextSeed(0U));
}

static void test_request_timing_changes_the_seed(void)
{
    /* Same generator state, different request times -> different seeds.
     * This is what prevents the first seed after power-up from always being
     * the same one. */
    UdsSecurity_SeedRandom(0xCAFEF00DUL);
    const uint32_t a = UdsSecurity_NextSeed(1000U);

    UdsSecurity_SeedRandom(0xCAFEF00DUL);
    const uint32_t b = UdsSecurity_NextSeed(1001U);

    TEST_ASSERT_TRUE(a != b);
}

int main(void)
{
    UnityBegin("UDS Security Access");

    RUN_TEST(test_known_key_vectors);
    RUN_TEST(test_key_is_never_the_seed_itself);
    RUN_TEST(test_generator_never_returns_zero);
    RUN_TEST(test_seeds_are_not_repeated_back_to_back);
    RUN_TEST(test_generator_first_step_matches_reference);
    RUN_TEST(test_request_timing_changes_the_seed);

    return UnityEnd();
}
