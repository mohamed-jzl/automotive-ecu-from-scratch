/**
 * @file    uds_security.c
 * @brief   Seed/key implementation - educational, NOT secure. See the header.
 */

#include "uds_security.h"

/* The "secret" of the algorithm. In a real ECU this would be a cryptographic
 * key stored in protected memory, never a constant visible in the source. */
#define SECURITY_MASK       0xA5C3F00FUL
#define SECURITY_OFFSET     0x1D2B3C4DUL

/* Fallback state for the generator. Any non-zero constant works. */
#define PRNG_DEFAULT_STATE  0x6D2B79F5UL

static uint32_t s_prng_state = PRNG_DEFAULT_STATE;

uint32_t UdsSecurity_ComputeKey(uint32_t seed)
{
    uint32_t key = seed ^ SECURITY_MASK;

    key = (key << 7) | (key >> 25);     /* rotate left by 7 bits */
    key = key + SECURITY_OFFSET;        /* unsigned addition wraps mod 2^32 */
    key = key ^ (key >> 11);            /* mix high bits into low bits */

    return key;
}

void UdsSecurity_SeedRandom(uint32_t entropy)
{
    s_prng_state = (entropy != 0U) ? entropy : PRNG_DEFAULT_STATE;
}

uint32_t UdsSecurity_NextSeed(uint32_t extra_entropy)
{
    uint32_t x = s_prng_state ^ extra_entropy;

    if (x == 0U)
    {
        x = PRNG_DEFAULT_STATE;     /* xorshift must never start from 0 */
    }

    /* Marsaglia's xorshift32: three shift-XOR steps, period 2^32 - 1.
     * Started from a non-zero value it never produces zero, but the loop
     * guarantees the "never 0" contract even if that assumption were broken. */
    do
    {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
    } while (x == 0U);

    s_prng_state = x;
    return x;
}
