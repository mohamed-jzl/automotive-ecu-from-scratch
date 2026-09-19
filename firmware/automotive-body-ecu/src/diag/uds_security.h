/**
 * @file    uds_security.h
 * @brief   Seed/key algorithm for SecurityAccess (UDS service 0x27).
 *
 * How seed/key works
 * ------------------
 * Some diagnostic operations are dangerous: rewriting the VIN, running an
 * actuator test, reflashing the firmware. SecurityAccess makes the tester
 * prove it is authorised before the ECU allows them:
 *
 *      tester -> ECU    27 01                 "give me a seed"
 *      ECU -> tester    67 01 3A 91 C4 07     random 32-bit seed
 *      tester computes  key = f(seed)          using a secret algorithm
 *      tester -> ECU    27 02 <key>           "here is my key"
 *      ECU -> tester    67 02                 unlocked
 *
 * The ECU runs the same f() and compares. Because the seed is random each
 * time, a key recorded from a previous session is useless - that is the
 * protection against replaying a captured bus trace.
 *
 * Honesty about this implementation
 * ---------------------------------
 * The algorithm below is a few shifts and XORs with two constants. Anyone who
 * reads this file, or captures three seed/key pairs, can recover it. That is
 * acceptable here because the goal is to demonstrate the *protocol* - the
 * session requirement, attempt counting, time-delay lockout and NRCs.
 *
 * Production ECUs use real cryptography: AES-128 CMAC with a per-vehicle key
 * held in a hardware security module (HSM), or asymmetric signatures under
 * ISO 14229-1 service 0x29 (Authentication). Never ship a hand-rolled
 * algorithm like this one to protect anything that matters.
 *
 * Randomness
 * ----------
 * The STM32F446 has no hardware random number generator, so the seed comes
 * from a xorshift32 pseudo-random generator seeded from the chip's unique ID
 * and the boot time. It is unpredictable enough to stop a trivial replay, but
 * it is not cryptographically secure - another reason this belongs in a
 * teaching project, not in a vehicle.
 */

#ifndef DIAG_UDS_SECURITY_H
#define DIAG_UDS_SECURITY_H

#include <stdint.h>

/**
 * @brief Compute the key expected for a given seed.
 *
 * Pure function. Implemented independently in
 * tools/can_tester/diag_client.py; the two are checked against shared test
 * vectors so the tester and the ECU can never silently disagree.
 */
uint32_t UdsSecurity_ComputeKey(uint32_t seed);

/**
 * @brief Seed the pseudo-random generator. Zero is replaced by a constant,
 *        because xorshift stays at zero forever if started there.
 */
void UdsSecurity_SeedRandom(uint32_t entropy);

/**
 * @brief Next random seed. Never returns 0, because a seed of 0 has a
 *        reserved meaning in UDS: "this level is already unlocked".
 *
 * @param extra_entropy  Mixed into the generator before it steps. The server
 *        passes the millisecond timestamp of the request. Without this, the
 *        generator would start from the same state after every reset, the
 *        first seed after power-up would always be identical, and a key
 *        recorded once could be replayed forever. The tester's timing is not
 *        predictable to the millisecond, so mixing it in breaks that.
 */
uint32_t UdsSecurity_NextSeed(uint32_t extra_entropy);

#endif /* DIAG_UDS_SECURITY_H */
