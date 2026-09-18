/**
 * @file    test_fault_manager.c
 * @brief   Unit tests for fault maturation, healing and reporting.
 *
 * The behaviour under test is fundamentally about *time* - how many
 * consecutive evaluations a condition must persist before it is believed.
 * On real hardware, provoking a marginal battery voltage that hovers around
 * a threshold for exactly the right number of cycles is close to impossible
 * to do repeatably.
 *
 * Because the module counts calls rather than reading a clock, the tests can
 * drive that timing exactly. This is the concrete payoff of keeping
 * diagnostic logic free of I/O: timing-dependent behaviour becomes
 * deterministic and provable instead of anecdotal.
 */

#include "unity_min.h"

#include "ecu_config.h"
#include "fault_manager.h"

void setUp(void)
{
    FaultManager_Init();
}

void tearDown(void)
{
}

/**
 * @brief Evaluate the same condition n times in a row.
 */
static void evaluate_n_times(FaultId_t id, bool condition, unsigned n)
{
    for (unsigned i = 0; i < n; i++)
    {
        (void)FaultManager_Update(id, condition);
    }
}

/**
 * @brief Latch a fault by running it through its full maturation.
 */
static void latch_fault(FaultId_t id)
{
    evaluate_n_times(id, true, ECU_FAULT_MATURATION_COUNT);
}

/* ========================================================================= */
/* Initial state                                                             */
/* ========================================================================= */

static void test_nothing_is_active_after_init(void)
{
    TEST_ASSERT_FALSE(FaultManager_IsAnyActive());
    TEST_ASSERT_EQUAL_UINT8(0, FaultManager_GetActiveCount());
    TEST_ASSERT_EQUAL_HEX16(0x0000, FaultManager_GetActiveBitmask());

    for (int i = 0; i < FAULT_COUNT; i++)
    {
        TEST_ASSERT_FALSE(FaultManager_IsActive((FaultId_t)i));
    }
}

/* ========================================================================= */
/* Maturation                                                                */
/* ========================================================================= */

static void test_single_detection_does_not_latch(void)
{
    /* The whole point of maturation. One bad reading - a noise spike, or the
     * voltage dip while the starter engages - must not put the vehicle into
     * limp mode. */
    (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, true);

    TEST_ASSERT_FALSE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
}

static void test_fault_latches_on_the_configured_count(void)
{
    /* One evaluation short of the threshold: still not latched. */
    evaluate_n_times(FAULT_BATTERY_UNDERVOLTAGE, true, ECU_FAULT_MATURATION_COUNT - 1U);
    TEST_ASSERT_FALSE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));

    /* The evaluation that reaches the threshold latches it. */
    (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, true);
    TEST_ASSERT_TRUE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
}

static void test_interrupted_maturation_restarts_from_zero(void)
{
    /* Two evaluations with the condition present... */
    evaluate_n_times(FAULT_BATTERY_UNDERVOLTAGE, true, ECU_FAULT_MATURATION_COUNT - 1U);

    /* ...then one clean one. An intermittent condition must not accumulate
     * credit across the gaps, or a signal that is merely noisy would
     * eventually latch as if it had been continuously faulty. */
    (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, false);

    /* One more present evaluation must now be far from enough. */
    (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, true);
    TEST_ASSERT_FALSE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
}

static void test_update_reports_the_latching_transition(void)
{
    bool changed = false;

    for (unsigned i = 0; i < ECU_FAULT_MATURATION_COUNT; i++)
    {
        changed = FaultManager_Update(FAULT_CAN_BUS_OFF, true);
    }

    /* Only the final call - the one that actually latched - reports a change.
     * This is what lets the caller log transitions instead of flooding the
     * trace with one line per evaluation. */
    TEST_ASSERT_TRUE(changed);

    TEST_ASSERT_FALSE(FaultManager_Update(FAULT_CAN_BUS_OFF, true));
}

/* ========================================================================= */
/* Healing                                                                   */
/* ========================================================================= */

static void test_healing_requires_more_cycles_than_latching(void)
{
    /* The asymmetry is deliberate and is the property being verified here:
     * quick to suspect, slow to forgive. */
    TEST_ASSERT_TRUE(ECU_FAULT_HEALING_COUNT > ECU_FAULT_MATURATION_COUNT);
}

static void test_fault_clears_only_after_full_healing(void)
{
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);
    TEST_ASSERT_TRUE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));

    /* One evaluation short of healed: still latched. */
    evaluate_n_times(FAULT_BATTERY_UNDERVOLTAGE, false, ECU_FAULT_HEALING_COUNT - 1U);
    TEST_ASSERT_TRUE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));

    /* The evaluation that reaches the threshold clears it. */
    (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, false);
    TEST_ASSERT_FALSE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
}

static void test_interrupted_healing_restarts_from_zero(void)
{
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);

    evaluate_n_times(FAULT_BATTERY_UNDERVOLTAGE, false, ECU_FAULT_HEALING_COUNT - 1U);

    /* A single reappearance voids all healing progress. A condition that
     * flickers must serve its full healing period again from the beginning. */
    (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, true);

    evaluate_n_times(FAULT_BATTERY_UNDERVOLTAGE, false, ECU_FAULT_HEALING_COUNT - 1U);
    TEST_ASSERT_TRUE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
}

static void test_chattering_condition_stays_latched(void)
{
    /* The realistic scenario the design exists for: a battery sitting exactly
     * on its threshold, alternating either side of it with noise. The fault
     * must settle into the safe interpretation rather than toggling. */
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);

    for (unsigned i = 0; i < 50; i++)
    {
        (void)FaultManager_Update(FAULT_BATTERY_UNDERVOLTAGE, (i % 2U) == 0U);
    }

    TEST_ASSERT_TRUE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
}

/* ========================================================================= */
/* Aggregation and reporting                                                 */
/* ========================================================================= */

static void test_faults_are_independent_of_one_another(void)
{
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);

    TEST_ASSERT_TRUE(FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE));
    TEST_ASSERT_FALSE(FaultManager_IsActive(FAULT_BATTERY_OVERVOLTAGE));
    TEST_ASSERT_FALSE(FaultManager_IsActive(FAULT_CAN_BUS_OFF));
    TEST_ASSERT_EQUAL_UINT8(1, FaultManager_GetActiveCount());
}

static void test_active_count_tracks_multiple_faults(void)
{
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);
    latch_fault(FAULT_CAN_BUS_OFF);
    latch_fault(FAULT_TASK_OVERRUN);

    TEST_ASSERT_EQUAL_UINT8(3, FaultManager_GetActiveCount());
    TEST_ASSERT_TRUE(FaultManager_IsAnyActive());
}

static void test_bitmask_sets_the_bit_matching_the_fault_id(void)
{
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);   /* id 0 -> bit 0 -> 0x0001 */
    TEST_ASSERT_EQUAL_HEX16(0x0001, FaultManager_GetActiveBitmask());

    latch_fault(FAULT_BATTERY_OVERVOLTAGE);    /* id 1 -> bit 1 -> 0x0002 */
    TEST_ASSERT_EQUAL_HEX16(0x0003, FaultManager_GetActiveBitmask());

    latch_fault(FAULT_CAN_BUS_OFF);            /* id 3 -> bit 3 -> 0x0008 */
    TEST_ASSERT_EQUAL_HEX16(0x000B, FaultManager_GetActiveBitmask());
}

static void test_every_fault_id_maps_to_a_distinct_bit(void)
{
    /* The bitmask is transmitted on CAN, so two faults sharing a bit would
     * make the network interface ambiguous. Latching all of them and counting
     * the set bits proves the mapping is one-to-one. */
    for (int i = 0; i < FAULT_COUNT; i++)
    {
        latch_fault((FaultId_t)i);
    }

    uint16_t mask      = FaultManager_GetActiveBitmask();
    int      bits_set  = 0;

    while (mask != 0U)
    {
        bits_set += (mask & 1U);
        mask >>= 1;
    }

    TEST_ASSERT_EQUAL_INT(FAULT_COUNT, bits_set);
    TEST_ASSERT_EQUAL_UINT8(FAULT_COUNT, FaultManager_GetActiveCount());
}

static void test_clear_all_resets_everything(void)
{
    latch_fault(FAULT_BATTERY_UNDERVOLTAGE);
    latch_fault(FAULT_CAN_BUS_OFF);

    FaultManager_ClearAll();

    TEST_ASSERT_FALSE(FaultManager_IsAnyActive());
    TEST_ASSERT_EQUAL_HEX16(0x0000, FaultManager_GetActiveBitmask());
}

/* ========================================================================= */
/* Robustness                                                                */
/* ========================================================================= */

static void test_out_of_range_id_is_handled_safely(void)
{
    /* Must not index past the internal arrays. */
    TEST_ASSERT_FALSE(FaultManager_Update((FaultId_t)99, true));
    TEST_ASSERT_FALSE(FaultManager_IsActive((FaultId_t)99));
    TEST_ASSERT_FALSE(FaultManager_IsAnyActive());
}

static void test_every_fault_has_a_usable_name(void)
{
    for (int i = 0; i < FAULT_COUNT; i++)
    {
        const char *name = FaultManager_GetName((FaultId_t)i);

        TEST_ASSERT_NOT_NULL(name);
        TEST_ASSERT_TRUE(name[0] != '\0');
    }

    TEST_ASSERT_EQUAL_STRING("UNKNOWN", FaultManager_GetName((FaultId_t)99));
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("Fault Manager");

    RUN_TEST(test_nothing_is_active_after_init);

    RUN_TEST(test_single_detection_does_not_latch);
    RUN_TEST(test_fault_latches_on_the_configured_count);
    RUN_TEST(test_interrupted_maturation_restarts_from_zero);
    RUN_TEST(test_update_reports_the_latching_transition);

    RUN_TEST(test_healing_requires_more_cycles_than_latching);
    RUN_TEST(test_fault_clears_only_after_full_healing);
    RUN_TEST(test_interrupted_healing_restarts_from_zero);
    RUN_TEST(test_chattering_condition_stays_latched);

    RUN_TEST(test_faults_are_independent_of_one_another);
    RUN_TEST(test_active_count_tracks_multiple_faults);
    RUN_TEST(test_bitmask_sets_the_bit_matching_the_fault_id);
    RUN_TEST(test_every_fault_id_maps_to_a_distinct_bit);
    RUN_TEST(test_clear_all_resets_everything);

    RUN_TEST(test_out_of_range_id_is_handled_safely);
    RUN_TEST(test_every_fault_has_a_usable_name);

    return UnityEnd();
}
