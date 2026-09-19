/**
 * @file    test_dtc_manager.c
 * @brief   Unit tests for DTC status bits, operation cycles, aging and storage.
 *
 * The DTC status byte is where diagnostic implementations most often disagree
 * with the standard, and where a tester reading the ECU will spot it at once.
 * Every assertion here checks an exact status value, written in binary where
 * it helps, against the bit definitions of ISO 14229-1 Annex D.
 *
 * Aging needs 40 ignition cycles. On a vehicle that is weeks of driving; here
 * it is a for-loop.
 */

#include "unity_min.h"
#include "dtc_manager.h"

#include <string.h>

#define IDX_VOLTAGE   0     /* lights the warning lamp */
#define IDX_BUS_OFF   1
#define IDX_ADC       2

static const DtcDefinition_t TABLE[] =
{
    { 0x056200UL, true  },  /* P0562-00 */
    { 0xC00188UL, false },  /* U0001-88 */
    { 0x900149UL, false },  /* B1001-49 */
};

static const DtcSnapshot_t ENV_LOW  = { 10500U, 3U };
static const DtcSnapshot_t ENV_HIGH = { 14000U, 2U };

void setUp(void)
{
    DtcManager_Init(TABLE, 3);
}

void tearDown(void)
{
}

/* ========================================================================= */
/* Status bits                                                               */
/* ========================================================================= */

static void test_initial_status_is_not_completed(void)
{
    /* 0x50 = TNCSLC + TNCTOC: "never tested since clear, not this cycle". */
    for (uint8_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(i));
    }
}

static void test_failure_sets_the_expected_bits(void)
{
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);

    /* 0x2F = TF | TFTOC | PDTC | CDTC | TFSLC   (0b0010_1111)
     * The two "not completed" bits are gone: the test has run. */
    TEST_ASSERT_EQUAL_HEX8(0x2F, DtcManager_GetStatus(IDX_BUS_OFF));
}

static void test_warning_lamp_dtc_also_sets_wir(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    TEST_ASSERT_EQUAL_HEX8(0xAF, DtcManager_GetStatus(IDX_VOLTAGE));   /* 0x2F | WIR */
}

static void test_pass_after_failure_keeps_the_history(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    DtcManager_ReportResult(IDX_VOLTAGE, false, &ENV_LOW);

    /* 0x2E: no longer failing (TF=0, WIR=0) but still confirmed, pending,
     * failed this cycle and since clear - an intermittent fault, which is
     * precisely what DTC memory exists to catch. */
    TEST_ASSERT_EQUAL_HEX8(0x2E, DtcManager_GetStatus(IDX_VOLTAGE));
}

static void test_pass_without_failure_only_clears_not_completed(void)
{
    DtcManager_ReportResult(IDX_ADC, false, NULL);
    TEST_ASSERT_EQUAL_HEX8(0x00, DtcManager_GetStatus(IDX_ADC));
}

static void test_reports_are_ignored_while_updates_are_disabled(void)
{
    DtcManager_SetUpdatesEnabled(false);
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);

    TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(IDX_VOLTAGE));

    DtcManager_SetUpdatesEnabled(true);
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    TEST_ASSERT_EQUAL_HEX8(0xAF, DtcManager_GetStatus(IDX_VOLTAGE));
}

/* ========================================================================= */
/* Occurrence counter and snapshot                                           */
/* ========================================================================= */

static void test_occurrence_counts_new_failures_not_repeated_reports(void)
{
    /* Held failing for three reports = ONE occurrence. */
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    TEST_ASSERT_EQUAL_UINT8(1, DtcManager_GetRecord(IDX_BUS_OFF)->occurrence_counter);

    /* Heal, fail again = second occurrence. */
    DtcManager_ReportResult(IDX_BUS_OFF, false, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    TEST_ASSERT_EQUAL_UINT8(2, DtcManager_GetRecord(IDX_BUS_OFF)->occurrence_counter);
}

static void test_snapshot_is_captured_at_first_failure_only(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    DtcManager_ReportResult(IDX_VOLTAGE, false, &ENV_HIGH);
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_HIGH);

    /* The freeze frame must describe the ORIGINAL event, not the latest. */
    const DtcRecord_t *record = DtcManager_GetRecord(IDX_VOLTAGE);
    TEST_ASSERT_TRUE(record->snapshot_valid);
    TEST_ASSERT_EQUAL_UINT16(10500, record->snapshot.battery_mv);
    TEST_ASSERT_EQUAL_UINT8(3, record->snapshot.vehicle_state);
}

/* ========================================================================= */
/* Operation cycles and aging                                                */
/* ========================================================================= */

static void test_new_cycle_resets_the_this_cycle_bits(void)
{
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, false, NULL);     /* 0x2E */

    DtcManager_StartOperationCycle();

    /* TFTOC cleared, TNCTOC set. PDTC stays: it failed in the cycle that just
     * ended. 0x2E & ~TFTOC | TNCTOC = 0x6C. */
    TEST_ASSERT_EQUAL_HEX8(0x6C, DtcManager_GetStatus(IDX_BUS_OFF));
}

static void test_pending_clears_after_a_clean_tested_cycle(void)
{
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_StartOperationCycle();                      /* cycle 2 begins */

    DtcManager_ReportResult(IDX_BUS_OFF, false, NULL);     /* tested, passed */
    DtcManager_StartOperationCycle();                      /* cycle 3 begins */

    TEST_ASSERT_EQUAL_HEX8(0, DtcManager_GetStatus(IDX_BUS_OFF) & DTC_STATUS_PENDING);
    TEST_ASSERT_TRUE((DtcManager_GetStatus(IDX_BUS_OFF) & DTC_STATUS_CONFIRMED) != 0);
}

static void test_pending_is_kept_when_the_test_never_ran(void)
{
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_StartOperationCycle();
    DtcManager_StartOperationCycle();      /* a cycle in which the monitor never ran */

    /* "Not tested" is not "passed": no evidence the fault has gone. */
    TEST_ASSERT_TRUE((DtcManager_GetStatus(IDX_BUS_OFF) & DTC_STATUS_PENDING) != 0);
}

static void test_confirmed_dtc_ages_out_after_the_threshold(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    DtcManager_ReportResult(IDX_VOLTAGE, false, &ENV_LOW);

    /* Off-by-one worth knowing: the cycle in which the DTC FAILED is not a
     * clean cycle. The first StartOperationCycle() below closes that failing
     * cycle without aging; each later one closes a clean cycle. So 40 calls
     * complete only 39 clean cycles - still confirmed. */
    for (unsigned i = 0; i < ECU_DTC_AGING_THRESHOLD; i++)
    {
        DtcManager_StartOperationCycle();
        DtcManager_ReportResult(IDX_VOLTAGE, false, &ENV_LOW);
    }
    TEST_ASSERT_TRUE((DtcManager_GetStatus(IDX_VOLTAGE) & DTC_STATUS_CONFIRMED) != 0);

    DtcManager_StartOperationCycle();      /* completes the 40th clean cycle */

    const DtcRecord_t *record = DtcManager_GetRecord(IDX_VOLTAGE);
    TEST_ASSERT_EQUAL_HEX8(0, record->status & DTC_STATUS_CONFIRMED);
    TEST_ASSERT_EQUAL_HEX8(0, record->status & DTC_STATUS_FAILED_SINCE_CLEAR);
    TEST_ASSERT_FALSE(record->snapshot_valid);
}

static void test_failing_again_restarts_aging(void)
{
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, false, NULL);

    /* 31 cycle starts: the first closes the failing cycle, 30 clean ones follow. */
    for (unsigned i = 0; i < 31; i++)
    {
        DtcManager_StartOperationCycle();
    }
    TEST_ASSERT_EQUAL_UINT8(30, DtcManager_GetRecord(IDX_BUS_OFF)->aging_counter);

    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    TEST_ASSERT_EQUAL_UINT8(0, DtcManager_GetRecord(IDX_BUS_OFF)->aging_counter);
}

/* ========================================================================= */
/* Queries, clearing, persistence                                            */
/* ========================================================================= */

static void test_count_by_mask_uses_any_bit_semantics(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);  /* confirmed + failing */
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, false, NULL);     /* confirmed, not failing */

    TEST_ASSERT_EQUAL_UINT8(2, DtcManager_CountByMask(DTC_STATUS_CONFIRMED));
    TEST_ASSERT_EQUAL_UINT8(1, DtcManager_CountByMask(DTC_STATUS_TEST_FAILED));
    TEST_ASSERT_EQUAL_UINT8(3, DtcManager_CountByMask(0xFF));   /* ADC still 0x50 */
}

static void test_clear_all_restores_initial_status(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);

    TEST_ASSERT_TRUE(DtcManager_Clear(0xFFFFFFUL));

    for (uint8_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(i));
        TEST_ASSERT_EQUAL_UINT8(0, DtcManager_GetRecord(i)->occurrence_counter);
    }
}

static void test_clear_single_dtc_leaves_the_others(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);

    TEST_ASSERT_TRUE(DtcManager_Clear(0xC00188UL));

    TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(IDX_BUS_OFF));
    TEST_ASSERT_EQUAL_HEX8(0xAF, DtcManager_GetStatus(IDX_VOLTAGE));
}

static void test_clear_unknown_dtc_is_refused(void)
{
    TEST_ASSERT_FALSE(DtcManager_Clear(0x123456UL));
}

static void test_find_by_code(void)
{
    TEST_ASSERT_EQUAL_INT(0,  DtcManager_FindByCode(0x056200UL));
    TEST_ASSERT_EQUAL_INT(2,  DtcManager_FindByCode(0x900149UL));
    TEST_ASSERT_EQUAL_INT(-1, DtcManager_FindByCode(0x000001UL));
}

static void test_dirty_flag_tracks_changes(void)
{
    TEST_ASSERT_FALSE(DtcManager_IsDirty());

    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    TEST_ASSERT_TRUE(DtcManager_IsDirty());

    DtcManager_ClearDirty();
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);   /* same status again */
    TEST_ASSERT_FALSE(DtcManager_IsDirty());            /* nothing to save   */
}

static void test_serialise_round_trip(void)
{
    DtcManager_ReportResult(IDX_VOLTAGE, true, &ENV_LOW);
    DtcManager_ReportResult(IDX_BUS_OFF, true, NULL);
    DtcManager_ReportResult(IDX_BUS_OFF, false, NULL);

    uint8_t        blob[64];
    const uint16_t length = DtcManager_Serialise(blob, sizeof(blob));
    TEST_ASSERT_EQUAL_UINT16(1 + 3 * DTC_RECORD_SERIALISED_SIZE, length);

    DtcManager_Init(TABLE, 3);                          /* simulate a reboot */
    TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(IDX_VOLTAGE));

    TEST_ASSERT_TRUE(DtcManager_Deserialise(blob, length));
    TEST_ASSERT_EQUAL_HEX8(0xAF, DtcManager_GetStatus(IDX_VOLTAGE));
    TEST_ASSERT_EQUAL_HEX8(0x2E, DtcManager_GetStatus(IDX_BUS_OFF));
    TEST_ASSERT_EQUAL_UINT16(10500, DtcManager_GetRecord(IDX_VOLTAGE)->snapshot.battery_mv);
}

static void test_deserialise_rejects_a_different_table(void)
{
    uint8_t        blob[64];
    const uint16_t length = DtcManager_Serialise(blob, sizeof(blob));

    /* Firmware update: the table now has 2 DTCs instead of 3. Loading the old
     * data would attach records to the wrong DTCs. */
    DtcManager_Init(TABLE, 2);
    TEST_ASSERT_FALSE(DtcManager_Deserialise(blob, length));
}

static void test_serialise_refuses_a_small_buffer(void)
{
    uint8_t blob[4];
    TEST_ASSERT_EQUAL_UINT16(0, DtcManager_Serialise(blob, sizeof(blob)));
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("DTC Manager");

    RUN_TEST(test_initial_status_is_not_completed);
    RUN_TEST(test_failure_sets_the_expected_bits);
    RUN_TEST(test_warning_lamp_dtc_also_sets_wir);
    RUN_TEST(test_pass_after_failure_keeps_the_history);
    RUN_TEST(test_pass_without_failure_only_clears_not_completed);
    RUN_TEST(test_reports_are_ignored_while_updates_are_disabled);

    RUN_TEST(test_occurrence_counts_new_failures_not_repeated_reports);
    RUN_TEST(test_snapshot_is_captured_at_first_failure_only);

    RUN_TEST(test_new_cycle_resets_the_this_cycle_bits);
    RUN_TEST(test_pending_clears_after_a_clean_tested_cycle);
    RUN_TEST(test_pending_is_kept_when_the_test_never_ran);
    RUN_TEST(test_confirmed_dtc_ages_out_after_the_threshold);
    RUN_TEST(test_failing_again_restarts_aging);

    RUN_TEST(test_count_by_mask_uses_any_bit_semantics);
    RUN_TEST(test_clear_all_restores_initial_status);
    RUN_TEST(test_clear_single_dtc_leaves_the_others);
    RUN_TEST(test_clear_unknown_dtc_is_refused);
    RUN_TEST(test_find_by_code);
    RUN_TEST(test_dirty_flag_tracks_changes);
    RUN_TEST(test_serialise_round_trip);
    RUN_TEST(test_deserialise_rejects_a_different_table);
    RUN_TEST(test_serialise_refuses_a_small_buffer);

    return UnityEnd();
}
