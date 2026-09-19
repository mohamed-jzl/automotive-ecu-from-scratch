/**
 * @file    test_uds_server.c
 * @brief   Unit tests for the UDS (ISO 14229-1) server.
 *
 * Every test sends raw request bytes and checks raw response bytes, exactly
 * as they would appear in a CAN trace. That is deliberate: the byte-level
 * behaviour IS the interface a tester depends on, so it is what gets tested.
 *
 * Reading guide for the expected responses:
 *
 *      positive   <SID + 0x40> <data...>        e.g. 22 -> 62, 10 -> 50
 *      negative   7F <SID> <NRC>                e.g. 7F 27 35 = invalid key
 *
 * The server is linked with the real DTC manager and the real security
 * algorithm; only the application's DID and routine tables are fakes.
 */

#include "unity_min.h"

#include "dtc_manager.h"
#include "ecu_config.h"
#include "uds_security.h"
#include "uds_server.h"

#include <string.h>

/* ========================================================================= */
/* Fake application tables                                                   */
/* ========================================================================= */

static uint8_t s_vin[17];
static bool    s_routine_ran;

static uint8_t read_vin(uint8_t *out)        { memcpy(out, s_vin, 17); return 0; }
static uint8_t read_battery(uint8_t *out)    { out[0] = 0x31; out[1] = 0x38; return 0; }  /* 12600 mV */
static uint8_t read_unavailable(uint8_t *out){ (void)out; return UDS_NRC_CONDITIONS_NOT_CORRECT; }

static uint8_t write_vin(const uint8_t *in)
{
    for (int i = 0; i < 17; i++)
    {
        if (in[i] == 'I') { return UDS_NRC_REQUEST_OUT_OF_RANGE; }   /* ISO 3779 */
    }
    memcpy(s_vin, in, 17);
    return 0;
}

static const UdsDidEntry_t DIDS[] =
{
    { 0xF190, 17, read_vin,         write_vin },
    { 0x0100,  2, read_battery,     NULL      },
    { 0x0200,  1, read_unavailable, NULL      },
};

static uint8_t routine_start(const uint8_t *in, uint16_t n, uint8_t *out, uint16_t size, uint16_t *len)
{
    (void)in; (void)n; (void)out; (void)size;
    s_routine_ran = true;
    *len = 0;
    return 0;
}

static uint8_t routine_results(const uint8_t *in, uint16_t n, uint8_t *out, uint16_t size, uint16_t *len)
{
    (void)in; (void)n; (void)size;
    out[0] = 0x02;          /* "completed" */
    *len   = 1;
    return 0;
}

static const UdsRoutineEntry_t ROUTINES[] =
{
    { 0x0201, true,  routine_start, NULL, routine_results },   /* needs security */
    { 0x0202, false, routine_start, NULL, NULL            },
};

static const DtcDefinition_t DTCS[] =
{
    { 0x056200UL, true  },      /* P0562-00 */
    { 0xC00188UL, false },      /* U0001-88 */
};

/* ========================================================================= */
/* Request helpers                                                           */
/* ========================================================================= */

static uint8_t  s_rsp[256];
static uint16_t s_len;
static uint32_t s_now;

static void send_raw(const uint8_t *request, uint16_t length, bool functional)
{
    memset(s_rsp, 0, sizeof(s_rsp));
    s_len = UdsServer_ProcessRequest(request, length, functional, s_rsp, sizeof(s_rsp), s_now);
}

/* REQ(0x22, 0xF1, 0x90) sends a physical request; FREQ(...) a functional one. */
#define REQ(...)  do { const uint8_t r_[] = { __VA_ARGS__ }; send_raw(r_, sizeof(r_), false); } while (0)
#define FREQ(...) do { const uint8_t r_[] = { __VA_ARGS__ }; send_raw(r_, sizeof(r_), true);  } while (0)

/** Assert the last response was exactly "7F <sid> <nrc>". */
#define ASSERT_NRC(sid, nrc)                           \
    do {                                               \
        TEST_ASSERT_EQUAL_UINT16(3, s_len);            \
        TEST_ASSERT_EQUAL_HEX8(0x7F, s_rsp[0]);        \
        TEST_ASSERT_EQUAL_HEX8((sid), s_rsp[1]);       \
        TEST_ASSERT_EQUAL_HEX8((nrc), s_rsp[2]);       \
    } while (0)

static void enter_extended(void)
{
    REQ(0x10, 0x03);
}

static void unlock(void)
{
    enter_extended();
    REQ(0x27, 0x01);
    const uint32_t seed = ((uint32_t)s_rsp[2] << 24) | ((uint32_t)s_rsp[3] << 16) |
                          ((uint32_t)s_rsp[4] << 8)  | s_rsp[5];
    const uint32_t key  = UdsSecurity_ComputeKey(seed);
    REQ(0x27, 0x02, (uint8_t)(key >> 24), (uint8_t)(key >> 16),
                    (uint8_t)(key >> 8),  (uint8_t)key);
}

void setUp(void)
{
    const UdsServerConfig_t config = { DIDS, 3, ROUTINES, 2 };

    s_now         = 1000;
    s_routine_ran = false;
    memcpy(s_vin, "00000000000000000", 17);

    DtcManager_Init(DTCS, 2);
    UdsSecurity_SeedRandom(0xC0FFEE11UL);
    UdsServer_Init(&config, s_now);
}

void tearDown(void)
{
}

/* ========================================================================= */
/* Generic rules                                                             */
/* ========================================================================= */

static void test_starts_in_default_session_locked(void)
{
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_DEFAULT, UdsServer_GetSession());
    TEST_ASSERT_FALSE(UdsServer_IsUnlocked());
}

static void test_unknown_service_is_rejected(void)
{
    REQ(0xAA, 0x00);
    ASSERT_NRC(0xAA, UDS_NRC_SERVICE_NOT_SUPPORTED);
}

static void test_service_in_wrong_session_is_rejected(void)
{
    REQ(0x27, 0x01);                    /* SecurityAccess needs extended */
    ASSERT_NRC(0x27, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
}

static void test_missing_subfunction_is_a_length_error(void)
{
    REQ(0x10);
    ASSERT_NRC(0x10, UDS_NRC_INCORRECT_MESSAGE_LENGTH);
}

/* ========================================================================= */
/* 0x10 DiagnosticSessionControl                                             */
/* ========================================================================= */

static void test_extended_session_response_carries_timing(void)
{
    REQ(0x10, 0x03);

    /* 50 03 | P2 = 0x0032 (50 ms) | P2* = 0x01F4 (500 x 10 ms = 5 s) */
    const uint8_t expected[] = { 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4 };
    TEST_ASSERT_EQUAL_UINT16(6, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 6);
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_EXTENDED, UdsServer_GetSession());
}

static void test_programming_session_is_not_supported(void)
{
    REQ(0x10, 0x02);
    ASSERT_NRC(0x10, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

static void test_session_request_with_extra_byte_is_rejected(void)
{
    REQ(0x10, 0x03, 0x00);
    ASSERT_NRC(0x10, UDS_NRC_INCORRECT_MESSAGE_LENGTH);
}

static void test_any_session_change_relocks_security(void)
{
    unlock();
    TEST_ASSERT_TRUE(UdsServer_IsUnlocked());

    REQ(0x10, 0x03);                    /* extended -> extended */
    TEST_ASSERT_FALSE(UdsServer_IsUnlocked());
}

/* ========================================================================= */
/* 0x3E TesterPresent and S3                                                 */
/* ========================================================================= */

static void test_tester_present_responds(void)
{
    REQ(0x3E, 0x00);
    TEST_ASSERT_EQUAL_UINT16(2, s_len);
    TEST_ASSERT_EQUAL_HEX8(0x7E, s_rsp[0]);
}

static void test_suppress_bit_silences_the_positive_response(void)
{
    REQ(0x3E, 0x80);
    TEST_ASSERT_EQUAL_UINT16(0, s_len);
}

static void test_suppress_bit_never_hides_an_error(void)
{
    REQ(0x3E, 0x81);                    /* sub-function 0x01 does not exist */
    ASSERT_NRC(0x3E, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

static void test_s3_timeout_returns_to_default_session(void)
{
    unlock();

    s_now += ECU_UDS_S3_SERVER_MS - 1U;
    UdsServer_Poll(s_now);
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_EXTENDED, UdsServer_GetSession());

    s_now += 1U;
    UdsServer_Poll(s_now);
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_DEFAULT, UdsServer_GetSession());
    TEST_ASSERT_FALSE(UdsServer_IsUnlocked());
}

static void test_tester_present_keeps_the_session_alive(void)
{
    enter_extended();

    for (int i = 0; i < 5; i++)         /* 5 x 4 s = 20 s, well past S3 */
    {
        s_now += 4000U;
        UdsServer_Poll(s_now);
        REQ(0x3E, 0x80);
    }

    TEST_ASSERT_EQUAL_INT(UDS_SESSION_EXTENDED, UdsServer_GetSession());
}

/* ========================================================================= */
/* 0x22 ReadDataByIdentifier                                                 */
/* ========================================================================= */

static void test_read_single_did(void)
{
    REQ(0x22, 0x01, 0x00);

    const uint8_t expected[] = { 0x62, 0x01, 0x00, 0x31, 0x38 };
    TEST_ASSERT_EQUAL_UINT16(5, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 5);
}

static void test_read_several_dids_skips_unsupported_ones(void)
{
    REQ(0x22, 0x01, 0x00, 0x12, 0x34, 0xF1, 0x90);   /* 0x1234 does not exist */

    /* 62 | 0100 + 2 bytes | F190 + 17 bytes = 1 + 4 + 19 = 24 bytes */
    TEST_ASSERT_EQUAL_UINT16(24, s_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_rsp[1]);
    TEST_ASSERT_EQUAL_HEX8(0xF1, s_rsp[5]);
    TEST_ASSERT_EQUAL_HEX8(0x90, s_rsp[6]);
    TEST_ASSERT_EQUAL_HEX8('0',  s_rsp[7]);
}

static void test_read_only_unsupported_dids_is_out_of_range(void)
{
    REQ(0x22, 0x12, 0x34);
    ASSERT_NRC(0x22, UDS_NRC_REQUEST_OUT_OF_RANGE);
}

static void test_read_odd_length_is_rejected(void)
{
    REQ(0x22, 0xF1);
    ASSERT_NRC(0x22, UDS_NRC_INCORRECT_MESSAGE_LENGTH);
}

static void test_read_failure_is_reported(void)
{
    REQ(0x22, 0x02, 0x00);
    ASSERT_NRC(0x22, UDS_NRC_CONDITIONS_NOT_CORRECT);
}

static void test_response_too_long_is_refused_not_overflowed(void)
{
    const uint8_t request[] = { 0x22, 0xF1, 0x90 };
    uint8_t small[8];

    s_len = UdsServer_ProcessRequest(request, 3, false, small, sizeof(small), s_now);

    TEST_ASSERT_EQUAL_UINT16(3, s_len);
    TEST_ASSERT_EQUAL_HEX8(0x7F, small[0]);
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_RESPONSE_TOO_LONG, small[2]);
}

/* ========================================================================= */
/* 0x2E WriteDataByIdentifier                                                */
/* ========================================================================= */

static void test_write_not_allowed_in_default_session(void)
{
    REQ(0x2E, 0xF1, 0x90, 'W');
    ASSERT_NRC(0x2E, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
}

static void test_write_requires_security(void)
{
    enter_extended();
    REQ(0x2E, 0xF1, 0x90, 'W','D','B','1','2','3','4','5','6','7','8','9','0','A','B','C','D');
    ASSERT_NRC(0x2E, UDS_NRC_SECURITY_ACCESS_DENIED);
}

static void test_write_unknown_did_is_out_of_range(void)
{
    enter_extended();
    REQ(0x2E, 0x01, 0x00, 0x12, 0x34);  /* 0x0100 exists but is read-only */
    ASSERT_NRC(0x2E, UDS_NRC_REQUEST_OUT_OF_RANGE);
}

static void test_write_wrong_length_is_rejected(void)
{
    unlock();
    REQ(0x2E, 0xF1, 0x90, 'W', 'D', 'B');
    ASSERT_NRC(0x2E, UDS_NRC_INCORRECT_MESSAGE_LENGTH);
}

static void test_write_invalid_value_is_rejected_by_the_application(void)
{
    unlock();
    REQ(0x2E, 0xF1, 0x90, 'W','D','B','I','2','3','4','5','6','7','8','9','0','A','B','C','D');
    ASSERT_NRC(0x2E, UDS_NRC_REQUEST_OUT_OF_RANGE);
}

static void test_write_succeeds_when_unlocked(void)
{
    unlock();
    REQ(0x2E, 0xF1, 0x90, 'W','D','B','1','2','3','4','5','6','7','8','9','0','A','B','C','D');

    const uint8_t expected[] = { 0x6E, 0xF1, 0x90 };
    TEST_ASSERT_EQUAL_UINT16(3, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 3);
    TEST_ASSERT_EQUAL_HEX8('W', s_vin[0]);
}

/* ========================================================================= */
/* 0x27 SecurityAccess                                                       */
/* ========================================================================= */

static void test_correct_key_unlocks(void)
{
    unlock();

    const uint8_t expected[] = { 0x67, 0x02 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 2);
    TEST_ASSERT_TRUE(UdsServer_IsUnlocked());
}

static void test_seed_is_zero_when_already_unlocked(void)
{
    unlock();
    REQ(0x27, 0x01);

    const uint8_t expected[] = { 0x67, 0x01, 0x00, 0x00, 0x00, 0x00 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 6);
}

static void test_key_without_seed_is_a_sequence_error(void)
{
    enter_extended();
    REQ(0x27, 0x02, 0x11, 0x22, 0x33, 0x44);
    ASSERT_NRC(0x27, UDS_NRC_REQUEST_SEQUENCE_ERROR);
}

static void test_wrong_key_is_rejected_and_consumes_the_seed(void)
{
    enter_extended();
    REQ(0x27, 0x01);
    REQ(0x27, 0x02, 0x00, 0x00, 0x00, 0x00);
    ASSERT_NRC(0x27, UDS_NRC_INVALID_KEY);

    /* A second guess against the same seed is refused: one seed, one try. */
    REQ(0x27, 0x02, 0x00, 0x00, 0x00, 0x01);
    ASSERT_NRC(0x27, UDS_NRC_REQUEST_SEQUENCE_ERROR);
}

static void test_three_wrong_keys_trigger_a_lockout(void)
{
    enter_extended();

    for (int attempt = 1; attempt <= 3; attempt++)
    {
        REQ(0x27, 0x01);
        REQ(0x27, 0x02, 0xDE, 0xAD, 0xBE, 0xEF);
    }
    ASSERT_NRC(0x27, UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS);

    /* During the delay, no seed is handed out at all. */
    s_now += ECU_UDS_SECURITY_LOCKOUT_MS - 1U;
    REQ(0x27, 0x01);
    ASSERT_NRC(0x27, UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED);

    /* Once it expires, seeds are available again. */
    s_now += 1U;
    REQ(0x27, 0x01);
    TEST_ASSERT_EQUAL_HEX8(0x67, s_rsp[0]);
}

static void test_unknown_security_level_is_rejected(void)
{
    enter_extended();
    REQ(0x27, 0x05);
    ASSERT_NRC(0x27, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

/* ========================================================================= */
/* 0x11 ECUReset                                                             */
/* ========================================================================= */

static void test_hard_reset_is_acknowledged_then_requested(void)
{
    uint8_t type = 0;

    REQ(0x11, 0x01);
    const uint8_t expected[] = { 0x51, 0x01 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 2);

    TEST_ASSERT_TRUE(UdsServer_TakeResetRequest(&type));
    TEST_ASSERT_EQUAL_HEX8(UDS_RESET_HARD, type);
    TEST_ASSERT_FALSE(UdsServer_TakeResetRequest(&type));   /* taken once only */
}

static void test_suppressed_reset_still_resets(void)
{
    REQ(0x11, 0x81);
    TEST_ASSERT_EQUAL_UINT16(0, s_len);
    TEST_ASSERT_TRUE(UdsServer_TakeResetRequest(NULL));
}

static void test_unsupported_reset_type_is_rejected(void)
{
    REQ(0x11, 0x05);
    ASSERT_NRC(0x11, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
    TEST_ASSERT_FALSE(UdsServer_TakeResetRequest(NULL));
}

/* ========================================================================= */
/* Functional addressing                                                     */
/* ========================================================================= */

static void test_functional_not_supported_errors_are_silent(void)
{
    FREQ(0xAA, 0x00);                   /* unknown service                 */
    TEST_ASSERT_EQUAL_UINT16(0, s_len);

    FREQ(0x27, 0x01);                   /* wrong session                   */
    TEST_ASSERT_EQUAL_UINT16(0, s_len);

    FREQ(0x22, 0x12, 0x34);             /* unsupported DID                 */
    TEST_ASSERT_EQUAL_UINT16(0, s_len);
}

static void test_functional_real_errors_are_still_reported(void)
{
    FREQ(0x22, 0xF1);                   /* malformed: not a "not supported" */
    ASSERT_NRC(0x22, UDS_NRC_INCORRECT_MESSAGE_LENGTH);
}

static void test_functional_request_gets_a_normal_positive_response(void)
{
    FREQ(0x3E, 0x00);
    TEST_ASSERT_EQUAL_UINT16(2, s_len);
}

/* ========================================================================= */
/* 0x19 ReadDTCInformation                                                   */
/* ========================================================================= */

static void test_count_dtcs_by_status_mask(void)
{
    DtcManager_ReportResult(0, true, NULL);
    REQ(0x19, 0x01, 0x08);              /* how many confirmed? */

    /* 59 01 | availability FF | format 01 | count 0x0001 */
    const uint8_t expected[] = { 0x59, 0x01, 0xFF, 0x01, 0x00, 0x01 };
    TEST_ASSERT_EQUAL_UINT16(6, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 6);
}

static void test_list_dtcs_by_status_mask(void)
{
    DtcManager_ReportResult(0, true, NULL);
    REQ(0x19, 0x02, 0x08);

    /* 59 02 FF | 05 62 00 AF  -  P0562-00, status 0xAF */
    const uint8_t expected[] = { 0x59, 0x02, 0xFF, 0x05, 0x62, 0x00, 0xAF };
    TEST_ASSERT_EQUAL_UINT16(7, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 7);
}

static void test_list_supported_dtcs(void)
{
    REQ(0x19, 0x0A);

    const uint8_t expected[] = { 0x59, 0x0A, 0xFF,
                                 0x05, 0x62, 0x00, 0x50,
                                 0xC0, 0x01, 0x88, 0x50 };
    TEST_ASSERT_EQUAL_UINT16(11, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, 11);
}

static void test_snapshot_record(void)
{
    const DtcSnapshot_t env = { 10500U, 3U };
    DtcManager_ReportResult(0, true, &env);

    REQ(0x19, 0x04, 0x05, 0x62, 0x00, 0x01);

    /* 59 04 | DTC 056200 | status AF | record 01 | 2 identifiers |
     * 0100 -> 0x2904 (10500 mV) | 0101 -> 03 (RUN) */
    const uint8_t expected[] = { 0x59, 0x04, 0x05, 0x62, 0x00, 0xAF,
                                 0x01, 0x02, 0x01, 0x00, 0x29, 0x04,
                                 0x01, 0x01, 0x03 };
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_rsp, sizeof(expected));
}

static void test_snapshot_absent_when_dtc_never_failed(void)
{
    REQ(0x19, 0x04, 0x05, 0x62, 0x00, 0x01);
    TEST_ASSERT_EQUAL_UINT16(6, s_len);          /* ends after the status byte */
}

static void test_snapshot_of_unknown_dtc_is_out_of_range(void)
{
    REQ(0x19, 0x04, 0x12, 0x34, 0x56, 0x01);
    ASSERT_NRC(0x19, UDS_NRC_REQUEST_OUT_OF_RANGE);
}

static void test_extended_data_record(void)
{
    DtcManager_ReportResult(1, true, NULL);
    DtcManager_ReportResult(1, false, NULL);
    DtcManager_ReportResult(1, true, NULL);     /* second occurrence */

    REQ(0x19, 0x06, 0xC0, 0x01, 0x88, 0xFF);

    /* 59 06 | C00188 | status | 01 occurrence=2 | 02 aging=0 */
    TEST_ASSERT_EQUAL_UINT16(10, s_len);
    TEST_ASSERT_EQUAL_HEX8(0x01, s_rsp[6]);
    TEST_ASSERT_EQUAL_HEX8(0x02, s_rsp[7]);
    TEST_ASSERT_EQUAL_HEX8(0x02, s_rsp[8]);
    TEST_ASSERT_EQUAL_HEX8(0x00, s_rsp[9]);
}

static void test_unsupported_dtc_report_type(void)
{
    REQ(0x19, 0x03);
    ASSERT_NRC(0x19, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

/* ========================================================================= */
/* 0x14 ClearDiagnosticInformation and 0x85 ControlDTCSetting                */
/* ========================================================================= */

static void test_clear_all_dtcs(void)
{
    DtcManager_ReportResult(0, true, NULL);
    REQ(0x14, 0xFF, 0xFF, 0xFF);

    TEST_ASSERT_EQUAL_UINT16(1, s_len);
    TEST_ASSERT_EQUAL_HEX8(0x54, s_rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(0));
}

static void test_clear_unknown_group_is_out_of_range(void)
{
    REQ(0x14, 0x12, 0x34, 0x56);
    ASSERT_NRC(0x14, UDS_NRC_REQUEST_OUT_OF_RANGE);
}

static void test_clear_wrong_length_is_rejected(void)
{
    REQ(0x14, 0xFF, 0xFF);
    ASSERT_NRC(0x14, UDS_NRC_INCORRECT_MESSAGE_LENGTH);
}

static void test_dtc_setting_off_freezes_dtcs_until_default_session(void)
{
    enter_extended();
    REQ(0x85, 0x02);
    TEST_ASSERT_EQUAL_HEX8(0xC5, s_rsp[0]);

    DtcManager_ReportResult(0, true, NULL);
    TEST_ASSERT_EQUAL_HEX8(0x50, DtcManager_GetStatus(0));     /* frozen */

    REQ(0x10, 0x01);                    /* back to default re-enables it */
    DtcManager_ReportResult(0, true, NULL);
    TEST_ASSERT_EQUAL_HEX8(0xAF, DtcManager_GetStatus(0));
}

/* ========================================================================= */
/* 0x31 RoutineControl                                                       */
/* ========================================================================= */

static void test_routine_needs_extended_session(void)
{
    REQ(0x31, 0x01, 0x02, 0x02);
    ASSERT_NRC(0x31, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
}

static void test_protected_routine_needs_security(void)
{
    enter_extended();
    REQ(0x31, 0x01, 0x02, 0x01);
    ASSERT_NRC(0x31, UDS_NRC_SECURITY_ACCESS_DENIED);
    TEST_ASSERT_FALSE(s_routine_ran);
}

static void test_routine_start_and_results(void)
{
    unlock();

    REQ(0x31, 0x01, 0x02, 0x01);
    const uint8_t started[] = { 0x71, 0x01, 0x02, 0x01 };
    TEST_ASSERT_EQUAL_UINT16(4, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(started, s_rsp, 4);
    TEST_ASSERT_TRUE(s_routine_ran);

    REQ(0x31, 0x03, 0x02, 0x01);
    const uint8_t results[] = { 0x71, 0x03, 0x02, 0x01, 0x02 };
    TEST_ASSERT_EQUAL_UINT16(5, s_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(results, s_rsp, 5);
}

static void test_unknown_routine_is_out_of_range(void)
{
    enter_extended();
    REQ(0x31, 0x01, 0xAB, 0xCD);
    ASSERT_NRC(0x31, UDS_NRC_REQUEST_OUT_OF_RANGE);
}

static void test_unsupported_routine_control_type(void)
{
    enter_extended();
    REQ(0x31, 0x02, 0x02, 0x02);        /* routine 0x0202 has no stop */
    ASSERT_NRC(0x31, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);

    REQ(0x31, 0x04, 0x02, 0x02);        /* 0x04 is not a routine control type */
    ASSERT_NRC(0x31, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("UDS Server");

    RUN_TEST(test_starts_in_default_session_locked);
    RUN_TEST(test_unknown_service_is_rejected);
    RUN_TEST(test_service_in_wrong_session_is_rejected);
    RUN_TEST(test_missing_subfunction_is_a_length_error);

    RUN_TEST(test_extended_session_response_carries_timing);
    RUN_TEST(test_programming_session_is_not_supported);
    RUN_TEST(test_session_request_with_extra_byte_is_rejected);
    RUN_TEST(test_any_session_change_relocks_security);

    RUN_TEST(test_tester_present_responds);
    RUN_TEST(test_suppress_bit_silences_the_positive_response);
    RUN_TEST(test_suppress_bit_never_hides_an_error);
    RUN_TEST(test_s3_timeout_returns_to_default_session);
    RUN_TEST(test_tester_present_keeps_the_session_alive);

    RUN_TEST(test_read_single_did);
    RUN_TEST(test_read_several_dids_skips_unsupported_ones);
    RUN_TEST(test_read_only_unsupported_dids_is_out_of_range);
    RUN_TEST(test_read_odd_length_is_rejected);
    RUN_TEST(test_read_failure_is_reported);
    RUN_TEST(test_response_too_long_is_refused_not_overflowed);

    RUN_TEST(test_write_not_allowed_in_default_session);
    RUN_TEST(test_write_requires_security);
    RUN_TEST(test_write_unknown_did_is_out_of_range);
    RUN_TEST(test_write_wrong_length_is_rejected);
    RUN_TEST(test_write_invalid_value_is_rejected_by_the_application);
    RUN_TEST(test_write_succeeds_when_unlocked);

    RUN_TEST(test_correct_key_unlocks);
    RUN_TEST(test_seed_is_zero_when_already_unlocked);
    RUN_TEST(test_key_without_seed_is_a_sequence_error);
    RUN_TEST(test_wrong_key_is_rejected_and_consumes_the_seed);
    RUN_TEST(test_three_wrong_keys_trigger_a_lockout);
    RUN_TEST(test_unknown_security_level_is_rejected);

    RUN_TEST(test_hard_reset_is_acknowledged_then_requested);
    RUN_TEST(test_suppressed_reset_still_resets);
    RUN_TEST(test_unsupported_reset_type_is_rejected);

    RUN_TEST(test_functional_not_supported_errors_are_silent);
    RUN_TEST(test_functional_real_errors_are_still_reported);
    RUN_TEST(test_functional_request_gets_a_normal_positive_response);

    RUN_TEST(test_count_dtcs_by_status_mask);
    RUN_TEST(test_list_dtcs_by_status_mask);
    RUN_TEST(test_list_supported_dtcs);
    RUN_TEST(test_snapshot_record);
    RUN_TEST(test_snapshot_absent_when_dtc_never_failed);
    RUN_TEST(test_snapshot_of_unknown_dtc_is_out_of_range);
    RUN_TEST(test_extended_data_record);
    RUN_TEST(test_unsupported_dtc_report_type);

    RUN_TEST(test_clear_all_dtcs);
    RUN_TEST(test_clear_unknown_group_is_out_of_range);
    RUN_TEST(test_clear_wrong_length_is_rejected);
    RUN_TEST(test_dtc_setting_off_freezes_dtcs_until_default_session);

    RUN_TEST(test_routine_needs_extended_session);
    RUN_TEST(test_protected_routine_needs_security);
    RUN_TEST(test_routine_start_and_results);
    RUN_TEST(test_unknown_routine_is_out_of_range);
    RUN_TEST(test_unsupported_routine_control_type);

    return UnityEnd();
}
