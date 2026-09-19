/**
 * @file    test_isotp.c
 * @brief   Unit tests for the ISO-TP (ISO 15765-2) transport layer.
 *
 * The ISO-TP module never touches a CAN controller: it receives frames through
 * IsoTp_OnFrame() and sends them through a function pointer. Here that pointer
 * leads to a fake that records every frame in an array, and can pretend the
 * CAN mailboxes are full. Time is just a number passed in.
 *
 * That makes it possible to test what is almost impossible to provoke on a
 * real bus on purpose: a lost Consecutive Frame, a tester that stops halfway
 * through a message, a receiver that answers WAIT eleven times, sequence
 * numbers wrapping from 15 back to 0.
 */

#include "unity_min.h"
#include "isotp.h"

#include <string.h>

/* ========================================================================= */
/* Fake CAN transmitter                                                      */
/* ========================================================================= */

#define RX_PHYS   0x7E0U
#define RX_FUNC   0x7DFU
#define TX_ID     0x7E8U
#define MAX_SENT  64

typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
} SentFrame_t;

static SentFrame_t s_sent[MAX_SENT];
static int         s_sent_count;
static int         s_refuse_next;       /* simulate "all mailboxes busy" */

static bool fake_send(uint32_t id, const uint8_t data[8], uint8_t dlc, void *context)
{
    (void)context;

    if (s_refuse_next > 0)
    {
        s_refuse_next--;
        return false;
    }

    if (s_sent_count < MAX_SENT)
    {
        s_sent[s_sent_count].id  = id;
        s_sent[s_sent_count].dlc = dlc;
        memcpy(s_sent[s_sent_count].data, data, 8);
        s_sent_count++;
    }
    return true;
}

static IsoTpLink_t s_link;

static void init_link(uint8_t block_size, uint8_t st_min)
{
    const IsoTpConfig_t config =
    {
        .rx_physical_id   = RX_PHYS,
        .rx_functional_id = RX_FUNC,
        .tx_id            = TX_ID,
        .padding_byte     = 0xCC,
        .block_size       = block_size,
        .st_min_ms        = st_min,
        .n_cr_timeout_ms  = 1000,
        .n_bs_timeout_ms  = 1000,
        .max_wait_frames  = 3,
        .send_frame       = fake_send,
        .send_context     = NULL,
    };
    IsoTp_Init(&s_link, &config);
}

void setUp(void)
{
    s_sent_count  = 0;
    s_refuse_next = 0;
    memset(s_sent, 0, sizeof(s_sent));
    init_link(0, 5);
}

void tearDown(void)
{
}

/** Feed an 8-byte frame. */
static void rx(uint32_t id, const uint8_t frame[8], uint32_t now)
{
    (void)IsoTp_OnFrame(&s_link, id, frame, 8, now);
}

/** Feed a complete multi-frame message as FF + CFs, like a tester would. */
static void rx_multi_frame(const uint8_t *msg, uint16_t length, uint32_t now)
{
    uint8_t frame[8];

    memset(frame, 0xCC, 8);
    frame[0] = (uint8_t)(0x10 | (length >> 8));
    frame[1] = (uint8_t)(length & 0xFF);
    memcpy(&frame[2], msg, 6);
    rx(RX_PHYS, frame, now);

    uint16_t offset = 6;
    uint8_t  sn     = 1;

    while (offset < length)
    {
        const uint16_t n = (uint16_t)((length - offset) < 7 ? (length - offset) : 7);

        memset(frame, 0xCC, 8);
        frame[0] = (uint8_t)(0x20 | sn);
        memcpy(&frame[1], &msg[offset], n);
        rx(RX_PHYS, frame, now);

        offset = (uint16_t)(offset + n);
        sn     = (uint8_t)((sn + 1) & 0x0F);
    }
}

/* ========================================================================= */
/* Receiving                                                                 */
/* ========================================================================= */

static void test_single_frame_is_received(void)
{
    const uint8_t frame[8] = { 0x03, 0x22, 0xF1, 0x90, 0xCC, 0xCC, 0xCC, 0xCC };
    rx(RX_PHYS, frame, 0);

    uint8_t  out[16];
    uint16_t length     = 0;
    bool     functional = true;

    TEST_ASSERT_TRUE(IsoTp_Receive(&s_link, out, sizeof(out), &length, &functional));
    TEST_ASSERT_EQUAL_UINT16(3, length);
    TEST_ASSERT_EQUAL_HEX8(0x22, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xF1, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x90, out[2]);
    TEST_ASSERT_FALSE(functional);
    TEST_ASSERT_EQUAL_INT(0, s_sent_count);     /* a SF needs no flow control */
}

static void test_functional_single_frame_is_flagged(void)
{
    const uint8_t frame[8] = { 0x02, 0x3E, 0x80, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };
    rx(RX_FUNC, frame, 0);

    uint8_t  out[8];
    uint16_t length     = 0;
    bool     functional = false;

    TEST_ASSERT_TRUE(IsoTp_Receive(&s_link, out, sizeof(out), &length, &functional));
    TEST_ASSERT_TRUE(functional);
}

static void test_foreign_identifier_is_not_consumed(void)
{
    const uint8_t frame[8] = { 0x02, 0x10, 0x03, 0, 0, 0, 0, 0 };

    TEST_ASSERT_FALSE(IsoTp_OnFrame(&s_link, 0x100, frame, 8, 0));
    TEST_ASSERT_TRUE(IsoTp_OnFrame(&s_link, RX_PHYS, frame, 8, 0));
}

static void test_invalid_single_frame_length_is_ignored(void)
{
    /* Length 0 (the CAN-FD escape) and length 8 are both illegal here. */
    const uint8_t zero[8]  = { 0x00, 0x22, 0, 0, 0, 0, 0, 0 };
    const uint8_t eight[8] = { 0x08, 0x22, 0, 0, 0, 0, 0, 0 };
    rx(RX_PHYS, zero, 0);
    rx(RX_PHYS, eight, 0);

    uint8_t  out[8];
    uint16_t length = 0;
    TEST_ASSERT_FALSE(IsoTp_Receive(&s_link, out, sizeof(out), &length, NULL));
}

static void test_first_frame_triggers_flow_control(void)
{
    init_link(4, 5);
    const uint8_t ff[8] = { 0x10, 0x14, 0x2E, 0xF1, 0x90, 0x57, 0x44, 0x42 };
    rx(RX_PHYS, ff, 0);

    /* Expected FC: [30 = continue][BS = 4][STmin = 5], padded with 0xCC. */
    const uint8_t expected[8] = { 0x30, 0x04, 0x05, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };

    TEST_ASSERT_EQUAL_INT(1, s_sent_count);
    TEST_ASSERT_EQUAL_UINT32(TX_ID, s_sent[0].id);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_sent[0].data, 8);
}

static void test_multi_frame_message_is_reassembled(void)
{
    uint8_t message[20];
    for (int i = 0; i < 20; i++) { message[i] = (uint8_t)(0xA0 + i); }

    rx_multi_frame(message, sizeof(message), 0);

    uint8_t  out[32];
    uint16_t length = 0;

    TEST_ASSERT_TRUE(IsoTp_Receive(&s_link, out, sizeof(out), &length, NULL));
    TEST_ASSERT_EQUAL_UINT16(20, length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(message, out, 20);
}

static void test_sequence_number_wraps_after_fifteen(void)
{
    /* 6 bytes in the FF + 7 x 18 CFs = 132 bytes: the CF sequence numbers run
     * 1..15, 0, 1, 2. A receiver that expected 16 after 15 would reject it. */
    uint8_t message[132];
    for (int i = 0; i < 132; i++) { message[i] = (uint8_t)i; }

    rx_multi_frame(message, sizeof(message), 0);

    uint8_t  out[200];
    uint16_t length = 0;

    TEST_ASSERT_TRUE(IsoTp_Receive(&s_link, out, sizeof(out), &length, NULL));
    TEST_ASSERT_EQUAL_UINT16(132, length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(message, out, 132);
}

static void test_wrong_sequence_number_discards_the_message(void)
{
    const uint8_t ff[8]  = { 0x10, 0x0A, 1, 2, 3, 4, 5, 6 };
    const uint8_t cf2[8] = { 0x22, 7, 8, 9, 10, 0xCC, 0xCC, 0xCC };  /* SN 2, but 1 expected */

    rx(RX_PHYS, ff, 0);
    rx(RX_PHYS, cf2, 1);

    uint8_t  out[16];
    uint16_t length = 0;

    /* Half a message must never reach the UDS layer. */
    TEST_ASSERT_FALSE(IsoTp_Receive(&s_link, out, sizeof(out), &length, NULL));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_WRONG_SN, IsoTp_GetLastError(&s_link));
}

static void test_missing_consecutive_frame_times_out(void)
{
    const uint8_t ff[8] = { 0x10, 0x0A, 1, 2, 3, 4, 5, 6 };
    rx(RX_PHYS, ff, 0);

    IsoTp_Poll(&s_link, 999);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_NONE, IsoTp_GetLastError(&s_link));

    IsoTp_Poll(&s_link, 1000);     /* N_Cr = 1000 ms has now expired */
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_TIMEOUT_CR, IsoTp_GetLastError(&s_link));

    /* A late CF must not resurrect the abandoned transfer. */
    const uint8_t cf1[8] = { 0x21, 7, 8, 9, 10, 0xCC, 0xCC, 0xCC };
    rx(RX_PHYS, cf1, 1001);

    uint8_t  out[16];
    uint16_t length = 0;
    TEST_ASSERT_FALSE(IsoTp_Receive(&s_link, out, sizeof(out), &length, NULL));
}

static void test_oversized_message_is_refused_with_overflow(void)
{
    /* 4000 bytes announced; our buffer is ECU_ISOTP_BUFFER_SIZE. */
    const uint8_t ff[8] = { 0x1F, 0xA0, 1, 2, 3, 4, 5, 6 };
    rx(RX_PHYS, ff, 0);

    TEST_ASSERT_EQUAL_INT(1, s_sent_count);
    TEST_ASSERT_EQUAL_HEX8(0x32, s_sent[0].data[0]);    /* FC status 2 = OVERFLOW */
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_RX_OVERFLOW, IsoTp_GetLastError(&s_link));
}

static void test_functional_first_frame_is_ignored(void)
{
    /* Functional requests must fit in one frame: with many ECUs listening,
     * nobody could agree on who sends the flow control. */
    const uint8_t ff[8] = { 0x10, 0x0A, 1, 2, 3, 4, 5, 6 };
    rx(RX_FUNC, ff, 0);

    TEST_ASSERT_EQUAL_INT(0, s_sent_count);
}

static void test_block_size_requests_flow_control_every_n_frames(void)
{
    init_link(2, 0);
    const uint8_t ff[8] = { 0x10, 0x1A, 1, 2, 3, 4, 5, 6 };  /* 26 bytes: FF + 3 CFs */
    rx(RX_PHYS, ff, 0);
    TEST_ASSERT_EQUAL_INT(1, s_sent_count);                   /* first FC */

    const uint8_t cf1[8] = { 0x21, 0, 0, 0, 0, 0, 0, 0 };
    const uint8_t cf2[8] = { 0x22, 0, 0, 0, 0, 0, 0, 0 };
    rx(RX_PHYS, cf1, 1);
    TEST_ASSERT_EQUAL_INT(1, s_sent_count);
    rx(RX_PHYS, cf2, 2);
    TEST_ASSERT_EQUAL_INT(2, s_sent_count);                   /* after BS = 2 CFs */
    TEST_ASSERT_EQUAL_HEX8(0x30, s_sent[1].data[0]);
}

static void test_receive_into_too_small_buffer_fails(void)
{
    const uint8_t frame[8] = { 0x05, 1, 2, 3, 4, 5, 0xCC, 0xCC };
    rx(RX_PHYS, frame, 0);

    uint8_t  out[3];
    uint16_t length = 0;
    TEST_ASSERT_FALSE(IsoTp_Receive(&s_link, out, sizeof(out), &length, NULL));
}

/* ========================================================================= */
/* Sending                                                                   */
/* ========================================================================= */

static void test_short_message_is_sent_as_single_frame(void)
{
    const uint8_t response[3] = { 0x50, 0x03, 0x00 };
    TEST_ASSERT_TRUE(IsoTp_Send(&s_link, response, 3, 0));

    const uint8_t expected[8] = { 0x03, 0x50, 0x03, 0x00, 0xCC, 0xCC, 0xCC, 0xCC };

    TEST_ASSERT_EQUAL_INT(1, s_sent_count);
    TEST_ASSERT_EQUAL_UINT32(TX_ID, s_sent[0].id);
    TEST_ASSERT_EQUAL_UINT8(8, s_sent[0].dlc);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, s_sent[0].data, 8);
    TEST_ASSERT_FALSE(IsoTp_IsTxBusy(&s_link));
}

static void test_long_message_waits_for_flow_control(void)
{
    uint8_t message[20];
    for (int i = 0; i < 20; i++) { message[i] = (uint8_t)i; }

    TEST_ASSERT_TRUE(IsoTp_Send(&s_link, message, 20, 0));

    /* Only the First Frame goes out: [10 14] + 6 bytes. */
    TEST_ASSERT_EQUAL_INT(1, s_sent_count);
    TEST_ASSERT_EQUAL_HEX8(0x10, s_sent[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x14, s_sent[0].data[1]);
    TEST_ASSERT_TRUE(IsoTp_IsTxBusy(&s_link));

    /* No FC yet: polling sends nothing more. */
    IsoTp_Poll(&s_link, 10);
    TEST_ASSERT_EQUAL_INT(1, s_sent_count);
}

static void test_flow_control_releases_consecutive_frames(void)
{
    uint8_t message[20];
    for (int i = 0; i < 20; i++) { message[i] = (uint8_t)i; }
    (void)IsoTp_Send(&s_link, message, 20, 0);

    const uint8_t fc[8] = { 0x30, 0x00, 0x00, 0, 0, 0, 0, 0 };   /* go, no limits */
    rx(RX_PHYS, fc, 5);
    IsoTp_Poll(&s_link, 5);

    /* FF + 2 CFs: 6 + 7 + 7 = 20 bytes. */
    TEST_ASSERT_EQUAL_INT(3, s_sent_count);
    TEST_ASSERT_EQUAL_HEX8(0x21, s_sent[1].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x06, s_sent[1].data[1]);     /* continues at byte 6 */
    TEST_ASSERT_EQUAL_HEX8(0x22, s_sent[2].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x13, s_sent[2].data[7]);     /* last byte = 19 */
    TEST_ASSERT_FALSE(IsoTp_IsTxBusy(&s_link));
}

static void test_st_min_spaces_consecutive_frames(void)
{
    uint8_t message[30];
    memset(message, 0x55, sizeof(message));
    (void)IsoTp_Send(&s_link, message, 30, 0);   /* FF + 4 CFs */

    const uint8_t fc[8] = { 0x30, 0x00, 0x0A, 0, 0, 0, 0, 0 };  /* STmin 10 ms */
    rx(RX_PHYS, fc, 0);

    IsoTp_Poll(&s_link, 0);
    TEST_ASSERT_EQUAL_INT(2, s_sent_count);      /* FF + CF1 */

    IsoTp_Poll(&s_link, 9);
    TEST_ASSERT_EQUAL_INT(2, s_sent_count);      /* too early */

    IsoTp_Poll(&s_link, 10);
    TEST_ASSERT_EQUAL_INT(3, s_sent_count);      /* CF2, exactly on time */
}

static void test_receiver_block_size_is_respected(void)
{
    uint8_t message[40];
    memset(message, 0x11, sizeof(message));
    (void)IsoTp_Send(&s_link, message, 40, 0);   /* FF + 5 CFs */

    const uint8_t fc[8] = { 0x30, 0x02, 0x00, 0, 0, 0, 0, 0 };  /* BS = 2 */
    rx(RX_PHYS, fc, 0);
    IsoTp_Poll(&s_link, 0);

    TEST_ASSERT_EQUAL_INT(3, s_sent_count);      /* FF + 2 CFs, then waits */

    IsoTp_Poll(&s_link, 1);
    TEST_ASSERT_EQUAL_INT(3, s_sent_count);

    rx(RX_PHYS, fc, 2);
    IsoTp_Poll(&s_link, 2);
    TEST_ASSERT_EQUAL_INT(5, s_sent_count);
}

static void test_missing_flow_control_times_out(void)
{
    uint8_t message[20] = {0};
    (void)IsoTp_Send(&s_link, message, 20, 0);

    IsoTp_Poll(&s_link, 1000);

    TEST_ASSERT_FALSE(IsoTp_IsTxBusy(&s_link));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_TIMEOUT_BS, IsoTp_GetLastError(&s_link));
}

static void test_too_many_wait_frames_abort_the_transfer(void)
{
    uint8_t message[20] = {0};
    (void)IsoTp_Send(&s_link, message, 20, 0);

    const uint8_t wait[8] = { 0x31, 0, 0, 0, 0, 0, 0, 0 };
    rx(RX_PHYS, wait, 1);
    rx(RX_PHYS, wait, 2);
    rx(RX_PHYS, wait, 3);
    TEST_ASSERT_TRUE(IsoTp_IsTxBusy(&s_link));   /* 3 WAITs allowed */

    rx(RX_PHYS, wait, 4);
    TEST_ASSERT_FALSE(IsoTp_IsTxBusy(&s_link));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_WAIT_LIMIT, IsoTp_GetLastError(&s_link));
}

static void test_overflow_flow_control_aborts_the_transfer(void)
{
    uint8_t message[20] = {0};
    (void)IsoTp_Send(&s_link, message, 20, 0);

    const uint8_t overflow[8] = { 0x32, 0, 0, 0, 0, 0, 0, 0 };
    rx(RX_PHYS, overflow, 1);

    TEST_ASSERT_FALSE(IsoTp_IsTxBusy(&s_link));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERROR_TX_OVERFLOW, IsoTp_GetLastError(&s_link));
}

static void test_busy_mailbox_is_retried_without_loss(void)
{
    uint8_t message[20];
    for (int i = 0; i < 20; i++) { message[i] = (uint8_t)i; }
    (void)IsoTp_Send(&s_link, message, 20, 0);

    const uint8_t fc[8] = { 0x30, 0x00, 0x00, 0, 0, 0, 0, 0 };
    rx(RX_PHYS, fc, 0);

    s_refuse_next = 1;                 /* CAN driver: "mailboxes full" once */
    IsoTp_Poll(&s_link, 0);
    TEST_ASSERT_EQUAL_INT(1, s_sent_count);

    IsoTp_Poll(&s_link, 1);            /* retried: CF1 then CF2, no gap, no duplicate */
    TEST_ASSERT_EQUAL_INT(3, s_sent_count);
    TEST_ASSERT_EQUAL_HEX8(0x21, s_sent[1].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x22, s_sent[2].data[0]);
}

static void test_second_send_while_busy_is_refused(void)
{
    uint8_t message[20] = {0};
    TEST_ASSERT_TRUE(IsoTp_Send(&s_link, message, 20, 0));
    TEST_ASSERT_FALSE(IsoTp_Send(&s_link, message, 3, 0));
}

static void test_invalid_send_arguments_are_refused(void)
{
    uint8_t message[300] = {0};
    TEST_ASSERT_FALSE(IsoTp_Send(&s_link, message, 0, 0));
    TEST_ASSERT_FALSE(IsoTp_Send(&s_link, message, 300, 0));   /* > buffer */
    TEST_ASSERT_FALSE(IsoTp_Send(&s_link, NULL, 3, 0));
}

static void test_st_min_decoding(void)
{
    TEST_ASSERT_EQUAL_UINT32(0,    IsoTp_DecodeStMin(0x00));
    TEST_ASSERT_EQUAL_UINT32(20,   IsoTp_DecodeStMin(0x14));
    TEST_ASSERT_EQUAL_UINT32(127,  IsoTp_DecodeStMin(0x7F));
    TEST_ASSERT_EQUAL_UINT32(1,    IsoTp_DecodeStMin(0xF1));  /* 100 us -> 1 ms */
    TEST_ASSERT_EQUAL_UINT32(1,    IsoTp_DecodeStMin(0xF9));  /* 900 us -> 1 ms */
    TEST_ASSERT_EQUAL_UINT32(127,  IsoTp_DecodeStMin(0x80));  /* reserved      */
    TEST_ASSERT_EQUAL_UINT32(127,  IsoTp_DecodeStMin(0xFA));  /* reserved      */
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("ISO-TP Transport Layer");

    RUN_TEST(test_single_frame_is_received);
    RUN_TEST(test_functional_single_frame_is_flagged);
    RUN_TEST(test_foreign_identifier_is_not_consumed);
    RUN_TEST(test_invalid_single_frame_length_is_ignored);
    RUN_TEST(test_first_frame_triggers_flow_control);
    RUN_TEST(test_multi_frame_message_is_reassembled);
    RUN_TEST(test_sequence_number_wraps_after_fifteen);
    RUN_TEST(test_wrong_sequence_number_discards_the_message);
    RUN_TEST(test_missing_consecutive_frame_times_out);
    RUN_TEST(test_oversized_message_is_refused_with_overflow);
    RUN_TEST(test_functional_first_frame_is_ignored);
    RUN_TEST(test_block_size_requests_flow_control_every_n_frames);
    RUN_TEST(test_receive_into_too_small_buffer_fails);

    RUN_TEST(test_short_message_is_sent_as_single_frame);
    RUN_TEST(test_long_message_waits_for_flow_control);
    RUN_TEST(test_flow_control_releases_consecutive_frames);
    RUN_TEST(test_st_min_spaces_consecutive_frames);
    RUN_TEST(test_receiver_block_size_is_respected);
    RUN_TEST(test_missing_flow_control_times_out);
    RUN_TEST(test_too_many_wait_frames_abort_the_transfer);
    RUN_TEST(test_overflow_flow_control_aborts_the_transfer);
    RUN_TEST(test_busy_mailbox_is_retried_without_loss);
    RUN_TEST(test_second_send_while_busy_is_refused);
    RUN_TEST(test_invalid_send_arguments_are_refused);
    RUN_TEST(test_st_min_decoding);

    return UnityEnd();
}
