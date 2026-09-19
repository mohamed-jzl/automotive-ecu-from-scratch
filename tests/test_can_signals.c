/**
 * @file    test_can_signals.c
 * @brief   Unit tests for CAN payload encoding, decoding and E2E protection.
 *
 * For a network interface, the exact byte layout *is* the specification.
 * Another ECU - written by another team, possibly at another company -
 * decodes these bytes according to docs/can_specification.md. If a
 * refactoring silently moves a signal by one bit, this ECU keeps working
 * perfectly and the other one starts reading nonsense.
 *
 * These tests therefore assert against hard-coded byte arrays rather than
 * against the encoder's own output. Expected values were computed
 * independently in Python (tools/can_tester/ecu_signals.py) from the written
 * specification, so a bug in the C encoder cannot hide by also appearing in
 * the expectation.
 *
 * The CRC implementation is additionally checked against the published check
 * value for CRC-8/SAE-J1850, which pins it to the standard rather than to
 * our own interpretation of it.
 */

#include "unity_min.h"
#include "can_signals.h"

#include <string.h>

void setUp(void)
{
}

void tearDown(void)
{
}

/* ========================================================================= */
/* CRC                                                                       */
/* ========================================================================= */

static void test_crc_matches_the_published_check_value(void)
{
    /* Every CRC specification publishes a "check" value: the CRC of the
     * ASCII string "123456789". For CRC-8/SAE-J1850 it is 0x4B.
     *
     * This single assertion proves the polynomial, the initial value, the
     * final XOR and the bit ordering are all correct together - which
     * testing against our own output never could. */
    const uint8_t check_input[] = "123456789";

    TEST_ASSERT_EQUAL_HEX8(0x4B, CanSignals_Crc8(check_input, 9U));
}

static void test_crc_of_known_buffers(void)
{
    const uint8_t zeros[7] = { 0, 0, 0, 0, 0, 0, 0 };
    const uint8_t ramp[7]  = { 1, 2, 3, 4, 5, 6, 7 };

    TEST_ASSERT_EQUAL_HEX8(0x0A, CanSignals_Crc8(zeros, 7U));
    TEST_ASSERT_EQUAL_HEX8(0x44, CanSignals_Crc8(ramp,  7U));
}

static void test_crc_detects_every_single_bit_error(void)
{
    /* CRC-8/SAE-J1850 has a Hamming distance of 4 over an 8-byte payload,
     * meaning it detects all 1-, 2- and 3-bit errors. Verifying the
     * single-bit case exhaustively - all 7 bytes x 8 bits = 56 flips -
     * demonstrates the property that matters most in practice. */
    uint8_t original[7] = { 0x9B, 0x38, 0x31, 0x00, 0x00, 0x00, 0x07 };
    const uint8_t reference = CanSignals_Crc8(original, 7U);

    for (int byte_index = 0; byte_index < 7; byte_index++)
    {
        for (int bit = 0; bit < 8; bit++)
        {
            uint8_t corrupted[7];
            memcpy(corrupted, original, sizeof(corrupted));
            corrupted[byte_index] ^= (uint8_t)(1U << bit);

            TEST_ASSERT_TRUE(CanSignals_Crc8(corrupted, 7U) != reference);
        }
    }
}

static void test_crc_of_null_pointer_is_safe(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x00, CanSignals_Crc8(NULL, 8U));
}

/* ========================================================================= */
/* BCM_VehicleStatus - exact byte layout                                     */
/* ========================================================================= */

static void test_vehicle_status_byte_layout_running(void)
{
    /* Scenario: engine running, ignition and brake active, headlights on,
     * battery at 12.600 V, no faults, alive counter 7.
     *
     * Expected payload derived from the specification:
     *   byte 0 = state 3 | ignition(0x08) | brake(0x10) | headlight(0x80)
     *          = 0x03 | 0x08 | 0x10 | 0x80 = 0x9B
     *   byte 1..2 = 12600 = 0x3138 little-endian -> 0x38, 0x31
     *   byte 6 = alive = 0x07
     *   byte 7 = CRC of bytes 0..6 = 0x07
     */
    const CanVehicleStatus_t input =
    {
        .vehicle_state   = 3U,      /* RUN */
        .ignition_on     = true,
        .brake_active    = true,
        .indicator_left  = false,
        .indicator_right = false,
        .headlight_on    = true,
        .battery_mv      = 12600U,
        .fault_count     = 0U,
        .fault_bitmask   = 0x0000U,
        .alive_counter   = 7U
    };

    const uint8_t expected[8] = { 0x9B, 0x38, 0x31, 0x00, 0x00, 0x00, 0x07, 0x07 };

    uint8_t actual[8];
    CanSignals_PackVehicleStatus(&input, actual);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, 8);
}

static void test_vehicle_status_byte_layout_faulted(void)
{
    /* Scenario: FAULT state, all outputs off, battery at 10.500 V,
     * two faults latched (bits 0 and 1), alive counter 42. */
    const CanVehicleStatus_t input =
    {
        .vehicle_state   = 4U,      /* FAULT */
        .ignition_on     = false,
        .brake_active    = false,
        .indicator_left  = false,
        .indicator_right = false,
        .headlight_on    = false,
        .battery_mv      = 10500U,
        .fault_count     = 2U,
        .fault_bitmask   = 0x0003U,
        .alive_counter   = 42U
    };

    const uint8_t expected[8] = { 0x04, 0x04, 0x29, 0x02, 0x03, 0x00, 0x2A, 0x44 };

    uint8_t actual[8];
    CanSignals_PackVehicleStatus(&input, actual);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, 8);
}

static void test_each_status_bit_occupies_its_own_position(void)
{
    /* Sets one flag at a time and checks that exactly the specified bit
     * appears. Catches the classic bug where two signals are accidentally
     * given the same bit position - which would work perfectly in every
     * test that only ever sets one of them. */
    CanVehicleStatus_t input;
    uint8_t            payload[8];

    memset(&input, 0, sizeof(input));
    input.ignition_on = true;
    CanSignals_PackVehicleStatus(&input, payload);
    TEST_ASSERT_EQUAL_HEX8(0x08, payload[0]);

    memset(&input, 0, sizeof(input));
    input.brake_active = true;
    CanSignals_PackVehicleStatus(&input, payload);
    TEST_ASSERT_EQUAL_HEX8(0x10, payload[0]);

    memset(&input, 0, sizeof(input));
    input.indicator_left = true;
    CanSignals_PackVehicleStatus(&input, payload);
    TEST_ASSERT_EQUAL_HEX8(0x20, payload[0]);

    memset(&input, 0, sizeof(input));
    input.indicator_right = true;
    CanSignals_PackVehicleStatus(&input, payload);
    TEST_ASSERT_EQUAL_HEX8(0x40, payload[0]);

    memset(&input, 0, sizeof(input));
    input.headlight_on = true;
    CanSignals_PackVehicleStatus(&input, payload);
    TEST_ASSERT_EQUAL_HEX8(0x80, payload[0]);
}

static void test_vehicle_state_is_confined_to_three_bits(void)
{
    /* A caller passing an out-of-range state must not corrupt the flag bits
     * that share byte 0 with it. */
    CanVehicleStatus_t input;
    memset(&input, 0, sizeof(input));
    input.vehicle_state = 0xFFU;

    uint8_t payload[8];
    CanSignals_PackVehicleStatus(&input, payload);

    TEST_ASSERT_EQUAL_HEX8(0x07, payload[0]);
}

/* ========================================================================= */
/* Round trips                                                               */
/* ========================================================================= */

static void test_vehicle_status_round_trip(void)
{
    const CanVehicleStatus_t original =
    {
        .vehicle_state   = 2U,
        .ignition_on     = true,
        .brake_active    = false,
        .indicator_left  = true,
        .indicator_right = false,
        .headlight_on    = true,
        .battery_mv      = 13800U,
        .fault_count     = 3U,
        .fault_bitmask   = 0x0015U,
        .alive_counter   = 200U
    };

    uint8_t payload[8];
    CanSignals_PackVehicleStatus(&original, payload);

    CanVehicleStatus_t decoded;
    TEST_ASSERT_TRUE(CanSignals_UnpackVehicleStatus(payload, &decoded));

    TEST_ASSERT_EQUAL_UINT8(original.vehicle_state,   decoded.vehicle_state);
    TEST_ASSERT_EQUAL_INT(original.ignition_on,       decoded.ignition_on);
    TEST_ASSERT_EQUAL_INT(original.brake_active,      decoded.brake_active);
    TEST_ASSERT_EQUAL_INT(original.indicator_left,    decoded.indicator_left);
    TEST_ASSERT_EQUAL_INT(original.indicator_right,   decoded.indicator_right);
    TEST_ASSERT_EQUAL_INT(original.headlight_on,      decoded.headlight_on);
    TEST_ASSERT_EQUAL_UINT16(original.battery_mv,     decoded.battery_mv);
    TEST_ASSERT_EQUAL_UINT8(original.fault_count,     decoded.fault_count);
    TEST_ASSERT_EQUAL_HEX16(original.fault_bitmask,   decoded.fault_bitmask);
    TEST_ASSERT_EQUAL_UINT8(original.alive_counter,   decoded.alive_counter);
}

static void test_diagnostics_round_trip(void)
{
    const CanDiagnostics_t original =
    {
        .uptime_seconds       = 3600U,
        .task_overrun_count   = 12U,
        .max_task_duration_ms = 4U,
        .can_tx_failures      = 7U,
        .alive_counter        = 99U
    };

    uint8_t payload[8];
    CanSignals_PackDiagnostics(&original, payload);

    CanDiagnostics_t decoded;
    TEST_ASSERT_TRUE(CanSignals_UnpackDiagnostics(payload, &decoded));

    TEST_ASSERT_EQUAL_UINT16(original.uptime_seconds,       decoded.uptime_seconds);
    TEST_ASSERT_EQUAL_UINT16(original.task_overrun_count,   decoded.task_overrun_count);
    TEST_ASSERT_EQUAL_UINT8(original.max_task_duration_ms,  decoded.max_task_duration_ms);
    TEST_ASSERT_EQUAL_UINT8(original.can_tx_failures,       decoded.can_tx_failures);
    TEST_ASSERT_EQUAL_UINT8(original.alive_counter,         decoded.alive_counter);
}

static void test_engine_status_round_trip(void)
{
    const CanEngineStatus_t original =
    {
        .engine_rpm            = 2500U,
        .vehicle_speed_kph_x10 = 875U,      /* 87.5 km/h */
        .coolant_temp_c        = 90,
        .engine_running        = true,
        .alive_counter         = 5U
    };

    uint8_t payload[8];
    CanSignals_PackEngineStatus(&original, payload);

    CanEngineStatus_t decoded;
    TEST_ASSERT_TRUE(CanSignals_UnpackEngineStatus(payload, &decoded));

    TEST_ASSERT_EQUAL_UINT16(original.engine_rpm,            decoded.engine_rpm);
    TEST_ASSERT_EQUAL_UINT16(original.vehicle_speed_kph_x10, decoded.vehicle_speed_kph_x10);
    TEST_ASSERT_EQUAL_INT8(original.coolant_temp_c,          decoded.coolant_temp_c);
    TEST_ASSERT_EQUAL_INT(original.engine_running,           decoded.engine_running);
}

/* ========================================================================= */
/* Signed value handling                                                     */
/* ========================================================================= */

static void test_negative_coolant_temperature_survives_the_round_trip(void)
{
    /* CAN signals are unsigned, so the encoder applies a +40 offset. This is
     * the classic place for a sign bug: a cold-start reading of -30 degrees
     * decoding as +226 would be invisible in a warm workshop and obvious in
     * a Moroccan winter morning. */
    CanEngineStatus_t original;
    memset(&original, 0, sizeof(original));
    original.coolant_temp_c = -30;

    uint8_t payload[8];
    CanSignals_PackEngineStatus(&original, payload);

    /* Raw byte must be -30 + 40 = 10. */
    TEST_ASSERT_EQUAL_HEX8(0x0A, payload[4]);

    CanEngineStatus_t decoded;
    TEST_ASSERT_TRUE(CanSignals_UnpackEngineStatus(payload, &decoded));
    TEST_ASSERT_EQUAL_INT8(-30, decoded.coolant_temp_c);
}

static void test_coolant_temperature_range_limits(void)
{
    const int8_t limits[] = { -40, -1, 0, 1, 100, 127 };

    for (unsigned i = 0; i < sizeof(limits) / sizeof(limits[0]); i++)
    {
        CanEngineStatus_t original;
        memset(&original, 0, sizeof(original));
        original.coolant_temp_c = limits[i];

        uint8_t payload[8];
        CanSignals_PackEngineStatus(&original, payload);

        CanEngineStatus_t decoded;
        TEST_ASSERT_TRUE(CanSignals_UnpackEngineStatus(payload, &decoded));
        TEST_ASSERT_EQUAL_INT8(limits[i], decoded.coolant_temp_c);
    }
}

/* ========================================================================= */
/* End-to-end protection                                                     */
/* ========================================================================= */

static void test_corrupted_payload_is_rejected(void)
{
    CanVehicleStatus_t original;
    memset(&original, 0, sizeof(original));
    original.battery_mv = 12000U;

    uint8_t payload[8];
    CanSignals_PackVehicleStatus(&original, payload);

    /* Flip one bit, exactly as a marginal transceiver or a faulty gateway
     * might. The decoder must refuse the frame. */
    payload[1] ^= 0x01U;

    CanVehicleStatus_t decoded;
    TEST_ASSERT_FALSE(CanSignals_UnpackVehicleStatus(payload, &decoded));
}

static void test_rejected_payload_does_not_modify_the_output(void)
{
    /* Critical safety property: a receiver that ignores the return value
     * must still be left holding its previous, known-good data rather than
     * a half-decoded corrupt frame. */
    CanVehicleStatus_t previous;
    memset(&previous, 0, sizeof(previous));
    previous.battery_mv    = 12345U;
    previous.vehicle_state = 3U;

    uint8_t corrupt[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };

    TEST_ASSERT_FALSE(CanSignals_UnpackVehicleStatus(corrupt, &previous));

    TEST_ASSERT_EQUAL_UINT16(12345U, previous.battery_mv);
    TEST_ASSERT_EQUAL_UINT8(3U,      previous.vehicle_state);
}

static void test_alive_counter_is_carried_through_unchanged(void)
{
    for (unsigned counter = 0; counter < 256U; counter++)
    {
        CanVehicleStatus_t original;
        memset(&original, 0, sizeof(original));
        original.alive_counter = (uint8_t)counter;

        uint8_t payload[8];
        CanSignals_PackVehicleStatus(&original, payload);

        CanVehicleStatus_t decoded;
        TEST_ASSERT_TRUE(CanSignals_UnpackVehicleStatus(payload, &decoded));
        TEST_ASSERT_EQUAL_UINT8((uint8_t)counter, decoded.alive_counter);
    }
}

/* ========================================================================= */
/* Robustness                                                                */
/* ========================================================================= */

static void test_null_arguments_are_handled_safely(void)
{
    uint8_t            payload[8] = {0};
    CanVehicleStatus_t status     = {0};

    /* None of these may dereference NULL. */
    CanSignals_PackVehicleStatus(NULL, payload);
    CanSignals_PackVehicleStatus(&status, NULL);

    TEST_ASSERT_FALSE(CanSignals_UnpackVehicleStatus(NULL, &status));
    TEST_ASSERT_FALSE(CanSignals_UnpackVehicleStatus(payload, NULL));
    TEST_ASSERT_FALSE(CanSignals_UnpackDiagnostics(NULL, NULL));
    TEST_ASSERT_FALSE(CanSignals_UnpackEngineStatus(NULL, NULL));
}

static void test_reserved_bytes_are_zeroed(void)
{
    /* The encoder must not leak whatever was previously in the caller's
     * buffer into bits the specification has not assigned yet. */
    uint8_t payload[8];
    memset(payload, 0xAA, sizeof(payload));

    CanVehicleStatus_t input;
    memset(&input, 0, sizeof(input));

    CanSignals_PackVehicleStatus(&input, payload);

    /* Byte 3 holds only a 4-bit fault count; the upper nibble is reserved. */
    TEST_ASSERT_EQUAL_HEX8(0x00, payload[3]);
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("CAN Signal Encoding");

    RUN_TEST(test_crc_matches_the_published_check_value);
    RUN_TEST(test_crc_of_known_buffers);
    RUN_TEST(test_crc_detects_every_single_bit_error);
    RUN_TEST(test_crc_of_null_pointer_is_safe);

    RUN_TEST(test_vehicle_status_byte_layout_running);
    RUN_TEST(test_vehicle_status_byte_layout_faulted);
    RUN_TEST(test_each_status_bit_occupies_its_own_position);
    RUN_TEST(test_vehicle_state_is_confined_to_three_bits);

    RUN_TEST(test_vehicle_status_round_trip);
    RUN_TEST(test_diagnostics_round_trip);
    RUN_TEST(test_engine_status_round_trip);

    RUN_TEST(test_negative_coolant_temperature_survives_the_round_trip);
    RUN_TEST(test_coolant_temperature_range_limits);

    RUN_TEST(test_corrupted_payload_is_rejected);
    RUN_TEST(test_rejected_payload_does_not_modify_the_output);
    RUN_TEST(test_alive_counter_is_carried_through_unchanged);

    RUN_TEST(test_null_arguments_are_handled_safely);
    RUN_TEST(test_reserved_bytes_are_zeroed);

    return UnityEnd();
}
