/**
 * @file    can_signals.c
 * @brief   CAN payload encoding implementation - see can_signals.h.
 *
 * Pure module: no HAL, no hardware, no global state. Every function maps its
 * inputs onto its outputs deterministically, which is what allows
 * tests/test_can_signals.c to verify the exact byte layout of every message
 * on a host PC - including corruption cases that would be almost impossible
 * to provoke deliberately on a real bus.
 */

#include "can_signals.h"

#include <string.h>

/* ========================================================================= */
/* Bit positions within byte 0 of BCM_VehicleStatus                          */
/*                                                                           */
/*   bit:  7        6        5        4       3        2  1  0               */
/*        head   ind_rt   ind_lt   brake   ignition   vehicle_state          */
/* ========================================================================= */

#define VS_STATE_MASK           0x07U   /* bits 0-2 */
#define VS_IGNITION_BIT         0x08U   /* bit  3   */
#define VS_BRAKE_BIT            0x10U   /* bit  4   */
#define VS_INDICATOR_LEFT_BIT   0x20U   /* bit  5   */
#define VS_INDICATOR_RIGHT_BIT  0x40U   /* bit  6   */
#define VS_HEADLIGHT_BIT        0x80U   /* bit  7   */

/** Fault count occupies the low nibble of byte 3. */
#define VS_FAULT_COUNT_MASK     0x0FU

/** Engine-running flag occupies bit 0 of byte 5 in PCM_EngineStatus. */
#define ES_ENGINE_RUNNING_BIT   0x01U

/**
 * @brief Offset applied to coolant temperature before transmission.
 *
 * CAN signals are unsigned by default. Adding 40 shifts the automotive range
 * of -40 to +215 degrees C into 0..255, so the value fits one byte with no
 * sign handling. This "physical value = raw - offset" convention is used
 * throughout real DBC files.
 */
#define ES_COOLANT_TEMP_OFFSET  40

/* Number of payload bytes covered by the CRC: everything except the CRC byte. */
#define CRC_COVERAGE_BYTES      (CAN_MESSAGE_DLC - 1U)

/* ========================================================================= */
/* CRC                                                                       */
/* ========================================================================= */

uint8_t CanSignals_Crc8(const uint8_t *data, uint8_t length)
{
    if (data == NULL)
    {
        return 0U;
    }

    /* SAE J1850 / AUTOSAR E2E Profile 1 parameters. */
    const uint8_t polynomial = 0x1DU;
    uint8_t       crc        = 0xFFU;   /* non-zero init: a run of leading zero
                                         * bytes then changes the result, so an
                                         * all-zero payload is distinguishable
                                         * from a shorter one                   */

    for (uint8_t byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= data[byte_index];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            /* Standard bitwise long division in GF(2): if the leading bit is
             * set, shift and subtract (XOR) the polynomial; otherwise just
             * shift. */
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ polynomial);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return (uint8_t)(crc ^ 0xFFU);   /* final XOR */
}

/**
 * @brief Write the CRC of bytes 0..6 into byte 7.
 */
static void can_append_crc(uint8_t payload[CAN_MESSAGE_DLC])
{
    payload[CAN_CRC_BYTE] = CanSignals_Crc8(payload, CRC_COVERAGE_BYTES);
}

/**
 * @brief Verify that byte 7 matches the CRC of bytes 0..6.
 */
static bool can_verify_crc(const uint8_t payload[CAN_MESSAGE_DLC])
{
    const uint8_t expected = CanSignals_Crc8(payload, CRC_COVERAGE_BYTES);
    return (payload[CAN_CRC_BYTE] == expected);
}

/* ========================================================================= */
/* BCM_VehicleStatus (0x100)                                                 */
/* ========================================================================= */

void CanSignals_PackVehicleStatus(const CanVehicleStatus_t *src,
                                  uint8_t dst[CAN_MESSAGE_DLC])
{
    if ((src == NULL) || (dst == NULL))
    {
        return;
    }

    /* Start from a known payload. Without this, reserved bits would carry
     * whatever happened to be in the caller's buffer, and a receiver that
     * later starts using those bits would decode garbage. */
    (void)memset(dst, 0, CAN_MESSAGE_DLC);

    /* --- Byte 0: state and boolean outputs, one bit each ----------------- */
    uint8_t status_byte = (uint8_t)(src->vehicle_state & VS_STATE_MASK);

    if (src->ignition_on)     { status_byte |= VS_IGNITION_BIT;        }
    if (src->brake_active)    { status_byte |= VS_BRAKE_BIT;           }
    if (src->indicator_left)  { status_byte |= VS_INDICATOR_LEFT_BIT;  }
    if (src->indicator_right) { status_byte |= VS_INDICATOR_RIGHT_BIT; }
    if (src->headlight_on)    { status_byte |= VS_HEADLIGHT_BIT;       }

    dst[0] = status_byte;

    /* --- Bytes 1-2: battery millivolts, little-endian -------------------- */
    dst[1] = (uint8_t)(src->battery_mv & 0xFFU);
    dst[2] = (uint8_t)((src->battery_mv >> 8) & 0xFFU);

    /* --- Byte 3: fault count in the low nibble --------------------------- */
    dst[3] = (uint8_t)(src->fault_count & VS_FAULT_COUNT_MASK);

    /* --- Bytes 4-5: fault bitmask, little-endian ------------------------- */
    dst[4] = (uint8_t)(src->fault_bitmask & 0xFFU);
    dst[5] = (uint8_t)((src->fault_bitmask >> 8) & 0xFFU);

    /* --- Bytes 6-7: end-to-end protection -------------------------------- */
    dst[CAN_ALIVE_COUNTER_BYTE] = src->alive_counter;
    can_append_crc(dst);
}

bool CanSignals_UnpackVehicleStatus(const uint8_t src[CAN_MESSAGE_DLC],
                                    CanVehicleStatus_t *dst)
{
    if ((src == NULL) || (dst == NULL))
    {
        return false;
    }

    /* CRC first: never populate the output from a payload we do not trust.
     * A caller that ignores the return value then still sees its previous,
     * known-good data rather than corrupted values. */
    if (!can_verify_crc(src))
    {
        return false;
    }

    const uint8_t status_byte = src[0];

    dst->vehicle_state   = (uint8_t)(status_byte & VS_STATE_MASK);
    dst->ignition_on     = ((status_byte & VS_IGNITION_BIT)        != 0U);
    dst->brake_active    = ((status_byte & VS_BRAKE_BIT)           != 0U);
    dst->indicator_left  = ((status_byte & VS_INDICATOR_LEFT_BIT)  != 0U);
    dst->indicator_right = ((status_byte & VS_INDICATOR_RIGHT_BIT) != 0U);
    dst->headlight_on    = ((status_byte & VS_HEADLIGHT_BIT)       != 0U);

    dst->battery_mv    = (uint16_t)((uint16_t)src[1] | ((uint16_t)src[2] << 8));
    dst->fault_count   = (uint8_t)(src[3] & VS_FAULT_COUNT_MASK);
    dst->fault_bitmask = (uint16_t)((uint16_t)src[4] | ((uint16_t)src[5] << 8));
    dst->alive_counter = src[CAN_ALIVE_COUNTER_BYTE];

    return true;
}

/* ========================================================================= */
/* BCM_DIAGNOSTICS (0x101)                                                   */
/* ========================================================================= */

void CanSignals_PackDiagnostics(const CanDiagnostics_t *src,
                                uint8_t dst[CAN_MESSAGE_DLC])
{
    if ((src == NULL) || (dst == NULL))
    {
        return;
    }

    (void)memset(dst, 0, CAN_MESSAGE_DLC);

    dst[0] = (uint8_t)(src->uptime_seconds & 0xFFU);
    dst[1] = (uint8_t)((src->uptime_seconds >> 8) & 0xFFU);

    dst[2] = (uint8_t)(src->task_overrun_count & 0xFFU);
    dst[3] = (uint8_t)((src->task_overrun_count >> 8) & 0xFFU);

    dst[4] = src->max_task_duration_ms;
    dst[5] = src->can_tx_failures;

    dst[CAN_ALIVE_COUNTER_BYTE] = src->alive_counter;
    can_append_crc(dst);
}

bool CanSignals_UnpackDiagnostics(const uint8_t src[CAN_MESSAGE_DLC],
                                  CanDiagnostics_t *dst)
{
    if ((src == NULL) || (dst == NULL))
    {
        return false;
    }

    if (!can_verify_crc(src))
    {
        return false;
    }

    dst->uptime_seconds       = (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
    dst->task_overrun_count   = (uint16_t)((uint16_t)src[2] | ((uint16_t)src[3] << 8));
    dst->max_task_duration_ms = src[4];
    dst->can_tx_failures      = src[5];
    dst->alive_counter        = src[CAN_ALIVE_COUNTER_BYTE];

    return true;
}

/* ========================================================================= */
/* PCM_EngineStatus (0x200)                                                  */
/* ========================================================================= */

void CanSignals_PackEngineStatus(const CanEngineStatus_t *src,
                                 uint8_t dst[CAN_MESSAGE_DLC])
{
    if ((src == NULL) || (dst == NULL))
    {
        return;
    }

    (void)memset(dst, 0, CAN_MESSAGE_DLC);

    dst[0] = (uint8_t)(src->engine_rpm & 0xFFU);
    dst[1] = (uint8_t)((src->engine_rpm >> 8) & 0xFFU);

    dst[2] = (uint8_t)(src->vehicle_speed_kph_x10 & 0xFFU);
    dst[3] = (uint8_t)((src->vehicle_speed_kph_x10 >> 8) & 0xFFU);

    /* Apply the offset so the signed temperature fits an unsigned byte.
     * The intermediate is int16_t: -40 + 40 = 0 and 215 + 40 = 255 both fit,
     * but doing the arithmetic in int8_t would overflow for hot readings. */
    const int16_t offset_temp = (int16_t)src->coolant_temp_c + ES_COOLANT_TEMP_OFFSET;
    dst[4] = (uint8_t)(offset_temp & 0xFF);

    dst[5] = src->engine_running ? ES_ENGINE_RUNNING_BIT : 0U;

    dst[CAN_ALIVE_COUNTER_BYTE] = src->alive_counter;
    can_append_crc(dst);
}

bool CanSignals_UnpackEngineStatus(const uint8_t src[CAN_MESSAGE_DLC],
                                   CanEngineStatus_t *dst)
{
    if ((src == NULL) || (dst == NULL))
    {
        return false;
    }

    if (!can_verify_crc(src))
    {
        return false;
    }

    dst->engine_rpm            = (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
    dst->vehicle_speed_kph_x10 = (uint16_t)((uint16_t)src[2] | ((uint16_t)src[3] << 8));

    /* Reverse the offset to recover the signed physical value. */
    dst->coolant_temp_c = (int8_t)((int16_t)src[4] - ES_COOLANT_TEMP_OFFSET);

    dst->engine_running = ((src[5] & ES_ENGINE_RUNNING_BIT) != 0U);
    dst->alive_counter  = src[CAN_ALIVE_COUNTER_BYTE];

    return true;
}
