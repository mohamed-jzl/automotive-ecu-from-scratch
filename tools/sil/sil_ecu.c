/**
 * @file    sil_ecu.c
 * @brief   Software-in-the-loop target: the REAL diagnostic stack, on a PC.
 *
 * What SIL means
 * --------------
 * Automotive verification climbs a ladder:
 *
 *      MIL  model-in-the-loop      a model of the behaviour (the ECU simulator
 *                                  in tools/can_tester/ecu_simulator.py)
 *      SIL  software-in-the-loop   the production C code, compiled for a PC
 *      HIL  hardware-in-the-loop   the production code on the real ECU
 *
 * This file is the SIL rung. It compiles the exact same diagnostic sources
 * that run on the STM32 - isotp.c, uds_server.c, dtc_manager.c,
 * uds_security.c, nvm_store.c, diag_manager.c - into a shared library.
 * Python loads it with ctypes and connects it to a virtual CAN bus, and the
 * system test suite then talks to it through udsoncan, an independent
 * open-source UDS client, exactly as it would talk to the real board.
 *
 * Two things make that valuable:
 *
 *   1. The C code is exercised end to end, through a third-party ISO-TP and
 *      UDS implementation, before any hardware exists. If our framing of a
 *      multi-frame response differs from what the standard means, udsoncan
 *      will refuse it - no amount of our own unit tests could show that.
 *
 *   2. It runs in CI. Every commit is checked against a real UDS client.
 *
 * What is simulated here, and only here
 * -------------------------------------
 *   - The application (battery voltage, vehicle state, faults): settable from
 *     Python, so tests can inject a fault on demand.
 *   - The flash: two RAM regions that obey flash programming rules. The
 *     "reset" keeps them, so DTC persistence across ECUReset is testable.
 *   - The CAN bus: frames go to a Python callback.
 * Everything under src/diag and src/services/nvm_store.c is the production code.
 */

#include "diag_manager.h"
#include "ecu_config.h"
#include "uds_defs.h"

#include <string.h>

#if defined(_WIN32)
#define SIL_API __declspec(dllexport)
#else
#define SIL_API __attribute__((visibility("default")))
#endif

/* ========================================================================= */
/* Simulated platform                                                        */
/* ========================================================================= */

typedef int (*SilTxFn_t)(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/* Smaller than on the ECU (128 KB) so tests that fill a region stay fast. */
#define SIL_REGION_SIZE     (16U * 1024U)

static uint8_t   s_flash[2][SIL_REGION_SIZE];
static SilTxFn_t s_tx_callback;
static uint32_t  s_now_ms;
static int       s_reset_requested;
static int       s_boot_count;

static bool sil_send_frame(uint32_t can_id, const uint8_t data[ISOTP_CAN_FRAME_SIZE],
                           uint8_t dlc, void *context)
{
    (void)context;
    return (s_tx_callback != NULL) && (s_tx_callback(can_id, data, dlc) != 0);
}

static void sil_system_reset(void)
{
    /* On the MCU this never returns. Here it only raises a flag; the Python
     * bridge sees it and "reboots" by calling sil_boot() again. */
    s_reset_requested = 1;
}

static bool sil_erase(uint8_t region, void *context)
{
    (void)context;
    memset(s_flash[region & 1U], 0xFF, SIL_REGION_SIZE);
    return true;
}

static bool sil_program(uint8_t region, uint32_t offset, uint32_t value, void *context)
{
    (void)context;

    if ((offset + 4U) > SIL_REGION_SIZE)
    {
        return false;
    }
    for (uint32_t i = 0U; i < 4U; i++)
    {
        /* Flash physics: programming can clear bits, never set them. */
        s_flash[region & 1U][offset + i] &= (uint8_t)((value >> (8U * i)) & 0xFFU);
    }
    return true;
}

static const NvmFlashIf_t SIL_FLASH =
{
    .region_base  = { s_flash[0], s_flash[1] },
    .region_size  = SIL_REGION_SIZE,
    .erase_region = sil_erase,
    .program_word = sil_program,
    .context      = NULL,
};

/* ========================================================================= */
/* Simulated application - mirrors src/app/diag_app.c                       */
/* ========================================================================= */

static uint16_t s_battery_mv    = 12600U;
static uint8_t  s_vehicle_state = 0U;
static uint16_t s_fault_bitmask = 0U;

static bool     s_lamp_ran;
static bool     s_lamp_active;
static uint32_t s_lamp_until_ms;

static void put_be16(uint8_t *out, uint16_t v) { out[0] = (uint8_t)(v >> 8); out[1] = (uint8_t)v; }

static uint8_t read_vin(uint8_t *out) { DiagManager_GetVin(out); return 0; }

static uint8_t write_vin(const uint8_t *in)
{
    for (uint8_t i = 0U; i < DIAG_VIN_LENGTH; i++)
    {
        const uint8_t c = in[i];
        const bool digit  = (c >= '0') && (c <= '9');
        const bool letter = (c >= 'A') && (c <= 'Z') && (c != 'I') && (c != 'O') && (c != 'Q');
        if (!digit && !letter) { return UDS_NRC_REQUEST_OUT_OF_RANGE; }
    }
    DiagManager_SetVin(in);
    return 0;
}

static uint8_t read_sw_version(uint8_t *out)
{
    out[0] = ECU_FW_VERSION_MAJOR; out[1] = ECU_FW_VERSION_MINOR; out[2] = ECU_FW_VERSION_PATCH;
    return 0;
}

static uint8_t read_serial(uint8_t *out)       { memcpy(out, "SIL-ECU-0001", 12); return 0; }
static uint8_t read_session(uint8_t *out)      { out[0] = (uint8_t)DiagManager_GetSession(); return 0; }
static uint8_t read_battery(uint8_t *out)      { put_be16(out, s_battery_mv); return 0; }
static uint8_t read_state(uint8_t *out)        { out[0] = s_vehicle_state; return 0; }
static uint8_t read_inputs(uint8_t *out)       { out[0] = 0U; return 0; }
static uint8_t read_faults(uint8_t *out)       { put_be16(out, s_fault_bitmask); return 0; }

static uint8_t read_uptime(uint8_t *out)
{
    const uint32_t s = s_now_ms / 1000U;
    out[0] = (uint8_t)(s >> 24); out[1] = (uint8_t)(s >> 16);
    out[2] = (uint8_t)(s >> 8);  out[3] = (uint8_t)s;
    return 0;
}

static const UdsDidEntry_t SIL_DIDS[] =
{
    { 0xF190, 17U, read_vin,        write_vin },
    { 0xF195,  3U, read_sw_version, NULL      },
    { 0xF18C, 12U, read_serial,     NULL      },
    { 0xF186,  1U, read_session,    NULL      },
    { 0x0100,  2U, read_battery,    NULL      },
    { 0x0101,  1U, read_state,      NULL      },
    { 0x0102,  1U, read_inputs,     NULL      },
    { 0x0103,  2U, read_faults,     NULL      },
    { 0x0104,  4U, read_uptime,     NULL      },
};

static bool lamp_active(void)
{
    if (s_lamp_active && ((int32_t)(s_now_ms - s_lamp_until_ms) >= 0))
    {
        s_lamp_active = false;
    }
    return s_lamp_active;
}

static uint8_t lamp_start(const uint8_t *in, uint16_t n, uint8_t *out, uint16_t size, uint16_t *len)
{
    (void)in; (void)n; (void)out; (void)size;
    if (s_vehicle_state == 3U) { return UDS_NRC_CONDITIONS_NOT_CORRECT; }   /* RUN */
    s_lamp_ran      = true;
    s_lamp_active   = true;
    s_lamp_until_ms = s_now_ms + ECU_LAMP_TEST_DURATION_MS;
    *len = 0U;
    return 0;
}

static uint8_t lamp_stop(const uint8_t *in, uint16_t n, uint8_t *out, uint16_t size, uint16_t *len)
{
    (void)in; (void)n; (void)out; (void)size;
    s_lamp_active = false;
    *len = 0U;
    return 0;
}

static uint8_t lamp_results(const uint8_t *in, uint16_t n, uint8_t *out, uint16_t size, uint16_t *len)
{
    (void)in; (void)n; (void)size;
    out[0] = (!s_lamp_ran) ? 0x00U : lamp_active() ? 0x01U : 0x02U;
    *len = 1U;
    return 0;
}

static const UdsRoutineEntry_t SIL_ROUTINES[] =
{
    { 0x0201, true, lamp_start, lamp_stop, lamp_results },
};

/* Same order as FaultId_t in fault_manager.h. */
static const DtcDefinition_t SIL_DTCS[] =
{
    { 0x056200UL, true  },  /* P0562-00  BATT_UNDERVOLT */
    { 0x056300UL, true  },  /* P0563-00  BATT_OVERVOLT  */
    { 0x900149UL, true  },  /* B1001-49  ADC_FAILURE    */
    { 0xC00188UL, false },  /* U0001-88  CAN_BUS_OFF    */
    { 0xC00100UL, false },  /* U0001-00  CAN_TX_FAIL    */
    { 0xF00049UL, false },  /* U3000-49  TASK_OVERRUN   */
};

#define SIL_DTC_COUNT  (sizeof(SIL_DTCS) / sizeof(SIL_DTCS[0]))

/* ========================================================================= */
/* Exported API (called from Python through ctypes)                          */
/* ========================================================================= */

SIL_API void sil_set_tx_callback(SilTxFn_t callback)
{
    s_tx_callback = callback;
}

/**
 * @brief Power up (or reboot) the simulated ECU.
 * @param erase_nvm  1 = factory-fresh flash; 0 = keep flash, like a real reset.
 * @return 1 if non-volatile storage came up, 0 otherwise.
 */
SIL_API int sil_boot(uint32_t now_ms, int erase_nvm)
{
    if (erase_nvm)
    {
        memset(s_flash, 0xFF, sizeof(s_flash));
    }

    /* A reset returns the WHOLE simulated ECU to its power-up state, not just
     * the diagnostic stack: on the real board the state machine restarts in
     * OFF and the battery is re-measured. Forgetting this once let a test that
     * set RUN leak into the next test (see docs/lessons_learned.md). */
    s_now_ms          = now_ms;
    s_reset_requested = 0;
    s_fault_bitmask   = 0U;
    s_battery_mv      = 12600U;
    s_vehicle_state   = 0U;
    s_lamp_ran        = false;
    s_lamp_active     = false;
    s_boot_count++;

    const DiagConfig_t config =
    {
        .send_frame    = sil_send_frame,
        .send_context  = NULL,
        .system_reset  = sil_system_reset,
        .nvm           = &SIL_FLASH,
        .entropy       = 0x5EED0000UL ^ now_ms,
        .dids          = SIL_DIDS,
        .did_count     = (uint8_t)(sizeof(SIL_DIDS) / sizeof(SIL_DIDS[0])),
        .routines      = SIL_ROUTINES,
        .routine_count = (uint8_t)(sizeof(SIL_ROUTINES) / sizeof(SIL_ROUTINES[0])),
        .dtcs          = SIL_DTCS,
        .dtc_count     = (uint8_t)SIL_DTC_COUNT,
    };

    return DiagManager_Init(&config, now_ms) ? 1 : 0;
}

SIL_API void sil_rx_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    s_now_ms = now_ms;
    (void)DiagManager_OnCanFrame(can_id, data, dlc, now_ms);
}

SIL_API void sil_poll(uint32_t now_ms)
{
    s_now_ms = now_ms;
    DiagManager_Poll(now_ms);
}

/** @return 1 once after the stack requested an ECU reset. */
SIL_API int sil_take_reset(void)
{
    const int requested = s_reset_requested;
    s_reset_requested = 0;
    return requested;
}

SIL_API int  sil_boot_count(void)                  { return s_boot_count; }
SIL_API void sil_set_battery_mv(uint16_t mv)        { s_battery_mv = mv; }
SIL_API void sil_set_vehicle_state(uint8_t state)   { s_vehicle_state = state; }
SIL_API void sil_start_operation_cycle(void)        { DiagManager_StartOperationCycle(); }
SIL_API int  sil_lamp_test_active(void)             { return lamp_active() ? 1 : 0; }

/**
 * @brief Inject one matured fault result, as body_control.c would report it.
 */
SIL_API void sil_report_fault(uint8_t fault, int failed)
{
    if (fault >= SIL_DTC_COUNT)
    {
        return;
    }

    if (failed) { s_fault_bitmask = (uint16_t)(s_fault_bitmask |  (1U << fault)); }
    else        { s_fault_bitmask = (uint16_t)(s_fault_bitmask & ~(1U << fault)); }

    const DtcSnapshot_t environment = { s_battery_mv, s_vehicle_state };
    DtcManager_ReportResult(fault, failed != 0, &environment);
}
