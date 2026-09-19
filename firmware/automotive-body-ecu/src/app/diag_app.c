/**
 * @file    diag_app.c
 * @brief   Body Control ECU diagnostics content and hardware bindings.
 *
 * Layer note: this is application code, so it reaches hardware only through
 * drivers (CanDriver, FlashDriver, McuDriver) - never through the HAL - with
 * the usual exception of HAL_GetTick() for time.
 */

#include "diag_app.h"

#include "body_control.h"
#include "vehicle_state_machine.h"

#include "can_manager.h"
#include "diag_manager.h"
#include "fault_manager.h"

#include "can_driver.h"
#include "flash_driver.h"
#include "gpio_driver.h"
#include "mcu_driver.h"

#include "ecu_config.h"
#include "stm32f4xx_hal.h"

#include <string.h>

/* ========================================================================= */
/* Data identifiers                                                          */
/* ========================================================================= */

#define DID_VIN                 0xF190U
#define DID_SOFTWARE_VERSION    0xF195U
#define DID_ECU_SERIAL          0xF18CU
#define DID_ACTIVE_SESSION      0xF186U
#define DID_BATTERY_VOLTAGE     0x0100U
#define DID_VEHICLE_STATE       0x0101U
#define DID_DIGITAL_INPUTS      0x0102U
#define DID_FAULT_BITMASK       0x0103U
#define DID_UPTIME              0x0104U

#define RID_LAMP_SELF_TEST      0x0201U

static void put_be16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)(value & 0xFFU);
}

static uint8_t read_vin(uint8_t *out)
{
    DiagManager_GetVin(out);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/**
 * @brief Accept a VIN only if every character is legal under ISO 3779.
 *
 * Digits and capital letters, except I, O and Q - excluded because they are
 * too easily confused with 1 and 0. Validating at the point of entry means a
 * typo during end-of-line programming is rejected with NRC 0x31 immediately,
 * instead of being stored and discovered at the first workshop visit.
 */
static uint8_t write_vin(const uint8_t *in)
{
    for (uint8_t i = 0U; i < DIAG_VIN_LENGTH; i++)
    {
        const uint8_t c = in[i];
        const bool digit  = (c >= (uint8_t)'0') && (c <= (uint8_t)'9');
        const bool letter = (c >= (uint8_t)'A') && (c <= (uint8_t)'Z') &&
                            (c != (uint8_t)'I') && (c != (uint8_t)'O') && (c != (uint8_t)'Q');

        if ((!digit) && (!letter))
        {
            return UDS_NRC_REQUEST_OUT_OF_RANGE;
        }
    }

    DiagManager_SetVin(in);
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_software_version(uint8_t *out)
{
    out[0] = ECU_FW_VERSION_MAJOR;
    out[1] = ECU_FW_VERSION_MINOR;
    out[2] = ECU_FW_VERSION_PATCH;
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_ecu_serial(uint8_t *out)
{
    McuDriver_ReadUniqueId(out);
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_active_session(uint8_t *out)
{
    out[0] = (uint8_t)DiagManager_GetSession();
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_battery_voltage(uint8_t *out)
{
    put_be16(out, (uint16_t)BodyControl_GetBatteryMv());
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_vehicle_state(uint8_t *out)
{
    out[0] = (uint8_t)VehicleStateMachine_GetState();
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_digital_inputs(uint8_t *out)
{
    uint8_t bits = 0U;

    if (GpioDriver_GetInput(DIN_IGNITION))   { bits |= 0x01U; }
    if (GpioDriver_GetInput(DIN_BRAKE))      { bits |= 0x02U; }
    if (GpioDriver_GetInput(DIN_TURN_LEFT))  { bits |= 0x04U; }
    if (GpioDriver_GetInput(DIN_TURN_RIGHT)) { bits |= 0x08U; }

    out[0] = bits;
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_fault_bitmask(uint8_t *out)
{
    put_be16(out, FaultManager_GetActiveBitmask());
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t read_uptime(uint8_t *out)
{
    const uint32_t seconds = HAL_GetTick() / 1000U;

    out[0] = (uint8_t)(seconds >> 24);
    out[1] = (uint8_t)(seconds >> 16);
    out[2] = (uint8_t)(seconds >> 8);
    out[3] = (uint8_t)(seconds & 0xFFU);
    return UDS_NRC_POSITIVE_RESPONSE;
}

static const UdsDidEntry_t DID_TABLE[] =
{
    /* DID                    len  read                    write     */
    { DID_VIN,                17U, read_vin,               write_vin },
    { DID_SOFTWARE_VERSION,    3U, read_software_version,  NULL      },
    { DID_ECU_SERIAL,         12U, read_ecu_serial,        NULL      },
    { DID_ACTIVE_SESSION,      1U, read_active_session,    NULL      },
    { DID_BATTERY_VOLTAGE,     2U, read_battery_voltage,   NULL      },
    { DID_VEHICLE_STATE,       1U, read_vehicle_state,     NULL      },
    { DID_DIGITAL_INPUTS,      1U, read_digital_inputs,    NULL      },
    { DID_FAULT_BITMASK,       2U, read_fault_bitmask,     NULL      },
    { DID_UPTIME,              4U, read_uptime,            NULL      },
};

/* ========================================================================= */
/* Routine: lamp self-test (RID 0x0201)                                      */
/*                                                                           */
/* Lights every exterior lamp for ECU_LAMP_TEST_DURATION_MS so a technician  */
/* can check the bulbs and wiring without driving the vehicle. This is an    */
/* actuator test - the ECU drives outputs on command - so it requires        */
/* security access, and it is refused while the engine runs.                 */
/* ========================================================================= */

#define LAMP_TEST_NEVER_RUN     0x00U
#define LAMP_TEST_RUNNING       0x01U
#define LAMP_TEST_COMPLETED     0x02U

static bool     s_lamp_test_ran       = false;
static bool     s_lamp_test_active    = false;
static uint32_t s_lamp_test_until_ms  = 0U;

static uint8_t lamp_test_start(const uint8_t *in, uint16_t in_length,
                               uint8_t *out, uint16_t out_size, uint16_t *out_length)
{
    (void)in; (void)in_length; (void)out; (void)out_size;

    /* Flashing every lamp on a moving vehicle would confuse other drivers:
     * the test is only allowed with the engine stopped. */
    if (VehicleStateMachine_GetState() == VEHICLE_STATE_RUN)
    {
        return UDS_NRC_CONDITIONS_NOT_CORRECT;
    }

    s_lamp_test_ran      = true;
    s_lamp_test_active   = true;
    s_lamp_test_until_ms = HAL_GetTick() + ECU_LAMP_TEST_DURATION_MS;
    *out_length          = 0U;
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t lamp_test_stop(const uint8_t *in, uint16_t in_length,
                              uint8_t *out, uint16_t out_size, uint16_t *out_length)
{
    (void)in; (void)in_length; (void)out; (void)out_size;

    s_lamp_test_active = false;
    *out_length        = 0U;
    return UDS_NRC_POSITIVE_RESPONSE;
}

static uint8_t lamp_test_results(const uint8_t *in, uint16_t in_length,
                                 uint8_t *out, uint16_t out_size, uint16_t *out_length)
{
    (void)in; (void)in_length;

    if (out_size < 1U)
    {
        return UDS_NRC_RESPONSE_TOO_LONG;
    }

    out[0] = (!s_lamp_test_ran)       ? LAMP_TEST_NEVER_RUN :
             DiagApp_IsLampTestActive() ? LAMP_TEST_RUNNING : LAMP_TEST_COMPLETED;
    *out_length = 1U;
    return UDS_NRC_POSITIVE_RESPONSE;
}

static const UdsRoutineEntry_t ROUTINE_TABLE[] =
{
    { RID_LAMP_SELF_TEST, true, lamp_test_start, lamp_test_stop, lamp_test_results },
};

/* ========================================================================= */
/* DTC table - index MUST match FaultId_t                                    */
/* ========================================================================= */

static const DtcDefinition_t DTC_TABLE[FAULT_COUNT] =
{
    /* Designated initialisers tie each row to its FaultId_t, so reordering the
     * enum can never silently attach a fault to the wrong DTC. */
    [FAULT_BATTERY_UNDERVOLTAGE] = { 0x056200UL, true  },   /* P0562-00 */
    [FAULT_BATTERY_OVERVOLTAGE]  = { 0x056300UL, true  },   /* P0563-00 */
    [FAULT_ADC_FAILURE]          = { 0x900149UL, true  },   /* B1001-49 */
    [FAULT_CAN_BUS_OFF]          = { 0xC00188UL, false },   /* U0001-88 */
    [FAULT_CAN_TX_FAILURE]       = { 0xC00100UL, false },   /* U0001-00 */
    [FAULT_TASK_OVERRUN]         = { 0xF00049UL, false },   /* U3000-49 */
};

/* ========================================================================= */
/* Hardware bindings                                                         */
/* ========================================================================= */

static bool bind_send_frame(uint32_t can_id, const uint8_t data[ISOTP_CAN_FRAME_SIZE],
                            uint8_t dlc, void *context)
{
    (void)context;

    CanFrame_t frame;
    frame.id  = can_id;
    frame.dlc = dlc;
    (void)memcpy(frame.data, data, dlc);

    return CanDriver_Transmit(&frame);
}

static const uint32_t NVM_REGION_ADDRESS[2] =
{
    ECU_NVM_REGION_A_ADDRESS,
    ECU_NVM_REGION_B_ADDRESS
};

static bool bind_erase_region(uint8_t region, void *context)
{
    (void)context;
    return FlashDriver_EraseSector((region == 0U) ? 6U : 7U);
}

static bool bind_program_word(uint8_t region, uint32_t offset, uint32_t value, void *context)
{
    (void)context;
    return FlashDriver_ProgramWord(NVM_REGION_ADDRESS[region & 1U] + offset, value);
}

static const NvmFlashIf_t NVM_FLASH =
{
    .region_base  = { (const uint8_t *)ECU_NVM_REGION_A_ADDRESS,
                      (const uint8_t *)ECU_NVM_REGION_B_ADDRESS },
    .region_size  = ECU_NVM_REGION_SIZE,
    .erase_region = bind_erase_region,
    .program_word = bind_program_word,
    .context      = NULL,
};

static bool on_can_frame(const CanFrame_t *frame)
{
    return DiagManager_OnCanFrame(frame->id, frame->data, frame->dlc, HAL_GetTick());
}

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

bool DiagApp_Init(void)
{
    /* Entropy for the security seed: the chip's unique ID folded into 32
     * bits. Different on every board, so two ECUs never share a sequence. */
    uint8_t  uid[MCU_UNIQUE_ID_LENGTH];
    uint32_t entropy = HAL_GetTick();

    McuDriver_ReadUniqueId(uid);
    for (uint8_t i = 0U; i < MCU_UNIQUE_ID_LENGTH; i++)
    {
        entropy = (entropy << 5) ^ (entropy >> 27) ^ uid[i];
    }

    const DiagConfig_t config =
    {
        .send_frame    = bind_send_frame,
        .send_context  = NULL,
        .system_reset  = McuDriver_SystemReset,
        .nvm           = &NVM_FLASH,
        .entropy       = entropy,
        .dids          = DID_TABLE,
        .did_count     = (uint8_t)(sizeof(DID_TABLE) / sizeof(DID_TABLE[0])),
        .routines      = ROUTINE_TABLE,
        .routine_count = (uint8_t)(sizeof(ROUTINE_TABLE) / sizeof(ROUTINE_TABLE[0])),
        .dtcs          = DTC_TABLE,
        .dtc_count     = (uint8_t)FAULT_COUNT,
    };

    const bool nvm_ok = DiagManager_Init(&config, HAL_GetTick());

    CanManager_SetFrameHandler(on_can_frame);

    return nvm_ok;
}

void DiagApp_Task(void)
{
    /* Receive first so a request that arrived since the last tick is
     * answered in this same tick. */
    CanManager_ProcessReceived();
    DiagManager_Poll(HAL_GetTick());
}

void DiagApp_ReportFault(FaultId_t fault, bool failed)
{
    const DtcSnapshot_t environment =
    {
        .battery_mv    = (uint16_t)BodyControl_GetBatteryMv(),
        .vehicle_state = (uint8_t)VehicleStateMachine_GetState(),
    };

    DtcManager_ReportResult((uint8_t)fault, failed, &environment);
}

void DiagApp_StartOperationCycle(void)
{
    DiagManager_StartOperationCycle();
}

bool DiagApp_IsLampTestActive(void)
{
    if (s_lamp_test_active &&
        ((int32_t)(HAL_GetTick() - s_lamp_test_until_ms) >= 0))
    {
        s_lamp_test_active = false;
    }
    return s_lamp_test_active;
}
