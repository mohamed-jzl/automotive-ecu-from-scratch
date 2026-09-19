/**
 * @file    diag_manager.c
 * @brief   Diagnostic stack glue - see diag_manager.h.
 */

#include "diag_manager.h"

#include "ecu_config.h"
#include "uds_security.h"

#include <string.h>

/* Layout of the record saved in flash:
 *   [0]      layout version - bump it whenever this layout changes
 *   [1..17]  VIN
 *   [18..]   DTC records as produced by DtcManager_Serialise()          */
#define PAYLOAD_LAYOUT_VERSION  1U
#define PAYLOAD_OFFSET_VIN      1U
#define PAYLOAD_OFFSET_DTC      (PAYLOAD_OFFSET_VIN + DIAG_VIN_LENGTH)

/* Delay between the reset response leaving the ISO-TP layer and the actual
 * reset. The last frame may still be sitting in a CAN mailbox; 20 ms is many
 * frame times at 500 kbit/s, enough for it to reach the wire. */
#define RESET_DELAY_MS          20U

/* Placeholder VIN before end-of-line programming writes the real one. */
static const uint8_t DEFAULT_VIN[DIAG_VIN_LENGTH] =
{
    '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'
};

static DiagConfig_t s_config;
static IsoTpLink_t  s_link;

static uint8_t      s_request[ECU_ISOTP_BUFFER_SIZE];
static uint8_t      s_response[ECU_ISOTP_BUFFER_SIZE];

static uint8_t      s_vin[DIAG_VIN_LENGTH];
static bool         s_vin_dirty;

static NvmStore_t   s_nvm;
static bool         s_nvm_ok;
static uint32_t     s_last_save_ms;

static bool         s_reset_pending;
static bool         s_reset_armed;
static uint32_t     s_reset_at_ms;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

/* ========================================================================= */
/* Persistence                                                               */
/* ========================================================================= */

static void restore_from_nvm(void)
{
    uint8_t  payload[NVM_PAYLOAD_MAX];
    uint16_t length = 0U;

    if (NvmStore_Read(&s_nvm, payload, sizeof(payload), &length) != NVM_OK)
    {
        return;                         /* first boot: keep the defaults */
    }

    if ((length < PAYLOAD_OFFSET_DTC) || (payload[0] != PAYLOAD_LAYOUT_VERSION))
    {
        /* Saved by a firmware with a different layout. Loading it would put
         * bytes into the wrong fields; starting clean is the safe choice. */
        s_vin_dirty = true;
        return;
    }

    (void)memcpy(s_vin, &payload[PAYLOAD_OFFSET_VIN], DIAG_VIN_LENGTH);

    if (!DtcManager_Deserialise(&payload[PAYLOAD_OFFSET_DTC],
                                (uint16_t)(length - PAYLOAD_OFFSET_DTC)))
    {
        s_vin_dirty = true;             /* DTC table changed: rewrite cleanly */
    }
}

/**
 * @brief Save VIN + DTCs if anything changed, at most once per interval.
 *
 * @param force  Ignore the rate limit (used just before a reset, so nothing
 *               changed in the last second is lost).
 */
static void save_if_needed(uint32_t now_ms, bool force)
{
    if ((!s_nvm_ok) || ((!DtcManager_IsDirty()) && (!s_vin_dirty)))
    {
        return;
    }

    if ((!force) && (!time_reached(now_ms, s_last_save_ms + ECU_NVM_MIN_WRITE_INTERVAL_MS)))
    {
        return;
    }

    uint8_t payload[NVM_PAYLOAD_MAX];

    payload[0] = PAYLOAD_LAYOUT_VERSION;
    (void)memcpy(&payload[PAYLOAD_OFFSET_VIN], s_vin, DIAG_VIN_LENGTH);

    const uint16_t dtc_bytes = DtcManager_Serialise(&payload[PAYLOAD_OFFSET_DTC],
                                                    (uint16_t)(sizeof(payload) - PAYLOAD_OFFSET_DTC));
    if (dtc_bytes == 0U)
    {
        return;
    }

    s_last_save_ms = now_ms;

    if (NvmStore_Write(&s_nvm, payload, (uint16_t)(PAYLOAD_OFFSET_DTC + dtc_bytes)) == NVM_OK)
    {
        DtcManager_ClearDirty();
        s_vin_dirty = false;
    }
    /* On NVM_FULL the data stays dirty in RAM and is retried; the next
     * start-up's maintenance pass makes room. Nothing is silently dropped. */
}

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

bool DiagManager_Init(const DiagConfig_t *config, uint32_t now_ms)
{
    if (config == NULL)
    {
        return false;
    }

    s_config        = *config;
    s_vin_dirty     = false;
    s_nvm_ok        = false;
    s_reset_pending = false;
    s_reset_armed   = false;
    s_last_save_ms  = now_ms;
    (void)memcpy(s_vin, DEFAULT_VIN, DIAG_VIN_LENGTH);

    const IsoTpConfig_t isotp_config =
    {
        .rx_physical_id   = ECU_DIAG_CAN_ID_PHYSICAL,
        .rx_functional_id = ECU_DIAG_CAN_ID_FUNCTIONAL,
        .tx_id            = ECU_DIAG_CAN_ID_RESPONSE,
        .padding_byte     = ECU_ISOTP_PADDING_BYTE,
        .block_size       = ECU_ISOTP_BLOCK_SIZE,
        .st_min_ms        = ECU_ISOTP_ST_MIN_MS,
        .n_cr_timeout_ms  = ECU_ISOTP_TIMEOUT_N_CR_MS,
        .n_bs_timeout_ms  = ECU_ISOTP_TIMEOUT_N_BS_MS,
        .max_wait_frames  = ECU_ISOTP_MAX_WAIT_FRAMES,
        .send_frame       = config->send_frame,
        .send_context     = config->send_context,
    };
    IsoTp_Init(&s_link, &isotp_config);

    DtcManager_Init(config->dtcs, config->dtc_count);

    const UdsServerConfig_t server_config =
    {
        .dids          = config->dids,
        .did_count     = config->did_count,
        .routines      = config->routines,
        .routine_count = config->routine_count,
    };
    UdsServer_Init(&server_config, now_ms);
    UdsSecurity_SeedRandom(config->entropy);

    if (config->nvm == NULL)
    {
        return false;
    }

    if ((NvmStore_Mount(&s_nvm, config->nvm) == NVM_OK) &&
        (NvmStore_Maintain(&s_nvm, ECU_NVM_COMPACT_THRESHOLD_PCT) == NVM_OK))
    {
        s_nvm_ok = true;
        restore_from_nvm();
    }

    return s_nvm_ok;
}

bool DiagManager_OnCanFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc,
                            uint32_t now_ms)
{
    return IsoTp_OnFrame(&s_link, can_id, data, dlc, now_ms);
}

void DiagManager_Poll(uint32_t now_ms)
{
    IsoTp_Poll(&s_link, now_ms);
    UdsServer_Poll(now_ms);

    /* Once a reset has been accepted, no further request is served: the ECU
     * is about to restart, and answering would suggest otherwise. */
    uint16_t length     = 0U;
    bool     functional = false;

    if ((!s_reset_pending) &&
        IsoTp_Receive(&s_link, s_request, sizeof(s_request), &length, &functional))
    {
        const uint16_t response_length =
            UdsServer_ProcessRequest(s_request, length, functional,
                                     s_response, sizeof(s_response), now_ms);

        if (response_length > 0U)
        {
            (void)IsoTp_Send(&s_link, s_response, response_length, now_ms);
        }
    }

    uint8_t reset_type = 0U;
    if (UdsServer_TakeResetRequest(&reset_type))
    {
        s_reset_pending = true;
        s_reset_armed   = false;
    }

    if (s_reset_pending)
    {
        /* Reset only after the positive response has fully left ISO-TP, then
         * wait a little longer for the last frame to clear the CAN mailbox. */
        if (!IsoTp_IsTxBusy(&s_link))
        {
            if (!s_reset_armed)
            {
                s_reset_armed = true;
                s_reset_at_ms = now_ms + RESET_DELAY_MS;
            }
            else if (time_reached(now_ms, s_reset_at_ms))
            {
                save_if_needed(now_ms, true);   /* do not lose recent changes */
                s_reset_pending = false;

                if (s_config.system_reset != NULL)
                {
                    s_config.system_reset();    /* does not return on target */
                }
            }
        }
        return;
    }

    save_if_needed(now_ms, false);
}

void DiagManager_StartOperationCycle(void)
{
    DtcManager_StartOperationCycle();
}

UdsSession_t DiagManager_GetSession(void)
{
    return UdsServer_GetSession();
}

void DiagManager_GetVin(uint8_t out[DIAG_VIN_LENGTH])
{
    if (out != NULL)
    {
        (void)memcpy(out, s_vin, DIAG_VIN_LENGTH);
    }
}

void DiagManager_SetVin(const uint8_t vin[DIAG_VIN_LENGTH])
{
    if (vin != NULL)
    {
        (void)memcpy(s_vin, vin, DIAG_VIN_LENGTH);
        s_vin_dirty = true;
    }
}

uint8_t DiagManager_GetNvmUsagePercent(void)
{
    return s_nvm_ok ? NvmStore_GetUsagePercent(&s_nvm) : 0U;
}
