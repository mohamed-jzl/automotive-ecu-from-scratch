/**
 * @file    can_manager.c
 * @brief   CAN service implementation - see can_manager.h for the rationale.
 */

#include "can_manager.h"

#include "can_driver.h"
#include "ecu_config.h"
#include "stm32f4xx_hal.h"

#include <string.h>

/* One alive counter per transmitted message. They are independent: each
 * message's receiver checks continuity of that message alone. */
static uint8_t s_alive_vehicle_status = 0U;
static uint8_t s_alive_diagnostics    = 0U;

/* Last successfully decoded engine status, plus when it arrived. */
static CanEngineStatus_t s_engine_status;
static uint32_t          s_engine_rx_timestamp_ms = 0U;
static bool              s_engine_ever_received   = false;

static uint32_t s_crc_error_count      = 0U;
static uint32_t s_received_frame_count = 0U;

static CanFrameHandlerFn_t s_frame_handler = NULL;

void CanManager_SetFrameHandler(CanFrameHandlerFn_t handler)
{
    s_frame_handler = handler;
}

bool CanManager_Init(void)
{
    s_alive_vehicle_status   = 0U;
    s_alive_diagnostics      = 0U;
    s_engine_rx_timestamp_ms = 0U;
    s_engine_ever_received   = false;
    s_crc_error_count        = 0U;
    s_received_frame_count   = 0U;

    (void)memset(&s_engine_status, 0, sizeof(s_engine_status));

    return CanDriver_Init();
}

bool CanManager_SendVehicleStatus(CanVehicleStatus_t *status)
{
    if (status == NULL)
    {
        return false;
    }

    /* Advance first, then send, so the very first frame carries counter 1.
     * Counter 0 stays reserved as "never transmitted", which lets a receiver
     * distinguish a genuine first frame from an uninitialised buffer. */
    s_alive_vehicle_status++;
    status->alive_counter = s_alive_vehicle_status;

    CanFrame_t frame;
    frame.id  = CAN_ID_BCM_VEHICLE_STATUS;
    frame.dlc = CAN_MESSAGE_DLC;

    CanSignals_PackVehicleStatus(status, frame.data);

    return CanDriver_Transmit(&frame);
}

bool CanManager_SendDiagnostics(CanDiagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return false;
    }

    s_alive_diagnostics++;
    diagnostics->alive_counter = s_alive_diagnostics;

    CanFrame_t frame;
    frame.id  = CAN_ID_BCM_DIAGNOSTICS;
    frame.dlc = CAN_MESSAGE_DLC;

    CanSignals_PackDiagnostics(diagnostics, frame.data);

    return CanDriver_Transmit(&frame);
}

void CanManager_ProcessReceived(void)
{
    CanFrame_t frame;

    /* Drain the whole FIFO rather than taking one frame per call. The bxCAN
     * receive FIFO holds three messages; at 500 kbit/s a burst can fill it in
     * under a millisecond, so leaving frames behind risks an overrun that
     * silently discards data. */
    while (CanDriver_Receive(&frame))
    {
        /* Diagnostic frames are claimed first. They may legitimately be
         * shorter than 8 bytes, so this must happen before the DLC check. */
        if ((s_frame_handler != NULL) && s_frame_handler(&frame))
        {
            continue;
        }

        if (frame.dlc != CAN_MESSAGE_DLC)
        {
            /* Every message in this project's interface is 8 bytes. A
             * different length means the frame is not ours, or the sender
             * disagrees with the specification. Either way, do not guess. */
            continue;
        }

        switch (frame.id)
        {
            case CAN_ID_PCM_ENGINE_STATUS:
            {
                CanEngineStatus_t decoded;

                if (CanSignals_UnpackEngineStatus(frame.data, &decoded))
                {
                    s_engine_status          = decoded;
                    s_engine_rx_timestamp_ms = HAL_GetTick();
                    s_engine_ever_received   = true;
                    s_received_frame_count++;
                }
                else
                {
                    /* CRC failed. Count it and discard: the previously stored
                     * value stays untouched and continues to age normally
                     * toward its timeout. */
                    s_crc_error_count++;
                }
                break;
            }

            default:
                /* Not a message this ECU subscribes to. Silently ignored -
                 * on a real vehicle bus the overwhelming majority of traffic
                 * belongs to other nodes. */
                break;
        }
    }
}

bool CanManager_GetEngineStatus(CanEngineStatus_t *out)
{
    if ((out == NULL) || (!s_engine_ever_received) || CanManager_IsEngineDataStale())
    {
        return false;
    }

    *out = s_engine_status;
    return true;
}

bool CanManager_IsEngineDataStale(void)
{
    if (!s_engine_ever_received)
    {
        /* Never received is treated as stale. Before the first frame arrives
         * there is no data, and "no data" must never be mistaken for zero. */
        return true;
    }

    const uint32_t age_ms = HAL_GetTick() - s_engine_rx_timestamp_ms;

    return (age_ms > ECU_CAN_RX_TIMEOUT_MS);
}

uint32_t CanManager_GetCrcErrorCount(void)
{
    return s_crc_error_count;
}

uint32_t CanManager_GetReceivedFrameCount(void)
{
    return s_received_frame_count;
}
