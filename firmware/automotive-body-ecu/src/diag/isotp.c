/**
 * @file    isotp.c
 * @brief   ISO-TP implementation - see isotp.h for the protocol overview.
 *
 * Two independent state machines live in one link:
 *
 *   RECEIVE    IDLE --SF--> COMPLETE
 *              IDLE --FF--> RECEIVING --CF..CF--> COMPLETE
 *              RECEIVING --timeout / wrong SN--> IDLE (message discarded)
 *
 *   TRANSMIT   IDLE --Send--> SEND_FIRST --SF sent--> IDLE
 *                                        --FF sent--> WAIT_FC
 *              WAIT_FC --FC CTS--> SENDING_CF --all sent--> IDLE
 *              SENDING_CF --block full--> WAIT_FC
 *              WAIT_FC --timeout / OVFLW--> IDLE (message abandoned)
 *
 * They are independent because ISO-TP is full duplex: the ECU may still be
 * sending one response while the tester starts the next request.
 */

#include "isotp.h"

#include <string.h>

/* Frame type, from the high nibble of the first byte. */
#define PCI_SINGLE_FRAME        0x0U
#define PCI_FIRST_FRAME         0x1U
#define PCI_CONSECUTIVE_FRAME   0x2U
#define PCI_FLOW_CONTROL        0x3U

/* Flow Control status, from the low nibble of an FC frame. */
#define FC_CONTINUE_TO_SEND     0x0U
#define FC_WAIT                 0x1U
#define FC_OVERFLOW             0x2U

/* Payload capacity of each frame type with 8-byte CAN frames. */
#define SF_MAX_DATA             7U
#define FF_DATA                 6U
#define CF_DATA                 7U

/* ========================================================================= */
/* Helpers                                                                   */
/* ========================================================================= */

/**
 * @brief Has @p deadline been reached, correctly across timer wrap-around?
 *
 * Casting the difference to a signed type is the standard trick: it gives the
 * right answer as long as the two times are less than ~24 days apart, even
 * when the 32-bit millisecond counter wraps from 0xFFFFFFFF back to 0.
 */
static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

/** @brief Start a frame filled with the padding byte. */
static void frame_clear(const IsoTpLink_t *link, uint8_t frame[ISOTP_CAN_FRAME_SIZE])
{
    (void)memset(frame, link->cfg.padding_byte, ISOTP_CAN_FRAME_SIZE);
}

static bool frame_send(IsoTpLink_t *link, const uint8_t frame[ISOTP_CAN_FRAME_SIZE])
{
    return link->cfg.send_frame(link->cfg.tx_id, frame, ISOTP_CAN_FRAME_SIZE,
                                link->cfg.send_context);
}

/**
 * @brief Send a Flow Control frame: [3S][BS][STmin].
 *
 * @return true if accepted by the CAN layer. A refused FC is remembered and
 *         retried from IsoTp_Poll(), because losing it would leave the sender
 *         waiting until its own timeout expired.
 */
static bool send_flow_control(IsoTpLink_t *link, uint8_t status)
{
    uint8_t frame[ISOTP_CAN_FRAME_SIZE];
    frame_clear(link, frame);

    frame[0] = (uint8_t)((PCI_FLOW_CONTROL << 4) | (status & 0x0FU));
    frame[1] = (status == FC_CONTINUE_TO_SEND) ? link->cfg.block_size : 0U;
    frame[2] = (status == FC_CONTINUE_TO_SEND) ? link->cfg.st_min_ms  : 0U;

    return frame_send(link, frame);
}

static void queue_flow_control(IsoTpLink_t *link, uint8_t status)
{
    link->rx_fc_status  = status;
    link->rx_fc_pending = !send_flow_control(link, status);
}

static void rx_abort(IsoTpLink_t *link, IsoTpError_t reason)
{
    link->rx_state      = ISOTP_RX_IDLE;
    link->rx_fc_pending = false;
    link->last_error    = reason;
}

static void tx_abort(IsoTpLink_t *link, IsoTpError_t reason)
{
    link->tx_state   = ISOTP_TX_IDLE;
    link->last_error = reason;
}

uint32_t IsoTp_DecodeStMin(uint8_t st_min)
{
    if (st_min <= 0x7FU)
    {
        return st_min;                  /* milliseconds */
    }
    if ((st_min >= 0xF1U) && (st_min <= 0xF9U))
    {
        return 1U;                      /* 100-900 us, rounded up to 1 ms */
    }
    return 0x7FU;                       /* reserved: treat as the maximum */
}

/* ========================================================================= */
/* Receive path                                                              */
/* ========================================================================= */

static void handle_single_frame(IsoTpLink_t *link, const uint8_t *data,
                                uint8_t dlc, bool functional)
{
    const uint8_t length = data[0] & 0x0FU;

    /* Length 0 is the CAN-FD escape sequence and is not valid on classic CAN;
     * a DLC too short to hold the announced bytes is a malformed frame. */
    if ((length == 0U) || (length > SF_MAX_DATA) || (dlc < (uint8_t)(length + 1U)))
    {
        return;
    }

    if (link->rx_state == ISOTP_RX_COMPLETE)
    {
        /* The previous request has not been processed yet. Dropping the new
         * one is safer than overwriting a message the server may be reading. */
        link->last_error = ISOTP_ERROR_RX_BUSY;
        return;
    }

    if ((link->rx_state == ISOTP_RX_RECEIVING) && functional)
    {
        /* A broadcast arriving mid-transfer is ignored rather than allowed to
         * destroy the physical message in progress (this link has one buffer). */
        return;
    }

    /* A physical SF during a multi-frame reception means the tester gave up on
     * the old message and started a new one; ISO 15765-2 says to process the
     * new one. */
    (void)memcpy(link->rx_buffer, &data[1], length);
    link->rx_expected_length = length;
    link->rx_received_length = length;
    link->rx_functional      = functional;
    link->rx_fc_pending      = false;
    link->rx_state           = ISOTP_RX_COMPLETE;
}

static void handle_first_frame(IsoTpLink_t *link, const uint8_t *data,
                               uint8_t dlc, uint32_t now_ms)
{
    if (dlc < ISOTP_CAN_FRAME_SIZE)
    {
        return;                         /* an FF must use the full frame */
    }

    const uint16_t length = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8) | data[1]);

    /* A message that fits in a Single Frame must not be sent as a First Frame. */
    if (length <= SF_MAX_DATA)
    {
        return;
    }

    if (link->rx_state == ISOTP_RX_COMPLETE)
    {
        link->last_error = ISOTP_ERROR_RX_BUSY;
        return;
    }

    if (length > sizeof(link->rx_buffer))
    {
        /* Tell the sender immediately that it cannot succeed, instead of
         * letting it send every frame and then silently discarding them. */
        link->rx_state   = ISOTP_RX_IDLE;
        link->last_error = ISOTP_ERROR_RX_OVERFLOW;
        queue_flow_control(link, FC_OVERFLOW);
        return;
    }

    (void)memcpy(link->rx_buffer, &data[2], FF_DATA);
    link->rx_expected_length = length;
    link->rx_received_length = FF_DATA;
    link->rx_next_sn         = 1U;       /* the first CF carries SN 1, not 0 */
    link->rx_block_count     = 0U;
    link->rx_functional      = false;
    link->rx_deadline_ms     = now_ms + link->cfg.n_cr_timeout_ms;
    link->rx_state           = ISOTP_RX_RECEIVING;

    queue_flow_control(link, FC_CONTINUE_TO_SEND);
}

static void handle_consecutive_frame(IsoTpLink_t *link, const uint8_t *data,
                                     uint8_t dlc, uint32_t now_ms)
{
    if (link->rx_state != ISOTP_RX_RECEIVING)
    {
        return;                         /* no transfer in progress: stray frame */
    }

    const uint8_t sequence = data[0] & 0x0FU;

    if (sequence != link->rx_next_sn)
    {
        /* A frame was lost or arrived out of order. The reassembled message
         * would be corrupt, so it is discarded entirely - delivering half a
         * message to the UDS layer would be far worse than delivering none. */
        rx_abort(link, ISOTP_ERROR_WRONG_SN);
        return;
    }

    const uint16_t remaining = (uint16_t)(link->rx_expected_length - link->rx_received_length);
    const uint8_t  count     = (remaining < CF_DATA) ? (uint8_t)remaining : (uint8_t)CF_DATA;

    if (dlc < (uint8_t)(count + 1U))
    {
        rx_abort(link, ISOTP_ERROR_WRONG_SN);
        return;
    }

    (void)memcpy(&link->rx_buffer[link->rx_received_length], &data[1], count);
    link->rx_received_length = (uint16_t)(link->rx_received_length + count);

    /* Sequence numbers are 4 bits and wrap 15 -> 0. */
    link->rx_next_sn = (uint8_t)((link->rx_next_sn + 1U) & 0x0FU);

    if (link->rx_received_length >= link->rx_expected_length)
    {
        link->rx_state = ISOTP_RX_COMPLETE;
        return;
    }

    link->rx_deadline_ms = now_ms + link->cfg.n_cr_timeout_ms;

    /* With a non-zero block size, the sender pauses after every BS frames and
     * waits for our permission to continue. */
    if (link->cfg.block_size > 0U)
    {
        link->rx_block_count++;

        if (link->rx_block_count >= link->cfg.block_size)
        {
            link->rx_block_count = 0U;
            queue_flow_control(link, FC_CONTINUE_TO_SEND);
        }
    }
}

/* ========================================================================= */
/* Transmit path                                                             */
/* ========================================================================= */

static void handle_flow_control(IsoTpLink_t *link, const uint8_t *data,
                                uint8_t dlc, uint32_t now_ms)
{
    if ((link->tx_state != ISOTP_TX_WAIT_FC) || (dlc < 3U))
    {
        return;                         /* not waiting for one: stray frame */
    }

    switch (data[0] & 0x0FU)
    {
        case FC_CONTINUE_TO_SEND:
            link->tx_block_size  = data[1];
            link->tx_st_min_ms   = IsoTp_DecodeStMin(data[2]);
            link->tx_block_count = 0U;
            link->tx_wait_count  = 0U;
            link->tx_next_cf_ms  = now_ms;      /* first CF may go right away */
            link->tx_state       = ISOTP_TX_SENDING_CF;
            break;

        case FC_WAIT:
            link->tx_wait_count++;
            if (link->tx_wait_count > link->cfg.max_wait_frames)
            {
                tx_abort(link, ISOTP_ERROR_WAIT_LIMIT);
            }
            else
            {
                link->tx_deadline_ms = now_ms + link->cfg.n_bs_timeout_ms;
            }
            break;

        case FC_OVERFLOW:
            tx_abort(link, ISOTP_ERROR_TX_OVERFLOW);
            break;

        default:
            tx_abort(link, ISOTP_ERROR_INVALID_FC);
            break;
    }
}

/**
 * @brief Try to send the Single Frame or First Frame of the pending message.
 */
static void try_send_first_frame(IsoTpLink_t *link, uint32_t now_ms)
{
    uint8_t frame[ISOTP_CAN_FRAME_SIZE];
    frame_clear(link, frame);

    if (link->tx_length <= SF_MAX_DATA)
    {
        frame[0] = (uint8_t)link->tx_length;                    /* [0L]    */
        (void)memcpy(&frame[1], link->tx_buffer, link->tx_length);
    }
    else
    {
        frame[0] = (uint8_t)((PCI_FIRST_FRAME << 4) |
                             ((link->tx_length >> 8) & 0x0FU)); /* [1L]    */
        frame[1] = (uint8_t)(link->tx_length & 0xFFU);          /* [LL]    */
        (void)memcpy(&frame[2], link->tx_buffer, FF_DATA);
    }

    if (!frame_send(link, frame))
    {
        return;                         /* stay in SEND_FIRST, retry on Poll */
    }

    if (link->tx_length <= SF_MAX_DATA)
    {
        link->tx_state = ISOTP_TX_IDLE;
    }
    else
    {
        link->tx_deadline_ms = now_ms + link->cfg.n_bs_timeout_ms;
        link->tx_state       = ISOTP_TX_WAIT_FC;
    }
}

/**
 * @brief Send as many Consecutive Frames as STmin, BS and the CAN mailboxes allow.
 *
 * With STmin 0 several frames go out in one call, until the CAN driver runs out
 * of mailboxes (it has three). The rest follow on the next poll. With STmin > 0
 * at most one frame leaves per STmin interval.
 */
static void send_consecutive_frames(IsoTpLink_t *link, uint32_t now_ms)
{
    while ((link->tx_state == ISOTP_TX_SENDING_CF) &&
           time_reached(now_ms, link->tx_next_cf_ms))
    {
        uint8_t frame[ISOTP_CAN_FRAME_SIZE];
        frame_clear(link, frame);

        const uint16_t remaining = (uint16_t)(link->tx_length - link->tx_offset);
        const uint8_t  count     = (remaining < CF_DATA) ? (uint8_t)remaining : (uint8_t)CF_DATA;

        frame[0] = (uint8_t)((PCI_CONSECUTIVE_FRAME << 4) | link->tx_next_sn);
        (void)memcpy(&frame[1], &link->tx_buffer[link->tx_offset], count);

        if (!frame_send(link, frame))
        {
            return;                     /* mailboxes full: resume next poll */
        }

        link->tx_offset  = (uint16_t)(link->tx_offset + count);
        link->tx_next_sn = (uint8_t)((link->tx_next_sn + 1U) & 0x0FU);

        if (link->tx_offset >= link->tx_length)
        {
            link->tx_state = ISOTP_TX_IDLE;
            return;
        }

        if (link->tx_block_size > 0U)
        {
            link->tx_block_count++;

            if (link->tx_block_count >= link->tx_block_size)
            {
                /* Block complete: stop and wait for the receiver's next FC. */
                link->tx_deadline_ms = now_ms + link->cfg.n_bs_timeout_ms;
                link->tx_state       = ISOTP_TX_WAIT_FC;
                return;
            }
        }

        link->tx_next_cf_ms = now_ms + link->tx_st_min_ms;
    }
}

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

void IsoTp_Init(IsoTpLink_t *link, const IsoTpConfig_t *config)
{
    if ((link == NULL) || (config == NULL))
    {
        return;
    }

    (void)memset(link, 0, sizeof(*link));
    link->cfg        = *config;
    link->rx_state   = ISOTP_RX_IDLE;
    link->tx_state   = ISOTP_TX_IDLE;
    link->last_error = ISOTP_ERROR_NONE;
}

bool IsoTp_OnFrame(IsoTpLink_t *link, uint32_t can_id,
                   const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    if (link == NULL)
    {
        return false;
    }

    const bool physical   = (can_id == link->cfg.rx_physical_id);
    const bool functional = (can_id == link->cfg.rx_functional_id);

    if ((!physical) && (!functional))
    {
        return false;                   /* someone else's frame */
    }

    if ((data == NULL) || (dlc == 0U))
    {
        return true;
    }

    switch (data[0] >> 4)
    {
        case PCI_SINGLE_FRAME:
            handle_single_frame(link, data, dlc, functional);
            break;

        /* Functional addressing is one-to-many, and a multi-frame exchange
         * needs one specific receiver to send flow control. So functional
         * requests must fit in a Single Frame; anything else is ignored. */
        case PCI_FIRST_FRAME:
            if (physical) { handle_first_frame(link, data, dlc, now_ms); }
            break;

        case PCI_CONSECUTIVE_FRAME:
            if (physical) { handle_consecutive_frame(link, data, dlc, now_ms); }
            break;

        case PCI_FLOW_CONTROL:
            if (physical) { handle_flow_control(link, data, dlc, now_ms); }
            break;

        default:
            break;                      /* reserved frame type: ignore */
    }

    return true;
}

void IsoTp_Poll(IsoTpLink_t *link, uint32_t now_ms)
{
    if (link == NULL)
    {
        return;
    }

    /* --- receive timeout (N_Cr) --- */
    if ((link->rx_state == ISOTP_RX_RECEIVING) &&
        time_reached(now_ms, link->rx_deadline_ms))
    {
        rx_abort(link, ISOTP_ERROR_TIMEOUT_CR);
    }

    /* --- retry a Flow Control the CAN layer refused earlier --- */
    if (link->rx_fc_pending)
    {
        link->rx_fc_pending = !send_flow_control(link, link->rx_fc_status);
    }

    /* --- transmit progress --- */
    switch (link->tx_state)
    {
        case ISOTP_TX_SEND_FIRST:
            try_send_first_frame(link, now_ms);
            break;

        case ISOTP_TX_WAIT_FC:
            if (time_reached(now_ms, link->tx_deadline_ms))
            {
                tx_abort(link, ISOTP_ERROR_TIMEOUT_BS);
            }
            break;

        case ISOTP_TX_SENDING_CF:
            send_consecutive_frames(link, now_ms);
            break;

        case ISOTP_TX_IDLE:
        default:
            break;
    }
}

bool IsoTp_Send(IsoTpLink_t *link, const uint8_t *data, uint16_t length,
                uint32_t now_ms)
{
    if ((link == NULL) || (data == NULL) || (length == 0U) ||
        (length > sizeof(link->tx_buffer)))
    {
        return false;
    }

    if (link->tx_state != ISOTP_TX_IDLE)
    {
        return false;                   /* one message at a time */
    }

    (void)memcpy(link->tx_buffer, data, length);
    link->tx_length      = length;
    link->tx_offset      = (length <= SF_MAX_DATA) ? length : (uint16_t)FF_DATA;
    link->tx_next_sn     = 1U;
    link->tx_block_count = 0U;
    link->tx_wait_count  = 0U;
    link->tx_state       = ISOTP_TX_SEND_FIRST;

    try_send_first_frame(link, now_ms);
    return true;
}

bool IsoTp_Receive(IsoTpLink_t *link, uint8_t *out, uint16_t out_size,
                   uint16_t *out_length, bool *out_functional)
{
    if ((link == NULL) || (out == NULL) || (out_length == NULL) ||
        (link->rx_state != ISOTP_RX_COMPLETE))
    {
        return false;
    }

    if (link->rx_received_length > out_size)
    {
        rx_abort(link, ISOTP_ERROR_RX_OVERFLOW);
        return false;
    }

    (void)memcpy(out, link->rx_buffer, link->rx_received_length);
    *out_length = link->rx_received_length;

    if (out_functional != NULL)
    {
        *out_functional = link->rx_functional;
    }

    link->rx_state = ISOTP_RX_IDLE;
    return true;
}

bool IsoTp_IsTxBusy(const IsoTpLink_t *link)
{
    return (link != NULL) && (link->tx_state != ISOTP_TX_IDLE);
}

IsoTpError_t IsoTp_GetLastError(const IsoTpLink_t *link)
{
    return (link != NULL) ? link->last_error : ISOTP_ERROR_NONE;
}
