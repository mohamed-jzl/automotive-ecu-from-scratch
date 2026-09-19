/**
 * @file    isotp.h
 * @brief   ISO-TP (ISO 15765-2): transport of long messages over 8-byte CAN frames.
 *
 * The problem
 * -----------
 * A classic CAN frame carries at most 8 bytes. A UDS message does not fit:
 * writing a 17-character VIN needs 20 bytes, and a DTC report can be hundreds.
 * ISO-TP is the layer that cuts a long message into frames on the sending side
 * and reassembles it on the receiving side - the same job TCP does for IP
 * packets, stripped down to what a microcontroller can afford.
 *
 * The four frame types
 * --------------------
 * The high nibble of the first byte (the PCI, Protocol Control Information)
 * says which kind of frame it is:
 *
 *   0x0  Single Frame (SF)     whole message fits: up to 7 bytes of data
 *        [0L][d1 .. d7]        L = length
 *
 *   0x1  First Frame (FF)      start of a long message: total length + 6 bytes
 *        [1L LL][d1 .. d6]     12-bit length, up to 4095
 *
 *   0x2  Consecutive Frame     the rest, 7 bytes at a time
 *        [2N][d .. d]          N = sequence number 1,2..15,0,1.. (detects loss)
 *
 *   0x3  Flow Control (FC)     receiver tells sender how to continue
 *        [3S][BS][STmin]       S = 0 continue / 1 wait / 2 overflow
 *
 * A 20-byte request therefore looks like this on the bus:
 *
 *      tester -> ECU   10 14 2E F1 90 57 44 42        First Frame, 20 bytes
 *      ECU -> tester   30 00 05 CC CC CC CC CC        FC: go, no block limit, 5 ms gap
 *      tester -> ECU   21 ...                         Consecutive Frame 1
 *      tester -> ECU   22 ...                         Consecutive Frame 2
 *
 * The Flow Control frame is how a slow receiver protects itself. Block Size
 * (BS) says how many frames to send before waiting for the next FC; STmin is
 * the minimum gap between frames. Our ECU asks for 5 ms so its receive queue
 * never overflows.
 *
 * Design: pure and driven from outside
 * ------------------------------------
 * This module never touches hardware and never reads a clock. Frames come in
 * through IsoTp_OnFrame(), go out through a function pointer supplied at
 * initialisation, and the current time is passed in as a parameter. That is
 * what lets tests/test_isotp.c replay exact frame sequences - including lost
 * frames, wrong sequence numbers and timeouts - with no CAN bus at all.
 *
 * Passing the send function as a pointer is called dependency injection. On
 * the ECU it points at the CAN driver; in the unit tests it points at a fake
 * that records frames; in the SIL build it points at a Python callback. The
 * ISO-TP code is identical in all three.
 */

#ifndef DIAG_ISOTP_H
#define DIAG_ISOTP_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"

/** Size of every frame this layer sends (padded) and the minimum it accepts. */
#define ISOTP_CAN_FRAME_SIZE    8U

/**
 * @brief Function the layer calls to put one frame on the bus.
 *
 * @return true if the frame was accepted for transmission. false means "try
 *         again later" (for example all CAN mailboxes are busy); the layer
 *         keeps the frame and retries on the next IsoTp_Poll().
 */
typedef bool (*IsoTpSendFrameFn_t)(uint32_t can_id,
                                   const uint8_t data[ISOTP_CAN_FRAME_SIZE],
                                   uint8_t dlc,
                                   void *context);

/**
 * @brief Static configuration of one ISO-TP link.
 */
typedef struct
{
    uint32_t           rx_physical_id;   /**< Requests addressed to this ECU only   */
    uint32_t           rx_functional_id; /**< Broadcast requests to all ECUs        */
    uint32_t           tx_id;            /**< Identifier this ECU answers on        */
    uint8_t            padding_byte;
    uint8_t            block_size;       /**< BS we request when receiving          */
    uint8_t            st_min_ms;        /**< STmin we request when receiving       */
    uint32_t           n_cr_timeout_ms;
    uint32_t           n_bs_timeout_ms;
    uint8_t            max_wait_frames;
    IsoTpSendFrameFn_t send_frame;
    void              *send_context;     /**< Passed back unchanged to send_frame   */
} IsoTpConfig_t;

/**
 * @brief Why the last transfer was abandoned, for diagnostics and tests.
 */
typedef enum
{
    ISOTP_ERROR_NONE = 0,
    ISOTP_ERROR_TIMEOUT_CR,         /**< Consecutive Frame did not arrive in time */
    ISOTP_ERROR_TIMEOUT_BS,         /**< Flow Control did not arrive in time      */
    ISOTP_ERROR_WRONG_SN,           /**< A Consecutive Frame was lost or reordered */
    ISOTP_ERROR_RX_OVERFLOW,        /**< Incoming message longer than our buffer  */
    ISOTP_ERROR_TX_OVERFLOW,        /**< Receiver reported it cannot take ours    */
    ISOTP_ERROR_WAIT_LIMIT,         /**< Receiver said WAIT too many times        */
    ISOTP_ERROR_INVALID_FC,         /**< Flow Control with an unknown status      */
    ISOTP_ERROR_RX_BUSY             /**< New request while the last is unread     */
} IsoTpError_t;

/* Internal states. Exposed only because the link struct lives in the caller's
 * memory; treat every field of IsoTpLink_t as private. */
typedef enum { ISOTP_RX_IDLE, ISOTP_RX_RECEIVING, ISOTP_RX_COMPLETE } IsoTpRxState_t;
typedef enum { ISOTP_TX_IDLE, ISOTP_TX_SEND_FIRST, ISOTP_TX_WAIT_FC,
               ISOTP_TX_SENDING_CF } IsoTpTxState_t;

/**
 * @brief One ISO-TP link (one tester <-> ECU conversation).
 *
 * Allocated by the caller, typically as a static variable, so the module needs
 * no dynamic memory. All fields are private: access them only through the API.
 */
typedef struct
{
    IsoTpConfig_t  cfg;

    /* --- receive side --- */
    IsoTpRxState_t rx_state;
    uint8_t        rx_buffer[ECU_ISOTP_BUFFER_SIZE];
    uint16_t       rx_expected_length;
    uint16_t       rx_received_length;
    uint8_t        rx_next_sn;
    uint8_t        rx_block_count;
    uint32_t       rx_deadline_ms;
    bool           rx_functional;
    bool           rx_fc_pending;          /**< FC could not be sent yet; retry */
    uint8_t        rx_fc_status;

    /* --- transmit side --- */
    IsoTpTxState_t tx_state;
    uint8_t        tx_buffer[ECU_ISOTP_BUFFER_SIZE];
    uint16_t       tx_length;
    uint16_t       tx_offset;
    uint8_t        tx_next_sn;
    uint8_t        tx_block_size;          /**< BS granted by the receiver's FC */
    uint8_t        tx_block_count;
    uint32_t       tx_st_min_ms;           /**< STmin granted by the receiver   */
    uint32_t       tx_next_cf_ms;
    uint32_t       tx_deadline_ms;
    uint8_t        tx_wait_count;

    IsoTpError_t   last_error;
} IsoTpLink_t;

/**
 * @brief Initialise a link. Must be called before any other function.
 */
void IsoTp_Init(IsoTpLink_t *link, const IsoTpConfig_t *config);

/**
 * @brief Feed one received CAN frame into the link.
 *
 * Frames with an identifier that does not belong to this link are ignored,
 * so the caller may pass every frame it receives.
 *
 * @return true if the frame belonged to this link.
 */
bool IsoTp_OnFrame(IsoTpLink_t *link, uint32_t can_id,
                   const uint8_t *data, uint8_t dlc, uint32_t now_ms);

/**
 * @brief Advance timers and continue any segmented transmission.
 *
 * Call periodically (every 5 ms on the ECU). Sends pending Consecutive Frames
 * as STmin allows and abandons transfers whose timeouts have expired.
 */
void IsoTp_Poll(IsoTpLink_t *link, uint32_t now_ms);

/**
 * @brief Start sending a message. Returns immediately; frames go out over
 *        the following IsoTp_Poll() calls.
 *
 * @return false if a transmission is already in progress, or if the message
 *         is empty or longer than ECU_ISOTP_BUFFER_SIZE.
 */
bool IsoTp_Send(IsoTpLink_t *link, const uint8_t *data, uint16_t length,
                uint32_t now_ms);

/**
 * @brief Take a completely received message out of the link.
 *
 * @param out_functional  Set to true if the request was functionally
 *                        addressed. UDS answers some errors differently for
 *                        broadcast requests, so the server needs to know.
 * @return true if a message was available and copied. The link is then free
 *         to receive the next message.
 */
bool IsoTp_Receive(IsoTpLink_t *link, uint8_t *out, uint16_t out_size,
                   uint16_t *out_length, bool *out_functional);

/** @brief true while a message is being sent (including waiting for FC). */
bool IsoTp_IsTxBusy(const IsoTpLink_t *link);

/** @brief Reason the most recent transfer was abandoned. */
IsoTpError_t IsoTp_GetLastError(const IsoTpLink_t *link);

/**
 * @brief Convert the STmin byte of a Flow Control frame into milliseconds.
 *
 *   0x00..0x7F  that many milliseconds
 *   0xF1..0xF9  100..900 microseconds -> rounded UP to 1 ms, because our
 *               timer only resolves milliseconds and waiting longer than asked
 *               is always allowed, while waiting less never is
 *   anything else is reserved: the standard says treat it as 0x7F (127 ms)
 */
uint32_t IsoTp_DecodeStMin(uint8_t st_min);

#endif /* DIAG_ISOTP_H */
