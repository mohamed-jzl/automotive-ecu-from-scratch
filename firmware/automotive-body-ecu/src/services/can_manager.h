/**
 * @file    can_manager.h
 * @brief   CAN communication service: periodic transmission, reception, freshness.
 *
 * Sits between the raw CAN driver and the application:
 *
 *      body_control.c          "the vehicle is in state RUN"
 *            |
 *            v
 *      can_manager.c           owns alive counters, timeouts, staleness
 *            |
 *            v
 *      can_signals.c           packs values into 8 bytes
 *            |
 *            v
 *      can_driver.c            talks to the bxCAN peripheral
 *
 * Signal timeout: the reason it exists
 * ------------------------------------
 * Receiving no CAN frame is not the same as receiving a zero. If the
 * powertrain ECU loses power, its last transmitted value stays in this ECU's
 * memory forever, and software that keeps using it believes the engine is
 * still turning at the last-reported speed. Every safety analysis of a CAN
 * network treats that as a hazard.
 *
 * The fix is to timestamp each reception and treat data older than
 * ECU_CAN_RX_TIMEOUT_MS as unusable. The timeout is set to roughly three
 * times the sender's transmit period, so a single lost frame is tolerated
 * while a genuinely dead sender is detected within a few hundred
 * milliseconds.
 *
 * "Missing" is a state the receiver must handle explicitly. "Stale but
 * plausible" is the failure mode that causes real accidents.
 */

#ifndef SERVICES_CAN_MANAGER_H
#define SERVICES_CAN_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "can_driver.h"
#include "can_signals.h"

/**
 * @brief Handler offered every received frame before normal processing.
 *
 * @return true if the handler consumed the frame.
 */
typedef bool (*CanFrameHandlerFn_t)(const CanFrame_t *frame);

/**
 * @brief Register a handler that sees every received frame first.
 *
 * Used by the diagnostic stack to claim 0x7E0/0x7DF frames. A registered
 * callback keeps the dependency pointing the right way: the CAN manager does
 * not need to know that diagnostics exist, it just offers frames.
 */
void CanManager_SetFrameHandler(CanFrameHandlerFn_t handler);

/**
 * @brief Initialise the CAN driver and reset all counters and timestamps.
 *
 * @return true if the CAN peripheral started successfully.
 */
bool CanManager_Init(void);

/**
 * @brief Transmit BCM_VehicleStatus (0x100).
 *
 * The alive counter is inserted and incremented by this function, so the
 * caller never has to track it. Centralising the counter is what guarantees
 * it advances exactly once per transmission.
 *
 * @param  status  Values to send. The alive_counter field is overwritten.
 * @return true if the frame was queued for transmission.
 */
bool CanManager_SendVehicleStatus(CanVehicleStatus_t *status);

/**
 * @brief Transmit BCM_DIAGNOSTICS (0x101).
 *
 * @param  diagnostics  Values to send. The alive_counter field is overwritten.
 * @return true if the frame was queued for transmission.
 */
bool CanManager_SendDiagnostics(CanDiagnostics_t *diagnostics);

/**
 * @brief Drain the receive FIFO and update the stored engine status.
 *
 * Call periodically. Frames with an unknown identifier or a failed CRC are
 * counted and discarded. Processes every waiting frame in one call so a
 * burst cannot accumulate and overflow the hardware FIFO.
 */
void CanManager_ProcessReceived(void);

/**
 * @brief Read the most recent engine status, if it is still fresh.
 *
 * @param  out  Receives the data. Untouched when the data is stale or absent.
 * @return true if valid, fresh data was copied; false if it has timed out or
 *         has never been received. A false return means "unknown", which the
 *         caller must handle as its own case - not as zero.
 */
bool CanManager_GetEngineStatus(CanEngineStatus_t *out);

/**
 * @brief Whether the engine status has exceeded its reception timeout.
 *
 * @return true if no valid frame has arrived within ECU_CAN_RX_TIMEOUT_MS.
 */
bool CanManager_IsEngineDataStale(void);

/**
 * @brief Number of frames discarded because their CRC did not match.
 *
 * A non-zero and growing value points at a physical-layer problem: missing
 * termination, a bit rate mismatch, or excessive bus length.
 */
uint32_t CanManager_GetCrcErrorCount(void);

/**
 * @brief Number of valid frames received and decoded since startup.
 */
uint32_t CanManager_GetReceivedFrameCount(void);

#endif /* SERVICES_CAN_MANAGER_H */
