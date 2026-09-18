/**
 * @file    can_driver.h
 * @brief   bxCAN transmit/receive driver for CAN1 at 500 kbit/s.
 *
 * Why vehicles use CAN
 * --------------------
 * A modern car contains 50-150 ECUs. Wiring every signal point-to-point would
 * need thousands of wires; CAN replaces that with a single twisted pair that
 * every node shares.
 *
 * CAN is a *differential* bus: the receiver looks at the voltage difference
 * between CAN_H and CAN_L, not at either wire's voltage against ground.
 * Electrical noise couples into both wires almost equally, so it cancels in
 * the difference. This is why CAN survives next to an ignition coil while a
 * single-ended UART would not. (Same principle as a differential amplifier
 * rejecting common-mode input.)
 *
 *      CAN_H  ────────╮   ╭──── 2.5 V idle, rises to ~3.5 V when dominant
 *                     │   │
 *                  differential = 0 V (recessive) or ~2 V (dominant)
 *                     │   │
 *      CAN_L  ────────╯   ╰──── 2.5 V idle, falls to ~1.5 V when dominant
 *
 * Arbitration without a bus master
 * --------------------------------
 * Every node may start transmitting when the bus is idle. A dominant bit (0)
 * always overrides a recessive bit (1) electrically - the bus behaves like a
 * wired-AND. While transmitting its identifier, each node monitors the bus;
 * if it sent recessive but reads dominant, another node is sending a
 * lower-numbered ID, so it backs off immediately and becomes a receiver.
 *
 * The result is non-destructive priority arbitration: the winning frame is
 * not delayed by even one bit, and *a lower numeric ID means higher priority*.
 * That is why safety-critical messages get low IDs and comfort messages get
 * high ones. Our body ECU deliberately uses 0x100+, above the powertrain.
 *
 * Hardware requirement
 * --------------------
 * The STM32 provides only logic-level TX and RX. An external transceiver
 * (MCP2551, TJA1050, SN65HVD230) converts those into the differential bus,
 * and the bus needs a 120 ohm termination resistor at each physical end -
 * matching the cable's characteristic impedance so signal reflections do not
 * corrupt the waveform. Two resistors total, regardless of node count.
 */

#ifndef DRIVERS_CAN_DRIVER_H
#define DRIVERS_CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/** Maximum payload of a classic CAN frame, in bytes. */
#define CAN_FRAME_MAX_DLC   8U

/**
 * @brief A classic CAN 2.0A frame with an 11-bit standard identifier.
 */
typedef struct
{
    uint32_t id;                        /**< Standard identifier, 0x000..0x7FF   */
    uint8_t  dlc;                       /**< Data length code, 0..8 bytes        */
    uint8_t  data[CAN_FRAME_MAX_DLC];   /**< Payload; bytes beyond dlc are unused */
} CanFrame_t;

/**
 * @brief Configure CAN1 pins, bit timing and acceptance filters, then start it.
 *
 * Bit timing is 500 kbit/s with an 85.7% sample point - see ECU_CAN_* in
 * ecu_config.h for the derivation. Every node on a bus must agree on both
 * the bit rate and, approximately, the sample point.
 *
 * The acceptance filter is configured to receive all standard identifiers.
 * A real ECU narrows this to only the IDs it needs, because filtering in
 * hardware costs nothing while filtering in software costs an interrupt and
 * a comparison for every frame on a busy bus.
 *
 * @return true on success, false if the HAL rejected the configuration.
 */
bool CanDriver_Init(void);

/**
 * @brief Queue a frame for transmission.
 *
 * bxCAN has three transmit mailboxes. If all three are occupied the call
 * fails immediately rather than blocking - the caller decides whether to
 * retry or to raise a fault. A persistently full mailbox means the bus is
 * not acknowledging frames, usually because this node is alone on the bus
 * or the termination is missing.
 *
 * @param  frame  Frame to send. Must be non-NULL with dlc <= 8.
 * @return true if the frame was accepted into a mailbox.
 */
bool CanDriver_Transmit(const CanFrame_t *frame);

/**
 * @brief Fetch one received frame, if any is waiting.
 *
 * Non-blocking. Polls the receive FIFO rather than using an interrupt, which
 * keeps the data flow explicit and the timing analysable. At 500 kbit/s a
 * frame takes at least ~230 us, and the 3-message FIFO gives roughly 700 us
 * of slack - far more than our 10 ms poll period would need if the bus were
 * busy, so a production version would move to interrupt-driven reception.
 *
 * @param  frame  Receives the frame. Untouched when no frame is available.
 * @return true if a frame was retrieved, false if the FIFO was empty.
 */
bool CanDriver_Receive(CanFrame_t *frame);

/**
 * @brief Report whether the controller has entered the bus-off state.
 *
 * CAN nodes maintain transmit and receive error counters. Each failed
 * transmission adds 8; each success subtracts 1. Above 255 the node declares
 * itself bus-off and disconnects from the bus entirely. This is a designed
 * safety property: a node with a stuck-dominant output would otherwise
 * destroy communication for every other ECU on the vehicle.
 *
 * @return true if the controller is bus-off and no longer participating.
 */
bool CanDriver_IsBusOff(void);

/**
 * @brief Number of transmissions that failed since startup.
 *
 * @return Cumulative transmit failure count.
 */
uint32_t CanDriver_GetTxFailureCount(void);

#endif /* DRIVERS_CAN_DRIVER_H */
