/**
 * @file    can_signals.h
 * @brief   CAN payload encoding and decoding - pure, hardware-free, testable.
 *
 * Frames, messages and signals
 * ----------------------------
 * These three words are used loosely in conversation and precisely in
 * automotive engineering. The distinction matters in interviews:
 *
 *      FRAME    the physical unit on the wire: identifier, DLC, up to 8 bytes
 *      MESSAGE  a named frame with a defined meaning ("BCM_VehicleStatus")
 *      SIGNAL   one value packed inside a message ("BatteryVoltage", bits 8-23)
 *
 * The bus carries frames. Engineers reason about signals. The translation
 * between the two is what this module does, and in a production project it
 * is generated automatically from a DBC database file. Writing it by hand
 * here makes the mechanism visible instead of magic - see
 * docs/can_specification.md for the equivalent DBC-style definition.
 *
 * End-to-end protection
 * ---------------------
 * CAN's own CRC protects against noise on the wire, but not against a
 * sender that has crashed, a gateway that duplicates a frame, or software
 * that transmits a stale buffer. Every message here therefore carries two
 * extra fields, following the principle of AUTOSAR's E2E Profile 1:
 *
 *      ALIVE COUNTER  increments every transmission and wraps at 255.
 *                     A receiver that sees the same value twice knows the
 *                     data is stale even though the frame arrived intact.
 *
 *      CRC-8          computed over the first seven bytes. Catches
 *                     corruption introduced *after* the CAN controller's own
 *                     check, for instance inside a routing gateway.
 *
 * Together they answer the question CAN alone cannot: "is this data fresh
 * and did it come from a sender that is still functioning?"
 *
 * Byte order
 * ----------
 * All multi-byte signals are little-endian ("Intel" byte order in DBC
 * terminology), matching the ARM Cortex-M's native layout so no conversion
 * is needed on this ECU. Big-endian ("Motorola") is equally common in
 * automotive networks; the only rule that matters is that every node on the
 * bus agrees, which is why the byte order is stated explicitly in the
 * interface specification rather than left implicit in the code.
 */

#ifndef SERVICES_CAN_SIGNALS_H
#define SERVICES_CAN_SIGNALS_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/* Message identifiers                                                       */
/* ========================================================================= */

/* A lower identifier wins CAN arbitration, so the numbering encodes priority.
 * Powertrain data (0x2xx) is deliberately given a *higher* number than body
 * data (0x1xx) here purely so this ECU's traffic is easy to spot in a trace;
 * in a real vehicle, safety-relevant powertrain messages would be assigned
 * the lower, higher-priority identifiers. */
#define CAN_ID_BCM_VEHICLE_STATUS   0x100U  /**< TX, every 100 ms */
#define CAN_ID_BCM_DIAGNOSTICS      0x101U  /**< TX, every 500 ms */
#define CAN_ID_PCM_ENGINE_STATUS    0x200U  /**< RX, every 100 ms */

/** Every message in this project uses the full 8-byte payload. */
#define CAN_MESSAGE_DLC             8U

/** Byte index of the rolling alive counter within every payload. */
#define CAN_ALIVE_COUNTER_BYTE      6U

/** Byte index of the CRC within every payload. */
#define CAN_CRC_BYTE                7U

/* ========================================================================= */
/* Message content structures                                                */
/* ========================================================================= */

/**
 * @brief Payload of BCM_VehicleStatus (0x100), transmitted every 100 ms.
 *
 * The primary broadcast of this ECU: what mode the vehicle is in and what
 * the body outputs are doing. An instrument cluster would consume this to
 * drive its telltales.
 */
typedef struct
{
    uint8_t  vehicle_state;     /**< VehicleState_t value, 0..4, 3 bits        */
    bool     ignition_on;       /**< Ignition input currently active           */
    bool     brake_active;      /**< Brake pedal pressed                       */
    bool     indicator_left;    /**< Left indicator lamp lit right now         */
    bool     indicator_right;   /**< Right indicator lamp lit right now        */
    bool     headlight_on;      /**< Headlight output energised                */
    uint16_t battery_mv;        /**< Battery voltage, millivolts, 1 mV/bit     */
    uint8_t  fault_count;       /**< Number of latched faults, 0..15           */
    uint16_t fault_bitmask;     /**< Bit N set means FaultId_t N is latched    */
    uint8_t  alive_counter;     /**< Rolling counter, incremented per transmit */
} CanVehicleStatus_t;

/**
 * @brief Payload of BCM_DIAGNOSTICS (0x101), transmitted every 500 ms.
 *
 * Health telemetry about the ECU itself rather than about the vehicle.
 * Equivalent to the data a diagnostic tester would request over UDS.
 */
typedef struct
{
    uint16_t uptime_seconds;        /**< Seconds since reset, wraps at ~18 hours */
    uint16_t task_overrun_count;    /**< Scheduler deadline misses since reset   */
    uint8_t  max_task_duration_ms;  /**< Worst-case task execution time observed  */
    uint8_t  can_tx_failures;       /**< Transmit failures, saturating at 255     */
    uint8_t  alive_counter;
} CanDiagnostics_t;

/**
 * @brief Payload of PCM_EngineStatus (0x200), received every 100 ms.
 *
 * Sent by the powertrain ECU. In this project the second Nucleo board plays
 * that role; the Python test framework can also produce it.
 */
typedef struct
{
    uint16_t engine_rpm;            /**< Engine speed, 1 rpm/bit, 0..8000         */
    uint16_t vehicle_speed_kph_x10; /**< Road speed, 0.1 km/h per bit             */
    int8_t   coolant_temp_c;        /**< Coolant temperature, -40..+215 degrees C */
    bool     engine_running;        /**< Powertrain reports the engine is turning */
    uint8_t  alive_counter;
} CanEngineStatus_t;

/* ========================================================================= */
/* CRC                                                                       */
/* ========================================================================= */

/**
 * @brief CRC-8 over a byte buffer, SAE J1850 parameters.
 *
 * Polynomial 0x1D, initial value 0xFF, final XOR 0xFF, no bit reflection.
 * These are the parameters used by AUTOSAR E2E Profile 1, chosen because
 * this polynomial has a Hamming distance of 4 for payloads up to 8 bytes:
 * it detects every possible 1-, 2- and 3-bit error in a CAN payload.
 *
 * Computed bit by bit rather than from a lookup table. The table version is
 * about eight times faster but costs 256 bytes of flash; at two messages per
 * 100 ms, the loop takes well under a microsecond and the flash is better
 * spent elsewhere.
 *
 * @param  data    Buffer to checksum. Returns 0 if NULL.
 * @param  length  Number of bytes to include.
 * @return The 8-bit CRC.
 */
uint8_t CanSignals_Crc8(const uint8_t *data, uint8_t length);

/* ========================================================================= */
/* Pack (encode) - structure to wire format                                  */
/* ========================================================================= */

/**
 * @brief Encode BCM_VehicleStatus into an 8-byte payload.
 *
 * Fills the alive counter and CRC bytes automatically.
 *
 * @param src  Values to encode. Ignored if NULL.
 * @param dst  Receives the 8-byte payload. Ignored if NULL.
 */
void CanSignals_PackVehicleStatus(const CanVehicleStatus_t *src,
                                  uint8_t dst[CAN_MESSAGE_DLC]);

/**
 * @brief Encode BCM_DIAGNOSTICS into an 8-byte payload.
 *
 * @param src  Values to encode. Ignored if NULL.
 * @param dst  Receives the 8-byte payload. Ignored if NULL.
 */
void CanSignals_PackDiagnostics(const CanDiagnostics_t *src,
                                uint8_t dst[CAN_MESSAGE_DLC]);

/**
 * @brief Encode PCM_EngineStatus into an 8-byte payload.
 *
 * This ECU receives rather than sends this message; the encoder exists so
 * that unit tests can round-trip it and so the second board can reuse the
 * same code. Sharing one implementation between sender and receiver is what
 * guarantees they agree on the layout.
 *
 * @param src  Values to encode. Ignored if NULL.
 * @param dst  Receives the 8-byte payload. Ignored if NULL.
 */
void CanSignals_PackEngineStatus(const CanEngineStatus_t *src,
                                 uint8_t dst[CAN_MESSAGE_DLC]);

/* ========================================================================= */
/* Unpack (decode) - wire format to structure                                */
/* ========================================================================= */

/**
 * @brief Decode a BCM_VehicleStatus payload, verifying the CRC first.
 *
 * @param  src  8-byte payload received from the bus.
 * @param  dst  Receives the decoded values. Untouched if the CRC fails.
 * @return true if the CRC matched and the payload was decoded.
 */
bool CanSignals_UnpackVehicleStatus(const uint8_t src[CAN_MESSAGE_DLC],
                                    CanVehicleStatus_t *dst);

/**
 * @brief Decode a BCM_DIAGNOSTICS payload, verifying the CRC first.
 *
 * @param  src  8-byte payload received from the bus.
 * @param  dst  Receives the decoded values. Untouched if the CRC fails.
 * @return true if the CRC matched and the payload was decoded.
 */
bool CanSignals_UnpackDiagnostics(const uint8_t src[CAN_MESSAGE_DLC],
                                  CanDiagnostics_t *dst);

/**
 * @brief Decode a PCM_EngineStatus payload, verifying the CRC first.
 *
 * Rejecting a frame whose CRC fails - rather than using it and hoping - is
 * the entire point of end-to-end protection. Corrupt data that is silently
 * accepted is far more dangerous than data that is known to be missing.
 *
 * @param  src  8-byte payload received from the bus.
 * @param  dst  Receives the decoded values. Untouched if the CRC fails.
 * @return true if the CRC matched and the payload was decoded.
 */
bool CanSignals_UnpackEngineStatus(const uint8_t src[CAN_MESSAGE_DLC],
                                   CanEngineStatus_t *dst);

#endif /* SERVICES_CAN_SIGNALS_H */
