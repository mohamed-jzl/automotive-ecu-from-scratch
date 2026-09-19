/**
 * @file    uds_server.h
 * @brief   UDS (ISO 14229-1) server: decodes requests, enforces the rules, builds responses.
 *
 * Where it sits
 * -------------
 *
 *      CAN frames --> ISO-TP (reassembly) --> UDS server --> ISO-TP --> CAN frames
 *                                               |
 *                          DID table, routine table, DTC manager
 *                          (supplied by the application)
 *
 * The server knows the protocol, not the vehicle. It knows that 0x22 means
 * "read data" and that writing needs an unlocked extended session, but it does
 * not know what DID 0x0100 contains. The application hands it a table of DIDs
 * with a read function for each; the server only calls them.
 *
 * This split mirrors AUTOSAR: there the protocol engine is called DCM
 * (Diagnostic Communication Manager, generic basic software) and the data
 * comes from the application software components. Keeping the protocol
 * generic is what lets one diagnostic stack serve many different ECUs.
 *
 * Services implemented
 * --------------------
 *
 *   SID   Service                          Sessions   Notes
 *   0x10  DiagnosticSessionControl         all        default / extended
 *   0x11  ECUReset                         all        hard / soft
 *   0x14  ClearDiagnosticInformation       def, ext   all DTCs or one
 *   0x19  ReadDTCInformation               def, ext   0x01 0x02 0x04 0x06 0x0A
 *   0x22  ReadDataByIdentifier             def, ext   several DIDs per request
 *   0x27  SecurityAccess                   ext        seed/key, lockout
 *   0x2E  WriteDataByIdentifier            ext        needs security
 *   0x31  RoutineControl                   ext        start / stop / results
 *   0x3E  TesterPresent                    all        keeps a session alive
 *   0x85  ControlDTCSetting                ext        freeze DTC updates
 *
 * Order of checks
 * ---------------
 * When a request breaks several rules at once, ISO 14229-1 fixes which NRC
 * wins. This server follows that order, so a tester always gets the answer
 * the standard predicts:
 *
 *   1. service supported?                    else 0x11
 *   2. service allowed in this session?      else 0x7F
 *   3. long enough to carry a sub-function?  else 0x13
 *   4. then, per service: sub-function (0x12), exact length (0x13),
 *      identifier supported (0x31), security (0x33), conditions (0x22)
 *
 * Pure module: requests in, responses out, time passed as a parameter.
 * Covered by tests/test_uds_server.c with no CAN bus involved.
 */

#ifndef DIAG_UDS_SERVER_H
#define DIAG_UDS_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "uds_defs.h"

/**
 * @brief Read one data identifier.
 * @param out  Buffer of exactly the DID's declared length.
 * @return UDS_NRC_POSITIVE_RESPONSE (0) on success, otherwise the NRC to send
 *         (typically 0x22 conditionsNotCorrect if the data is unavailable).
 */
typedef uint8_t (*UdsDidReadFn_t)(uint8_t *out);

/**
 * @brief Write one data identifier.
 * @param in  Exactly the DID's declared length of bytes, already length-checked.
 * @return 0 on success, otherwise the NRC to send (0x31 for an invalid value).
 */
typedef uint8_t (*UdsDidWriteFn_t)(const uint8_t *in);

/**
 * @brief One entry of the application's DID table.
 */
typedef struct
{
    uint16_t        did;
    uint8_t         length;     /**< Fixed data length in bytes                */
    UdsDidReadFn_t  read;       /**< NULL: not readable                        */
    UdsDidWriteFn_t write;      /**< NULL: read-only. Writing always requires
                                     the extended session AND security access */
} UdsDidEntry_t;

/**
 * @brief Start, stop or query one routine.
 *
 * @param in        Optional parameters sent by the tester after the RID.
 * @param out       Where to write result bytes to append to the response.
 * @param out_len   Set to the number of result bytes written.
 * @return 0 on success, otherwise the NRC to send.
 */
typedef uint8_t (*UdsRoutineFn_t)(const uint8_t *in, uint16_t in_length,
                                  uint8_t *out, uint16_t out_size,
                                  uint16_t *out_length);

/**
 * @brief One entry of the application's routine table.
 *
 * A routine is a function the tester can ask the ECU to run: an actuator test,
 * a calibration, a memory erase before flashing. Unlike a DID it is an action,
 * not a value.
 */
typedef struct
{
    uint16_t       rid;
    bool           requires_security;
    UdsRoutineFn_t start;       /**< NULL: that sub-function is not supported */
    UdsRoutineFn_t stop;
    UdsRoutineFn_t results;
} UdsRoutineEntry_t;

typedef struct
{
    const UdsDidEntry_t     *dids;
    uint8_t                  did_count;
    const UdsRoutineEntry_t *routines;
    uint8_t                  routine_count;
} UdsServerConfig_t;

/**
 * @brief Reset the server to the default session, locked, no reset pending.
 */
void UdsServer_Init(const UdsServerConfig_t *config, uint32_t now_ms);

/**
 * @brief Process one complete request and build the response.
 *
 * @param functional  true if the request arrived on the functional (broadcast)
 *                    address. Some negative responses are then suppressed so
 *                    that ECUs which do not support a service stay silent
 *                    instead of flooding the bus with refusals.
 * @return Length of the response written into @p response, or 0 when no
 *         response must be sent (suppressed positive response, or suppressed
 *         NRC for a functional request).
 */
uint16_t UdsServer_ProcessRequest(const uint8_t *request, uint16_t request_length,
                                  bool functional,
                                  uint8_t *response, uint16_t response_size,
                                  uint32_t now_ms);

/**
 * @brief Run the S3 session timer. Call periodically.
 */
void UdsServer_Poll(uint32_t now_ms);

UdsSession_t UdsServer_GetSession(void);
bool         UdsServer_IsUnlocked(void);

/**
 * @brief Take a pending ECUReset request, if any.
 *
 * The server never resets the MCU itself: it only records that a reset was
 * asked for. The caller must first let the positive response leave the bus,
 * otherwise the tester would never learn its request was accepted.
 *
 * @param reset_type  Receives UDS_RESET_HARD or UDS_RESET_SOFT.
 * @return true if a reset was pending (it is then cleared).
 */
bool UdsServer_TakeResetRequest(uint8_t *reset_type);

#endif /* DIAG_UDS_SERVER_H */
