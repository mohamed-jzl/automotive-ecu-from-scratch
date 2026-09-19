/**
 * @file    uds_defs.h
 * @brief   UDS (ISO 14229-1) protocol constants: service IDs, NRCs, sessions.
 *
 * What UDS is
 * -----------
 * Unified Diagnostic Services is the request/response protocol a workshop
 * tester, an end-of-line test bench or a flashing tool uses to talk to an ECU.
 * Every message starts with a Service Identifier (SID) byte saying what is
 * being asked:
 *
 *      Request      22 F1 90            "ReadDataByIdentifier, DID 0xF190 (VIN)"
 *      Positive     62 F1 90 57 44 ...   SID + 0x40, then the data
 *      Negative     7F 22 31             0x7F, the SID refused, and WHY (NRC)
 *
 * A positive response always echoes the request SID plus 0x40. A negative
 * response is always three bytes: 0x7F, the rejected SID, and a Negative
 * Response Code explaining the refusal. Those two rules are enough to decode
 * any UDS trace by eye.
 *
 * The numeric values below are fixed by the standard. They are part of the
 * interface with every diagnostic tester in the world, so they are never
 * renumbered or "improved".
 */

#ifndef DIAG_UDS_DEFS_H
#define DIAG_UDS_DEFS_H

#include <stdint.h>

/* ========================================================================= */
/* Service identifiers implemented by this ECU                               */
/* ========================================================================= */

#define UDS_SID_DIAGNOSTIC_SESSION_CONTROL      0x10U
#define UDS_SID_ECU_RESET                       0x11U
#define UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION    0x14U
#define UDS_SID_READ_DTC_INFORMATION            0x19U
#define UDS_SID_READ_DATA_BY_IDENTIFIER         0x22U
#define UDS_SID_SECURITY_ACCESS                 0x27U
#define UDS_SID_WRITE_DATA_BY_IDENTIFIER        0x2EU
#define UDS_SID_ROUTINE_CONTROL                 0x31U
#define UDS_SID_TESTER_PRESENT                  0x3EU
#define UDS_SID_CONTROL_DTC_SETTING             0x85U

/** Added to the request SID to form the positive response SID. */
#define UDS_POSITIVE_RESPONSE_OFFSET            0x40U

/** First byte of every negative response. */
#define UDS_NEGATIVE_RESPONSE_SID               0x7FU

/**
 * Bit 7 of the sub-function byte: "suppressPosRspMsgIndicationBit".
 * When set, the tester is saying "do it, but do not answer if it worked".
 * Used for periodic TesterPresent so the bus is not filled with replies.
 * A negative response is still sent - errors are never silent.
 */
#define UDS_SUPPRESS_POSITIVE_RESPONSE_BIT      0x80U
#define UDS_SUBFUNCTION_MASK                    0x7FU

/* ========================================================================= */
/* Negative Response Codes (ISO 14229-1 Annex A)                             */
/* ========================================================================= */

#define UDS_NRC_POSITIVE_RESPONSE                       0x00U /* internal: "no error" */
#define UDS_NRC_GENERAL_REJECT                          0x10U
#define UDS_NRC_SERVICE_NOT_SUPPORTED                   0x11U
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED               0x12U
#define UDS_NRC_INCORRECT_MESSAGE_LENGTH                0x13U
#define UDS_NRC_RESPONSE_TOO_LONG                       0x14U
#define UDS_NRC_CONDITIONS_NOT_CORRECT                  0x22U
#define UDS_NRC_REQUEST_SEQUENCE_ERROR                  0x24U
#define UDS_NRC_REQUEST_OUT_OF_RANGE                    0x31U
#define UDS_NRC_SECURITY_ACCESS_DENIED                  0x33U
#define UDS_NRC_INVALID_KEY                             0x35U
#define UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS             0x36U
#define UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED         0x37U
#define UDS_NRC_GENERAL_PROGRAMMING_FAILURE             0x72U
#define UDS_NRC_RESPONSE_PENDING                        0x78U
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_SESSION    0x7EU
#define UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION        0x7FU

/* ========================================================================= */
/* Diagnostic sessions (service 0x10)                                        */
/* ========================================================================= */

/**
 * A session is a mode that decides which services are allowed.
 *
 *   DEFAULT    Entered at power-up. Read-only, safe: reading data and DTCs.
 *   EXTENDED   Unlocks writing, security access, routines, DTC control.
 *   PROGRAMMING Used by a bootloader to reflash the ECU. Not supported here.
 *
 * Restricting dangerous services to a session that must be entered
 * deliberately means a tester cannot trigger them by accident.
 */
typedef enum
{
    UDS_SESSION_DEFAULT     = 0x01,
    UDS_SESSION_PROGRAMMING = 0x02,
    UDS_SESSION_EXTENDED    = 0x03
} UdsSession_t;

/* ECUReset (0x11) sub-functions */
#define UDS_RESET_HARD                          0x01U
#define UDS_RESET_SOFT                          0x03U

/* SecurityAccess (0x27) sub-functions: odd = request seed, even = send key. */
#define UDS_SECURITY_REQUEST_SEED               0x01U
#define UDS_SECURITY_SEND_KEY                   0x02U

/* ReadDTCInformation (0x19) sub-functions */
#define UDS_RDTC_NUMBER_BY_STATUS_MASK          0x01U
#define UDS_RDTC_DTC_BY_STATUS_MASK             0x02U
#define UDS_RDTC_SNAPSHOT_BY_DTC_NUMBER         0x04U
#define UDS_RDTC_EXT_DATA_BY_DTC_NUMBER         0x06U
#define UDS_RDTC_SUPPORTED_DTC                  0x0AU

/** DTCFormatIdentifier reported by 0x19 0x01: ISO 14229-1 3-byte format. */
#define UDS_DTC_FORMAT_ISO14229_1               0x01U

/** 0xFFFFFF in ClearDiagnosticInformation means "all groups". */
#define UDS_DTC_GROUP_ALL                       0xFFFFFFUL

/* RoutineControl (0x31) sub-functions */
#define UDS_ROUTINE_START                       0x01U
#define UDS_ROUTINE_STOP                        0x02U
#define UDS_ROUTINE_REQUEST_RESULTS             0x03U

/* ControlDTCSetting (0x85) sub-functions */
#define UDS_DTC_SETTING_ON                      0x01U
#define UDS_DTC_SETTING_OFF                     0x02U

#endif /* DIAG_UDS_DEFS_H */
