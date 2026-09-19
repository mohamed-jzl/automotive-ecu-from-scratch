/**
 * @file    uds_server.c
 * @brief   UDS service dispatch and handlers - see uds_server.h.
 *
 * Structure: a constant service table maps each SID to a handler and to the
 * sessions it is allowed in. The dispatcher applies the checks common to every
 * service; each handler applies the checks specific to its own service. Adding
 * a service is one table row and one function - the same table-driven pattern
 * as the vehicle state machine.
 */

#include "uds_server.h"

#include "dtc_manager.h"
#include "ecu_config.h"
#include "uds_security.h"

#include <string.h>

/* ========================================================================= */
/* Types                                                                     */
/* ========================================================================= */

typedef struct
{
    const uint8_t *data;
    uint16_t       length;
    bool           functional;
    uint32_t       now_ms;
} UdsRequest_t;

/**
 * @brief Response under construction.
 *
 * Every byte goes through response_put(), which refuses to write past the end
 * and raises the overflow flag instead. The dispatcher turns that flag into
 * NRC 0x14 (responseTooLong). A handler therefore cannot overrun the buffer
 * however many DTCs or DIDs it reports.
 */
typedef struct
{
    uint8_t  *data;
    uint16_t  size;
    uint16_t  length;
    bool      overflow;
} UdsResponse_t;

typedef uint8_t (*UdsServiceHandler_t)(const UdsRequest_t *req, UdsResponse_t *rsp);

/* Session masks: one bit per session so a service can list where it is allowed. */
#define SESSION_BIT(s)      ((uint8_t)(1U << ((uint8_t)(s) - 1U)))
#define IN_DEFAULT          SESSION_BIT(UDS_SESSION_DEFAULT)
#define IN_EXTENDED         SESSION_BIT(UDS_SESSION_EXTENDED)
#define IN_ALL              ((uint8_t)0xFFU)

typedef struct
{
    uint8_t             sid;
    bool                has_subfunction;
    uint8_t             allowed_sessions;
    UdsServiceHandler_t handler;
} UdsServiceEntry_t;

/** Limit on DIDs per 0x22 request, to bound the response and the work done. */
#define MAX_DIDS_PER_REQUEST    8U

/* Identifiers used inside the DTC snapshot record (freeze frame). */
#define SNAPSHOT_RECORD_NUMBER  0x01U
#define SNAPSHOT_DID_BATTERY    0x0100U
#define SNAPSHOT_DID_STATE      0x0101U

/* Extended data record numbers for 0x19 0x06. */
#define EXT_RECORD_OCCURRENCE   0x01U
#define EXT_RECORD_AGING        0x02U
#define EXT_RECORD_ALL          0xFFU

/* ========================================================================= */
/* Server state                                                              */
/* ========================================================================= */

static UdsServerConfig_t s_config;
static UdsSession_t      s_session;
static uint32_t          s_last_request_ms;

static bool              s_unlocked;
static bool              s_seed_pending;
static uint32_t          s_seed;
static uint8_t           s_failed_attempts;
static bool              s_lockout_active;
static uint32_t          s_lockout_until_ms;

static bool              s_reset_pending;
static uint8_t           s_reset_type;

/* ========================================================================= */
/* Helpers                                                                   */
/* ========================================================================= */

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void response_put(UdsResponse_t *rsp, uint8_t value)
{
    if (rsp->length >= rsp->size)
    {
        rsp->overflow = true;
        return;
    }
    rsp->data[rsp->length++] = value;
}

static void response_put16(UdsResponse_t *rsp, uint16_t value)
{
    /* UDS carries multi-byte values big-endian (most significant byte first).
     * Note the contrast with our CAN signals, which are little-endian: byte
     * order is a property of each protocol, never an assumption. */
    response_put(rsp, (uint8_t)(value >> 8));
    response_put(rsp, (uint8_t)(value & 0xFFU));
}

static void response_put24(UdsResponse_t *rsp, uint32_t value)
{
    response_put(rsp, (uint8_t)((value >> 16) & 0xFFU));
    response_put(rsp, (uint8_t)((value >> 8) & 0xFFU));
    response_put(rsp, (uint8_t)(value & 0xFFU));
}

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t read_be24(const uint8_t *p)
{
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

static uint32_t read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | p[3];
}

/**
 * @brief Change session. Every session change re-locks security.
 *
 * Re-locking on ANY transition - not just on the way back to default - is an
 * ISO 14229-1 requirement, and the reason is simple: security is granted to a
 * session, so a new session starts without it.
 */
static void enter_session(UdsSession_t session)
{
    s_session      = session;
    s_unlocked     = false;
    s_seed_pending = false;

    if (session == UDS_SESSION_DEFAULT)
    {
        /* ControlDTCSetting(off) must never outlive the extended session.
         * Otherwise a tester that disconnects mid-repair would leave the ECU
         * blind to real faults until the next power cycle. */
        DtcManager_SetUpdatesEnabled(true);
    }
}

static bool security_lockout_active(uint32_t now_ms)
{
    if (s_lockout_active && time_reached(now_ms, s_lockout_until_ms))
    {
        s_lockout_active = false;
    }
    return s_lockout_active;
}

static const UdsDidEntry_t *find_did(uint16_t did)
{
    for (uint8_t i = 0U; i < s_config.did_count; i++)
    {
        if (s_config.dids[i].did == did)
        {
            return &s_config.dids[i];
        }
    }
    return NULL;
}

static const UdsRoutineEntry_t *find_routine(uint16_t rid)
{
    for (uint8_t i = 0U; i < s_config.routine_count; i++)
    {
        if (s_config.routines[i].rid == rid)
        {
            return &s_config.routines[i];
        }
    }
    return NULL;
}

/* ========================================================================= */
/* 0x10 DiagnosticSessionControl                                             */
/* ========================================================================= */

static uint8_t handle_session_control(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length != 2U)
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint8_t session = req->data[1] & UDS_SUBFUNCTION_MASK;

    /* The programming session exists in the standard but needs a bootloader,
     * which this ECU does not have. Refusing it explicitly is better than
     * entering a mode that cannot do what the tester expects. */
    if ((session != UDS_SESSION_DEFAULT) && (session != UDS_SESSION_EXTENDED))
    {
        return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    enter_session((UdsSession_t)session);

    /* The response tells the tester how long it may have to wait for us:
     * P2 in ms, and P2* in units of 10 ms, both big-endian. */
    response_put(rsp, UDS_SID_DIAGNOSTIC_SESSION_CONTROL + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put(rsp, session);
    response_put16(rsp, (uint16_t)ECU_UDS_P2_SERVER_MS);
    response_put16(rsp, (uint16_t)(ECU_UDS_P2_STAR_SERVER_MS / 10U));
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* 0x11 ECUReset                                                             */
/* ========================================================================= */

static uint8_t handle_ecu_reset(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length != 2U)
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint8_t type = req->data[1] & UDS_SUBFUNCTION_MASK;

    if ((type != UDS_RESET_HARD) && (type != UDS_RESET_SOFT))
    {
        return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    /* Only recorded here. The reset itself happens later, once the positive
     * response has actually been transmitted - see UdsServer_TakeResetRequest. */
    s_reset_pending = true;
    s_reset_type    = type;

    response_put(rsp, UDS_SID_ECU_RESET + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put(rsp, type);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* 0x3E TesterPresent                                                        */
/* ========================================================================= */

static uint8_t handle_tester_present(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length != 2U)
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    if ((req->data[1] & UDS_SUBFUNCTION_MASK) != 0x00U)
    {
        return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    /* Nothing else to do: merely receiving a request has already restarted
     * the S3 timer in the dispatcher. That is the whole point of this service. */
    response_put(rsp, UDS_SID_TESTER_PRESENT + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put(rsp, 0x00U);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* 0x22 ReadDataByIdentifier                                                 */
/* ========================================================================= */

static uint8_t handle_read_did(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    /* SID followed by one or more 2-byte DIDs: length must be 1 + 2n. */
    if ((req->length < 3U) || (((req->length - 1U) % 2U) != 0U))
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint16_t did_count = (uint16_t)((req->length - 1U) / 2U);

    if (did_count > MAX_DIDS_PER_REQUEST)
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    response_put(rsp, UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET);

    uint16_t supported = 0U;

    for (uint16_t i = 0U; i < did_count; i++)
    {
        const uint16_t       did   = read_be16(&req->data[1U + (2U * i)]);
        const UdsDidEntry_t *entry = find_did(did);

        /* ISO 14229-1: unsupported DIDs are left out of the response. Only
         * when NONE of the requested DIDs is supported is it an error. */
        if ((entry == NULL) || (entry->read == NULL))
        {
            continue;
        }

        if ((uint32_t)rsp->length + 2U + entry->length > rsp->size)
        {
            return UDS_NRC_RESPONSE_TOO_LONG;
        }

        response_put16(rsp, did);

        const uint8_t nrc = entry->read(&rsp->data[rsp->length]);
        if (nrc != UDS_NRC_POSITIVE_RESPONSE)
        {
            return nrc;
        }

        rsp->length = (uint16_t)(rsp->length + entry->length);
        supported++;
    }

    return (supported > 0U) ? UDS_NRC_POSITIVE_RESPONSE : UDS_NRC_REQUEST_OUT_OF_RANGE;
}

/* ========================================================================= */
/* 0x2E WriteDataByIdentifier                                                */
/* ========================================================================= */

static uint8_t handle_write_did(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length < 4U)       /* SID + DID + at least one data byte */
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint16_t       did   = read_be16(&req->data[1]);
    const UdsDidEntry_t *entry = find_did(did);

    if ((entry == NULL) || (entry->write == NULL))
    {
        return UDS_NRC_REQUEST_OUT_OF_RANGE;
    }

    if (!s_unlocked)
    {
        return UDS_NRC_SECURITY_ACCESS_DENIED;
    }

    if ((uint16_t)(req->length - 3U) != entry->length)
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint8_t nrc = entry->write(&req->data[3]);
    if (nrc != UDS_NRC_POSITIVE_RESPONSE)
    {
        return nrc;
    }

    response_put(rsp, UDS_SID_WRITE_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put16(rsp, did);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* 0x27 SecurityAccess                                                       */
/* ========================================================================= */

static uint8_t handle_security_access(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    const uint8_t type = req->data[1] & UDS_SUBFUNCTION_MASK;

    if (type == UDS_SECURITY_REQUEST_SEED)
    {
        if (req->length != 2U)
        {
            return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
        }

        /* During the penalty delay no seed is handed out, so no key can be
         * tried: this is what makes brute-forcing the key impractical. */
        if (security_lockout_active(req->now_ms))
        {
            return UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED;
        }

        uint32_t seed = 0U;     /* all zeros = "already unlocked" (ISO rule) */

        if (!s_unlocked)
        {
            seed           = UdsSecurity_NextSeed(req->now_ms);
            s_seed         = seed;
            s_seed_pending = true;
        }

        response_put(rsp, UDS_SID_SECURITY_ACCESS + UDS_POSITIVE_RESPONSE_OFFSET);
        response_put(rsp, UDS_SECURITY_REQUEST_SEED);
        response_put16(rsp, (uint16_t)(seed >> 16));
        response_put16(rsp, (uint16_t)(seed & 0xFFFFU));
        return UDS_NRC_POSITIVE_RESPONSE;
    }

    if (type == UDS_SECURITY_SEND_KEY)
    {
        if (req->length != 6U)
        {
            return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
        }

        /* A key is only meaningful as the answer to a seed we just issued. */
        if (!s_seed_pending)
        {
            return UDS_NRC_REQUEST_SEQUENCE_ERROR;
        }

        /* Single use: whatever happens next, this seed is spent. Allowing
         * several keys per seed would let an attacker try keys without ever
         * seeing the seed change. */
        s_seed_pending = false;

        const uint32_t key = read_be32(&req->data[2]);

        if (key != UdsSecurity_ComputeKey(s_seed))
        {
            s_failed_attempts++;

            if (s_failed_attempts >= ECU_UDS_SECURITY_MAX_ATTEMPTS)
            {
                s_failed_attempts  = 0U;
                s_lockout_active   = true;
                s_lockout_until_ms = req->now_ms + ECU_UDS_SECURITY_LOCKOUT_MS;
                return UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS;
            }
            return UDS_NRC_INVALID_KEY;
        }

        s_unlocked        = true;
        s_failed_attempts = 0U;

        response_put(rsp, UDS_SID_SECURITY_ACCESS + UDS_POSITIVE_RESPONSE_OFFSET);
        response_put(rsp, UDS_SECURITY_SEND_KEY);
        return UDS_NRC_POSITIVE_RESPONSE;
    }

    return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
}

/* ========================================================================= */
/* 0x19 ReadDTCInformation                                                   */
/* ========================================================================= */

/** Append "DTC (3 bytes) + status (1 byte)" for one DTC. */
static void put_dtc_and_status(UdsResponse_t *rsp, uint8_t index)
{
    response_put24(rsp, DtcManager_GetCode(index));
    response_put(rsp, DtcManager_GetStatus(index));
}

static uint8_t handle_read_dtc(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    const uint8_t report_type = req->data[1] & UDS_SUBFUNCTION_MASK;

    response_put(rsp, UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put(rsp, report_type);

    switch (report_type)
    {
        case UDS_RDTC_NUMBER_BY_STATUS_MASK:        /* 19 01 <mask> */
        {
            if (req->length != 3U) { return UDS_NRC_INCORRECT_MESSAGE_LENGTH; }

            const uint8_t count = DtcManager_CountByMask(req->data[2]);

            response_put(rsp, DTC_STATUS_AVAILABILITY_MASK);
            response_put(rsp, UDS_DTC_FORMAT_ISO14229_1);
            response_put16(rsp, count);
            return UDS_NRC_POSITIVE_RESPONSE;
        }

        case UDS_RDTC_DTC_BY_STATUS_MASK:           /* 19 02 <mask> */
        {
            if (req->length != 3U) { return UDS_NRC_INCORRECT_MESSAGE_LENGTH; }

            const uint8_t mask = req->data[2];

            response_put(rsp, DTC_STATUS_AVAILABILITY_MASK);
            for (uint8_t i = 0U; i < DtcManager_GetCount(); i++)
            {
                if ((DtcManager_GetStatus(i) & mask & DTC_STATUS_AVAILABILITY_MASK) != 0U)
                {
                    put_dtc_and_status(rsp, i);
                }
            }
            return UDS_NRC_POSITIVE_RESPONSE;
        }

        case UDS_RDTC_SUPPORTED_DTC:                /* 19 0A */
        {
            if (req->length != 2U) { return UDS_NRC_INCORRECT_MESSAGE_LENGTH; }

            response_put(rsp, DTC_STATUS_AVAILABILITY_MASK);
            for (uint8_t i = 0U; i < DtcManager_GetCount(); i++)
            {
                put_dtc_and_status(rsp, i);
            }
            return UDS_NRC_POSITIVE_RESPONSE;
        }

        case UDS_RDTC_SNAPSHOT_BY_DTC_NUMBER:       /* 19 04 <DTC:3> <record> */
        {
            if (req->length != 6U) { return UDS_NRC_INCORRECT_MESSAGE_LENGTH; }

            const int16_t index  = DtcManager_FindByCode(read_be24(&req->data[2]));
            const uint8_t record = req->data[5];

            if ((index < 0) ||
                ((record != SNAPSHOT_RECORD_NUMBER) && (record != 0xFFU)))
            {
                return UDS_NRC_REQUEST_OUT_OF_RANGE;
            }

            const DtcRecord_t *dtc = DtcManager_GetRecord((uint8_t)index);

            put_dtc_and_status(rsp, (uint8_t)index);

            /* A DTC that never failed has no freeze frame: the response then
             * simply ends after the status byte, as the standard allows. */
            if (dtc->snapshot_valid)
            {
                response_put(rsp, SNAPSHOT_RECORD_NUMBER);
                response_put(rsp, 2U);                          /* identifiers */
                response_put16(rsp, SNAPSHOT_DID_BATTERY);
                response_put16(rsp, dtc->snapshot.battery_mv);
                response_put16(rsp, SNAPSHOT_DID_STATE);
                response_put(rsp, dtc->snapshot.vehicle_state);
            }
            return UDS_NRC_POSITIVE_RESPONSE;
        }

        case UDS_RDTC_EXT_DATA_BY_DTC_NUMBER:       /* 19 06 <DTC:3> <record> */
        {
            if (req->length != 6U) { return UDS_NRC_INCORRECT_MESSAGE_LENGTH; }

            const int16_t index  = DtcManager_FindByCode(read_be24(&req->data[2]));
            const uint8_t record = req->data[5];

            if ((index < 0) ||
                ((record != EXT_RECORD_OCCURRENCE) && (record != EXT_RECORD_AGING) &&
                 (record != EXT_RECORD_ALL)))
            {
                return UDS_NRC_REQUEST_OUT_OF_RANGE;
            }

            const DtcRecord_t *dtc = DtcManager_GetRecord((uint8_t)index);

            put_dtc_and_status(rsp, (uint8_t)index);

            if ((record == EXT_RECORD_OCCURRENCE) || (record == EXT_RECORD_ALL))
            {
                response_put(rsp, EXT_RECORD_OCCURRENCE);
                response_put(rsp, dtc->occurrence_counter);
            }
            if ((record == EXT_RECORD_AGING) || (record == EXT_RECORD_ALL))
            {
                response_put(rsp, EXT_RECORD_AGING);
                response_put(rsp, dtc->aging_counter);
            }
            return UDS_NRC_POSITIVE_RESPONSE;
        }

        default:
            return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }
}

/* ========================================================================= */
/* 0x14 ClearDiagnosticInformation                                           */
/* ========================================================================= */

static uint8_t handle_clear_dtc(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length != 4U)      /* SID + 3-byte group */
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    if (!DtcManager_Clear(read_be24(&req->data[1])))
    {
        return UDS_NRC_REQUEST_OUT_OF_RANGE;
    }

    response_put(rsp, UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* 0x31 RoutineControl                                                       */
/* ========================================================================= */

static uint8_t handle_routine_control(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length < 4U)       /* SID + sub-function + 2-byte RID */
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint8_t control = req->data[1] & UDS_SUBFUNCTION_MASK;

    if ((control < UDS_ROUTINE_START) || (control > UDS_ROUTINE_REQUEST_RESULTS))
    {
        return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    const uint16_t           rid   = read_be16(&req->data[2]);
    const UdsRoutineEntry_t *entry = find_routine(rid);

    if (entry == NULL)
    {
        return UDS_NRC_REQUEST_OUT_OF_RANGE;
    }

    if (entry->requires_security && (!s_unlocked))
    {
        return UDS_NRC_SECURITY_ACCESS_DENIED;
    }

    const UdsRoutineFn_t function =
        (control == UDS_ROUTINE_START) ? entry->start :
        (control == UDS_ROUTINE_STOP)  ? entry->stop  : entry->results;

    if (function == NULL)
    {
        return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    response_put(rsp, UDS_SID_ROUTINE_CONTROL + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put(rsp, control);
    response_put16(rsp, rid);

    if (rsp->overflow)
    {
        return UDS_NRC_RESPONSE_TOO_LONG;
    }

    uint16_t written = 0U;
    const uint8_t nrc = function(&req->data[4], (uint16_t)(req->length - 4U),
                                 &rsp->data[rsp->length],
                                 (uint16_t)(rsp->size - rsp->length), &written);
    if (nrc != UDS_NRC_POSITIVE_RESPONSE)
    {
        return nrc;
    }

    rsp->length = (uint16_t)(rsp->length + written);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* 0x85 ControlDTCSetting                                                    */
/* ========================================================================= */

static uint8_t handle_control_dtc_setting(const UdsRequest_t *req, UdsResponse_t *rsp)
{
    if (req->length != 2U)
    {
        return UDS_NRC_INCORRECT_MESSAGE_LENGTH;
    }

    const uint8_t setting = req->data[1] & UDS_SUBFUNCTION_MASK;

    if (setting == UDS_DTC_SETTING_ON)
    {
        DtcManager_SetUpdatesEnabled(true);
    }
    else if (setting == UDS_DTC_SETTING_OFF)
    {
        DtcManager_SetUpdatesEnabled(false);
    }
    else
    {
        return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    response_put(rsp, UDS_SID_CONTROL_DTC_SETTING + UDS_POSITIVE_RESPONSE_OFFSET);
    response_put(rsp, setting);
    return UDS_NRC_POSITIVE_RESPONSE;
}

/* ========================================================================= */
/* Service table                                                             */
/* ========================================================================= */

static const UdsServiceEntry_t SERVICE_TABLE[] =
{
    /* SID                                   subfn  sessions                 handler */
    { UDS_SID_DIAGNOSTIC_SESSION_CONTROL,   true,  IN_ALL,                  handle_session_control     },
    { UDS_SID_ECU_RESET,                    true,  IN_ALL,                  handle_ecu_reset           },
    { UDS_SID_CLEAR_DIAGNOSTIC_INFORMATION, false, IN_DEFAULT | IN_EXTENDED, handle_clear_dtc          },
    { UDS_SID_READ_DTC_INFORMATION,         true,  IN_DEFAULT | IN_EXTENDED, handle_read_dtc           },
    { UDS_SID_READ_DATA_BY_IDENTIFIER,      false, IN_DEFAULT | IN_EXTENDED, handle_read_did           },
    { UDS_SID_SECURITY_ACCESS,              true,  IN_EXTENDED,             handle_security_access     },
    { UDS_SID_WRITE_DATA_BY_IDENTIFIER,     false, IN_EXTENDED,             handle_write_did           },
    { UDS_SID_ROUTINE_CONTROL,              true,  IN_EXTENDED,             handle_routine_control     },
    { UDS_SID_TESTER_PRESENT,               true,  IN_ALL,                  handle_tester_present      },
    { UDS_SID_CONTROL_DTC_SETTING,          true,  IN_EXTENDED,             handle_control_dtc_setting },
};

#define SERVICE_COUNT  (sizeof(SERVICE_TABLE) / sizeof(SERVICE_TABLE[0]))

static const UdsServiceEntry_t *find_service(uint8_t sid)
{
    for (uint8_t i = 0U; i < (uint8_t)SERVICE_COUNT; i++)
    {
        if (SERVICE_TABLE[i].sid == sid)
        {
            return &SERVICE_TABLE[i];
        }
    }
    return NULL;
}

/**
 * @brief Build a negative response, applying the functional-addressing rule.
 *
 * For a functional (broadcast) request, ISO 14229-1 says an ECU must stay
 * silent rather than answer "not supported" (0x11, 0x12, 0x31, 0x7E, 0x7F).
 * Otherwise one broadcast to 40 ECUs would produce 40 refusals from every ECU
 * that simply does not implement the service. Real errors are still reported.
 */
static uint16_t negative_response(uint8_t sid, uint8_t nrc, bool functional,
                                  uint8_t *response)
{
    if (functional &&
        ((nrc == UDS_NRC_SERVICE_NOT_SUPPORTED) ||
         (nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED) ||
         (nrc == UDS_NRC_REQUEST_OUT_OF_RANGE) ||
         (nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_SESSION) ||
         (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION)))
    {
        return 0U;
    }

    response[0] = UDS_NEGATIVE_RESPONSE_SID;
    response[1] = sid;
    response[2] = nrc;
    return 3U;
}

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

void UdsServer_Init(const UdsServerConfig_t *config, uint32_t now_ms)
{
    if (config != NULL)
    {
        s_config = *config;
    }
    else
    {
        (void)memset(&s_config, 0, sizeof(s_config));
    }

    s_failed_attempts = 0U;
    s_lockout_active  = false;
    s_reset_pending   = false;
    s_reset_type      = 0U;
    s_last_request_ms = now_ms;

    enter_session(UDS_SESSION_DEFAULT);
}

uint16_t UdsServer_ProcessRequest(const uint8_t *request, uint16_t request_length,
                                  bool functional,
                                  uint8_t *response, uint16_t response_size,
                                  uint32_t now_ms)
{
    if ((request == NULL) || (request_length == 0U) ||
        (response == NULL) || (response_size < 3U))
    {
        return 0U;
    }

    /* Any request at all restarts the S3 timer. */
    s_last_request_ms = now_ms;

    const uint8_t            sid     = request[0];
    const UdsServiceEntry_t *service = find_service(sid);

    /* Check 1: do we implement this service at all? */
    if (service == NULL)
    {
        return negative_response(sid, UDS_NRC_SERVICE_NOT_SUPPORTED, functional, response);
    }

    /* Check 2: is it allowed in the current session? */
    if ((service->allowed_sessions & SESSION_BIT(s_session)) == 0U)
    {
        return negative_response(sid, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION,
                                 functional, response);
    }

    /* Check 3: services with a sub-function need at least SID + sub-function. */
    if (service->has_subfunction && (request_length < 2U))
    {
        return negative_response(sid, UDS_NRC_INCORRECT_MESSAGE_LENGTH, functional, response);
    }

    const bool suppress_positive = service->has_subfunction &&
                                   ((request[1] & UDS_SUPPRESS_POSITIVE_RESPONSE_BIT) != 0U);

    const UdsRequest_t req = { request, request_length, functional, now_ms };
    UdsResponse_t      rsp = { response, response_size, 0U, false };

    uint8_t nrc = service->handler(&req, &rsp);

    if ((nrc == UDS_NRC_POSITIVE_RESPONSE) && rsp.overflow)
    {
        nrc = UDS_NRC_RESPONSE_TOO_LONG;
    }

    if (nrc != UDS_NRC_POSITIVE_RESPONSE)
    {
        /* The suppress bit never hides an error: the tester asked to skip the
         * "OK", not to be kept in the dark about a failure. */
        return negative_response(sid, nrc, functional, response);
    }

    return suppress_positive ? 0U : rsp.length;
}

void UdsServer_Poll(uint32_t now_ms)
{
    /* S3 timeout: a non-default session nobody is using falls back to default,
     * which also re-locks security. An abandoned laptop must never leave a car
     * sitting unlocked in an extended session. */
    if ((s_session != UDS_SESSION_DEFAULT) &&
        time_reached(now_ms, s_last_request_ms + ECU_UDS_S3_SERVER_MS))
    {
        enter_session(UDS_SESSION_DEFAULT);
    }
}

UdsSession_t UdsServer_GetSession(void)
{
    return s_session;
}

bool UdsServer_IsUnlocked(void)
{
    return s_unlocked;
}

bool UdsServer_TakeResetRequest(uint8_t *reset_type)
{
    if (!s_reset_pending)
    {
        return false;
    }

    s_reset_pending = false;

    if (reset_type != NULL)
    {
        *reset_type = s_reset_type;
    }
    return true;
}
