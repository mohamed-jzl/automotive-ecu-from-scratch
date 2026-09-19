/**
 * @file    diag_manager.h
 * @brief   Glue for the diagnostic stack: CAN -> ISO-TP -> UDS -> ISO-TP -> CAN,
 *          plus DTC persistence and deferred ECU reset.
 *
 *      CAN driver                         diag_manager                      application
 *      ----------        frame            ------------                      -----------
 *      0x7E0/0x7DF  ------------------>   IsoTp_OnFrame
 *                                         IsoTp_Receive  --> UdsServer ---> DID / routine
 *                                                                           callbacks
 *      0x7E8        <------------------   IsoTp_Send     <-- response
 *
 *                                         DtcManager  <---------------  fault reports
 *                                             |
 *                                         NvmStore  --> flash (via callbacks)
 *
 * Everything hardware-specific arrives through DiagConfig_t: how to send a CAN
 * frame, how to reset the MCU, how to erase and program flash. The manager
 * itself contains no HAL call, so the complete stack - this file included -
 * also runs on a PC as a software-in-the-loop (SIL) target, where the Python
 * test suite talks to it with the same UDS library it uses on the real board.
 */

#ifndef DIAG_DIAG_MANAGER_H
#define DIAG_DIAG_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "dtc_manager.h"
#include "isotp.h"
#include "nvm_store.h"
#include "uds_server.h"

/** Length of a Vehicle Identification Number (ISO 3779). */
#define DIAG_VIN_LENGTH     17U

/**
 * @brief Everything the diagnostic stack needs from the platform and the application.
 */
typedef struct
{
    /* platform */
    IsoTpSendFrameFn_t       send_frame;        /**< Put one CAN frame on the bus  */
    void                    *send_context;
    void                   (*system_reset)(void); /**< Reset the MCU; never returns on target */
    const NvmFlashIf_t      *nvm;               /**< NULL disables persistence      */
    uint32_t                 entropy;           /**< Seeds the security PRNG        */

    /* application */
    const UdsDidEntry_t     *dids;
    uint8_t                  did_count;
    const UdsRoutineEntry_t *routines;
    uint8_t                  routine_count;
    const DtcDefinition_t   *dtcs;
    uint8_t                  dtc_count;
} DiagConfig_t;

/**
 * @brief Initialise the whole diagnostic stack and restore saved data.
 *
 * MAY ERASE A FLASH SECTOR (via NvmStore_Maintain): must be called at start-up,
 * before the watchdog is armed.
 *
 * @return true if non-volatile storage is available and was read successfully
 *         (or was empty on first boot). false means DTCs will not survive a
 *         reset; the stack still works otherwise.
 */
bool DiagManager_Init(const DiagConfig_t *config, uint32_t now_ms);

/**
 * @brief Offer a received CAN frame to the diagnostic stack.
 *
 * @return true if the frame was a diagnostic frame and has been consumed.
 */
bool DiagManager_OnCanFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc,
                            uint32_t now_ms);

/**
 * @brief Process pending requests, advance ISO-TP, save DTCs, perform a
 *        requested reset. Call every 5 ms.
 */
void DiagManager_Poll(uint32_t now_ms);

/**
 * @brief Start a new DTC operation cycle. The application calls this at each
 *        ignition-on (OFF -> ACC transition).
 */
void DiagManager_StartOperationCycle(void);

/** @brief Current UDS session, for the ActiveDiagnosticSession DID (0xF186). */
UdsSession_t DiagManager_GetSession(void);

/** @brief Copy the stored VIN (17 ASCII characters, not NUL-terminated). */
void DiagManager_GetVin(uint8_t out[DIAG_VIN_LENGTH]);

/**
 * @brief Store a new VIN. It is written to flash on the next save.
 *        Validation is the caller's job (see diag_app.c).
 */
void DiagManager_SetVin(const uint8_t vin[DIAG_VIN_LENGTH]);

/** @brief Active NVM region usage, 0-100 %, or 0 without persistence. */
uint8_t DiagManager_GetNvmUsagePercent(void);

#endif /* DIAG_DIAG_MANAGER_H */
