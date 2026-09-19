/**
 * @file    dtc_manager.h
 * @brief   Diagnostic Trouble Code memory, following ISO 14229-1 Annex D.
 *
 * Fault vs DTC
 * ------------
 * The fault manager (src/services/fault_manager.c) answers "is something wrong
 * RIGHT NOW?" - it drives the ECU's live reaction, like entering the FAULT
 * state. The DTC manager answers a different question: "what has gone wrong,
 * and when, so that a technician can find out later?" It is the ECU's memory
 * of its own problems, read by a workshop tester long after the event.
 *
 * DTC codes
 * ---------
 * A DTC is three bytes. The first two follow SAE J2012 and are what people
 * quote ("P0562"); the third is the Failure Type Byte (FTB) that says *how*
 * the component failed.
 *
 *      P0562-00 -> 0x05 0x62 0x00        B1001-49 -> 0x90 0x01 0x49
 *
 *      bits 15-14 of the code = letter:   00 P  powertrain    01 C  chassis
 *                                         10 B  body          11 U  network
 *      then four hex digits (the first limited to 0-3).
 *
 * The status byte
 * ---------------
 * Every DTC carries one status byte. Each bit answers one yes/no question.
 * Reading these bits is the most-asked practical UDS interview question:
 *
 *   bit 0  TF      testFailed                    failing right now
 *   bit 1  TFTOC   testFailedThisOperationCycle  failed at least once this cycle
 *   bit 2  PDTC    pendingDTC                    failed this or the last cycle
 *   bit 3  CDTC    confirmedDTC                  failure confirmed and stored
 *   bit 4  TNCSLC  testNotCompletedSinceLastClear
 *   bit 5  TFSLC   testFailedSinceLastClear
 *   bit 6  TNCTOC  testNotCompletedThisOperationCycle
 *   bit 7  WIR     warningIndicatorRequested     the dashboard lamp is on
 *
 * Example: status 0x2E = 0b0010_1110 = TFTOC + PDTC + CDTC + TFSLC. The fault
 * happened this cycle and is stored, but is NOT present at this moment (TF=0):
 * an intermittent fault, which is exactly the kind a technician needs DTC
 * memory to find.
 *
 * Operation cycle and aging
 * -------------------------
 * An operation cycle is one "use" of the vehicle; here, one ignition cycle.
 * At the start of each cycle the "this cycle" bits reset, and every confirmed
 * DTC that did not fail again moves one step closer to aging out. After
 * ECU_DTC_AGING_THRESHOLD clean cycles it is erased automatically: a fault
 * that healed itself months ago should not keep a car in the workshop.
 *
 * Confirmation
 * ------------
 * In many ECUs a DTC becomes "confirmed" only after failing in two separate
 * cycles. Here the fault manager has already matured the fault before
 * reporting it (3 consecutive detections), so the first report of a matured
 * fault confirms the DTC directly. The two layers are deliberately not both
 * debouncing the same signal.
 *
 * Pure module: no hardware, no clock. Fully covered by tests/test_dtc_manager.c.
 */

#ifndef DIAG_DTC_MANAGER_H
#define DIAG_DTC_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"

/* Status bits (ISO 14229-1 D.2) */
#define DTC_STATUS_TEST_FAILED                  0x01U
#define DTC_STATUS_TEST_FAILED_THIS_CYCLE       0x02U
#define DTC_STATUS_PENDING                      0x04U
#define DTC_STATUS_CONFIRMED                    0x08U
#define DTC_STATUS_NOT_COMPLETED_SINCE_CLEAR    0x10U
#define DTC_STATUS_FAILED_SINCE_CLEAR           0x20U
#define DTC_STATUS_NOT_COMPLETED_THIS_CYCLE     0x40U
#define DTC_STATUS_WARNING_INDICATOR            0x80U

/** Every bit above is implemented, so every bit is "available" to a tester. */
#define DTC_STATUS_AVAILABILITY_MASK            0xFFU

/** Status of a DTC that has never been tested since it was last cleared. */
#define DTC_STATUS_INITIAL  (DTC_STATUS_NOT_COMPLETED_SINCE_CLEAR | \
                             DTC_STATUS_NOT_COMPLETED_THIS_CYCLE)

/** Bytes per DTC in the serialised (persisted) form. */
#define DTC_RECORD_SERIALISED_SIZE              8U

/**
 * @brief Static definition of one DTC, supplied by the application.
 */
typedef struct
{
    uint32_t code;              /**< 24-bit DTC: J2012 code + failure type byte */
    bool     warning_lamp;      /**< Does this fault light the dashboard lamp?  */
} DtcDefinition_t;

/**
 * @brief Environment captured when a DTC is first confirmed ("freeze frame").
 *
 * A DTC saying "battery voltage low" is far more useful when it also says
 * what the voltage was and what the vehicle was doing at the moment it
 * happened. The technician can then tell a flat battery (engine OFF, 10.2 V)
 * from a charging fault (engine RUN, 10.8 V).
 */
typedef struct
{
    uint16_t battery_mv;
    uint8_t  vehicle_state;
} DtcSnapshot_t;

/**
 * @brief Everything stored about one DTC.
 */
typedef struct
{
    uint8_t       status;
    uint8_t       occurrence_counter;   /**< Times TF went 0->1, saturating at 255 */
    uint8_t       aging_counter;        /**< Clean cycles since the last failure   */
    bool          snapshot_valid;
    DtcSnapshot_t snapshot;
} DtcRecord_t;

/**
 * @brief Install the DTC table and reset every DTC to its initial status.
 *
 * @param definitions  Application table. Must outlive the manager (static const).
 * @param count        Number of entries, at most ECU_DTC_MAX_COUNT.
 */
void DtcManager_Init(const DtcDefinition_t *definitions, uint8_t count);

/**
 * @brief Report the outcome of one test of a monitored condition.
 *
 * Call periodically for every DTC, pass or fail. Reporting passes matters:
 * that is what clears "test not completed" and proves the monitor ran.
 *
 * @param index        Position in the definition table.
 * @param failed       true if the condition is currently failing.
 * @param environment  Snapshot to store if this report confirms the DTC.
 *                     May be NULL.
 */
void DtcManager_ReportResult(uint8_t index, bool failed,
                             const DtcSnapshot_t *environment);

/**
 * @brief Begin a new operation cycle (called at each ignition-on).
 *
 * Clears the "this cycle" bits, clears PENDING for DTCs that passed the whole
 * previous cycle, and ages confirmed DTCs that did not fail again.
 */
void DtcManager_StartOperationCycle(void);

/**
 * @brief Enable or disable status updates (UDS ControlDTCSetting, 0x85).
 *
 * A technician disables DTC setting before deliberately provoking faults -
 * unplugging a sensor during a repair, for instance - so that the ECU does not
 * store dozens of DTCs caused by the repair itself.
 */
void DtcManager_SetUpdatesEnabled(bool enabled);
bool DtcManager_AreUpdatesEnabled(void);

/**
 * @brief Clear one DTC, or all of them (UDS ClearDiagnosticInformation, 0x14).
 *
 * @param group  A DTC code to clear that DTC, or 0xFFFFFF for all.
 * @return false if @p group matches no DTC (the caller answers NRC 0x31).
 */
bool DtcManager_Clear(uint32_t group);

/* --- queries --- */
uint8_t  DtcManager_GetCount(void);
uint32_t DtcManager_GetCode(uint8_t index);
uint8_t  DtcManager_GetStatus(uint8_t index);
const DtcRecord_t *DtcManager_GetRecord(uint8_t index);

/** @return Index of the DTC with this code, or -1. */
int16_t DtcManager_FindByCode(uint32_t code);

/**
 * @brief Count DTCs whose status matches a tester-supplied mask.
 *
 * ISO 14229-1 rule: a DTC matches when (status & mask) != 0 - "any of these
 * bits", not "all of them". Mask 0x08 therefore asks "which DTCs are
 * confirmed?", and mask 0xFF asks "which DTCs have any bit set at all?".
 */
uint8_t DtcManager_CountByMask(uint8_t mask);

/* --- persistence --- */

/** @brief true when something worth saving has changed since the last save. */
bool DtcManager_IsDirty(void);
void DtcManager_ClearDirty(void);

/**
 * @brief Serialise every record into bytes for non-volatile storage.
 *
 * Written field by field rather than with memcpy of the struct. A struct's
 * memory layout - padding, byte order, the size of bool - is up to the
 * compiler, so copying it raw would tie the stored format to one compiler's
 * choices. An explicit byte format stays readable after a toolchain upgrade.
 *
 * @return Number of bytes written, or 0 if @p size is too small.
 */
uint16_t DtcManager_Serialise(uint8_t *out, uint16_t size);

/**
 * @brief Restore records from bytes produced by DtcManager_Serialise().
 *
 * @return false if the data does not match the current DTC table (for example
 *         after a firmware update added a DTC). The caller then keeps the
 *         freshly initialised state instead of loading mismatched data.
 */
bool DtcManager_Deserialise(const uint8_t *in, uint16_t length);

#endif /* DIAG_DTC_MANAGER_H */
