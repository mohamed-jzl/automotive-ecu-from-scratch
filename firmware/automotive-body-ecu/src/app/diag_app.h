/**
 * @file    diag_app.h
 * @brief   What this particular ECU exposes through diagnostics.
 *
 * The diagnostic stack in src/diag/ is generic: it would work unchanged in a
 * door module or a seat controller. This file is the part that belongs to the
 * Body Control ECU specifically:
 *
 *   - the DID table   (which data a tester can read or write)
 *   - the DTC table   (which fault produces which trouble code)
 *   - the routines    (which actions a tester can trigger)
 *   - the bindings    (how the stack reaches CAN, flash and reset on this board)
 *
 * Data identifiers
 * ----------------
 *   DID     Name                        Len  Access         Format
 *   0xF190  VIN                         17   read / write*  ASCII
 *   0xF195  Software version             3   read           major, minor, patch
 *   0xF18C  ECU serial number           12   read           STM32 unique ID
 *   0xF186  Active diagnostic session    1   read           0x01 / 0x03
 *   0x0100  Battery voltage              2   read           mV, big-endian
 *   0x0101  Vehicle state                1   read           0=OFF .. 4=FAULT
 *   0x0102  Digital inputs               1   read           bit0 ign, 1 brake,
 *                                                           2 left, 3 right
 *   0x0103  Active fault bitmask         2   read           FaultId_t bits
 *   0x0104  Uptime                       4   read           seconds
 *
 *   * write requires the extended session and security access.
 *
 * 0xF1xx identifiers are standardised by ISO 14229-1 Annex C, so any generic
 * tester knows what 0xF190 means. 0x0100-0x0104 are this project's own.
 *
 * DTCs
 * ----
 *   Fault            DTC        Bytes      Meaning
 *   BATT_UNDERVOLT   P0562-00   05 62 00   System voltage low
 *   BATT_OVERVOLT    P0563-00   05 63 00   System voltage high
 *   ADC_FAILURE      B1001-49   90 01 49   Battery sense - internal failure
 *   CAN_BUS_OFF      U0001-88   C0 01 88   High-speed CAN - bus off
 *   CAN_TX_FAIL      U0001-00   C0 01 00   High-speed CAN - general failure
 *   TASK_OVERRUN     U3000-49   F0 00 49   Control module - internal failure
 *
 * P0562 and P0563 are the real, standard SAE codes for system voltage. The
 * failure type bytes (0x49 internal electronic failure, 0x88 bus off) come
 * from the SAE J2012-DA list.
 */

#ifndef APP_DIAG_APP_H
#define APP_DIAG_APP_H

#include <stdbool.h>

#include "fault_manager.h"

/**
 * @brief Build the configuration, bind it to the hardware, start the stack.
 *
 * Must run after CanManager_Init() and BEFORE the watchdog is armed: it may
 * erase a flash sector while compacting the non-volatile store.
 *
 * @return true if non-volatile storage is working.
 */
bool DiagApp_Init(void);

/**
 * @brief Receive CAN frames and run the diagnostic stack. Called every 5 ms.
 */
void DiagApp_Task(void);

/**
 * @brief Forward one fault evaluation to the DTC memory, with a freeze frame.
 */
void DiagApp_ReportFault(FaultId_t fault, bool failed);

/**
 * @brief Begin a new DTC operation cycle (called at each ignition-on).
 */
void DiagApp_StartOperationCycle(void);

/**
 * @brief true while the lamp self-test routine (RID 0x0201) is running.
 *        The output task then lights every lamp regardless of vehicle state.
 */
bool DiagApp_IsLampTestActive(void);

#endif /* APP_DIAG_APP_H */
