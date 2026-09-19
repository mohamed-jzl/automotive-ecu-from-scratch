/**
 * @file    dtc_manager.c
 * @brief   DTC status handling - see dtc_manager.h for the status bit meanings.
 */

#include "dtc_manager.h"

#include <string.h>

static const DtcDefinition_t *s_definitions   = NULL;
static uint8_t                s_count         = 0U;
static DtcRecord_t            s_records[ECU_DTC_MAX_COUNT];
static bool                   s_updates_enabled = true;
static bool                   s_dirty         = false;

static void record_reset(DtcRecord_t *record)
{
    (void)memset(record, 0, sizeof(*record));
    record->status = DTC_STATUS_INITIAL;
}

void DtcManager_Init(const DtcDefinition_t *definitions, uint8_t count)
{
    s_definitions     = definitions;
    s_count           = (count > ECU_DTC_MAX_COUNT) ? (uint8_t)ECU_DTC_MAX_COUNT : count;
    s_updates_enabled = true;
    s_dirty           = false;

    for (uint8_t i = 0U; i < (uint8_t)ECU_DTC_MAX_COUNT; i++)
    {
        record_reset(&s_records[i]);
    }
}

void DtcManager_ReportResult(uint8_t index, bool failed,
                             const DtcSnapshot_t *environment)
{
    if ((index >= s_count) || (!s_updates_enabled))
    {
        return;
    }

    DtcRecord_t  *record = &s_records[index];
    const uint8_t before = record->status;

    if (failed)
    {
        /* A new failure event, as opposed to the same failure still present. */
        if ((record->status & DTC_STATUS_TEST_FAILED) == 0U)
        {
            if (record->occurrence_counter < UINT8_MAX)
            {
                record->occurrence_counter++;
            }
        }

        record->status |= (uint8_t)(DTC_STATUS_TEST_FAILED |
                                    DTC_STATUS_TEST_FAILED_THIS_CYCLE |
                                    DTC_STATUS_PENDING |
                                    DTC_STATUS_CONFIRMED |
                                    DTC_STATUS_FAILED_SINCE_CLEAR);

        /* The test has now run, so it is no longer "not completed". */
        record->status &= (uint8_t)~(DTC_STATUS_NOT_COMPLETED_SINCE_CLEAR |
                                     DTC_STATUS_NOT_COMPLETED_THIS_CYCLE);

        record->aging_counter = 0U;     /* failing again restarts aging */

        if (s_definitions[index].warning_lamp)
        {
            record->status |= DTC_STATUS_WARNING_INDICATOR;
        }

        /* Freeze frame: captured once, at the FIRST confirmation. Overwriting
         * it on every report would replace the conditions that caused the
         * fault with the conditions of whenever it was last reported. */
        if ((!record->snapshot_valid) && (environment != NULL))
        {
            record->snapshot       = *environment;
            record->snapshot_valid = true;
        }
    }
    else
    {
        record->status &= (uint8_t)~(DTC_STATUS_TEST_FAILED |
                                     DTC_STATUS_NOT_COMPLETED_SINCE_CLEAR |
                                     DTC_STATUS_NOT_COMPLETED_THIS_CYCLE);

        /* Simplification: the warning lamp follows the live fault. Production
         * strategies often keep it lit for several clean cycles. */
        if (s_definitions[index].warning_lamp)
        {
            record->status &= (uint8_t)~DTC_STATUS_WARNING_INDICATOR;
        }
    }

    if (record->status != before)
    {
        s_dirty = true;
    }
}

void DtcManager_StartOperationCycle(void)
{
    for (uint8_t i = 0U; i < s_count; i++)
    {
        DtcRecord_t  *record = &s_records[i];
        const uint8_t status = record->status;

        const bool failed_last_cycle = (status & DTC_STATUS_TEST_FAILED_THIS_CYCLE)  != 0U;
        const bool tested_last_cycle = (status & DTC_STATUS_NOT_COMPLETED_THIS_CYCLE) == 0U;

        if (!failed_last_cycle)
        {
            /* A whole cycle tested and passed: the fault is no longer pending. */
            if (tested_last_cycle)
            {
                record->status &= (uint8_t)~DTC_STATUS_PENDING;
            }

            /* Aging: one more clean cycle for a stored DTC. */
            if ((status & DTC_STATUS_CONFIRMED) != 0U)
            {
                record->aging_counter++;

                if (record->aging_counter >= ECU_DTC_AGING_THRESHOLD)
                {
                    /* Healed for long enough: forget it. The "since last clear"
                     * history is also dropped, as if a technician cleared it. */
                    record->status &= (uint8_t)~(DTC_STATUS_CONFIRMED |
                                                 DTC_STATUS_PENDING |
                                                 DTC_STATUS_FAILED_SINCE_CLEAR |
                                                 DTC_STATUS_WARNING_INDICATOR);
                    record->aging_counter      = 0U;
                    record->occurrence_counter = 0U;
                    record->snapshot_valid     = false;
                }
            }
        }

        /* New cycle: nothing has failed or been tested in it yet. */
        record->status &= (uint8_t)~DTC_STATUS_TEST_FAILED_THIS_CYCLE;
        record->status |= DTC_STATUS_NOT_COMPLETED_THIS_CYCLE;
    }

    s_dirty = true;     /* aging counters changed even if status bits did not */
}

void DtcManager_SetUpdatesEnabled(bool enabled)
{
    s_updates_enabled = enabled;
}

bool DtcManager_AreUpdatesEnabled(void)
{
    return s_updates_enabled;
}

bool DtcManager_Clear(uint32_t group)
{
    if (group == 0xFFFFFFUL)
    {
        for (uint8_t i = 0U; i < s_count; i++)
        {
            record_reset(&s_records[i]);
        }
        s_dirty = true;
        return true;
    }

    const int16_t index = DtcManager_FindByCode(group);

    if (index < 0)
    {
        return false;
    }

    record_reset(&s_records[index]);
    s_dirty = true;
    return true;
}

uint8_t DtcManager_GetCount(void)
{
    return s_count;
}

uint32_t DtcManager_GetCode(uint8_t index)
{
    return (index < s_count) ? (s_definitions[index].code & 0xFFFFFFUL) : 0U;
}

uint8_t DtcManager_GetStatus(uint8_t index)
{
    return (index < s_count) ? s_records[index].status : 0U;
}

const DtcRecord_t *DtcManager_GetRecord(uint8_t index)
{
    return (index < s_count) ? &s_records[index] : NULL;
}

int16_t DtcManager_FindByCode(uint32_t code)
{
    for (uint8_t i = 0U; i < s_count; i++)
    {
        if ((s_definitions[i].code & 0xFFFFFFUL) == (code & 0xFFFFFFUL))
        {
            return (int16_t)i;
        }
    }
    return -1;
}

uint8_t DtcManager_CountByMask(uint8_t mask)
{
    uint8_t matches = 0U;

    for (uint8_t i = 0U; i < s_count; i++)
    {
        if ((s_records[i].status & mask & DTC_STATUS_AVAILABILITY_MASK) != 0U)
        {
            matches++;
        }
    }
    return matches;
}

bool DtcManager_IsDirty(void)
{
    return s_dirty;
}

void DtcManager_ClearDirty(void)
{
    s_dirty = false;
}

uint16_t DtcManager_Serialise(uint8_t *out, uint16_t size)
{
    const uint16_t needed = (uint16_t)(1U + (s_count * DTC_RECORD_SERIALISED_SIZE));

    if ((out == NULL) || (size < needed))
    {
        return 0U;
    }

    out[0] = s_count;   /* lets Deserialise detect a table size mismatch */

    for (uint8_t i = 0U; i < s_count; i++)
    {
        const DtcRecord_t *r = &s_records[i];
        uint8_t           *p = &out[1U + (i * DTC_RECORD_SERIALISED_SIZE)];

        p[0] = r->status;
        p[1] = r->occurrence_counter;
        p[2] = r->aging_counter;
        p[3] = r->snapshot_valid ? 1U : 0U;
        p[4] = (uint8_t)(r->snapshot.battery_mv >> 8);
        p[5] = (uint8_t)(r->snapshot.battery_mv & 0xFFU);
        p[6] = r->snapshot.vehicle_state;
        p[7] = 0U;                      /* reserved for a future field */
    }

    return needed;
}

bool DtcManager_Deserialise(const uint8_t *in, uint16_t length)
{
    if ((in == NULL) || (length < 1U) || (in[0] != s_count) ||
        (length < (uint16_t)(1U + (s_count * DTC_RECORD_SERIALISED_SIZE))))
    {
        return false;
    }

    for (uint8_t i = 0U; i < s_count; i++)
    {
        const uint8_t *p = &in[1U + (i * DTC_RECORD_SERIALISED_SIZE)];
        DtcRecord_t   *r = &s_records[i];

        r->status                  = p[0];
        r->occurrence_counter      = p[1];
        r->aging_counter           = p[2];
        r->snapshot_valid          = (p[3] != 0U);
        r->snapshot.battery_mv     = (uint16_t)(((uint16_t)p[4] << 8) | p[5]);
        r->snapshot.vehicle_state  = p[6];
    }

    s_dirty = false;
    return true;
}
