/**
 * @file    scheduler.c
 * @brief   Time-triggered scheduler implementation - see scheduler.h.
 */

#include "scheduler.h"

#include "ecu_config.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Bookkeeping for one registered task.
 */
typedef struct
{
    const char        *name;
    SchedulerTaskFn_t  function;
    uint32_t           period_ms;
    uint32_t           next_due_ms;        /**< Tick at which this task runs next */
    uint32_t           max_duration_ms;    /**< Worst-case execution time observed */
    uint32_t           activation_count;
} SchedulerTask_t;

static SchedulerTask_t s_tasks[SCHEDULER_MAX_TASKS];
static uint8_t         s_task_count     = 0U;
static uint32_t        s_overrun_count  = 0U;

void Scheduler_Init(void)
{
    s_task_count    = 0U;
    s_overrun_count = 0U;

    for (uint8_t i = 0U; i < SCHEDULER_MAX_TASKS; i++)
    {
        s_tasks[i].name             = "?";
        s_tasks[i].function         = NULL;
        s_tasks[i].period_ms        = 0U;
        s_tasks[i].next_due_ms      = 0U;
        s_tasks[i].max_duration_ms  = 0U;
        s_tasks[i].activation_count = 0U;
    }
}

bool Scheduler_RegisterTask(const char *name,
                            SchedulerTaskFn_t function,
                            uint32_t period_ms)
{
    if ((s_task_count >= SCHEDULER_MAX_TASKS) ||
        (function == NULL) ||
        (period_ms == 0U))
    {
        return false;
    }

    SchedulerTask_t *task = &s_tasks[s_task_count];

    task->name             = (name != NULL) ? name : "?";
    task->function         = function;
    task->period_ms        = period_ms;
    task->max_duration_ms  = 0U;
    task->activation_count = 0U;

    /* Schedule the first activation one full period from now rather than
     * immediately. Otherwise every task would fire simultaneously on the
     * first loop iteration, producing a startup spike that is not
     * representative of steady-state timing. */
    task->next_due_ms = HAL_GetTick() + period_ms;

    s_task_count++;
    return true;
}

void Scheduler_Run(void)
{
    const uint32_t now_ms = HAL_GetTick();

    for (uint8_t i = 0U; i < s_task_count; i++)
    {
        SchedulerTask_t *task = &s_tasks[i];

        /* Signed comparison of the difference is the correct way to test
         * "has this deadline passed" with a wrapping counter. Comparing
         * (now >= next_due) directly would misbehave for ~49 days after
         * every wrap of the 32-bit millisecond tick. */
        if ((int32_t)(now_ms - task->next_due_ms) < 0)
        {
            continue;
        }

        const uint32_t start_ms = HAL_GetTick();

        task->function();

        const uint32_t duration_ms = HAL_GetTick() - start_ms;

        if (duration_ms > task->max_duration_ms)
        {
            task->max_duration_ms = duration_ms;
        }

        if (duration_ms > ECU_TASK_OVERRUN_LIMIT_MS)
        {
            s_overrun_count++;
        }

        task->activation_count++;

        /* Advance by exactly one period from the *scheduled* time, not from
         * the current time. This keeps the long-term average period exact:
         * a late activation does not push all subsequent ones later, so
         * timing errors cannot accumulate. */
        task->next_due_ms += task->period_ms;

        /* If the task fell more than a whole period behind - because an
         * earlier task overran badly - resynchronise instead of trying to
         * catch up with a burst of back-to-back activations. Bursting would
         * make an overload worse exactly when the system can least afford it. */
        if ((int32_t)(now_ms - task->next_due_ms) > 0)
        {
            task->next_due_ms = now_ms + task->period_ms;
        }
    }
}

uint32_t Scheduler_GetOverrunCount(void)
{
    return s_overrun_count;
}

uint32_t Scheduler_GetMaxDurationMs(uint8_t task_index)
{
    if (task_index >= s_task_count)
    {
        return 0U;
    }
    return s_tasks[task_index].max_duration_ms;
}

uint8_t Scheduler_GetTaskCount(void)
{
    return s_task_count;
}

const char *Scheduler_GetTaskName(uint8_t task_index)
{
    if (task_index >= s_task_count)
    {
        return "?";
    }
    return s_tasks[task_index].name;
}
