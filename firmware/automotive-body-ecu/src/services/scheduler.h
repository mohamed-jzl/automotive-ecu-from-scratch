/**
 * @file    scheduler.h
 * @brief   Cooperative time-triggered scheduler - the heartbeat of the ECU.
 *
 * The problem with a plain main loop
 * ----------------------------------
 * A naive bare-metal loop looks like this:
 *
 *      while (1) { read_inputs(); control(); send_can(); HAL_Delay(10); }
 *
 * It works, and then it stops working. Everything runs at one rate even
 * though inputs want 10 ms and CAN wants 100 ms. The delay is *added to*
 * execution time, so the real period drifts with the workload. And nothing
 * measures whether the work actually fits in the time available.
 *
 * The time-triggered alternative
 * ------------------------------
 * Register each task with its own period. The scheduler checks the elapsed
 * time and runs whatever is due:
 *
 *      t=0ms    [10ms task]
 *      t=10ms   [10ms task]
 *      ...
 *      t=50ms   [10ms task][50ms task]
 *      t=100ms  [10ms task][50ms task][100ms task]
 *
 * This is the classic pattern from Pont's "Patterns for Time-Triggered
 * Embedded Systems" and it is what a large amount of real automotive
 * firmware runs on, below the level where an RTOS is justified.
 *
 * Why cooperative rather than pre-emptive
 * ---------------------------------------
 * A task runs to completion; nothing interrupts it. There are therefore no
 * race conditions between tasks, no mutexes, and no per-task stacks - one
 * stack serves everything. The cost is that a long task delays every other
 * task, so each one must be short and must never block. In exchange the
 * timing is simple enough to analyse by hand, which is exactly the trade an
 * RTOS gives up.
 *
 * Execution-time monitoring
 * -------------------------
 * The scheduler times every activation and records the worst case seen. A
 * task exceeding ECU_TASK_OVERRUN_LIMIT_MS is counted as an overrun and
 * escalated to a diagnostic fault. Measuring worst-case execution time is a
 * requirement in automotive timing analysis, not an optional nicety - it is
 * the evidence that the schedule actually closes.
 */

#ifndef SERVICES_SCHEDULER_H
#define SERVICES_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

/** Maximum number of tasks the scheduler can hold. */
#define SCHEDULER_MAX_TASKS   8U

/**
 * @brief Signature of a scheduled task.
 *
 * A task takes no arguments and returns nothing: it reads shared state,
 * acts, and returns promptly. It must never block, never spin, and never
 * call any delay function.
 */
typedef void (*SchedulerTaskFn_t)(void);

/**
 * @brief Reset the scheduler and discard all registered tasks.
 */
void Scheduler_Init(void);

/**
 * @brief Register a periodic task.
 *
 * @param  name       Short identifier used in diagnostic output. Must point to
 *                    storage that outlives the scheduler, e.g. a string literal.
 * @param  function   Task entry point. Must not be NULL.
 * @param  period_ms  Activation period in milliseconds. Must be non-zero.
 * @return true if registered, false if the table is full or an argument is invalid.
 */
bool Scheduler_RegisterTask(const char *name,
                            SchedulerTaskFn_t function,
                            uint32_t period_ms);

/**
 * @brief Run every task whose period has elapsed.
 *
 * Call this continuously from the main loop. Returns immediately when nothing
 * is due, so the loop spins freely between activations.
 *
 * Tasks are evaluated in registration order, which makes the execution
 * sequence deterministic when several fall due in the same millisecond.
 * Register in data-flow order - sample inputs before the logic that consumes
 * them - so that every cycle uses fresh data rather than last cycle's.
 */
void Scheduler_Run(void);

/**
 * @brief Total number of task activations that exceeded the time budget.
 *
 * @return Cumulative overrun count across all tasks.
 */
uint32_t Scheduler_GetOverrunCount(void);

/**
 * @brief Worst-case execution time observed for a task.
 *
 * @param  task_index  Registration index, 0-based.
 * @return Longest activation seen, in milliseconds, or 0 if the index is invalid.
 */
uint32_t Scheduler_GetMaxDurationMs(uint8_t task_index);

/**
 * @brief Number of tasks currently registered.
 */
uint8_t Scheduler_GetTaskCount(void);

/**
 * @brief Name of a registered task.
 *
 * @param  task_index  Registration index, 0-based.
 * @return The task's name, or "?" if the index is invalid.
 */
const char *Scheduler_GetTaskName(uint8_t task_index);

#endif /* SERVICES_SCHEDULER_H */
