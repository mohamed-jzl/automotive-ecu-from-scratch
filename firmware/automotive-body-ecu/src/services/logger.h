/**
 * @file    logger.h
 * @brief   Severity-tagged logging over UART.
 *
 * Why logging matters more in embedded than elsewhere
 * ---------------------------------------------------
 * A desktop program that misbehaves can be stopped in a debugger and
 * inspected. An ECU reacting to a brake pedal cannot: halting it destroys the
 * very timing you are trying to observe. Worse, many embedded bugs only
 * appear at full speed, so a breakpoint makes them vanish.
 *
 * A log stream is the alternative - a continuous record of what the firmware
 * believed was happening, produced without stopping it. It is also the only
 * practical way to see inside an ECU during an automated test run, where no
 * human is watching.
 *
 * Output format
 * -------------
 *      [       1234][INFO ] State: OFF -> ACC
 *      [       1567][WARN ] Battery low: 10850 mV
 *      [       1580][ERROR] Fault latched: BATT_UNDERVOLT
 *       ^            ^      ^
 *       |            |      +-- message
 *       |            +--------- severity, fixed width so columns line up
 *       +---------------------- milliseconds since reset
 *
 * The timestamp is what makes the log useful: it turns a list of events into
 * a timeline, and lets you measure the latency between a stimulus and the
 * ECU's reaction directly from the trace.
 *
 * Cost awareness
 * --------------
 * Every log line is transmitted synchronously and blocks the CPU for roughly
 * 87 us per character. A 50-character line costs ~4.3 ms - close to half of
 * the 10 ms control period. Logging inside a fast task will therefore break
 * the timing it was added to investigate. Log state *changes*, not state.
 *
 * Levels below ECU_LOG_LEVEL_COMPILED are removed by the preprocessor, so a
 * release build carries neither the code nor the format strings.
 */

#ifndef SERVICES_LOGGER_H
#define SERVICES_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"

/**
 * @brief Message severity. Values come from ecu_config.h so that the
 *        compile-time filter and this enum can never disagree.
 */
typedef enum
{
    LOG_LEVEL_DEBUG = ECU_LOG_LEVEL_DEBUG,  /**< Detailed tracing, bring-up only   */
    LOG_LEVEL_INFO  = ECU_LOG_LEVEL_INFO,   /**< Normal, noteworthy events         */
    LOG_LEVEL_WARN  = ECU_LOG_LEVEL_WARN,   /**< Abnormal but handled              */
    LOG_LEVEL_ERROR = ECU_LOG_LEVEL_ERROR   /**< Fault condition detected          */
} LogLevel_t;

/**
 * @brief Initialise the logging service and its underlying UART.
 *
 * @return true if the UART came up successfully.
 */
bool Logger_Init(void);

/**
 * @brief Emit one formatted log line. Prefer the LOG_* macros below.
 *
 * Adds the timestamp, the severity tag and a trailing CRLF automatically.
 * Lines longer than ECU_LOG_LINE_MAX are truncated rather than overflowing.
 *
 * @param level   Severity of this message.
 * @param format  printf-style format string.
 * @param ...     Arguments matching @p format.
 */
void Logger_Print(LogLevel_t level, const char *format, ...);

/**
 * @brief Print the startup banner: firmware name, version and reset cause.
 *
 * @param was_watchdog_reset  true if the previous reset came from the watchdog.
 */
void Logger_PrintBanner(bool was_watchdog_reset);

/* -------------------------------------------------------------------------
 * Convenience macros with compile-time filtering.
 *
 * The ##__VA_ARGS__ form is a GNU extension that swallows the preceding comma
 * when no variadic arguments are supplied, so LOG_INFO("text") works as well
 * as LOG_INFO("value=%d", x). The project is built with -std=gnu11, where
 * this is supported. (C23's __VA_OPT__ is the standard equivalent.)
 * ------------------------------------------------------------------------- */

#if (ECU_LOG_LEVEL_COMPILED <= ECU_LOG_LEVEL_DEBUG)
#define LOG_DEBUG(fmt, ...)  Logger_Print(LOG_LEVEL_DEBUG, (fmt), ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)  ((void)0)
#endif

#if (ECU_LOG_LEVEL_COMPILED <= ECU_LOG_LEVEL_INFO)
#define LOG_INFO(fmt, ...)   Logger_Print(LOG_LEVEL_INFO,  (fmt), ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...)   ((void)0)
#endif

#if (ECU_LOG_LEVEL_COMPILED <= ECU_LOG_LEVEL_WARN)
#define LOG_WARN(fmt, ...)   Logger_Print(LOG_LEVEL_WARN,  (fmt), ##__VA_ARGS__)
#else
#define LOG_WARN(fmt, ...)   ((void)0)
#endif

#if (ECU_LOG_LEVEL_COMPILED <= ECU_LOG_LEVEL_ERROR)
#define LOG_ERROR(fmt, ...)  Logger_Print(LOG_LEVEL_ERROR, (fmt), ##__VA_ARGS__)
#else
#define LOG_ERROR(fmt, ...)  ((void)0)
#endif

#endif /* SERVICES_LOGGER_H */
