/**
 * @file    logger.c
 * @brief   Logging implementation - see logger.h for usage and cost notes.
 */

#include "logger.h"

#include "uart_driver.h"
#include "stm32f4xx_hal.h"

#include <stdarg.h>
#include <stdio.h>

static bool s_initialised = false;

/**
 * @brief Fixed-width severity tags.
 *
 * Padded to five characters so the message column lines up in the terminal,
 * which makes a long trace far easier to scan by eye. Indexed by LogLevel_t.
 */
static const char *const LOG_LEVEL_TAG[] =
{
    [LOG_LEVEL_DEBUG] = "DEBUG",
    [LOG_LEVEL_INFO]  = "INFO ",
    [LOG_LEVEL_WARN]  = "WARN ",
    [LOG_LEVEL_ERROR] = "ERROR"
};

bool Logger_Init(void)
{
    s_initialised = UartDriver_Init();
    return s_initialised;
}

void Logger_Print(LogLevel_t level, const char *format, ...)
{
    if ((!s_initialised) || (format == NULL))
    {
        return;
    }

    /* Buffer lives on the stack, so concurrent calls from different contexts
     * cannot interleave into one another's text. It is sized by
     * ECU_LOG_LINE_MAX and accounted for in the stack budget - a 128-byte
     * frame is significant on a device with 128 KB of RAM total, which is
     * why the limit is a named constant rather than an arbitrary number. */
    char line[ECU_LOG_LINE_MAX];

    /* Header: milliseconds since reset, then the severity tag.
     * The 11-character width keeps the column aligned until the tick counter
     * passes 10^11 ms, which is about 3 years of uptime. */
    int written = snprintf(line, sizeof(line), "[%11lu][%s] ",
                           (unsigned long)HAL_GetTick(),
                           LOG_LEVEL_TAG[level]);

    /* snprintf returns the length it *would* have produced. A value at or
     * beyond the buffer size means the header alone filled it, so there is
     * no room for a message and nothing further should be appended. */
    if ((written < 0) || ((size_t)written >= sizeof(line)))
    {
        return;
    }

    va_list args;
    va_start(args, format);

    /* vsnprintf always NUL-terminates and never writes past the limit, so an
     * over-long message is truncated rather than corrupting the stack.
     *
     * Note: with --specs=nano.specs the floating-point formatting specifiers
     * (%f, %g) are stubbed out to save roughly 10 KB of flash. Log integers
     * only - for a voltage, log millivolts rather than volts. */
    const int body = vsnprintf(&line[written], sizeof(line) - (size_t)written,
                               format, args);

    va_end(args);

    if (body < 0)
    {
        return;
    }

    (void)UartDriver_WriteString(line);

    /* CRLF rather than a bare LF: PuTTY, Tera Term and the Windows serial
     * monitors all render CRLF correctly, while some treat a lone LF as a
     * line feed without a carriage return and produce a staircase. */
    (void)UartDriver_WriteString("\r\n");
}

void Logger_PrintBanner(bool was_watchdog_reset)
{
    if (!s_initialised)
    {
        return;
    }

    (void)UartDriver_WriteString("\r\n");
    (void)UartDriver_WriteString(
        "===========================================\r\n");
    (void)UartDriver_WriteString(
        "  Automotive Body Control ECU\r\n");

    Logger_Print(LOG_LEVEL_INFO, "Firmware v%u.%u.%u  target STM32F446RE",
                 ECU_FW_VERSION_MAJOR, ECU_FW_VERSION_MINOR, ECU_FW_VERSION_PATCH);

    Logger_Print(LOG_LEVEL_INFO, "System clock %lu Hz",
                 (unsigned long)ECU_SYSCLK_HZ);

    /* The reset cause is the single most valuable line in the whole log.
     * A power-on reset is routine; a watchdog reset means the firmware hung
     * and recovered, which is a defect that must be investigated even though
     * the ECU appears to be working now. */
    if (was_watchdog_reset)
    {
        Logger_Print(LOG_LEVEL_ERROR,
                     "Reset cause: WATCHDOG - previous run stopped responding");
    }
    else
    {
        Logger_Print(LOG_LEVEL_INFO, "Reset cause: power-on or manual reset");
    }

    (void)UartDriver_WriteString(
        "===========================================\r\n");
}
