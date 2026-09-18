/**
 * @file    uart_driver.h
 * @brief   USART2 transmit driver used by the logging service.
 *
 * On the Nucleo-F446RE, USART2 (PA2/PA3) is wired to the ST-Link's virtual
 * COM port. That means log output travels over the *same* USB cable used to
 * flash and debug the board, and appears on the PC as a normal serial port -
 * no extra USB-to-serial adapter is needed.
 *
 * Frame format: 115200 baud, 8 data bits, no parity, 1 stop bit ("115200 8N1").
 *
 * Transmission is blocking with a timeout. At 115200 baud one character takes
 * 10 bits / 115200 = ~87 us, so a 60-character log line blocks the CPU for
 * roughly 5 ms. That is acceptable for a teaching ECU but is exactly the kind
 * of design a production ECU avoids - see the note in uart_driver.c.
 */

#ifndef DRIVERS_UART_DRIVER_H
#define DRIVERS_UART_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Configure USART2 and its pins for 115200 8N1 transmission.
 *
 * @return true on success, false if the HAL rejected the configuration.
 */
bool UartDriver_Init(void);

/**
 * @brief Send a byte buffer and block until it has left the transmitter.
 *
 * @param  data    Pointer to the bytes to send. Ignored if NULL.
 * @param  length  Number of bytes to send.
 * @return true if the whole buffer was transmitted before the timeout.
 */
bool UartDriver_Write(const uint8_t *data, uint16_t length);

/**
 * @brief Send a NUL-terminated C string.
 *
 * @param  text  String to send. Ignored if NULL.
 * @return true if the whole string was transmitted before the timeout.
 */
bool UartDriver_WriteString(const char *text);

#endif /* DRIVERS_UART_DRIVER_H */
