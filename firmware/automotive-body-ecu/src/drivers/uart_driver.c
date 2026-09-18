/**
 * @file    uart_driver.c
 * @brief   USART2 transmit implementation.
 *
 * Why blocking transmission, and when that becomes wrong
 * -----------------------------------------------------
 * HAL_UART_Transmit() spins until the last bit has been shifted out. That is
 * simple, deterministic, and easy to reason about, which is why it is the
 * right starting point.
 *
 * It is also a latency hazard: a 60-character line at 115200 baud stalls the
 * CPU for about 5 ms, and our control task must run every 10 ms. Logging too
 * much therefore *causes* the deadline misses it was meant to help diagnose.
 *
 * Production ECUs solve this with a ring buffer plus DMA or interrupt-driven
 * transmission, so the CPU only copies bytes into the buffer and returns
 * immediately. That is a deliberate next exercise for this project rather
 * than a hidden shortcut - the limitation is documented in
 * docs/lessons_learned.md and in the test plan.
 */

#include "uart_driver.h"

#include "ecu_config.h"
#include "pin_config.h"
#include "stm32f4xx_hal.h"

#include <string.h>

/* HAL handle for USART2. Static: no other module may reach the peripheral
 * except through this driver's API. */
static UART_HandleTypeDef s_uart;

/* Set once initialisation succeeds. Every write is rejected before that, so a
 * logging call made too early fails silently instead of faulting. */
static bool s_initialised = false;

/* Worst-case time allowed for one transmit call to complete. Generous enough
 * that it is never hit in normal operation; if it is, the peripheral is
 * genuinely stuck and blocking forever would hang the ECU. */
#define UART_TX_TIMEOUT_MS   100U

bool UartDriver_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Both the GPIO bank and the USART peripheral need their bus clocks
     * enabled before any of their registers respond to writes. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = TX, PA3 = RX, both in alternate-function mode so the USART
     * peripheral - not the GPIO output register - drives the pin. */
    gpio.Pin       = PIN_UART_TX_PIN | PIN_UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;               /* idle line is high; a pull-up keeps
                                                 * it there if the cable is unplugged */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH; /* fast edges keep the eye diagram
                                                 * clean at high baud rates           */
    gpio.Alternate = PIN_UART_AF;               /* AF7 routes these pins to USART2     */
    HAL_GPIO_Init(PIN_UART_TX_PORT, &gpio);

    s_uart.Instance          = PIN_UART_INSTANCE;
    s_uart.Init.BaudRate     = ECU_UART_BAUDRATE;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;  /* no RTS/CTS on the ST-Link VCP */
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16; /* 16x oversampling gives the
                                                      * best tolerance to baud-rate
                                                      * mismatch between the two ends */

    if (HAL_UART_Init(&s_uart) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    s_initialised = true;
    return true;
}

bool UartDriver_Write(const uint8_t *data, uint16_t length)
{
    if ((!s_initialised) || (data == NULL) || (length == 0U))
    {
        return false;
    }

    /* The HAL takes a non-const pointer even though it only reads the buffer.
     * The cast is safe and is confined to this single line so the rest of the
     * codebase can keep its const-correctness. */
    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(&s_uart, (uint8_t *)data, length, UART_TX_TIMEOUT_MS);

    return (status == HAL_OK);
}

bool UartDriver_WriteString(const char *text)
{
    if (text == NULL)
    {
        return false;
    }

    const size_t length = strlen(text);

    /* HAL_UART_Transmit counts bytes in a uint16_t. Refuse anything longer
     * rather than silently truncating via an implicit narrowing conversion. */
    if (length > UINT16_MAX)
    {
        return false;
    }

    return UartDriver_Write((const uint8_t *)text, (uint16_t)length);
}
