/**
 * @file    pin_config.h
 * @brief   Central hardware pin map for the Body Control ECU.
 *
 * Every physical pin used by the ECU is named here exactly once.
 * Drivers refer to these macros only - never to raw GPIO_PIN_x values.
 *
 * Consequence: porting the firmware to a different board is a single-file change.
 *
 * Target board: STM32 Nucleo-F446RE (STM32F446RET6, LQFP64)
 */

#ifndef CONFIG_PIN_CONFIG_H
#define CONFIG_PIN_CONFIG_H

#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------------- */
/* Digital inputs                                                            */
/* ------------------------------------------------------------------------- */

/* Ignition switch - onboard blue user button B1.
 * Wired ACTIVE LOW: the button shorts the pin to GND when pressed. */
#define PIN_BTN_IGNITION_PORT       GPIOC
#define PIN_BTN_IGNITION_PIN        GPIO_PIN_13

/* Brake pedal - external push button to GND, internal pull-up enabled. */
#define PIN_BTN_BRAKE_PORT          GPIOC
#define PIN_BTN_BRAKE_PIN           GPIO_PIN_0

/* Turn signal stalk, left and right. */
#define PIN_BTN_TURN_LEFT_PORT      GPIOC
#define PIN_BTN_TURN_LEFT_PIN       GPIO_PIN_1

#define PIN_BTN_TURN_RIGHT_PORT     GPIOC
#define PIN_BTN_TURN_RIGHT_PIN      GPIO_PIN_2

/* ------------------------------------------------------------------------- */
/* Digital outputs                                                           */
/* ------------------------------------------------------------------------- */

/* ECU status / heartbeat - onboard green LED LD2. ACTIVE HIGH. */
#define PIN_LED_STATUS_PORT         GPIOA
#define PIN_LED_STATUS_PIN          GPIO_PIN_5

/* Headlight relay output (LED + series resistor on the bench). */
#define PIN_LED_HEADLIGHT_PORT      GPIOB
#define PIN_LED_HEADLIGHT_PIN       GPIO_PIN_0

/* Turn indicator outputs. */
#define PIN_LED_IND_LEFT_PORT       GPIOB
#define PIN_LED_IND_LEFT_PIN        GPIO_PIN_1

#define PIN_LED_IND_RIGHT_PORT      GPIOB
#define PIN_LED_IND_RIGHT_PIN       GPIO_PIN_2

/* ------------------------------------------------------------------------- */
/* Analog input                                                              */
/* ------------------------------------------------------------------------- */

/* Battery voltage sense, behind a resistor divider. ADC1 channel 0. */
#define PIN_ADC_BATTERY_PORT        GPIOA
#define PIN_ADC_BATTERY_PIN         GPIO_PIN_0
#define PIN_ADC_BATTERY_CHANNEL     ADC_CHANNEL_0

/* ------------------------------------------------------------------------- */
/* PWM output                                                                */
/* ------------------------------------------------------------------------- */

/* Brake light with variable intensity. TIM3 channel 1, alternate function 2. */
#define PIN_PWM_BRAKE_PORT          GPIOB
#define PIN_PWM_BRAKE_PIN           GPIO_PIN_4
#define PIN_PWM_BRAKE_AF            GPIO_AF2_TIM3
#define PIN_PWM_BRAKE_TIMER         TIM3
#define PIN_PWM_BRAKE_CHANNEL       TIM_CHANNEL_1

/* ------------------------------------------------------------------------- */
/* UART - debug console                                                      */
/* ------------------------------------------------------------------------- */

/* USART2 is routed to the ST-Link virtual COM port on the Nucleo board,
 * so logs appear on the PC over the same USB cable used for flashing. */
#define PIN_UART_TX_PORT            GPIOA
#define PIN_UART_TX_PIN             GPIO_PIN_2
#define PIN_UART_RX_PORT            GPIOA
#define PIN_UART_RX_PIN             GPIO_PIN_3
#define PIN_UART_AF                 GPIO_AF7_USART2
#define PIN_UART_INSTANCE           USART2

/* ------------------------------------------------------------------------- */
/* CAN bus                                                                   */
/* ------------------------------------------------------------------------- */

/* CAN1 on PB8/PB9, alternate function 9.
 * These pins go to an external CAN transceiver (MCP2551 / SN65HVD230);
 * the STM32 provides logic-level TX/RX only, never the differential bus. */
#define PIN_CAN_RX_PORT             GPIOB
#define PIN_CAN_RX_PIN              GPIO_PIN_8
#define PIN_CAN_TX_PORT             GPIOB
#define PIN_CAN_TX_PIN              GPIO_PIN_9
#define PIN_CAN_AF                  GPIO_AF9_CAN1
#define PIN_CAN_INSTANCE            CAN1

#endif /* CONFIG_PIN_CONFIG_H */
