/**
 * @file    pwm_driver.c
 * @brief   TIM3 PWM implementation - see pwm_driver.h for the theory.
 */

#include "pwm_driver.h"

#include "ecu_config.h"
#include "pin_config.h"
#include "stm32f4xx_hal.h"

static TIM_HandleTypeDef s_timer;
static bool              s_initialised = false;
static uint16_t          s_duty_ticks  = 0U;

bool PwmDriver_Init(void)
{
    GPIO_InitTypeDef       gpio    = {0};
    TIM_OC_InitTypeDef     channel = {0};
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PB4 in alternate-function mode so the timer's compare output, rather
     * than the GPIO output register, drives the pin. */
    gpio.Pin       = PIN_PWM_BRAKE_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;   /* 1 kHz needs no fast slew rate, and a
                                             * slow edge radiates far less EMI     */
    gpio.Alternate = PIN_PWM_BRAKE_AF;      /* AF2 maps PB4 to TIM3_CH1            */
    HAL_GPIO_Init(PIN_PWM_BRAKE_PORT, &gpio);

    /* Timing derivation (full explanation in ecu_config.h):
     *
     *   TIM3 clock       = 84 MHz  (APB1 timer domain)
     *   Prescaler  = 83  -> counter clock = 84 MHz / 84  = 1 MHz
     *   Period     = 999 -> PWM frequency = 1 MHz / 1000 = 1 kHz
     *
     * Both registers are "value minus one" because the hardware counts from
     * zero: a period register of 999 produces 1000 counter states.
     */
    s_timer.Instance               = PIN_PWM_BRAKE_TIMER;
    s_timer.Init.Prescaler         = ECU_PWM_PRESCALER;
    s_timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_timer.Init.Period            = ECU_PWM_PERIOD_TICKS;
    s_timer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;

    /* Preload makes a write to the auto-reload register take effect only at
     * the next update event, so changing the period can never produce a
     * truncated or stretched pulse. */
    s_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&s_timer) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    /* This timer drives nothing else, so it needs no master/slave trigger
     * output. Configured explicitly rather than left at reset defaults. */
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    (void)HAL_TIMEx_MasterConfigSynchronization(&s_timer, &master);

    /* PWM mode 1: output is ACTIVE while counter < CCR, inactive afterwards.
     * Combined with active-high polarity, a larger CCR means a brighter lamp. */
    channel.OCMode     = TIM_OCMODE_PWM1;
    channel.Pulse      = 0U;                    /* start dark - never flash a lamp
                                                 * during initialisation          */
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&s_timer, &channel, PIN_PWM_BRAKE_CHANNEL) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    if (HAL_TIM_PWM_Start(&s_timer, PIN_PWM_BRAKE_CHANNEL) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    s_duty_ticks  = 0U;
    s_initialised = true;
    return true;
}

void PwmDriver_SetBrakeDuty(uint16_t duty_ticks)
{
    if (!s_initialised)
    {
        return;
    }

    if (duty_ticks > ECU_PWM_DUTY_MAX)
    {
        duty_ticks = ECU_PWM_DUTY_MAX;
    }

    s_duty_ticks = duty_ticks;

    /* Writing the compare register is a single 32-bit store, so it is
     * inherently atomic on Cortex-M4 - no critical section is required even
     * though the timer hardware reads this register continuously. The compare
     * register is also preloaded, so the new value is latched at the next
     * update event and the current PWM cycle finishes cleanly. */
    __HAL_TIM_SET_COMPARE(&s_timer, PIN_PWM_BRAKE_CHANNEL, duty_ticks);
}

uint16_t PwmDriver_GetBrakeDuty(void)
{
    return s_duty_ticks;
}
