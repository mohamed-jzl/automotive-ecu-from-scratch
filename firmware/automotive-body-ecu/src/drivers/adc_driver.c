/**
 * @file    adc_driver.c
 * @brief   ADC1 acquisition implementation - see adc_driver.h for the theory.
 */

#include "adc_driver.h"

#include "pin_config.h"
#include "stm32f4xx_hal.h"

static ADC_HandleTypeDef s_adc;
static bool              s_initialised     = false;
static uint16_t          s_last_raw_count  = 0U;

/* How long a single conversion is allowed to take. A 12-bit conversion with
 * 84 cycles of sampling time at 21 MHz needs about 5.4 us, so 10 ms is four
 * orders of magnitude of margin: if this timeout ever fires, the peripheral
 * is broken, not slow. */
#define ADC_CONVERSION_TIMEOUT_MS   10U

bool AdcDriver_Init(void)
{
    GPIO_InitTypeDef      gpio    = {0};
    ADC_ChannelConfTypeDef channel = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* Analog mode disconnects the pin's digital input buffer entirely.
     * Leaving that buffer connected to a slowly-varying analog voltage would
     * let it sit in its linear region and draw significant supply current. */
    gpio.Pin  = PIN_ADC_BATTERY_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;   /* any pull resistor would load the divider and
                                * corrupt the measurement */
    HAL_GPIO_Init(PIN_ADC_BATTERY_PORT, &gpio);

    s_adc.Instance = ADC1;

    /* The STM32F446 ADC is specified for a maximum clock of 36 MHz. APB2 runs
     * at 84 MHz, so the smallest legal divider is 4, giving 21 MHz. Exceeding
     * the specified maximum does not fail loudly - it quietly degrades
     * accuracy, which is far harder to diagnose. */
    s_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;

    s_adc.Init.Resolution = ADC_RESOLUTION_12B;
    s_adc.Init.DataAlign  = ADC_DATAALIGN_RIGHT;  /* value in bits 11..0, so the
                                                   * result needs no shifting   */

    /* Single channel, software triggered, one conversion per request.
     * Scan and continuous modes exist for multi-channel or free-running
     * acquisition; neither is needed to read one battery voltage. */
    s_adc.Init.ScanConvMode          = DISABLE;
    s_adc.Init.ContinuousConvMode    = DISABLE;
    s_adc.Init.DiscontinuousConvMode = DISABLE;
    s_adc.Init.NbrOfConversion       = 1U;
    s_adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    s_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    s_adc.Init.DMAContinuousRequests = DISABLE;

    if (HAL_ADC_Init(&s_adc) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    channel.Channel = PIN_ADC_BATTERY_CHANNEL;
    channel.Rank    = 1U;

    /* Sampling time must let the internal sample-and-hold capacitor charge
     * through the source impedance. Our divider presents 10k || 2.2k = 1.8k.
     * 84 ADC cycles at 21 MHz is 4 us, which is many RC time constants for a
     * 1.8k source - comfortably sufficient. Too short a sampling time is a
     * classic cause of readings that are consistently low. */
    channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&s_adc, &channel) != HAL_OK)
    {
        s_initialised = false;
        return false;
    }

    s_initialised = true;
    return true;
}

/**
 * @brief Run one conversion and return the raw count.
 *
 * @param  out_raw  Receives the 12-bit result.
 * @return true if the conversion completed within the timeout.
 */
static bool adc_convert_once(uint16_t *out_raw)
{
    if (HAL_ADC_Start(&s_adc) != HAL_OK)
    {
        return false;
    }

    /* Polling keeps the data flow obvious. An interrupt or DMA-driven design
     * would free the CPU during the 5 us conversion, which matters when
     * sampling many channels at kHz rates but not for one reading per 500 ms. */
    if (HAL_ADC_PollForConversion(&s_adc, ADC_CONVERSION_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&s_adc);
        return false;
    }

    *out_raw = (uint16_t)HAL_ADC_GetValue(&s_adc);

    (void)HAL_ADC_Stop(&s_adc);
    return true;
}

bool AdcDriver_ReadBatteryMv(uint32_t *out_millivolts)
{
    if ((!s_initialised) || (out_millivolts == NULL))
    {
        return false;
    }

    uint32_t accumulator = 0U;

    for (uint8_t i = 0U; i < ECU_ADC_OVERSAMPLE_COUNT; i++)
    {
        uint16_t raw = 0U;

        if (!adc_convert_once(&raw))
        {
            /* Report the failure rather than returning a half-averaged value.
             * The caller escalates this to a diagnostic fault. */
            return false;
        }

        accumulator += raw;
    }

    /* Maximum accumulator value is 8 * 4095 = 32760, so no overflow risk. */
    const uint16_t averaged_raw = (uint16_t)(accumulator / ECU_ADC_OVERSAMPLE_COUNT);

    s_last_raw_count = averaged_raw;
    *out_millivolts  = AdcDriver_RawToBatteryMv(averaged_raw);

    return true;
}

uint16_t AdcDriver_GetLastRawCount(void)
{
    return s_last_raw_count;
}
