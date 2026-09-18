/**
 * @file    adc_driver.h
 * @brief   Battery voltage acquisition through ADC1 channel 0 (PA0).
 *
 * The analog-to-digital converter is, electrically, a programmable voltmeter.
 * It compares the input against a reference (VREF+ = 3.3 V here) and reports
 * the result as a 12-bit integer between 0 and 4095:
 *
 *      raw = round( V_pin / VREF * 4095 )
 *
 * One least-significant bit therefore represents 3300 mV / 4095 = 0.806 mV.
 * That quantisation step is the *floor* on measurement resolution - no amount
 * of software can recover detail finer than one LSB.
 *
 * A 12 V battery exceeds the 3.3 V input range, so an external resistor
 * divider scales it down first (see ECU_ADC_DIVIDER_* in ecu_config.h). The
 * driver reverses that scaling in software to report true battery millivolts.
 *
 * This header deliberately contains no STM32 HAL types. The unit conversion
 * is exposed as a pure inline function so it can be tested on a host PC with
 * plain gcc, with no microcontroller and no hardware involved.
 */

#ifndef DRIVERS_ADC_DRIVER_H
#define DRIVERS_ADC_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"

/**
 * @brief Convert a raw ADC count into battery millivolts.
 *
 * Pure function: no hardware access, no global state, fully deterministic.
 * That is what makes it unit-testable - see tests/test_adc_conversion.c.
 *
 * The calculation is performed in two steps rather than one expression:
 *
 *     v_pin = raw * VREF / 4095                 (pin voltage)
 *     v_bat = v_pin * (R1 + R2) / R2            (undo the divider)
 *
 * Folding both steps into a single fraction would be marginally more accurate
 * but overflows 32 bits (4095 * 3300 * 12200 is about 1.6e11). Splitting the
 * calculation costs at most 1 mV of rounding, which is far below the ~0.8 mV
 * quantisation step multiplied by the divider ratio, so it is not the
 * dominant error term.
 *
 * @param  raw_count  ADC reading, 0 .. ECU_ADC_MAX_COUNT. Values above the
 *                    maximum are clamped rather than allowed to overflow.
 * @return Battery voltage in millivolts.
 */
static inline uint32_t AdcDriver_RawToBatteryMv(uint16_t raw_count)
{
    uint32_t raw = (uint32_t)raw_count;

    if (raw > ECU_ADC_MAX_COUNT)
    {
        raw = ECU_ADC_MAX_COUNT;
    }

    /* Step 1: voltage present at the MCU pin. */
    const uint32_t pin_mv = (raw * ECU_ADC_VREF_MV) / ECU_ADC_MAX_COUNT;

    /* Step 2: undo the external divider to recover the battery voltage. */
    const uint32_t divider_sum = ECU_ADC_DIVIDER_R1_OHM + ECU_ADC_DIVIDER_R2_OHM;

    return (pin_mv * divider_sum) / ECU_ADC_DIVIDER_R2_OHM;
}

/**
 * @brief Configure ADC1, its clock, and the PA0 analog input pin.
 *
 * @return true on success, false if the HAL rejected the configuration.
 */
bool AdcDriver_Init(void);

/**
 * @brief Take an averaged battery voltage measurement.
 *
 * Performs ECU_ADC_OVERSAMPLE_COUNT conversions and averages them. Averaging
 * N independent samples reduces uncorrelated noise by a factor of sqrt(N);
 * with N = 8 that is an improvement of about 2.8x, worth roughly 1.5 extra
 * bits of effective resolution.
 *
 * Note that averaging only helps against *random* noise. A systematic error -
 * a resistor that is 1% off its nominal value, or a reference voltage that
 * drifts with temperature - is unaffected and must be handled by calibration.
 *
 * @param  out_millivolts  Receives the battery voltage in mV. Untouched on failure.
 * @return true if every conversion completed successfully.
 */
bool AdcDriver_ReadBatteryMv(uint32_t *out_millivolts);

/**
 * @brief Return the most recent raw ADC count, for diagnostics and logging.
 *
 * @return Last raw value read, or 0 if no conversion has completed yet.
 */
uint16_t AdcDriver_GetLastRawCount(void);

#endif /* DRIVERS_ADC_DRIVER_H */
