/**
 * @file    test_adc_conversion.c
 * @brief   Unit tests for the raw-ADC-count to battery-millivolt conversion.
 *
 * The conversion is a pure inline function in adc_driver.h, deliberately
 * separated from the hardware access in adc_driver.c. That separation is
 * what makes it testable: the arithmetic can be verified exhaustively on a
 * PC, while the part that genuinely needs silicon is tested on the bench.
 *
 * Splitting "measure the thing" from "interpret the measurement" is a
 * generally useful habit. The interpretation is where the bugs that matter
 * live - wrong divider ratio, integer overflow, truncation in the wrong
 * order - and it is the half that can be proven correct without hardware.
 *
 * Expected values were computed independently in Python from the resistor
 * values and reference voltage in ecu_config.h, not by running this code.
 */

#include "unity_min.h"

#include "adc_driver.h"
#include "ecu_config.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* ========================================================================= */
/* Endpoints                                                                 */
/* ========================================================================= */

static void test_zero_count_is_zero_volts(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, AdcDriver_RawToBatteryMv(0U));
}

static void test_full_scale_is_the_divider_maximum(void)
{
    /* 4095 counts = 3.300 V at the pin.
     * Reversing the 10k/2.2k divider: 3300 * 12200 / 2200 = 18300 mV.
     *
     * This is the design headroom: the sense circuit covers the entire
     * automotive range, from a 9 V cranking dip to a 15 V charging voltage,
     * without ever clipping. */
    TEST_ASSERT_EQUAL_UINT32(18300U, AdcDriver_RawToBatteryMv(ECU_ADC_MAX_COUNT));
}

static void test_known_conversion_points(void)
{
    /* Values computed independently from the specification. */
    TEST_ASSERT_EQUAL_UINT32(4464U,  AdcDriver_RawToBatteryMv(1000U));
    TEST_ASSERT_EQUAL_UINT32(9150U,  AdcDriver_RawToBatteryMv(2048U));
    TEST_ASSERT_EQUAL_UINT32(13403U, AdcDriver_RawToBatteryMv(3000U));
}

/* ========================================================================= */
/* Robustness                                                                */
/* ========================================================================= */

static void test_out_of_range_count_is_clamped_not_wrapped(void)
{
    /* A 12-bit ADC cannot produce these, but a corrupted buffer or a future
     * change to a 16-bit converter could. Clamping keeps the result
     * physically plausible; allowing it through would produce a voltage
     * reading above the supply rail, which downstream logic would treat as
     * a genuine overvoltage fault. */
    const uint32_t at_max = AdcDriver_RawToBatteryMv(ECU_ADC_MAX_COUNT);

    TEST_ASSERT_EQUAL_UINT32(at_max, AdcDriver_RawToBatteryMv(5000U));
    TEST_ASSERT_EQUAL_UINT32(at_max, AdcDriver_RawToBatteryMv(65535U));
}

static void test_conversion_is_monotonic(void)
{
    /* Sweeps the entire input domain - all 4096 possible readings - and
     * requires that the result never decreases as the count rises.
     *
     * Monotonicity is the property that catches integer-division mistakes:
     * reorder the multiply and divide and the function still looks roughly
     * right at the endpoints while producing small backward steps in the
     * middle. Exhaustive sweeps like this are only affordable because the
     * function is pure. */
    uint32_t previous = 0U;

    for (uint32_t raw = 0U; raw <= ECU_ADC_MAX_COUNT; raw++)
    {
        const uint32_t current = AdcDriver_RawToBatteryMv((uint16_t)raw);

        TEST_ASSERT_TRUE(current >= previous);
        previous = current;
    }
}

static void test_no_overflow_across_the_whole_range(void)
{
    /* The intermediate product raw * VREF reaches 13.5 million and the
     * second step reaches 40 million - both well inside uint32_t. A single
     * combined expression would reach 1.6e11 and silently wrap. This test
     * would catch that regression by seeing an impossibly large result. */
    for (uint32_t raw = 0U; raw <= ECU_ADC_MAX_COUNT; raw++)
    {
        TEST_ASSERT_TRUE(AdcDriver_RawToBatteryMv((uint16_t)raw) <= 18300U);
    }
}

/* ========================================================================= */
/* Threshold relevance                                                       */
/* ========================================================================= */

static void test_fault_thresholds_are_inside_the_measurable_range(void)
{
    /* A threshold the hardware cannot reach is a fault that can never fire.
     * Checking the configuration against the sense circuit catches that at
     * build time rather than during a vehicle test. */
    const uint32_t full_scale = AdcDriver_RawToBatteryMv(ECU_ADC_MAX_COUNT);

    TEST_ASSERT_TRUE(ECU_BATT_UNDERVOLTAGE_MV < full_scale);
    TEST_ASSERT_TRUE(ECU_BATT_OVERVOLTAGE_MV  < full_scale);
    TEST_ASSERT_TRUE(ECU_BATT_NOMINAL_MV      < full_scale);

    /* And the thresholds must be ordered sensibly relative to each other. */
    TEST_ASSERT_TRUE(ECU_BATT_UNDERVOLTAGE_MV < ECU_BATT_NOMINAL_MV);
    TEST_ASSERT_TRUE(ECU_BATT_NOMINAL_MV      < ECU_BATT_OVERVOLTAGE_MV);
}

static void test_hysteresis_does_not_overlap_the_thresholds(void)
{
    /* If the hysteresis band were wider than the gap between the under- and
     * overvoltage thresholds, the two faults could be active simultaneously
     * for the same voltage. The configuration must make that impossible. */
    TEST_ASSERT_TRUE((ECU_BATT_UNDERVOLTAGE_MV + ECU_BATT_HYSTERESIS_MV) <
                     (ECU_BATT_OVERVOLTAGE_MV  - ECU_BATT_HYSTERESIS_MV));
}

/* ========================================================================= */

int main(void)
{
    UnityBegin("ADC Conversion");

    RUN_TEST(test_zero_count_is_zero_volts);
    RUN_TEST(test_full_scale_is_the_divider_maximum);
    RUN_TEST(test_known_conversion_points);

    RUN_TEST(test_out_of_range_count_is_clamped_not_wrapped);
    RUN_TEST(test_conversion_is_monotonic);
    RUN_TEST(test_no_overflow_across_the_whole_range);

    RUN_TEST(test_fault_thresholds_are_inside_the_measurable_range);
    RUN_TEST(test_hysteresis_does_not_overlap_the_thresholds);

    return UnityEnd();
}
