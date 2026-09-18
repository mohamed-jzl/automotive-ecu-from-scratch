/**
 * @file    gpio_driver.c
 * @brief   Digital I/O implementation - see gpio_driver.h for the contract.
 *
 * Design note: both inputs and outputs are described by constant tables in
 * flash rather than by a switch statement. Adding a new lamp is one table row,
 * not a new branch in three different functions. This is the standard way
 * production drivers stay maintainable as the pin count grows.
 */

#include "gpio_driver.h"

#include "ecu_config.h"
#include "pin_config.h"
#include "stm32f4xx_hal.h"

/* ========================================================================= */
/* Descriptor tables                                                         */
/* ========================================================================= */

/**
 * @brief Static description of one physical input pin.
 */
typedef struct
{
    GPIO_TypeDef *port;         /**< GPIO bank (GPIOA / GPIOB / GPIOC)      */
    uint16_t      pin;          /**< Pin mask within that bank              */
    bool          active_low;   /**< true if electrical LOW means "active"  */
    uint32_t      pull;         /**< Internal pull resistor configuration   */
} GpioInputDescriptor_t;

/**
 * @brief Static description of one physical output pin.
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    bool          active_low;
} GpioOutputDescriptor_t;

/* Order MUST match the DigitalInput_t enum. */
static const GpioInputDescriptor_t GPIO_INPUT_TABLE[DIN_COUNT] =
{
    /* The onboard B1 button already has an external pull-up on the Nucleo
     * board, so no internal pull is required for the ignition input. */
    [DIN_IGNITION]   = { PIN_BTN_IGNITION_PORT,   PIN_BTN_IGNITION_PIN,
                         true,  GPIO_NOPULL   },

    /* External buttons short the pin to ground when pressed, so the internal
     * pull-up is what defines the idle level. Without it the pin would float
     * and read random noise. */
    [DIN_BRAKE]      = { PIN_BTN_BRAKE_PORT,      PIN_BTN_BRAKE_PIN,
                         true,  GPIO_PULLUP   },
    [DIN_TURN_LEFT]  = { PIN_BTN_TURN_LEFT_PORT,  PIN_BTN_TURN_LEFT_PIN,
                         true,  GPIO_PULLUP   },
    [DIN_TURN_RIGHT] = { PIN_BTN_TURN_RIGHT_PORT, PIN_BTN_TURN_RIGHT_PIN,
                         true,  GPIO_PULLUP   },
};

/* Order MUST match the DigitalOutput_t enum. */
static const GpioOutputDescriptor_t GPIO_OUTPUT_TABLE[DOUT_COUNT] =
{
    [DOUT_STATUS_LED]      = { PIN_LED_STATUS_PORT,    PIN_LED_STATUS_PIN,    false },
    [DOUT_HEADLIGHT]       = { PIN_LED_HEADLIGHT_PORT, PIN_LED_HEADLIGHT_PIN, false },
    [DOUT_INDICATOR_LEFT]  = { PIN_LED_IND_LEFT_PORT,  PIN_LED_IND_LEFT_PIN,  false },
    [DOUT_INDICATOR_RIGHT] = { PIN_LED_IND_RIGHT_PORT, PIN_LED_IND_RIGHT_PIN, false },
};

/* ========================================================================= */
/* Runtime state                                                             */
/* ========================================================================= */

/**
 * @brief Debounce state for one input.
 */
typedef struct
{
    bool     stable_state;      /**< Last accepted (debounced) logical level   */
    bool     candidate_state;   /**< Level currently being confirmed           */
    uint8_t  match_count;       /**< Consecutive samples matching candidate    */
    uint32_t active_since_ms;   /**< HAL tick when the input became active     */
} InputDebounceState_t;

static InputDebounceState_t s_input_state[DIN_COUNT];

/* Shadow copy of commanded output levels. Reading a pin's output data
 * register would also work, but a shadow copy keeps the read path identical
 * on host-side test builds where no hardware exists. */
static bool s_output_state[DOUT_COUNT];

/* ========================================================================= */
/* Internal helpers                                                          */
/* ========================================================================= */

/**
 * @brief Convert an electrical pin level into a logical "active" flag.
 */
static inline bool gpio_to_logical(bool pin_is_high, bool active_low)
{
    return active_low ? (!pin_is_high) : pin_is_high;
}

/**
 * @brief Convert a logical "active" flag into the electrical level to drive.
 */
static inline GPIO_PinState gpio_from_logical(bool active, bool active_low)
{
    const bool drive_high = active_low ? (!active) : active;
    return drive_high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/**
 * @brief Enable the peripheral clocks for every GPIO bank the ECU uses.
 *
 * On STM32, a peripheral is electrically disconnected from the bus until its
 * clock is enabled. Writing to its registers beforehand silently does
 * nothing - a classic source of "my pin does not work" bugs.
 */
static void gpio_enable_bank_clocks(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

void GpioDriver_Init(void)
{
    GPIO_InitTypeDef init = {0};

    gpio_enable_bank_clocks();

    /* --- Outputs ---------------------------------------------------------
     * Configured before inputs and driven inactive immediately, so the ECU
     * never energises a lamp or relay during the initialisation window. */
    for (uint8_t i = 0U; i < (uint8_t)DOUT_COUNT; i++)
    {
        const GpioOutputDescriptor_t *desc = &GPIO_OUTPUT_TABLE[i];

        HAL_GPIO_WritePin(desc->port, desc->pin,
                          gpio_from_logical(false, desc->active_low));

        init.Pin   = desc->pin;
        init.Mode  = GPIO_MODE_OUTPUT_PP;   /* push-pull: actively drives both levels */
        init.Pull  = GPIO_NOPULL;           /* irrelevant for a driven output         */
        init.Speed = GPIO_SPEED_FREQ_LOW;   /* lamps do not need fast edges, and a
                                             * slow slew rate radiates less EMI       */
        HAL_GPIO_Init(desc->port, &init);

        s_output_state[i] = false;
    }

    /* --- Inputs --------------------------------------------------------- */
    for (uint8_t i = 0U; i < (uint8_t)DIN_COUNT; i++)
    {
        const GpioInputDescriptor_t *desc = &GPIO_INPUT_TABLE[i];

        init.Pin   = desc->pin;
        init.Mode  = GPIO_MODE_INPUT;
        init.Pull  = desc->pull;
        init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(desc->port, &init);

        /* Seed the debounce state from the real pin level so the ECU does
         * not report a spurious edge on its very first sample. */
        const bool pin_high = (HAL_GPIO_ReadPin(desc->port, desc->pin) == GPIO_PIN_SET);
        const bool logical  = gpio_to_logical(pin_high, desc->active_low);

        s_input_state[i].stable_state    = logical;
        s_input_state[i].candidate_state = logical;
        s_input_state[i].match_count     = 0U;
        s_input_state[i].active_since_ms = logical ? HAL_GetTick() : 0U;
    }
}

void GpioDriver_SampleInputs(void)
{
    const uint32_t now_ms = HAL_GetTick();

    for (uint8_t i = 0U; i < (uint8_t)DIN_COUNT; i++)
    {
        const GpioInputDescriptor_t *desc  = &GPIO_INPUT_TABLE[i];
        InputDebounceState_t        *state = &s_input_state[i];

        const bool pin_high = (HAL_GPIO_ReadPin(desc->port, desc->pin) == GPIO_PIN_SET);
        const bool sampled  = gpio_to_logical(pin_high, desc->active_low);

        if (sampled == state->stable_state)
        {
            /* Reading agrees with the accepted level: nothing to confirm.
             * Reset the counter so that a burst of bounces has to start
             * over rather than accumulating across separate disturbances. */
            state->match_count     = 0U;
            state->candidate_state = sampled;
            continue;
        }

        if (sampled == state->candidate_state)
        {
            state->match_count++;
        }
        else
        {
            /* The reading changed again mid-confirmation: this is bounce.
             * Restart the count against the new candidate. */
            state->candidate_state = sampled;
            state->match_count     = 1U;
        }

        if (state->match_count >= ECU_INPUT_DEBOUNCE_SAMPLES)
        {
            state->stable_state = state->candidate_state;
            state->match_count  = 0U;

            /* Timestamp rising edges so callers can measure press duration. */
            state->active_since_ms = state->stable_state ? now_ms : 0U;
        }
    }
}

bool GpioDriver_GetInput(DigitalInput_t input)
{
    if (input >= DIN_COUNT)
    {
        return false;
    }
    return s_input_state[input].stable_state;
}

uint32_t GpioDriver_GetActiveDurationMs(DigitalInput_t input)
{
    if ((input >= DIN_COUNT) || (!s_input_state[input].stable_state))
    {
        return 0U;
    }

    /* Unsigned subtraction handles the 32-bit HAL tick wrapping correctly:
     * at 1 kHz the counter wraps every ~49.7 days, and (now - then) still
     * yields the true elapsed time across the wrap. */
    return HAL_GetTick() - s_input_state[input].active_since_ms;
}

void GpioDriver_SetOutput(DigitalOutput_t output, bool active)
{
    if (output >= DOUT_COUNT)
    {
        return;
    }

    const GpioOutputDescriptor_t *desc = &GPIO_OUTPUT_TABLE[output];

    HAL_GPIO_WritePin(desc->port, desc->pin,
                      gpio_from_logical(active, desc->active_low));

    s_output_state[output] = active;
}

bool GpioDriver_GetOutput(DigitalOutput_t output)
{
    if (output >= DOUT_COUNT)
    {
        return false;
    }
    return s_output_state[output];
}

void GpioDriver_AllOutputsOff(void)
{
    for (uint8_t i = 0U; i < (uint8_t)DOUT_COUNT; i++)
    {
        GpioDriver_SetOutput((DigitalOutput_t)i, false);
    }
}
