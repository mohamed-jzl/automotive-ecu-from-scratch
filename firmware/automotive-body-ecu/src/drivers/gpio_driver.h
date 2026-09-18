/**
 * @file    gpio_driver.h
 * @brief   Digital input/output abstraction for the Body Control ECU.
 *
 * This driver is the only module allowed to touch GPIO registers or the
 * STM32 HAL GPIO API. Everything above it speaks in terms of *logical ECU
 * signals* ("is the brake pedal pressed?") rather than electrical facts
 * ("is PC0 low?").
 *
 * Two responsibilities:
 *
 *   1. Polarity translation. Some inputs are wired active low (pressing the
 *      button pulls the pin to ground). The driver inverts those internally
 *      so that GpioDriver_GetInput() always returns true for "active".
 *
 *   2. Debouncing. A mechanical switch does not produce a clean edge; its
 *      contacts bounce for several milliseconds. The driver samples each
 *      input periodically and only reports a new level once it has been
 *      stable for ECU_INPUT_DEBOUNCE_SAMPLES consecutive samples.
 *
 * Debouncing here is *non-blocking*: it counts samples across successive
 * task activations instead of stalling the CPU in a delay loop. An ECU that
 * blocks for 20 ms inside an input read cannot also meet a 10 ms control
 * deadline, so blocking debounce is never acceptable in production firmware.
 *
 * Usage contract: call GpioDriver_SampleInputs() exactly once per 10 ms task.
 * GpioDriver_GetInput() is then a cheap read of already-debounced state and
 * may be called as often as needed.
 */

#ifndef DRIVERS_GPIO_DRIVER_H
#define DRIVERS_GPIO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Logical digital inputs of the ECU.
 *
 * The enum order defines the order of the internal descriptor table, so it
 * must stay in sync with GPIO_INPUT_TABLE in gpio_driver.c.
 */
typedef enum
{
    DIN_IGNITION = 0,   /**< Ignition switch (onboard user button)  */
    DIN_BRAKE,          /**< Brake pedal switch                     */
    DIN_TURN_LEFT,      /**< Turn signal stalk, left                */
    DIN_TURN_RIGHT,     /**< Turn signal stalk, right               */
    DIN_COUNT           /**< Number of inputs - not a real input    */
} DigitalInput_t;

/**
 * @brief Logical digital outputs of the ECU.
 */
typedef enum
{
    DOUT_STATUS_LED = 0,    /**< ECU heartbeat / alive indicator    */
    DOUT_HEADLIGHT,         /**< Headlight relay command            */
    DOUT_INDICATOR_LEFT,    /**< Left turn indicator lamp           */
    DOUT_INDICATOR_RIGHT,   /**< Right turn indicator lamp          */
    DOUT_COUNT              /**< Number of outputs                  */
} DigitalOutput_t;

/**
 * @brief Configure every GPIO pin used by the ECU and drive all outputs low.
 *
 * Must be called once at startup, before any other function in this module.
 * Enables the GPIOA/GPIOB/GPIOC peripheral clocks itself, so it does not
 * depend on CubeMX-generated initialisation.
 */
void GpioDriver_Init(void);

/**
 * @brief Read every physical input and update the debounced state machine.
 *
 * Call once per 10 ms task tick. Calling it faster shortens the effective
 * debounce window; calling it slower lengthens it.
 */
void GpioDriver_SampleInputs(void);

/**
 * @brief Get the debounced logical state of an input.
 *
 * @param  input  Which input to read.
 * @return true if the input is ACTIVE (button pressed / switch closed),
 *         false if inactive or if @p input is out of range.
 */
bool GpioDriver_GetInput(DigitalInput_t input);

/**
 * @brief Report how long an input has been continuously active.
 *
 * Used to distinguish a short ignition press (advance position) from a long
 * press (crank the engine) without duplicating timing logic in the caller.
 *
 * @param  input  Which input to query.
 * @return Milliseconds since the input last became active, or 0 if inactive.
 */
uint32_t GpioDriver_GetActiveDurationMs(DigitalInput_t input);

/**
 * @brief Drive a logical output.
 *
 * Polarity translation is applied internally, so @p active always means
 * "lamp on / relay energised" regardless of how the pin is wired.
 *
 * @param output  Which output to drive.
 * @param active  true to activate, false to deactivate.
 */
void GpioDriver_SetOutput(DigitalOutput_t output, bool active);

/**
 * @brief Read back the last commanded state of an output.
 *
 * Reads the driver's shadow copy, not the pin, so it reports what the
 * software intended. Comparing this against a real current-sense input is
 * how production ECUs detect a blown bulb or an open circuit.
 *
 * @param  output  Which output to query.
 * @return true if the output is currently commanded active.
 */
bool GpioDriver_GetOutput(DigitalOutput_t output);

/**
 * @brief Force every output to its safe (inactive) state.
 *
 * Called by the fault handler when the ECU enters its fail-safe state.
 */
void GpioDriver_AllOutputsOff(void);

#endif /* DRIVERS_GPIO_DRIVER_H */
