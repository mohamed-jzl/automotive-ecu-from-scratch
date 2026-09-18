/**
 * @file    vehicle_state_machine.h
 * @brief   Vehicle operating-mode state machine - the brain of the ECU.
 *
 * Why a state machine rather than a pile of if-statements
 * ------------------------------------------------------
 * The ECU's behaviour depends on more than its current inputs; it depends on
 * how it got here. Pressing the ignition button means "wake up" from OFF and
 * "shut down" from RUN - the same stimulus, opposite results. Encoding that
 * with boolean flags produces code where every new feature multiplies the
 * number of reachable combinations, and where nobody can say with confidence
 * which combinations are legal.
 *
 * A finite state machine makes the legal set explicit and finite. Three
 * definitions carry all of it:
 *
 *      STATE       what the system currently is          (OFF, ACC, ON, ...)
 *      EVENT       something that just happened          (button pressed)
 *      TRANSITION  state + event -> new state            (the rules)
 *
 * Anything not listed as a transition simply cannot happen. That is a much
 * stronger guarantee than "we tested it and it seemed fine", and it is why
 * automotive software uses this pattern for operating modes, diagnostic
 * sessions, charging sequences and network management alike.
 *
 * The vehicle modes
 * -----------------
 *
 *           ┌──────────────────────────────────────────────┐
 *           │                                              │
 *           ▼           short press          short press   │
 *       ┌───────┐  ──────────────────▶  ┌───────┐          │
 *       │  OFF  │                       │  ACC  │          │
 *       └───────┘                       └───────┘          │
 *           ▲                                │             │
 *           │                     short press│             │
 *           │                                ▼             │
 *           │  short press              ┌───────┐          │
 *           ├───────────────────────────│  ON   │          │
 *           │                           └───────┘          │
 *           │                                │             │
 *           │                      long press│             │
 *           │                                ▼             │
 *           │  short press              ┌───────┐          │
 *           └───────────────────────────│  RUN  │          │
 *                                       └───────┘          │
 *                                                          │
 *       any state ── fault detected ──▶ ┌───────┐          │
 *                                       │ FAULT │──────────┘
 *                                       └───────┘  fault cleared
 *
 *      OFF    Everything dormant. Only the ECU itself is powered.
 *      ACC    Accessories live: radio, windows. Ignition circuit off.
 *      ON     Full electrical system live: lighting, instruments.
 *      RUN    Engine running. Reached by holding the button (cranking).
 *      FAULT  Fail-safe. Entered from anywhere, leaves only when healthy.
 *
 * Purity
 * ------
 * This module has no hardware access, no HAL include and no time source.
 * Given a state and an event it returns the next state, always the same one.
 * That is what lets tests/test_vehicle_state_machine.c walk every transition
 * in the table - including the illegal ones - in a few microseconds on a PC,
 * with no board connected.
 */

#ifndef APP_VEHICLE_STATE_MACHINE_H
#define APP_VEHICLE_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Vehicle operating modes.
 *
 * The numeric values are transmitted on CAN in a 3-bit signal, so they are
 * part of the network interface. Never renumber; append only.
 */
typedef enum
{
    VEHICLE_STATE_OFF   = 0,    /**< Dormant                                 */
    VEHICLE_STATE_ACC   = 1,    /**< Accessories powered                     */
    VEHICLE_STATE_ON    = 2,    /**< Full electrical system powered          */
    VEHICLE_STATE_RUN   = 3,    /**< Engine running                          */
    VEHICLE_STATE_FAULT = 4,    /**< Fail-safe, degraded operation           */
    VEHICLE_STATE_COUNT = 5     /**< Number of states - not a real state     */
} VehicleState_t;

/**
 * @brief Stimuli the state machine reacts to.
 *
 * Events are produced by the application layer from debounced inputs and
 * from the fault manager. Translating raw signals into named events is what
 * keeps this module free of timing and hardware concerns.
 */
typedef enum
{
    VSM_EVENT_NONE = 0,             /**< Nothing happened; state is unchanged  */
    VSM_EVENT_IGNITION_SHORT_PRESS, /**< Button pressed and released quickly   */
    VSM_EVENT_IGNITION_LONG_PRESS,  /**< Button held past the crank threshold  */
    VSM_EVENT_FAULT_SET,            /**< A fault latched                       */
    VSM_EVENT_FAULT_CLEARED,        /**< The last active fault healed          */
    VSM_EVENT_COUNT                 /**< Number of events                      */
} VsmEvent_t;

/**
 * @brief What a given state permits the ECU to do.
 *
 * Keeping this as data derived from the state - rather than as scattered
 * "if (state == X || state == Y)" checks in the output code - means the
 * rules live in exactly one place and can be tested directly.
 */
typedef struct
{
    bool accessories_enabled;   /**< Comfort loads may be powered (ACC and above) */
    bool lighting_enabled;      /**< Exterior lighting may operate (ON and above) */
    bool engine_running;        /**< Powertrain is turning (RUN only)             */
} VsmCapabilities_t;

/**
 * @brief Reset the state machine to VEHICLE_STATE_OFF.
 */
void VehicleStateMachine_Init(void);

/**
 * @brief Apply an event and return the resulting state.
 *
 * Events with no matching transition from the current state are ignored, and
 * the state is left untouched. Silently ignoring an impossible event is the
 * correct behaviour: it means an unexpected stimulus can never drive the ECU
 * into an undefined condition.
 *
 * @param  event  Stimulus to apply.
 * @return The state after processing, which equals the previous state when
 *         the event did not match any transition.
 */
VehicleState_t VehicleStateMachine_HandleEvent(VsmEvent_t event);

/**
 * @brief Current state.
 */
VehicleState_t VehicleStateMachine_GetState(void);

/**
 * @brief Whether the most recent HandleEvent call changed the state.
 *
 * Lets the caller log and act on transitions only, instead of re-evaluating
 * unchanged state every cycle.
 *
 * @return true if the last event caused a transition.
 */
bool VehicleStateMachine_DidStateChange(void);

/**
 * @brief Capabilities granted by a state.
 *
 * Pure function of its argument - no dependency on the current state - so it
 * can be tested exhaustively across all five states.
 *
 * @param  state  State to describe.
 * @return The capability set. An out-of-range state yields all-false, the
 *         safe interpretation.
 */
VsmCapabilities_t VehicleStateMachine_GetCapabilities(VehicleState_t state);

/**
 * @brief Human-readable state name for logs and CAN diagnostics.
 *
 * @param  state  State to name.
 * @return Static string, or "INVALID" if out of range. Never NULL.
 */
const char *VehicleStateMachine_GetStateName(VehicleState_t state);

/**
 * @brief Human-readable event name, for tracing.
 *
 * @param  event  Event to name.
 * @return Static string, or "INVALID" if out of range. Never NULL.
 */
const char *VehicleStateMachine_GetEventName(VsmEvent_t event);

#endif /* APP_VEHICLE_STATE_MACHINE_H */
