/**
 * @file    vehicle_state_machine.c
 * @brief   State machine implementation - see the header for the diagram.
 *
 * Implemented as a transition *table* rather than nested switch statements.
 * The table is the specification: a reviewer reads eight rows and knows every
 * legal move in the system. A switch-based version spreads the same
 * information across branches where a missing case looks identical to a
 * deliberate omission.
 *
 * The table is const, so it lives in flash and costs no RAM.
 */

#include "vehicle_state_machine.h"

/**
 * @brief One row of the transition table: "in this state, this event leads here".
 */
typedef struct
{
    VehicleState_t from;
    VsmEvent_t     event;
    VehicleState_t to;
} VsmTransition_t;

/**
 * @brief The complete legal behaviour of the ECU's operating modes.
 *
 * Any (state, event) pair absent from this table is not a legal move and is
 * ignored at runtime.
 *
 * Fault entry is deliberately NOT listed here. It applies from every state,
 * so encoding it as four near-identical rows would invite one of them to be
 * forgotten when a state is added later. It is handled as an explicit rule
 * before the table is consulted - see VehicleStateMachine_HandleEvent().
 */
static const VsmTransition_t VSM_TRANSITION_TABLE[] =
{
    /* Power-up sequence: each short press advances one position, exactly
     * like turning a physical ignition key one detent at a time. */
    { VEHICLE_STATE_OFF,   VSM_EVENT_IGNITION_SHORT_PRESS, VEHICLE_STATE_ACC },
    { VEHICLE_STATE_ACC,   VSM_EVENT_IGNITION_SHORT_PRESS, VEHICLE_STATE_ON  },

    /* Cranking: holding the button from ON starts the engine. A long press
     * is required so that the engine can never be started by an accidental
     * brush against the button. */
    { VEHICLE_STATE_ON,    VSM_EVENT_IGNITION_LONG_PRESS,  VEHICLE_STATE_RUN },

    /* Shutdown: from ON or RUN a short press returns directly to OFF rather
     * than stepping back down through the intermediate positions. This
     * matches how a real key behaves and, more importantly, means shutting
     * the vehicle down is always a single action. */
    { VEHICLE_STATE_ON,    VSM_EVENT_IGNITION_SHORT_PRESS, VEHICLE_STATE_OFF },
    { VEHICLE_STATE_RUN,   VSM_EVENT_IGNITION_SHORT_PRESS, VEHICLE_STATE_OFF },

    /* Fault recovery: healing returns the ECU to OFF, never straight back to
     * RUN. Restarting a vehicle function without a deliberate driver action
     * would be unsafe - the driver must knowingly re-engage. */
    { VEHICLE_STATE_FAULT, VSM_EVENT_FAULT_CLEARED,        VEHICLE_STATE_OFF },
};

#define VSM_TRANSITION_COUNT \
    (sizeof(VSM_TRANSITION_TABLE) / sizeof(VSM_TRANSITION_TABLE[0]))

static const char *const VSM_STATE_NAMES[VEHICLE_STATE_COUNT] =
{
    [VEHICLE_STATE_OFF]   = "OFF",
    [VEHICLE_STATE_ACC]   = "ACC",
    [VEHICLE_STATE_ON]    = "ON",
    [VEHICLE_STATE_RUN]   = "RUN",
    [VEHICLE_STATE_FAULT] = "FAULT"
};

static const char *const VSM_EVENT_NAMES[VSM_EVENT_COUNT] =
{
    [VSM_EVENT_NONE]                  = "NONE",
    [VSM_EVENT_IGNITION_SHORT_PRESS]  = "IGN_SHORT",
    [VSM_EVENT_IGNITION_LONG_PRESS]   = "IGN_LONG",
    [VSM_EVENT_FAULT_SET]             = "FAULT_SET",
    [VSM_EVENT_FAULT_CLEARED]         = "FAULT_CLR"
};

static VehicleState_t s_current_state = VEHICLE_STATE_OFF;
static bool           s_state_changed = false;

void VehicleStateMachine_Init(void)
{
    s_current_state = VEHICLE_STATE_OFF;
    s_state_changed = false;
}

VehicleState_t VehicleStateMachine_HandleEvent(VsmEvent_t event)
{
    s_state_changed = false;

    if (event >= VSM_EVENT_COUNT)
    {
        return s_current_state;
    }

    /* --- Global rule, evaluated before the table -------------------------
     * A fault takes precedence over everything. It applies from every state
     * and can never be overridden by a driver input, which is precisely what
     * "fail-safe" has to mean: the ECU's own judgement wins. */
    if (event == VSM_EVENT_FAULT_SET)
    {
        if (s_current_state != VEHICLE_STATE_FAULT)
        {
            s_current_state = VEHICLE_STATE_FAULT;
            s_state_changed = true;
        }
        return s_current_state;
    }

    /* --- Table lookup ---------------------------------------------------- */
    for (uint8_t i = 0U; i < (uint8_t)VSM_TRANSITION_COUNT; i++)
    {
        const VsmTransition_t *row = &VSM_TRANSITION_TABLE[i];

        if ((row->from == s_current_state) && (row->event == event))
        {
            s_current_state = row->to;
            s_state_changed = true;
            return s_current_state;
        }
    }

    /* No matching row: the event is not legal from here. Ignore it and hold
     * the current state. This is the guarantee the table structure buys -
     * an unexpected stimulus cannot produce an undefined state. */
    return s_current_state;
}

VehicleState_t VehicleStateMachine_GetState(void)
{
    return s_current_state;
}

bool VehicleStateMachine_DidStateChange(void)
{
    return s_state_changed;
}

VsmCapabilities_t VehicleStateMachine_GetCapabilities(VehicleState_t state)
{
    VsmCapabilities_t caps = { false, false, false };

    switch (state)
    {
        case VEHICLE_STATE_ACC:
            caps.accessories_enabled = true;
            break;

        case VEHICLE_STATE_ON:
            caps.accessories_enabled = true;
            caps.lighting_enabled    = true;
            break;

        case VEHICLE_STATE_RUN:
            caps.accessories_enabled = true;
            caps.lighting_enabled    = true;
            caps.engine_running      = true;
            break;

        case VEHICLE_STATE_OFF:
        case VEHICLE_STATE_FAULT:
        default:
            /* OFF and FAULT both grant nothing, and an out-of-range state
             * falls here too. Defaulting to "no capability" means any future
             * state added without updating this function degrades safely
             * rather than accidentally enabling an actuator. */
            break;
    }

    return caps;
}

const char *VehicleStateMachine_GetStateName(VehicleState_t state)
{
    if (state >= VEHICLE_STATE_COUNT)
    {
        return "INVALID";
    }
    return VSM_STATE_NAMES[state];
}

const char *VehicleStateMachine_GetEventName(VsmEvent_t event)
{
    if (event >= VSM_EVENT_COUNT)
    {
        return "INVALID";
    }
    return VSM_EVENT_NAMES[event];
}
