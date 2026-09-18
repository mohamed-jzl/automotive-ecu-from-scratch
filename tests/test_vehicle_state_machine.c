/**
 * @file    test_vehicle_state_machine.c
 * @brief   Unit tests for the vehicle operating-mode state machine.
 *
 * Runs on a host PC with no microcontroller involved, because the module
 * under test is pure: it has no hardware access and no time source. That is
 * what lets this suite exercise every transition in the specification -
 * including the ones that are supposed to be impossible - in microseconds.
 *
 * Provoking the same coverage on real hardware would mean pressing a button
 * in dozens of precisely-timed sequences and watching an LED. These tests
 * replace that with evidence that can be re-run on every commit.
 *
 * Test strategy:
 *   1. Nominal path      - the transitions the driver uses every day.
 *   2. Fault priority    - faults must override driver input from any state.
 *   3. Negative testing  - illegal events must be ignored, not crash.
 *   4. Exhaustive sweep  - no (state, event) pair may produce an invalid state.
 *   5. Capabilities      - each state grants exactly the right permissions.
 */

#include "unity_min.h"
#include "vehicle_state_machine.h"

/* Every test starts from a freshly reset state machine, so no test can be
 * influenced by the one that ran before it. */
void setUp(void)
{
    VehicleStateMachine_Init();
}

void tearDown(void)
{
}

/**
 * @brief Drive the machine from OFF up to a requested state.
 *
 * Uses only public API - the test never reaches inside the module to force
 * private state. A test that manipulates internals stops being evidence that
 * the public behaviour is correct.
 */
static void drive_to_state(VehicleState_t target)
{
    VehicleStateMachine_Init();

    switch (target)
    {
        case VEHICLE_STATE_RUN:
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS); /* ACC */
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS); /* ON  */
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_LONG_PRESS);  /* RUN */
            break;

        case VEHICLE_STATE_ON:
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS);
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS);
            break;

        case VEHICLE_STATE_ACC:
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS);
            break;

        case VEHICLE_STATE_FAULT:
            (void)VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_SET);
            break;

        case VEHICLE_STATE_OFF:
        default:
            break;
    }
}

/* ========================================================================= */
/* 1. Nominal path                                                           */
/* ========================================================================= */

static void test_initial_state_is_off(void)
{
    /* Safety property: the ECU must power up with everything inactive, never
     * in a state that could energise an actuator before the driver asks. */
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_OFF, VehicleStateMachine_GetState());
}

static void test_off_to_acc_on_short_press(void)
{
    const VehicleState_t result =
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS);

    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_ACC, result);
    TEST_ASSERT_TRUE(VehicleStateMachine_DidStateChange());
}

static void test_acc_to_on_on_short_press(void)
{
    drive_to_state(VEHICLE_STATE_ACC);

    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_ON,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS));
}

static void test_on_to_run_on_long_press(void)
{
    drive_to_state(VEHICLE_STATE_ON);

    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_RUN,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_LONG_PRESS));
}

static void test_full_startup_sequence(void)
{
    /* The complete gesture a driver performs: press, press, hold. */
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_ACC,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS));
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_ON,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS));
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_RUN,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_LONG_PRESS));
}

static void test_shutdown_from_run_is_single_action(void)
{
    drive_to_state(VEHICLE_STATE_RUN);

    /* Shutting the vehicle down must never require stepping back down
     * through ON and ACC. One press, engine off. */
    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_OFF,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS));
}

static void test_shutdown_from_on_is_single_action(void)
{
    drive_to_state(VEHICLE_STATE_ON);

    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_OFF,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS));
}

/* ========================================================================= */
/* 2. Fault priority                                                         */
/* ========================================================================= */

static void test_fault_is_reachable_from_every_state(void)
{
    const VehicleState_t states[] =
    {
        VEHICLE_STATE_OFF, VEHICLE_STATE_ACC,
        VEHICLE_STATE_ON,  VEHICLE_STATE_RUN
    };

    for (unsigned i = 0; i < sizeof(states) / sizeof(states[0]); i++)
    {
        drive_to_state(states[i]);

        TEST_ASSERT_EQUAL_INT(
            VEHICLE_STATE_FAULT,
            VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_SET));
    }
}

static void test_driver_input_cannot_override_a_fault(void)
{
    drive_to_state(VEHICLE_STATE_FAULT);

    /* The core safety property of the whole module. However the driver
     * presses the button, the ECU stays in its fail-safe state until the
     * underlying condition actually heals. */
    (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_SHORT_PRESS);
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_FAULT, VehicleStateMachine_GetState());

    (void)VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_LONG_PRESS);
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_FAULT, VehicleStateMachine_GetState());
}

static void test_fault_recovery_returns_to_off_not_to_run(void)
{
    drive_to_state(VEHICLE_STATE_RUN);
    (void)VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_SET);

    const VehicleState_t recovered =
        VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_CLEARED);

    /* Recovering straight back to RUN would restart a vehicle function with
     * no driver action, which is unacceptable regardless of how healthy the
     * ECU now believes itself to be. */
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_OFF, recovered);
}

static void test_repeated_fault_set_does_not_report_a_change(void)
{
    drive_to_state(VEHICLE_STATE_ON);

    (void)VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_SET);
    TEST_ASSERT_TRUE(VehicleStateMachine_DidStateChange());

    /* Already in FAULT: a second identical event is not a transition.
     * This is what stops the log filling with duplicate fault entries. */
    (void)VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_SET);
    TEST_ASSERT_FALSE(VehicleStateMachine_DidStateChange());
}

/* ========================================================================= */
/* 3. Negative testing - illegal events must be ignored                      */
/* ========================================================================= */

static void test_long_press_from_off_is_ignored(void)
{
    /* Holding the button from OFF must not jump straight to RUN: the engine
     * can only be cranked from ON. */
    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_OFF,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_LONG_PRESS));
    TEST_ASSERT_FALSE(VehicleStateMachine_DidStateChange());
}

static void test_long_press_from_acc_is_ignored(void)
{
    drive_to_state(VEHICLE_STATE_ACC);

    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_ACC,
        VehicleStateMachine_HandleEvent(VSM_EVENT_IGNITION_LONG_PRESS));
}

static void test_fault_cleared_when_no_fault_is_ignored(void)
{
    drive_to_state(VEHICLE_STATE_ON);

    TEST_ASSERT_EQUAL_INT(
        VEHICLE_STATE_ON,
        VehicleStateMachine_HandleEvent(VSM_EVENT_FAULT_CLEARED));
}

static void test_none_event_never_changes_state(void)
{
    drive_to_state(VEHICLE_STATE_RUN);

    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_RUN,
        VehicleStateMachine_HandleEvent(VSM_EVENT_NONE));
    TEST_ASSERT_FALSE(VehicleStateMachine_DidStateChange());
}

static void test_out_of_range_event_is_rejected_safely(void)
{
    drive_to_state(VEHICLE_STATE_ON);

    /* Robustness against a caller bug or memory corruption: an event value
     * outside the enum must not index past the transition table. */
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_ON,
        VehicleStateMachine_HandleEvent((VsmEvent_t)99));
    TEST_ASSERT_EQUAL_INT(VEHICLE_STATE_ON,
        VehicleStateMachine_HandleEvent((VsmEvent_t)VSM_EVENT_COUNT));
}

/* ========================================================================= */
/* 4. Exhaustive sweep                                                       */
/* ========================================================================= */

static void test_no_event_can_produce_an_invalid_state(void)
{
    /* Every state crossed with every event: 5 x 5 = 25 combinations, the
     * complete input space of the module. Whatever happens, the resulting
     * state must remain inside the declared enum.
     *
     * This is the kind of exhaustive proof that is only affordable because
     * the module is pure. On hardware it would be a week of test cases. */
    for (int state = 0; state < VEHICLE_STATE_COUNT; state++)
    {
        for (int event = 0; event < VSM_EVENT_COUNT; event++)
        {
            drive_to_state((VehicleState_t)state);

            const VehicleState_t result =
                VehicleStateMachine_HandleEvent((VsmEvent_t)event);

            /* Only the upper bound is checked. VEHICLE_STATE_OFF is 0 and the
             * enum's underlying type is unsigned, so a ">= OFF" assertion is
             * tautologically true - the compiler says so under -Wtype-limits,
             * and an assertion that can never fail is worse than none at all
             * because it looks like coverage it does not provide. */
            TEST_ASSERT_TRUE(result < VEHICLE_STATE_COUNT);
        }
    }
}

/* ========================================================================= */
/* 5. Capabilities                                                           */
/* ========================================================================= */

static void test_off_grants_nothing(void)
{
    const VsmCapabilities_t caps =
        VehicleStateMachine_GetCapabilities(VEHICLE_STATE_OFF);

    TEST_ASSERT_FALSE(caps.accessories_enabled);
    TEST_ASSERT_FALSE(caps.lighting_enabled);
    TEST_ASSERT_FALSE(caps.engine_running);
}

static void test_acc_grants_accessories_only(void)
{
    const VsmCapabilities_t caps =
        VehicleStateMachine_GetCapabilities(VEHICLE_STATE_ACC);

    TEST_ASSERT_TRUE(caps.accessories_enabled);
    TEST_ASSERT_FALSE(caps.lighting_enabled);
    TEST_ASSERT_FALSE(caps.engine_running);
}

static void test_on_grants_lighting_but_not_engine(void)
{
    const VsmCapabilities_t caps =
        VehicleStateMachine_GetCapabilities(VEHICLE_STATE_ON);

    TEST_ASSERT_TRUE(caps.accessories_enabled);
    TEST_ASSERT_TRUE(caps.lighting_enabled);
    TEST_ASSERT_FALSE(caps.engine_running);
}

static void test_run_grants_everything(void)
{
    const VsmCapabilities_t caps =
        VehicleStateMachine_GetCapabilities(VEHICLE_STATE_RUN);

    TEST_ASSERT_TRUE(caps.accessories_enabled);
    TEST_ASSERT_TRUE(caps.lighting_enabled);
    TEST_ASSERT_TRUE(caps.engine_running);
}

static void test_fault_grants_nothing(void)
{
    const VsmCapabilities_t caps =
        VehicleStateMachine_GetCapabilities(VEHICLE_STATE_FAULT);

    /* This is what "fail-safe" means in code: in the fault state the ECU
     * grants no permission to energise anything. */
    TEST_ASSERT_FALSE(caps.accessories_enabled);
    TEST_ASSERT_FALSE(caps.lighting_enabled);
    TEST_ASSERT_FALSE(caps.engine_running);
}

static void test_invalid_state_grants_nothing(void)
{
    const VsmCapabilities_t caps =
        VehicleStateMachine_GetCapabilities((VehicleState_t)42);

    /* An unknown state must degrade to the safe interpretation rather than
     * accidentally enabling an actuator. */
    TEST_ASSERT_FALSE(caps.accessories_enabled);
    TEST_ASSERT_FALSE(caps.lighting_enabled);
    TEST_ASSERT_FALSE(caps.engine_running);
}

/* ========================================================================= */
/* Names                                                                     */
/* ========================================================================= */

static void test_state_names_are_correct(void)
{
    TEST_ASSERT_EQUAL_STRING("OFF",   VehicleStateMachine_GetStateName(VEHICLE_STATE_OFF));
    TEST_ASSERT_EQUAL_STRING("ACC",   VehicleStateMachine_GetStateName(VEHICLE_STATE_ACC));
    TEST_ASSERT_EQUAL_STRING("ON",    VehicleStateMachine_GetStateName(VEHICLE_STATE_ON));
    TEST_ASSERT_EQUAL_STRING("RUN",   VehicleStateMachine_GetStateName(VEHICLE_STATE_RUN));
    TEST_ASSERT_EQUAL_STRING("FAULT", VehicleStateMachine_GetStateName(VEHICLE_STATE_FAULT));
}

static void test_names_never_return_null(void)
{
    /* The logger passes these straight to a %s format specifier, where a
     * NULL would be undefined behaviour. */
    TEST_ASSERT_EQUAL_STRING("INVALID", VehicleStateMachine_GetStateName((VehicleState_t)99));
    TEST_ASSERT_EQUAL_STRING("INVALID", VehicleStateMachine_GetEventName((VsmEvent_t)99));
}

/* =========================================================================
 * Suite entry point
 *
 * Each test file links into its own executable, as Ceedling does. That keeps
 * one suite's setUp/tearDown from colliding with another's, and means a
 * crash in one suite cannot prevent the others from reporting.
 * ========================================================================= */

int main(void)
{
    UnityBegin("Vehicle State Machine");

    RUN_TEST(test_initial_state_is_off);
    RUN_TEST(test_off_to_acc_on_short_press);
    RUN_TEST(test_acc_to_on_on_short_press);
    RUN_TEST(test_on_to_run_on_long_press);
    RUN_TEST(test_full_startup_sequence);
    RUN_TEST(test_shutdown_from_run_is_single_action);
    RUN_TEST(test_shutdown_from_on_is_single_action);

    RUN_TEST(test_fault_is_reachable_from_every_state);
    RUN_TEST(test_driver_input_cannot_override_a_fault);
    RUN_TEST(test_fault_recovery_returns_to_off_not_to_run);
    RUN_TEST(test_repeated_fault_set_does_not_report_a_change);

    RUN_TEST(test_long_press_from_off_is_ignored);
    RUN_TEST(test_long_press_from_acc_is_ignored);
    RUN_TEST(test_fault_cleared_when_no_fault_is_ignored);
    RUN_TEST(test_none_event_never_changes_state);
    RUN_TEST(test_out_of_range_event_is_rejected_safely);

    RUN_TEST(test_no_event_can_produce_an_invalid_state);

    RUN_TEST(test_off_grants_nothing);
    RUN_TEST(test_acc_grants_accessories_only);
    RUN_TEST(test_on_grants_lighting_but_not_engine);
    RUN_TEST(test_run_grants_everything);
    RUN_TEST(test_fault_grants_nothing);
    RUN_TEST(test_invalid_state_grants_nothing);

    RUN_TEST(test_state_names_are_correct);
    RUN_TEST(test_names_never_return_null);

    return UnityEnd();
}
