/**
 * @file    body_control.c
 * @brief   Application layer implementation - see body_control.h.
 *
 * Layer discipline: this file calls services and drivers. It never touches
 * the STM32 HAL directly, with the single exception of HAL_GetTick() for
 * timing. Keeping that boundary is what makes the layer diagram in
 * docs/software_architecture.md a description of the code rather than an
 * aspiration about it.
 */

#include "body_control.h"

#include "diag_app.h"
#include "vehicle_state_machine.h"

#include "can_manager.h"
#include "diag_manager.h"
#include "fault_manager.h"
#include "logger.h"
#include "scheduler.h"

#include "adc_driver.h"
#include "can_driver.h"
#include "gpio_driver.h"
#include "pwm_driver.h"

#include "ecu_config.h"
#include "stm32f4xx_hal.h"

/* ========================================================================= */
/* Module state                                                              */
/* ========================================================================= */

/* Ignition edge detection. The state machine needs discrete press events,
 * but GPIO only offers a level, so the edges are reconstructed here. */
static bool s_ignition_was_active = false;
static bool s_long_press_fired    = false;

/* Latest battery measurement, shared from the 500 ms task to the 100 ms CAN
 * task. Both run from the same cooperative scheduler on the same thread, so
 * no synchronisation is required: a task always runs to completion before
 * another starts. Under an RTOS with pre-emption this would need a mutex or
 * an atomic access - which is exactly the complexity the cooperative model
 * is trading away. */
static uint32_t s_battery_mv = ECU_BATT_NOMINAL_MV;

static bool s_init_ok = false;

/* ========================================================================= */
/* Input processing                                                          */
/* ========================================================================= */

/**
 * @brief Turn the ignition input level into a discrete state machine event.
 *
 * Two gestures are distinguished:
 *
 *      SHORT PRESS  press and release within ECU_IGNITION_LONG_PRESS_MS.
 *                   Reported on *release*, because until the button comes
 *                   back up it is not yet known to be short.
 *
 *      LONG PRESS   held past the threshold. Reported the moment the
 *                   threshold is crossed, while the button is still down -
 *                   exactly like a real key, where the starter engages as
 *                   you hold it rather than when you let go.
 *
 * A latch ensures the long press fires once per hold, not once per task tick.
 *
 * @return The event to feed the state machine, or VSM_EVENT_NONE.
 */
static VsmEvent_t body_control_read_ignition_event(void)
{
    const bool     is_active   = GpioDriver_GetInput(DIN_IGNITION);
    const uint32_t duration_ms = GpioDriver_GetActiveDurationMs(DIN_IGNITION);

    VsmEvent_t event = VSM_EVENT_NONE;

    if (is_active && (!s_ignition_was_active))
    {
        /* Rising edge: a new gesture begins. Arm the long-press latch. */
        s_long_press_fired = false;
    }
    else if (is_active && s_ignition_was_active)
    {
        /* Still held: check whether it has just become a long press. */
        if ((!s_long_press_fired) && (duration_ms >= ECU_IGNITION_LONG_PRESS_MS))
        {
            event              = VSM_EVENT_IGNITION_LONG_PRESS;
            s_long_press_fired = true;
        }
    }
    else if ((!is_active) && s_ignition_was_active)
    {
        /* Falling edge: released. If the long press already fired, this
         * release is merely its end and must not also count as a short
         * press - otherwise starting the engine would immediately shut the
         * vehicle down again. */
        if (!s_long_press_fired)
        {
            event = VSM_EVENT_IGNITION_SHORT_PRESS;
        }
    }
    else
    {
        /* Idle: not pressed now, not pressed before. */
    }

    s_ignition_was_active = is_active;

    return event;
}

/* ========================================================================= */
/* Fault evaluation                                                          */
/* ========================================================================= */

/**
 * @brief Evaluate undervoltage with hysteresis.
 *
 * The threshold to *clear* the fault sits above the threshold to *set* it:
 *
 *      set    when V < 11.00 V
 *      clear  when V > 11.30 V          (11.00 + 0.30 hysteresis)
 *
 * Without this gap, a battery sitting at exactly 11.00 V would set and clear
 * the fault on alternate readings as noise moved it a few millivolts either
 * way. Hysteresis converts an ambiguous boundary into two unambiguous ones -
 * the same trick a Schmitt trigger applies to a noisy analogue input.
 *
 * @param  battery_mv  Measured battery voltage.
 * @return true if the undervoltage condition is present right now.
 */
static bool body_control_is_undervoltage(uint32_t battery_mv)
{
    if (FaultManager_IsActive(FAULT_BATTERY_UNDERVOLTAGE))
    {
        return (battery_mv < (ECU_BATT_UNDERVOLTAGE_MV + ECU_BATT_HYSTERESIS_MV));
    }
    return (battery_mv < ECU_BATT_UNDERVOLTAGE_MV);
}

/**
 * @brief Evaluate overvoltage with hysteresis, mirroring the undervoltage logic.
 */
static bool body_control_is_overvoltage(uint32_t battery_mv)
{
    if (FaultManager_IsActive(FAULT_BATTERY_OVERVOLTAGE))
    {
        return (battery_mv > (ECU_BATT_OVERVOLTAGE_MV - ECU_BATT_HYSTERESIS_MV));
    }
    return (battery_mv > ECU_BATT_OVERVOLTAGE_MV);
}

/**
 * @brief Feed one fault evaluation into the manager and log any transition.
 *
 * Logging only on change is what keeps the trace readable: a fault that stays
 * active for a minute produces two lines, not six hundred.
 */
static void body_control_update_fault(FaultId_t id, bool condition_present)
{
    const bool changed = FaultManager_Update(id, condition_present);

    /* Every evaluation - pass or fail - is forwarded to DTC memory, not just
     * the transitions. A passing result is what clears "test not completed"
     * and proves to a tester that the monitor actually ran this cycle. */
    DiagApp_ReportFault(id, FaultManager_IsActive(id));

    if (!changed)
    {
        return;     /* no change - nothing worth logging */
    }

    if (FaultManager_IsActive(id))
    {
        LOG_ERROR("Fault SET:     %s", FaultManager_GetName(id));
    }
    else
    {
        LOG_INFO("Fault CLEARED: %s", FaultManager_GetName(id));
    }
}

/* ========================================================================= */
/* Task: 10 ms - inputs and state machine                                    */
/* ========================================================================= */

static void body_control_task_10ms(void)
{
    /* Sample first. Everything below this line reads the results of this
     * call, so the whole cycle sees one consistent snapshot of the inputs
     * rather than values that shift underneath it. */
    GpioDriver_SampleInputs();

    const bool fault_active      = FaultManager_IsAnyActive();
    const bool in_fault_state    = (VehicleStateMachine_GetState() == VEHICLE_STATE_FAULT);

    VsmEvent_t event;

    /* Fault transitions take priority over driver input. A driver pressing
     * the ignition button must not be able to override the ECU's own
     * judgement that something is wrong. */
    if (fault_active && (!in_fault_state))
    {
        event = VSM_EVENT_FAULT_SET;
    }
    else if ((!fault_active) && in_fault_state)
    {
        event = VSM_EVENT_FAULT_CLEARED;
    }
    else
    {
        event = body_control_read_ignition_event();
    }

    const VehicleState_t previous = VehicleStateMachine_GetState();
    const VehicleState_t current  = VehicleStateMachine_HandleEvent(event);

    if (VehicleStateMachine_DidStateChange())
    {
        LOG_INFO("State: %s -> %s  (event %s)",
                 VehicleStateMachine_GetStateName(previous),
                 VehicleStateMachine_GetStateName(current),
                 VehicleStateMachine_GetEventName(event));

        /* Leaving OFF is ignition-on: a new DTC operation cycle begins. */
        if ((previous == VEHICLE_STATE_OFF) && (current == VEHICLE_STATE_ACC))
        {
            DiagApp_StartOperationCycle();
        }
    }

    /* Scheduler health is evaluated here because this is the fastest task:
     * an overrun anywhere shows up within one cycle of happening. */
    body_control_update_fault(FAULT_TASK_OVERRUN,
                              Scheduler_GetOverrunCount() > 0U);
}

/* ========================================================================= */
/* Task: 50 ms - output actuation                                            */
/* ========================================================================= */

/**
 * @brief Global flash phase for the turn indicators.
 *
 * Derived directly from the millisecond tick rather than from a toggling
 * flag. Two consequences, both deliberate:
 *
 *   - Left and right are always in phase, so hazard lights flash together
 *     with no extra synchronisation code.
 *   - The phase is stateless, so it cannot drift and needs no reset when an
 *     indicator is switched on part-way through a cycle.
 */
static bool body_control_indicator_phase(void)
{
    return ((HAL_GetTick() / ECU_INDICATOR_HALF_PERIOD_MS) % 2U) == 0U;
}

/**
 * @brief Drive the status LED with a pattern that encodes the ECU's state.
 *
 * A single LED carrying real diagnostic information is worth far more than
 * one that is simply on. Any technician with no equipment can read it:
 *
 *      slow blink (1 Hz)   ECU alive, vehicle OFF
 *      solid on            vehicle powered - ACC, ON or RUN
 *      fast blink (5 Hz)   fault latched, ECU in fail-safe
 *
 * The OFF pattern doubles as a heartbeat: if it stops, the main loop has
 * stopped, which is visible from across the bench.
 */
static void body_control_update_status_led(VehicleState_t state)
{
    bool led_on;

    switch (state)
    {
        case VEHICLE_STATE_FAULT:
            led_on = ((HAL_GetTick() / 100U) % 2U) == 0U;   /* 5 Hz */
            break;

        case VEHICLE_STATE_OFF:
            led_on = ((HAL_GetTick() / 500U) % 2U) == 0U;   /* 1 Hz */
            break;

        default:
            led_on = true;                                   /* solid */
            break;
    }

    GpioDriver_SetOutput(DOUT_STATUS_LED, led_on);
}

static void body_control_task_50ms(void)
{
    const VehicleState_t    state = VehicleStateMachine_GetState();
    const VsmCapabilities_t caps  = VehicleStateMachine_GetCapabilities(state);

    body_control_update_status_led(state);

    /* A diagnostic lamp self-test overrides normal lighting logic: every lamp
     * on, brake light at full brightness, for the duration of the routine.
     * The routine itself refuses to start while the engine is running. */
    if (DiagApp_IsLampTestActive())
    {
        GpioDriver_SetOutput(DOUT_HEADLIGHT, true);
        GpioDriver_SetOutput(DOUT_INDICATOR_LEFT, true);
        GpioDriver_SetOutput(DOUT_INDICATOR_RIGHT, true);
        PwmDriver_SetBrakeDuty(ECU_PWM_BRAKE_FULL);
        return;
    }

    /* In FAULT, and in any state that does not permit lighting, every
     * exterior lamp is forced off in one call. Making the fail-safe path a
     * single explicit statement - rather than relying on each lamp's own
     * logic to happen to evaluate false - means it cannot be partially
     * applied by a future change. */
    if (!caps.lighting_enabled)
    {
        GpioDriver_SetOutput(DOUT_HEADLIGHT, false);
        GpioDriver_SetOutput(DOUT_INDICATOR_LEFT, false);
        GpioDriver_SetOutput(DOUT_INDICATOR_RIGHT, false);
        PwmDriver_SetBrakeDuty(ECU_PWM_BRAKE_OFF);
        return;
    }

    /* --- Headlights ------------------------------------------------------
     * Driven by the lighting capability. A production BCM would combine this
     * with a light switch input and an ambient light sensor. */
    GpioDriver_SetOutput(DOUT_HEADLIGHT, true);

    /* --- Turn indicators ------------------------------------------------- */
    const bool want_left  = GpioDriver_GetInput(DIN_TURN_LEFT);
    const bool want_right = GpioDriver_GetInput(DIN_TURN_RIGHT);
    const bool phase      = body_control_indicator_phase();

    /* Both stalks at once is the hazard-light gesture. It falls out of the
     * logic for free because both lamps share one phase. */
    GpioDriver_SetOutput(DOUT_INDICATOR_LEFT,  want_left  && phase);
    GpioDriver_SetOutput(DOUT_INDICATOR_RIGHT, want_right && phase);

    /* --- Brake light -----------------------------------------------------
     * Two intensities from one lamp, which is how a real combined
     * tail/brake light works: dim while running, full brightness under
     * braking. PWM makes that a single-lamp, single-wire solution. */
    const bool braking = GpioDriver_GetInput(DIN_BRAKE);

    PwmDriver_SetBrakeDuty(braking ? ECU_PWM_BRAKE_FULL
                                   : ECU_PWM_BRAKE_TAIL_LIGHT);
}

/* ========================================================================= */
/* Task: 100 ms - CAN transmit and receive                                   */
/* ========================================================================= */

static void body_control_task_100ms(void)
{
    /* Reception now happens every 5 ms in the diagnostic task (DiagApp_Task),
     * so this frame already reflects the latest data from the network. */
    CanVehicleStatus_t status;

    status.vehicle_state   = (uint8_t)VehicleStateMachine_GetState();
    status.ignition_on     = GpioDriver_GetInput(DIN_IGNITION);
    status.brake_active    = GpioDriver_GetInput(DIN_BRAKE);
    status.indicator_left  = GpioDriver_GetOutput(DOUT_INDICATOR_LEFT);
    status.indicator_right = GpioDriver_GetOutput(DOUT_INDICATOR_RIGHT);
    status.headlight_on    = GpioDriver_GetOutput(DOUT_HEADLIGHT);
    status.battery_mv      = (uint16_t)s_battery_mv;
    status.fault_count     = FaultManager_GetActiveCount();
    status.fault_bitmask   = FaultManager_GetActiveBitmask();
    status.alive_counter   = 0U;    /* filled in by CanManager */

    /* Indicator signals report the *lamp*, not the switch, so a receiver
     * sees them flashing at the real rate. That matters because an
     * instrument cluster telltale is required to mirror the exterior lamp -
     * including staying dark when a bulb has failed. */

    const bool transmitted = CanManager_SendVehicleStatus(&status);

    body_control_update_fault(FAULT_CAN_TX_FAILURE, !transmitted);
    body_control_update_fault(FAULT_CAN_BUS_OFF,    CanDriver_IsBusOff());
}

/* ========================================================================= */
/* Task: 500 ms - battery measurement and diagnostics                        */
/* ========================================================================= */

static void body_control_task_500ms(void)
{
    /* --- Battery measurement --------------------------------------------- */
    uint32_t   measured_mv = 0U;
    const bool adc_ok      = AdcDriver_ReadBatteryMv(&measured_mv);

    body_control_update_fault(FAULT_ADC_FAILURE, !adc_ok);

    if (adc_ok)
    {
        s_battery_mv = measured_mv;

        body_control_update_fault(FAULT_BATTERY_UNDERVOLTAGE,
                                  body_control_is_undervoltage(measured_mv));
        body_control_update_fault(FAULT_BATTERY_OVERVOLTAGE,
                                  body_control_is_overvoltage(measured_mv));
    }
    else
    {
        /* The measurement failed, so the previous value is now of unknown
         * age. Voltage faults are evaluated as "not present" rather than
         * being judged on data we no longer trust: the ADC_FAILURE fault
         * above already reports that something is wrong, and raising a
         * second, speculative voltage fault would mislead the diagnosis. */
        body_control_update_fault(FAULT_BATTERY_UNDERVOLTAGE, false);
        body_control_update_fault(FAULT_BATTERY_OVERVOLTAGE,  false);
    }

    /* --- Diagnostics broadcast ------------------------------------------- */
    const uint32_t uptime_s = HAL_GetTick() / 1000U;

    /* Worst-case execution time across all registered tasks. This is the
     * number that proves - or disproves - that the schedule closes. */
    uint32_t worst_task_ms = 0U;
    for (uint8_t i = 0U; i < Scheduler_GetTaskCount(); i++)
    {
        const uint32_t duration = Scheduler_GetMaxDurationMs(i);
        if (duration > worst_task_ms)
        {
            worst_task_ms = duration;
        }
    }

    CanDiagnostics_t diagnostics;

    /* Every field is narrowed deliberately, with saturation rather than
     * wraparound: a counter that has exceeded its CAN signal range should
     * report "at least 255", never fold back to a small and reassuring
     * number. */
    diagnostics.uptime_seconds = (uint16_t)((uptime_s > UINT16_MAX)
                                            ? UINT16_MAX : uptime_s);

    const uint32_t overruns = Scheduler_GetOverrunCount();
    diagnostics.task_overrun_count = (uint16_t)((overruns > UINT16_MAX)
                                                ? UINT16_MAX : overruns);

    diagnostics.max_task_duration_ms = (uint8_t)((worst_task_ms > UINT8_MAX)
                                                  ? UINT8_MAX : worst_task_ms);

    const uint32_t tx_failures = CanDriver_GetTxFailureCount();
    diagnostics.can_tx_failures = (uint8_t)((tx_failures > UINT8_MAX)
                                             ? UINT8_MAX : tx_failures);

    diagnostics.alive_counter = 0U;     /* filled in by CanManager */

    (void)CanManager_SendDiagnostics(&diagnostics);

    /* --- Periodic health line -------------------------------------------
     * At DEBUG level, so a release build carries neither the call nor the
     * format string and pays nothing for it. */
    LOG_DEBUG("state=%s batt=%lumV faults=0x%04X rx=%lu",
              VehicleStateMachine_GetStateName(VehicleStateMachine_GetState()),
              (unsigned long)s_battery_mv,
              (unsigned int)FaultManager_GetActiveBitmask(),
              (unsigned long)CanManager_GetReceivedFrameCount());
}

/* ========================================================================= */
/* Initialisation                                                            */
/* ========================================================================= */

bool BodyControl_Init(bool was_watchdog_reset)
{
    bool ok = true;

    /* 1. Logging first: everything after this point can report its own
     *    failure, which is the difference between a diagnosable bring-up
     *    and a board that simply does nothing. */
    if (!Logger_Init())
    {
        /* Nothing can be reported - the reporting channel is what failed.
         * Recorded so BodyControl_IsHealthy() still reflects reality. */
        ok = false;
    }

    Logger_PrintBanner(was_watchdog_reset);

    /* 2. Drivers: put the hardware into a defined state. */
    GpioDriver_Init();

    if (!PwmDriver_Init())
    {
        LOG_ERROR("Init failed: PWM (TIM3)");
        ok = false;
    }

    if (!AdcDriver_Init())
    {
        LOG_ERROR("Init failed: ADC1");
        ok = false;
    }

    /* 3. Services. */
    FaultManager_Init();

    if (!CanManager_Init())
    {
        /* Not fatal. Without a transceiver attached, CAN initialisation is
         * expected to fail on a bare bench setup, and every local function -
         * lighting, ignition, diagnostics over UART - still works. Degrading
         * rather than refusing to start is the correct behaviour for a
         * non-safety-critical communication channel. */
        LOG_WARN("CAN unavailable - continuing without bus communication");
        ok = false;
    }

    /* Diagnostics. May erase a flash sector while compacting the DTC store,
     * which takes up to ~2 s - acceptable only because the watchdog is not
     * armed yet (main.c arms it after BodyControl_Init returns). */
    if (DiagApp_Init())
    {
        LOG_INFO("Diagnostics ready - UDS on 0x%03X/0x%03X, NVM %u%% used",
                 (unsigned int)ECU_DIAG_CAN_ID_PHYSICAL,
                 (unsigned int)ECU_DIAG_CAN_ID_RESPONSE,
                 (unsigned int)DiagManager_GetNvmUsagePercent());
    }
    else
    {
        LOG_WARN("NVM unavailable - DTCs will not survive a reset");
        ok = false;
    }

    /* 4. Application state. */
    VehicleStateMachine_Init();

    s_ignition_was_active = false;
    s_long_press_fired    = false;
    s_battery_mv          = ECU_BATT_NOMINAL_MV;

    /* 5. Scheduler last. Registration order is execution order when several
     *    tasks fall due together, so tasks are registered in data-flow
     *    order: sample, then act, then communicate, then diagnose. */
    Scheduler_Init();

    /* Reception + diagnostics first: every other task then works with the
     * freshest frames. At 5 ms, a UDS request is answered well inside the
     * 50 ms P2 limit the ECU advertises. */
    ok &= Scheduler_RegisterTask("diag_rx",   DiagApp_Task,
                                 ECU_TASK_PERIOD_5MS);
    ok &= Scheduler_RegisterTask("input_sm",  body_control_task_10ms,
                                 ECU_TASK_PERIOD_10MS);
    ok &= Scheduler_RegisterTask("outputs",   body_control_task_50ms,
                                 ECU_TASK_PERIOD_50MS);
    ok &= Scheduler_RegisterTask("can_comms", body_control_task_100ms,
                                 ECU_TASK_PERIOD_100MS);
    ok &= Scheduler_RegisterTask("diag",      body_control_task_500ms,
                                 ECU_TASK_PERIOD_500MS);

    LOG_INFO("Body control initialised - %u tasks registered",
             Scheduler_GetTaskCount());

    s_init_ok = ok;
    return ok;
}

bool BodyControl_IsHealthy(void)
{
    return s_init_ok;
}

uint32_t BodyControl_GetBatteryMv(void)
{
    return s_battery_mv;
}
