/* Host tests for launch control */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "../Core/Src/launch_control.c"

static int failures = 0;
static int checks = 0;

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_near(float got, float want, float tol, const char *what)
{
    checks++;
    if (fabsf(got - want) > tol) {
        failures++;
        printf("  FAIL: %s (got %.6f want %.6f tol %.6f)\n", what, got, want, tol);
    }
}

static void check_eq(int32_t got, int32_t want, const char *what)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL: %s (got %d want %d)\n", what, (int)got, (int)want);
    }
}

#define STEP_MS 10u

static uint32_t test_tick;
static float test_front_ms;
static int test_sensor_live;

static void lc_reset_all(void)
{
    launch_control_enable = 0;
    front_wheel_speed_ms = 0.0f;
    last_wheel_speed_tick = 0;
    current_tick_ms = 0;
    test_tick = 1000;
    test_front_ms = 0.0f;
    test_sensor_live = 1;
    lc_init();
    lc_feed_tick(test_tick);
}

static void set_front(float ms)
{
    test_front_ms = ms;
}

/* m/s to CAN km/h x10 */
static uint16_t ms_to_x10(float ms)
{
    return (uint16_t)(ms * 36.0f + 0.5f);
}

/* m/s to motor rpm */
static uint32_t ms_to_rpm(float ms)
{
    return (uint32_t)(ms * LC_MS_TO_RPM + 0.5f);
}

static int32_t step_dt(int32_t torque, uint32_t rpm, float tps, uint32_t dt_ms)
{
    test_tick += dt_ms;
    lc_feed_tick(test_tick);
    if (test_sensor_live) {
        lc_feed_wheel_speed(ms_to_x10(test_front_ms));
    }
    return lc_update(torque, rpm, tps);
}

static int32_t step(int32_t torque, uint32_t rpm, float tps)
{
    return step_dt(torque, rpm, tps, STEP_MS);
}

static void arm_and_launch(float front_ms)
{
    set_front(0.0f);
    launch_control_enable = 1;
    step(1000, 0, 0.5f);
    set_front(front_ms);
}

static void test_conversions(void)
{
    printf("test_conversions\n");

    /* 36.0 km/h is 10 m/s */
    lc_reset_all();
    lc_feed_wheel_speed(360);
    check_near(front_wheel_speed_ms, 10.0f, 1e-4f, "360 km/h x10 -> 10 m/s");

    lc_feed_wheel_speed(0);
    check_near(front_wheel_speed_ms, 0.0f, 1e-6f, "zero km/h -> zero m/s");

    check_near(motor_rpm_to_wheel_ms(1000), 1000.0f * LC_RPM_TO_MS, 1e-4f, "rpm -> m/s");
    check_near(motor_rpm_to_wheel_ms(0), 0.0f, 1e-6f, "zero rpm -> zero m/s");

    /* Conversion constants are exact inverses */
    check_near(LC_RPM_TO_MS * LC_MS_TO_RPM, 1.0f, 1e-5f, "RPM_TO_MS inverts MS_TO_RPM");
}

static void test_map_interp(void)
{
    printf("test_map_interp\n");

    static const uint16_t bp[4] = {0, 1000, 2000, 3000};
    static const uint16_t val[4] = {100, 200, 400, 400};

    check_eq((int32_t)map_interp(bp, val, 4, 0), 100, "at first breakpoint");
    check_eq((int32_t)map_interp(bp, val, 4, 1000), 200, "at an interior breakpoint");
    check_eq((int32_t)map_interp(bp, val, 4, 3000), 400, "at last breakpoint");

    check_eq((int32_t)map_interp(bp, val, 4, 500), 150, "midpoint of first segment");
    check_eq((int32_t)map_interp(bp, val, 4, 1500), 300, "midpoint of second segment");
    check_eq((int32_t)map_interp(bp, val, 4, 250), 125, "quarter of first segment");

    /* Clamping outside the table, not extrapolation. */
    check_eq((int32_t)map_interp(bp, val, 4, 99999), 400, "above table clamps high");

    /* A flat segment returns its value rather than dividing by zero. */
    check_eq((int32_t)map_interp(bp, val, 4, 2500), 400, "flat segment");

    /* Degenerate table, the shape of the real one today: no out-of-bounds
     * read, no divide by zero. */
    static const uint16_t flat_bp[4] = {0, 0, 0, 0};
    static const uint16_t flat_val[4] = {10, 10, 10, 10};
    check_eq((int32_t)map_interp(flat_bp, flat_val, 4, 0), 10, "degenerate table at zero");
    check_eq((int32_t)map_interp(flat_bp, flat_val, 4, 5000), 10, "degenerate table at speed");
}

static void test_slip_ratio(void)
{
    printf("test_slip_ratio\n");

    /* Below the 0.5 m/s floor slip is forced to zero. */
    lc_reset_all();
    set_front(0.0f);
    step(1000, 1, 0.0f);
    check_near(dbg.slip_ratio_raw, 0.0f, 1e-6f, "no slip below the speed floor");

    /* Matched speeds mean no slip. */
    lc_reset_all();
    set_front(10.0f);
    step(1000, ms_to_rpm(10.0f), 0.0f);
    check_near(dbg.slip_ratio_raw, 0.0f, 0.01f, "front == rear gives zero slip");

    /* Rear faster than front: lambda = (rear - front) / max. */
    lc_reset_all();
    set_front(8.0f);
    step(1000, ms_to_rpm(10.0f), 0.0f);
    check_near(dbg.slip_ratio_raw, 0.2f, 0.01f, "(10-8)/10 = 0.2");

    /* Front faster than rear (braking) clamps to zero, not negative. */
    lc_reset_all();
    set_front(20.0f);
    step(1000, ms_to_rpm(10.0f), 0.0f);
    check_near(dbg.slip_ratio_raw, 0.0f, 1e-6f, "negative slip clamps to zero");

    /* A stationary front wheel with the rear spinning is full slip. */
    lc_reset_all();
    set_front(0.0f);
    step(1000, ms_to_rpm(10.0f), 0.0f);
    check_near(dbg.slip_ratio_raw, 1.0f, 1e-6f, "stationary front is full slip");
}

static void test_slip_filter_and_rate(void)
{
    printf("test_slip_filter_and_rate\n");

    /* At the nominal period the weight is LC_SLIP_FILTER_ALPHA, so a step to
     * raw slip 1.0 from 0 lands at (1 - alpha). */
    lc_reset_all();
    set_front(0.0f);
    const uint32_t rpm10 = ms_to_rpm(10.0f);

    /* The filter seeds on the first launching pass, so launch with both speeds
     * at zero; the step to raw 1.0 below is then a genuine filter step. */
    arm_and_launch(0.0f);
    step(1000, 0u, 0.5f);
    check_near(dbg.slip_ratio, 0.0f, 1e-6f, "filter seeds at the measured slip");
    check_near(dbg.slip_rate, 0.0f, 1e-6f, "seeded pass reports no slip rate");

    step(1000, rpm10, 0.5f);
    const float after_one = 1.0f - LC_SLIP_FILTER_ALPHA;
    check_near(dbg.slip_ratio, after_one, 1e-3f, "one filter step at nominal dt");

    step(1000, rpm10, 0.5f);
    const float after_two = after_one * LC_SLIP_FILTER_ALPHA + (1.0f - LC_SLIP_FILTER_ALPHA);
    check_near(dbg.slip_ratio, after_two, 1e-3f, "two filter steps at nominal dt");

    /* Slip rate is the per-call delta over the measured period. */
    check_near(dbg.slip_rate, (after_two - after_one) / ((float)STEP_MS * 0.001f), 1e-1f,
               "slip rate is d(slip)/dt");

    /* Settles toward the raw value. */
    for (int i = 0; i < 200; i++) {
        step(1000, rpm10, 0.5f);
    }
    check_near(dbg.slip_ratio, 1.0f, 1e-3f, "filter settles at raw slip");
    check_near(dbg.slip_rate, 0.0f, 1e-2f, "slip rate settles at zero");
}

static void test_measured_dt(void)
{
    printf("test_measured_dt\n");

    /* No previous tick after a reset: falls back to the nominal period. */
    lc_reset_all();
    step(1000, 0, 0.0f);
    check_near(dbg.dt_s, LC_DT_S, 1e-6f, "first update falls back to nominal dt");

    /* Later updates report the measured delta. */
    step_dt(1000, 0, 0.0f, 25u);
    check_near(dbg.dt_s, 0.025f, 1e-6f, "25 ms step measured");
    step_dt(1000, 0, 0.0f, 7u);
    check_near(dbg.dt_s, 0.007f, 1e-6f, "7 ms step measured");

    /* Two updates inside one ms clamp to the floor, not divide by zero. */
    step_dt(1000, 0, 0.0f, 0u);
    check_near(dbg.dt_s, LC_DT_MIN_S, 1e-6f, "zero elapsed clamps to the floor");
    check(isfinite(dbg.slip_rate), "slip rate stays finite at zero elapsed");

    /* A long stall clamps to the ceiling. */
    step_dt(1000, 0, 0.0f, 5000u);
    check_near(dbg.dt_s, LC_DT_MAX_S, 1e-6f, "a stall clamps to the ceiling");

    /* Wrap-safe across a tick rollover. */
    lc_reset_all();
    test_tick = 0xFFFFFFF0u;
    lc_feed_tick(test_tick);
    step(1000, 0, 0.0f); /* nominal, first update */
    step_dt(1000, 0, 0.0f, 20u);
    check_near(dbg.dt_s, 0.020f, 1e-6f, "dt is correct across a tick wraparound");
}

static void test_sensor_health(void)
{
    printf("test_sensor_health\n");

    /* Never fed: unhealthy. */
    lc_reset_all();
    test_sensor_live = 0;
    step(1000, 0, 0.0f);
    check(dbg.sensor_healthy == 0, "unhealthy before any wheel speed frame");

    /* Fresh frame: healthy. */
    lc_reset_all();
    set_front(10.0f);
    step(1000, 0, 0.0f);
    check(dbg.sensor_healthy == 1, "healthy with a fresh frame");

    /* Stop broadcasting; exactly at the timeout is still healthy. */
    test_sensor_live = 0;
    step_dt(1000, 0, 0.0f, LC_SENSOR_TIMEOUT_MS);
    check(dbg.sensor_healthy == 1, "healthy at exactly the timeout");

    /* One millisecond past it. */
    step_dt(1000, 0, 0.0f, 1u);
    check(dbg.sensor_healthy == 0, "unhealthy one ms past the timeout");

    /* Tick wraparound: unsigned subtraction must still yield a small elapsed. */
    lc_reset_all();
    test_tick = 0xFFFFFFF0u;
    lc_feed_tick(test_tick);
    set_front(10.0f);
    step_dt(1000, 0, 0.0f, 21u); /* wraps past zero */
    check(dbg.sensor_healthy == 1, "healthy across a tick wraparound");
}

static void test_state_machine(void)
{
    printf("test_state_machine\n");

    /* Disabled: never leaves ARMED however hard the driver presses. */
    lc_reset_all();
    set_front(10.0f);
    step(1000, 0, 1.0f);
    check(dbg.state == LC_STATE_ARMED, "stays armed while disabled");

    /* Enable arms it. */
    launch_control_enable = 1;
    step(1000, 0, 0.0f);
    check(dbg.state == LC_STATE_ARMED, "stays armed on enable");

    /* Below the trigger it stays armed. */
    step(1000, 0, LC_THROTTLE_TRIGGER - 0.01f);
    check(dbg.state == LC_STATE_ARMED, "stays armed below the throttle trigger");

    /* At the trigger it launches. Front speed is below the crossover. */
    set_front(1.0f);
    step(1000, 0, LC_THROTTLE_TRIGGER);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "armed -> open loop at the trigger");

    /* Open loop holds while the car is slower than the crossover. */
    step(1000, 0, 0.5f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "open loop below crossover speed");

    /* Past the crossover it closes the loop. */
    set_front(10.0f);
    step(1000, 0, 0.5f);
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "open -> closed loop above crossover");

    /* A dead sensor drops back to open loop. */
    test_sensor_live = 0;
    step_dt(1000, 0, 0.5f, LC_SENSOR_TIMEOUT_MS + 1u);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "closed -> open loop when the sensor dies");

    /* Lifting off ends the launch. */
    step(1000, 0, 0.0f);
    check(dbg.state == LC_STATE_ARMED, "lift off returns to armed");

    /* Exit speed ends the launch. */
    lc_reset_all();
    arm_and_launch(10.0f);
    step(1000, 0, 0.5f);
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "reached closed loop");
    set_front(LC_EXIT_SPEED_MS + 1.0f);
    step(1000, 0, 0.5f);
    check(dbg.state == LC_STATE_ARMED, "exit speed returns to armed");

    /* Disabling mid-launch ends it */
    lc_reset_all();
    arm_and_launch(1.0f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "launching before disable");
    launch_control_enable = 0;
    step(1000, 0, 0.5f);
    check(dbg.state == LC_STATE_ARMED, "disable mid-launch returns to armed");
}

static void test_negative_torque_passthrough(void)
{
    printf("test_negative_torque_passthrough\n");

    /* Regen torque is negative and passes through unchanged */
    const int32_t regen = -250;

    lc_reset_all();
    set_front(10.0f);
    check_eq(step(regen, 0, 0.0f), regen, "regen passes through while disabled");

    launch_control_enable = 1;
    step(regen, 0, 0.0f);
    check_eq(step(regen, 0, 0.0f), regen, "regen passes through while armed");

    lc_reset_all();
    arm_and_launch(1.0f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "open loop reached");
    check_eq(step(regen, 0, 0.5f), regen, "the map never clobbers regen torque");

    /* Zero torque boundary */
    check_eq(step(0, 0, 0.5f), 0, "zero torque passes through");
}

static void test_torque_limiting(void)
{
    printf("test_torque_limiting\n");

    /* Open loop takes the map value, which is below the driver request. */
    lc_reset_all();
    arm_and_launch(1.0f);
    const int32_t out = step(2200, 0, 0.5f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "open loop");
    check_eq(dbg.map_torque, (int32_t)lc_map_lookup(0), "map torque published");
    check_eq(out, dbg.map_torque < 2200 ? dbg.map_torque : 2200,
             "open loop commands the map torque, capped at the request");

    /* Never above the driver request. */
    lc_reset_all();
    arm_and_launch(10.0f);
    for (int i = 0; i < 50; i++) {
        const int32_t t = step(500, ms_to_rpm(10.0f), 0.5f);
        check(t <= 500, "output never exceeds the driver request");
        check(t >= 0, "output never goes negative in a launching state");
    }
}

static void test_slip_rate_cut(void)
{
    printf("test_slip_rate_cut\n");

    /* Slip from zero to full in one step exceeds LC_SLIP_RATE_MAX. Front speed
     * stays below the crossover, so this is the open-loop path, which is the
     * only one that applies the cut; closed loop uses LC_KD. */
    lc_reset_all();
    arm_and_launch(1.0f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "open loop");

    /* Seed the filter at zero slip first; the rate is only meaningful once the
     * filter holds a settled value. */
    step(2200, ms_to_rpm(1.0f), 0.5f);
    check_near(dbg.slip_rate, 0.0f, 1e-6f, "seeded pass reports no slip rate");

    const int32_t out = step(2200, ms_to_rpm(100.0f), 0.5f);
    check(dbg.slip_rate > LC_SLIP_RATE_MAX, "slip rate exceeded the limit");
    check(dbg.slip_rate_cut == 1, "slip rate cut flag raised");
    check_eq(out, (int32_t)((float)dbg.map_torque * LC_SLIP_RATE_CUT_MULT),
             "torque scaled by the cut multiplier");

    /* Once slip settles the flag clears. */
    for (int i = 0; i < 100; i++) {
        step(2200, ms_to_rpm(100.0f), 0.5f);
    }
    check(dbg.slip_rate_cut == 0, "slip rate cut clears once slip settles");
}

static void test_pid_behaviour(void)
{
    printf("test_pid_behaviour\n");

    /* Slip above target drives the controller output negative. */
    lc_reset_all();
    arm_and_launch(3.0f);
    const uint32_t spin_rpm = ms_to_rpm(6.0f); /* 50% slip */
    step(2200, spin_rpm, 0.5f);
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "closed loop");

    for (int i = 0; i < 20; i++) {
        step(2200, spin_rpm, 0.5f);
    }
    check(dbg.slip_ratio > LC_SLIP_TARGET, "slip is above target");
    check(dbg.pid_output < 0.0f, "PID output is negative above target slip");
    check(dbg.lc_torque < 2200, "torque cut below the driver request");
    check(fabsf(dbg.pid_i_term) <= 2200.0f + 1.0f, "I term clamped to driver torque");

    /* Cleared on the same call that leaves closed loop. */
    launch_control_enable = 0;
    check_eq(step(2200, spin_rpm, 0.5f), 2200, "transition call passes torque through");
    check(dbg.state == LC_STATE_ARMED, "back to armed");
    check_near(dbg.pid_i_term, 0.0f, 1e-6f, "I term reset on the transition call");
    check_near(dbg.pid_output, 0.0f, 1e-6f, "PID output reset on the transition call");
    check_near(dbg.pid_p_term, 0.0f, 1e-6f, "P term reset on the transition call");
    check_near(dbg.pid_d_term, 0.0f, 1e-6f, "D term reset on the transition call");
    check(dbg.slip_rate_cut == 0, "slip rate cut cleared on the transition call");
}

/* Every route back to ARMED clears the controller on the transition call. */
static void test_armed_entry_clears_pid(void)
{
    printf("test_armed_entry_clears_pid\n");

    const uint32_t spin_rpm = ms_to_rpm(6.0f);

    /* Route 1: lift off out of closed loop. */
    lc_reset_all();
    arm_and_launch(3.0f);
    for (int i = 0; i < 20; i++) {
        step(2200, spin_rpm, 0.5f);
    }
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "closed loop");
    check(dbg.pid_i_term != 0.0f, "I term wound up");
    step(2200, spin_rpm, 0.0f); /* lift off */
    check(dbg.state == LC_STATE_ARMED, "lift off -> armed");
    check_near(dbg.pid_i_term, 0.0f, 1e-6f, "lift off clears the I term");
    check_near(dbg.pid_d_term, 0.0f, 1e-6f, "lift off clears the D term");

    /* Route 2: exceed the exit speed. */
    lc_reset_all();
    arm_and_launch(3.0f);
    for (int i = 0; i < 20; i++) {
        step(2200, spin_rpm, 0.5f);
    }
    check(dbg.pid_i_term != 0.0f, "I term wound up again");
    set_front(LC_EXIT_SPEED_MS + 1.0f);
    step(2200, spin_rpm, 0.5f);
    check(dbg.state == LC_STATE_ARMED, "exit speed -> armed");
    check_near(dbg.pid_i_term, 0.0f, 1e-6f, "exit speed clears the I term");
    check_near(dbg.pid_d_term, 0.0f, 1e-6f, "exit speed clears the D term");
}

/* Reaching LC_EXIT_SPEED_MS latches, so a launch cannot restart while the car
 * is still above that speed with the throttle held. */
static void test_exit_speed_does_not_rearm(void)
{
    printf("test_exit_speed_does_not_rearm\n");

    lc_reset_all();
    arm_and_launch(3.0f);
    step(2200, ms_to_rpm(3.3f), 0.9f);
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "closed loop before the exit");

    set_front(25.0f); /* past LC_EXIT_SPEED_MS, driver still flat */
    int left_armed = 0;
    int trimmed_torque = 0;
    for (int i = 0; i < 40; i++) {
        if (step(2200, ms_to_rpm(25.0f), 0.9f) != 2200) {
            trimmed_torque = 1;
        }
        if (dbg.state != LC_STATE_ARMED) {
            left_armed = 1;
        }
    }
    check(!left_armed, "stays armed above exit speed while the throttle is held");
    check(!trimmed_torque, "torque passes through untouched after the exit");

    /* Lifting releases the latch so the next launch still works. */
    step(0, ms_to_rpm(25.0f), 0.0f);
    check(dbg.state == LC_STATE_ARMED, "re-arms once the driver lifts");
}

static void test_closed_loop_derivative(void)
{
    printf("test_closed_loop_derivative\n");

    lc_reset_all();
    arm_and_launch(3.0f);

    /* Closed loop seeds slip filter */
    step(2200, ms_to_rpm(3.3f), 0.9f);
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "closed loop");
    check_near(dbg.pid_d_term, 0.0f, 1.0f, "no derivative kick on entry");

    for (int i = 0; i < 10; i++) {
        step(2200, ms_to_rpm(3.3f), 0.9f);
    }
    check_near(dbg.pid_d_term, 0.0f, 1.0f, "no derivative term at steady slip");

    const int32_t out = step(2200, ms_to_rpm(6.0f), 0.9f);
    check(dbg.slip_rate > 0.0f, "slip rate is positive as slip rises");
    check(dbg.pid_d_term < 0.0f, "derivative term trims torque as slip rises");
    check(out < 2200, "torque is cut below the driver request as slip rises");
    check(dbg.slip_rate_cut == 0, "closed loop does not use the slip rate cut");

    for (int i = 0; i < 25; i++) {
        step(2200, ms_to_rpm(6.0f), 0.9f);
    }
    check(fabsf(dbg.pid_d_term) < fabsf(dbg.pid_p_term),
          "proportional term holds the steady state once slip settles");
}

static void test_no_launch_at_speed(void)
{
    printf("test_no_launch_at_speed\n");
    lc_reset_all();
    launch_control_enable = 1;

    set_front(10.0f);
    step(1500, ms_to_rpm(10.0f), 0.8f);
    check(dbg.state == LC_STATE_ARMED, "does not trigger launch while moving at speed");
    check_eq(step(1500, ms_to_rpm(10.0f), 0.8f), 1500, "torque passes through untouched at speed");

    set_front(0.0f);
    step(1500, 0, 0.0f);
    step(1500, 0, 0.8f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "triggers launch at standstill");
}

static void test_bumpless_crossover(void)
{
    printf("test_bumpless_crossover\n");
    lc_reset_all();
    set_front(0.0f);
    launch_control_enable = 1;
    step(2000, 0, 0.8f);
    check(dbg.state == LC_STATE_LAUNCHING_OPENLOOP, "open loop");

    set_front(1.9f);
    const int32_t t_open = step(2000, ms_to_rpm(2.11f), 0.8f);
    check_eq(t_open, dbg.map_torque, "command is map torque in open loop");

    set_front(2.1f);
    const int32_t t_closed = step(2000, ms_to_rpm(2.33f), 0.8f);
    check(dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP, "crossed over to closed loop");
    check(abs(t_closed - dbg.map_torque) <= 10, "bumpless transfer from open loop to closed loop");
}

int main(void)
{
    test_conversions();
    test_map_interp();
    test_slip_ratio();
    test_slip_filter_and_rate();
    test_measured_dt();
    test_sensor_health();
    test_state_machine();
    test_negative_torque_passthrough();
    test_torque_limiting();
    test_slip_rate_cut();
    test_exit_speed_does_not_rearm();
    test_closed_loop_derivative();
    test_pid_behaviour();
    test_armed_entry_clears_pid();
    test_no_launch_at_speed();
    test_bumpless_crossover();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
