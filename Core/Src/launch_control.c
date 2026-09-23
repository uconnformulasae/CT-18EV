#include "launch_control.h"
#include <math.h>

/* Do not set to 1 until lc_map_rpm/lc_map_torque are populated. They are
 * placeholders: every breakpoint 0 rpm, every value 1.0 Nm, so the lookup
 * returns 1.0 Nm at any speed and the car is clamped to a crawl. */
volatile uint8_t launch_control_enable = 0;

#define LC_MAP_SIZE 8

// RPM vs Torque maps
static const uint16_t lc_map_rpm[LC_MAP_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

// These are x10 values
static const uint16_t lc_map_torque[LC_MAP_SIZE] = {10, 10, 10, 10, 10, 10, 10, 10};

/* Interpolate a table of monotonically increasing breakpoints. Separate from
 * lc_map_lookup() so tests can drive it with a populated table. */
static uint32_t map_interp(const uint16_t *bp, const uint16_t *val, int n, uint32_t x)
{
    // Clamp to the ends of the table
    if (x <= bp[0]) {
        return val[0];
    }
    if (x >= bp[n - 1]) {
        return val[n - 1];
    }

    // Find the bracketing breakpoints and interpolate
    for (int i = 0; i < n - 1; i++) {
        if (x < bp[i + 1]) {
            const uint32_t bp_lo = bp[i];
            const uint32_t bp_hi = bp[i + 1];
            const uint32_t val_lo = val[i];
            const uint32_t val_hi = val[i + 1];
            const uint32_t bp_range = bp_hi - bp_lo;

            if (bp_range == 0) {
                return val_lo;
            }

            // Linear interpolation
            const int32_t val_range = (int32_t)val_hi - (int32_t)val_lo;
            const uint32_t offset = x - bp_lo;
            return (uint32_t)((int32_t)val_lo + (val_range * (int32_t)offset) / (int32_t)bp_range);
        }
    }

    return val[n - 1];
}

static uint32_t lc_map_lookup(uint32_t motor_speed_rpm)
{
    return map_interp(lc_map_rpm, lc_map_torque, LC_MAP_SIZE, motor_speed_rpm);
}


static volatile float front_wheel_speed_ms = 0.0f;

/* Timestamp of the last AiM wheel speed CAN message. Written after the speed
 * it belongs to and read before it; see lc_feed_wheel_speed(). */
static volatile uint32_t last_wheel_speed_tick = 0;

/* Current system tick, fed from the main loop and read by the CAN RX ISR, so
 * it must not be cached in a register. */
static volatile uint32_t current_tick_ms = 0;

// Tick at the previous lc_update(), for the measured loop period.
static uint32_t last_update_tick = 0;
static uint8_t have_last_update = 0;

// Debug/telemetry state exposed via lc_get_debug()
static lc_debug_t dbg;

static float prev_slip_ratio = 0.0f;

// Whether the previous pass was launching; gates the slip filter in lc_update().
static uint8_t was_launching = 0;

/* Set when a launch ends by reaching LC_EXIT_SPEED_MS, cleared when the driver
 * lifts. Blocks re-arming so ARMED cannot cycle straight back into a launch
 * while the car is still above exit speed with the throttle pinned. */
static uint8_t exit_latched = 0;

static float motor_rpm_to_wheel_ms(uint32_t motor_speed_rpm)
{
    return (float)motor_speed_rpm * LC_RPM_TO_MS;
}

/* Slip filter weight for a given period. expf() is a software routine on this
 * Cortex-M3 (-mfloat-abi=soft), so the result is cached per whole millisecond
 * of loop period, which dt is already clamped to. The weight is a pure
 * function of the period, so entries never need invalidating, and zero is a
 * safe "not computed" marker: the smallest weight in range is about 6e-6. */
static float alpha_cache[LC_DT_MAX_MS + 1u];

static float slip_filter_alpha(float dt)
{
    uint32_t ms = (uint32_t)(dt * 1000.0f + 0.5f);
    if (ms > LC_DT_MAX_MS) {
        ms = LC_DT_MAX_MS;
    }
    if (alpha_cache[ms] == 0.0f) {
        alpha_cache[ms] = expf(-dt / LC_SLIP_TAU_S);
    }
    return alpha_cache[ms];
}

static void lc_reset_pid(void)
{
    dbg.pid_i_term = 0.0f;
    dbg.pid_p_term = 0.0f;
    dbg.pid_d_term = 0.0f;
    dbg.pid_output = 0.0f;
    dbg.slip_rate_cut = 0;
}

/* Clear on the transition, not on the next pass through the ARMED case, so the
 * published PID terms match the published state. */
static void lc_enter_armed(void)
{
    dbg.state = LC_STATE_ARMED;
    lc_reset_pid();
}

void lc_init(void)
{
    dbg.state = LC_STATE_ARMED;
    dbg.slip_ratio = 0.0f;
    dbg.slip_ratio_raw = 0.0f;
    dbg.slip_rate = 0.0f;
    dbg.pid_p_term = 0.0f;
    dbg.pid_i_term = 0.0f;
    dbg.pid_d_term = 0.0f;
    dbg.pid_output = 0.0f;
    dbg.lc_torque = 0;
    dbg.map_torque = 0;
    dbg.vehicle_speed = 0.0f;
    dbg.rear_wheel_speed = 0.0f;
    dbg.sensor_healthy = 0;
    dbg.slip_rate_cut = 0;
    dbg.dt_s = LC_DT_S;
    prev_slip_ratio = 0.0f;
    was_launching = 0;
    exit_latched = 0;
    last_update_tick = 0;
    have_last_update = 0;
}

void lc_feed_wheel_speed(uint16_t front_left_speed_x10)
{
    /* Runs in the CAN RX ISR, which can land between lc_update()'s two loads.
     * Speed is published before its timestamp and read after it, so a torn
     * read pairs a fresh speed with the previous frame's timestamp: the sensor
     * reads staler than it is, never fresher. Both objects are volatile, which
     * holds the store order. */
    front_wheel_speed_ms = (float)front_left_speed_x10 * LC_KMH10_TO_MS;
    last_wheel_speed_tick = current_tick_ms;
}

void lc_feed_tick(uint32_t tick_ms)
{
    current_tick_ms = tick_ms;
}

const lc_debug_t *lc_get_debug(void)
{
    return &dbg;
}

lc_state_t lc_get_state(void)
{
    return dbg.state;
}

int32_t lc_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined)
{

    /* Measured loop period; the slip derivative and integral term are both
     * per-second. Subtraction is wrap-safe; bounds cover a stall or a double
     * call. */
    float dt = LC_DT_S;
    if (have_last_update) {
        dt = (float)(current_tick_ms - last_update_tick) * 0.001f;
    }
    if (dt < LC_DT_MIN_S) {
        dt = LC_DT_MIN_S;
    }
    if (dt > LC_DT_MAX_S) {
        dt = LC_DT_MAX_S;
    }
    last_update_tick = current_tick_ms;
    have_last_update = 1;
    dbg.dt_s = dt;

    // Timestamp before speed; see lc_feed_wheel_speed() for why the order matters.
    uint32_t wheel_tick = last_wheel_speed_tick;
    float v_front = front_wheel_speed_ms;

    uint8_t sensor_ok = 1;
    uint32_t elapsed = current_tick_ms - wheel_tick;
    // Handle tick wraparound if elapsed is huge, sensor timed out
    if (elapsed > LC_SENSOR_TIMEOUT_MS || wheel_tick == 0) {
        sensor_ok = 0;
    }
    dbg.sensor_healthy = sensor_ok;

    float v_rear = motor_rpm_to_wheel_ms(motor_speed_rpm);

    dbg.rear_wheel_speed = v_rear;
    dbg.vehicle_speed = v_front;

    // Slip ratio: lambda = (v_rear - v_front) / max(v_rear, v_front, epsilon)
    float max_speed = fmaxf(v_rear, v_front);
    float slip_raw = 0.0f;

    // maybe make this value bigger?
    if (max_speed > 0.5f) {

        slip_raw = (v_rear - v_front) / max_speed; // only when meaningful
    }
    slip_raw = fmaxf(0.0f, fminf(slip_raw, 1.0f));
    dbg.slip_ratio_raw = slip_raw;

    /* The filter only carries history through a launch. Entering one seeds it
     * with the measured slip, so slip_rate reports the tyres rather than the
     * filter settling up from zero. dbg.state is still the previous pass's
     * value here; the transition switch runs below.
     *
     * The weight follows the measured period, holding the time constant at
     * LC_SLIP_TAU_S under jitter; at dt == LC_DT_S it is LC_SLIP_FILTER_ALPHA. */
    const uint8_t launching =
        (dbg.state == LC_STATE_LAUNCHING_OPENLOOP || dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP);
    if (launching && was_launching) {
        const float alpha = slip_filter_alpha(dt);
        dbg.slip_ratio = dbg.slip_ratio * alpha + slip_raw * (1.0f - alpha);

        // Slip rate (dlambda/dt)
        dbg.slip_rate = (dbg.slip_ratio - prev_slip_ratio) / dt;
    } else {
        dbg.slip_ratio = slip_raw;
        dbg.slip_rate = 0.0f;
    }
    prev_slip_ratio = dbg.slip_ratio;
    was_launching = launching;

    switch (dbg.state) {

    case LC_STATE_ARMED:
        lc_reset_pid();

        // Lifting off below crossover speed releases the exit latch.
        if (tps_combined < LC_THROTTLE_RELEASE && v_front <= LC_CROSSOVER_SPEED_MS) {
            exit_latched = 0;
        }

        if (launch_control_enable && !exit_latched && v_front <= LC_CROSSOVER_SPEED_MS &&
            tps_combined >= LC_THROTTLE_TRIGGER) {
            dbg.state = LC_STATE_LAUNCHING_OPENLOOP;
        }
        break;

    case LC_STATE_LAUNCHING_OPENLOOP:

        if (!launch_control_enable || tps_combined < LC_THROTTLE_RELEASE) {
            lc_enter_armed();
        }
        // Crossover to closed loop when vehicle speed is reliable
        else if (sensor_ok && v_front > LC_CROSSOVER_SPEED_MS) {
            dbg.state = LC_STATE_LAUNCHING_CLOSEDLOOP;

            int32_t map_t = (int32_t)lc_map_lookup(motor_speed_rpm);
            dbg.map_torque = map_t;

            float error = LC_SLIP_TARGET - dbg.slip_ratio;
            float p_term = LC_KP * error;
            float d_term = -LC_KD * dbg.slip_rate;

            float target_i =
                (float)map_t - (float)driver_torque - p_term - d_term - LC_KI * error * dt;
            if (target_i > 0.0f) {
                target_i = 0.0f;
            }
            dbg.pid_i_term = target_i;
        }
        break;

    case LC_STATE_LAUNCHING_CLOSEDLOOP:

        if (!launch_control_enable || tps_combined < LC_THROTTLE_RELEASE) {
            lc_enter_armed();
        } else if (v_front > LC_EXIT_SPEED_MS) {
            /* Latch, or ARMED re-arms on the next pass while the car is
             * still above exit speed and the driver is still flat. */
            exit_latched = 1;
            lc_enter_armed();
        }
        // Fall back to open-loop if sensor dies
        else if (!sensor_ok) {
            dbg.state = LC_STATE_LAUNCHING_OPENLOOP;
        }
        break;
    }

    /* LC only reduces positive drive torque. Regen is negative and passes
     * through, so the clamps below can assume a positive bound. */
    if (driver_torque <= 0) {
        dbg.map_torque = driver_torque;
        dbg.lc_torque = driver_torque;
        dbg.slip_rate_cut = 0;
        return driver_torque;
    }

    int32_t lc_torque_limit = driver_torque; // Default: passthrough

    switch (dbg.state) {

    case LC_STATE_ARMED:

        dbg.map_torque = driver_torque;
        dbg.lc_torque = driver_torque;
        return driver_torque;

    case LC_STATE_LAUNCHING_OPENLOOP: {

        int32_t map_t = (int32_t)lc_map_lookup(motor_speed_rpm);
        dbg.map_torque = map_t;
        lc_torque_limit = map_t;

        // Even in open loop, if we have sensor data, apply slip rate limiter
        if (sensor_ok && dbg.slip_rate > LC_SLIP_RATE_MAX) {
            lc_torque_limit = (int32_t)((float)lc_torque_limit * LC_SLIP_RATE_CUT_MULT);
            dbg.slip_rate_cut = 1;
        } else {
            dbg.slip_rate_cut = 0;
        }
        break;
    }

    case LC_STATE_LAUNCHING_CLOSEDLOOP: {

        //  error = lambda_target - lambda_actual
        //  Positive error: below target slip → allow more torque
        //  Negative error: above target slip → reduce torque
        float error = LC_SLIP_TARGET - dbg.slip_ratio;

        dbg.pid_p_term = LC_KP * error;

        /* Derivative on the measurement rather than the error. The target is
         * fixed so the two are equivalent today, but this keeps a target
         * change from kicking the output. Rising slip is a positive slip_rate,
         * so the term goes negative and trims torque before the proportional
         * term has had time to build. */
        dbg.pid_d_term = -LC_KD * dbg.slip_rate;

        /* Conditional integration: accumulate only when doing so would not
         * drive the command further into a limit. Saturation is enforced by
         * the output clamp below and nowhere else; a limit written back into
         * pid_i_term outlives the transient that set it. */
        float i_candidate = dbg.pid_i_term + LC_KI * error * dt;
        float cl_candidate = (float)driver_torque + dbg.pid_p_term + dbg.pid_d_term + i_candidate;

        if (cl_candidate > (float)driver_torque) {
            if (error < 0.0f) {
                dbg.pid_i_term = i_candidate;
            }
        } else if (cl_candidate < 0.0f) {
            if (error > 0.0f) {
                dbg.pid_i_term = i_candidate;
            }
        } else {
            dbg.pid_i_term = i_candidate;
        }

        // Controller output
        dbg.pid_output = dbg.pid_p_term + dbg.pid_i_term + dbg.pid_d_term;

        /* The driver's request is the operating point and the controller
         * supplies only the deviation from it. The gains are sized for that
         * deviation, so dropping the feedforward means retuning them. */
        float cl_torque = (float)driver_torque + dbg.pid_output;

        // Clamp the torque values between zero and driver_torque
        if (cl_torque < 0.0f)
            cl_torque = 0.0f;
        if (cl_torque > (float)driver_torque)
            cl_torque = (float)driver_torque;

        lc_torque_limit = (int32_t)cl_torque;
        // look up from table based on motor speed.
        int32_t map_t = (int32_t)lc_map_lookup(motor_speed_rpm);
        dbg.map_torque = map_t;

        /* No slip-rate cut here: LC_KD covers rising slip continuously, and
         * the clamp above already lets the controller command zero. */
        dbg.slip_rate_cut = 0;
        break;
    }
    }
    // clamps it to driver torque
    if (lc_torque_limit > driver_torque) {
        lc_torque_limit = driver_torque;
    }

    dbg.lc_torque = lc_torque_limit;
    return lc_torque_limit;
}
