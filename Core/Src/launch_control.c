#include "launch_control.h"
#include <math.h>

volatile uint8_t launch_control_enable = 0;

#define LC_MAP_SIZE 8

static const uint16_t lc_map_rpm[LC_MAP_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};
static const uint16_t lc_map_torque[LC_MAP_SIZE] = {10, 10, 10, 10, 10, 10, 10, 10};

static uint32_t map_interp(const uint16_t *bp, const uint16_t *val, int n, uint32_t x)
{
    if (x <= bp[0]) {
        return val[0];
    }
    if (x >= bp[n - 1]) {
        return val[n - 1];
    }

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
static volatile uint32_t last_wheel_speed_tick = 0;
static volatile uint32_t current_tick_ms = 0;

static uint32_t last_update_tick = 0;
static uint8_t have_last_update = 0;
static lc_debug_t dbg;
static float prev_slip_ratio = 0.0f;
static uint8_t was_launching = 0;
static uint8_t exit_latched = 0;

static float motor_rpm_to_wheel_ms(uint32_t motor_speed_rpm)
{
    return (float)motor_speed_rpm * LC_RPM_TO_MS;
}

/* Alpha cache per integer ms */
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

    uint32_t wheel_tick = last_wheel_speed_tick;
    float v_front = front_wheel_speed_ms;

    uint8_t sensor_ok = 1;
    uint32_t elapsed = current_tick_ms - wheel_tick;
    if (elapsed > LC_SENSOR_TIMEOUT_MS || wheel_tick == 0) {
        sensor_ok = 0;
    }
    dbg.sensor_healthy = sensor_ok;

    float v_rear = motor_rpm_to_wheel_ms(motor_speed_rpm);
    dbg.rear_wheel_speed = v_rear;
    dbg.vehicle_speed = v_front;

    /* Slip ratio: (v_rear - v_front) / max(v_rear, v_front) */
    float max_speed = fmaxf(v_rear, v_front);
    float slip_raw = 0.0f;
    if (max_speed > 0.5f) {
        slip_raw = (v_rear - v_front) / max_speed;
    }
    slip_raw = fmaxf(0.0f, fminf(slip_raw, 1.0f));
    dbg.slip_ratio_raw = slip_raw;

    const uint8_t launching =
        (dbg.state == LC_STATE_LAUNCHING_OPENLOOP || dbg.state == LC_STATE_LAUNCHING_CLOSEDLOOP);
    if (launching && was_launching) {
        const float alpha = slip_filter_alpha(dt);
        dbg.slip_ratio = dbg.slip_ratio * alpha + slip_raw * (1.0f - alpha);
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
        } else if (sensor_ok && v_front > LC_CROSSOVER_SPEED_MS) {
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
            exit_latched = 1;
            lc_enter_armed();
        } else if (!sensor_ok) {
            dbg.state = LC_STATE_LAUNCHING_OPENLOOP;
        }
        break;
    }

    /* Pass through regen (negative torque) */
    if (driver_torque <= 0) {
        dbg.map_torque = driver_torque;
        dbg.lc_torque = driver_torque;
        dbg.slip_rate_cut = 0;
        return driver_torque;
    }

    int32_t lc_torque_limit = driver_torque;

    switch (dbg.state) {

    case LC_STATE_ARMED:
        dbg.map_torque = driver_torque;
        dbg.lc_torque = driver_torque;
        return driver_torque;

    case LC_STATE_LAUNCHING_OPENLOOP: {
        int32_t map_t = (int32_t)lc_map_lookup(motor_speed_rpm);
        dbg.map_torque = map_t;
        lc_torque_limit = map_t;

        if (sensor_ok && dbg.slip_rate > LC_SLIP_RATE_MAX) {
            lc_torque_limit = (int32_t)((float)lc_torque_limit * LC_SLIP_RATE_CUT_MULT);
            dbg.slip_rate_cut = 1;
        } else {
            dbg.slip_rate_cut = 0;
        }
        break;
    }

    case LC_STATE_LAUNCHING_CLOSEDLOOP: {
        float error = LC_SLIP_TARGET - dbg.slip_ratio;

        dbg.pid_p_term = LC_KP * error;
        dbg.pid_d_term = -LC_KD * dbg.slip_rate;

        /* Anti-windup clamping */
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

        dbg.pid_output = dbg.pid_p_term + dbg.pid_i_term + dbg.pid_d_term;

        float cl_torque = (float)driver_torque + dbg.pid_output;
        if (cl_torque < 0.0f)
            cl_torque = 0.0f;
        if (cl_torque > (float)driver_torque)
            cl_torque = (float)driver_torque;

        lc_torque_limit = (int32_t)cl_torque;
        dbg.map_torque = (int32_t)lc_map_lookup(motor_speed_rpm);
        dbg.slip_rate_cut = 0;
        break;
    }
    }

    if (lc_torque_limit > driver_torque) {
        lc_torque_limit = driver_torque;
    }

    dbg.lc_torque = lc_torque_limit;
    return lc_torque_limit;
}
