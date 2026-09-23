#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include <stdint.h>

#define LC_GEAR_RATIO 3.636f
#define LC_TIRE_RADIUS_M 0.2f

#define LC_AIM_WHEEL_SPEED_CAN_ID 0x300
#define LC_DEBUG_CAN_ID 0x557

#define LC_SLIP_TARGET        0.10f
#define LC_CROSSOVER_SPEED_MS 2.0f
#define LC_EXIT_SPEED_MS      20.0f

/* Throttle thresholds */
#define LC_THROTTLE_TRIGGER 0.40f
#define LC_THROTTLE_RELEASE 0.05f

/* Closed-loop PID gains */
#define LC_KP 5000.0f
#define LC_KI 500.0f
#define LC_KD 330.0f

/* Open-loop slip rate cut */
#define LC_SLIP_RATE_MAX      3.0f
#define LC_SLIP_RATE_CUT_MULT 0.5f

/* Slip filter */
#define LC_SLIP_TAU_S        0.025f
#define LC_SLIP_FILTER_ALPHA 0.670320f

#define LC_SENSOR_TIMEOUT_MS 100
#define LC_DT_S 0.010f

/* Loop period bounds */
#define LC_DT_MIN_MS 1u
#define LC_DT_MAX_MS 100u
#define LC_DT_MIN_S (LC_DT_MIN_MS * 0.001f)
#define LC_DT_MAX_S (LC_DT_MAX_MS * 0.001f)

/* Conversions */
#define LC_RPM_TO_MS ((2.0f * 3.14159265f / 60.0f) * LC_TIRE_RADIUS_M / LC_GEAR_RATIO)
#define LC_MS_TO_RPM (LC_GEAR_RATIO * 60.0f / (2.0f * 3.14159265f * LC_TIRE_RADIUS_M))
#define LC_KMH10_TO_MS (1.0f / 36.0f)

typedef enum {
    LC_STATE_ARMED,
    LC_STATE_LAUNCHING_OPENLOOP,
    LC_STATE_LAUNCHING_CLOSEDLOOP
} lc_state_t;

typedef struct {
    lc_state_t state;
    float slip_ratio;       /* Filtered slip ratio */
    float slip_ratio_raw;   /* Unfiltered slip ratio */
    float slip_rate;        /* d(slip)/dt */
    float dt_s;             /* Loop period (s) */
    float pid_p_term;       /* Proportional term */
    float pid_i_term;       /* Integral term */
    float pid_d_term;       /* Derivative term */
    float pid_output;       /* Total PID output */
    int32_t lc_torque;      /* LC torque limit (0.1 Nm) */
    int32_t map_torque;     /* Map torque (0.1 Nm) */
    float vehicle_speed;    /* Front wheel speed (m/s) */
    float rear_wheel_speed; /* Motor wheel speed (m/s) */
    uint8_t sensor_healthy; /* Wheel speed CAN active */
    uint8_t slip_rate_cut;  /* Slip rate cut active */
} lc_debug_t;

void lc_init(void);

/* Returns commanded torque (0.1 Nm) */
int32_t lc_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined);

/* Wheel speed input (0.1 km/h) */
void lc_feed_wheel_speed(uint16_t front_left_speed_x10);

void lc_feed_tick(uint32_t tick_ms);
const lc_debug_t *lc_get_debug(void);
lc_state_t lc_get_state(void);
extern volatile uint8_t launch_control_enable;

#endif /* LAUNCH_CONTROL_H */
