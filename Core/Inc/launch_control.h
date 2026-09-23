/**
 * @file    launch_control.h
 * @brief   Launch control system for FSAE EV acceleration event
 */

#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include <stdint.h>

/* Overall reduction: motor RPM / wheel RPM, so greater than 1. The reciprocal
 * round-trips through LC_MS_TO_RPM just as well, so only the physical check in
 * test_conversions pins the direction of the conversion. */
#define LC_GEAR_RATIO 3.636f

// tire radius in meters
#define LC_TIRE_RADIUS_M 0.2f

/* CAN ID for the AiM front wheel speed broadcast. The car carries one front
 * wheel speed sensor, on the left, and its reading is taken as vehicle speed. */
#define LC_AIM_WHEEL_SPEED_CAN_ID 0x300

/* Reserved: main.c no longer transmits this frame. To restore it, add an
 * lc_pack_debug() here mirroring soc_kf_pack_state(). */
#define LC_DEBUG_CAN_ID 0x557

#define LC_SLIP_TARGET        0.10f
#define LC_CROSSOVER_SPEED_MS 2.0f
#define LC_EXIT_SPEED_MS      20.0f

// Minimum throttle position to trigger launch. Driver gotta be rlly bout it
#define LC_THROTTLE_TRIGGER 0.40f

// Below this the driver has lifted: ends a launch, and clears the exit latch.
#define LC_THROTTLE_RELEASE 0.05f

#define LC_KP 5000.0f
#define LC_KI 500.0f

/* Derivative gain, acting on slip rate. At 3 /s it contributes about
 * -1000 x10Nm, roughly half a typical request. A starting point, not a tuned
 * value: this needs track data. */
#define LC_KD 330.0f

/* Hard slip-rate cut, applied in open loop only, where the torque map is the
 * primary limiter and nothing else responds to slip. Closed loop uses LC_KD,
 * which covers the same job continuously. */
#define LC_SLIP_RATE_MAX      3.0f
#define LC_SLIP_RATE_CUT_MULT 0.5f

/* Slip filter time constant, chosen from the signal rather than from a target
 * weight: the AiM speed is quantised to 0.1 km/h and LC_KD differentiates it,
 * so it needs smoothing across several passes. 25 ms costs about two passes of
 * lag. */
#define LC_SLIP_TAU_S 0.025f

// What the weight reduces to at the nominal period: expf(-LC_DT_S / LC_SLIP_TAU_S).
#define LC_SLIP_FILTER_ALPHA 0.670320f

// AiM front wheel speed CAN message timeout (ms).
#define LC_SENSOR_TIMEOUT_MS 100

// Nominal loop period. Fallback for the first update after a reset; later
// passes use the measured tick delta.
#define LC_DT_S 0.010f

/* Bounds on the measured period: no divide by zero, no huge integral step
 * after a stall. Whole milliseconds are the source of truth, because the tick
 * is in milliseconds and LC_DT_MAX_MS also sizes the slip filter weight cache. */
#define LC_DT_MIN_MS 1u
#define LC_DT_MAX_MS 100u
#define LC_DT_MIN_S (LC_DT_MIN_MS * 0.001f)
#define LC_DT_MAX_S (LC_DT_MAX_MS * 0.001f)

// Precomputed conversion: RPM → m/s = RPM * (2π/60) * tire_radius / gear_ratio
#define LC_RPM_TO_MS ((2.0f * 3.14159265f / 60.0f) * LC_TIRE_RADIUS_M / LC_GEAR_RATIO)
// Precomputed inverse: m/s → RPM
#define LC_MS_TO_RPM (LC_GEAR_RATIO * 60.0f / (2.0f * 3.14159265f * LC_TIRE_RADIUS_M))
// Precomputed: km/h×10 → m/s = val / 36.0
#define LC_KMH10_TO_MS (1.0f / 36.0f)

typedef enum {
    /* Resting state: torque passes through, controller held clear, waiting
     * for the throttle trigger. lc_update() runs only while LC is enabled,
     * so this state is always armed. */
    LC_STATE_ARMED,
    LC_STATE_LAUNCHING_OPENLOOP,
    LC_STATE_LAUNCHING_CLOSEDLOOP
} lc_state_t;

typedef struct {
    lc_state_t state;
    float slip_ratio;       // Current filtered slip ratio
    float slip_ratio_raw;   // Unfiltered slip ratio
    float slip_rate;        // dλ/dt (slip rate of change)
    float dt_s;             // Measured loop period used this pass (s)
    float pid_p_term;       // Proportional term output
    float pid_i_term;       // Integral term (accumulated)
    float pid_d_term;       // Derivative term, acting on slip rate
    float pid_output;       // Combined PID output (correction)
    int32_t lc_torque;      // Torque limit from LC (×10 units)
    int32_t map_torque;     // Open-loop map torque (×10 units)
    float vehicle_speed;    // Estimated vehicle speed (m/s)
    float rear_wheel_speed; // Rear wheel speed from motor (m/s)
    uint8_t sensor_healthy; // 1 = front wheel speed CAN is live
    uint8_t slip_rate_cut;  // 1 = emergency slip rate cut active
} lc_debug_t;

void lc_init(void);

/* Returns torque to command, x10 Nm. Only reduces positive drive torque; a
 * negative (regen) request passes through. */
int32_t lc_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined);
/* Front wheel speed from the AiM broadcast, in km/h x10. One sensor, on the
 * left front; its reading is used directly as vehicle speed. */
void lc_feed_wheel_speed(uint16_t front_left_speed_x10);
void lc_feed_tick(uint32_t tick_ms);
const lc_debug_t *lc_get_debug(void);
lc_state_t lc_get_state(void);
extern volatile uint8_t launch_control_enable;

#endif /* LAUNCH_CONTROL_H */
