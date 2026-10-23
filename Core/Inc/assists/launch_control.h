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

#define LC_OPENLOOP_TESTING
#define LC_OPENLOOP_STANDS

#define LC_OPENLOOP_ABORT_RAMP_NMPS 1500.0f
#define LC_OPENLOOP_TESTING_TIMEOUT_MS 700

#ifndef LC_OPENLOOP_STANDS

// Rough guesses based on CT17 Endurance sim config
#define LC_LASH_TAKEUP_TIME_MS 30 			// somewhat long, safe to decrease as long as lash impact doesn't show up on logs
#define LC_OPENLOOP_TAKEUP_TORQUE_NM 10.0f 	// estimate, drag torque + 5Nm
#define LC_OPENLOOP_KNEE_TORQUE_NM 40.0f 	// step up to ~90 for testing
#define LC_OPENLOOP_HOLD_TORQUE_NM 70.0f 	// step up to ~165 for testing (for mu=1.45, a=0.9g; NOTE: check tune, might be power limited)

#define LC_OPENLOOP_TESTING_MAX_RPM 5000

#else

#define LC_LASH_TAKEUP_TIME_MS 30
#define LC_OPENLOOP_TAKEUP_TORQUE_NM 10.0f
#define LC_OPENLOOP_KNEE_TORQUE_NM 18.0f
#define LC_OPENLOOP_HOLD_TORQUE_NM 33.0f

#define LC_OPENLOOP_TESTING_MAX_RPM 2500

#endif

#define LC_DRIVELINE_RAMP_RATE_NMPS 600 	// increase rate to as fast as driveline will allow without ringing (check Iq, rpm in 200hz logs)
#define LC_TRACTION_RAMP_RATE_NMPS 300 		// okay to increase, stay comfortably below traction limit (don't spin out wheels even on cold tires/dirty surface)

typedef enum {
	LC_STATE_IDLE,
    LC_STATE_ARMED,
    LC_STATE_LAUNCHING_OPENLOOP,
    LC_STATE_LAUNCHING_CLOSEDLOOP,
	LC_STATE_DONE,
	LC_STATE_FAULT
} lc_state_t;

typedef enum {
	LC_EXIT_DRIVER_PRESSED_BRAKE,
	LC_EXIT_DRIVER_RELEASED_THROTTLE,
	LC_EXIT_INVERTER_LOST,
	LC_EXIT_WSS_LOST,
	LC_EXIT_HIT_POWER_LIMIT,
	LC_EXIT_HIT_TIME_LIMIT
} lc_exit_reason_t;

typedef struct lc_pid {
	float slip_ratio_raw;		// raw slip ratio
	float slip_ratio_filtered;	// low pass filtered slip ratio

	float p;					// pid proportional term
	float i;					// pid integral term
	float d;					// pid derivative term

	float pid_torque;			// closed loop output
} lc_pid_t;

typedef struct lc_ff {
	uint32_t tick_start;		// tick at which active launch control began (ms)
	uint32_t tick_current;		// current tick (ms)

	float ff_torque;			// open loop output torque
} lc_ff_t;

typedef struct {
    lc_state_t state;

    lc_ff_t ff;
    lc_pid_t pid;
} lc_data_t;

void lc_init(lc_data_t* data);

/* Returns commanded torque (0.1 Nm) */
int32_t lc_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined);

/* Wheel speed input (0.1 km/h) */
void lc_feed_wheel_speed(uint16_t front_left_speed_x10);

void lc_feed_tick(uint32_t tick_ms);
const lc_data_t *lc_get_debug(void);
lc_state_t lc_get_state(void);
extern volatile uint8_t launch_control_enable;

#endif /* LAUNCH_CONTROL_H */
