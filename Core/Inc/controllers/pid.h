/*
 * pid.h
 *
 *  Created on: Oct 29, 2026
 *      Author: Madelyn
 */

#ifndef INC_CONTROLLERS_PID_H_
#define INC_CONTROLLERS_PID_H_

#include <stdbool.h>

typedef struct pid_controller {
	float kp;
	float ki;
	float kd;

	float setpoint;
	float saturation_low;
	float saturation_high;
	float derivative_tau;

	float p_term;
	float i_term;
	float d_term;
	float output;

	float measurement;
	float error;

	bool has_previous;
	float last_measurement;
	float last_error;
	float last_derivative;
} pid_controller_t;

void pid_set(pid_controller_t* self, float setpoint);
float pid_update(pid_controller_t* self, float measurement, float dt);

#endif /* INC_CONTROLLERS_PID_H_ */
