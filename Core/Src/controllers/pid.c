/*
 * pid.c
 *
 *  Created on: Oct 29, 2026
 *      Author: Madelyn
 */

#include "controllers/pid.h"

void pid_set(pid_controller_t* self, float setpoint) {
	self->setpoint = setpoint;
}

void pid_reset(pid_controller_t* self) {
	self->p = 0;
	self->i = 0;
	self->d = 0;

	self->last_measurement = 0;
	self->last_error = 0;

	self->has_previous = 0;
}

float pid_update(pid_controller_t* self, float measurement, float dt) {
	if (dt <= 0.0f) {
		// hold the previous output if no time has elapsed
		return self->output;
	}

	float error = self->setpoint - measurement;

	float p_term = self->kp * error;
	float i_term = self->i + self->ki * error * dt;

	// lowpass filtered derivative (s)/(tau*s+1) discretized under backwards-euler
	float a = self->derivative_tau/(self->derivative_tau + dt);
	float b = 1/(self->derivative_tau + dt);
	float raw_derivative = a * self->d + b * (measurement - self->last_measurement);
	float d_term = self->has_previous ? self->kd * raw_derivative : 0;

	float candidate_output = p_term + i_term + d_term;

	self->p_term = p_term;

	// conditional integration, only integrate if it does not push the controller further into saturation
	if (candidate_output > self->saturation_high) {
		if (error < 0.0f) {
			self->i_term = i_term;
		}
	} else if (output < self->saturation_low) {
		if (error > 0.0f) {
			self->i_term = i_term;
		}
	} else {
		self->i_term = i_term;
	}

	self->d_term = d_term;

	// store measurement and error values for debugging
	self->measurement = measurement;
	self->error = error;

	// store previous measurement and error values for derivative computation
	self->last_measurement = measurement;
	self->last_error = error;
	self->last_derivative = raw_derivative;
	self->has_previous = true;

	// clamp and commit the final output
	self->output = fmaxf(fminf(self->p + self->i + self->d, pid->saturation_high), pid->saturation_low);

	return self->output;
}
