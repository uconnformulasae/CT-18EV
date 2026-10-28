/*
 * resolver_feedback.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#include <sensors/car_constants.h>
#include "sensors/resolver_feedback.h"
#include "math.h"

void resolver_feedback_init(resolver_feedback_t* self) {
	*self = (resolver_feedback_t){
		.motor_speed_raw = 0,
		.data_ready = false,

		.motor_speed = 0,
		.rear_velocity = 0
	};
}

void resolver_feedback_feed(resolver_feedback_t* self, int32_t raw_motor_speed) {
	self->motor_speed = raw_motor_speed;
	self->data_ready = true;
}

void resolver_feedback_upkeep(resolver_feedback_t* self) {
	if(!self->data_ready) return;
	self->data_ready = false;

	// TODO: sensor sanity checks
	self->motor_speed = self->motor_speed_raw;
	self->rear_velocity = self->motor_speed * MOTOR_RPM_TO_MPS;
}

float resolver_feedback_speed(resolver_feedback_t* self) {
	return self->motor_speed * MOTOR_RPM_TO_MPS;
}
