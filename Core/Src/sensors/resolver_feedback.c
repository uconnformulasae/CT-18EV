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
		.sensor_status = make_sensor_status(),
		.motor_speed = 0,
		.wheel_speed = 0
	};
}

void resolver_feedback_feed(resolver_feedback_t* self, int32_t raw_motor_speed) {
	self->motor_speed = raw_motor_speed;
}

float resolver_feedback_speed(resolver_feedback_t* self) {
	return self->motor_speed * RPM_TO_MPS;
}
