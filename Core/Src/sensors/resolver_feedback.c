/*
 * resolver_feedback.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#include "resolver_feedback.h"
#include "math.h"

#define GEAR_RATIO (40.0f/11.0f)
#define TIRE_RADIUS_M (8.1f*0.0254f)
#define RPM_TO_MPS (2.0f * M_PI * TIRE_RADIUS_M)/(GEAR_RATIO * 60.0f)

void init_resolver_feedback(resolver_feedback_t* self) {
	*self = (resolver_feedback_t){
		.sensor_status = make_sensor_status(),
		.motor_speed = 0,
		.wheel_speed = 0
	};
}

void feed_resolver_feedback(resolver_feedback_t* self, int32_t raw_motor_speed) {
	self->motor_speed = raw_motor_speed;
}

float get_rear_wheel_speed(resolver_feedback_t* self) {
	return self->motor_speed * RPM_TO_MPS;
}
