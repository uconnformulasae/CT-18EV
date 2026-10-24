/*
 * speed_encoder.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */


#include "sensors/speed_encoder.h"
#include "sensors/conversion_factors.h"
#include <math.h>

#define SECONDS_PER_MICROSECOND 0.000001f

void speed_encoder_init(speed_encoder_t* self) {
	*self = (speed_encoder_t){
		.sensor_status = make_sensor_status(),
		.edges_seen = 0,
		.edge_microseconds = 0,

		.wheel_speed = 0
	};
}

void speed_encoder_feed(speed_encoder_t* self, uint32_t current_edge_us) {
	self->edges_seen += 1;
	self->period_measurement_us = current_edge_us - self->last_edge_us;
	self->last_edge_us = current_edge_us;
}

float speed_encoder_speed(speed_encoder_t* self) {
	float rpm = 60.0f/(SPEED_ENCODER_PPR * self->period_measurement_us * SECONDS_PER_MICROSECOND);
	return rpm * RPM_TO_MPS;
}
