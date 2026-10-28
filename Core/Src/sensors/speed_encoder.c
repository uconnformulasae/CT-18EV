/*
 * speed_encoder.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */


#include "sensors/speed_encoder.h"
#include <math.h>
#include "sensors/car_constants.h"

#define SECONDS_PER_MICROSECOND 0.000001f

void speed_encoder_init(speed_encoder_t* self) {
	*self = (speed_encoder_t){
		.encoder_count_raw = 0,
		.period_us_raw = 0,
		.edge_us_raw = 0,
		.data_ready = false,

		.front_velocity = 0
	};
}
void speed_encoder_feed(speed_encoder_t* self, uint32_t edge_us_raw) {
	self->encoder_count_raw += 1;
	self->period_us_raw = edge_us_raw - self->edge_us_raw;
	self->edge_us_raw = edge_us_raw;
	self->data_ready = true;
}

void speed_encoder_upkeep(speed_encoder_t* self) {
	if(!self->data_ready) return;
	self->data_ready = false;

	// TODO: sensor sanity checks

	float rpm = 60.0f/(SPEED_ENCODER_PPR * self->period_us_raw * SECONDS_PER_MICROSECOND);
	self->front_velocity_mps = rpm * WHEEL_RPM_TO_MPS;
}
