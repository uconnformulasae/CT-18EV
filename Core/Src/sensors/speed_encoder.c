/*
 * speed_encoder.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */


#include "sensors/speed_encoder.h"
#include <math.h>
#include <sensors/car_constants.h>

void speed_encoder_init(speed_encoder_t* self) {
	*self = (speed_encoder_t){
		.sensor_status = make_sensor_status(),
		.edges_seen = 0,
		.edge_microseconds = 0,

		.wheel_speed = 0
	};
}

void speed_encoder_feed(speed_encoder_t* self, uint32_t current_edge_us, uint32_t current_tick_ms) {
	self->edges_seen += 1;
	uint32_t candidate_period_us = current_edge_us - self->last_edge_us;

	if (candidate_period_us < SPEED_ENCODER_IMPOSSIBLE_VELO_PERIOD_US) {
		self->sensor_faults.impossible_velo_count += 1;
		return;
	}

	if((candidate_period_us - self->period_measurement_us) < SPEED_ENCODER_IMPOSSIBLE_ACCEL_PERIOD_DELTA_US) {
		self->sensor_faults.impossible_accel_count += 1;
		return;
	}

	self->period_measurement_us = current_edge_us - self->last_edge_us;
	self->last_edge_us = current_edge_us;
	self->last_edge_ms = current_tick_ms;
}

void speed_encoder_upkeep(speed_encoder_t* self, uint32_t current_tick_ms) {
	// mark sensor dead if we haven't received an update in SPEED_ENCODER_TIMEOUT_PERIODS periods
	uint32_t timeout_ms = self->period_measurement_us/1000 * SPEED_ENCODER_TIMEOUT_PERIODS;
	uint32_t diff_ms = current_tick_ms - self->last_edge_ms;

	if(diff_ms >= timeout_ms) {
		self->sensor_status = SENSOR_OFFLINE;
	}
}

float speed_encoder_speed(speed_encoder_t* self) {
	if (self->sensor_status == SENSOR_ONLINE) {
		float rpm = 60.0f/(SPEED_ENCODER_PPR * self->period_measurement_us * SECONDS_PER_MICROSECOND);
		return rpm * RPM_TO_MPS;
	} else {
		return 0.0f;
	}

}
