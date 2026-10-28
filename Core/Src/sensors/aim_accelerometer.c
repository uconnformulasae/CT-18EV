/*
 * aim_accelerometer.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */


#include "aim_accelerometer.h"

// TODO: Calibrate reference biases on level ground in shop

void aim_accelerometer_init(aim_accelerometer_t* self) {
	*self = (aim_accelerometer_t){
		.accel_x_raw = 0,
		.accel_y_raw = 0,
		.accel_z_raw = 0,
		.data_ready = false,

		.accel_x_bias = 0,
		.accel_y_bias = 0,
		.accel_z_bias = 0,

		.accel_x = 0,
		.accel_y = 0,
		.accel_z = 0
	};
}

void aim_accelerometer_calibrate(aim_accelerometer_t* self) {
	// TODO: Handle zero biasing while staged (need state machine?)
}

void aim_accelerometer_feed(aim_accelerometer_t* self, int32_t accel_x_raw, int32_t accel_y_raw, int32_t accel_z_raw) {
	self->accel_x_raw = accel_x_raw;
	self->accel_y_raw = accel_y_raw;
	self->accel_z_raw = accel_z_raw;
	self->data_ready = true;
}

void aim_accelerometer_upkeep(aim_accelerometer_t* self) {
	if(!data_ready) return;
	self->data_ready = false;

	// TODO: Sensor sanity checks

	self->accel_x = (self->accel_x_raw - self->accel_x_bias) / SENSOR_AIM_ACCEL_SCALE;
	self->accel_y = (self->accel_y_raw - self->accel_y_bias) / SENSOR_AIM_ACCEL_SCALE;
	self->accel_z = (self->accel_z_raw - self->accel_z_bias) / SENSOR_AIM_ACCEL_SCALE;
}
