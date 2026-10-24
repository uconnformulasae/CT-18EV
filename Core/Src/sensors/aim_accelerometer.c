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
		.sensor_status = make_sensor_status(),
		.accel_x_bias = 0,
		.accel_y_bias = 0,
		.accel_z_bias = 0,

		.accel_x_raw = 0,
		.accel_y_raw = 0,
		.accel_z_raw = 0
	};
}

void aim_accelerometer_calibrate(aim_accelerometer_t* self) {
	// TODO: Handle zero biasing while staged (need state machine?)
}

void aim_accelerometer_feed(aim_accelerometer_t* self, int32_t accel_x_raw, int32_t accel_y_raw, int32_t accel_z_raw) {
	self->accel_x_raw = accel_x_raw;
	self->accel_y_raw = accel_y_raw;
	self->accel_z_raw = accel_z_raw;
}

float aim_accelerometer_get_accel_x(aim_accelerometer_t* self) {
	return self->accel_x_raw * SENSOR_AIM_ACCEL_SCALE;
}
float aim_accelerometer_get_accel_y(aim_accelerometer_t* self) {
	return self->accel_y_raw * SENSOR_AIM_ACCEL_SCALE;
}

float aim_accelerometer_get_accel_z(aim_accelerometer_t* self) {
	return self->accel_z_raw * SENSOR_AIM_ACCEL_SCALE;
}
