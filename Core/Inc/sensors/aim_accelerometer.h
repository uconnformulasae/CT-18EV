/*
 * aim_accelerometer.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "sensors/sensor_status.h"

#define SENSOR_AIM_ACCEL_CAN_ID 0x301
#define SENSOR_AIM_ACCEL_SCALE 100.0f

typedef struct aim_accelerometer {
	sensor_status_t sensor_status;	// sensor validity
	int32_t accel_x_bias;			// longitudinal zero-bias at standstill
	int32_t accel_y_bias;			// lateral zero-bias at standstill
	int32_t accel_z_bias;			// vertical zero-bias at standstill

	volatile int32_t raw_accel_x;	// longitudinal acceleration reading (0.01 * m/s^2)
	volatile int32_t raw_accel_y;	// lateral acceleration reading (0.01 * m/s^2)
	volatile int32_t raw_accel_z;	// vertical acceleration reading (0.01 * m/s^2)
} aim_accelerometer_t;

void aim_accelerometer_init(aim_accelerometer_t* self);
void aim_accelerometer_calibrate(aim_accelerometer_t* self);
void aim_accelerometer_feed(aim_accelerometer_t* self, int32_t accel_x_raw, int32_t accel_y_raw, int32_t accel_z_raw);

float aim_accelerometer_get_accel_x(aim_accelerometer_t* self);
float aim_accelerometer_get_accel_y(aim_accelerometer_t* self);
float aim_accelerometer_get_accel_z(aim_accelerometer_t* self);

#endif /* INC_IMU_H_ */
