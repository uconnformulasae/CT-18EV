/*
 * imu.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "sensors/sensor_status.h"

typedef struct aim_accelerometer {
	sensor_status_t sensor_status;	// sensor validity
	int32_t accel_bias;				// zero-bias at standstill

	float longitudinal_accel;		// longitudinal acceleration reading (m/s^2)
} aim_accelerometer_t;

#endif /* INC_IMU_H_ */
