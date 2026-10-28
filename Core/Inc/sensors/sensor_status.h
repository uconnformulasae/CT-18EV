/*
 * sensor_status.h
 *
 *  Created on: Oct 24, 2026
 *      Author: Madelyn
 */

#ifndef INC_SENSORS_SENSOR_STATUS_H_
#define INC_SENSORS_SENSOR_STATUS_H_

typedef enum sensor_status {
	SENSOR_OKAY,
	SENSOR_DEGRADED,
	SENSOR_FAULTY,
	SENSOR_DEAD
} sensor_status_t;

#endif /* INC_SENSORS_SENSOR_STATUS_H_ */
