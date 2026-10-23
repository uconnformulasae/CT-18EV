/*
 * health.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_SENSORS_SENSOR_STATUS_H_
#define INC_SENSORS_SENSOR_STATUS_H_

#include <stdbool.h>

typedef struct sensor_status {
	bool sensor_online;
	bool sensor_usable;
	uint32_t last_rx_ms;
} sensor_status_t;

#endif /* INC_SENSORS_SENSOR_STATUS_H_ */
