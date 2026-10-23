/*
 * sensor_status.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_SENSORS_SENSOR_STATUS_H_
#define INC_SENSORS_SENSOR_STATUS_H_

#include <stdbool.h>

typedef struct sensor_status {
	volatile bool sensor_online;
	volatile bool sensor_usable;
	volatile uint32_t last_rx_ms;
} sensor_status_t;

sensor_status_t make_sensor_status();

void update_sensor_online(sensor_status_t* self, uint32_t current_time_ms, uint32_t timeout_threshold_ms);
void update_sensor_usable(sensor_status_t* self, bool is_usable);

#endif /* INC_SENSORS_SENSOR_STATUS_H_ */
