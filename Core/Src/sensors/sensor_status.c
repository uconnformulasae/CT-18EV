/*
 * sensor_status.c
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#include "sensors/sensor_status.h"

sensor_status_t make_sensor_status() {
	return (sensor_status_t){
		.sensor_online = false,
		.sensor_usable = false,
		.last_rx_ms = 0
	};
}

void update_sensor_online(sensor_status_t* self, uint32_t current_time_ms, uint32_t timeout_threshold_ms) {
	uint32_t silent_time_ms = current_time_ms - last_rx_ms;

	if (silent_time_ms >= timeout_threshold) {
		self->sensor_online = false;
	}
}

void update_sensor_usable(sensor_status_t* self, bool is_usable) {
	self->sensor_usable = is_usable;
}
