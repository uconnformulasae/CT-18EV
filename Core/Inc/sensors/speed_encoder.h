/*
 * speed_encoder.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_WSS_H_
#define INC_WSS_H_

#include "sensors/sensor_status.h"

#define SPEED_ENCODER_PPR 36 // TODO: Update when tone ring is machined

typedef struct speed_encoder {
	sensor_status_t sensor_status; 	// sensor validity
	volatile uint16_t edges_seen;	// number of edges seen
	volatile uint32_t last_edge_us;	// timestamp of last received edge (us)
	volatile uint32_t period_measurement_us;
} speed_encoder_t;

void init_speed_encoder(speed_encoder_t* self);
void feed_speed_encoder(speed_encoder_t* self, uint32_t current_edge_us);
float speed_encoder_get_front_wheel_speed(speed_encoder_t* self);

#endif /* INC_WSS_H_ */
