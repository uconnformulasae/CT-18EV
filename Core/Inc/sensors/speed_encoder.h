/*
 * speed_encoder.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_WSS_H_
#define INC_WSS_H_

#include <stdbool.h>
#include <math.h>
#include "sensors/sensor_status.h"

typedef struct speed_encoder {
	volatile uint32_t encoder_count_raw;
	volatile uint32_t period_us_raw;
	volatile uint32_t edge_us_raw;
	volatile bool data_ready;

	float front_velocity;
} speed_encoder_t;

void speed_encoder_init(speed_encoder_t* self);
void speed_encoder_feed(speed_encoder_t* self, uint32_t edge_us);
void speed_encoder_upkeep(speed_encoder_t* self);


#endif /* INC_WSS_H_ */
