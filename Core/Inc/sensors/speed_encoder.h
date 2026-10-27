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

#define SECONDS_PER_MICROSECOND 0.000001f
#define MICROSECONDS_PER_SECOND 1000000

#define SPEED_ENCODER_PPR 36 // TODO: Update when tone ring is machined
#define SPEED_ENCODER_TIMEOUT_PERIODS 3

typedef struct speed_encoder_faults {
	volatile uint8_t impossible_accel_count;
	volatile uint8_t impossible_velo_count;
} speed_encoder_faults_t;

typedef struct speed_encoder {
	volatile sensor_status_t sensor_status;
	speed_encoder_faults_t sensor_faults;
	volatile uint32_t edges_seen;			// number of edges seen
	volatile uint32_t last_edge_us;			// timestamp of last received edge from microsecond timer, used for period measurements (us)
	volatile uint32_t last_edge_ms;			// timestamp of last received from SysTick, used for timeouts (ms)
	volatile uint32_t period_measurement_us;
} speed_encoder_t;

void speed_encoder_init(speed_encoder_t* self);
void speed_encoder_feed(speed_encoder_t* self, uint32_t current_edge_us);
void speed_encoder_upkeep(speed_encoder_t* self, uint32_t current_tick_ms);
float speed_encoder_speed(speed_encoder_t* self);

#endif /* INC_WSS_H_ */
