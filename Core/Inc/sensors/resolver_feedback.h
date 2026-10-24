/*
 * resolver_feedback.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_INV_H_
#define INC_INV_H_

#include "sensors/sensor_status.h"

typedef struct resolver_feedback {
	sensor_status_t sensor_status; 	// sensor validity
	volatile int32_t motor_speed;	// motor rpm from inverter resolver broadcast (rpm)
} resolver_feedback_t;

void resolver_feedback_init(resolver_feedback_t* self);
void resolver_feedback_feed(resolver_feedback_t* self, int32_t raw_motor_speed);

float resolver_feedback_speed(resolver_feedback_t* self);

#endif /* INC_INV_H_ */
