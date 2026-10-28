/*
 * resolver_feedback.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_INV_H_
#define INC_INV_H_

#include <stdbool.h>

typedef struct resolver_feedback {
	volatile int32_t motor_speed_raw;	// motor rpm from inverter resolver broadcast (rpm)
	volatile bool data_ready;			// fresh unread data flag

	int32_t motor_speed;
	float rear_velocity;
} resolver_feedback_t;

void resolver_feedback_init(resolver_feedback_t* self);
void resolver_feedback_feed(resolver_feedback_t* self, int32_t raw_motor_speed);
void resolver_feedback_upkeep(resolver_feedback_t* self);

float resolver_feedback_speed(resolver_feedback_t* self);

#endif /* INC_INV_H_ */
