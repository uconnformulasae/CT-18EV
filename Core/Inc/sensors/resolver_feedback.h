/*
 * inv.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_INV_H_
#define INC_INV_H_

#include "sensors/sensor_status.h"

typedef struct resolver_feedback {
	sensor_status_t sensor_status; 	// sensor validity

	float motor_speed;				// motor rpm from inverter broadcast (rpm)
	float wheel_speed;				// rear wheel speed calculated from inverter speed broadcast (m/s)
} resolver_feedback_t;

#endif /* INC_INV_H_ */
