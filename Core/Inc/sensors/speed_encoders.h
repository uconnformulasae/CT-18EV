/*
 * wss.h
 *
 *  Created on: Oct 23, 2026
 *      Author: Madelyn
 */

#ifndef INC_WSS_H_
#define INC_WSS_H_

#include "sensors/sensor_status.h"

typedef struct speed_encoders {
	sensor_status_t sensor_status; 	// sensor validity
	uint16_t edges_seen;			// number of edges seen
	uint32_t edge_microseconds;		// timestamp of last received edge (us)

	float wheel_speed;				// computed wheel speed from encoder edge period (m/s)
} speed_encoders_t;



#endif /* INC_WSS_H_ */
