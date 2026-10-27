/*
 * conversion_factors.h
 *
 *  Created on: Oct 24, 2026
 *      Author: Madelyn
 */

#ifndef INC_SENSORS_CAR_CONSTANTS_H_
#define INC_SENSORS_CAR_CONSTANTS_H_

#define GEAR_RATIO (40.0f/11.0f)
#define TIRE_RADIUS_M (8.1f*0.0254f)
#define TIRE_DIAMETER_M (16.2f*0.0254f)
#define TIRE_CIRCUMFERENCE (2.0f*M_PI*TIRE_RADIUS_M)
#define RPM_TO_MPS (TIRE_CIRCUMFERENCE/(GEAR_RATIO * 60.0f))

// rough estimates
#define TOP_SPEED_MPS 36
#define TOP_ACCEL_MPS2 15

#endif /* INC_SENSORS_CAR_CONSTANTS_H_ */
