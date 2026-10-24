/*
 * conversion_factors.h
 *
 *  Created on: Oct 24, 2026
 *      Author: Madelyn
 */

#ifndef INC_SENSORS_CONVERSION_FACTORS_H_
#define INC_SENSORS_CONVERSION_FACTORS_H_

#define GEAR_RATIO (40.0f/11.0f)
#define TIRE_RADIUS_M (8.1f*0.0254f)
#define RPM_TO_MPS (2.0f * M_PI * TIRE_RADIUS_M)/(GEAR_RATIO * 60.0f)

#endif /* INC_SENSORS_CONVERSION_FACTORS_H_ */
