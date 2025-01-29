/*
 * parameters.h
 *
 *  Created on: Jan. 29, 2025
 *      Author: jeffr
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// Limits
constexpr float AIRSPEED_CRUISE = 20; // Meters per second, not knots!
constexpr float TRIM_THROTTLE = 0.3; // Do I even need this? I can use integrator?
constexpr float PTCH_LIM_MAX_DEG = 10;
constexpr float PTCH_LIM_MIN_DEG = -10;
constexpr float ROLL_LIM_DEG = 10;

#endif /* PARAMETERS_H_ */
