/*
 * parameters.h
 *
 *  Created on: Jan. 29, 2025
 *      Author: jeffr
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// Airspeed
constexpr float AIRSPEED_CRUISE = 20; // Meters per second

// Throttle
constexpr float TRIM_THROTTLE = 0.3; // Steady-state cruise throttle
constexpr float THR_MIN = 0; // Maximum throttle between 0 and 1
constexpr float THR_MAX = 1; // Minimum throttle between 0 and 1

// Attitude
constexpr float PTCH_LIM_MAX_DEG = 10;
constexpr float PTCH_LIM_MIN_DEG = -10;
constexpr float ROLL_LIM_DEG = 10;
constexpr bool RUDDER_ONLY = true;

// Takeoff
constexpr float TAKEOFF_ALT = 10; // Altitude that the plane will climb to during takeoff meters per second
constexpr float TAKEOFF_THR = 1; // Throttle set during takeoff between 0 and 1
constexpr float TAKEOFF_AIRSPD = 10; // Rotation speed meters per second during takeoff, ignored during hand launch

#endif /* PARAMETERS_H_ */
