/*
 * parameters.h
 *
 *  Created on: Jan. 29, 2025
 *      Author: jeffr
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// Airspeed
constexpr float AIRSPEED_CRUISE = 15; // Meters per second
constexpr float AIRSPEED_LANDING = 12;

// Throttle
constexpr float TRIM_THROTTLE = 0.1; // Steady-state cruise throttle
constexpr float THR_MIN = 0; // Maximum throttle between 0 and 1
constexpr float THR_MAX = 1; // Minimum throttle between 0 and 1
constexpr float THR_DEADZONE = 0; // When throttle is set below this value, it is set to 0. This is mandatory for transitioning from boot to flight.

// Attitude
constexpr float PTCH_LIM_MAX_DEG = 15;
constexpr float PTCH_LIM_MIN_DEG = -15;
constexpr float ROLL_LIM_DEG = 20; // Maximum roll angle in either direction
constexpr bool RUDDER_ONLY = false;

// Autoland
constexpr float LAND_GS_DEG = 10; // Landing glideslope angle
constexpr float LAND_FLARE_ALT = 25; // Flare altitude
constexpr float FLARE_SINK_RATE = 1;

// Takeoff
constexpr float TAKEOFF_ALT = 10; // Altitude that the plane will climb to during takeoff meters per second
constexpr float TAKEOFF_THR = 1; // Throttle set during takeoff between 0 and 1
constexpr float TAKEOFF_AIRSPD = 10; // Rotation speed meters per second during takeoff, ignored during hand launch
constexpr float TAKEOFF_PTCH = 10; // Pitch during takeoff

// Guidance
constexpr float MIN_DIST_WP = 50; // Distance in meters from waypoint until switching to next, "radius of acceptance"
constexpr uint8_t MAX_NUM_WPTS = 100; // Maximum number of waypoints

// Radio
constexpr uint8_t TELEM_PKT_LEN = 40;

#endif /* PARAMETERS_H_ */
