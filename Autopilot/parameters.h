#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// Need to move parameters to HAL because it changes depending on Flight or Pitl

// Airspeed
constexpr float AIRSPEED_CRUISE = 18; // Meters per second
constexpr float AIRSPEED_LANDING = 15;

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
constexpr float LAND_GS_DEG = 6; // Landing glideslope angle
constexpr float LAND_FLARE_ALT = 3; // Flare altitude
constexpr float FLARE_SINK_RATE = 0.3;
constexpr float FLARE_PITCH_MAX_DEG = 10; // Maximum pitch during flare

// Takeoff
constexpr float TAKEOFF_ALT = 5; // Altitude that the plane will climb to during takeoff meters per second
constexpr float TAKEOFF_THR = 1; // Throttle set during takeoff between 0 and 1
constexpr float TAKEOFF_AIRSPD = 10; // Rotation speed meters per second during takeoff, ignored during hand launch
constexpr float TAKEOFF_PTCH = 10; // Pitch during takeoff

// Guidance
constexpr float MIN_DIST_WP = 50; // Distance in meters from waypoint until switching to next, "radius of acceptance"
constexpr uint8_t MAX_NUM_WPTS = 100; // Maximum number of waypoints

// Radio
constexpr uint8_t TELEM_PKT_LEN = 40;

// AHRS
constexpr float AHRS_FUSION_ACC_MIN = 0.5; // Minimum acceleration in inertial frame (g) for accelerometer fusion
constexpr float AHRS_FUSION_ACC_MAX = 2; // Maximum acceleration in gs for accelerometer fusion
constexpr float MAG_DECL = -10.2; // Degrees, determined from online calculator
constexpr float HARD_IRON[3] = {-46.301146, 3.866545, -71.601346};
constexpr float SOFT_IRON[3][3] = {{1.189985, 0.015110, -0.066520},
						  	  	   {0.015110, 1.205787, -0.039344},
								   {-0.066520, -0.039344, 1.183604}};

// Servos
constexpr uint16_t ELEVATOR_MIN_DUTY = 1000; // Duty cycle in us
constexpr uint16_t ELEVATOR_MAX_DUTY = 2000;
constexpr uint16_t AILERON_MIN_DUTY = 1000;
constexpr uint16_t AILERON_MAX_DUTY = 2000;
constexpr uint16_t THROTTLE_MIN_DUTY = 1000;
constexpr uint16_t THROTTLE_MAX_DUTY = 2000;

// Total energy control system (TECS)
constexpr float TECS_MASS_KG = 0.4; // Mass of the plane in Kg

#endif /* PARAMETERS_H_ */
