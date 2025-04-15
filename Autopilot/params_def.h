// Takeoff
PARAM(TKO_ALT, ParamFloat, 10.0f)       // Altitude to detect when takeoff is complete, m
PARAM(TKO_PTCH, ParamFloat, 15.0f)      // Constant pitch hold during takeoff, deg

// Mission
PARAM(MIS_THR, ParamFloat, 0.7f)        // Steady-state cruise throttle
PARAM(MIS_SPD, ParamFloat, 15.0f)       // Cruise speed, m/s
PARAM(MIN_SPD, ParamFloat, 10.0f)       // Minimum allowed airspeed, m/s
PARAM(MAX_SPD, ParamFloat, 25.0f)       // Maximum allowed airspeed, m/s

// Landing
PARAM(LND_SPD, ParamFloat, 12.0f)       // Landing approach speed, m/s
PARAM(LND_FL_ALT, ParamFloat, 5.0f)     // Altitude to start flare, m
PARAM(LND_FL_SINK, ParamFloat, 0.5f)    // Target sink rate for flare, m/s

// Attitude Control
PARAM(ATT_PTCH_KP, ParamFloat, 0.1f)    // Pitch control proportional gain
PARAM(ATT_PTCH_KI, ParamFloat, 0.05f)   // Pitch control integral gain
PARAM(ATT_ROLL_KP, ParamFloat, 0.1f)    // Roll control proportional gain
PARAM(ATT_ROLL_KI, ParamFloat, 0.05f)   // Roll control integral gain

// TECS
PARAM(TECS_THR_KP, ParamFloat, 0.05f)   // Total energy control proportional gain
PARAM(TECS_THR_KI, ParamFloat, 0.02f)   // Total energy control integral gain
PARAM(TECS_PTCH_KP, ParamFloat, 0.1f)   // Energy balance proportional gain
PARAM(TECS_PTCH_KI, ParamFloat, 0.05f)  // Energy balance integral gain
PARAM(TECS_PTCH_LIM, ParamFloat, 30.0f) // Maximum pitch angle, deg

// L1 Controller
PARAM(L1_PERIOD, ParamFloat, 20.0f)     // L1 controller period
PARAM(L1_ROLL_LIM, ParamFloat, 45.0f)   // L1 controller roll limit

// Navigator
PARAM(NAV_ACC_RAD, ParamFloat, 5.0f)    // Waypoint acceptance radius, m

// Mixer
PARAM(PWM_MIN_ELE, ParamInt, 1000)      // Min duty elevator, us
PARAM(PWM_MIN_RUD, ParamInt, 1000)      // Min duty rudder, us
PARAM(PWM_MIN_THR, ParamInt, 1000)      // Min duty throttle, us
PARAM(PWM_MIN_AUX1, ParamInt, 1000)     // Min duty auxiliary channel 1, us
PARAM(PWM_MIN_AUX2, ParamInt, 1000)     // Min duty auxiliary channel 2, us
PARAM(PWM_MIN_AUX3, ParamInt, 1000)     // Min duty auxiliary channel 3, us
PARAM(PWM_MAX_ELE, ParamInt, 2000)      // Max duty elevator, us
PARAM(PWM_MAX_RUD, ParamInt, 2000)      // Max duty rudder, us
PARAM(PWM_MAX_THR, ParamInt, 2000)      // Max duty throttle, us
PARAM(PWM_MAX_AUX1, ParamInt, 2000)     // Max duty auxiliary channel 1, us
PARAM(PWM_MAX_AUX2, ParamInt, 2000)     // Max duty auxiliary channel 2, us
PARAM(PWM_MAX_AUX3, ParamInt, 2000)     // Max duty auxiliary channel 3, us
PARAM(PWM_REV_ELE, ParamInt, 0)         // Reverse elevator channel
PARAM(PWM_REV_RUD, ParamInt, 0)         // Reverse rudder channel
PARAM(PWM_REV_THR, ParamInt, 0)         // Reverse throttle channel
PARAM(PWM_REV_AUX1, ParamInt, 0)        // Reverse auxiliary channel 1
PARAM(PWM_REV_AUX2, ParamInt, 0)        // Reverse auxiliary channel 2
PARAM(PWM_REV_AUX3, ParamInt, 0)        // Reverse auxiliary channel 3

// RC transmitter input
PARAM(RC_MAX_DUTY, ParamInt, 2000)      // Stick input max duty, us
PARAM(RC_MIN_DUTY, ParamInt, 1000)      // Stick input min duty, us

// AHRS
PARAM(AHRS_BETA_GAIN, ParamFloat, 0.1f) // Madgwick filter beta gain
PARAM(AHRS_MAG_DECL, ParamFloat, 0.0f)  // Magnetic declination, deg
PARAM(AHRS_ACC_MAX, ParamFloat, 3.0f)   // Max accel for fusion, g

// Sensor calibration
PARAM(GYR_OFF_X, ParamFloat, 0.0f)
PARAM(GYR_OFF_Y, ParamFloat, 0.0f)
PARAM(GYR_OFF_Z, ParamFloat, 0.0f)
PARAM(ACC_OFF_X, ParamFloat, 0.0f)
PARAM(ACC_OFF_Y, ParamFloat, 0.0f)
PARAM(ACC_OFF_Z, ParamFloat, 0.0f)
PARAM(MAG_HI_X, ParamFloat, 0.0f)       // Magnetometer hard-iron calibration
PARAM(MAG_HI_Y, ParamFloat, 0.0f)
PARAM(MAG_HI_Z, ParamFloat, 0.0f)
PARAM(MAG_SI_XX, ParamFloat, 1.0f)      // Magnetometer soft-iron calibration
PARAM(MAG_SI_XY, ParamFloat, 0.0f)
PARAM(MAG_SI_XZ, ParamFloat, 0.0f)
PARAM(MAG_SI_YX, ParamFloat, 0.0f)
PARAM(MAG_SI_YY, ParamFloat, 1.0f)
PARAM(MAG_SI_YZ, ParamFloat, 0.0f)
PARAM(MAG_SI_ZX, ParamFloat, 0.0f)
PARAM(MAG_SI_ZY, ParamFloat, 0.0f)
PARAM(MAG_SI_ZZ, ParamFloat, 1.0f)

// Kalman filter
PARAM(EKF_BARO_VAR, ParamFloat, 1.0f)   // Barometer variance
PARAM(EKF_GNSS_VAR, ParamFloat, 0.25f)  // GNSS variance
PARAM(EKF_OF_VAR, ParamFloat, 0.1f)     // Optical flow variance
PARAM(EKF_OF_MIN, ParamInt, -100)       // Minimum accepted reading, pixels/sec
PARAM(EKF_OF_MAX, ParamInt, 100)        // Maximum accepted reading, pixels/sec

// System
PARAM(ENABLE_HITL, ParamInt, 0)         // Enable hardware-in-the-loop testing
