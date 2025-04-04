// Attitude control
PARAM(ATT_PTCH_KP, PARAM_TYPE_FLOAT) // Pitch control proportional gain
PARAM(ATT_PTCH_KI, PARAM_TYPE_FLOAT) // Pitch control integral gain
PARAM(ATT_ROLL_KP, PARAM_TYPE_FLOAT) // Roll control proportional gain
PARAM(ATT_ROLL_KI, PARAM_TYPE_FLOAT) // Roll control integral gain

// Takeoff
PARAM(TKO_ALT, PARAM_TYPE_FLOAT) // Altitude to detect when takeoff is complete, m
PARAM(TKO_PTCH, PARAM_TYPE_FLOAT) // Constant pitch hold during takeoff, deg
PARAM(TKO_ROLL_LIM, PARAM_TYPE_FLOAT) // Roll limit during takeoff, deg

// Landing
PARAM(LND_FLARE_ALT, PARAM_TYPE_FLOAT) // Altitude to start flare, m
PARAM(LND_FLARE_SINK, PARAM_TYPE_FLOAT) // Target sink rate for flare, m/s

// TECS
PARAM(TECS_SPD_CRUISE, PARAM_TYPE_FLOAT) // Cruise speed, m/s
PARAM(TECS_SPD_LND, PARAM_TYPE_FLOAT) // Landing approach speed, m/s
PARAM(TECS_MIN_SPD, PARAM_TYPE_FLOAT) // Minimum allowed airspeed, m/s
PARAM(TECS_MAX_SPD, PARAM_TYPE_FLOAT) // Maximum allowed airspeed, m/s
PARAM(TECS_THR_KP, PARAM_TYPE_FLOAT) // Total energy control proportional gain
PARAM(TECS_THR_KI, PARAM_TYPE_FLOAT) // Total energy control integral gain
PARAM(TECS_PTCH_KP, PARAM_TYPE_FLOAT) // Energy balance proportional gain
PARAM(TECS_PTCH_KI, PARAM_TYPE_FLOAT) // Energy balance integral gain
PARAM(TECS_PTCH_LIM, PARAM_TYPE_FLOAT) // Maximum pitch angle, deg
PARAM(TECS_THR_CRUISE, PARAM_TYPE_FLOAT) // Steady-state cruise throttle

// L1 Controller
PARAM(L1_PERIOD, PARAM_TYPE_FLOAT) // L1 controller period
PARAM(L1_ROLL_LIM, PARAM_TYPE_FLOAT) // L1 controller roll limit

// Navigator
PARAM(NAV_ACC_RAD, PARAM_TYPE_FLOAT) // Waypoint acceptance radius, m

// Mixer
PARAM(PWM_MIN_ELE, PARAM_TYPE_INT32) // Min duty elevator, us
PARAM(PWM_MIN_RUD, PARAM_TYPE_INT32) // Min duty rudder, us
PARAM(PWM_MIN_THR, PARAM_TYPE_INT32) // Min duty throttle, us
PARAM(PWM_MIN_AUX1, PARAM_TYPE_INT32) // Min duty auxiliary channel 1, us
PARAM(PWM_MIN_AUX2, PARAM_TYPE_INT32) // Min duty auxiliary channel 2, us
PARAM(PWM_MIN_AUX3, PARAM_TYPE_INT32) // Min duty auxiliary channel 3, us
PARAM(PWM_MAX_ELE, PARAM_TYPE_INT32) // Max duty elevator, us
PARAM(PWM_MAX_RUD, PARAM_TYPE_INT32) // Max duty rudder, us
PARAM(PWM_MAX_THR, PARAM_TYPE_INT32) // Max duty throttle, us
PARAM(PWM_MAX_AUX1, PARAM_TYPE_INT32) // Max duty auxiliary channel 1, us
PARAM(PWM_MAX_AUX2, PARAM_TYPE_INT32) // Max duty auxiliary channel 2, us
PARAM(PWM_MAX_AUX3, PARAM_TYPE_INT32) // Max duty auxiliary channel 3, us
PARAM(PWM_REV_ELE, PARAM_TYPE_INT32) // Reverse elevator channel
PARAM(PWM_REV_RUD, PARAM_TYPE_INT32) // Reverse rudder channel
PARAM(PWM_REV_THR, PARAM_TYPE_INT32) // Reverse throttle channel
PARAM(PWM_REV_AUX1, PARAM_TYPE_INT32) // Reverse auxiliary channel 1
PARAM(PWM_REV_AUX2, PARAM_TYPE_INT32) // Reverse auxiliary channel 2
PARAM(PWM_REV_AUX3, PARAM_TYPE_INT32) // Reverse auxiliary channel 3

// RC transmitter input
PARAM(RC_MAX_DUTY, PARAM_TYPE_INT32) // Stick input max duty, us
PARAM(RC_MIN_DUTY, PARAM_TYPE_INT32) // Stick input min duty, us

// AHRS
PARAM(AHRS_BETA_GAIN, PARAM_TYPE_FLOAT) // Madgwick filter beta gain
PARAM(AHRS_MAG_DECL, PARAM_TYPE_FLOAT) // Magnetic declination, deg
PARAM(AHRS_ACC_MAX, PARAM_TYPE_FLOAT) // Max acceleration to enable sensor fusion, g
PARAM(AHRS_HI_X, PARAM_TYPE_FLOAT) // Magnetometer hard-iron calibration
PARAM(AHRS_HI_Y, PARAM_TYPE_FLOAT)
PARAM(AHRS_HI_Z, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_XX, PARAM_TYPE_FLOAT) // Magnetometer soft-iron calibration
PARAM(AHRS_SI_XY, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_XZ, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_YX, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_YY, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_YZ, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_ZX, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_ZY, PARAM_TYPE_FLOAT)
PARAM(AHRS_SI_ZZ, PARAM_TYPE_FLOAT)
PARAM(GYR_OFF_X, PARAM_TYPE_FLOAT)
PARAM(GYR_OFF_Y, PARAM_TYPE_FLOAT)
PARAM(GYR_OFF_Z, PARAM_TYPE_FLOAT)
PARAM(ACC_OFF_X, PARAM_TYPE_FLOAT)
PARAM(ACC_OFF_Y, PARAM_TYPE_FLOAT)
PARAM(ACC_OFF_Z, PARAM_TYPE_FLOAT)

// Kalman filter
PARAM(EKF_BARO_VAR, PARAM_TYPE_FLOAT) // Barometer variance
PARAM(EKF_GNSS_VAR, PARAM_TYPE_FLOAT) // GNSS variance
PARAM(EKF_OF_ENABLE, PARAM_TYPE_INT32) // Enable optical flow sensor
PARAM(EKF_OF_MIN, PARAM_TYPE_INT32) // Minimum accepted reading, pixels/sec
PARAM(EKF_OF_MAX, PARAM_TYPE_INT32) // Maximum accepted reading, pixels/sec
