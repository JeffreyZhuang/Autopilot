#ifndef VEHICLE_PARAMETERS_H_
#define VEHICLE_PARAMETERS_H_

enum class FlightState {
    TAKEOFF = 1,
    LAND = 2
};

struct Vehicle {
    // Constraints
    float airspeed_min;
    float airspeed_max;
    float airspeed_cruise;
    float pitch_limit_max;
    float pitch_limit_min;
    float roll_limit;

    // Monitor
    float batt_current;
    float batt_voltage;
    float autopilot_current;
    float autopilot_voltage;

    // State machine
    FlightState state = FlightState::TAKEOFF;

    // IMU
    float imu_ax;
    float imu_ay;
    float imu_az;
    float imu_gx;
    float imu_gy;
    float imu_gz;
    float imu_temp;

    // Compass
    float compass_mx;
    float compass_my;
    float compass_mz;

    // Barometer
    float baro_alt;
    float baro_temp;
    
    // AHRS
    float ahrs_roll;
    float ahrs_pitch;
    float ahrs_yaw;
};

#endif /* VEHICLE_PARAMETERS_H_ */