#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "hal.h"
#include "kalman.h"

/**
 * @brief Calculates the position of the plane
 *
 */
class Navigation
{
public:
    Navigation(HAL * hal, Plane * plane);
    void execute();

    float acc_n; // Rotated to the world frame
	float acc_e;
	float acc_d;

private:
    HAL * _hal;
    Plane * _plane;
    Kalman kalman;

    uint64_t last_imu_timestamp;
    uint64_t last_gnss_timestamp;
    uint64_t last_ahrs_timestamp;
    uint64_t l

    float gnss_n; // meters
    float gnss_e;
    float gnss_d;

    float g = 9.80665;

    void read_imu();
	void read_gnss();
	void read_ahrs();
	bool check_new_imu_data();
	bool check_new_gnss_data();
	bool check_new_ahrs_data();
	void update_plane();

	void predict_imu();

	void update_gps();
	void update_baro();
};

#endif
