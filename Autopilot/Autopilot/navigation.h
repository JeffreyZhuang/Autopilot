#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <plane.h>
#include <hal.h>

// For now, only GNSS and accelerometer. No baro yet for simplicity.
#define EKF_N 6 // Size of state space (pos_n, pos_e, pos_d, vel_n, vel_e, vel_d)
#define EKF_M 3 // Size of observation space (acc_n, acc_e, acc_d)

#include "tinyekf.h"

static const float predict_dt = 1 / 400;
static const float update_dt = 1 / 10;

static const float Q[EKF_N*EKF_N] =
{
	1, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0,
	0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 1
};

static const float R[EKF_M*EKF_M] =
{
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};

static const float F[EKF_N*EKF_N] =
{
	1, 0, 0, update_dt, 0,  	   0,
	0, 1, 0, 0,  	    update_dt, 0,
	0, 0, 1, 0,  		0,  	   update_dt,
	0, 0, 0, 1,  		0,  	   0,
	0, 0, 0, 0,  		1,  	   0,
	0, 0, 0, 0,  		0,  	   1
};

static const float H[EKF_M*EKF_N] =
{
	0.5*predict_dt*predict_dt, 0,         				  0,
	0,         				   0.5*predict_dt*predict_dt, 0,
	0,         				   0,         			      0.5*predict_dt*predict_dt,
	predict_dt,     		   0,         				  0,
	0,         				   predict_dt, 				  0,
	0,         				   0,         				  predict_dt
};

static ekf_t ekf;

/**
 * @brief Calculates the position of the plane
 *
 */
class Navigation
{
public:
    Navigation(HAL * hal, Plane * plane);
    void execute();
    void prediction_step();
    void update_step();
private:
    bool check_new_imu_data();
    bool check_new_gnss_data();

    HAL * _hal;
    Plane * _plane;
    uint64_t last_imu_timestamp;
    uint64_t last_gnss_timestamp;
    uint64_t time;
};

#endif
