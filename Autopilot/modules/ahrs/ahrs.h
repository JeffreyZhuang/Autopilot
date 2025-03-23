#ifndef AHRS_H
#define AHRS_H

#include "lib/madgwick/madgwick.h"
#include "lib/moving_average/moving_avg.h"
#include "plane.h"
#include "parameters.h"
#include "hal.h"
#include "module.h"
#include <stdio.h>
#include <math.h>

enum class Ahrs_state
{
	INITIALIZATION,
	RUNNING
};

/**
 * @brief Attitude Heading Reference System
 *
 */
class AHRS : public Module
{
public:
    AHRS(HAL* hal, Plane* plane);

    void update();
    bool is_converged();

private:
    Madgwick filter;
    Ahrs_state ahrs_state = Ahrs_state::INITIALIZATION;

    static constexpr size_t window_size = 100;
    float window_ax[window_size];
    float window_ay[window_size];
    float window_az[window_size];
    float window_mx[window_size];
    float window_my[window_size];
    float window_mz[window_size];
    MovingAverage avg_ax;
    MovingAverage avg_ay;
    MovingAverage avg_az;
    MovingAverage avg_mx;
	MovingAverage avg_my;
	MovingAverage avg_mz;

	IMU_data imu_data;
	Mag_data mag_data;

	void update_initialization();
	void update_running();
	void update_gyro();
	void update_imu();
	void update_imu_mag();
	void set_initial_angles();
	void publish_ahrs();
	void apply_compass_calibration(float mag_data[3]);
	bool is_accel_reliable();
};

#endif
