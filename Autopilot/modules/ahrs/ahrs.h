#ifndef AHRS_H
#define AHRS_H

#include <data_bus.h>
#include "lib/madgwick/madgwick.h"
#include "lib/moving_average/moving_avg.h"
#include "hal.h"
#include "module.h"
#include "params.h"
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
    AHRS(HAL* hal, Data_bus* data_bus);

    void update();
    bool is_converged();

private:
    Madgwick filter;
    Ahrs_state ahrs_state = Ahrs_state::INITIALIZATION;

    static constexpr size_t window_size = 100; // TODO: Better capitalization
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

	Subscriber<IMU_data> _imu_sub;
	Subscriber<Mag_data> _mag_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<Time_data> _time_sub;

	Publisher<AHRS_data> _ahrs_pub;

	Time_data _time_data{};
	IMU_data _imu_data{};
	Mag_data _mag_data{};
	Modes_data _modes_data{};

	void update_initialization();
	void update_running();
	void update_gyro();
	void update_imu();
	void update_imu_mag();
	void set_initial_angles();
	void publish_ahrs();
	bool is_accel_reliable();
};

#endif
