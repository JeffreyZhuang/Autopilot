#ifndef AHRS_H
#define AHRS_H

#include <lib/constants/constants.h>
#include <lib/hal/hal.h>
#include <lib/module/module.h>
#include "lib/data_bus/data_bus.h"
#include "lib/madgwick/madgwick.h"
#include "lib/moving_average/moving_avg.h"
#include "lib/parameters/params.h"
#include "lib/utils/utils.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief Attitude Heading Reference System
 *
 */
class AHRS : public Module
{
public:
    AHRS(HAL* hal, DataBus* data_bus);

    void update() override;

private:
    uint64_t _last_time = 0;
    float _dt = 0;

    Madgwick filter;

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

	Publisher<AHRS_data> _ahrs_pub;

	IMU_data _imu_data{};
	Mag_data _mag_data{};
	Modes_data _modes_data{};
	AHRS_data _ahrs_data;

	void update_initialization();
	void update_running();
	void update_gyro();
	void update_imu();
	void update_imu_mag();
	void set_initial_angles();
	void publish_ahrs();
	bool is_accel_reliable();
	void update_parameters();
};

#endif
