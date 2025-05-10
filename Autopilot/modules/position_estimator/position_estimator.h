#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "lib/parameters/params.h"
#include "lib/kalman/kalman.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "constants.h"
#include "module.h"
#include <stdio.h>

static constexpr int n = 6;
static constexpr int m = 3;

/**
 * @brief Calculates the position and altitude of the plane
 */
class PositionEstimator : public Module
{
public:
    PositionEstimator(HAL* hal, Data_bus* data_bus);

    void update() override;

private:
    uint64_t _last_time = 0;
	float _dt = 0;

    Kalman kalman;

    Subscriber<Modes_data> _modes_sub;
    Subscriber<IMU_data> _imu_sub;
    Subscriber<Baro_data> _baro_sub;
    Subscriber<GNSS_data> _gnss_sub;
    Subscriber<OF_data> _of_sub;
    Subscriber<AHRS_data> _ahrs_sub;

    Publisher<local_position_s> _local_pos_pub;

    local_position_s _local_pos{};
    OF_data _of_data{};
    Modes_data _modes_data{};
    GNSS_data _gnss_data{};
    Baro_data _baro_data{};
    AHRS_data _ahrs_data{};
    IMU_data _imu_data{};

    void update_initialization();
    void update_running();

    void predict_accel();
	void update_gps();
	void update_baro();
	void update_plane();
	void update_of_agl();

    // Kalman
    Eigen::MatrixXf get_a(float dt);
    Eigen::MatrixXf get_b(float dt);
    Eigen::MatrixXf get_q();

	Eigen::Vector3f inertial_to_ned(const Eigen::Vector3f& imu_measurement, float roll, float pitch, float yaw);

	bool is_of_reliable();
};

#endif
