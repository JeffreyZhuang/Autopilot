#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "lib/kalman/kalman.h"
#include "lib/moving_average/moving_avg.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "constants.h"
#include "module.h"
#include "params.h"
#include <stdio.h>

static constexpr int n = 6;
static constexpr int m = 3;

enum class Pos_estimator_state
{
	INITIALIZATION,
	RUNNING
};

/**
 * @brief Calculates the position and altitude of the plane
 */
class Position_estimator : public Module
{
public:
    Position_estimator(HAL* hal, Data_bus* data_bus);

    void update() override;

private:
    Kalman kalman;

    Pos_estimator_state pos_estimator_state = Pos_estimator_state::INITIALIZATION;

    MovingAverage avg_baro;
    MovingAverage avg_lat;
    MovingAverage avg_lon;
    static constexpr uint8_t window_len = 50;
    float window_baro[window_len];
    float window_lat[window_len];
    float window_lon[window_len];

    Subscriber<time_s> _time_sub;
    Subscriber<Modes_data> _modes_sub;
    Subscriber<IMU_data> _imu_sub;
    Subscriber<Baro_data> _baro_sub;
    Subscriber<GNSS_data> _gnss_sub;
    Subscriber<OF_data> _of_sub;
    Subscriber<AHRS_data> _ahrs_sub;
    Subscriber<Telem_data> _telem_sub;
    Subscriber<home_position_s> _home_position_sub;

    Publisher<Pos_est_data> _pos_est_pub;

    Pos_est_data _pos_est_data{};
    OF_data _of_data{};
    Modes_data _modes_data{};
    GNSS_data _gnss_data{};
    Baro_data _baro_data{};
    AHRS_data _ahrs_data{};
    IMU_data _imu_data{};
    time_s _time{};
    Telem_data _telem_data{};

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
