#ifndef MODULES_STORAGE_STORAGE_H_
#define MODULES_STORAGE_STORAGE_H_

#include "data_bus.h"
#include "hal.h"
#include "module.h"
#include <stdint.h>
#include <cstring>
#include <stdio.h>

extern "C"
{
#include "lib/aplink_c/aplink.h"
#include "lib/aplink_c/aplink_messages.h"
}

class Storage : public Module
{
public:
	Storage(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<IMU_data> _imu_sub;
	Subscriber<Mag_data> _mag_sub;
	Subscriber<GNSS_data> _gnss_sub;
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<Baro_data> _baro_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<time_s> _time_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<AHRS_data> _ahrs_sub;

	IMU_data _imu_data{};
	Mag_data _mag_data{};
	GNSS_data _gnss_data{};
	Pos_est_data _pos_est_data{};
	Baro_data _baro_data{};
	Modes_data _modes_data{};
	time_s _time{};
	RC_data _rc_data{};
	AHRS_data _ahrs_data{};

	aplink_msg msg;

	void write();
};

#endif /* MODULES_STORAGE_STORAGE_H_ */
