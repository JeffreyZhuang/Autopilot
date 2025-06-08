#ifndef MODULES_STORAGE_STORAGE_H_
#define MODULES_STORAGE_STORAGE_H_

#include <lib/data_bus/data_bus.h>
#include <lib/hal/hal.h>
#include <lib/module/module.h>
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
	Storage(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	bool file_created = false;

	Subscriber<IMU_data> _imu_sub;
	Subscriber<Mag_data> _mag_sub;
	Subscriber<GNSS_data> _gnss_sub;
	Subscriber<local_position_s> _local_pos_sub;
	Subscriber<Baro_data> _baro_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<AHRS_data> _ahrs_sub;

	IMU_data _imu_data{};
	Mag_data _mag_data{};
	GNSS_data _gnss_data{};
	local_position_s _local_pos;
	Baro_data _baro_data{};
	Modes_data _modes_data{};
	RC_data _rc_data{};
	AHRS_data _ahrs_data{};

	aplink_msg msg;

	void write();
};

#endif /* MODULES_STORAGE_STORAGE_H_ */
