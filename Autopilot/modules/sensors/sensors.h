#ifndef MODULES_SENSORS_SENSORS_H_
#define MODULES_SENSORS_SENSORS_H_

#include "module.h"
#include "constants.h"
#include "params.h"

class Sensors : Module
{
public:
	Sensors(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;
	Subscriber<hitl_sensors_s> _hitl_sensors_sub;
	Subscriber<Telem_data> _telem_sub;

	Publisher<IMU_data> _imu_pub;
	Publisher<Mag_data> _mag_pub;
	Publisher<Baro_data> _baro_pub;
	Publisher<OF_data> _of_pub;
	Publisher<GNSS_data> _gnss_pub;
	Publisher<Power_data> _power_pub;
	Publisher<time_s> _time_pub;

	time_s _time{};
	Modes_data _modes_data;
	hitl_sensors_s _hitl_sensors;
};

#endif /* MODULES_SENSORS_SENSORS_H_ */
