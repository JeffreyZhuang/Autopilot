#ifndef MODULES_SENSORS_SENSORS_H_
#define MODULES_SENSORS_SENSORS_H_

#include "lib/parameters/params.h"
#include "module.h"
#include "constants.h"

class Sensors : Module
{
public:
	Sensors(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;
	Subscriber<hitl_sensors_s> _hitl_sensors_sub;

	Publisher<IMU_data> _imu_pub;
	Publisher<Mag_data> _mag_pub;
	Publisher<Baro_data> _baro_pub;
	Publisher<OF_data> _of_pub;
	Publisher<GNSS_data> _gnss_pub;
	Publisher<Power_data> _power_pub;

	Modes_data _modes_data;
	hitl_sensors_s _hitl_sensors;

	void update_calibration();
	void update_flight();
	void update_hitl();
};

#endif /* MODULES_SENSORS_SENSORS_H_ */
