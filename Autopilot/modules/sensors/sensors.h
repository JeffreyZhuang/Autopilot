#ifndef MODULES_SENSORS_SENSORS_H_
#define MODULES_SENSORS_SENSORS_H_

#include "parameters.h"
#include "module.h"

class Sensors : Module
{
public:
	Sensors(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;
	Subscriber<HITL_data> _hitl_sub;

	Publisher<IMU_data> _imu_pub;
	Publisher<Mag_data> _mag_pub;
	Publisher<Baro_data> _baro_pub;
	Publisher<OF_data> _of_pub;
	Publisher<GNSS_data> _gnss_pub;
	Publisher<Power_data> _power_pub;

	Modes_data _modes_data;
	HITL_data _hitl_data;
};

#endif /* MODULES_SENSORS_SENSORS_H_ */
