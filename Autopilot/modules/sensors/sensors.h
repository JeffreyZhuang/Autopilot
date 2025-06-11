#ifndef MODULES_SENSORS_SENSORS_H_
#define MODULES_SENSORS_SENSORS_H_

#include "lib/constants/constants.h"
#include "lib/module/module.h"
#include "lib/parameters/params.h"

class Sensors : Module
{
public:
	Sensors(HAL* hal, DataBus* data_bus);

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
	Publisher<uncalibrated_imu_s> _unc_imu_pub;
	Publisher<uncalibrated_mag_s> _unc_mag_pub;

	Modes_data _modes_data;
	hitl_sensors_s _hitl_sensors;

	bool _enable_hitl = false;

	// Parameters
	float _gyr_off_x;
	float _gyr_off_y;
	float _gyr_off_z;
	float _acc_off_x;
	float _acc_off_y;
	float _acc_off_z;
	float _hi_x;
	float _hi_y;
	float _hi_z;
	float _si_xx;
	float _si_xy;
	float _si_xz;
	float _si_yx;
	float _si_yy;
	float _si_yz;
	float _si_zx;
	float _si_zy;
	float _si_zz;

	void parameters_update();

	void update_load_params();
	void update_flight();
	void update_hitl();
	void update_calibration();
};

#endif /* MODULES_SENSORS_SENSORS_H_ */
