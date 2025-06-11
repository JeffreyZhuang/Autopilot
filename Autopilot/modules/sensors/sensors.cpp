#include "sensors.h"

Sensors::Sensors(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _hitl_sensors_sub(data_bus->hitl_sensors_node),
	  _imu_pub(data_bus->imu_node),
	  _mag_pub(data_bus->mag_node),
	  _baro_pub(data_bus->baro_node),
	  _of_pub(data_bus->of_node),
	  _gnss_pub(data_bus->gnss_node),
	  _power_pub(data_bus->power_node),
	  _unc_imu_pub(data_bus->uncalibrated_imu_node),
	  _unc_mag_pub(data_bus->uncalibrated_mag_node)
{
}

void Sensors::parameters_update()
{
	param_get(GYR_OFF_X, &_gyr_off_x);
	param_get(GYR_OFF_Y, &_gyr_off_y);
	param_get(GYR_OFF_Z, &_gyr_off_z);
	param_get(ACC_OFF_X, &_acc_off_x);
	param_get(ACC_OFF_Y, &_acc_off_y);
	param_get(ACC_OFF_Z, &_acc_off_z);
	param_get(MAG_HI_X, &_hi_x);
	param_get(MAG_HI_Y, &_hi_y);
	param_get(MAG_HI_Z, &_hi_z);
	param_get(MAG_SI_XX, &_si_xx);
	param_get(MAG_SI_XY, &_si_xy);
	param_get(MAG_SI_XZ, &_si_xz);
	param_get(MAG_SI_YX, &_si_yx);
	param_get(MAG_SI_YY, &_si_yy);
	param_get(MAG_SI_YZ, &_si_yz);
	param_get(MAG_SI_ZX, &_si_zx);
	param_get(MAG_SI_ZY, &_si_zy);
	param_get(MAG_SI_ZZ, &_si_zz);
}

void Sensors::update()
{
	parameters_update();

	_modes_data = _modes_sub.get();

	switch (_modes_data.system_mode)
	{
	case System_mode::LOAD_PARAMS:
		update_load_params();
		break;
	case System_mode::STARTUP:
	case System_mode::FLIGHT:
		if (_enable_hitl)
		{
			update_hitl();
		}
		else
		{
			update_flight();
		}
		break;
	}
}

void Sensors::update_load_params()
{
	// This function detects if HITL simulator is connected by
	// checking if HITL data is received

	// HITL simulator MUST to be connected during startup mode to enable HITL
	// That prevents the AHRS from initializing before HITL started

	if (_hitl_sensors_sub.check_new())
	{
		_enable_hitl = true;
	}
}

void Sensors::update_flight()
{
	float ax, ay, az, gx, gy, gz;

	if (_hal->read_imu(&ax, &ay, &az, &gx, &gy, &gz))
	{
		_unc_imu_pub.publish(uncalibrated_imu_s{gx, gy, gz, ax, ay, az, _hal->get_time_us()});

		// Apply IMU calibration
		gx -= -_gyr_off_x;
		gy -= _gyr_off_y;
		gz -= _gyr_off_z;
		ax -= _acc_off_x;
		ay -= _acc_off_y;
		az -= _acc_off_z;

		_imu_pub.publish(IMU_data{gx, gy, gz, ax, ay, az, _hal->get_time_us()});
	}

	float baro_alt;

	if (_hal->read_baro(&baro_alt))
	{
		_baro_pub.publish(Baro_data{baro_alt, _hal->get_time_us()});
	}

	float mx, my, mz;

	if (_hal->read_mag(&mx, &my, &mz))
	{
		_unc_mag_pub.publish(uncalibrated_mag_s{mx, my, mz, _hal->get_time_us()});

		// Apply hard-iron offsets
		mx -= _hi_x;
		my -= _hi_y;
		mz -= _hi_z;

		// Apply soft-iron scaling
		mx = _si_xx * mx + _si_xy * my + _si_xz * mz;
		my = _si_yx * mx + _si_yy * my + _si_yz * mz;
		mz = _si_zx * mx + _si_zy * my + _si_zz * mz;

		_mag_pub.publish(Mag_data{mx, my, mz, _hal->get_time_us()});
	}

	int16_t of_x, of_y;

	if (_hal->read_optical_flow(&of_x, &of_y))
	{
		_of_pub.publish(OF_data{of_x, of_y, _hal->get_time_us()});
	}
}

void Sensors::update_hitl()
{
	if (_hitl_sensors_sub.check_new())
	{
		_hitl_sensors = _hitl_sensors_sub.get();

		_imu_pub.publish(IMU_data{
			.gx = _hitl_sensors.imu_gx,
			.gy = _hitl_sensors.imu_gy,
			.gz = _hitl_sensors.imu_gz,
			.ax = _hitl_sensors.imu_ax,
			.ay = _hitl_sensors.imu_ay,
			.az = _hitl_sensors.imu_az,
			.timestamp = _hal->get_time_us()
		});

		_baro_pub.publish(Baro_data{
			_hitl_sensors.baro_asl,
			_hal->get_time_us()
		});

		_gnss_pub.publish(GNSS_data{
			.lat = (double)_hitl_sensors.gps_lat / 1E7,
			.lon = (double)_hitl_sensors.gps_lon / 1E7,
			.timestamp = _hal->get_time_us()
		});
	}
}
