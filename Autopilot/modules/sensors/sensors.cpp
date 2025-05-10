#include "sensors.h"

Sensors::Sensors(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _hitl_sensors_sub(data_bus->hitl_sensors_node),
	  _imu_pub(data_bus->imu_node),
	  _mag_pub(data_bus->mag_node),
	  _baro_pub(data_bus->baro_node),
	  _of_pub(data_bus->of_node),
	  _gnss_pub(data_bus->gnss_node),
	  _power_pub(data_bus->power_node)
{
}

void Sensors::update()
{
	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode == System_mode::CALIBRATION)
	{
		update_calibration();
	}
	else if (_modes_data.system_mode == System_mode::FLIGHT ||
			 _modes_data.system_mode == System_mode::STARTUP)
	{
		bool enable_hitl;
		param_get(ENABLE_HITL, &enable_hitl);
		if (enable_hitl)
		{
			update_hitl();
		}
		else
		{
			update_flight();
		}
	}
}

void Sensors::update_flight()
{
	float ax, ay, az, gx, gy, gz;
	if (_hal->read_imu(&ax, &ay, &az, &gx, &gy, &gz))
	{
		// Apply IMU calibration
		float gyr_off_x;
		float gyr_off_y;
		float gyr_off_z;
		float acc_off_x;
		float acc_off_y;
		float acc_off_z;

		param_get(GYR_OFF_X, &gyr_off_x);
		param_get(GYR_OFF_Y, &gyr_off_y);
		param_get(GYR_OFF_Z, &gyr_off_z);
		param_get(ACC_OFF_X, &acc_off_x);
		param_get(ACC_OFF_Y, &acc_off_y);
		param_get(ACC_OFF_Z, &acc_off_z);

		gx -= gyr_off_x;
		gy -= gyr_off_y;
		gz -= gyr_off_z;
		ax -= acc_off_x;
		ay -= acc_off_y;
		az -= acc_off_z;

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
		float hi_x, hi_y, hi_z;
		param_get(MAG_HI_X, &hi_x);
		param_get(MAG_HI_Y, &hi_y);
		param_get(MAG_HI_Z, &hi_z);

		// Apply hard-iron offsets
		float hi_cal[3];
		hi_cal[0] = mx - hi_x;
		hi_cal[1] = my - hi_y;
		hi_cal[2] = mz - hi_z;

		// Apply soft-iron scaling
		mx = (param_get_float(MAG_SI_XX) * hi_cal[0]) +
			 (param_get_float(MAG_SI_XY) * hi_cal[1]) +
			 (param_get_float(MAG_SI_XZ) * hi_cal[2]);
		my = (param_get_float(MAG_SI_YX) * hi_cal[0]) +
			 (param_get_float(MAG_SI_YY) * hi_cal[1]) +
			 (param_get_float(MAG_SI_YZ) * hi_cal[2]);
		mz = (param_get_float(MAG_SI_ZX) * hi_cal[0]) +
			 (param_get_float(MAG_SI_ZY) * hi_cal[1]) +
			 (param_get_float(MAG_SI_ZZ) * hi_cal[2]);

		_mag_pub.publish(Mag_data{mx, my, mz, _hal->get_time_us()});
	}

	int16_t of_x, of_y;
	if (_hal->read_optical_flow(&of_x, &of_y))
	{
		_of_pub.publish(OF_data{of_x, of_y, _hal->get_time_us()});
	}
}

// Send raw sensor data without calibration
void Sensors::update_calibration()
{
	float ax, ay, az, gx, gy, gz;
	if (_hal->read_imu(&ax, &ay, &az, &gx, &gy, &gz))
	{
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
		_mag_pub.publish(Mag_data{mx, my, mz, _hal->get_time_us()});
	}
}

void Sensors::update_hitl()
{
	if (_hitl_sensors_sub.check_new())
	{
		_hitl_sensors = _hitl_sensors_sub.get();

		_imu_pub.publish(IMU_data{_hitl_sensors.imu_gx, _hitl_sensors.imu_gy, _hitl_sensors.imu_gz,
								  _hitl_sensors.imu_ax, _hitl_sensors.imu_ay, _hitl_sensors.imu_az});
		_baro_pub.publish(Baro_data{_hitl_sensors.baro_asl, _hal->get_time_us()});
		_gnss_pub.publish(GNSS_data{
			.lat = (double)_hitl_sensors.gps_lat / 1E7,
			.lon = (double)_hitl_sensors.gps_lon / 1E7,
			.timestamp = _hal->get_time_us()
		});
	}
}
