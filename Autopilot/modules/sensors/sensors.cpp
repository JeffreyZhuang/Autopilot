#include "sensors.h"

Sensors::Sensors(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _hitl_sub(data_bus->hitl_node),
	  _telem_sub(data_bus->telem_node),
	  _imu_pub(data_bus->imu_node),
	  _mag_pub(data_bus->mag_node),
	  _baro_pub(data_bus->baro_node),
	  _of_pub(data_bus->of_node),
	  _gnss_pub(data_bus->gnss_node),
	  _power_pub(data_bus->power_node),
	  _time_pub(data_bus->time_node)
{
}

void Sensors::update()
{
	// Need to do this for every module. If AHRS updates at 100hz but imu updates at 30hz, it needs to recalculate
	uint64_t time = _hal->get_time_us();

	if (_time_data.timestamp > 0)
	{
		_time_data.dt_s = (time - _time_data.timestamp) * US_TO_S;
	}
	else
	{
		// Initialize
		_time_data.dt_s = 0;
	}

	_time_data.timestamp = time;
	_time_data.loop_iteration++;

	_time_pub.publish(_time_data);

	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode == System_mode::LOAD_PARAMS)
	{

	}
	else if (_modes_data.system_mode == System_mode::CALIBRATION)
	{

	}
	else if (_modes_data.system_mode == System_mode::FLIGHT ||
			 _modes_data.system_mode == System_mode::STARTUP)
	{
		uint64_t time = _hal->get_time_us();

		if (_telem_sub.get().hitl_enable)
		{
			if (_hitl_sub.check_new())
			{
				_hitl_data = _hitl_sub.get();

				_imu_pub.publish(IMU_data{_hitl_data.imu_gx, _hitl_data.imu_gy, _hitl_data.imu_gz,
										  _hitl_data.imu_ax, _hitl_data.imu_ay, _hitl_data.imu_az});
				_baro_pub.publish(Baro_data{_hitl_data.baro_asl, time});
			}
		}
		else
		{
			float ax, ay, az, gx, gy, gz;
			if (_hal->read_imu(&ax, &ay, &az, &gx, &gy, &gz))
			{
				// Apply IMU calibration
				gx -= param_get_float(GYR_OFF_X);
				gy -= param_get_float(GYR_OFF_Y);
				gz -= param_get_float(GYR_OFF_Z);

				ax -= param_get_float(ACC_OFF_X);
				ay -= param_get_float(ACC_OFF_Y);
				az -= param_get_float(ACC_OFF_Z);

				_imu_pub.publish(IMU_data{gx, gy, gz, ax, ay, az, time});
			}

			float baro_alt;
			if (_hal->read_baro(&baro_alt))
			{
				_baro_pub.publish(Baro_data{baro_alt, time});
			}

			float mx, my, mz;
			if (_hal->read_mag(&mx, &my, &mz))
			{
				// Storage for hard-iron calibrated magnetometer data
				float hi_cal[3];

				// Apply hard-iron offsets
				hi_cal[0] = mx - param_get_float(AHRS_HI_X);
				hi_cal[1] = my - param_get_float(AHRS_HI_Y);
				hi_cal[2] = mz - param_get_float(AHRS_HI_Z);

				// Apply soft-iron scaling
				mx = (param_get_float(AHRS_SI_XX) * hi_cal[0]) +
					 (param_get_float(AHRS_SI_XY) * hi_cal[1]) +
					 (param_get_float(AHRS_SI_XZ) * hi_cal[2]);
				my = (param_get_float(AHRS_SI_YX) * hi_cal[0]) +
					 (param_get_float(AHRS_SI_YY) * hi_cal[1]) +
					 (param_get_float(AHRS_SI_YZ) * hi_cal[2]);
				mz = (param_get_float(AHRS_SI_ZX) * hi_cal[0]) +
					 (param_get_float(AHRS_SI_ZY) * hi_cal[1]) +
					 (param_get_float(AHRS_SI_ZZ) * hi_cal[2]);

				_mag_pub.publish(Mag_data{mx, my, mz, time});
			}

			int16_t of_x, of_y;
			if (_hal->read_optical_flow(&of_x, &of_y))
			{
				_of_pub.publish(OF_data{of_x, of_y, time});
			}
		}
	}
}
