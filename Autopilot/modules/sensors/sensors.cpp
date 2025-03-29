#include "sensors.h"

//Update function:
//if not config mode
//if hitl enable
// use hitl data
// else
// use sensor data

// Have a struct in data bus for hitl data


// Do calibration for IMU and mag

// hal has bool read_imu(*ax, *ay, *az) and then sensors module publishes

Sensors::Sensors(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _hitl_sub(data_bus->hitl_node),
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

	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		uint64_t time = _hal->get_time_us();

		if (get_params()->hitl.enable)
		{
			if (_hitl_sub.check_new())
			{
				_hitl_data = _hitl_sub.get();

				_imu_pub.publish(IMU_data{_hitl_data.gx, _hitl_data.gy, _hitl_data.gz,
										  _hitl_data.ax, _hitl_data.ay, _hitl_data.az});
				_baro_pub.publish(Baro_data{_hitl_data.asl, time});
			}
		}
		else
		{
			float ax, ay, az, gx, gy, gz;
			if (_hal->read_imu(&ax, &ay, &az, &gx, &gy, &gz))
			{
				// Apply IMU calibration

				_imu_pub.publish(IMU_data{gx, gy, gz, ax, ay, az, time});
			}

			float baro_alt;
			if (_hal->read_baro(&baro_alt))
			{
				if (baro_alt < 100000)
				{
					_baro_pub.publish(Baro_data{baro_alt, time});
				}
			}

			float mx, my, mz;
			if (_hal->read_mag(&mx, &my, &mz))
			{
				_mag_pub.publish(Mag_data{mx, my, mz, time});
			}
		}
	}
}
