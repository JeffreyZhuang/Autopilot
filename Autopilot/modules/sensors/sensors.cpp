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
		if (get_params()->hitl.enable)
		{
			read_hitl();
		}
		else
		{
			read_sensors();
		}
	}
}

void Sensors::read_sensors()
{
	uint64_t time = _hal->get_time_us();

	float ax, ay, az, gx, gy, gz;
	if (_hal->read_imu(&ax, &ay, &az, &gx, &gy, &gz))
	{
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

void Sensors::read_hitl()
{
	uint64_t time = _hal->get_time_us();
	_imu_pub.publish(IMU_data{data->gx, data->gy, data->gz, data->ax, data->ay, data->az, time});
	_mag_pub.publish(Mag_data{data->mx, data->my, data->mz, time});
	_baro_pub.publish(Baro_data{data->asl, time});
	_gnss_pub.publish(GNSS_data{(double)data->lat * 1E-7, (double)data->lon * 1E-7, data->asl, 10, true, time});
	_of_pub.publish(OF_data{data->of_x, data->of_y, time});
}
