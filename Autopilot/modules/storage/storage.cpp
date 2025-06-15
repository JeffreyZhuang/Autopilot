#include "modules/storage/storage.h"

Storage::Storage(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _imu_sub(data_bus->imu_node),
	  _baro_sub(data_bus->baro_node),
	  _modes_sub(data_bus->modes_node),
	  _local_pos_sub(data_bus->local_position_node),
	  _mag_sub(data_bus->mag_node),
	  _gnss_sub(data_bus->gnss_node),
	  _rc_sub(data_bus->rc_node),
	  _ahrs_sub(data_bus->ahrs_node)
{
}

void Storage::update()
{
	_imu_data = _imu_sub.get();
	_baro_data = _baro_sub.get();
	_rc_data = _rc_sub.get();
	_local_pos = _local_pos_sub.get();
	_mag_data = _mag_sub.get();
	_modes_data = _modes_sub.get();
	_gnss_data = _gnss_sub.get();
	_ahrs_data = _ahrs_sub.get();

	if (_modes_data.system_mode != System_mode::LOAD_PARAMS)
	{
		if (!file_created)
		{
			if (_gnss_data.fix)
			{
				char filename[50];

				sprintf(filename, "%d-%d-%d-%d-%d-%d",
						_gnss_data.year, _gnss_data.month,
						_gnss_data.day, _gnss_data.hours,
						_gnss_data.minutes, _gnss_data.seconds);

				_hal->create_file(filename, strlen(filename));
				file_created = true;
			}
		}
		else
		{
			write();
		}
	}
}

void Storage::write()
{
	aplink_flight_log msg;
	msg.time_us = _hal->get_time_us();
	msg.roll = _ahrs_data.roll;
	msg.pitch = _ahrs_data.pitch;
	msg.yaw = _ahrs_data.yaw;
	msg.lat = _gnss_data.lat;
	msg.lon = _gnss_data.lon;
	msg.system_mode = get_mode_id(
		_modes_data.system_mode,
		_modes_data.flight_mode,
		_modes_data.auto_mode,
		_modes_data.manual_mode
	);

	uint8_t buffer[MAX_PACKET_LEN];
	uint16_t size = aplink_flight_log_pack(msg, buffer);
	for (int i = 0; i < size; i++)
	{
		_hal->write_storage(buffer[i]);
	}
}
