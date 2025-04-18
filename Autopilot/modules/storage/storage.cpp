#include "modules/storage/storage.h"

Storage::Storage(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _imu_sub(data_bus->imu_node),
	  _baro_sub(data_bus->baro_node),
	  _modes_sub(data_bus->modes_node),
	  _local_pos_sub(data_bus->local_position_node),
	  _mag_sub(data_bus->mag_node),
	  _gnss_sub(data_bus->gnss_node),
	  _time_sub(data_bus->time_node),
	  _rc_sub(data_bus->rc_node),
	  _ahrs_sub(data_bus->ahrs_node)
{
}

void Storage::update()
{
	_time = _time_sub.get();
	_imu_data = _imu_sub.get();
	_baro_data = _baro_sub.get();
	_rc_data = _rc_sub.get();
	_local_pos = _local_pos_sub.get();
	_mag_data = _mag_sub.get();
	_modes_data = _modes_sub.get();
	_gnss_data = _gnss_sub.get();
	_ahrs_data = _ahrs_sub.get();

	if (_modes_data.system_mode == System_mode::LOAD_PARAMS)
	{
		if (_time.unix_epoch_time > 0)
		{
			char filename[20];
			sprintf(filename, "%ld", _time.unix_epoch_time);
			_hal->create_file(filename, strlen(filename));
		}
	}
	else
	{
		write();
	}
}

void Storage::write()
{
	aplink_gps_raw gps_raw;
	gps_raw.lat = _gnss_data.lat;
	gps_raw.lon = _gnss_data.lon;
	gps_raw.sats = _gnss_data.sats;
	gps_raw.fix = _gnss_data.fix;
	uint8_t gps_raw_buff[MAX_PACKET_LEN];
	uint16_t gps_raw_len = aplink_gps_raw_pack(gps_raw, gps_raw_buff);
	for (int i = 0; i < gps_raw_len; i++)
	{
		_hal->write_storage(gps_raw_buff[i]);
	}

	aplink_vehicle_status_full vehicle_status_full;
	vehicle_status_full.roll = _ahrs_data.roll;
	vehicle_status_full.pitch = _ahrs_data.pitch;
	vehicle_status_full.yaw = _ahrs_data.yaw;
	uint8_t vehicle_status_full_buff[MAX_PACKET_LEN];
	uint16_t vehicle_status_full_len = aplink_vehicle_status_full_pack(vehicle_status_full, vehicle_status_full_buff);
	for (int i = 0; i < vehicle_status_full_len; i++)
	{
		_hal->write_storage(vehicle_status_full_buff[i]);
	}
}
