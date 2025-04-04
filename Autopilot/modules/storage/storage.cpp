#include "modules/storage/storage.h"

Storage::Storage(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _imu_sub(data_bus->imu_node),
	  _baro_sub(data_bus->baro_node),
	  _modes_sub(data_bus->modes_node),
	  _pos_est_sub(data_bus->pos_est_node),
	  _mag_sub(data_bus->mag_node),
	  _gnss_sub(data_bus->gnss_node),
	  _time_sub(data_bus->time_node),
	  _rc_sub(data_bus->rc_node),
	  _ahrs_sub(data_bus->ahrs_node)
{
}

void Storage::update()
{
	_time_data = _time_sub.get();
	_imu_data = _imu_sub.get();
	_baro_data = _baro_sub.get();
	_rc_data = _rc_sub.get();
	_pos_est_data = _pos_est_sub.get();
	_mag_data = _mag_sub.get();
	_modes_data = _modes_sub.get();
	_gnss_data = _gnss_sub.get();
	_ahrs_data = _ahrs_sub.get();

	if (_modes_data.system_mode == System_mode::LOAD_PARAMS)
	{
		if (_time_data.unix_epoch_time > 0)
		{
			char filename[20];
			sprintf(filename, "%ld", _time_data.unix_epoch_time);
			_hal->create_file(filename, strlen(filename));
		}
	}
	else if (_modes_data.system_mode == System_mode::STARTUP)
	{
		write();
	}
	else if (_modes_data.system_mode == System_mode::FLIGHT)
	{
		write();
	}
	else if (_modes_data.system_mode == System_mode::DOWNLOAD_LOGS)
	{
		read();
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

	aplink_vfr_hud vfr_hud;
	vfr_hud.roll = _ahrs_data.roll;
	vfr_hud.pitch = _ahrs_data.pitch;
	vfr_hud.heading = _ahrs_data.yaw;
	uint8_t vfr_hud_buff[MAX_PACKET_LEN];
	uint16_t vfr_hud_len = aplink_vfr_hud_pack(vfr_hud, vfr_hud_buff);
	for (int i = 0; i < vfr_hud_len; i++)
	{
		_hal->write_storage(vfr_hud_buff[i]);
	}
}

// Read one packet
void Storage::read()
{
	uint8_t byte;
	while (_hal->read_storage(&byte, 1))
	{
		if (aplink_parse_byte(&msg, byte))
		{
			if (msg.msg_id == VFR_HUD_MSG_ID)
			{
				aplink_vfr_hud vfr_hud;
				if (aplink_vfr_hud_msg_decode(&msg, &vfr_hud))
				{
					printf("VFR_HUD: %f\n", vfr_hud.roll);
					break;
				}
			}
			else if (msg.msg_id == GPS_RAW_MSG_ID)
			{
				break;
			}
			else
			{
				break;
			}
		}
	}
}
