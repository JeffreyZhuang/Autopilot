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
	  _rc_sub(data_bus->rc_node)
{
}

void Storage::update()
{
	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		_time_data = _time_sub.get();
		_imu_data = _imu_sub.get();
		_baro_data = _baro_sub.get();
		_rc_data = _rc_sub.get();
		_pos_est_data = _pos_est_sub.get();
		_mag_data = _mag_sub.get();

		write();
	}
}

void Storage::write()
{
	// Create packet with aplink
	// Write function
}

// Read one packet
void Storage::read()
{
	uint8_t byte;
	while (_hal->read_storage(&byte, 1))
	{
		// Parse aplink
	}
}
