#include <flight_hal.h>

void Flight_hal::init_logger()
{
	_sd.initialize();
}

void Flight_hal::write_storage_buffer()
{
	Sd_packet p;
	p.time = HAL_GetTick();
	p.acc_z = _plane->imu_az;
	p.alt = _plane->baro_alt;
	_sd.append_buffer(p);
}

void Flight_hal::flush_storage_buffer()
{
	_sd.write();
}

void Flight_hal::read_storage()
{
	_sd.read();
}
