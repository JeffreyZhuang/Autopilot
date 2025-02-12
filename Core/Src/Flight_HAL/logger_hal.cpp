#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_logger()
{
	_sd.initialize();
}

void Flight_hal::write_storage_buffer(uint8_t* data, uint16_t len)
{
	Sd_packet p;
	p.time = _plane->time;
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
