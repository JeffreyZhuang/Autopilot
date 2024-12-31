#include <flight_hal.h>

void Derived_hal::init_logger()
{
	_sd.initialize();
}

void Derived_hal::write_storage_buffer()
{
	Sd_packet p;
	p.time = get_time_us();
	p.acc_z = _plane->imu_az;
	p.alt = _plane->baro_alt;
	_sd.append_buffer(p);
}

void Derived_hal::flush_storage_buffer()
{
	_sd.write();
}

void Derived_hal::read_storage()
{
	_sd.read();
}
