#include "derived_hal.h"

void Derived_hal::write_sd()
{
	Sd_packet p;
	p.time = _plane->time;
	p.acc_z = _plane->imu_az;
	p.alt = _plane->baro_alt;
	sd.append_buffer(p);
}

void Derived_hal::flush_sd()
{
	sd.write();
}

void Derived_hal::read_sd()
{
	sd.read();
}
