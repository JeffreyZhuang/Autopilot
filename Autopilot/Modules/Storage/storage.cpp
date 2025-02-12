#include "storage.h"

Storage::Storage(Plane* plane, HAL* hal)
{
	_plane = plane;
	_hal = hal;
}

void Storage::write()
{
	Storage_packet packet;
	packet.time = _plane->time;
	packet.acc_z = _plane->imu_az;
	packet.alt = _plane->baro_alt;

	_hal->write_storage_buffer(packet, sizeof(packet));
}

void Storage::flush()
{
	_hal->flush_storage_buffer();
}

void Storage::read()
{

}
