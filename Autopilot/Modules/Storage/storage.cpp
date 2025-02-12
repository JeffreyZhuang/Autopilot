#include "storage.h"

#include <stdio.h> // Testing, remove later

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

	uint8_t data[sizeof(packet)];
	memcpy(data, &packet, sizeof(packet));
	_hal->write_storage_buffer(data, sizeof(packet));
}

void Storage::flush()
{
	_hal->flush_storage_buffer();
}

void Storage::read()
{
	// Read 10 packets
	for (int i = 0; i < 10; i++)
	{
		Storage_packet packet;
		uint8_t rx_buff[sizeof(Storage_packet)];
		_hal->read_storage(rx_buff, sizeof(rx_buff));
		memcpy(&packet, rx_buff, sizeof(packet));

		printf("%d %f", (uint32_t)packet.time, packet.acc_z);
	}
}
