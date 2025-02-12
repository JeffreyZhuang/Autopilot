#include "storage.h"

#include <stdio.h> // Testing, remove later
#include <inttypes.h>

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
	if (_plane->systemMode != SystemMode::FLIGHT)
		_hal->flush_storage_buffer();
}

void Storage::read()
{
	printf("Start Read\n");

	// Read 10 packets
	for (int i = 0; i < 10; i++)
	{
		Storage_packet packet;
		uint8_t rx_buff[sizeof(Storage_packet)];
		_hal->read_storage(rx_buff, sizeof(rx_buff));
		memcpy(&packet, rx_buff, sizeof(packet));

		printf("%" PRIu64 " %f\n", packet.time, packet.acc_z);
	}

	printf("Done\n");
	while (1);


	// Keep reading single byte until start byte
	// Then COBS decode and put in Storage_packet struct to read
}
