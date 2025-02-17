#include "storage.h"

Storage::Storage(Plane* plane, HAL* hal)
{
	_plane = plane;
	_hal = hal;
}

void Storage::write()
{
	// Create struct
	Storage_payload payload = {
		_plane->loop_iteration,
		_plane->time,
		{_plane->imu_gx, _plane->imu_gy, _plane->imu_gz},
		{_plane->imu_ax, _plane->imu_ay, _plane->imu_az},
		{_plane->compass_mx, _plane->compass_my, _plane->compass_mz},
		{_plane->nav_pos_north, _plane->nav_pos_east, _plane->nav_pos_down},
		{_plane->nav_vel_north, _plane->nav_vel_east, _plane->nav_vel_down},
		_plane->baro_alt,
		_plane->rc_rudder,
		_plane->rc_elevator,
		_plane->rc_throttle,
		_plane->gnss_lat,
		_plane->gnss_lon,
		_plane->gps_fix,
		_plane->mode_id
	};

	// Convert struct to byte array
	uint8_t payload_arr[payload_size];
	memcpy(payload_arr, &payload, payload_size);

	// COBS encode
	uint8_t payload_cobs[payload_size + 1];
	cobs_encode(payload_cobs, sizeof(payload_cobs), payload_arr, sizeof(payload_arr));

	// Add start byte to complete packet
	uint8_t packet[packet_size];
	packet[0] = 0; // Start byte
	for (uint i = 1; i < packet_size; i++) // Copy over payload to packet
	{
		packet[i] = payload_cobs[i - 1];
	}

	// Double buffering:
	// If back_buffer is not full, add data to back_buffer
	// If back_buffer is full and front_buffer is not full, swap back and front buffers add data to back_buffer
	// If both buffers are full, there is no way to store the data so throw out the data
	bool back_buff_full = back_buff_last_idx == buffer_size;
	if (!back_buff_full)
	{
		for (uint i = 0; i < packet_size; i++)
		{
			back_buffer[back_buff_last_idx] = packet[i];
			back_buff_last_idx++;
		}
	}
	else if (back_buff_full && !front_buff_full)
	{
		// Copy back buffer to front buffer
		memcpy(front_buffer, back_buffer, buffer_size);
		front_buff_full = true;

		// Reset buffer index
		back_buff_last_idx = 0;

		// Add packet to buffer
		for (uint i = 0; i < packet_size; i++)
		{
			back_buffer[back_buff_last_idx] = packet[i];
			back_buff_last_idx++;
		}
	}
	else
	{
		// Throw out data if both buffers full
//		printf("Both buffers full\n");
	}
}

void Storage::flush()
{
	if (front_buff_full)
	{
//		printf("Flushed Buffer\n");

		_hal->write_storage_buffer(front_buffer, buffer_size);
		front_buff_full = false;

		_hal->flush_storage_buffer();
	}
}

void Storage::read()
{
	while (true)
	{
		// Read storage to look for start byte
		uint8_t start_byte[1];
		_hal->read_storage(start_byte, 1);

		// Detect start byte
		if (start_byte[0] == 0)
		{
			// Read payload with cobs
			uint8_t payload_cobs[payload_size + 1];
			_hal->read_storage(payload_cobs, sizeof(payload_cobs));

			// Decode cobs
			uint8_t payload_arr[payload_size];
			cobs_decode(payload_arr, sizeof(payload_arr), payload_cobs, sizeof(payload_cobs));

			// Convert byte array into struct
			Storage_payload payload;
			memcpy(&payload, payload_arr, sizeof(payload));

			// Print payload
			printf("%d\n", payload.loop_iteration);
			_hal->delay_us(10000); // Must be here
			// If there is a corrupt bit, it takes extra long to print and will skip some
		}
	}
}
