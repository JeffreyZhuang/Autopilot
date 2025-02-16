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
	// Create struct
	Storage_payload payload;
	payload.loop_iteration = _plane->loop_iteration;
	payload.time = _plane->time;

	// Convert struct to byte array
	uint8_t payload_arr[sizeof(payload)];
	memcpy(payload_arr, &payload, sizeof(payload));

	// COBS encode
	uint8_t payload_cobs[sizeof(payload) + 1];
	cobs_encode(payload_cobs, sizeof(payload_cobs), payload_arr, sizeof(payload_arr));

	// Add start byte to complete packet
	uint8_t packet[sizeof(payload_cobs) + 1];
	packet[0] = 0; // Start byte
	for (uint i = 1; i < sizeof(packet); i++) // Copy over payload to packet
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
		for (int i = 0; i < sizeof(packet); i++)
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
		for (int i = 0; i < sizeof(packet); i++)
		{
			back_buffer[back_buff_last_idx] = packet[i];
			back_buff_last_idx++;
		}
	}
	else
	{
		// Throw out data if both buffers full
		printf("Both buffers full\n");
	}
}

void Storage::flush()
{
	if (front_buff_full)
	{
		printf("time: %ld, front_buffer size: %d\n", _hal->get_time_us(), sizeof(front_buffer));

		_hal->write_storage_buffer(front_buffer, buffer_size);
		front_buff_full = false;

		_hal->flush_storage_buffer();
	}
}

void Storage::read()
{
	while (true)
	{
		Storage_payload payload;
		uint8_t payload_cobs[sizeof(payload) + 1];

		// Read storage to look for start byte
		uint8_t start_byte[1];
		_hal->read_storage(start_byte, sizeof(start_byte));

		// Detect start byte
		if (start_byte[0] == 0)
		{
			// Read payload with cobs
			_hal->read_storage(payload_cobs, sizeof(payload_cobs));

			// Decode cobs
			uint8_t payload_arr[sizeof(payload)];
			cobs_decode(payload_arr, sizeof(payload_arr), payload_cobs, sizeof(payload_cobs));

			// Convert byte array into struct
			memcpy(&payload, payload_arr, sizeof(payload));

			// Print payload
			printf("%d %d\n", payload.loop_iteration, payload.time);
		}
	}
}
