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
	payload.c[0] = 'h';
	payload.c[1] = 'i';
	payload.loop_iteration = 69;

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








	// Try decoding

//	// Read payload with cobs
//	uint8_t payload_cobs_test[sizeof(Storage_payload) + 1];
//	for (int i = 0; i < sizeof(payload_cobs_test); i++)
//	{
//		payload_cobs_test[i] = packet[i + 1];
//	}
//
//	// Decode cobs
//	uint8_t payload_arr_test[sizeof(Storage_payload)];
//	cobs_decode(payload_arr_test, sizeof(payload_arr_test), payload_cobs_test, sizeof(payload_cobs_test));
//
//	// Convert byte array into struct
//	Storage_payload payload_test;
//	memcpy(&payload_test, payload_arr_test, sizeof(payload_test));
//
//	// Print payload
//	printf("%c %c %d %c\n", payload_test.c[0], payload_test.c[1], payload_test.loop_iteration, payload_test.a);






	// I suspect its here, putting the struct into array



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






		// Read payload with cobs
		uint8_t payload_cobs_test[sizeof(Storage_payload) + 1];
		for (int i = 0; i < sizeof(payload_cobs_test); i++)
		{
			payload_cobs_test[i] = back_buffer[i + 1];
		}

		// Decode cobs
		uint8_t payload_arr_test[sizeof(Storage_payload)];
		cobs_decode(payload_arr_test, sizeof(payload_arr_test), payload_cobs_test, sizeof(payload_cobs_test));

		// Convert byte array into struct
		Storage_payload payload_test;
		memcpy(&payload_test, payload_arr_test, sizeof(payload_test));

		// Print payload
		printf("%c %c %d %c\n", payload_test.c[0], payload_test.c[1], payload_test.loop_iteration, payload_test.a);
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
		printf("Flushed Buffer\n");

		_hal->write_storage_buffer(front_buffer, buffer_size);
		front_buff_full = false;

		_hal->flush_storage_buffer();
	}
}

void Storage::read()
{
	while (true)
	{
		//testing
		uint8_t packet[sizeof(Storage_payload) + 2];
		_hal->read_storage(packet, sizeof(packet));

		uint8_t payload_cobs_test[sizeof(Storage_payload) + 1];
		for (int i = 0; i < sizeof(payload_cobs_test); i++)
		{
			payload_cobs_test[i] = packet[i + 1];
		}

		// Decode cobs
		uint8_t payload_arr_test[sizeof(Storage_payload)];
		cobs_decode(payload_arr_test, sizeof(payload_arr_test), payload_cobs_test, sizeof(payload_cobs_test));

		// Convert byte array into struct
		Storage_payload payload_test;
		memcpy(&payload_test, payload_arr_test, sizeof(payload_test));

		// Print payload
		printf("%c %c %d %c\n", payload_test.c[0], payload_test.c[1], payload_test.loop_iteration, payload_test.a);





//		// Read storage to look for start byte
//		uint8_t start_byte[1];
//		_hal->read_storage(start_byte, sizeof(start_byte));
//
//		// Detect start byte
//		if (start_byte[0] == 0)
//		{
//			// Read payload with cobs
//			uint8_t payload_cobs[sizeof(Storage_payload) + 1];
//			_hal->read_storage(payload_cobs, sizeof(payload_cobs));
//
//			// Decode cobs
//			uint8_t payload_arr[sizeof(Storage_payload)];
//			cobs_decode(payload_arr, sizeof(payload_arr), payload_cobs, sizeof(payload_cobs));
//
//			// Convert byte array into struct
//			Storage_payload payload;
//			memcpy(&payload, payload_arr, sizeof(payload));
//
//			// Print payload
//			printf("%c %c %d %c\n", payload.c[0], payload.c[1], payload.loop_iteration, payload.a);
//		}
	}
}
