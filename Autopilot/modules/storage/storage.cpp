#include "modules/storage/storage.h"

Storage::Storage(HAL* hal, Plane* plane) : Module(hal, plane)
{
}

void Storage::update()
{
	if (_plane->system_mode != Plane::System_mode::CONFIG)
	{
		write();
	}
}

void Storage::update_background()
{
	if (_plane->system_mode != Plane::System_mode::CONFIG)
	{
		flush();
	}
}

void Storage::write()
{
	// Create struct
	Storage_payload payload = create_payload();

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

Storage_payload Storage::create_payload()
{
	Plane::IMU_data imu_data = _plane->get_imu_data(imu_handle);
	Plane::Mag_data mag_data = _plane->get_mag_data(mag_handle);
	Plane::GNSS_data gnss_data = _plane->get_gnss_data(gnss_handle);
	Plane::Pos_est_data pos_est_data = _plane->get_pos_est_data(pos_est_handle);

	Storage_payload payload = {
		_plane->loop_iteration,
		_plane->time_us + _plane->us_since_epoch,
		{imu_data.gx, imu_data.gy, imu_data.gz},
		{imu_data.ax, imu_data.ay, imu_data.az},
		{mag_data.x, mag_data.y, mag_data.z},
		{pos_est_data.pos_n, pos_est_data.pos_e, pos_est_data.pos_d},
		{pos_est_data.vel_n, pos_est_data.vel_e, pos_est_data.vel_d},
		_plane->get_baro_data(baro_handle).alt,
		{_plane->rc_ail_norm, _plane->rc_ele_norm, _plane->rc_rud_norm, _plane->rc_thr_norm},
		gnss_data.lat,
		gnss_data.lon,
		gnss_data.fix,
		static_cast<uint8_t>(_plane->flight_mode)
	};

	return payload;
}

void Storage::read()
{
	while (true)
	{
		// Read storage to look for start byte
		uint8_t start_byte[1];
		if (!_hal->read_storage(start_byte, 1))
		{
			break; // End of file
		}

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

			break; // Break after one payload has been read
		}
	}
}
