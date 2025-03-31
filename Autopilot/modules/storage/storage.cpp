#include "modules/storage/storage.h"

Storage::Storage(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _imu_sub(data_bus->imu_node),
	  _baro_sub(data_bus->baro_node),
	  _modes_sub(data_bus->modes_node),
	  _pos_est_sub(data_bus->pos_est_node),
	  _mag_sub(data_bus->mag_node),
	  _gnss_sub(data_bus->gnss_node),
	  _time_sub(data_bus->time_node),
	  _rc_sub(data_bus->rc_node)
{
}

void Storage::update()
{
	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		_time_data = _time_sub.get();
		_imu_data = _imu_sub.get();
		_baro_data = _baro_sub.get();
		_rc_data = _rc_sub.get();
		_pos_est_data = _pos_est_sub.get();
		_mag_data = _mag_sub.get();

		write();
	}
}

void Storage::update_background()
{
	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		flush();
	}
}

void Storage::write()
{
	Storage_payload payload = create_payload();
	uint8_t packet[aplink_calc_packet_size(sizeof(payload))];
	telem_link.pack(packet, reinterpret_cast<uint8_t*>(&payload), sizeof(payload), STORAGE_MSG_ID);

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
	Storage_payload payload = {
		_time_data.loop_iteration,
		_time_data.timestamp + _time_data.us_since_epoch,
		{_imu_data.gx, _imu_data.gy, _imu_data.gz},
		{_imu_data.ax, _imu_data.ay, _imu_data.az},
		{_mag_data.x, _mag_data.y, _mag_data.z},
		{_pos_est_data.pos_n, _pos_est_data.pos_e, _pos_est_data.pos_d},
		{_pos_est_data.vel_n, _pos_est_data.vel_e, _pos_est_data.vel_d},
		_baro_data.alt,
		{_rc_data.ail_norm, _rc_data.ele_norm, _rc_data.rud_norm, _rc_data.thr_norm},
		_gnss_data.lat,
		_gnss_data.lon,
		_gnss_data.fix,
		static_cast<uint8_t>(_modes_data.flight_mode)
	};

	return payload;
}

// Read one packet
void Storage::read()
{
	uint8_t start_byte[1];

	while (_hal->read_storage(start_byte, 1))
	{
		// Detect start byte
		if (start_byte[0] == 0)
		{
			// Read payload with cobs
			uint8_t payload_cobs[payload_size + 1];
			if (_hal->read_storage(payload_cobs, sizeof(payload_cobs)))
			{
				// Decode cobs
				uint8_t payload_arr[payload_size];
				cobs_decode(payload_arr, sizeof(payload_arr), payload_cobs, sizeof(payload_cobs));

				// Convert byte array into struct
				Storage_payload payload;
				memcpy(&payload, payload_arr, sizeof(payload));

				// Print payload
				printf("%d\n", payload.loop_iteration);
			}
		}
	}
}
