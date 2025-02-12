#include <sd.h>

void Sd::initialize()
{
	f_mount(&fatfs, SDPath, 1);
	FRESULT res = f_open(&fil, "data.bin", FA_WRITE | FA_READ | FA_CREATE_ALWAYS); // Don't use txt. Write code to turn into txt at end of flight.
	if (res != FR_OK)
	{
		printf("SD card failed. Make sure it is inserted.\n");
		while (1);
	}
}

void Sd::write()
{
	if (front_buff_full)
	{
		printf("time: %ld, front_buffer size: %d\n", HAL_GetTick(), sizeof(front_buffer));

		UINT bytes_written;
		FRESULT res = f_write(&fil, front_buffer, sizeof(front_buffer), &bytes_written);
		if (res != FR_OK)
		{
			printf("Error during writing\n");
		}

		front_buff_full = false;

		res = f_sync(&fil);
		if (res != FR_OK)
		{
			printf("Error during sync\n");
		}
	}
}

void Sd::read(uint8_t* rx_buff, uint16_t size)
{
	f_close(&fil);

	FRESULT res = f_open(&fil, "data.bin", FA_READ);
	if (res != FR_OK)
	{
		printf("Error when opening file\n");
	}

	UINT bytes_read;
	f_read(&fil, rx_buff, size, &bytes_read);
}

void Sd::append_buffer(uint8_t* packet, uint16_t size)
{
	for (uint16_t i = 0; i < size; i++)
	{
		append_byte(packet[i]);
	}
}

void Sd::append_byte(uint8_t byte)
{
	// If back_buffer is not full, add data to back_buffer
	// If back_buffer is full and front_buffer is not full, swap back and front buffers add data to back_buffer
	// If both buffers are full, there is no way to store the data so throw out the data
	bool back_buff_full = back_buff_last_idx == buffer_max_len;
	if (!back_buff_full)
	{
		back_buffer[back_buff_last_idx] = byte;
		back_buff_last_idx++;
	}
	else if (back_buff_full && !front_buff_full)
	{
		// Copy back buffer to front buffer
		memcpy(front_buffer, back_buffer, buffer_max_len);
		front_buff_full = true;

		// Add byte to back buffer
		back_buffer[0] = byte;

		// Reset buffer index
		back_buff_last_idx = 1;
	}
	else
	{
		// Throw out data if both buffers full
	}
}
