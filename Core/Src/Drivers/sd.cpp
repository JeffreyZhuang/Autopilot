/*
 * microsd.c
 *
 *  Created on: Dec 23, 2024
 *      Author: jeffr
 */

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
		else
		{
//			printf("Write success\n");
		}

		front_buff_full = false;

		res = f_sync(&fil);
		if (res != FR_OK)
		{
			printf("Error during sync\n");
		}
		else
		{
//			printf("Sync success\n");
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
	// If back_buffer is not full, add data to back_buffer
	// If back_buffer is full and front_buffer is not full, swap back and front buffers add data to back_buffer
	// If both buffers are full, there is no way to store the data so throw out the data
	bool back_buff_full = back_buff_last_idx + size > buffer_len;

	if (!back_buff_full)
	{
		// Add data to back buffer
		for (uint32_t i = back_buff_last_idx; i < back_buff_last_idx + size; i++)
		{
			back_buffer[i] = packet[i];
		}
		back_buff_last_idx += size;
	}
	else if (back_buff_full && !front_buff_full)
	{
		// Copy back buffer to front buffer
		memcpy(front_buffer, back_buffer, back_buff_last_idx);
		front_buff_full = true;

		// Reset back buffer
		back_buff_last_idx = 0;

		// Add data to back buffer
		for (int i = 0; i < size; i++)
		{
			back_buffer[i] = packet[i];
		}
		back_buff_last_idx = size;
	}
	else if (back_buff_full && front_buff_full)
	{
		// Throw out data
	}
}
