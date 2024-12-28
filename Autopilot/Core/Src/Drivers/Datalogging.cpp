/*
 * microsd.c
 *
 *  Created on: Dec 23, 2024
 *      Author: jeffr
 */

#include <Datalogging.h>

void Datalogging::initialize()
{
	f_mount(&fatfs, SDPath, 1);
	FRESULT res = f_open(&fil, "data.bin", FA_WRITE | FA_READ | FA_CREATE_ALWAYS); // Don't use txt. Write code to turn into txt at end of flight.
	if (res != FR_OK)
	{
		printf("SD card failed. Make sure it is inserted.\n");
		while (1);
	}
}

void Datalogging::write()
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

void Datalogging::read()
{
	f_close(&fil);

	FRESULT res = f_open(&fil, "data.bin", FA_READ);
	if (res != FR_OK)
	{
		printf("Error when opening file\n");

		return; // Multiple exit points bad
	}

	while (1)
	{
		Datalogging_packet p;
		UINT bytes_read;
		res = f_read(&fil, &p, datalogging_packet_size, &bytes_read);

		// Write human readable to txt file

		if (res != FR_OK || bytes_read == 0) break;

		printf("%ld %f %f\n", p.time, p.acc_z, p.alt);
	}

	while (1);
}

void Datalogging::append_buffer(Datalogging_packet p)
{
	// If back_buffer is not full, add data to back_buffer
	// If back_buffer is full and front_buffer is not full, swap back and front buffers add data to back_buffer
	// If both buffers are full, there is no way to store the data so throw out the data
	bool back_buff_full = back_buff_idx == buffer_len;

	if (!back_buff_full)
	{
		// Add data to back buffer
		back_buffer[back_buff_idx] = p;
		back_buff_idx++;
	}
	else if (back_buff_full && !front_buff_full)
	{

		// Copy back buffer to front buffer
		memcpy(front_buffer, back_buffer, sizeof(back_buffer));
		front_buff_full = true;

		// Reset back buffer
		back_buff_idx = 0;

		// Add data to back buffer
		back_buffer[back_buff_idx] = p;
		back_buff_idx++;
	}
	else if (back_buff_full && front_buff_full)
	{
		// Throw out data
	}
}
