#include "sd.h"

void Sd::initialize()
{
	f_mount(&fatfs, SDPath, 1);

	// Find next available file name
	unsigned int file_idx = 1;
	while (true)
	{
		sprintf(filename, "log%u.bin", file_idx);
		FILINFO fno;
		if (f_stat(filename, &fno) != FR_OK)
		{
			// File does not exist, so we can use this name
			break;
		}
		file_idx++;
	}

	FRESULT res = f_open(&fil, filename, FA_WRITE | FA_CREATE_NEW);
	if (res != FR_OK)
	{
		printf("SD card failed. Make sure it is inserted.\n");
		while (1);
	}
}

void Sd::write(uint8_t* tx_buff, uint16_t size)
{
	if (!closed)
	{
		UINT bytes_written;
		FRESULT res = f_write(&fil, tx_buff, sizeof(tx_buff), &bytes_written);
		if (res != FR_OK)
		{
			printf("Error during writing\n");
		}
	}
}

void Sd::read(uint8_t* rx_buff, uint16_t size)
{
	if (!closed)
	{
		closed = true;

		f_close(&fil);

		FRESULT res = f_open(&fil, filename, FA_READ);
		if (res != FR_OK)
		{
			printf("Error when opening file\n");
		}
	}

	UINT bytes_read;
	FRESULT res = f_read(&fil, rx_buff, size, &bytes_read);
	if (res != FR_OK)
	{
		printf("Error during read\n");
	}
}

void Sd::flush()
{
	if (!closed)
	{
		FRESULT res = f_sync(&fil);
		if (res != FR_OK)
		{
			printf("Error during sync\n");
		}
	}
}

//void Sd::initialize()
//{
//	f_mount(&fatfs, SDPath, 1);
//
//	// Find next available file name
//	unsigned int file_idx = 1;
//	while (true)
//	{
//		sprintf(filename, "log%u.bin", file_idx);
//		FILINFO fno;
//		if (f_stat(filename, &fno) != FR_OK)
//		{
//			// File does not exist, so we can use this name
//			break;
//		}
//		file_idx++;
//	}
//
//	FRESULT res = f_open(&fil, filename, FA_WRITE | FA_CREATE_NEW);
//	if (res != FR_OK)
//	{
//		printf("SD card failed. Make sure it is inserted.\n");
//		while (1);
//	}
//}
//
//void Sd::write()
//{
//	if (front_buff_full && !closed)
//	{
//		printf("time: %ld, front_buffer size: %d\n", HAL_GetTick(), sizeof(front_buffer));
//
//		UINT bytes_written;
//		FRESULT res = f_write(&fil, front_buffer, sizeof(front_buffer), &bytes_written);
//		if (res != FR_OK)
//		{
//			printf("Error during writing\n");
//		}
//
//		front_buff_full = false;
//
//		res = f_sync(&fil);
//		if (res != FR_OK)
//		{
//			printf("Error during sync\n");
//		}
//	}
//}
//
//void Sd::read(uint8_t* rx_buff, uint16_t size)
//{
//	if (!closed)
//	{
//		closed = true;
//
//		f_close(&fil);
//
//		FRESULT res = f_open(&fil, filename, FA_READ);
//		if (res != FR_OK)
//		{
//			printf("Error when opening file\n");
//		}
//	}
//
//	UINT bytes_read;
//	FRESULT res = f_read(&fil, rx_buff, size, &bytes_read);
//}
//
//void Sd::append_buffer(uint8_t* packet, uint16_t size)
//{
//	for (uint16_t i = 0; i < size; i++)
//	{
//		append_byte(packet[i]);
//	}
//}
//
//void Sd::append_byte(uint8_t byte)
//{
//	// Double buffering:
//	// If back_buffer is not full, add data to back_buffer
//	// If back_buffer is full and front_buffer is not full, swap back and front buffers add data to back_buffer
//	// If both buffers are full, there is no way to store the data so throw out the data
//	bool back_buff_full = back_buff_last_idx == buffer_max_len;
//	if (!back_buff_full)
//	{
//		back_buffer[back_buff_last_idx] = byte;
//		back_buff_last_idx++;
//	}
//	else if (back_buff_full && !front_buff_full)
//	{
//		// Copy back buffer to front buffer
//		memcpy(front_buffer, back_buffer, buffer_max_len);
//		front_buff_full = true;
//
//		// Add byte to back buffer
//		back_buffer[0] = byte;
//
//		// Reset buffer index
//		back_buff_last_idx = 1;
//	}
//	else
//	{
//		// Throw out data if both buffers full
//		printf("Both buffers full\n");
//	}
//}
