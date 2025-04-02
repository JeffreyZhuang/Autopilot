#include "Drivers/sd.h"

#define RING_BUFFER_SIZE (4096) // Must be a power of 2

static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

Sd::Sd()
{
	ring_buffer_setup(&ring_buffer, data_buffer, RING_BUFFER_SIZE);
}

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

// Append byte to ring buffer
void Sd::write_byte(uint8_t byte)
{
	ring_buffer_write(&ring_buffer, byte);
}

// Empty ring buffer and sync to micro-sd card
void Sd::interrupt_callback()
{
	if (!reading)
	{
		uint8_t buffer[RING_BUFFER_SIZE];
		uint16_t i = 0;
		while (!ring_buffer_empty(&ring_buffer))
		{
			ring_buffer_read(&ring_buffer, buffer[i++]);
		}

		UINT bytes_written;
		FRESULT res = f_write(&fil, buffer, i, &bytes_written);
		if (res != FR_OK)
		{
			printf("Error during writing\n");
		}

		res = f_sync(&fil);
		if (res != FR_OK)
		{
			printf("Error during sync\n");
		}
	}
}

bool Sd::read(uint8_t* rx_buff, uint16_t size)
{
	if (!reading)
	{
		reading = true;

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

	// Check if end of file is reached
	return bytes_read == size;
}
