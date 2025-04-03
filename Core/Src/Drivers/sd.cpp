#include "Drivers/sd.h"

#define RING_BUFFER_SIZE (4096) // Must be a power of 2

static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

Sd::Sd()
{
	ring_buffer_setup(&ring_buffer, data_buffer, RING_BUFFER_SIZE);
	f_mount(&fatfs, SDPath, 1);
}

void Sd::create_file(char name[], uint8_t len)
{
	if (sd_mode == SDMode::IDLE)
	{
		strncpy(file_name, name, len);
		sd_mode = SDMode::CREATE_FILE;
	}
}

// Append byte to ring buffer
bool Sd::write_byte(uint8_t byte)
{
	if (sd_mode == SDMode::WRITE)
	{
		ring_buffer_write(&ring_buffer, byte);

		return true;
	}

	return false;
}

bool Sd::read(uint8_t* rx_buff, uint16_t size)
{
	if (sd_mode == SDMode::WRITE)
	{
		sd_mode = SDMode::SWITCH_TO_READ;
	}
	else if (sd_mode == SDMode::READ)
	{
		UINT bytes_read;
		FRESULT res = f_read(&fil, rx_buff, size, &bytes_read);
		if (res != FR_OK)
		{
			printf("Error during read\n");
		}

		// Check if end of file is reached
		return bytes_read == size;
	}

	return false;
}

// Empty ring buffer and sync to micro-sd card
void Sd::interrupt_callback()
{
	if (sd_mode == SDMode::CREATE_FILE)
	{
		FRESULT res = f_open(&fil, file_name, FA_WRITE | FA_CREATE_NEW);
		if (res != FR_OK)
		{
			printf("SD card failed. Make sure it is inserted.\n");
			while (1);
		}

		sd_mode = SDMode::WRITE;
	}
	else if (sd_mode == SDMode::WRITE)
	{
		// Empty ring buffer
		uint8_t buffer[RING_BUFFER_SIZE];
		uint16_t i = 0;
		while (!ring_buffer_empty(&ring_buffer))
		{
			ring_buffer_read(&ring_buffer, &buffer[i++]);
		}

		// Save to storage
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
	else if (sd_mode == SDMode::SWITCH_TO_READ)
	{
		f_close(&fil);

		FRESULT res = f_open(&fil, file_name, FA_READ);
		if (res != FR_OK)
		{
			printf("Error when opening file\n");
		}

		sd_mode = SDMode::READ;
	}
}
