#include "Drivers/sd.h"

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

// Move flush to write
void Sd::write(uint8_t* tx_buff, uint16_t size)
{
	if (!reading)
	{
		UINT bytes_written;
		FRESULT res = f_write(&fil, tx_buff, size, &bytes_written);
		if (res != FR_OK)
		{
			printf("Error during writing\n");
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
	return !(bytes_read < size);
}

void Sd::flush()
{
	if (!reading)
	{
		FRESULT res = f_sync(&fil);
		if (res != FR_OK)
		{
			printf("Error during sync\n");
		}
	}
}
