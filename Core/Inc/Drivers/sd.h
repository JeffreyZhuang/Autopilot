#ifndef INC_SD_H_
#define INC_SD_H_

#include <stdio.h>
#include <string.h>

extern "C" {
	#include "fatfs.h"
	#include "usbd_cdc_if.h"
}

class Sd
{
public:
	void initialize();
	void write(uint8_t* tx_buff, uint16_t size);
	bool read(uint8_t* rx_buff, uint16_t size);
	void flush();
private:
	FATFS fatfs;
	FIL fil;
	char filename[50];
	bool reading = false;
};

#endif /* INC_SD_H_ */
