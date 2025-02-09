/*
 * gps.hpp
 *
 *  Created on: Dec 23, 2024
 *      Author: jeffr
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstring>

extern "C"
{
#include "usbd_cdc_if.h"
#include "minmea.h"
}

/**
 * Configure GNSS in U-center to ONLY output GGA NMEA sentences
 */
class GNSS
{
public:
	GNSS(UART_HandleTypeDef* uart);
	void setup();
	bool read();
	void dma_complete();

	double lat = 0; // deg
	double lon = 0; // deg
	uint8_t sats = 0;
	bool fix = false;
private:
	UART_HandleTypeDef* _uart;

	uint8_t rx_buffer[1];

	static constexpr uint8_t max_sentence_len = 100;
	char sentence[max_sentence_len];
	uint8_t sentence_index = 0;
	bool sentence_started = false;
	bool new_data = false;

	double minmea_tocoord_double(const struct minmea_float *f);
};

#endif /* INC_GNSS_H_ */
