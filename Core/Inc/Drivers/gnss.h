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

class GNSS
{
public:
	GNSS(UART_HandleTypeDef* uart);
	void setup();
	void dma_complete();
	bool parse(uint8_t sentence[]);

	double lat = 0; // deg
	double lon = 0;
	uint8_t sats = 0;
	int fix_quality = 0;
private:
	UART_HandleTypeDef* _uart;

	uint8_t rx_buffer[1];

	static constexpr uint8_t nmea_sentence_len = 100;
	uint8_t nmea_sentence[nmea_sentence_len];
	uint8_t complete_nmea_sentence[nmea_sentence_len];
	uint8_t last_sentence_index = 0;

	bool new_data = false;
};

#endif /* INC_GNSS_H_ */
