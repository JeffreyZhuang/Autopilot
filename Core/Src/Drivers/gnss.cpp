/*
 * gps.cpp
 *
 *  Created on: Dec 23, 2024
 *      Author: jeffr
 */

#include <gnss.h>

GNSS::GNSS(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void GNSS::setup()
{
	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}

bool GNSS::parse(uint8_t sentence[])
{
	if (buffer_full)
	{
		// Return sentence
		for (int i = 0; i < sizeof(complete_nmea_sentence) / sizeof(complete_nmea_sentence[0]); i++)
		{
			sentence[i] = complete_nmea_sentence[i];
		}

		buffer_full = false;

		// Parse
		char* line = (char*)sentence;
		struct minmea_sentence_gga frame;
		if (minmea_parse_gga(&frame, line))
		{
			lat = minmea_tocoord(&frame.latitude);
			lon = minmea_tocoord(&frame.longitude);
			sats = frame.satellites_tracked;
			fix_quality = frame.fix_quality;

			return true;
		}
	}

	return false;
}

// Remember to disable UBX output! It is only made for NMEA packets, not UBX.
void GNSS::dma_complete()
{
	// Append recieved byte to GNSS sentence
	nmea_sentence[last_sentence_index] = rx_buffer[0];
	last_sentence_index++;

	// Prevent out of range index
	if (last_sentence_index == nmea_sentence_len)
	{
		last_sentence_index = nmea_sentence_len - 1;
	}

	if ((char)(rx_buffer[0]) == '\n')
	{
		buffer_full = false;

		// Copy gnss_sentence to complete_nmea_sentence and set nmea_sentence to 0
		for (int i = 0; i < nmea_sentence_len; i++)
		{
			complete_nmea_sentence[i] = nmea_sentence[i];
			nmea_sentence[i] = 0;
		}

		buffer_full = true;
		last_sentence_index = 0;
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
