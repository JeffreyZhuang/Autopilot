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
	if (new_data)
	{
		// Return sentence
		for (int i = 0; i < sizeof(complete_nmea_sentence) / sizeof(complete_nmea_sentence[0]); i++)
		{
			sentence[i] = complete_nmea_sentence[i];
		}

		new_data = false;

		// Parse
		char* line = (char*)sentence;
		switch (minmea_sentence_id(line, false))
		{
		case MINMEA_SENTENCE_RMC:
		{
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, line))
			{
				lat = minmea_tocoord(&frame.latitude);
				lon = minmea_tocoord(&frame.longitude);
			}

			break;
		}
		case MINMEA_SENTENCE_GSV:
		{
			struct minmea_sentence_gsv frame;
			if (minmea_parse_gsv(&frame, line))
			{
				sats = frame.total_sats;
			}

			break;
		}
		}

		return true;
	}

	return false;
}

// Remember to disable UBX output! It is only made for NMEA packets, not UBX.
void GNSS::callback()
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
		new_data = false;

		// Copy gnss_sentence to complete_nmea_sentence and set nmea_sentence to 0
		for (int i = 0; i < nmea_sentence_len; i++)
		{
			complete_nmea_sentence[i] = nmea_sentence[i];
			nmea_sentence[i] = 0;
		}

		new_data = true;
		last_sentence_index = 0;
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
