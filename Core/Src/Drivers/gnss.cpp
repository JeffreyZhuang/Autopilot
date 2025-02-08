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

bool GNSS::parse()
{
	if (buffer_full)
	{
		buffer_full = false;

		// Parse
		char* line = (char*)complete_sentence;
		struct minmea_sentence_gga frame;
		if (minmea_parse_gga(&frame, line))
		{
			lat = minmea_tocoord_double(&frame.latitude);
			lon = minmea_tocoord_double(&frame.longitude);
			sats = frame.satellites_tracked;
			fix_quality = frame.fix_quality == 1;

			return true;
		}
	}

	return false;
}

// Remember to disable UBX output! It is only made for NMEA packets, not UBX.
void GNSS::dma_complete()
{
	if (!buffer_full)
	{
		// Append recieved byte to GNSS sentence
		working_sentence[last_sentence_index] = rx_buffer[0];
		last_sentence_index++;

		// Prevent out of range index
		// This is sketch, need to refactor
		if (last_sentence_index == sentence_len)
		{
			last_sentence_index = sentence_len - 1;
		}

		if ((char)(rx_buffer[0]) == '\n')
		{
			// Copy gnss_sentence to complete sentence and set working sentence to 0
			for (int i = 0; i < sentence_len; i++)
			{
				complete_sentence[i] = working_sentence[i];
				working_sentence[i] = 0;
			}

			buffer_full = true;
			last_sentence_index = 0;
		}
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}

double GNSS::minmea_tocoord_double(const struct minmea_float *f)
{
    if (f->scale == 0)
        return NAN;
    if (f->scale  > (INT_LEAST32_MAX / 100))
        return NAN;
    if (f->scale < (INT_LEAST32_MIN / 100))
        return NAN;
    int_least32_t degrees = f->value / (f->scale * 100);
    int_least32_t minutes = f->value % (f->scale * 100);
    return (double) degrees + (double) minutes / (60 * f->scale);
}
