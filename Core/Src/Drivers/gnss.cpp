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

bool GNSS::read()
{
	if (new_data)
	{
		new_data = false;

		return true;
	}

	return false;
}

// Remember to disable UBX output! It is only made for NMEA packets, not UBX.
void GNSS::dma_complete()
{
	char c = rx_buffer[0];

	if (c == '$')
	{
		sentence[0] = c;
		sentence_index = 1;
		sentence_started = true;
	}
	else if (c == '\r' && sentence_started && sentence_index < max_sentence_len)
	{
		sentence[sentence_index] = c;
		sentence_index = 0;
		sentence_started = false;

		struct minmea_sentence_gga frame;
		if (minmea_parse_gga(&frame, (char*)sentence))
		{
//			printf("%s\n", sentence);

			lat = minmea_tocoord_double(&frame.latitude);
			lon = minmea_tocoord_double(&frame.longitude);
			sats = frame.satellites_tracked;
			fix = frame.fix_quality == 1;

			new_data = true;
		}
	}
	else if (sentence_started && sentence_index < max_sentence_len)
	{
		sentence[sentence_index] = c;
		sentence_index++;
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
