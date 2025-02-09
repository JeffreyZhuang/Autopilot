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
//	if (buffer_full)
//	{
//		buffer_full = false;
//
//		// Parse
//		char* line = (char*)complete_sentence;
//		struct minmea_sentence_gga frame;
//		if (minmea_parse_gga(&frame, line))
//		{
//			lat = minmea_tocoord_double(&frame.latitude);
//			lon = minmea_tocoord_double(&frame.longitude);
//			sats = frame.satellites_tracked;
//			fix = frame.fix_quality == 1;
//
//			return true;
//		}
//	}

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

		printf("%s\n", sentence);
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
