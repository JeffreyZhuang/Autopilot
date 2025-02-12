#include "mlrs_telem.h"

#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

#include <cstdio> // printf testing

Mlrs_telem::Mlrs_telem(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void Mlrs_telem::setup()
{
	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}

void Mlrs_telem::transmit(uint8_t tx_buff[], int len)
{
	HAL_UART_Transmit(_uart, tx_buff, len, 1000);
}

bool Mlrs_telem::read(uint8_t packet[])
{
	if (new_packet)
	{
		memcpy(packet, complete_packet, sizeof(complete_packet));
		new_packet = false;

		return true;
	}

	return false;
}

void Mlrs_telem::dma_complete()
{
	// Detect start byte
	if (rx_buffer[0] == 0)
	{
		in_reading = true;
	}

	if (in_reading)
	{
//		printf("%c%c%c%c%c%c%c%c ", BYTE_TO_BINARY(rx_buffer[0]));

		working_packet[packet_index] = rx_buffer[0];
		packet_index++;

		if (packet_index == packet_len)
		{
			in_reading = false;
			packet_index = 0;

			if (!new_packet)
			{
				for (int i = 0; i < packet_len; i++)
				{
					complete_packet[i] = working_packet[i];
				}

				new_packet = true;
//				printf("\n");
			}
		}
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
