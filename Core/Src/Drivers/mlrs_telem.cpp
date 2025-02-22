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

bool Mlrs_telem::read(uint8_t packet[], uint8_t* size)
{
	if (new_packet)
	{
		memcpy(packet, complete_packet, sizeof(complete_packet));
		*size = complete_packet_len;
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
		packet_index = 0;
	}

	// Detect length byte
	if (in_reading && packet_index == 1)
	{
		payload_len = rx_buffer[0];
	}

	// Read payload
	if (in_reading && packet_index > 1 && packet_index < payload_len)
	{
		working_packet[packet_index] = rx_buffer[0];
		packet_index++;

		if (packet_index == payload_len)
		{
			in_reading = false;

			if (!new_packet)
			{
				for (int i = 0; i < payload_len; i++)
				{
					complete_packet[i] = working_packet[i];
				}
				complete_packet_len = payload_len;

				new_packet = true;
			}
		}
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
