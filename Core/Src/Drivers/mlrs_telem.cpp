#include "mlrs_telem.h"

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
	working_packet[packet_index] = rx_buffer[0];
	packet_index++;

	if (packet_index == packet_len) // Fix to detect start byte
	{
		if (!new_packet)
		{
			for (int i = 0; i < packet_len; i++)
			{
				complete_packet[i] = working_packet[i];
			}

			new_packet = true;
		}

		packet_index = 0;
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
