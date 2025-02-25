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

// Get latest packet and the size of the packet
bool Mlrs_telem::read(uint8_t packet[], uint8_t* size)
{
	if (new_packet)
	{
		memcpy(packet, complete_packet, packet_len);
		*size = packet_len;
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
		in_pkt = true;
	}

	if (in_pkt)
	{
		// Get length byte and determine packet length
		if (wrking_pkt_idx == 1)
		{
			packet_len = rx_buffer[0] + 3; // Add 3 because header
		}

		// Append new byte to working buffer
		working_packet[wrking_pkt_idx] = rx_buffer[0];
		wrking_pkt_idx++;

		// Check if packet completed
		if (wrking_pkt_idx == packet_len)
		{
			// Only update complete packet if it has been read
			if (!new_packet)
			{
				// Copy working packet to complete packet
				memcpy(complete_packet, working_packet, max_packet_len);
				new_packet = true;

				wrking_pkt_idx = 0;
				in_pkt = false;
			}
		}
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
