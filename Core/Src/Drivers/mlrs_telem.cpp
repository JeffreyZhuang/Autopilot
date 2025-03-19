#include "Drivers/mlrs_telem.h"

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
bool Mlrs_telem::read(uint8_t packet[], uint16_t* size)
{
	if (new_packet)
	{
		printf("Mlrs_telem packet_len: %d\n", packet_len);
		memcpy(packet, complete_packet, packet_len); // Memcpy mem fault
		*size = packet_len;
		new_packet = false;
		return true;
	}

	return false;
}

void Mlrs_telem::dma_complete()
{
	// Detect start byte
	// If I remove !new_packet there is mem fault from concurrency issue
	// Due to changing packet_len
	if (rx_buffer[0] == 0 && !new_packet)
	{
		in_pkt = true;
	}

	if (in_pkt)
	{
		// Get length byte and determine packet length
		if (wrking_pkt_idx == 1)
		{
			packet_len = rx_buffer[0] + header_len;
		}

		// Append new byte to working buffer
		working_packet[wrking_pkt_idx] = rx_buffer[0];
		wrking_pkt_idx++;

		// Check if packet completed
		if (wrking_pkt_idx == packet_len)
		{
			// Copy working packet to complete packet
			memcpy(complete_packet, working_packet, max_packet_len);
			new_packet = true;

			// Reset
			wrking_pkt_idx = 0;
			in_pkt = false;
		}
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
