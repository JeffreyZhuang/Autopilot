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

bool Mlrs_telem::read()
{
	if (new_packet)
	{
		new_packet = false;

//		for (int i = 0; i < 40; i++)
//		{
//			printf("%c%c%c%c%c%c%c%c ", BYTE_TO_BINARY(complete_packet[i]));
//		}
//		printf("\n");

		// Remove start byte
		uint8_t packet_no_start_byte[packet_len - 1];
		for (int i = 0; i < packet_len - 1; i++)
		{
			packet_no_start_byte[i] = complete_packet[i + 1];
		}

		uint8_t payload[packet_len - 2]; // Subtract two since removed start byte and COBS byte
		cobs_decode_result decode_result = cobs_decode(payload, packet_len - 2, packet_no_start_byte, packet_len - 1);

		for (int i = 0; i < packet_len - 2; i++)
		{
			printf("%c%c%c%c%c%c%c%c ", BYTE_TO_BINARY(payload[i]));
		}
		printf("\n");

		if (payload[0] == 2)
		{
			Waypoint_packet waypoint_packet;
			memcpy(&waypoint_packet, payload, sizeof(Waypoint_packet));
	//		printf("%d\n", waypoint_packet.payload_type);
			printf("%f %f %f\n", waypoint_packet.waypoint[0], waypoint_packet.waypoint[1], waypoint_packet.waypoint[2]);
		}

		return true;
	}

	return false;
}

void Mlrs_telem::dma_complete()
{
	working_packet[packet_index] = rx_buffer[0];
	packet_index++;

	if (packet_index == 40)
	{
		if (!new_packet)
		{
			for (int i = 0; i < 40; i++)
			{
				complete_packet[i] = working_packet[i];
			}

			new_packet = true;
		}

		packet_index = 0;
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
