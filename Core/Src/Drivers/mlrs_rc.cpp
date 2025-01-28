#include <mlrs_rc.h>

#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

MLRS::MLRS(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void MLRS::setup()
{
	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}

void MLRS::dma_complete()
{
//	printf("%c%c%c%c%c%c%c%c ", BYTE_TO_BINARY(rx_buffer[0]));

	frame[frame_idx++] = rx_buffer[0];

	if (frame_idx == frame_len)
	{
		for (int i = 0; i < frame_len; i++)
		{
			SBus_ParseByte(frame[i]);
		}

		SBus_DecodeFrame();

		for (int i = 0; i < num_channels; i++)
		{
			rc_data[i] = SBus_GetChannel(i);
//			printf("%d ", rc_data[i]);
		}

//		printf("\n");

		frame_idx = 0;
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
