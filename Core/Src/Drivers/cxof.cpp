#include "Drivers/cxof.h"

Cxof::Cxof(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void Cxof::setup()
{
	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}

bool Cxof::read(Cxof_frame *frame)
{
	if (new_data)
	{
		*frame = result;
		new_data = false;
		return true;
	}
	return false;
}

void Cxof::dma_complete()
{
	uint8_t byte = rx_buffer[0];

	if (byte == CXOF_HEADER)
	{
		frame_started = true;
	}

	if (frame_started)
	{
		working_frame[frame_idx] = byte;
		frame_idx++;

		if (frame_idx == CXOF_FRAME_LEN)
		{
			if (byte == CXOF_FOOTER && !new_data)
			{
				memcpy(&result, working_frame, sizeof(Cxof_frame));
				new_data = true;
			}

			frame_idx = 0;
			frame_started = false;
		}
	}

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
