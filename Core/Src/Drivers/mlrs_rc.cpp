#include "Drivers/mlrs_rc.h"

Mlrs_rc::Mlrs_rc(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void Mlrs_rc::setup()
{
	HAL_UART_Receive_DMA(_uart, _rx_buffer, 1);
}

void Mlrs_rc::dma_complete()
{
	if (SBus_ParseByte(_rx_buffer[0]) == SBUS_FRAME_READY)
	{
		SBus_DecodeFrame();

		for (int i = 0; i < 16; i++)
		{
			_rc_data[i] = SBus_GetChannel(i);
		}
	}

	HAL_UART_Receive_DMA(_uart, _rx_buffer, 1);
}

void Mlrs_rc::get_rc_data(uint16_t out[], int size)
{
	for (int i = 0; i < size; i++)
	{
		out[i] = _rc_data[i];
	}
}
