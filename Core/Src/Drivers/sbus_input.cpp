#include <Drivers/sbus_input.h>

SBUS_input::SBUS_input(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void SBUS_input::setup()
{
	HAL_UART_Receive_DMA(_uart, _rx_buffer, 1);
}

void SBUS_input::dma_complete()
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

void SBUS_input::get_rc_data(uint16_t out[], int size)
{
	for (int i = 0; i < size; i++)
	{
		out[i] = _rc_data[i];
	}
}
