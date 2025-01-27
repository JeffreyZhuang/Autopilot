#include "mlrs.h"

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
	printf("%s\n", rx_buffer);

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
