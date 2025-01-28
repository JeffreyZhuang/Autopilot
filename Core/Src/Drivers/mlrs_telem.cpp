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

void Mlrs_telem::dma_complete()
{
	printf("%c", rx_buffer[0]);

	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
