#include "Drivers/uart_stream.h"

Uart_stream::Uart_stream(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void Uart_stream::setup()
{
	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}

void Uart_stream::transmit(uint8_t tx_buff[], int len)
{
	HAL_UART_Transmit(_uart, tx_buff, len, 1000);
}

bool Uart_stream::read_byte(uint8_t* byte)
{
	return ring_buffer_read(&ring_buffer, byte);
}

bool Uart_stream::buffer_empty()
{
	return ring_buffer_empty(&ring_buffer);
}

void Uart_stream::dma_complete()
{
	ring_buffer_write(&ring_buffer, rx_buffer[0]);
	HAL_UART_Receive_DMA(_uart, rx_buffer, 1);
}
