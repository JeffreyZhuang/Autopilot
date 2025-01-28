#include "mlrs_telem.h"

Mlrs_telem::Mlrs_telem(UART_HandleTypeDef* uart)
{
	_uart = uart;
}

void Mlrs_telem::setup()
{

}

void Mlrs_telem::transmit()
{
	char tx_buff[] = "Hello\n";
	HAL_UART_Transmit(_uart, (uint8_t*)tx_buff, strlen(tx_buff), 1000);

	printf("Uart transmit\n");
}

void Mlrs_telem::dma_complete()
{

}
