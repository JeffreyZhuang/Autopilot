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
	uint8_t tx_buff[] = {69};
	HAL_UART_Transmit(_uart, tx_buff,  sizeof(tx_buff), 1000);
}

void Mlrs_telem::dma_complete()
{

}
