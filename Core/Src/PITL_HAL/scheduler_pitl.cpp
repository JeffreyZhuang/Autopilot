#include "pitl_hal.h"

void Pitl_hal::start_main_task(void (*task)())
{
	uint64_t prev_time = get_time_us();

	while (1)
	{
		if (get_time_us() - prev_time >= get_main_dt() * s_to_us)
		{
			task();
		}
	}
}

float Pitl_hal::get_main_dt() const
{
	return 0.01;
}

#if PITL_ENABLE
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	Pitl_hal::get_instance()->usb_rx_callback(Buf, Len);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart4)
	{
		Pitl_hal::rc_dma_complete();
	}
	else if (huart == &huart6)
	{
		Pitl_hal::telemetry_dma_complete();
	}
}
#endif
