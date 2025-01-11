#include "pitl_hal.h"

void Pitl_hal::set_main_task(void (*task)())
{
	uint64_t prev_time = get_time_us();

	while (1)
	{
		// Limit loop rate to 400Hz
		if (get_time_us() - prev_time >= main_dt * 1000000)
		{
			task();
		}
	}
}

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	Pitl_hal::get_instance()->usb_rx_callback(Buf, Len);
}


