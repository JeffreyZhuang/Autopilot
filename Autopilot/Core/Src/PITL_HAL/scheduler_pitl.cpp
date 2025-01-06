#include "pitl_hal.h"

Pitl_hal* Pitl_hal::_instance = nullptr;

void Pitl_hal::set_main_task(void (*task)())
{
	uint64_t prev_time = get_time_us();
	uint64_t dt = 1000000 / 100;

	while (1)
	{
		// Limit loop rate to 400Hz
		if (get_time_us() - prev_time > dt)
		{
			task();
		}
	}
}

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	// Read sensor data from USB and add to plane struct
	printf("Received: %s\n", Buf);
}
