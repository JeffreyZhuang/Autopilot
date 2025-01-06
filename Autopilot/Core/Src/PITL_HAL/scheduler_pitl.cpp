#include "pitl_hal.h"

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
