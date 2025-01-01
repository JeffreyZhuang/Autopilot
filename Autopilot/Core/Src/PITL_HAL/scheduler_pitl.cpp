#include "pitl_hal.h"

void Pitl_hal::set_main_task(void (*task)())
{
	while (1)
	{
		task();
	}
}
