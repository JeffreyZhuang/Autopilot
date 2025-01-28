#include "pitl_hal.h"

uint64_t Pitl_hal::get_time_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5) * 10;
}

void Pitl_hal::delay_us(uint64_t us)
{
	uint64_t start = get_time_us();
	while (get_time_us() - start < us);
}
