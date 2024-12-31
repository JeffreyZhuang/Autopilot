#include <flight_hal.h>

// Counter increments every 10 us
// Resolution is 10 us
// Maximum allowed time is 42949672950 us or 12 hours
// SysTick has maximum time of 4.7 hours
uint64_t Flight_hal::get_time_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5) * 10;
}

void Flight_hal::delay_us(uint64_t us)
{
	uint64_t start = get_time_us();
	while (get_time_us() - start < us);
}
