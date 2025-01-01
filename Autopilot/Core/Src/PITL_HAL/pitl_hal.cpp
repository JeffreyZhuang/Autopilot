#include "pitl_hal.h"

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{

}

void Pitl_hal::init()
{

}

void Pitl_hal::read_sensors()
{
	// Instead of setting sensor values of plane struct, set AHRS and navigation values from USB
}

// Debug
void Pitl_hal::debug_print(char * str)
{
	printf(str);
}

void Pitl_hal::usb_print(char * str)
{
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

void Pitl_hal::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}

// Time
uint64_t Pitl_hal::get_time_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5) * 10;
}

void Pitl_hal::delay_us(uint64_t us)
{
	uint64_t start = get_time_us();
	while (get_time_us() - start < us);
}


// Scheduler
void Pitl_hal::set_main_task(void (*task)())
{
	while (1)
	{
		task();
	}
}

void Pitl_hal::set_background_task(void (*task)()) {};
