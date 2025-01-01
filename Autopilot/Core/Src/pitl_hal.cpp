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
void debug_print(char * str);
void usb_print(char * str);
void toggle_led();

// Time
void delay_us(uint64_t us);
uint64_t get_time_us();

// Scheduler
void set_main_task(void (*task)());
void set_background_task(void (*task)());
