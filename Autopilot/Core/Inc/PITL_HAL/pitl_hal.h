/*
 * pitl_hal.h
 *
 *  Created on: Dec. 30, 2024
 *      Author: jeffr
 */

#ifndef INC_PITL_HAL_H_
#define INC_PITL_HAL_H_

#include "hal.h"

extern "C"
{
#include "usbd_cdc_if.h"
#include "main.h"
}

extern TIM_HandleTypeDef htim5;

// Processor in the loop using USB
class Pitl_hal : public HAL
{
public:
	Pitl_hal(Plane* plane);
	void init();
	void read_sensors();

	// Logger
	void write_storage_buffer() {};
	void flush_storage_buffer() {};
	void read_storage() {};

	// Debug
	void debug_print(char * str);
	void usb_print(char * str);
	void toggle_led();

	// Servos
	void set_elevator(float deg) {};
	void set_rudder(float deg) {};

	// Time
	void delay_us(uint64_t us);
	uint64_t get_time_us();

	// Scheduler
	void set_main_task(void (*task)());
	void set_background_task(void (*task)());

private:
	Plane* _plane;
};


#endif /* INC_PITL_HAL_H_ */
