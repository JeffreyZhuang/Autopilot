/*
 * pitl_hal.h
 *
 *  Created on: Dec. 30, 2024
 *      Author: jeffr
 */

#ifndef INC_PITL_HAL_H_
#define INC_PITL_HAL_H_

#include "hal.h"
#include "mlrs.h"

extern "C"
{
#include "usbd_cdc_if.h"
#include "main.h"
}

extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart4;

struct Pitl_packet
{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float mx;
	float my;
	float mz;
	float asl;
	float lat;
	float lon;
};

// Processor in the loop using USB
class Pitl_hal : public HAL
{
public:
	Pitl_hal(Plane* plane);
	void init();
	void read_sensors();
	void read_rc();
	void read_pitl();

	static void telemetry_dma_complete()
	{
		if (_instance != nullptr)
		{
			_instance->mlrs.dma_complete();
		}
	}

	// Logger
	void write_storage_buffer() {};
	void flush_storage_buffer() {};
	void read_storage() {};

	// Debug
	void debug_print(char * str);
	void usb_print(char * str) {}; // Removed because USB reserved for PITL
	void toggle_led();

	// Servos
	void set_elevator(float deg);
	void set_rudder(float deg);

	// Time
	void delay_us(uint64_t us);
	uint64_t get_time_us();

	// Scheduler
	void set_main_task(void (*task)());
	void set_background_task(void (*task)()) {};
	void usb_rx_callback(uint8_t* Buf, uint32_t Len);

	static Pitl_hal *get_instance() { return _instance; };

private:
	Plane* _plane;

	MLRS mlrs;

	float _elevator;
	float _rudder;

	// USB
	Pitl_packet* usb_buff1;
	Pitl_packet* usb_buff2;
	bool buff1_active = true;
	bool buff1_ready = false;
	bool buff2_ready = false;

	static Pitl_hal* _instance;
};


#endif /* INC_PITL_HAL_H_ */
