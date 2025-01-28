/*
 * pitl_hal.h
 *
 *  Created on: Dec. 30, 2024
 *      Author: jeffr
 */

#ifndef INC_PITL_HAL_H_
#define INC_PITL_HAL_H_

#include <mlrs_rc.h>
#include "hal.h"
#include "mlrs_telem.h"

extern "C"
{
#include "usbd_cdc_if.h"
#include "main.h"
}

extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

struct Pitl_rx_packet
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

// Use aileron instead of rudder. Afterwards I just set aileron command directly to rudder.
// The mixer is in charge of choosing between rudder and aileron
struct Pitl_tx_packet
{
	float aileron;
	float elevator;
	float throttle;
	uint8_t footer = '\n';
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

	// Telemetry
	static void rc_dma_complete()
	{
		if (_instance != nullptr)
		{
			_instance->mlrs_rc.dma_complete();
		}
	}
	static void telemetry_dma_complete()
	{
		if (_instance != nullptr)
		{
			_instance->mlrs_telem.dma_complete();
		}
	}
	void transmit_telem(uint8_t tx_buff[], int len);

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
	void set_throttle(float throttle);

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

	// Telemetry
	Mlrs_rc mlrs_rc;
	Mlrs_telem mlrs_telem;

	// USB
	Pitl_tx_packet pitl_tx_packet;
	Pitl_rx_packet* usb_buff1;
	Pitl_rx_packet* usb_buff2;
	bool buff1_active = true;
	bool buff1_ready = false;
	bool buff2_ready = false;

	static Pitl_hal* _instance;
};


#endif /* INC_PITL_HAL_H_ */
