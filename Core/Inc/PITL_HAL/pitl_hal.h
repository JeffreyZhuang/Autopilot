/*
 * pitl_hal.h
 *
 *  Created on: Dec. 30, 2024
 *      Author: jeffr
 */

#ifndef INC_PITL_HAL_H_
#define INC_PITL_HAL_H_

#include "mlrs_rc.h"
#include "mlrs_telem.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include "constants.h"

extern "C"
{
#include "usbd_cdc_if.h"
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
	float agl;
};

struct Pitl_tx_packet
{
	float value[3];
};

// Processor in the loop using USB
class Pitl_hal : public HAL
{
public:
	Pitl_hal(Plane* plane);
	void init();

	void read_sensors();
	void read_pitl();

	// Telemetry
	static void rc_dma_complete() { _instance->mlrs_rc.dma_complete(); }
	static void telemetry_dma_complete() { _instance->mlrs_telem.dma_complete(); }
	void transmit_telem(uint8_t tx_buff[], int len);
	bool read_telem(uint8_t rx_buff[], uint16_t* size) override;
	void get_rc_input(uint16_t duty[], uint8_t num_channels) override;

	// Logger
	void write_storage_buffer(uint8_t* packet, uint16_t len) {};
	void flush_storage_buffer() {};
	void read_storage(uint8_t* rx_buff, uint16_t size) {};

	// Debug
	void debug_print(char * str) override;
	void usb_print(char * str) override {}; // Removed because USB reserved for PITL
	void toggle_led();

	// Servos
	void set_duty(uint8_t channel, uint16_t duty_us) override;

	// Time
	void delay_us(uint64_t us) override;
	uint64_t get_time_us() override;

	// Scheduler
	void set_main_task(void (*task)());
	void execute_main_task();
	void set_background_task(void (*task)()) {};
	void usb_rx_callback(uint8_t* Buf, uint32_t Len);
	float get_main_dt() const;

	static Pitl_hal *get_instance() { return _instance; };

private:
	Plane* _plane;

	// Telemetry
	Mlrs_rc mlrs_rc;
	Mlrs_telem mlrs_telem;

	// USB
	Pitl_tx_packet pitl_tx_packet{0, 0, 0};
	Pitl_rx_packet* usb_buff1;
	Pitl_rx_packet* usb_buff2;
	bool buff1_active = true;
	bool buff1_ready = false;
	bool buff2_ready = false;

	static Pitl_hal* _instance;
};


#endif /* INC_PITL_HAL_H_ */
