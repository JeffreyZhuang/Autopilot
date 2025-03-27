#ifndef INC_FLIGHT_HAL_H_
#define INC_FLIGHT_HAL_H_

#include <data_bus.h>
#include <Drivers/sbus_input.h>
#include <lib/utils/utils.h>
#include "Drivers/servo.h"
#include "Drivers/gnss.h"
#include "Drivers/icm42688p.h"
#include "Drivers/ina219.h"
#include "Drivers/mlx90393.h"
#include "Drivers/sd.h"
#include "Drivers/uart_stream.h"
#include "Drivers/cxof.h"
#include "parameters.h"
#include "hal.h"

extern "C"
{
#include "usbd_cdc_if.h"
#include "Drivers/barometer.h"
}

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

struct Hitl_rx_packet
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
	int32_t lat;
	int32_t lon;
	int16_t of_x;
	int16_t of_y;
};

struct Hitl_tx_packet
{
	uint16_t ele_duty = 0;
	uint16_t rud_duty = 0;
	uint16_t thr_duty = 0;
};

class Flight_hal : public HAL
{
public:
	Flight_hal();

	void init() override;

	// imu_hal.cpp
	void init_imu();
	void read_imu();

	// baro_hal.cpp
	void init_baro();
	void read_baro();

	// gnss_hal.cpp
	void init_gnss();
	void read_gnss();
	static void gnss_dma_complete() { _instance->_gnss.dma_complete(); }

	// compass_hal.cpp
	void init_compass();
	void read_compass();

	// optical_flow_hal.cpp
	void init_of();
	void read_of();
	static void of_dma_complete() { _instance->cxof.dma_complete(); }

	// logger_hal.cpp
	void init_logger();
	void write_storage_buffer(uint8_t* packet, uint16_t len) override;
	void flush_storage_buffer() override;
	bool read_storage(uint8_t* rx_buff, uint16_t size) override;

	// debug_hal.cpp
	void debug_print(char * str) override;
	void toggle_led() override;

	// time_hal.cpp
	void delay_us(uint64_t) override;
	uint64_t get_time_us() const override;

	// servos_hal.cpp
	void init_servos();

	// power_monitor_hal.cpp
	void read_power_monitor();

	// telemetry_hal.cpp
	void init_telem();
	void transmit_telem(uint8_t tx_buff[], int len) override;
	bool read_telem(uint8_t* byte) override;
	bool telem_buffer_empty() override;
	static void telemetry_dma_complete() { _instance->telem.dma_complete(); }

	// RC
	void get_rc_input(uint16_t duty[], uint8_t num_channels) override;
	static void rc_dma_complete() { _instance->sbus_input.dma_complete(); }

	// USB
	void usb_rx_callback(uint8_t* Buf, uint32_t Len);

	// scheduler_hal.cpp
	void start_main_task(void (*task)()) override;
	void start_background_task(void (*task)()) override;
	void execute_main_task();
	static void main_task_callback() { _instance->execute_main_task(); }

	static Flight_hal *get_instance() { return _instance; };

private:
	ICM42688 _imu;
	INA219 _ina219;
	Adafruit_MLX90393 _mag;
	GNSS _gnss;
	Sd _sd;
	SBUS_input sbus_input;
	Uart_stream telem;
	Servo servo1;
	Servo servo2;
	Cxof cxof;

	Publisher<IMU_data> _imu_pub;
	Publisher<Baro_data> _baro_pub;
	Publisher<GNSS_data> _gnss_pub;
	Publisher<Mag_data> _mag_pub;
	Publisher<OF_data> _of_pub;
	Publisher<Power_data> _power_pub;

	void usb_print_flight(char * str) override;

	void read_sensors_flight() override;
	void read_sensors_hitl() override;

	void set_pwm_flight(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
						uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty) override;
	void set_pwm_hitl(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
					  uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty) override;

	// HITL USB Double Buffering
	Hitl_rx_packet* usb_buff1;
	Hitl_rx_packet* usb_buff2;
	bool buff1_active = true;
	bool buff1_ready = false;
	bool buff2_ready = false;

	// scheduler_hal.cpp
	void (*main_task)() = nullptr;
	void (*background_task)() = nullptr;

	static Flight_hal* _instance;
};

#endif /* INC_FLIGHT_HAL_H_ */
