#ifndef INC_FLIGHT_HAL_H_
#define INC_FLIGHT_HAL_H_

#include "servo.h"
#include "hal.h"
#include "plane.h"
#include "gnss.h"
#include "icm42688p.h"
#include "ina219.h"
#include "mlx90393.h"
#include "sd.h"
#include "mlrs_rc.h"
#include "mlrs_telem.h"

extern "C"
{
#include "usbd_cdc_if.h"
#include "barometer.h"
}

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern SD_HandleTypeDef hsd;

extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

class Flight_hal : public HAL
{
public:
	Flight_hal(Plane * plane);

	void init();
	void read_sensors();

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

	// logger_hal.cpp
	void init_logger();
	void write_storage_buffer();
	void flush_storage_buffer();
	void read_storage();

	// debug_hal.cpp
	void debug_print(char * str);
	void usb_print(char * str);
	void toggle_led();

	// time_hal.cpp
	void delay_us(uint64_t);
	uint64_t get_time_us();

	// servos_hal.cpp
	void init_servos();
	void set_elevator(float deg);
	void set_rudder(float deg);
	void set_throttle(float throttle);

	// power_monitor_hal.cpp
	void read_power_monitor();

	// telemetry_hal.cpp
	void init_telem();
	void read_rc();
	void transmit_telem(uint8_t tx_buff[], int len);
	bool read_telem();
	static void rc_dma_complete() { _instance->mlrs_rc.dma_complete(); }
	static void telemetry_dma_complete() { _instance->mlrs_telem.dma_complete(); }

	// scheduler_hal.cpp
	void set_main_task(void (*task)());
	void set_background_task(void (*task)());
	void execute_main_task();
	static void main_task_callback() { _instance->execute_main_task(); }

private:
	Plane* _plane;
	ICM42688 _imu;
	INA219 _ina219;
	Adafruit_MLX90393 _mag;
	GNSS _gnss;
	Sd _sd;
	Mlrs_rc mlrs_rc;
	Mlrs_telem mlrs_telem;
	Servo servo_elevator;
//	Servo servo_aileron;

	float _hard_iron[3] = {52.67, -5.27, 81.54};
	float _soft_iron[3][3] = {{1.031, 0.015, -0.0032},
	                          {0.015, 0.967, -0.025},
	                          {-0.032, -0.025, 1.005}};

	// scheduler_hal.cpp
	void (*main_task)() = nullptr;
	void (*background_task)() = nullptr;
	static Flight_hal* _instance;
};

#endif /* INC_FLIGHT_HAL_H_ */
