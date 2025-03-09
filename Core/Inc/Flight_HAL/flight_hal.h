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
#include "cxof.h"
#include "Lib/Utils/utils.h"
#include "parameters.h"

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

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

class Flight_hal : public HAL
{
public:
	Flight_hal(Plane* plane);

	void init() override;
	void read_sensors() override;

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

	// rangefinder_hal.cpp
	void init_rangefinder();
	void read_rangefinder();
	static void rangefinder_dma_complete() { _instance->rangefinder_dma_complete(); }

	// logger_hal.cpp
	void init_logger();
	void write_storage_buffer(uint8_t* packet, uint16_t len) override;
	void flush_storage_buffer() override;
	void read_storage(uint8_t* rx_buff, uint16_t size) override;

	// debug_hal.cpp
	void debug_print(char * str) override;
	void usb_print(char * str) override;
	void toggle_led() override;

	// time_hal.cpp
	void delay_us(uint64_t) override;
	uint64_t get_time_us() override;

	// servos_hal.cpp
	void init_servos();
	void set_duty(uint8_t channel, uint16_t duty_us) override;

	// power_monitor_hal.cpp
	void read_power_monitor();

	// telemetry_hal.cpp
	void init_telem();
	void transmit_telem(uint8_t tx_buff[], int len) override;
	bool read_telem(uint8_t rx_buff[], uint16_t* size) override;
	void get_rc_input(uint16_t duty[], uint8_t num_channels) override;
	static void rc_dma_complete() { _instance->mlrs_rc.dma_complete(); }
	static void telemetry_dma_complete() { _instance->mlrs_telem.dma_complete(); }

	// scheduler_hal.cpp
	void start_main_task(void (*task)()) override;
	void start_background_task(void (*task)()) override;
	float get_main_dt() const override;
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
	Servo servo1;
	Servo servo2;
	Cxof cxof;

	// scheduler_hal.cpp
	void (*main_task)() = nullptr;
	void (*background_task)() = nullptr;

	static Flight_hal* _instance;
};

#endif /* INC_FLIGHT_HAL_H_ */
